
#include <list>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/markermap.h>
#include "utils/system.h"
#include "basictypes/misc.h"
#include "basictypes/debug.h"
#include "basictypes/timers.h"
#include "optimization/pnpsolver.h"
#include "optimization/globaloptimizer.h"
#include "optimization/ippe.h"
#include "basictypes/io_utils.h"
#include "map_types/keyframedatabase.h"
#include "utils/mapinitializer.h"
#include "utils/mapmanager.h"
#include "map.h"
#include "basictypes/se3.h"
#include "basictypes/osadapter.h"
#include "map_types/covisgraph.h"
#include "utils/frameextractor.h"
#include "basictypes/hash.h"
using namespace std;

namespace ucoslam {
Params System::m_params;

Params &System::getParams() { return m_params; }

uint32_t System::getCurrentKeyFrameIndex() {
    return m_keyframe_Index;
}

std::shared_ptr<Map> System::getMap() {
    return m_map;
}

System::System() {
    m_frame_extractor = std::make_shared<FrameExtractor>();
    m_map_initializer = std::make_shared<MapInitializer>();
    m_map_manager = std::make_shared<MapManager>();
}

System::~System(){
    waitForFinished();
}

void System::updateParams(const Params &params ) {
    waitForFinished();
    m_map_manager = std::make_shared<MapManager>();
    m_params = params;
    updateFrameExtractorParam();
}


void System::updateFrameExtractorParam() {
    m_params.nthreads_feature_detector = max (1, m_params.nthreads_feature_detector);
    std::shared_ptr<Feature2DSerializable> frame_extractor = Feature2DSerializable::create(
                  m_params.kpDescriptorType);
    frame_extractor->setParams(m_params.extraParams);
    m_params.maxDescDistance = frame_extractor ->getMinDescDistance();
    m_frame_extractor->setParams(frame_extractor, m_params);
    m_frame_extractor->removeFromMarkers() = m_params.removeKeyPointsIntoMarkers;
    m_frame_extractor->detectMarkers() = m_params.detectMarkers;
    m_frame_extractor->detectKeyPoints() = m_params.detectKeyPoints;
}

void System::setParams(std::shared_ptr<Map> map, const Params &params,const string &vocabulary)
{
    m_map = map;
    m_params = params;
    updateFrameExtractorParam();
    if(m_map->isEmpty())
    {
        m_state = STATE_LOST;
        if(!vocabulary.empty()) {
            m_map->TheKFDataBase.loadFromFile(vocabulary);
        }
        MapInitializer::Params map_initializer_param;
        if(m_params.forceInitializationFromMarkers)
            map_initializer_param.mode = MapInitializer::ARUCO;
        else
            map_initializer_param.mode = MapInitializer::BOTH;
        map_initializer_param.minDistance = m_params.minBaseLine;
        map_initializer_param.markerSize = m_params.aruco_markerSize;
        map_initializer_param.aruco_minerrratio_valid = m_params.aruco_minerrratio_valid;
        map_initializer_param.allowArucoOneFrame = m_params.aruco_allowOneFrameInitialization;
        map_initializer_param.max_makr_rep_err = 2.5;
        map_initializer_param.minDescDistance = m_params.maxDescDistance;
        m_map_initializer->setParams(map_initializer_param);
    }
    else
        m_state = STATE_LOST;
}


void System::waitForFinished() {
    m_map_manager->stop();
    m_map_manager->mapUpdate();
    if(m_map_manager->bigChange())
    {
        m_current_frame.pose_f2g = m_map_manager->getLastAddedKFPose();
        m_se3 = m_current_frame.pose_f2g;
    }
}

void System::resetTracker() {
    waitForFinished();
    m_keyframe_Index = -1;
    m_se3 = se3();
    m_state = STATE_LOST;
    m_current_frame.clear();
    m_prev_frame.clear();
    _14463320619150402643 = cv::Mat();
    m_fseq_idx = -1;
}

cv::Mat System::process(const Frame &frame) {
    se3 _16937225862434286412 = m_se3;
    if((void*)&frame != (void* )&m_current_frame)
    {
        swap(m_prev_frame, m_current_frame);
        m_current_frame = frame;
    }
    if(m_mode == MODE_SLAM && !m_map_manager->hasMap())
        m_map_manager->setMap(m_map);
    if(!m_params.runSequential && m_mode == MODE_SLAM)
        m_map_manager->start();
    for(auto &id_value : m_prev_frame.ids)
    {
        if(id_value != std::numeric_limits<uint32_t>::max())
        {
            if(!m_map->map_points.is(id_value))
                id_value = std::numeric_limits<uint32_t>::max();
            else if(m_map->map_points[id_value].isBad())
                    id_value = std::numeric_limits< uint32_t >::max();
        }
    }
    if(m_map->isEmpty() && m_mode == MODE_SLAM)
    {
        if(processFirstFrame(m_current_frame))
            m_state = STATE_TRACKING;
    }
    else
    {
        if(m_state == STATE_TRACKING)
        {
            m_keyframe_Index = retrieveKeyFrame(m_prev_frame, m_se3);
            m_se3 = updatePose(m_current_frame, m_se3);
            if(!m_se3.isValid())
                m_state = STATE_LOST;
        }
        if(m_state == STATE_LOST)
        {
            se3 pose;
            if(reLocalization(m_current_frame, pose))
            {
                m_state = STATE_TRACKING;
                m_se3 = pose;
                m_keyframe_Index = retrieveKeyFrame(m_current_frame, m_se3);
                m_fseq_idx = m_current_frame.fseq_idx;

            }
        }
        if(m_state == STATE_TRACKING)
        {
            m_current_frame.pose_f2g = m_se3;
            if (m_mode == MODE_SLAM && ((m_current_frame.fseq_idx >= m_fseq_idx + 5)
            || (m_fseq_idx == -1)))
                m_map_manager->newFrame(m_current_frame, m_keyframe_Index);
        }
    }
    if(m_state == STATE_LOST && m_mode == MODE_SLAM 
      && m_map->keyframes.size() <= 5 && m_map->keyframes.size() !=0)
    {
        m_map_manager->reset();
        m_map->clear();
        m_map_initializer->reset();
        m_map_manager->setMap(m_map);
    }
    if(m_state == STATE_TRACKING)
    {
        _14463320619150402643 = cv::Mat::eye(4, 4, 5);
        if(_16937225862434286412.isValid())
        {
            _14463320619150402643 = m_se3.convert() * _16937225862434286412.convert().inv();
        }
    }
    else
    {
        _14463320619150402643 = cv::Mat();
    }

    m_current_frame.pose_f2g = m_se3;
    if(++_13033649816026327368 > (10 * 4 * 12 * 34 * 6) / 2)
        m_se3 = cv::Mat();
    if(m_state == STATE_LOST)
        return cv::Mat();
    else
        return m_se3;
}


cv::Mat System::process(cv::Mat& in_image, const ImageParams& ip, 
    uint32_t frameseq_idx, const cv::Mat& depth, const cv::Mat &R_image) {
    swap(m_prev_frame, m_current_frame);
    std::thread map_update_thread;
    if (m_mode == MODE_SLAM)
        map_update_thread = std::thread([&] (){
            if(m_map_manager->mapUpdate())
            {
                if(m_map_manager->bigChange())
                {
                    m_current_frame.pose_f2g = m_map_manager->getLastAddedKFPose();
                    m_se3 = m_map_manager->getLastAddedKFPose();
                }
            };
    });
    if(depth.empty() && R_image.empty())
        m_frame_extractor->process(in_image, ip, m_current_frame, frameseq_idx);
    else if(R_image.empty())
        m_frame_extractor->process_rgbd(in_image, depth, ip, m_current_frame, frameseq_idx);
    else
        m_frame_extractor->processStereo(in_image, R_image, ip, m_current_frame, frameseq_idx);
    if(m_params.autoAdjustKpSensitivity)
    {
        int diff_features = m_params.maxFeatures - m_current_frame.und_kpts.size();
        if(diff_features > 0)
        {
            float percent = 1.0f - (float(diff_features) / float(m_current_frame.und_kpts.size()));
            float sensitivity = m_frame_extractor->getSensitivity() + percent;
            sensitivity = std::max(sensitivity, 1.0f);
            m_frame_extractor->setSensitivity(sensitivity);
        }
        else
        {
            m_frame_extractor->setSensitivity(m_frame_extractor->getSensitivity() * 0.95);
        }
    }
    if(m_mode == MODE_SLAM)
        map_update_thread.join();

    cv::Mat _3005399805025936106 = process(m_current_frame);
    float scale = sqrt(float(m_current_frame.imageParams.CamSize.area()) / float(in_image.size().area()));
    drawImage(in_image, 1. / scale);

    auto uint_to_str = [] (const uint32_t& uint_value) {
        std::stringstream sstr;
        sstr << uint_value;
        return sstr.str();
    };
    putText(in_image, "Map Points:" + uint_to_str(m_map->map_points.size()), cv::Point(20, in_image.rows - 20));
    putText(in_image, "Map Markers:" + uint_to_str(m_map->map_markers.size()), cv::Point(20, in_image.rows - 40));
    putText(in_image, "KeyFrames:" + uint_to_str(m_map->keyframes.size()), cv::Point(20, in_image.rows - 60));
    int count = 0 ;
    for(auto id_value : m_current_frame.ids)
        if(id_value != std::numeric_limits<uint32_t >::max())
            count++;
    putText(in_image,"Matches:" + uint_to_str(count), cv::Point(20, in_image.rows - 80));
    if(fabs(scale - 1) > 1e-3)
        putText(in_image, "Img.Size:" + uint_to_str(m_current_frame.imageParams.CamSize.width) + "x"
        + uint_to_str(m_current_frame.imageParams.CamSize.height), cv::Point(20, in_image.rows - 100));
    return _3005399805025936106;
}


void System::putText(cv::Mat &img, string text, cv::Point origin) {

    float scale = float(img.cols) / float(1280);
    cv::putText( img, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5 * scale, cv::Scalar(0,0,0) , 3 * scale);
    cv::putText( img, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5 * scale, cv::Scalar(125,255,255), 1 * scale);
}

string System::getSignatureStr() const {

    return hashToStr(getHashValue());
}

uint64_t System::getHashValue(bool flag) const {
    Hash hash_value;
    hash_value += m_map->getSignature(flag);
    if(flag)
        cout << "\\tSystem 1. sig=" << hash_value << endl;

    hash_value += m_params.getSignature();
    if(flag)
        cout << "\\tSystem 2. sig=" << hash_value << endl;

    for( int i = 0; i < 6; i++)
        hash_value += m_se3[i];

    if(flag)
        cout << "\\tSystem 3. sig=" << hash_value << endl;

    hash_value.add(m_keyframe_Index);
    if(flag)
        cout << "\\tSystem 4. sig=" << hash_value << endl;

    hash_value += m_current_frame.getSignature();
    if(flag)
        cout << "\\tSystem 5. sig=" <<  hash_value << endl;

    hash_value += firstFrameFlag;
    if(flag)
        cout << "\\tSystem 7. sig=" << hash_value << endl;
    
    hash_value += m_state ;
    if(flag)
        cout << "\\tSystem 8. sig=" << hash_value << endl;

    hash_value += m_mode ;
    if(flag)
        cout << "\\tSystem 9. sig=" << hash_value << endl;

    hash_value += m_prev_frame.getSignature();
    if(flag)
        cout << "\\tSystem 10.sig=" << hash_value << endl;

    hash_value += m_map_manager->getSignature();
    if(flag)
        cout << "\\tSystem 11.sig=" << hash_value << endl;

    hash_value += _14463320619150402643;
    hash_value += m_fseq_idx;
    if(flag)
        cout << "\\tSystem 12.sig=" << hash_value << endl;

    return hash_value;

}

// cv::Mat System::_4145838251597814913(const Frame &frame) {

//     // std::vector <uint32_t> _4240939334638385660;
//     vector<pair<cv::Mat,double>> _5923954032168212568;
//     vector<cv::Point3f> point_3d_vector;
//     vector<cv::Point2f> point_2d_vector;
//     for( auto marker_observation : frame.markers)
//     {
//         if(m_map->map_markers.find(marker_observation.id) != m_map->map_markers.end())
//         {

//             ucoslam::Marker &marker = m_map->map_markers[marker_observation.id];
//             cv::Mat _9983235290341257781 = marker.pose_g2m;
//             auto point_3d = marker.get3DPoints();
//             point_3d_vector.insert(point_3d_vector.end(), point_3d.begin(), point_3d.end());
//             point_2d_vector.insert(point_2d_vector.end(), marker_observation.und_corners.begin(), marker_observation.und_corners.end());
//             auto _1515389571633683069 = IPPE::solvePnP(m_params.aruco_markerSize, marker_observation.und_corners,
//                 frame.imageParams.CameraMatrix,frame.imageParams.Distorsion);
//             for(auto _16937226146608628973 : _1515389571633683069)
//                 _5923954032168212568.push_back(make_pair(_16937226146608628973 * _9983235290341257781.inv(), -1));
//         }
//     }
//     if(point_3d_vector.size() == 0 )
//         return cv::Mat();
//     for( auto &_46082575779853237 : _5923954032168212568)
//     {
//         vector<cv::Point2f> proj_point_2d_vector;
//         se3 _16937226146609453200 = _46082575779853237.first;
//         project(point_3d_vector, frame.imageParams.CameraMatrix, _16937226146609453200.convert(), proj_point_2d_vector);
//         _46082575779853237.second = 0;
//         for(size_t i = 0; i < proj_point_2d_vector.size(); i++)
//             _46082575779853237.second += (proj_point_2d_vector[i].x - point_2d_vector[i].x) * 
//                                          (proj_point_2d_vector[i].x - point_2d_vector[i].x) + 
//                                          (proj_point_2d_vector[i].y - point_2d_vector[i].y) *
//                                          (proj_point_2d_vector[i].y - point_2d_vector[i].y);
//     }
//     std::sort(_5923954032168212568.begin(), _5923954032168212568.end(), [](const pair <cv::Mat,double>& left,
//         const pair<cv::Mat,double>& right) {
//             return left.second < right.second;
//     });
//     return _5923954032168212568[0].first;
// }

bool System::processFirstFrame(Frame &frame) {
    
    bool flag;
    if(frame.imageParams.isStereoCamera())
    {
        flag = processSteroFrame(frame);
    }
    else
    {
        flag = processNonSteroFrame(frame);
    }
    if(!flag)
        return flag;
    m_se3 = m_map->keyframes.back().pose_f2g;
    m_keyframe_Index = m_map->keyframes.back().idx;
    firstFrameFlag = true;
    return true;
}

bool System::processNonSteroFrame(Frame &frame) {

    if(!m_map_initializer->process(frame, m_map))
        return false;

    if (m_map->keyframes.size() > 1 && m_map->map_points.size() > 0)
    {
        frame.ids = m_map->keyframes.back().ids;
    }

    globalOptimization();
    if(m_map->map_markers.size() == 0)
    {
        if(m_map->map_points.size() < 50)
        {
            m_map->clear();
            return false;
        }
        float scale_factor = 1. / m_map->getFrameMedianDepth(m_map->keyframes.front().idx);
        cv::Mat transform_f2g_inv = m_map->keyframes.back().pose_f2g.inv();
        transform_f2g_inv.col(3).rowRange(0, 3) = transform_f2g_inv.col(3).rowRange(0, 3) * scale_factor;
        m_map->keyframes.back().pose_f2g = transform_f2g_inv.inv();
        for(auto &point : m_map->map_points)
        {
            point.scalePoint(scale_factor);
        }
    }
    m_se3 = m_map->keyframes.back().pose_f2g;
    return true;

}

bool System::processSteroFrame(Frame &frame) {

    if(m_params.KPNonMaximaSuppresion)
        frame.nonMaximaSuppresion();
    int count = 0;
    for(size_t i = 0; i < frame.und_kpts.size(); i++)
    {
        if(frame.getDepth(i) > 0 && frame.imageParams.isClosePoint(frame.getDepth(i))
              && !frame.flags[i].is(Frame::FLAG_NONMAXIMA))
            count++;
    }
    if(count < 100)
        return false;
    frame.pose_f2g.setUnity();
    Frame& new_frame = m_map->addKeyFrame(frame);
    for(size_t i = 0; i < frame.und_kpts.size(); i++)
    {
        cv::Point3f coordinates;
        if(frame.getDepth(i) > 0 && frame.imageParams.isClosePoint(frame.getDepth(i))
              && !frame.flags[i].is( Frame::FLAG_NONMAXIMA))
        {
            coordinates = frame.get3dStereoPoint(i);
            auto& new_point = m_map->addNewPoint(new_frame.fseq_idx);
            new_point.kfSinceAddition = 1;
            new_point.setCoordinates(coordinates);
            new_point.setStereo(true);
            m_map->addMapPointObservation(new_point.id, new_frame.idx, i);
            frame.ids[i] = new_point.id;
        }
    }
    for(const auto& marker : frame.markers)
    {
        m_map->addMarker(marker);
    }
    return true;
}

string System::hashToStr(uint64_t hash_value ) const {

    string result;
    string alphabet = "qwertyuiopasdfghjklzxcvbnm1234567890QWERTYUIOPASDFGHJKLZXCVBNM";
    uchar *origin = (uchar *)&hash_value;
    int str_len = sizeof (hash_value) / sizeof(uchar);
    for(int index = 0 ; index < str_len; index ++)
    {
        result.push_back(alphabet[origin[index] % alphabet.size()]);
    }
    return result;
}

uint32_t System::retrieveKeyFrame(const Frame &frame, const se3& se3_transform) {

    int64_t ref_keyframe_index = -1;
    if(m_params.detectKeyPoints)
        ref_keyframe_index = m_map->getReferenceKeyFrame(frame, 1);

    if(ref_keyframe_index != -1)
        return ref_keyframe_index;
    if(m_map->map_markers.size() == 0)
    {
        return m_keyframe_Index;
    }
    vector<uint32_t> marker_id_vector;
    for(auto marker : frame.markers)
    {
        auto iter = m_map->map_markers.find(marker.id);
        if( iter != m_map->map_markers.end())
        {
            if(iter->second.pose_g2m.isValid())
                marker_id_vector.push_back(marker.id);

        }
    }
    pair<uint32_t,float > result(std::numeric_limits<uint32_t>::max(), std::numeric_limits<float>::max());
    for(auto marker_id : marker_id_vector)
    {
        for(const auto&frame_index : m_map->map_markers[marker_id].frames)
        {
            auto distance = se3_transform.t_dist(m_map->keyframes[frame_index].pose_f2g);
            if(result.second > distance)
                result = {frame_index, distance};
        }
    }
    return result.first;
}


std::vector<System::KeyPointRelocResult> System::getKeyPointRelocResult(Frame &frame, se3 &se3_transform,
          const std::set<uint32_t> &excluded_frames) {

    if(frame.ids.size() == 0)
        return {};
    if(m_map->TheKFDataBase.isEmpty())
        return {};
    vector<uint32_t> candidates = m_map->relocalizationCandidates(frame, excluded_frames);

    if(candidates.size() == 0)
       return {};
    vector<System::KeyPointRelocResult> relocalization_results(candidates.size());
    FrameMatcher frame_matcher;
    frame_matcher.setParams(frame, FrameMatcher::MODE_ALL, m_params.maxDescDistance * 2);

    #pragma omp parallel for
    for(int i = 0 ; i < candidates.size(); i++)
    {
        auto index = candidates[i];
        auto &key_frame = m_map->keyframes[index];
        relocalization_results[i].matches = frame_matcher.match(key_frame, FrameMatcher::MODE_ASSIGNED);
        for( auto &result : relocalization_results[i].matches)
        {
            std::swap(result.queryIdx, result.trainIdx);
            result.trainIdx = key_frame.ids[result.trainIdx];
        }
        for( int j = 0; j < relocalization_results[i].matches.size(); j++)
        {
            auto &_175247759380 = relocalization_results[i].matches[ j ].trainIdx;
            if( !m_map->map_points.is(_175247759380))
                relocalization_results[i].matches[j].trainIdx = -1;

            if( m_map->map_points[_175247759380 ].isBad())
                relocalization_results[i].matches[j].trainIdx = -1;
        }
        remove_unused_matches(relocalization_results[i].matches);
        if(relocalization_results[i].matches.size() < 25)
            continue ;

        relocalization_results[i].se3_transform = key_frame.pose_f2g;
        PnPSolver::solvePnPRansac(frame, m_map,
            relocalization_results[i].matches, relocalization_results[i].se3_transform);
        if(relocalization_results[i].matches.size() < 15)
            continue ;

        relocalization_results[i].matches = 
            m_map->matchFrameToMapPoints(m_map->TheKpGraph.getNeighborsVLevel2(index, true) ,
             frame, relocalization_results[i].se3_transform ,m_params.maxDescDistance*2, 2.5,true );
        if( relocalization_results[i].matches.size() < 30)
            continue ;

        PnPSolver::solvePnp(frame, m_map, relocalization_results[i].matches, relocalization_results[i].se3_transform);
        if(relocalization_results[i].matches.size() < 30)
            continue ;

        relocalization_results[i].frame_ids = frame.ids;
        for( auto match : relocalization_results[i].matches)
            relocalization_results[i].frame_ids[match.queryIdx] = match.trainIdx;

    }
    std::remove_if(relocalization_results.begin(), relocalization_results.end(), [](const KeyPointRelocResult &result) {
              return result.matches.size() <= 30;});
    std::sort(relocalization_results.begin(), relocalization_results.end(), [](const KeyPointRelocResult &left, 
                  const KeyPointRelocResult &right) {
                  return left.matches.size() > right.matches.size();});

    return relocalization_results;
}


bool System::reLocalizationWithKeyPoints(Frame &frame, se3 &se3_transform, const std::set<uint32_t> &excluded_frames) {
     
    auto results = getKeyPointRelocResult(frame, se3_transform, excluded_frames);
    if(results.size() == 0)
        return false;
    if(results[0].matches.size() > 30)
    {
        se3_transform = results[0].se3_transform;
        frame.ids = results[0].frame_ids;
        return true;
    }
    else
        return false;
}

bool System::reLocalizationWithMarkers(Frame &frame, se3 &se3_transform) {

    if(frame.markers.size() == 0)
        return false;
    vector<uint32_t> marker_id_vector;
    for(auto &marker : frame.markers)
    {
        auto iter = m_map->map_markers.find(marker.id);
        if( iter != m_map->map_markers.end())
            if(iter->second.pose_g2m.isValid())
                marker_id_vector.push_back(marker.id);
    }
    if(marker_id_vector.size() == 0)
        return false;

    se3_transform = m_map->getBestPoseFromValidMarkers(frame, marker_id_vector,
        m_params.aruco_minerrratio_valid);

    return se3_transform.isValid();
}

bool System::reLocalization(Frame &frame, se3 &se3_transform) {

    se3_transform = se3();
    if(m_params.reLocalizationWithMarkers)
    {
        if(reLocalizationWithMarkers(frame, se3_transform))
            return true;
    }
    if(m_params.reLocalizationWithKeyPoints)
    {
        if(reLocalizationWithKeyPoints(frame, se3_transform))
            return true;
    }
    return false;
}

std::vector<cv::DMatch> System::matchFrames(Frame& curr_frame, Frame &prev_frame, float dist_thresh, float proj_dist_thresh) {

    std::vector<cv::DMatch> matches;
    for(size_t i = 0; i < prev_frame.ids.size(); i++)
    {
        uint32_t keypoint_id = prev_frame.ids[i];
        if(keypoint_id != std::numeric_limits<uint32_t>::max())
        {
            if(m_map->map_points.is(keypoint_id))
            {
                MapPoint &map_point = m_map->map_points[keypoint_id];
                if(map_point.isBad())
                    continue;
                auto proj_point = curr_frame.project(map_point.getCoordinates(), true, true);
                if(isnan(proj_point.x))
                    continue;
                float scale_factor = curr_frame.scaleFactors[prev_frame.und_kpts[i].octave] ;
                int octave = prev_frame.und_kpts[i].octave;
                std::vector<uint32_t> key_points_indexes = curr_frame.getKeyPointsInRegion(proj_point,
                      proj_dist_thresh * scale_factor, octave, octave);
                float _16940367568811467085 = dist_thresh + 0.01, min_desc_dist = std::numeric_limits<float>::max();
                uint32_t query_idx = std::numeric_limits<uint32_t>::max();
                for(auto index : key_points_indexes)
                {
                    if(curr_frame.und_kpts[index].octave == prev_frame.und_kpts[i].octave)
                    {
                        float dist = MapPoint::getDescDistance(prev_frame.desc, i, curr_frame.desc, index);
                        if(dist < _16940367568811467085)
                        {
                            _16940367568811467085 = dist;
                            query_idx = index;
                        }
                        else if(dist < min_desc_dist)
                        {
                            min_desc_dist = dist;
                        }
                    }
                }
                if(query_idx != std::numeric_limits<uint32_t>::max() && _16940367568811467085 < 0.7 * min_desc_dist)
                {
                    cv::DMatch match;
                    match.queryIdx = query_idx;
                    match.trainIdx = map_point.id;
                    match.distance = _16940367568811467085;
                    matches.push_back(match);
                }
            }
        }
    }

    filter_ambiguous_query(matches);
    return matches;
}



se3 System::updatePose(Frame &frame, se3 se3_transform) {

    std::vector<cv::DMatch> matches;
    se3 update_se3_transform = se3_transform;
    if(m_map->map_points.size() > 0)
    {
        if(!_14463320619150402643.empty())
            update_se3_transform = _14463320619150402643 * update_se3_transform.convert();
        frame.pose_f2g = update_se3_transform;
        matches = matchFrames(frame, m_prev_frame, m_params.maxDescDistance * 1.5, m_params.projDistThr);
        int good_matches_num = 0;
        if(matches.size() > 30)
        {
            auto estimated_pose = update_se3_transform;
            good_matches_num = PnPSolver::solvePnp(frame, m_map, matches, estimated_pose, m_keyframe_Index);
            if(good_matches_num > 30)
                update_se3_transform = estimated_pose;
        }
        else 
        {
            FrameMatcher frame_matcher(FrameMatcher::TYPE_FLANN);
            frame_matcher.setParams(m_map->keyframes[m_keyframe_Index], FrameMatcher::MODE_ASSIGNED,
                m_params.maxDescDistance * 2, 0.6, true, 3);

            matches = frame_matcher.match(frame, FrameMatcher::MODE_ALL);
            if(matches.size() > 30)
            {
                for(auto &match : matches)
                    match.trainIdx = m_map->keyframes[m_keyframe_Index].ids[match.trainIdx];

                auto estimated_pose =  update_se3_transform;
                good_matches_num = PnPSolver::solvePnp(frame, m_map, matches, estimated_pose, m_keyframe_Index);
                if( good_matches_num > 30 )
                    update_se3_transform = estimated_pose;
            }
            else
               good_matches_num = 0;
        }
        float max_repj_dist;
        if(good_matches_num > 30)
        {
            max_repj_dist = 4;
            for( auto match : matches)
            {
                m_map->map_points[match.trainIdx].lastFIdxSeen = frame.fseq_idx ;
                m_map->map_points[match.trainIdx].setVisible();
            }
        }
        else 
        {
            matches.clear();
            max_repj_dist = m_params.projDistThr;
        }
        auto new_matches = m_map->matchFrameToMapPoints(
          m_map->TheKpGraph.getNeighborsVLevel2(m_keyframe_Index, true), 
          frame, update_se3_transform, m_params.maxDescDistance * 2, max_repj_dist, true);

        matches.insert(matches.end(), new_matches.begin(), new_matches.end());
        filter_ambiguous_query(matches);
    }
    int update_matches = PnPSolver::solvePnp(frame, m_map, matches, update_se3_transform, m_keyframe_Index);
    bool flag = false;
    if(frame.markers.size() > 0) 
    {
        int count = 0;
        for(size_t i = 0; i < frame.markers.size(); i++)
        {
            auto iter = m_map->map_markers.find(frame.markers[i].id);
            if(iter == m_map->map_markers.end())
                continue;
            if (!iter->second.pose_g2m.isValid())
                continue;
            count++;
            if(frame.markers[i].poses.err_ratio < m_params.aruco_minerrratio_valid)
                continue;
            flag = true;
            break ;
        }
        if(count > 1)
            flag = true;

    }
    if(update_matches < 30 && !flag)
    {
      return se3();
    }
    for(size_t i = 0 ;i < matches.size(); i++)
    {
        m_map->map_points[matches[i].trainIdx].setSeen();
        frame.ids[matches[i].queryIdx] = matches[i].trainIdx;
        if(matches[i].imgIdx == -1)
            frame.flags[matches[i].queryIdx].set(Frame::FLAG_OUTLIER, true);
    }
    return update_se3_transform;
}


void System::drawImage(cv::Mat &image, float scale) const {
    int lineWidth = float(image.cols) / 640.f;
    cv::Point2f point(lineWidth, lineWidth);
    bool flag = false;
    if(m_state == STATE_TRACKING)
    {
        for(size_t i = 0; i < m_current_frame.ids.size(); i++)
        {
            if(m_current_frame.ids[i] != std::numeric_limits<uint32_t>::max()) 
            {
                if(!m_map->map_points.is(m_current_frame.ids[i]))
                    continue;
                cv::Scalar color(0, 255,0);
                if(!m_map->map_points[m_current_frame.ids[i]].isStable()) 
                {
                    color = cv::Scalar(0, 0, 255);
                }
                cv::rectangle(image, scale * (m_current_frame.kpts[i] - point),
                scale * (m_current_frame.kpts[i] + point), color, lineWidth);
            }
            else if(flag)
            {
                cv::Scalar color(255, 0, 0);
                cv::rectangle(image, scale * (m_current_frame.kpts[i] - point),
                scale * (m_current_frame.kpts[i] + point), color, lineWidth);
            }
        }
    }
    else if(m_map->isEmpty())
    {
        for(auto iter : m_current_frame.kpts)
            cv::rectangle(image, scale * (iter - point), 
            scale * (iter + point), cv::Scalar(255, 0, 0), lineWidth);
    }
    for(auto marker : m_current_frame.markers)
    {
        cv::Scalar color = cv::Scalar(0, 244, 0);
        if(m_map->map_markers.count(marker.id) != 0)
        {
            if(m_map->map_markers.at(marker.id).pose_g2m.isValid())
                color = cv::Scalar(255, 0, 0);
            else
                color = cv::Scalar(0, 0, 255);
        }
        for(auto &iter : marker.corners)
            iter *= scale;
        for(auto &iter : marker.und_corners)
            iter *= scale;
        marker.draw(image, color);
    }
}


void System::globalOptimization() {
    auto optimizer = GlobalOptimizer::create(m_params.global_optimizer);
    GlobalOptimizer::ParamSet param_set(debug::Debug::getLevel() >= 11);
    param_set.fixFirstFrame = true ;
    param_set.nIters = 10 ;
    param_set.markersOptWeight = getParams().markersOptWeight;
    param_set.minMarkersForMaxWeight = getParams().minMarkersForMaxWeight;
    param_set.InPlaneMarkers = getParams().inPlaneMarkers;
    optimizer->optimize(m_map, param_set);
    m_map->removeBadAssociations(optimizer->getBadAssociations(), 2);
}

uint32_t System::getLastProcessedFrame() const {
  return m_current_frame.fseq_idx;
}

void System::setMode(MODES mode) {
    m_mode = mode;
}

void System::clear() {
    m_map_manager = std::make_shared<MapManager>();
    firstFrameFlag = false;
    m_state = STATE_LOST;
    m_map.reset();
    m_map_initializer = std::make_shared<MapInitializer>();
    _14463320619150402643 = cv::Mat();
    m_fseq_idx = -1;
}

void System::saveToFile(string filepath)
{
    waitForFinished();
    fstream fstr(filepath, ios_base::binary | ios_base::out);
    if(!fstr)
      throw std::runtime_error(string(__PRETTY_FUNCTION__) + "could not open file for writing:" + filepath);
    io_write <uint64_t>(182312, fstr);
    m_map->toStream(fstr);
    m_params.toStream(fstr);
    fstr.write((char* )&m_se3, sizeof(m_se3));
    fstr.write((char* )&m_keyframe_Index, sizeof(m_keyframe_Index));
    fstr.write((char* )&firstFrameFlag, sizeof(firstFrameFlag));
    fstr.write((char* )&m_state, sizeof(m_state));
    fstr.write((char* )&m_mode, sizeof(m_mode));
    m_current_frame.toStream(fstr);
    m_prev_frame.toStream(fstr);
    m_frame_extractor->toStream(fstr);
    m_map_manager->toStream(fstr);
    toStream__(_14463320619150402643, fstr);
    fstr.write((char* )&m_fseq_idx,sizeof(m_fseq_idx));
    fstr.write((char* )&_13033649816026327368,sizeof(_13033649816026327368));
    fstr.flush();
}

void System::readFromFile(string filepath) {
    ifstream istr(filepath, ios::binary);
    if(!istr)
        throw std::runtime_error(string(__PRETTY_FUNCTION__ ) + "could not open file for reading:" + filepath);
    if(io_read <uint64_t>(istr) != 182312)
        throw std::runtime_error(string(__PRETTY_FUNCTION__ ) + "invalid file type:" + filepath);
    m_map = std::make_shared<Map>();
    m_map->fromStream(istr);
    m_params.fromStream(istr);
    istr.read((char*)&m_se3, sizeof(m_se3));
    istr.read((char*)&m_keyframe_Index, sizeof(m_keyframe_Index));
    istr.read((char*)&firstFrameFlag, sizeof(firstFrameFlag));
    istr.read((char*)&m_state, sizeof(m_state));
    istr.read((char*)&m_mode, sizeof(m_mode));
    m_current_frame.fromStream(istr);
    m_prev_frame.fromStream(istr);
    m_frame_extractor->fromStream(istr);
    m_map_manager->fromStream(istr);
    fromStream__(_14463320619150402643, istr);
    istr.read((char*)&m_fseq_idx, sizeof(m_fseq_idx));
    istr.read((char*)&_13033649816026327368, sizeof(_13033649816026327368));
}
}
