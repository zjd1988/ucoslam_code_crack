#include "utils/frameextractor.h"
#include "basictypes/cvversioning.h"
#include "basictypes/misc.h"
#include "basictypes/osadapter.h"
#include "basictypes/timers.h"
#include "map_types/mappoint.h"
#include "optimization/ippe.h"
#include <aruco/markerdetector.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
using namespace std;
namespace ucoslam
{

  void FrameExtractor::toStream(std::ostream &str) const
  {
    uint64_t signature = 1923123;
    str.write((char *)&signature, sizeof(signature));
    m_feature_detector->toStream(str);

    str.write((char *)&_12273370977065616393, sizeof(_12273370977065616393));
    str.write((char *)&_12350051723532614025, sizeof(_12350051723532614025));
    str.write((char *)&m_detect_marker, sizeof(m_detect_marker));
    str.write((char *)&m_detect_keypoints, sizeof(m_detect_keypoints));
    str.write((char *)&m_marker_size, sizeof(m_marker_size));
    str.write((char *)&m_feat_params, sizeof(m_feat_params));
    str.write((char *)&m_max_desc_distance, sizeof(m_max_desc_distance));
    m_marker_detector->toStream(str);
    m_ucoslam_params.toStream(str);
  }

  void FrameExtractor::fromStream(std::istream &str)
  {

    uint64_t signature = 1923123;
    str.read((char *)&signature, sizeof(signature));
    if (signature != 1923123)
      throw std::runtime_error(string(__PRETTY_FUNCTION__) + "invalid signature");
    m_feature_detector = Feature2DSerializable::fromStream(str);
    str.read((char *)&_12273370977065616393, sizeof(_12273370977065616393));
    str.read((char *)&_12350051723532614025, sizeof(_12350051723532614025));
    str.read((char *)&m_detect_marker, sizeof(m_detect_marker));
    str.read((char *)&m_detect_keypoints, sizeof(m_detect_keypoints));
    str.read((char *)&m_marker_size, sizeof(m_marker_size));
    str.read((char *)&m_feat_params, sizeof(m_feat_params));
    str.read((char *)&m_max_desc_distance, sizeof(m_max_desc_distance));
    m_marker_detector->fromStream(str);
    m_ucoslam_params.fromStream(str);
  }

  FrameExtractor::FrameExtractor()
  {
    m_marker_detector = std::make_shared<aruco::MarkerDetector>();
  }

  void FrameExtractor::setParams(std::shared_ptr<Feature2DSerializable> feature_detector, const Params &params)
  {

    m_feature_detector = feature_detector;
    aruco::MarkerDetector::Params marker_detect_params;
    marker_detect_params.setDetectionMode(
        aruco::MarkerDetector::Params::getDetectionModeFromString(params.aruco_DetectionMode),
        params.aruco_minMarkerSize);
    marker_detect_params.dictionary = params.aruco_Dictionary;
    marker_detect_params.setCornerRefinementMethod(
        aruco::MarkerDetector::Params::getCornerRefinementMethodFromString(params.aruco_CornerRefimentMethod));

    m_ucoslam_params = params;
    m_detect_marker = params.detectMarkers;
    m_detect_keypoints = params.detectKeyPoints;
    m_marker_detector->setParameters(marker_detect_params);
    m_marker_size = params.aruco_markerSize;
    m_feat_params = Feature2DSerializable::FeatParams(
        params.maxFeatures, params.nOctaveLevels,
        params.scaleFactor,
        params.nthreads_feature_detector);

    m_max_desc_distance = params.maxDescDistance;
  }

  void FrameExtractor::setSensitivity(float v)
  {
    if (m_feature_detector)
      m_feature_detector->setSensitivity(v);
  }

  float FrameExtractor::getSensitivity()
  {
    if(!m_feature_detector)
      throw std::runtime_error(string(__PRETTY_FUNCTION__) + "Should not call this function since the class is not initialized");
    return m_feature_detector->getSensitivity();
  }

  void FrameExtractor::processStereo(const cv::Mat &LeftRect, const cv::Mat &RightRect,
                                     const ImageParams &ip, Frame &frame, uint32_t frameseq_idx) {

    prepare(m_ucoslam_params.kptImageScaleFactor, LeftRect, ip, RightRect);
    extract(m_image_stat_info[0], frame, frameseq_idx);
    frame.depth.resize(frame.und_kpts.size());
    for (size_t i = 0; i < frame.depth.size(); i++)
      frame.depth[i] = 0;
    vector<cv::KeyPoint> keypoints_vector;
    cv::Mat descriptors;
    m_feature_detector->detectAndCompute(m_image_stat_info[1].resized_gray_image, cv::Mat(),
        keypoints_vector, descriptors, m_feat_params);
    vector<vector<int>> keypoints_indexes_vector(m_image_stat_info[1].resized_gray_image.rows);

    for (size_t i = 0; i < keypoints_vector.size(); i++)
    {
      double zero = 0;
      double keypoint_y = keypoints_vector[i].pt.y;
      int min_y = std::max(0, int(std::round(keypoint_y - zero)));
      int max_y = std::min(m_image_stat_info[1].resized_gray_image.rows - 1, int(std::round(keypoint_y + zero)));
      for (int index = min_y; index <= max_y; index++)
        keypoints_indexes_vector[index].push_back(i);
    }
    int count = 0;
    for (size_t i = 0; i < frame.und_kpts.size(); i++)
    {
      int keypoint_y = std::round(frame.und_kpts[i].pt.y);
      vector<int> &keypoint_indexes = keypoints_indexes_vector[keypoint_y];
      int match_index = -1;
      double min_desc_dist = std::numeric_limits<double>::max();
      for (size_t j = 0; j < keypoint_indexes.size(); j++)
      {
        int index = keypoint_indexes[j];
        if (keypoints_vector[index].pt.x > frame.und_kpts[i].pt.x ||
            std::abs(keypoints_vector[index].octave - frame.und_kpts[i].octave) > 1)
          continue;
        auto distance = MapPoint::getDescDistance(frame.desc, i, descriptors, index);
        if (distance < m_max_desc_distance)
        {
          if (distance < min_desc_dist)
          {
            min_desc_dist = distance;
            match_index = index;
          }
        }
      }
      if (match_index != -1)
      {
        int reserve_value = 7;
        int half_reserve_value = reserve_value / 2;
        int left_keypoint_x = std::round(frame.und_kpts[i].pt.x);
        int left_keypoint_y = std::round(frame.und_kpts[i].pt.y);
        if (left_keypoint_x < half_reserve_value || left_keypoint_x + half_reserve_value >= LeftRect.cols)
          continue;
        if (left_keypoint_y < half_reserve_value || left_keypoint_y + half_reserve_value >= LeftRect.rows)
          continue;
        
        int right_keypoint_x = std::round(keypoints_vector[match_index].pt.x);
        int right_keypoint_y = std::round(keypoints_vector[match_index].pt.y);

        if (right_keypoint_x < half_reserve_value || right_keypoint_x + half_reserve_value >= RightRect.cols)
          continue;
        if (right_keypoint_y < half_reserve_value || right_keypoint_y + half_reserve_value >= RightRect.rows)
          continue;

        int _192620453243540296 = 7;
        int patch_num = _192620453243540296 * 2 + 1;
        vector<double> patch_diff_vec(patch_num);
        cv::Mat left_keypoint_patch = m_image_stat_info[0].resized_gray_image(
                cv::Range(left_keypoint_y - half_reserve_value,
                          left_keypoint_y + half_reserve_value),
                cv::Range(left_keypoint_x - half_reserve_value,
                          left_keypoint_x + half_reserve_value));
        int _1522763743271507664 = std::max(-_192620453243540296, -right_keypoint_x);
        int _1522768890174422034 = std::min(_192620453243540296, RightRect.cols - 1 - right_keypoint_x);
        double min_diff_sum = std::numeric_limits<double>::max();
        int patch_match_index = -1;
        for (int _46082575015037928 = _1522763743271507664; _46082575015037928 <= _1522768890174422034; _46082575015037928++)
        {
          int _6806984958533886427 = _46082575015037928 + _192620453243540296;
          int _175247759696 = right_keypoint_x + _46082575015037928;
          cv::Mat right_keypoint_patch = m_image_stat_info[1].resized_gray_image(
                  cv::Range(right_keypoint_y - half_reserve_value, right_keypoint_y + half_reserve_value),
                  cv::Range(_175247759696 - half_reserve_value, _175247759696 + half_reserve_value));
          cv::Mat patch_diff;
          cv::absdiff(left_keypoint_patch, right_keypoint_patch, patch_diff);
          double absdiff_sum = cv::sum(patch_diff)[0];
          if (absdiff_sum < min_diff_sum)
          {
            min_diff_sum = absdiff_sum;
            patch_match_index = _6806984958533886427;
          }
          patch_diff_vec[_6806984958533886427] = absdiff_sum;
        }
        if (patch_match_index > _1522763743271507664 + _192620453243540296 && patch_match_index < _1522768890174422034 + _192620453243540296)
        {
          double _175247759918 = patch_diff_vec[patch_match_index - 1];
          double _175247759917 = patch_diff_vec[patch_match_index];
          double _175247759916 = patch_diff_vec[patch_match_index + 1];
          double _16989176769678974579 = 0.5 * ( _175247759918 - _175247759916) / (_175247759918 + _175247759916 - 2 * _175247759917)
                  + patch_match_index - _192620453243540296;
          double _3378217371725605483 = keypoints_vector[match_index].pt.x +_16989176769678974579;
          frame.depth[i] = ( ip.bl * ip.fx()) / (frame.und_kpts[i].pt.x -_3378217371725605483);
          count++;
        }
      }
    }
  }


  void FrameExtractor::process_rgbd(const cv::Mat &image, const cv::Mat &depthImage,
          const ImageParams &ip, Frame &frame, uint32_t frameseq_idx) {

    prepare(1, image, ip);
    extract(m_image_stat_info[0] , frame, frameseq_idx);
    frame.depth.resize(frame.und_kpts.size());
    for ( size_t i = 0; i < frame.depth.size(); i++ )
      frame.depth[i] = 0;
    for (size_t i = 0; i < frame.kpts.size(); i++)
    {
      if( depthImage.at<uint16_t>( frame.kpts[i]) != 0)
      {
        frame.depth[i] = depthImage.at<uint16_t>(frame.kpts[i]) * ip.rgb_depthscale;
      }
    }
  }

  void FrameExtractor::process(const cv::Mat &image, const ImageParams &ip, Frame &frame,
      uint32_t frameseq_idx) {

    prepare(m_ucoslam_params.kptImageScaleFactor, image, ip);
    extract(m_image_stat_info[0], frame, frameseq_idx);
  }

  void FrameExtractor::prepare(float scale_factor, const cv::Mat &first_image,
                            const ImageParams &ip, const cv::Mat &second_image) {

    int image_count = 1;
    if ( !second_image.empty())
      image_count++;
    m_image_stat_info.resize(image_count);
    if (first_image.channels() == 3)
      cv::cvtColor(first_image, m_image_stat_info[0].origin_gray_image, cv::COLOR_BGR2GRAY);
    else
      m_image_stat_info[0].origin_gray_image = first_image;
    m_image_stat_info[0].origin_ip = ip;
    if (!second_image.empty())
    {
      if (second_image.channels() == 3)
        cv::cvtColor( second_image, m_image_stat_info[1].origin_gray_image, cv::COLOR_BGR2GRAY);
      else
        m_image_stat_info[1].origin_gray_image = second_image;
      m_image_stat_info[1].origin_ip = ip;
    }
    if (fabs(1 - scale_factor) > 1e-3)
    {
      cv::Size image_size(first_image.cols * scale_factor, first_image.rows * scale_factor);
      if (image_size.width % 4 != 0)
        image_size.width += 4 - image_size.width % 4;
      if (image_size.height % 2 != 0)
        image_size.height++;
      cv::resize(m_image_stat_info[0].origin_gray_image, m_image_stat_info[0].resized_gray_image, image_size);
      m_image_stat_info[0].resized_ip = m_image_stat_info[0].origin_ip;
      m_image_stat_info[0].resized_ip.resize(image_size);
      m_image_stat_info[0].scale_orign_resized.first = float( image_size.height) / float(first_image.rows);
      m_image_stat_info[0].scale_orign_resized.second = float(image_size.width) / float(first_image.cols);
      if (!second_image.empty())
      {
        cv::resize(m_image_stat_info[1].origin_gray_image, m_image_stat_info[1].resized_gray_image, image_size);
        m_image_stat_info[1].resized_ip = m_image_stat_info[2].origin_ip;
        m_image_stat_info[1].resized_ip.resize(image_size);
        m_image_stat_info[1].scale_orign_resized.first = float(image_size.height) / float(second_image.rows);
        m_image_stat_info[1].scale_orign_resized.second =float(image_size.width) / float(second_image.cols);
      }
    }
    else
    {
      for (int i = 0; i < m_image_stat_info.size(); i++)
      {
        m_image_stat_info[i].resized_gray_image = m_image_stat_info[i].origin_gray_image;
        m_image_stat_info[i].resized_ip = m_image_stat_info[i].origin_ip;
        m_image_stat_info[i].scale_orign_resized = std::pair<float, float>(1, 1);
      }
    }
  }

  void FrameExtractor::extract(const ImageInfo&image_info, Frame &frame, uint32_t frameseq_idx) {

    frame.clear();
    vector<cv::KeyPoint> detect_keypoints;
    std::thread feature_detect_thread([&] {
      if (m_detect_keypoints)
      {
        m_feature_detector->detectAndCompute(m_image_stat_info[0].resized_gray_image,
                  cv::Mat(), detect_keypoints, frame.desc, m_feat_params);
        frame.KpDescType = m_feature_detector->getDescriptorType();
      }
    });

    std::thread marker_detect_thread([&] {
      if(m_detect_marker) 
      {
        vector<aruco::Marker> markers_vector = m_marker_detector->detect(image_info.origin_gray_image);
        for (const auto &marker : markers_vector)
        {
          ucoslam::MarkerObservation marker_observation;
          marker_observation.id = marker.id;
          marker_observation.ssize = m_marker_size;
          marker_observation.corners = marker;
          marker_observation.dict_info = marker.dict_info;
          auto pose_esti = IPPE::solvePnP_(m_marker_size, marker, image_info.origin_ip.CameraMatrix, image_info.origin_ip.Distorsion);
          for (int i = 0; i < 2; i++)
          {
            marker_observation.poses.errs[i] = pose_esti[i].second;
            marker_observation.poses.sols[i]= pose_esti[i].first.clone();
          }
          marker_observation.poses.err_ratio = pose_esti[1].second / pose_esti[0].second;
          for (auto &corner : marker_observation.corners)
          {
            corner.x *= image_info.scale_orign_resized.first;
            corner.y *= image_info.scale_orign_resized.second;
          }
          frame.markers.push_back(marker_observation);
        }
      }
    });

    feature_detect_thread.join();
    marker_detect_thread.join();
    if (debug::Debug::getLevel() >= 100 || m_save_image_flag)
      image_info.origin_gray_image.copyTo(frame.image);
    frame.scaleFactors.resize(m_feature_detector->getParams().nOctaveLevels);
    double scale_factor = m_feature_detector->getParams().scaleFactor;
    frame.scaleFactors[0] = 1;
    for (size_t i = 1; i < frame.scaleFactors.size();i++)
      frame.scaleFactors[i] = frame.scaleFactors[i - 1] * scale_factor;
    if(detect_keypoints.size() > 0)
    {
      vector<cv::Point2f> undistort_points;
      undistort_points.reserve(detect_keypoints.size());
      for (auto keypoint : detect_keypoints)
        undistort_points.push_back(keypoint.pt);
      m_image_stat_info[0].resized_ip.undistortPoints(undistort_points);
      frame.und_kpts = detect_keypoints;
      frame.kpts.resize(detect_keypoints.size());
      for(size_t i = 0; i < detect_keypoints.size(); i++)
      {
        frame.kpts[i] = detect_keypoints[i].pt;
        frame.und_kpts[i].pt = undistort_points[i];
      }
    }
    for (auto &marker : frame.markers)
    {
      marker.und_corners = marker.corners;
      m_image_stat_info[0].resized_ip.undistortPoints(marker.und_corners);
    }
    frame.flags.resize(frame.und_kpts.size());
    for (auto &flag : frame.flags)
      flag.reset();
    frame.ids.resize(frame.und_kpts.size());
    uint32_t max_uint_value = std::numeric_limits<uint32_t>::max();
    for (auto &id : frame.ids)
      id = max_uint_value;
    frame.idx = std::numeric_limits<uint32_t>::max();
    frame.fseq_idx = frameseq_idx;
    frame.imageParams = m_image_stat_info[0].resized_ip;
    frame.create_kdtree();
    frame.minXY = cv::Point2f(0, 0);
    frame.maxXY = cv::Point2f(frame.imageParams.CamSize.width, frame.imageParams.CamSize.height);
    if(frame.imageParams.Distorsion.total() != 0)
    {
      vector<cv::Point2f> point = {frame.minXY, frame.maxXY};
      frame.imageParams.undistortPoints(point);
      frame.minXY = point[0];
      frame.maxXY = point[1];
    }
  }
} // namespace ucoslam
