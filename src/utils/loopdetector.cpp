#include "utils/loopdetector.h"
#include "optimization/graphoptsim3.h"
#include "basictypes/misc.h"
#include "utils/system.h"
#include "basictypes/timers.h"
#include "optimization/pnpsolver.h"
#include "basictypes/io_utils.h"
#include "basictypes/hash.h"
#include "utils/framematcher.h"
#include "basictypes/osadapter.h"
#include <xflann/xflann.h>
using namespace std;
namespace ucoslam {

void LoopDetector::setParams(std::shared_ptr<Map> map) 
{
  TheMap = map;
}

LoopDetector::LoopClosureInfo LoopDetector::detectLoopFromMarkers(Frame &frame, int32_t curRefKf)
{
  vector<LoopClosureInfo> loop_closure_infos;
  if(TheMap->map_markers.size() > 0 && frame.markers.size() != 0)
  {
    loop_closure_infos = getLoopClosureInfo(frame, curRefKf);
  }
  if (loop_closure_infos.size() == 0)
    return LoopDetector::LoopClosureInfo();
  for (auto &info : loop_closure_infos)
    _6767859416531794787(frame, info);

  int index = 0;
  if(loop_closure_infos.size() > 1)
  {
    double reprojet_err0 = calcGlobalReprojChi2(loop_closure_infos[0]);
    double reprojet_err1 = calcGlobalReprojChi2(loop_closure_infos[1]);
    if (reprojet_err1 < reprojet_err0)
      index = 1;
  }

  auto &lcsol = loop_closure_infos[index];
  lcsol.map_matches = TheMap->matchFrameToMapPoints(
          TheMap->TheKpGraph.getNeighborsV(lcsol.matchingFrameIdx, true),
          frame, lcsol.expectedPos,System::getParams().maxDescDistance *1.5,
          2.5, false, true);
  return lcsol;
}

LoopDetector::LoopClosureInfo LoopDetector::detectLoopFromKeyPoints(
            Frame &frame, int32_t curRefKf) 
{

  vector<LoopClosureInfo> loop_closure_info;
  if( frame.ids.size()!= 0) 
  {
    loop_closure_info = _8671179296205241382(frame, curRefKf);
  }
  if(loop_closure_info.size() == 0)
    return LoopDetector::LoopClosureInfo();
  else 
  {
    _6767859416531794787(frame, loop_closure_info[0]);
    return loop_closure_info[0];
  }
}

vector<LoopDetector::LoopClosureInfo> LoopDetector::getLoopClosureInfo(Frame &frame, int64_t curRefKf)
{

  auto getValidMarkerCount = [&](const vector<uint32_t>& marker_ids) {
    int count = 0;
    for (auto index : marker_ids)
      if (TheMap->map_markers[index].pose_g2m.isValid())
        count++;
    return count;
  };
  // LoopClosureInfo _11093822343890;
  std::set<uint32_t> frame_indexes = TheMap->getNeighborKeyFrames(curRefKf, true);
  std::set<uint32_t> keyframe_marker_ids;
  for (auto index : frame_indexes)
    for (auto marker : TheMap->keyframes[index].markers)
      keyframe_marker_ids.insert(marker.id);

  vector<uint32_t> marker_ids;
  for(size_t i = 0; i < frame.markers.size(); i++)
  {
    auto &marker = frame.markers[i];
    if(keyframe_marker_ids.count(marker.id) != 0)
      continue;
    auto iter = TheMap->map_markers.find(marker.id);
    if (iter != TheMap->map_markers.end())
    {
      marker_ids.push_back(marker.id);
    }
  }
  if(getValidMarkerCount(marker_ids) == 0)
    return {};

  vector<se3> _15853982152702744598;
  se3 _14756128340231943706 = TheMap->getBestPoseFromValidMarkers(frame, marker_ids, 4);
  if(!_14756128340231943706.isValid() && getValidMarkerCount(marker_ids) < 3)
  {
    for(auto marker_id : marker_ids)
    {
      auto iter = std::find_if(frame.markers.begin(),frame.markers.end(),
                [&](const ucoslam::MarkerObservation &marker_observation){
                    return marker_observation.id == marker_id;
                });

      if(iter != frame.markers.end())
        frame.markers.erase(iter);
    }
    return {};
  }

  if(_14756128340231943706.isValid())
    _15853982152702744598.push_back(_14756128340231943706);
  else 
  {
    int min_err_index = 0;
    for (size_t i = 1; i < marker_ids.size(); i++)
      if (frame.getMarkerPoseIPPE(marker_ids[i]).err_ratio >
          frame.getMarkerPoseIPPE(marker_ids[min_err_index]).err_ratio)
        min_err_index = i;
    int marker_index = marker_ids[min_err_index];
    cv::Mat _706246308705962 = TheMap->map_markers[marker_index].pose_g2m.inv();
    se3 _46082575775659958 = se3(frame.getMarkerPoseIPPE(marker_index).sols[0] * _706246308705962);
    se3 _46082575775659953 = se3(frame.getMarkerPoseIPPE(marker_index).sols[1] * _706246308705962);
    _15853982152702744598.push_back(_46082575775659958);
    _15853982152702744598.push_back(_46082575775659953);
  }

  uint32_t fseq_idx = std::numeric_limits<uint32_t>::max(), matching_frame_idx =std::numeric_limits<uint32_t>::max();
  for (auto index : marker_ids)
  {
    for (auto frame_index : TheMap->map_markers[index].frames)
    {
      if (fseq_idx > TheMap->keyframes[frame_index].fseq_idx) 
      {
        fseq_idx = TheMap->keyframes[frame_index].fseq_idx;
        matching_frame_idx = frame_index;
      }
    }
  }
  vector<LoopClosureInfo> result;
  for (size_t i = 0; i < _15853982152702744598.size(); i++)
  {
    LoopClosureInfo loop_closure_info;
    loop_closure_info.curRefFrame = curRefKf;
    loop_closure_info.expectedPos = _15853982152702744598[i];
    loop_closure_info.matchingFrameIdx = matching_frame_idx;
    result.push_back(loop_closure_info);
  }
  return result;
}

void LoopDetector::_6767859416531794787(Frame &frame, LoopClosureInfo &loop_closure_info)
{
  CovisGraph &graph = TheMap->TheKpGraph;
  if(frame.idx == std::numeric_limits< uint32_t>::max())
    frame.idx = TheMap->getNextFrameIndex();

  vector<pair<uint32_t, uint32_t>> all_edges = graph.getAllEdges();
  if(!graph.isEdge(loop_closure_info.curRefFrame, frame.idx))

    ;

  all_edges.push_back({loop_closure_info.curRefFrame, frame.idx});
  auto &poses = loop_closure_info.optimPoses;
  std::map<uint64_t, float> edge_weights;
  float max_edge_weight = 0;
  for (auto edge : all_edges)
  {
    if(poses.count(edge.first) == 0 && TheMap->keyframes.is(edge.first))
      poses[edge.first] = TheMap->keyframes[edge.first].pose_f2g;
    if(poses.count(edge.second)==0 && TheMap->keyframes.is(edge.second))
      poses[edge.second] = TheMap->keyframes[edge.second].pose_f2g;

    float edge_weight = 0;
    if(TheMap->TheKpGraph.isEdge(edge.first, edge.second))
      edge_weight = TheMap->TheKpGraph.getEdge(edge.first, edge.second);
    max_edge_weight = std::max(edge_weight, max_edge_weight);

    edge_weights[CovisGraph::join(edge.first, edge.second)] = edge_weight;
  }

  if(!graph.isEdge(frame.idx, loop_closure_info.matchingFrameIdx)) 
  {
    all_edges.push_back({frame.idx, loop_closure_info.matchingFrameIdx});
    edge_weights[CovisGraph::join(frame.idx, loop_closure_info.matchingFrameIdx)] = max_edge_weight;
  }
  poses[frame.idx] = frame.pose_f2g;

#pragma message "warning: must check fixscale"

  loopClosurePathOptimizationg2o(all_edges, frame.idx,
      loop_closure_info.matchingFrameIdx, loop_closure_info.expectedPos, poses, false);
}

void LoopDetector::correctMap(const LoopClosureInfo &lcsol)
{
  auto _1524145530871351014 = [](const cv::Mat &pose) {

    float norm_value = cv::norm(pose.col(0));
    cv::Mat _175247761732 = pose;
    cv::Mat region = pose.rowRange(0, 3).colRange(0, 3);
    region = (1. / norm_value) * region;
    return _175247761732;
  };

  for (auto &marker_info : TheMap->map_markers)
  {
    Marker &marker = marker_info.second;
    if(!marker.pose_g2m.isValid())
      continue;

    bool flag = false;
    for (auto frame_index : marker.frames)
    {
      if (lcsol.optimPoses.count(frame_index) == 0)
        continue;
      const Frame &frame = TheMap->keyframes[frame_index];
      cv::Mat _11093822386652 = frame.pose_f2g * marker.pose_g2m;
      auto _3005399769884158829 = _1524145530871351014(lcsol.optimPoses.at(frame_index));
      marker.pose_g2m = _3005399769884158829.inv() * _11093822386652;
      flag = true;
      break;
    }
    if(!flag)
      throw std::runtime_error(string(__PRETTY_FUNCTION__) + "Internal error. Could not correct marker");
  }
  for(auto &map_point : TheMap->map_points) 
  {
    Frame &keyframe = TheMap->keyframes[map_point.getObservingFrames().front().first];
    cv::Point3f point_coor = keyframe.pose_f2g * map_point.getCoordinates();
    cv::Mat pose = lcsol.optimPoses.at(keyframe.idx);
    float norm_value = cv::norm(pose.col(0));

    cv::Mat region = pose.rowRange(0, 3).colRange(0, 3);
    region *= (1. / norm_value);
    cv::Mat inv_pose = pose.inv();
    region = inv_pose.rowRange(0, 3).colRange(0, 3);
    region *= (1. / norm_value);
    map_point.setCoordinates(inv_pose * point_coor);
  }

  for (const auto pose_info : lcsol.optimPoses)
  {
    if (TheMap->keyframes.count(pose_info.first))
    {

      TheMap->keyframes[pose_info.first].pose_f2g = _1524145530871351014(pose_info.second);
    }
  }
}

double LoopDetector::calcGlobalReprojChi2(const LoopClosureInfo &lcsol)
{
  std::map<uint32_t, se3> marker_pose;
  std::map<uint32_t, cv::Point3f> map_point_info;
  std::map<uint32_t, Se3Transform> frame_pose;
  for (auto marker_info : TheMap->map_markers)
    marker_pose[marker_info.first] = marker_info.second.pose_g2m;
  for (auto map_point : TheMap->map_points)
    map_point_info.insert({map_point.id, map_point.getCoordinates()});
  for (auto frame : TheMap->keyframes)
    frame_pose[frame.idx] = frame.pose_f2g;
  correctMap(lcsol);
  vector<uint32_t> frame_indexes;
  for(auto pose_info : lcsol.optimPoses)
    if (TheMap->keyframes.is(pose_info.first))
      frame_indexes.push_back(pose_info.first);
  auto reprojet_err = TheMap->globalReprojChi2(frame_indexes, 0, 0, true, true);
  for (auto &marker_info : TheMap->map_markers)
    marker_info.second.pose_g2m = marker_pose[marker_info.first];
  for(auto &frame : TheMap->keyframes)
    frame.pose_f2g = frame_pose[frame.idx];
  for(auto &map_point : TheMap->map_points)
    map_point.setCoordinates(map_point_info[map_point.id]);
  return reprojet_err;
}

// vector<cv::DMatch> LoopDetector::_15519652129875366702(std::shared_ptr<Map> _11093822290287,
//         const Frame &, uint32_t _175247760268,
//         float _1686565542397313973, void *_16614902379131698322)

// {
//   xflann::Index *_6806993289704731004 =(xflann::Index *) _16614902379131698322;
//   Frame &_3005401612717573609 = _11093822290287->keyframes[_175247760268];
//   vector<uint32_t> _706246332805075;
//   _706246332805075.reserve( _3005401612717573609.und_kpts.size());

//   for (auto _46082543320896749 : _3005401612717573609.ids) 
//   {
//     if(_46082543320896749 !=std::numeric_limits<uint32_t>::max())
//       _706246332805075.push_back(_46082543320896749);
//   }
//   if(_706246332805075.size() == 0)
//     return {};

//   cv::Mat _2341894709189292181;
//   _11093822290287->map_points[_706246332805075[0]].getDescriptor(_2341894709189292181);
//   cv::Mat _7690820325099119768(_706246332805075.size(), _2341894709189292181.cols, _2341894709189292181.type());

//   for (size_t i = 0; i <_706246332805075.size(); i++)
//   {
//     cv::Mat _11093822304159 = _7690820325099119768.row(i);
//     _11093822290287->map_points[_706246332805075[i]].getDescriptor(_11093822304159);
//   }

//   cv::Mat indices, distances;
//   _6806993289704731004->search(_7690820325099119768, 2, indices, distances, xflann::KnnSearchParams(32, true));

//   if(distances.type() == 4)
//     distances.convertTo(distances, 5);

//   float _16988745808737061586 = _1686565542397313973;
//   float _14756094128870157179 = 0.9;
//   vector<cv::DMatch> matches;
//   for (int i = 0; i < indices.rows; i++) 
//   {
//     if(distances.at<float>(i, 0) < _16988745808737061586)
//     {
//       if (distances.at<float>(i, 0)<float(_14756094128870157179 * float(distances.at<float>(i, 1))))
//       {

//         cv::DMatch match;
//         match.queryIdx = indices.at<int>(i, 0);
//         match.trainIdx =_706246332805075[i];
//         match.distance = distances.at<float>(i, 0);
//         matches.push_back(match);
//       }
//     }
//   }

//   filter_ambiguous_query(matches);

//   return matches;
// }

std::vector<LoopDetector::LoopClosureInfo> LoopDetector::_8671179296205241382(Frame &frame, int32_t curRefKf) 
{
  // auto _5829441678613027716 = [](const uint32_t &_11093821926013) {
  //   std::stringstream _706246330191125;
  //   _706246330191125 << _11093821926013;
  //   return _706246330191125.str();
  // };

  if (!System::getParams().detectKeyPoints)
    return {};

  if (TheMap->TheKFDataBase.isEmpty())
    return {};
  auto frame_indexes = TheMap->TheKpGraph.getNeighbors(curRefKf, true);

  if(frame_indexes.size() == TheMap->keyframes.size())
    return {};

  float score = 1;
  for (auto i : frame_indexes)
    score = std::min(score, TheMap->TheKFDataBase.score(frame, TheMap->keyframes[i]));

  auto _13899933296976059300 = TheMap->TheKFDataBase.relocalizationCandidates(
          frame, TheMap->keyframes, TheMap->TheKpGraph, true, score / 2., frame_indexes);

  if(_13899933296976059300.size() == 0)
    return {};

  struct _1648036655000763724 
  {
    _1648036655000763724(uint32_t _2654435871) 
    {
      _17013904265820854 = _2654435871;
    }
    uint32_t _17013904265820854;
    cv::Mat _9332970625915525982 = cv::Mat();
    uint32_t _1994458605759073584 = 0;
    bool _1087568825552921310 = false;
    vector<cv::DMatch> _6744006314306065854;
  };
  vector<_1648036655000763724> _7244985490283130860;
  for (auto _175247760320 : _13899933296976059300)
    _7244985490283130860.push_back(_1648036655000763724(_175247760320));

  FrameMatcher frame_matcher;
  frame_matcher.setParams(frame, FrameMatcher::MODE_ALL, System::getParams().maxDescDistance * 2);

#pragma message "warning: Check the loop detector is correct with the FrameMatcher"

  for (auto &_706246351566300 : _7244985490283130860)
  {
    vector<cv::Point2f> _16937225740828189009;
    vector<cv::Point3f> _16937225740828192533;
    vector<cv::DMatch> _6807036698572949990;
    if (1)
    {
      auto &_3005399814901981436 = TheMap->keyframes[_706246351566300._17013904265820854];
      _6807036698572949990 = frame_matcher.match(_3005399814901981436, FrameMatcher::MODE_ASSIGNED);
      if (_6807036698572949990.size() < 30) 
      {
        _706246351566300._1087568825552921310 = true;
        continue;
      }

      for (auto _2654435878 : _6807036698572949990)
      {
        _16937225740828189009.push_back(frame.und_kpts[_2654435878.trainIdx].pt);
        _16937225740828192533.push_back(TheMap->map_points[_3005399814901981436.ids[_2654435878.queryIdx]].getCoordinates());
      }
    }

    cv::Mat _175247759698, _175247759831;
    vector<int> _6807141016080266659;
    cv::solvePnPRansac(_16937225740828192533, _16937225740828189009,
        frame.imageParams.CameraMatrix, cv::Mat::zeros(1, 5,5),
        _175247759698, _175247759831, false, 100, 2.5, 0.99,
        _6807141016080266659);
    if(_6807141016080266659.size() <15)
    {
      _706246351566300._1087568825552921310 = true;
      continue;
    }
    _706246351566300._1994458605759073584 = _6807141016080266659.size();
    _706246351566300._9332970625915525982 = getRTMatrix(_175247759698, _175247759831, 5);
    _706246351566300._6744006314306065854.reserve(_6807141016080266659.size());
    for (auto _175247760141 : _6807141016080266659)
      _706246351566300._6744006314306065854.push_back(_6807036698572949990[_175247760141]);
  }

  _7244985490283130860.erase(std::remove_if(_7244985490283130860.begin(),_7244985490283130860.end(),[](
    const _1648036655000763724 &_2654435868) {
    return _2654435868._1087568825552921310;
    }), _7244985490283130860.end());

  if( _7244985490283130860.size() == 0)
    return {};
  std::sort(_7244985490283130860.begin() ,_7244985490283130860.end(),[](
               const _1648036655000763724 &_175247762797,
               const _1648036655000763724 &_175247762798){
             return _175247762797._1994458605759073584 > _175247762798._1994458605759073584;
           });
  for (auto &_16119892890339758111 : _7244985490283130860) 
  {
    for (auto &_175247759380 : TheMap->map_points)
      _175247759380.lastFIdxSeen = std::numeric_limits<uint32_t>::max();
    _16119892890339758111._6744006314306065854 =TheMap->matchFrameToMapPoints(TheMap->
            TheKpGraph.getNeighborsV(_7244985490283130860[0]._17013904265820854,true),
            frame, _7244985490283130860[0]._9332970625915525982,
            System::getParams().maxDescDistance *1.5,2.5, false, true);

    if (_16119892890339758111._6744006314306065854.size()< 40)
      continue;
    se3 _6807035026667062616 = _16119892890339758111._9332970625915525982;
    PnPSolver::solvePnp(frame, TheMap, _16119892890339758111._6744006314306065854, _6807035026667062616, curRefKf);
    _16119892890339758111._9332970625915525982 = _6807035026667062616;
  }

  std::sort(_7244985490283130860.begin(),_7244985490283130860.end(),[](
              const _1648036655000763724 &_175247762797,
              const _1648036655000763724 &_175247762798){

            return _175247762797._6744006314306065854.size()>_175247762798._6744006314306065854.size();

          });

  if(_7244985490283130860[0]._6744006314306065854.size() < 30)
    return {};

  LoopClosureInfo result;
  result.curRefFrame = curRefKf;
  result.matchingFrameIdx = _7244985490283130860[0]._17013904265820854;
  result.expectedPos =_7244985490283130860[0]._9332970625915525982;
  result.map_matches =_7244985490283130860[0]._6744006314306065854;
  return {result};
}

void LoopDetector::LoopClosureInfo ::toStream(ostream &rtr) const 
{
    rtr.write((char *)&curRefFrame,sizeof(curRefFrame));
    rtr.write((char *)&matchingFrameIdx,sizeof(matchingFrameIdx));
    toStream__(expectedPos, rtr);
    toStream__(map_matches, rtr);
    toStream__kv( optimPoses, rtr);
}

void LoopDetector::LoopClosureInfo::fromStream(istream &rtr)
{
    rtr.read((char *)&curRefFrame, sizeof(curRefFrame));
    rtr.read((char *)&matchingFrameIdx,sizeof(matchingFrameIdx));
    fromStream__(expectedPos, rtr);
    fromStream__(map_matches, rtr);
    fromStream__kv(optimPoses, rtr);
}

uint64_t LoopDetector::LoopClosureInfo::getSignature()
{
    Hash hash_value;
    hash_value += curRefFrame;
    hash_value += matchingFrameIdx;
    hash_value += expectedPos;
    for (const auto &match : map_matches)
    {
        hash_value += match.distance;
        hash_value += match.imgIdx;
        hash_value += match.trainIdx;
        hash_value += match.queryIdx;
    }
    for (const auto &pose : optimPoses) 
    {
        hash_value += pose.first;
        hash_value += pose.second;
    }
    return hash_value;
}
}
