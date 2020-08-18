#ifndef _UCOSLAM_LOOPDETECTOR_H
#define _UCOSLAM_LOOPDETECTOR_H
#include "map.h"
#include <thread>
namespace ucoslam {

class LoopDetector {
  std::shared_ptr<Map> TheMap;

public:
  struct LoopClosureInfo {
    void clear()
    {
      optimPoses.clear();
    }
    bool foundLoop() const 
    {
      return optimPoses.size() != 0;
    }
    uint32_t curRefFrame = std::numeric_limits<uint32_t>::max();
    uint32_t matchingFrameIdx =std::numeric_limits<uint32_t>::max();
    cv::Mat expectedPos;
    std::vector<cv::DMatch> map_matches;
    std::map<uint32_t, cv::Mat> optimPoses;
    void toStream(ostream &rtr)const;
    void fromStream(istream &rtr);
    uint64_t getSignature();
  };

  void setParams(std::shared_ptr<Map> map);
  LoopClosureInfo detectLoopFromMarkers(Frame &frame, int32_t curRefKf);
  LoopClosureInfo detectLoopFromKeyPoints(Frame &frame, int32_t curRefKf);
  void correctMap(const LoopClosureInfo &lcsol);
private:
  vector<LoopClosureInfo> getLoopClosureInfo(Frame &frame, int64_t curRefKf);
  std::vector<LoopClosureInfo> _8671179296205241382(Frame &frame, int32_t curRefKf);
  void _6767859416531794787(Frame &frame, LoopClosureInfo &loop_closure_info);
  double calcGlobalReprojChi2(const LoopClosureInfo &lcsol);
  // void _16786277844087146768(Frame &frame,const LoopClosureInfo &_46082543426142246);
  // vector<cv::DMatch> _15519652129875366702(std::shared_ptr<Map> _11093822290287,
  //                       const Frame &, uint32_t _175247760268,
  //                       float _1686565542397313973, void *_6806993289704731004);
};
}
#endif
