#ifndef UCOSLAM_SYSTEM_H
#define UCOSLAM_SYSTEM_H
#include "basictypes/se3.h"
#include "imageparams.h"
#include "map_types/frame.h"
#include "ucoslamtypes.h"
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>

namespace ucoslam {

class MapManager;
class MapInitializer;
class Map;
class FrameExtractor;
class System {

  friend class ucoslam_Debugger;

  public:
    System();
    ~System();
    void setParams(std::shared_ptr<Map> map, const ucoslam::Params &params, const std::string &vocabulary = "");
    void clear();
    static Params &getParams();
    cv::Mat process(cv::Mat &in_image, const ImageParams &ip, uint32_t frameseq_idx,
            const cv::Mat &depth = cv::Mat(), const cv::Mat &R_image = cv::Mat());
    void resetTracker();
    void setMode(MODES mode);
    uint32_t getLastProcessedFrame() const;
    void saveToFile(std::string filepath);
    void readFromFile(std::string filepath);
    std::string getSignatureStr() const;
    void globalOptimization();
    void waitForFinished() ;
    uint32_t getCurrentKeyFrameIndex(); 
    std::shared_ptr<Map> getMap();
    cv::Mat process(const Frame &frame);
    void updateParams(const Params &p);

  private:
    friend class DebugTest;
    // pair<cv::Mat, cv::Mat> _8992046403730870353( const cv::Mat &_16998117773731312345, ImageParams &_175247760147,
    //     const cv::Mat &_6806993204424110071);
    uint64_t getHashValue(bool flag = false) const;
    void updateFrameExtractorParam();
    bool reLocalization(Frame &frame, se3 &se3_transform);
    struct KeyPointRelocResult{
      se3 se3_transform;
      int reserve = 0;
      std::vector<uint32_t> frame_ids;
      std::vector<cv::DMatch> matches;
    };
    bool reLocalizationWithKeyPoints(Frame &frame, se3 &se3_transform, const std::set<uint32_t>& excluded_frames = {});
    std::vector<KeyPointRelocResult> getKeyPointRelocResult(Frame &frame, se3 &se3_transform, const std::set<uint32_t> &excluded_frames = {});
    bool reLocalizationWithMarkers(Frame &frame, se3& se3_transform);
    void drawImage(cv::Mat &image, float scale) const;
    string hashToStr(uint64_t) const;
    uint32_t retrieveKeyFrame(const Frame &frame, const se3& se3_transform);
    // cv::Mat _4145838251597814913(const Frame &frame);
    bool processFirstFrame(Frame &frame);
    bool processSteroFrame(Frame &frame);
    bool processNonSteroFrame(Frame &frame);
    se3 updatePose(Frame &frame, se3 se3_transform);
    std::vector<cv::DMatch> matchFrames(Frame &curr_frame, Frame &prev_frame, float dist_thresh, float proj_dist);
    void putText(cv::Mat &img, string text, cv::Point origin);
    static Params m_params;
    std::shared_ptr<Map> m_map;
    std::shared_ptr<FrameExtractor> m_frame_extractor;
    Frame m_current_frame, m_prev_frame;
    std::shared_ptr<MapInitializer> m_map_initializer;
    bool firstFrameFlag = false;
    se3 m_se3;
    int64_t m_keyframe_Index = -1;
    STATE m_state = STATE_LOST;
    MODES m_mode = MODE_SLAM;
    std::shared_ptr<MapManager> m_map_manager;
    cv::Mat m_transformation;
    uint64_t m_frame_count = 0;
    int64_t m_fseq_idx = -1;
};
}

#endif