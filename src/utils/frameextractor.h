#ifndef ucoslam_FrameExtractor_H
#define ucoslam_FrameExtractor_H
#include "featureextractors/feature2dserializable.h"
#include "imageparams.h"
#include "map_types/frame.h"
#include <memory>
#include <memory>
#include <opencv2/core/core.hpp>
#include <vector>

namespace aruco 
{
  class MarkerDetector;
};

namespace ucoslam 
{

class FrameExtractor {

public:
  FrameExtractor();
  void setParams(std::shared_ptr<Feature2DSerializable>
          feature_detector,const ucoslam::Params &params);
  void process(const cv::Mat &image, const ImageParams &ip, Frame &frame,
          uint32_t frameseq_idx = std::numeric_limits<uint32_t>::max());
  void process_rgbd(const cv::Mat &image, const cv::Mat &depthImage,
      const ImageParams &ip, Frame &frame, uint32_t frameseq_idx =std::numeric_limits<uint32_t>::max());
  void processStereo(const cv::Mat &LeftRect, const cv::Mat &RightRect, const ImageParams &ip,
      Frame &frame, uint32_t frameseq_idx =std::numeric_limits<uint32_t>::max());

  bool &removeFromMarkers()
  {
    return _12350051723532614025;
  }
  bool& detectMarkers() 
  {
    return m_detect_marker;
  }
  bool& detectKeyPoints()
  {
    return m_detect_keypoints;
  }
  bool& saveImage()
  {
    return m_save_image_flag;
  }
  void toStream(std::ostream &str) const;
  void fromStream(std::istream &str);
  void setSensitivity( float v);
  float getSensitivity();

private:
  struct ImageInfo {
    cv::Mat origin_gray_image, resized_gray_image;
    ImageParams origin_ip, resized_ip;
    pair<float, float> scale_orign_resized;  // scale between origin and resized(resized_image/origin_image) 
                                             // first :height scale  second: width scale
  };

  std::shared_ptr<Feature2DSerializable> m_feature_detector;
  void prepare(float scale_factor, const cv::Mat &first_image, const ImageParams &ip, const cv::Mat &second_image = cv::Mat());
  void extract(const ImageInfo &image_info, Frame &frame, uint32_t frameseq_idx = std::numeric_limits<uint32_t>::max());
  vector<ImageInfo> m_image_stat_info;
  uint32_t _12273370977065616393 = 0;
  bool _12350051723532614025 = true;
  bool m_detect_marker = true;
  bool m_detect_keypoints = true;
  bool m_save_image_flag = false;
  Feature2DSerializable::FeatParams m_feat_params;
  std::shared_ptr<aruco::MarkerDetector> m_marker_detector;
  float m_marker_size = 0;
  float m_max_desc_distance = 0;
//   ucoslam::Params _13116459431724108758;
  ucoslam::Params m_ucoslam_params;
};
}

#endif