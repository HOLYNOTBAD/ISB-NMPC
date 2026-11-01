#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace yolo {

struct DetectionBox {
  cv::Rect2f rect;
  float confidence{0.0F};
  int class_id{-1};
};

// 可复用的单实例 TensorRT 检测器。输入可以是全图、ROI 或图块，输出坐标
// 始终对应传入图像。
class YoloDetector {
 public:
  explicit YoloDetector(const std::string& engine_file,
                        float confidence_threshold = 0.3F,
                        float nms_threshold = 0.3F);
  ~YoloDetector();

  YoloDetector(const YoloDetector&) = delete;
  YoloDetector& operator=(const YoloDetector&) = delete;

  std::vector<DetectionBox> detect(const cv::Mat& image);
  const char* platformName() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace yolo
