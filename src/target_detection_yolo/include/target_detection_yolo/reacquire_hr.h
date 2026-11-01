#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "target_detection_yolo/detection_strategy.h"

namespace target_detection_yolo {

struct ReacquireHrConfig {
  int tile_width{640};
  int tile_height{480};
  double overlap_ratio{0.20};
  double min_confidence{0.0};
  double confirmation_min_confidence{0.55};
  TargetAssociationConfig association;
};

// 每帧从画面中心向外扫描一个原生分辨率图块；可信目标出现后保持当前
// 图块，供下一帧重复确认。离开并重新进入本模式时，调度管线会调用
// reset()，使下一轮搜索重新从中心开始。
class ReacquireHrStrategy final : public DetectionStrategy {
 public:
  ReacquireHrStrategy(Detector* detector,
                      const ReacquireHrConfig& config = ReacquireHrConfig());

  DetectionMode mode() const override { return DetectionMode::REACQUIRE_HR; }
  void reset() override;
  DetectionResult detect(const cv::Mat& image,
                         const TrackingContext& context) override;

  size_t tileIndex() const { return tile_index_; }
  size_t tileCount() const { return tiles_.size(); }

 private:
  void rebuildTiles(const cv::Size& image_size);

  Detector* detector_;
  ReacquireHrConfig config_;
  cv::Size image_size_;
  std::vector<cv::Rect> tiles_;
  size_t tile_index_{0};
  bool has_confirmation_target_{false};
  DetectionBox confirmation_target_;
};

}  // namespace target_detection_yolo
