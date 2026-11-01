#pragma once

#include "target_detection_yolo/detection_strategy.h"

namespace target_detection_yolo {

struct TrackLrConfig {
  double min_confidence{0.0};
  TargetAssociationConfig association;
};

class TrackLrStrategy final : public DetectionStrategy {
 public:
  TrackLrStrategy(Detector* detector,
                  const TrackLrConfig& config = TrackLrConfig());

  DetectionMode mode() const override { return DetectionMode::TRACK_LR; }
  void reset() override {}
  DetectionResult detect(const cv::Mat& image,
                         const TrackingContext& context) override;

 private:
  Detector* detector_;
  TrackLrConfig config_;
};

}  // namespace target_detection_yolo
