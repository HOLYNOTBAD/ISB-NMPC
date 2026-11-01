#pragma once

#include "target_detection_yolo/detection_strategy.h"

namespace target_detection_yolo {

struct TrackHrConfig {
  int roi_width{640};
  int roi_height{480};
  double min_confidence{0.0};
  double prediction_association_radius_px{360.0};
  TargetAssociationConfig association;
};

class TrackHrStrategy final : public DetectionStrategy {
 public:
  TrackHrStrategy(Detector* detector,
                  const TrackHrConfig& config = TrackHrConfig());

  DetectionMode mode() const override { return DetectionMode::TRACK_HR; }
  void reset() override {}
  DetectionResult detect(const cv::Mat& image,
                         const TrackingContext& context) override;

 private:
  Detector* detector_;
  TrackHrConfig config_;
};

}  // namespace target_detection_yolo
