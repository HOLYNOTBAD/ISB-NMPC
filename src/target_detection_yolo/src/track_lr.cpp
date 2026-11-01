#include "target_detection_yolo/track_lr.h"

#include <cmath>
#include <stdexcept>

namespace target_detection_yolo {

TrackLrStrategy::TrackLrStrategy(Detector* detector,
                                 const TrackLrConfig& config)
    : detector_(detector), config_(config) {
  if (detector_ == nullptr) {
    throw std::invalid_argument("TrackLrStrategy requires a detector");
  }
  if (!std::isfinite(config_.min_confidence) ||
      config_.min_confidence < 0.0 || config_.min_confidence > 1.0 ||
      !config_.association.isValid()) {
    throw std::invalid_argument("invalid TRACK_LR configuration");
  }
}

DetectionResult TrackLrStrategy::detect(const cv::Mat& image,
                                        const TrackingContext& context) {
  if (image.empty()) {
    return DetectionResult();
  }
  const cv::Rect full_image(0, 0, image.cols, image.rows);
  DetectionResult result = detail::normalizeDetections(
      detector_->detect(image), image.size(), cv::Point(0, 0), full_image,
      config_.min_confidence);
  if (context.has_previous_target) {
    detail::associateTarget(&result, context.previous_target,
                            config_.association);
  }
  return result;
}

}  // namespace target_detection_yolo
