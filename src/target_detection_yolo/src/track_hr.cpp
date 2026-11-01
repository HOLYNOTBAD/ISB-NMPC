#include "target_detection_yolo/track_hr.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace target_detection_yolo {

TrackHrStrategy::TrackHrStrategy(Detector* detector,
                                 const TrackHrConfig& config)
    : detector_(detector), config_(config) {
  if (detector_ == nullptr) {
    throw std::invalid_argument("TrackHrStrategy requires a detector");
  }
  if (config_.roi_width <= 0 || config_.roi_height <= 0 ||
      !std::isfinite(config_.min_confidence) ||
      config_.min_confidence < 0.0 || config_.min_confidence > 1.0 ||
      !std::isfinite(config_.prediction_association_radius_px) ||
      config_.prediction_association_radius_px <= 0.0 ||
      !config_.association.isValid()) {
    throw std::invalid_argument("TRACK_HR ROI dimensions must be positive");
  }
}

DetectionResult TrackHrStrategy::detect(const cv::Mat& image,
                                        const TrackingContext& context) {
  if (image.empty() || !context.has_previous_target) {
    return DetectionResult();
  }

  const int roi_width = std::min(config_.roi_width, image.cols);
  const int roi_height = std::min(config_.roi_height, image.rows);
  cv::Point2f center(
      context.previous_target.rect.x + context.previous_target.rect.width / 2.0F,
      context.previous_target.rect.y + context.previous_target.rect.height / 2.0F);
  DetectionBox association_reference = context.previous_target;
  const bool has_finite_prediction =
      context.has_predicted_target_center &&
      std::isfinite(context.predicted_target_center.x) &&
      std::isfinite(context.predicted_target_center.y);
  if (has_finite_prediction) {
    // The prediction publisher already suppresses output unless the S2
    // tracker is healthy. Trust that forward estimate here: requiring the
    // previous raw box to remain inside the predicted ROI rejects exactly the
    // large, high-speed displacement for which forward ROI placement exists.
    center = context.predicted_target_center;
    association_reference.rect.x = center.x - association_reference.rect.width / 2.0F;
    association_reference.rect.y = center.y - association_reference.rect.height / 2.0F;
  }
  const int max_x = image.cols - roi_width;
  const int max_y = image.rows - roi_height;
  const int roi_x = std::max(
      0, std::min(max_x, static_cast<int>(std::lround(center.x - roi_width / 2.0F))));
  const int roi_y = std::max(
      0, std::min(max_y, static_cast<int>(std::lround(center.y - roi_height / 2.0F))));
  const cv::Rect roi(roi_x, roi_y, roi_width, roi_height);

  DetectionResult result = detail::normalizeDetections(
      detector_->detect(image(roi)), roi.size(), roi.tl(), roi,
      config_.min_confidence);
  if (has_finite_prediction) {
    // EKF 预测误差以像素计，不应继续使用“小目标自身尺寸”的归一化门限；
    // 否则目标虽在 ROI 内，仍会因快速像面运动被关联层拒绝。
    size_t best_index = result.candidates.size();
    double best_score = -std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < result.candidates.size(); ++index) {
      const DetectionBox& candidate = result.candidates[index];
      if (context.previous_target.class_id >= 0 && candidate.class_id >= 0 &&
          context.previous_target.class_id != candidate.class_id) {
        continue;
      }
      const cv::Point2f candidate_center(
          candidate.rect.x + candidate.rect.width / 2.0F,
          candidate.rect.y + candidate.rect.height / 2.0F);
      const double distance = cv::norm(candidate_center - center);
      if (distance > config_.prediction_association_radius_px) {
        continue;
      }
      const double score = candidate.confidence -
                           0.25 * distance /
                               config_.prediction_association_radius_px;
      if (best_index == result.candidates.size() || score > best_score) {
        best_index = index;
        best_score = score;
      }
    }
    if (best_index == result.candidates.size()) {
      result.valid = false;
    } else {
      detail::promoteCandidate(&result, best_index);
    }
  } else {
    detail::associateTarget(&result, association_reference,
                            config_.association);
  }
  return result;
}

}  // namespace target_detection_yolo
