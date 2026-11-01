#include "target_detection_yolo/reacquire_hr.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace target_detection_yolo {
namespace {

std::vector<int> axisStarts(int image_length, int tile_length,
                            double overlap_ratio) {
  if (tile_length >= image_length) {
    return {0};
  }
  const int step = std::max(
      1, static_cast<int>(std::lround(tile_length * (1.0 - overlap_ratio))));
  std::vector<int> starts;
  for (int position = 0; position + tile_length < image_length;
       position += step) {
    starts.push_back(position);
  }
  const int final_position = image_length - tile_length;
  if (starts.empty() || starts.back() != final_position) {
    starts.push_back(final_position);
  }
  return starts;
}

}  // namespace

ReacquireHrStrategy::ReacquireHrStrategy(Detector* detector,
                                         const ReacquireHrConfig& config)
    : detector_(detector), config_(config) {
  if (detector_ == nullptr) {
    throw std::invalid_argument("ReacquireHrStrategy requires a detector");
  }
  if (config_.tile_width <= 0 || config_.tile_height <= 0 ||
      !std::isfinite(config_.overlap_ratio) || config_.overlap_ratio < 0.0 ||
      config_.overlap_ratio >= 1.0 || !std::isfinite(config_.min_confidence) ||
      config_.min_confidence < 0.0 || config_.min_confidence > 1.0 ||
      !std::isfinite(config_.confirmation_min_confidence) ||
      config_.confirmation_min_confidence < 0.0 ||
      config_.confirmation_min_confidence > 1.0 ||
      !config_.association.isValid()) {
    throw std::invalid_argument("invalid REACQUIRE_HR tile configuration");
  }
}

void ReacquireHrStrategy::reset() {
  tile_index_ = 0;
  has_confirmation_target_ = false;
}

void ReacquireHrStrategy::rebuildTiles(const cv::Size& image_size) {
  image_size_ = image_size;
  tiles_.clear();
  tile_index_ = 0;
  has_confirmation_target_ = false;

  const int tile_width = std::min(config_.tile_width, image_size.width);
  const int tile_height = std::min(config_.tile_height, image_size.height);
  const std::vector<int> x_starts =
      axisStarts(image_size.width, tile_width, config_.overlap_ratio);
  const std::vector<int> y_starts =
      axisStarts(image_size.height, tile_height, config_.overlap_ratio);
  for (int y : y_starts) {
    for (int x : x_starts) {
      tiles_.emplace_back(x, y, tile_width, tile_height);
    }
  }

  // Search the most likely region first. A row-major sweep starts in the
  // top-left corner and can spend most of a sweep away from a target that was
  // just lost near the optical axis. Keep the same complete tile set, but visit
  // it from the image center outwards.
  const cv::Point2d image_center(
      0.5 * static_cast<double>(image_size.width),
      0.5 * static_cast<double>(image_size.height));
  std::stable_sort(
      tiles_.begin(), tiles_.end(),
      [&image_center](const cv::Rect& lhs, const cv::Rect& rhs) {
        const auto distance_squared = [&image_center](const cv::Rect& tile) {
          const double dx =
              static_cast<double>(tile.x) + 0.5 * tile.width - image_center.x;
          const double dy =
              static_cast<double>(tile.y) + 0.5 * tile.height - image_center.y;
          return dx * dx + dy * dy;
        };
        return distance_squared(lhs) < distance_squared(rhs);
      });
}

DetectionResult ReacquireHrStrategy::detect(const cv::Mat& image,
                                            const TrackingContext&) {
  if (image.empty()) {
    return DetectionResult();
  }
  if (tiles_.empty() || image.size() != image_size_) {
    rebuildTiles(image.size());
  }

  const cv::Rect tile = tiles_[tile_index_];
  DetectionResult result = detail::normalizeDetections(
      detector_->detect(image(tile)), tile.size(), tile.tl(), tile,
      config_.min_confidence);

  const bool strong_detection =
      result.valid &&
      result.target.confidence >= config_.confirmation_min_confidence;
  if (strong_detection) {
    const DetectionBox strongest_candidate = result.target;
    if (!has_confirmation_target_) {
      confirmation_target_ = strongest_candidate;
      has_confirmation_target_ = true;
    } else if (detail::associateTarget(
                   &result, confirmation_target_, config_.association,
                   config_.confirmation_min_confidence)) {
      confirmation_target_ = result.target;
    } else {
      // 不一致的强检测作为新的确认起点，但本帧必须向 FSM 报告漏检，打断
      // 原有连续计数，防止两个不同目标共同完成重捕获确认。
      confirmation_target_ = strongest_candidate;
      result.valid = false;
    }
  } else {
    has_confirmation_target_ = false;
    ++tile_index_;
    if (tile_index_ >= tiles_.size()) {
      tile_index_ = 0;
      result.sweep_completed = true;
    }
  }
  return result;
}

}  // namespace target_detection_yolo
