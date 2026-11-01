#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "target_detection_yolo/detection_fsm.h"

namespace target_detection_yolo {

// 检测框坐标始终使用当前策略输入的像素坐标。策略返回前必须将其恢复到
// 原始相机图像坐标。
struct DetectionBox {
  cv::Rect2f rect;
  float confidence{0.0F};
  int class_id{-1};
};

// 三个策略共享的历史信息。previous_target 使用原始相机图像坐标。
struct TrackingContext {
  bool has_previous_target{false};
  DetectionBox previous_target;
  // TRACK_HR 可选前馈中心，由 S2/IMU 滤波预测提供。坐标位于原图中。
  bool has_predicted_target_center{false};
  cv::Point2f predicted_target_center;
};

struct DetectionResult {
  bool valid{false};
  DetectionBox target;
  std::vector<DetectionBox> candidates;

  // 本轮真正送入检测器的原图区域，供调试图像和性能统计使用。
  cv::Rect processed_region;
  // 仅 REACQUIRE_HR 使用：本轮结束后是否刚好完成一次全图扫描。
  bool sweep_completed{false};
};

// 跨帧目标关联门限。候选框只要满足 IoU 或归一化中心距离中的任一门限，
// 即可参与打分；已知类别不一致的候选框会被拒绝。
struct TargetAssociationConfig {
  double min_iou{0.05};
  double max_center_distance_factor{3.0};

  bool isValid() const {
    return std::isfinite(min_iou) && min_iou >= 0.0 && min_iou <= 1.0 &&
           std::isfinite(max_center_distance_factor) &&
           max_center_distance_factor > 0.0;
  }
};

// YOLO/TensorRT 的最小共享接口。实现类只加载一份 engine；返回框的坐标
// 必须对应传入 image，而不是网络 letterbox 输入坐标。
class Detector {
 public:
  virtual ~Detector() = default;
  virtual std::vector<DetectionBox> detect(const cv::Mat& image) = 0;
};

// 三种检测业务的统一接口。类内部可以保存扫描游标等模式私有状态。
class DetectionStrategy {
 public:
  virtual ~DetectionStrategy() = default;

  virtual DetectionMode mode() const = 0;
  virtual void reset() = 0;
  virtual DetectionResult detect(const cv::Mat& image,
                                 const TrackingContext& context) = 0;
};

namespace detail {

inline double intersectionOverUnion(const cv::Rect2f& left,
                                    const cv::Rect2f& right) {
  const cv::Rect2f intersection = left & right;
  const double intersection_area =
      std::max(0.0F, intersection.width) *
      std::max(0.0F, intersection.height);
  const double union_area = left.area() + right.area() - intersection_area;
  return union_area > 0.0 ? intersection_area / union_area : 0.0;
}

inline double normalizedCenterDistance(const cv::Rect2f& reference,
                                       const cv::Rect2f& candidate) {
  const cv::Point2f reference_center(
      reference.x + reference.width / 2.0F,
      reference.y + reference.height / 2.0F);
  const cv::Point2f candidate_center(candidate.x + candidate.width / 2.0F,
                                    candidate.y + candidate.height / 2.0F);
  const double dx =
      static_cast<double>(reference_center.x - candidate_center.x);
  const double dy =
      static_cast<double>(reference_center.y - candidate_center.y);
  // 只使用历史目标的尺度，防止异常巨大的候选框人为缩小归一化距离。
  const double reference_scale = std::max(
      1.0, std::hypot(static_cast<double>(reference.width),
                      static_cast<double>(reference.height)));
  return std::hypot(dx, dy) / reference_scale;
}

inline void promoteCandidate(DetectionResult* result, size_t index) {
  if (result == nullptr || index >= result->candidates.size()) {
    return;
  }
  if (index != 0U) {
    std::swap(result->candidates.front(), result->candidates[index]);
  }
  result->valid = true;
  result->target = result->candidates.front();
}

// 从候选框中选择与 reference 最一致的目标。返回 false 表示本帧只有无关
// 检测，此时调用方应将其作为一次漏检，而不是跳到另一个目标。
inline bool associateTarget(DetectionResult* result,
                            const DetectionBox& reference,
                            const TargetAssociationConfig& config,
                            double min_confidence = 0.0) {
  if (result == nullptr || !config.isValid()) {
    return false;
  }

  size_t best_index = result->candidates.size();
  double best_score = -std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < result->candidates.size(); ++index) {
    const DetectionBox& candidate = result->candidates[index];
    if (candidate.confidence < min_confidence) {
      continue;
    }
    if (reference.class_id >= 0 && candidate.class_id >= 0 &&
        reference.class_id != candidate.class_id) {
      continue;
    }
    const double iou = intersectionOverUnion(reference.rect, candidate.rect);
    const double distance =
        normalizedCenterDistance(reference.rect, candidate.rect);
    if (iou < config.min_iou &&
        distance > config.max_center_distance_factor) {
      continue;
    }

    // IoU 主导重叠目标，中心距离用于无重叠但运动连续的目标，置信度只作为
    // 次要判据，避免远处高置信度误检抢走当前目标。
    const double score = 2.0 * iou - 0.15 * distance +
                         0.25 * candidate.confidence;
    if (best_index == result->candidates.size() || score > best_score) {
      best_index = index;
      best_score = score;
    }
  }

  if (best_index == result->candidates.size()) {
    result->valid = false;
    return false;
  }
  promoteCandidate(result, best_index);
  return true;
}

// 将一个局部检测区域中的框裁剪并平移回原图坐标，同时选择最高置信度框。
inline DetectionResult normalizeDetections(
    const std::vector<DetectionBox>& local_detections,
    const cv::Size& local_image_size, const cv::Point& global_offset,
    const cv::Rect& processed_region, double min_confidence) {
  DetectionResult result;
  result.processed_region = processed_region;
  if (local_image_size.width <= 0 || local_image_size.height <= 0) {
    return result;
  }

  const cv::Rect2f local_bounds(0.0F, 0.0F,
                               static_cast<float>(local_image_size.width),
                               static_cast<float>(local_image_size.height));
  for (const DetectionBox& detection : local_detections) {
    if (!std::isfinite(detection.confidence) ||
        detection.confidence < min_confidence ||
        !std::isfinite(detection.rect.x) ||
        !std::isfinite(detection.rect.y) ||
        !std::isfinite(detection.rect.width) ||
        !std::isfinite(detection.rect.height) || detection.rect.width <= 0.0F ||
        detection.rect.height <= 0.0F) {
      continue;
    }

    cv::Rect2f clipped = detection.rect & local_bounds;
    if (clipped.width <= 0.0F || clipped.height <= 0.0F) {
      continue;
    }
    clipped.x += static_cast<float>(global_offset.x);
    clipped.y += static_cast<float>(global_offset.y);

    DetectionBox normalized = detection;
    normalized.rect = clipped;
    result.candidates.push_back(normalized);
  }

  if (!result.candidates.empty()) {
    const auto best = std::max_element(
        result.candidates.begin(), result.candidates.end(),
        [](const DetectionBox& left, const DetectionBox& right) {
          return left.confidence < right.confidence;
        });
    promoteCandidate(&result,
                     static_cast<size_t>(best - result.candidates.begin()));
  }
  return result;
}

}  // namespace detail

}  // namespace target_detection_yolo
