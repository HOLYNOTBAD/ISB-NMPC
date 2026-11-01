#include "target_detection_yolo/detection_fsm.h"

#include <algorithm>
#include <cmath>

namespace target_detection_yolo {

bool DetectionFsmConfig::isValid(std::string* reason) const {
  // 参数在进入运行期前统一校验，避免非法阈值导致状态机永久锁死或抖动。
  const auto reject = [reason](const std::string& message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return false;
  };

  if (!std::isfinite(downscale_factor) || downscale_factor <= 0.0) {
    return reject("downscale_factor must be finite and positive");
  }
  if (!std::isfinite(reacquire_min_confidence) ||
      !std::isfinite(hr_lost_confidence) ||
      reacquire_min_confidence < 0.0 || reacquire_min_confidence > 1.0 ||
      hr_lost_confidence < 0.0 || hr_lost_confidence > 1.0 ||
      reacquire_min_confidence <= hr_lost_confidence) {
    return reject(
        "reacquire confidence must exceed HR lost confidence and both must "
        "be in [0, 1]");
  }
  if (!std::isfinite(lr_enter_min_target_size_px) ||
      !std::isfinite(lr_exit_target_size_px) ||
      lr_enter_min_target_size_px <= lr_exit_target_size_px ||
      lr_exit_target_size_px < 0.0) {
    return reject("scale thresholds must be finite and enter must exceed exit");
  }
  if (!std::isfinite(lr_enter_min_confidence) ||
      !std::isfinite(lr_exit_confidence) ||
      lr_enter_min_confidence < 0.0 || lr_enter_min_confidence > 1.0 ||
      lr_exit_confidence < 0.0 || lr_exit_confidence > 1.0 ||
      lr_enter_min_confidence <= lr_exit_confidence) {
    return reject(
        "confidence thresholds must be in [0, 1] and enter must exceed exit");
  }
  if (reacquire_consecutive_detections <= 0 ||
      hr_lost_consecutive_failures <= 0 ||
      lr_enter_consecutive_detections <= 0 ||
      lr_exit_consecutive_failures <= 0) {
    return reject("consecutive-frame thresholds must be positive");
  }
  return true;
}

DetectionFsm::DetectionFsm(const DetectionFsmConfig& config) {
  configure(config);
}

bool DetectionFsm::configure(const DetectionFsmConfig& config,
                             std::string* reason) {
  if (!config.isValid(reason)) {
    return false;
  }
  config_ = config;
  reacquire_streak_ = 0;
  enter_streak_ = 0;
  exit_streak_ = 0;
  return true;
}

void DetectionFsm::reset(DetectionMode mode) {
  mode_ = mode;
  reacquire_streak_ = 0;
  enter_streak_ = 0;
  exit_streak_ = 0;
}

DetectionFsmUpdate DetectionFsm::update(
    const DetectionObservation& observation) {
  DetectionFsmUpdate result;
  result.previous_mode = mode_;

  // 非有限数、负尺寸和越界置信度均按漏检处理，防止 NaN 污染连续计数。
  const bool finite_observation =
      observation.valid && std::isfinite(observation.target_min_size_full_px) &&
      std::isfinite(observation.confidence) &&
      observation.target_min_size_full_px >= 0.0 &&
      observation.confidence >= 0.0 && observation.confidence <= 1.0;
  // 外部统一传入原图坐标；状态机内部只在此处完成一次尺度换算。
  const double low_size =
      finite_observation
          ? observation.target_min_size_full_px / config_.downscale_factor
          : 0.0;

  if (mode_ == DetectionMode::REACQUIRE_HR) {
    // 分块全局搜索必须连续获得高置信度目标，才建立局部跟踪 ROI。
    const bool target_reacquired =
        finite_observation &&
        observation.confidence >= config_.reacquire_min_confidence;
    reacquire_streak_ = target_reacquired ? reacquire_streak_ + 1 : 0;
    enter_streak_ = 0;
    exit_streak_ = 0;
    if (reacquire_streak_ >= config_.reacquire_consecutive_detections) {
      mode_ = DetectionMode::TRACK_HR;
      reacquire_streak_ = 0;
    }
  } else if (mode_ == DetectionMode::TRACK_HR) {
    // 只有“大目标 + 高置信度”连续出现，才允许切换到全图低分辨率检测。
    const bool ready_for_lr =
        finite_observation &&
        low_size >= config_.lr_enter_min_target_size_px &&
        observation.confidence >= config_.lr_enter_min_confidence;
    enter_streak_ = ready_for_lr ? enter_streak_ + 1 : 0;
    const bool target_lost =
        !observation.predicted &&
        (!finite_observation ||
         observation.confidence < config_.hr_lost_confidence);
    exit_streak_ = target_lost ? exit_streak_ + 1 : 0;
    reacquire_streak_ = 0;
    if (exit_streak_ >= config_.hr_lost_consecutive_failures) {
      mode_ = DetectionMode::REACQUIRE_HR;
      enter_streak_ = 0;
      exit_streak_ = 0;
    } else if (enter_streak_ >= config_.lr_enter_consecutive_detections) {
      mode_ = DetectionMode::TRACK_LR;
      enter_streak_ = 0;
    }
  } else {
    // 尺度过小、置信度过低和漏检均视为退出 LR 的失败事件。
    const bool should_leave_lr =
        !finite_observation || low_size < config_.lr_exit_target_size_px ||
        observation.confidence < config_.lr_exit_confidence;
    exit_streak_ = should_leave_lr ? exit_streak_ + 1 : 0;
    reacquire_streak_ = 0;
    enter_streak_ = 0;
    if (exit_streak_ >= config_.lr_exit_consecutive_failures) {
      mode_ = DetectionMode::TRACK_HR;
      exit_streak_ = 0;
    }
  }

  result.mode = mode_;
  result.changed = result.mode != result.previous_mode;
  result.target_min_size_low_px = low_size;
  result.reacquire_streak = reacquire_streak_;
  result.enter_streak = enter_streak_;
  result.exit_streak = exit_streak_;
  return result;
}

const char* DetectionFsm::toString(DetectionMode mode) {
  switch (mode) {
    case DetectionMode::REACQUIRE_HR:
      return "REACQUIRE_HR";
    case DetectionMode::TRACK_HR:
      return "TRACK_HR";
    case DetectionMode::TRACK_LR:
      return "TRACK_LR";
  }
  return "UNKNOWN";
}

}  // namespace target_detection_yolo
