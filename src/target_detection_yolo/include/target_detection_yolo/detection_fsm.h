#pragma once

#include <string>

namespace target_detection_yolo {

// 检测分辨率模式：
// REACQUIRE_HR：在原始图像上分块执行全局搜索，用于初始捕获或目标丢失。
// TRACK_HR：从原始图像裁剪原生分辨率 ROI，适合远距离小目标。
// TRACK_LR：将整幅原图缩小后检测，适合缩小后仍有足够像素的近距离目标。
enum class DetectionMode {
  REACQUIRE_HR,
  TRACK_HR,
  TRACK_LR,
};

// 三模式切换参数。所有尺寸阈值均定义在检测器低分辨率输入尺度下，
// 例如原图最短边为 81 px、缩放比例为 4.05 时，等效尺寸为 20 px。
// 进入和退出阈值刻意不同，以形成滞回并抑制模式抖动。
struct DetectionFsmConfig {
  double downscale_factor{4.05};
  double reacquire_min_confidence{0.55};
  int reacquire_consecutive_detections{2};
  double hr_lost_confidence{0.35};
  int hr_lost_consecutive_failures{3};
  double lr_enter_min_target_size_px{20.0};
  double lr_enter_min_confidence{0.55};
  int lr_enter_consecutive_detections{3};
  double lr_exit_target_size_px{14.0};
  double lr_exit_confidence{0.35};
  int lr_exit_consecutive_failures{2};

  bool isValid(std::string* reason = nullptr) const;
};

// 单次检测观测。检测框必须已经恢复到原始图像坐标，尺寸取宽高最短边。
// valid=false 表示本轮没有可信的目标观测。
struct DetectionObservation {
  bool valid{false};
  // No visual measurement this frame, but a time-valid EKF ROI prediction is
  // available. This is a coast frame, not a measurement.
  bool predicted{false};
  double target_min_size_full_px{0.0};
  double confidence{0.0};
};

// 每次状态机更新的结果，同时暴露连续计数，便于日志和调试可视化。
struct DetectionFsmUpdate {
  DetectionMode previous_mode{DetectionMode::REACQUIRE_HR};
  DetectionMode mode{DetectionMode::REACQUIRE_HR};
  bool changed{false};
  double target_min_size_low_px{0.0};
  int reacquire_streak{0};
  int enter_streak{0};
  int exit_streak{0};
};

// 基于目标尺度和置信度的三模式有限状态机。
//
// REACQUIRE_HR -> TRACK_HR：分块全局搜索连续获得可信目标。
// TRACK_HR -> TRACK_LR：目标足够大且置信度足够高，并连续满足指定次数。
// TRACK_HR -> REACQUIRE_HR：ROI 跟踪连续丢失目标。
// TRACK_LR -> TRACK_HR：目标过小、低置信度或漏检，并连续满足指定次数。
//
// 本类只负责模式决策，不执行图像缩放、ROI 裁剪或神经网络推理，因而可由
// 后续检测节点在每次完成目标关联后调用。
class DetectionFsm {
 public:
  explicit DetectionFsm(
      const DetectionFsmConfig& config = DetectionFsmConfig());

  bool configure(const DetectionFsmConfig& config,
                 std::string* reason = nullptr);

  // 清空连续计数，并将状态机置于指定模式。默认从高分辨率分块全局搜索
  // 启动，确保尚无目标位置时能够完成初始捕获。
  void reset(DetectionMode mode = DetectionMode::REACQUIRE_HR);

  // 输入一帧关联后的检测结果并执行一次状态转移。predicted 只允许
  // TRACK_HR 短时保持，不会被计入重捕获确认或真实检测计数。
  DetectionFsmUpdate update(const DetectionObservation& observation);

  DetectionMode mode() const { return mode_; }
  const DetectionFsmConfig& config() const { return config_; }

  static const char* toString(DetectionMode mode);

 private:
  DetectionFsmConfig config_;
  DetectionMode mode_{DetectionMode::REACQUIRE_HR};
  int reacquire_streak_{0};
  int enter_streak_{0};
  int exit_streak_{0};
};

}  // namespace target_detection_yolo
