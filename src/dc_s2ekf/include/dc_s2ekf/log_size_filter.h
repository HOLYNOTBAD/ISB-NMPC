#pragma once

#include "dc_s2ekf/filter_common.h"

namespace dc_s2ekf {

struct LogSizeFilterConfig {
  double min_box_size_px = 2.0;
  double measurement_std_px = 5.0;
  double process_noise = 3.0;
};

// LogSizeFilter tracks log(width) and log(height) with a constant-velocity model.
//（LogSizeFilter使用匀速模型跟踪log(宽度)与log(高度)。）
//
// Separating size from bearing keeps both covariance paths independently testable.
//（将尺寸与视线估计分离，使两条协方差计算路径均可独立测试。）
class LogSizeFilter {
 public:
  void configure(const LogSizeFilterConfig& config) { config_ = config; }

  // Initialize log-size state from bbox width and height.
  //（根据bbox宽高初始化对数尺寸状态。）
  void initialize(const Measurement& measurement);
  // Predict and update size with quality-scaled measurement uncertainty.
  //（预测尺寸，并以测量质量缩放后的不确定度进行更新。）
  void update(const Measurement& measurement, double dt,
              double measurement_quality);
  SizeState predictSize(double dt) const;

 private:
  static void propagateState(double dt, SizeState* state);
  void predict(double dt);
  void updateScalar(int measurement_index, double measurement,
                    double measurement_variance);

  LogSizeFilterConfig config_;
  SizeState state_{};
  SizeCovariance covariance_{};
};

}  // namespace dc_s2ekf
