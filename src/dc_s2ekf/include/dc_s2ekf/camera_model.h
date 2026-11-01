#pragma once

#include <array>

#include <swarm_msgs/BoundingBox.h>

#include "dc_s2ekf/filter_common.h"

namespace dc_s2ekf {

enum class CameraDistortionModel {
  NONE,
  EQUIDISTANT,
};

struct CameraModelConfig {
  int image_width = 640;
  int image_height = 480;
  double fx = 115.0;
  double fy = 115.0;
  double cx = 320.0;
  double cy = 240.0;
  CameraDistortionModel distortion_model = CameraDistortionModel::NONE;
  std::array<double, 4> distortion_coeffs{{0.0, 0.0, 0.0, 0.0}};
  double min_box_size_px = 2.0;
  double min_project_z = 0.05;
};

// CameraModel converts between pixels, bearings, and bbox wire messages.
//（CameraModel负责像素、视线向量与bbox消息格式之间的转换。）
//
// It owns no ROS subscriptions, keeping camera geometry independently testable.
//（该类不持有ROS订阅，使相机几何逻辑可独立测试。）
class CameraModel {
 public:
  void configure(const CameraModelConfig& config) { config_ = config; }

  // Convert an integer wire-format bbox to [center_u, center_v, width, height].
  //（将整数bbox消息转换为[中心u、中心v、宽、高]。）
  Measurement boxToMeasurement(const swarm_msgs::BoundingBox& box) const;
  void measurementToBox(const Measurement& measurement,
                        swarm_msgs::BoundingBox* box) const;

  // Convert a pixel to a normalized optical-frame bearing.
  //（将像素坐标转换为相机光学系单位视线向量。）
  math::Vec3 bearingFromPixel(double u, double v) const;
  // Project a bearing; return false when it is behind the camera plane.
  //（投影视线向量；当其位于相机平面后方时返回false。）
  bool pixelFromBearing(const math::Vec3& bearing_camera,
                        double* u, double* v) const;
  void bearingJacobiansFromPixel(double u, double v,
                                 math::Vec3* derivative_u,
                                 math::Vec3* derivative_v) const;

 private:
  CameraModelConfig config_;
};

}  // namespace dc_s2ekf
