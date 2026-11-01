#include "dc_s2ekf/camera_model.h"

#include <algorithm>
#include <cmath>

#include "dc_s2ekf/filter_common.h"

namespace dc_s2ekf {
namespace {

constexpr double kHalfPi = 1.57079632679489661923;

double equidistantRadius(double theta,
                         const std::array<double, 4>& coefficients) {
  const double theta2 = theta * theta;
  const double theta4 = theta2 * theta2;
  const double theta6 = theta4 * theta2;
  const double theta8 = theta4 * theta4;
  return theta * (1.0 + coefficients[0] * theta2 +
                  coefficients[1] * theta4 + coefficients[2] * theta6 +
                  coefficients[3] * theta8);
}

double invertEquidistantRadius(
    double distorted_radius, const std::array<double, 4>& coefficients) {
  if (distorted_radius <= math::kEps) {
    return 0.0;
  }

  // Camera rays handled by this model must remain in the forward hemisphere.
  // Newton iterations are clamped to that physical interval. Kalibr's
  // equidistant models are monotonic over the calibrated field of view.
  double theta = math::clamp(distorted_radius, 0.0, kHalfPi - 1e-8);
  for (int iteration = 0; iteration < 12; ++iteration) {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    const double residual =
        equidistantRadius(theta, coefficients) - distorted_radius;
    const double derivative =
        1.0 + 3.0 * coefficients[0] * theta2 +
        5.0 * coefficients[1] * theta4 +
        7.0 * coefficients[2] * theta6 +
        9.0 * coefficients[3] * theta8;
    if (!std::isfinite(derivative) || std::abs(derivative) < 1e-10) {
      break;
    }
    const double next = math::clamp(theta - residual / derivative, 0.0,
                                    kHalfPi - 1e-8);
    if (std::abs(next - theta) < 1e-12) {
      theta = next;
      break;
    }
    theta = next;
  }
  return theta;
}

}  // namespace

Measurement CameraModel::boxToMeasurement(
    const swarm_msgs::BoundingBox& box) const {
  const double xmin = static_cast<double>(box.xmin);
  const double ymin = static_cast<double>(box.ymin);
  const double xmax = static_cast<double>(box.xmax);
  const double ymax = static_cast<double>(box.ymax);
  return {0.5 * (xmin + xmax),
          0.5 * (ymin + ymax),
          std::max(config_.min_box_size_px, xmax - xmin),
          std::max(config_.min_box_size_px, ymax - ymin)};
}

void CameraModel::measurementToBox(const Measurement& measurement,
                                   swarm_msgs::BoundingBox* box) const {
  if (box == nullptr) {
    return;
  }
  const double width = std::max(config_.min_box_size_px, measurement[2]);
  const double height = std::max(config_.min_box_size_px, measurement[3]);
  const double xmin = math::clamp(
      measurement[0] - 0.5 * width, 0.0,
      static_cast<double>(config_.image_width - 1));
  const double xmax = math::clamp(
      measurement[0] + 0.5 * width, 0.0,
      static_cast<double>(config_.image_width - 1));
  const double ymin = math::clamp(
      measurement[1] - 0.5 * height, 0.0,
      static_cast<double>(config_.image_height - 1));
  const double ymax = math::clamp(
      measurement[1] + 0.5 * height, 0.0,
      static_cast<double>(config_.image_height - 1));
  box->xmin = static_cast<int64_t>(std::llround(std::min(xmin, xmax)));
  box->xmax = static_cast<int64_t>(std::llround(std::max(xmin, xmax)));
  box->ymin = static_cast<int64_t>(std::llround(std::min(ymin, ymax)));
  box->ymax = static_cast<int64_t>(std::llround(std::max(ymin, ymax)));
  // Preserve predicted dimensions when clipping the visible rectangle because
  // the controller consumes a/b as its scale state.
  //（裁剪可见矩形时保留预测尺寸，因为控制器将a/b作为尺度状态。）
  box->a = width;
  box->b = height;
}

math::Vec3 CameraModel::bearingFromPixel(double u, double v) const {
  double x = (u - config_.cx) / std::max(config_.fx, 1e-6);
  double y = (v - config_.cy) / std::max(config_.fy, 1e-6);
  if (config_.distortion_model == CameraDistortionModel::EQUIDISTANT) {
    const double distorted_radius = std::hypot(x, y);
    if (distorted_radius > math::kEps) {
      const double theta = invertEquidistantRadius(
          distorted_radius, config_.distortion_coeffs);
      const double scale = std::tan(theta) / distorted_radius;
      x *= scale;
      y *= scale;
    }
  }
  return math::normalize(math::makeVec(x, y, 1.0));
}

bool CameraModel::pixelFromBearing(const math::Vec3& bearing_camera,
                                   double* u, double* v) const {
  if (u == nullptr || v == nullptr || !std::isfinite(bearing_camera.z) ||
      bearing_camera.z <= config_.min_project_z) {
    return false;
  }
  double x = bearing_camera.x / bearing_camera.z;
  double y = bearing_camera.y / bearing_camera.z;
  if (config_.distortion_model == CameraDistortionModel::EQUIDISTANT) {
    const double radius = std::hypot(x, y);
    if (radius > math::kEps) {
      const double theta = std::atan(radius);
      const double distorted_radius =
          equidistantRadius(theta, config_.distortion_coeffs);
      const double scale = distorted_radius / radius;
      x *= scale;
      y *= scale;
    }
  }
  *u = math::clamp(config_.fx * x + config_.cx, 0.0,
                   static_cast<double>(config_.image_width - 1));
  *v = math::clamp(config_.fy * y + config_.cy, 0.0,
                   static_cast<double>(config_.image_height - 1));
  return std::isfinite(*u) && std::isfinite(*v);
}

void CameraModel::bearingJacobiansFromPixel(
    double u, double v, math::Vec3* derivative_u,
    math::Vec3* derivative_v) const {
  if (config_.distortion_model == CameraDistortionModel::EQUIDISTANT) {
    // A central difference keeps the covariance mapping consistent with the
    // same inverse equidistant projection used for the measurement itself.
    constexpr double kPixelStep = 1e-3;
    const math::Vec3 bearing = bearingFromPixel(u, v);
    if (derivative_u != nullptr) {
      const Eigen::Vector3d difference =
          math::toEigen(bearingFromPixel(u + kPixelStep, v)) -
          math::toEigen(bearingFromPixel(u - kPixelStep, v));
      *derivative_u = math::projectTangent(
          math::fromEigen(difference / (2.0 * kPixelStep)), bearing);
    }
    if (derivative_v != nullptr) {
      const Eigen::Vector3d difference =
          math::toEigen(bearingFromPixel(u, v + kPixelStep)) -
          math::toEigen(bearingFromPixel(u, v - kPixelStep));
      *derivative_v = math::projectTangent(
          math::fromEigen(difference / (2.0 * kPixelStep)), bearing);
    }
    return;
  }

  const math::Vec3 ray = math::makeVec(
      (u - config_.cx) / std::max(config_.fx, 1e-6),
      (v - config_.cy) / std::max(config_.fy, 1e-6), 1.0);
  const double ray_norm = std::max(math::norm(ray), 1e-6);
  const math::Vec3 bearing = math::normalize(ray);
  if (derivative_u != nullptr) {
    *derivative_u = math::projectTangent(
        math::makeVec(1.0 / std::max(config_.fx, 1e-6), 0.0, 0.0),
        bearing) / ray_norm;
  }
  if (derivative_v != nullptr) {
    *derivative_v = math::projectTangent(
        math::makeVec(0.0, 1.0 / std::max(config_.fy, 1e-6), 0.0),
        bearing) / ray_norm;
  }
}

}  // namespace dc_s2ekf
