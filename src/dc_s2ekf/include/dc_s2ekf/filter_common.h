#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

namespace dc_s2ekf {
namespace math {

// ==========================================================================
//  Basic Geometry Types（基础几何类型）
// ==========================================================================
constexpr double kEps = 1e-9;

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quat {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// ==========================================================================
//  Eigen Conversions and Vector Operators（Eigen转换与向量运算符）
// ==========================================================================
inline Vec3 makeVec(double x, double y, double z) { return Vec3{x, y, z}; }

inline Eigen::Vector3d toEigen(const Vec3& value) {
  return Eigen::Vector3d(value.x, value.y, value.z);
}

inline Vec3 fromEigen(const Eigen::Vector3d& value) {
  return makeVec(value.x(), value.y(), value.z());
}

inline Eigen::Quaterniond toEigen(const Quat& value) {
  return Eigen::Quaterniond(value.w, value.x, value.y, value.z);
}

inline Quat fromEigen(const Eigen::Quaterniond& value) {
  return Quat{value.w(), value.x(), value.y(), value.z()};
}

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
  return fromEigen(toEigen(a) + toEigen(b));
}

inline Vec3 operator*(double scale, const Vec3& value) {
  return fromEigen(scale * toEigen(value));
}

inline Vec3 operator*(const Vec3& value, double scale) {
  return scale * value;
}

inline Vec3 operator/(const Vec3& value, double scale) {
  return fromEigen(toEigen(value) / scale);
}

// ==========================================================================
//  Vector Geometry（向量几何）
// ==========================================================================
inline double dot(const Vec3& a, const Vec3& b) {
  return toEigen(a).dot(toEigen(b));
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return fromEigen(toEigen(a).cross(toEigen(b)));
}

inline double norm(const Vec3& value) {
  return toEigen(value).norm();
}

inline Vec3 normalize(const Vec3& value,
                      const Vec3& fallback = Vec3{0.0, 0.0, 1.0}) {
  const double length = norm(value);
  if (!std::isfinite(length) || length < kEps) {
    return fallback;
  }
  return value / length;
}

inline Vec3 projectTangent(const Vec3& value, const Vec3& unit_axis) {
  const Eigen::Vector3d axis = toEigen(unit_axis);
  return fromEigen(toEigen(value) - axis.dot(toEigen(value)) * axis);
}

// ==========================================================================
//  Scalar Utilities（标量工具）
// ==========================================================================
inline double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

inline double smoothstep(double edge0, double edge1, double value) {
  if (edge1 <= edge0 + 1e-6) {
    return value >= edge1 ? 1.0 : 0.0;
  }
  const double ratio = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
  return ratio * ratio * (3.0 - 2.0 * ratio);
}

// ==========================================================================
//  Quaternion and Rotation Utilities（四元数与旋转工具）
// ==========================================================================
inline Quat normalize(const Quat& value) {
  const Eigen::Quaterniond quaternion = toEigen(value);
  const double length = quaternion.norm();
  if (!std::isfinite(length) || length < kEps) {
    return Quat{};
  }
  return fromEigen(Eigen::Quaterniond(quaternion.coeffs() / length));
}

inline Eigen::Matrix3d quatToMatrix(const Quat& quaternion) {
  const Quat q = normalize(quaternion);
  return toEigen(q).toRotationMatrix();
}

inline Vec3 rotateMat(const Eigen::Matrix3d& rotation, const Vec3& value) {
  return fromEigen(rotation * toEigen(value));
}

inline Quat slerp(Quat a, Quat b, double ratio) {
  a = normalize(a);
  b = normalize(b);
  return fromEigen(
      toEigen(a).slerp(clamp(ratio, 0.0, 1.0), toEigen(b)).normalized());
}

// ==========================================================================
//  ROS Message and Time Utilities（ROS消息与时间工具）
// ==========================================================================
inline Quat fromMsg(const geometry_msgs::Quaternion& value) {
  return normalize(Quat{value.w, value.x, value.y, value.z});
}

inline Quat fromMsg(const sensor_msgs::ImuConstPtr& message) {
  return fromMsg(message->orientation);
}

inline double stampToSec(const ros::Time& stamp) {
  return stamp.isZero() ? std::numeric_limits<double>::quiet_NaN()
                        : stamp.toSec();
}

// ros::Time subtraction first constructs a signed 32-bit ros::Duration and
// throws when two clocks/epochs differ by more than about 68 years. Convert
// each endpoint independently so malformed or mixed-clock input can be
// rejected without terminating the node.
inline double timeDeltaSec(const ros::Time& lhs, const ros::Time& rhs) {
  return lhs.toSec() - rhs.toSec();
}

// Shift a ROS timestamp without constructing an intermediate negative Time.
// This is needed near the beginning of simulated time, where subtracting even
// a small fallback delay from ros::Time can otherwise throw.
inline ros::Time shiftedTime(const ros::Time& stamp, double offset_s) {
  const double shifted_s = stamp.toSec() + offset_s;
  if (!std::isfinite(shifted_s) || shifted_s <= 0.0) {
    return ros::Time();
  }
  const double max_time_s = std::nextafter(4294967296.0, 0.0);
  ros::Time result;
  result.fromSec(std::min(shifted_s, max_time_s));
  return result;
}

}  // namespace math
}  // namespace dc_s2ekf

namespace dc_s2ekf {

// ==========================================================================
//  Shared Filter State Types（滤波器公共状态类型）
// ==========================================================================
using Measurement = Eigen::Vector4d;
using SizeState = Eigen::Vector4d;
using SizeCovariance = Eigen::Matrix4d;
using S2ErrorState = Eigen::Matrix<double, 4, 1>;
using S2Covariance = Eigen::Matrix<double, 4, 4>;
using S2Gain = Eigen::Matrix<double, 4, 2>;

struct MeasurementQuality {
  double weight = 1.0;
  double area_weight = 1.0;
  double confidence_weight = 1.0;
  double edge_weight = 1.0;
  double jump_weight = 1.0;
  double jump_rate_px_s = 0.0;
};

// ==========================================================================
//  S2 Filter Types（S2滤波器类型）
// ==========================================================================
struct S2FilterState {
  math::Vec3 bearing_c{0.0, 0.0, 1.0};
  math::Vec3 omega_c{};
  math::Vec3 basis_e1_c{1.0, 0.0, 0.0};
  math::Vec3 basis_e2_c{0.0, 1.0, 0.0};
  S2Covariance P = S2Covariance::Zero();
};

struct S2MeasurementNoise {
  math::Vec3 pixel_u_jacobian_c{};
  math::Vec3 pixel_v_jacobian_c{};
  double variance_u_px2 = 16.0;
  double variance_v_px2 = 16.0;
};

enum class S2TrackPhase {
  INIT_BEARING,
  TRACKING
};

namespace eigen {

// ==========================================================================
//  Covariance Matrix Utilities（协方差矩阵工具）
// ==========================================================================
// Materialize matrix products to avoid Eigen aliases during in-place updates.
template <int N>
Eigen::Matrix<double, N, N> similarityTransform(
    const Eigen::Matrix<double, N, N>& transform,
    const Eigen::Matrix<double, N, N>& covariance) {
  Eigen::Matrix<double, N, N> left;
  left.noalias() = transform * covariance;
  Eigen::Matrix<double, N, N> result;
  result.noalias() = left * transform.transpose();
  return result;
}

template <int N, int M>
Eigen::Matrix<double, N, N> josephCovariance(
    const Eigen::Matrix<double, N, N>& covariance,
    const Eigen::Matrix<double, N, M>& gain,
    const Eigen::Matrix<double, M, N>& measurement,
    const Eigen::Matrix<double, M, M>& measurement_noise) {
  Eigen::Matrix<double, N, N> residual =
      Eigen::Matrix<double, N, N>::Identity();
  residual.noalias() -= gain * measurement;
  Eigen::Matrix<double, N, N> result =
      similarityTransform<N>(residual, covariance);
  Eigen::Matrix<double, N, M> gain_noise;
  gain_noise.noalias() = gain * measurement_noise;
  result.noalias() += gain_noise * gain.transpose();
  return result;
}

template <int N, int M>
Eigen::Matrix<double, N, N> josephCovariance(
    const Eigen::Matrix<double, N, N>& covariance,
    const Eigen::Matrix<double, N, M>& gain,
    const Eigen::Matrix<double, M, M>& measurement_noise) {
  Eigen::Matrix<double, M, N> measurement =
      Eigen::Matrix<double, M, N>::Zero();
  measurement.template leftCols<M>().setIdentity();
  return josephCovariance<N, M>(
      covariance, gain, measurement, measurement_noise);
}

template <int N>
void symmetrizeAndFloor(
    Eigen::Matrix<double, N, N>* covariance,
    const Eigen::Matrix<double, N, 1>& diagonal_floor) {
  if (covariance == nullptr) {
    return;
  }
  *covariance = (0.5 * (*covariance + covariance->transpose())).eval();
  covariance->diagonal() =
      covariance->diagonal().cwiseMax(diagonal_floor);
}

template <int N>
void symmetrizeAndFloor(Eigen::Matrix<double, N, N>* covariance,
                        double diagonal_floor) {
  symmetrizeAndFloor<N>(
      covariance, Eigen::Matrix<double, N, 1>::Constant(diagonal_floor));
}

}  // namespace eigen
}  // namespace dc_s2ekf
