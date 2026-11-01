#pragma once

#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

namespace isb_nmpc {

class Flatness {
 public:
  void computeFlatInputwithHopfFibration(const Eigen::Vector3d& thr_acc,
                                         const Eigen::Vector3d& jer,
                                         const double& yaw,
                                         const double& yawd,
                                         const Eigen::Quaterniond& att_est,
                                         Eigen::Quaterniond& att,
                                         Eigen::Vector3d& omg) const {
    static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);

    const double thrust_norm = thr_acc.norm();
    if (thrust_norm <= 0.1) {
      std::cout << "Near singularity!!!!!" << std::endl;
      omg = omg_old;
      att = att_est;
      return;
    }

    const Eigen::Vector3d abc = thr_acc.normalized();
    const double a = abc(0);
    const double b = abc(1);
    const double c = abc(2);

    const Eigen::Matrix3d projector =
        (thr_acc.dot(thr_acc) * Eigen::Matrix3d::Identity() -
         thr_acc * thr_acc.transpose()) /
        (thrust_norm * thr_acc.squaredNorm());
    const Eigen::Vector3d abc_dot = projector * jer;
    const double a_dot = abc_dot(0);
    const double b_dot = abc_dot(1);
    const double c_dot = abc_dot(2);

    if (1.0 + c > 1e-3) {
      const double norm = std::sqrt(2.0 * (1.0 + c));
      const Eigen::Quaterniond q((1.0 + c) / norm, -b / norm, a / norm, 0.0);
      const Eigen::Quaterniond q_yaw(
          std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
      att = q * q_yaw;

      const double syaw = std::sin(yaw);
      const double cyaw = std::cos(yaw);
      omg(0) =
          syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot / (c + 1.0);
      omg(1) =
          cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot / (c + 1.0);
      omg(2) = (b * a_dot - a * b_dot) / (1.0 + c) + yawd;
    } else {
      std::cout << "Near singularity!!!!!" << std::endl;
      omg = omg_old;
      att = att_est;
    }

    omg_old = omg;
  }
};

}  // namespace isb_nmpc
