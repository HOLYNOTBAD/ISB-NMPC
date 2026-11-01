#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <ros/ros.h>

namespace isb_nmpc {

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

static constexpr int kSamples = ACADO_N;
static constexpr int kStateSize = ACADO_NX;
static constexpr int kRefSize = ACADO_NY;
static constexpr int kEndRefSize = ACADO_NYN;
static constexpr int kInputSize = ACADO_NU;
static constexpr int kCostSize = ACADO_NY - ACADO_NU;
static constexpr int kOdSize = ACADO_NOD;
static constexpr int kAffineConstraintSize = ACADO_N * (ACADO_NPAC + 1);

static_assert(kStateSize == 18,
              "Regenerate so3_quadrotor_mpc_codegen after changing SO3 MPC dimensions.");
static_assert(kInputSize == 7,
              "Regenerate so3_quadrotor_mpc_codegen after changing SO3 MPC dimensions.");
static_assert(kRefSize == 16,
              "Regenerate so3_quadrotor_mpc_codegen after changing SO3 MPC dimensions.");
static_assert(kEndRefSize == 3,
              "Regenerate so3_quadrotor_mpc_codegen after changing SO3 MPC dimensions.");
static_assert(kOdSize == 26,
              "Regenerate so3_quadrotor_mpc_codegen after changing SO3 MPC dimensions.");

enum AUGMENTED_STATE {
  kStateFocusW = 10,
  kStateFocusX = 11,
  kStateFocusY = 12,
  kStateFocusZ = 13,
  kStateRange = 14,
  kStateInterceptSpeed = 15,
  kStateSlackAccumulatorU = 16,
  kStateSlackAccumulatorV = 17,
};

enum INPUT {
  kThrust = 0,
  kRateX = 1,
  kRateY = 2,
  kRateZ = 3,
  kInterceptSpeedRate = 4,
  kSlackU = 5,
  kSlackV = 6,
};

enum ONLINE_DATA {
  kOdCameraQuaternion = 0,
  kOdTargetVelocity = 4,
  kOdMaxInterceptSpeed = 7,
  kOdMaxDeltaInterceptSpeed = 8,
  kOdThrustLimits = 9,
  kOdRateLimits = 11,
  kOdTerminalLinePoint = 14,
  kOdTerminalLineDirection = 17,
  kOdTimeScale = 20,
  kOdImageCenter = 21,
  kOdImageRadius = 23,
  kOdTanMaxRoll = 25,
};

static constexpr int kRefInterceptSpeed = 8;

template <typename T>
class So3MpcWrapper {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  So3MpcWrapper();
  So3MpcWrapper(
      const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R);

  bool setCosts(
      const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
      const T terminal_line_cost = 0.0,
      const T state_cost_scaling = 0.0,
      const T input_cost_scaling = 0.0);

  bool setLimits(T min_thrust, T max_thrust,
                 T max_rollrate, T max_pitchrate, T max_yawrate);
  bool setMaxRollAngle(T max_roll_angle_rad);
  bool setCameraParameters(const Eigen::Quaternion<T>& q_B_C);
  bool setTargetVelocity(
      const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& velocity);
  bool setInterceptSpeedLimits(T max_intercept_speed,
                               T min_intercept_speed,
                               T los_speed_schedule_kh,
                               T los_speed_schedule_kv,
                               T max_delta_intercept_speed);
  bool setInterceptSpeedReference(const Eigen::Quaternion<T>& q_image);
  bool setImagePlaneParameters(T center_u, T center_v,
                               T radius_u, T radius_v);
  bool setTerminalLine(
      const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& point,
      const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& direction);
  bool setTimeScale(T time_scale);

  bool setReferencePose(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
  bool setTrajectory(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs);
  bool setWarmStart(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs);

  // Clear ACADO's numerical workspace between independent interception trials.
  // Configuration stored in acadoVariables is reapplied by the controller.
  void resetSolverWorkspace();
  bool solve(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
  bool update(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
              bool do_preparation = true);
  bool prepare();
  // MPC Debug Logging （MPC调试日志）
  int getLastFeedbackStatus() const { return last_feedback_status_; }
  bool getLastPreparationOk() const { return last_preparation_ok_; }
  T getLastKkt() const { return last_kkt_; }
  // MPC Debug Logging End

  void getState(int node_index,
                Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state);
  void getStates(
      Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states);
  void getInput(int node_index,
                Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input);
  void getInputs(
      Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs);
  T getTimestep() const { return dt_; }
  T getScheduledInterceptSpeed() const { return scheduled_intercept_speed_; }

 private:
  void fillRunningReference();
  void fillTerminalReference();
  void initializeOnlineData();
  bool updateCostWeights();
  bool updateInterceptSpeedRateBounds();

  Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
      acado_reference_{acadoVariables.y};
  Eigen::Map<Eigen::Matrix<float, kEndRefSize, 1, Eigen::ColMajor>>
      acado_reference_end_{acadoVariables.yN};
  Eigen::Map<Eigen::Matrix<float, kStateSize, 1, Eigen::ColMajor>>
      acado_initial_state_{acadoVariables.x0};
  Eigen::Map<Eigen::Matrix<float, kStateSize, kSamples + 1, Eigen::ColMajor>>
      acado_states_{acadoVariables.x};
  Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
      acado_inputs_{acadoVariables.u};
  Eigen::Map<Eigen::Matrix<float, kOdSize, kSamples + 1, Eigen::ColMajor>>
      acado_online_data_{acadoVariables.od};
  Eigen::Map<Eigen::Matrix<float, kRefSize, kRefSize * kSamples>>
      acado_W_{acadoVariables.W};
  Eigen::Map<Eigen::Matrix<float, kEndRefSize, kEndRefSize>>
      acado_W_end_{acadoVariables.WN};
  Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
      acado_lower_bounds_{acadoVariables.lbValues};
  Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
      acado_upper_bounds_{acadoVariables.ubValues};
  Eigen::Map<Eigen::Matrix<float, kAffineConstraintSize, 1, Eigen::ColMajor>>
      acado_lower_path_bounds_{acadoVariables.lbAValues};
  Eigen::Map<Eigen::Matrix<float, kAffineConstraintSize, 1, Eigen::ColMajor>>
      acado_upper_path_bounds_{acadoVariables.ubAValues};

  Eigen::Matrix<T, kRefSize, kRefSize> W_ =
      Eigen::Matrix<T, kRefSize, kRefSize>::Identity();
  Eigen::Matrix<T, kEndRefSize, kEndRefSize> WN_ =
      Eigen::Matrix<T, kEndRefSize, kEndRefSize>::Identity();
  Eigen::Matrix<T, kCostSize, kCostSize> Q_ =
      Eigen::Matrix<T, kCostSize, kCostSize>::Zero();
  Eigen::Matrix<T, kInputSize, kInputSize> R_ =
      Eigen::Matrix<T, kInputSize, kInputSize>::Zero();
  T terminal_line_cost_{0.0};
  T state_cost_scaling_{0.0};
  T input_cost_scaling_{0.0};
  T max_intercept_speed_{2.0};
  T min_intercept_speed_{0.0};
  T los_speed_schedule_kh_{0.0};
  T los_speed_schedule_kv_{0.0};
  T scheduled_intercept_speed_{2.0};
  T max_delta_intercept_speed_{1.0};

  bool acado_is_prepared_{false};
  const T dt_{0.05};
  // MPC Debug Logging （MPC调试日志）
  int last_feedback_status_{0};
  bool last_preparation_ok_{true};
  T last_kkt_{0.0};
  // MPC Debug Logging End
};

}  // namespace isb_nmpc
