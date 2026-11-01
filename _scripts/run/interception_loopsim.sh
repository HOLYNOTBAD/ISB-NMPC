#!/bin/bash
# Start repeated Gazebo interception simulations.
# Select the camera profile here; load_camera_param.launch loads it for all nodes.

set -e

SH_DIR=$(dirname "$0")
WS_DIR=$(realpath "${SH_DIR}/../..")

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f "${WS_DIR}/devel/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_DIR}/devel/setup.bash"
else
    echo "Workspace is not built: ${WS_DIR}/devel/setup.bash is missing." >&2
    exit 2
fi

export ENABLE_DC_S2EKF="${ENABLE_DC_S2EKF:-1}"
export ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
export ENABLE_ROSBAG="${ENABLE_ROSBAG:-0}"
export ENABLE_ISBNMPC_LOG="${ENABLE_ISBNMPC_LOG:-0}"
export CONTROLLER_DRY_RUN="${CONTROLLER_DRY_RUN:-false}"
export TARGET_DETECTION_YOLO_ENGINE_FILE="${TARGET_DETECTION_YOLO_ENGINE_FILE:-${WS_DIR}/src/yolo/weights/balloon_640x480_rtx5080.engine}"
export ROSBAG_OUTPUT_DIR="${ROSBAG_OUTPUT_DIR:-${WS_DIR}/rosbag/loop_simulation}"
export ROSBAG_OUTPUT_SUFFIX="${ROSBAG_OUTPUT_SUFFIX:-interception}"
export LOOP_COUNT="${LOOP_COUNT:-10}"
export LOOP_TIMEOUT="${LOOP_TIMEOUT:-55.0}"
export LOOP_OUTPUT_DIR="${LOOP_OUTPUT_DIR:-${WS_DIR}/log/loop_interception}"
export ENABLE_LOOP_RESULTS="${ENABLE_LOOP_RESULTS:-0}"
CAMERA_PARAM_FILE="${WS_DIR}/src/camera_driver/config/camera/camera_params_usb_imx577_30degree.yaml"

if [ ! -f "${CAMERA_PARAM_FILE}" ]; then
    echo "Camera profile not found: ${CAMERA_PARAM_FILE}" >&2
    exit 2
fi

CAMERA_PARAM_FILE=$(realpath "${CAMERA_PARAM_FILE}")

if [ ! -f "${TARGET_DETECTION_YOLO_ENGINE_FILE}" ]; then
    echo "TensorRT engine not found: ${TARGET_DETECTION_YOLO_ENGINE_FILE}" >&2
    echo "Set TARGET_DETECTION_YOLO_ENGINE_FILE to an engine built for this machine." >&2
    exit 2
fi

# Loop simulation uses the same nominal camera model as single simulation.
python3 "${WS_DIR}/src/holydrone_ros/scripts/sync_gazebo_camera_model.py" \
    --config "${CAMERA_PARAM_FILE}"

if [ "${ENABLE_DC_S2EKF}" = "1" ]; then
    export DETECTION_OUTPUT_BBOX_TOPIC=/tracker/bbox_raw
else
    export DETECTION_OUTPUT_BBOX_TOPIC=/tracker/bbox
fi

CHILD_PIDS=()
SHUTTING_DOWN=0

start_launch() {
    echo "[interception_loopsim] roslaunch $*"
    roslaunch "$@" &
    CHILD_PIDS+=("$!")
}

start_rosbag() {
    local output_prefix="${ROSBAG_OUTPUT_DIR}/${ROSBAG_OUTPUT_SUFFIX}"
    local topics=(
        /nmpc/uavStateEstimate /nmpc/targetStateEstimate
        /target_detection/mode /target_detection/roi
        /nmpc/target_in_main_view /nmpc/failure
        /tracker/bbox_raw /tracker/bbox /tracker/bbox_roi_prediction
        /balloons/masspoint /balloons/intercepted /balloons/event
        /nmpc/p_image /nmpc/p_target /nmpc/q_image
        /fsm/intercepted /fsm/fsm_state /fsm/motion_trigger
        /target_manager/attack_sequence /target_manager/active_target
        /camera/image_raw /camera/image_raw/mjpeg /gazebo/model_states
        /tracker/p_image_truth_hsv
        /mavros/state /mavros/time_reference /mavros/imu/data
        /mavros/local_position/pose /mavros/local_position/velocity_local
        /mavros/odometry/in /mavros/vision_pose/pose
        /vrpn_client_node/holydrone/pose /vrpn_client_node/holydrone/twist
        /mavros/setpoint_raw/attitude
        /isb_nmpc/solve_latency_ms /isb_nmpc/predicted_trajectory_vis
        /isb_nmpc/control_command
    )

    mkdir -p "${ROSBAG_OUTPUT_DIR}"
    echo "[interception_loopsim] rosbag record -o ${output_prefix} (${#topics[@]} topics)"
    rosbag record -o "${output_prefix}" "${topics[@]}" &
    CHILD_PIDS+=("$!")
}

cleanup() {
    if [ "${SHUTTING_DOWN}" = "1" ]; then
        return
    fi
    SHUTTING_DOWN=1
    trap - INT TERM EXIT
    if [ "${#CHILD_PIDS[@]}" -gt 0 ]; then
        kill -INT "${CHILD_PIDS[@]}" >/dev/null 2>&1 || true
        wait "${CHILD_PIDS[@]}" 2>/dev/null || true
    fi
}

wait_for_master() {
    local first_pid="$1"
    local _
    for _ in {1..100}; do
        if rosparam get /rosversion >/dev/null 2>&1; then
            return 0
        fi
        if ! kill -0 "${first_pid}" >/dev/null 2>&1; then
            echo "The first roslaunch process exited before ROS master was ready." >&2
            return 1
        fi
        sleep 0.1
    done
    echo "Timed out waiting for ROS master." >&2
    return 1
}

trap 'cleanup; exit 130' INT TERM
trap cleanup EXIT

start_launch holydrone sitl.launch px4_sim_speed_factor:=4.0
wait_for_master "${CHILD_PIDS[0]}"
# Load synchronously: all consumers must start after the parameters are available.
roslaunch camera_driver load_camera_param.launch \
    camera_param_file:="${CAMERA_PARAM_FILE}"
start_launch holydrone mavros_sim.launch
start_launch target_register gazebo_balloon_loopsim.launch
start_launch target_detection_yolo target_detection_yolo_sim.launch \
    engine_file:="${TARGET_DETECTION_YOLO_ENGINE_FILE}"
if [ "${ENABLE_DC_S2EKF}" = "1" ]; then
    start_launch dc_s2ekf bbox_s2_delay_ekf.launch
fi
start_launch target_estimate target_estimate.launch
start_launch plan_manage fsm_loopsim.launch
start_launch isb_nmpc so3_mpc_loopsim.launch
[ "${ENABLE_RVIZ}" = "1" ] && start_launch interception_visualization rviz_loopsim.launch
start_launch loop_simulation loop_simulation.launch

# The manager uses identical defaults in every mode.
start_launch target_manager target_manager.launch

if [ "${ENABLE_ROSBAG}" = "1" ]; then
    start_rosbag
fi

echo "[interception_loopsim] ${#CHILD_PIDS[@]} child processes are running."
set +e
wait -n "${CHILD_PIDS[@]}"
STATUS=$?
set -e
exit "${STATUS}"
