#!/bin/bash
# Start a single Gazebo interception simulation.
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
export ENABLE_LIVE_FILTER_EVAL="${ENABLE_LIVE_FILTER_EVAL:-1}"
export ENABLE_ISBNMPC_LOG="${ENABLE_ISBNMPC_LOG:-0}"
export CONTROLLER_DRY_RUN="${CONTROLLER_DRY_RUN:-false}"
export ENABLE_LAUNCH_TERMINALS="${ENABLE_LAUNCH_TERMINALS:-1}"
export TARGET_DETECTION_YOLO_ENGINE_FILE="${TARGET_DETECTION_YOLO_ENGINE_FILE:-${WS_DIR}/src/yolo/weights/balloon_640x480_rtx5080.engine}"
export ROSBAG_OUTPUT_DIR="${ROSBAG_OUTPUT_DIR:-${WS_DIR}/rosbag/simulation}"
export ROSBAG_OUTPUT_SUFFIX="${ROSBAG_OUTPUT_SUFFIX:-interception}"
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

# Keep the Gazebo sensor pose and intrinsics identical to the calibration used
# by target_estimate and MPC. The command is a no-op when they already match.
python3 "${WS_DIR}/src/holydrone_ros/scripts/sync_gazebo_camera_model.py" \
    --config "${CAMERA_PARAM_FILE}"

if [ "${ENABLE_DC_S2EKF}" = "1" ]; then
    export DETECTION_OUTPUT_BBOX_TOPIC=/tracker/bbox_raw
else
    export DETECTION_OUTPUT_BBOX_TOPIC=/tracker/bbox
fi

USE_LAUNCH_TERMINALS=0
if [ "${ENABLE_LAUNCH_TERMINALS}" = "1" ]; then
    if command -v gnome-terminal >/dev/null 2>&1 &&
       { [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; }; then
        USE_LAUNCH_TERMINALS=1
    else
        echo "[interception_sim] No graphical terminal session detected; using the current terminal." >&2
    fi
fi

CHILD_PIDS=()
SHUTTING_DOWN=0
TERMINAL_WINDOW_STARTED=0

start_process() {
    local title="$1"
    shift

    if [ "${USE_LAUNCH_TERMINALS}" = "1" ]; then
        local terminal_mode="--tab"
        if [ "${TERMINAL_WINDOW_STARTED}" = "0" ]; then
            terminal_mode="--window"
            TERMINAL_WINDOW_STARTED=1
        fi
        # shellcheck disable=SC2016  # Expanded by the child bash, not here.
        gnome-terminal "${terminal_mode}" --title="${title}" -- bash -lc '
            workspace=$1
            shift
            source /opt/ros/noetic/setup.bash
            source "${workspace}/devel/setup.bash"
            exec "$@"
        ' bash "${WS_DIR}" "$@"
    else
        "$@" &
        CHILD_PIDS+=("$!")
    fi
}

start_launch() {
    local package="$1"
    local launch_file="$2"
    local title="SIM | ${package}/${launch_file}"
    case "${package}/${launch_file}" in
        target_register/gazebo_balloon_sim.launch)
            title="01 INTERACTIVE | BALLOON KEYBOARD"
            ;;
        plan_manage/fsm_sim.launch)
            title="02 INTERACTIVE | FSM (ENTER)"
            ;;
    esac
    echo "[interception_sim] roslaunch $*"
    start_process "${title}" roslaunch "$@"
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
    echo "[interception_sim] rosbag record -o ${output_prefix} (${#topics[@]} topics)"
    start_process "SIM | rosbag record" \
        rosbag record -o "${output_prefix}" "${topics[@]}"
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
    local _
    for _ in {1..100}; do
        if rosparam get /rosversion >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.1
    done
    echo "Timed out waiting for ROS master." >&2
    return 1
}

trap 'cleanup; exit 130' INT TERM
trap cleanup EXIT

start_launch holydrone sitl.launch
wait_for_master
# Load synchronously: all consumers must start after the parameters are available.
roslaunch camera_driver load_camera_param.launch \
    camera_param_file:="${CAMERA_PARAM_FILE}"
start_launch target_register gazebo_balloon_sim.launch
start_launch plan_manage fsm_sim.launch
start_launch holydrone mavros_sim.launch
start_launch holydrone tpv_camera.launch
start_launch target_detection_yolo target_detection_yolo_sim.launch \
    engine_file:="${TARGET_DETECTION_YOLO_ENGINE_FILE}"
start_launch target_detection_hsv target_detection_hsv.launch \
    image_topic:=/camera/image_raw \
    detection_topic:=/tracker/p_image_truth_hsv
if [ "${ENABLE_DC_S2EKF}" = "1" ]; then
    start_launch dc_s2ekf bbox_s2_delay_ekf.launch
fi
[ "${ENABLE_LIVE_FILTER_EVAL}" = "1" ] && start_launch dc_s2ekf live_filter_eval.launch
start_launch target_estimate target_estimate.launch
start_launch isb_nmpc so3_mpc_sim.launch
[ "${ENABLE_RVIZ}" = "1" ] && start_launch interception_visualization rviz_sim.launch

# The manager uses identical defaults in every mode.
start_launch target_manager target_manager.launch

if [ "${ENABLE_ROSBAG}" = "1" ]; then
    start_rosbag
fi

if [ "${USE_LAUNCH_TERMINALS}" = "1" ]; then
    echo "[interception_sim] Launch terminals started. Use _scripts/run/kill.sh to stop the stack."
    exit 0
fi

echo "[interception_sim] ${#CHILD_PIDS[@]} child processes are running."
set +e
wait -n "${CHILD_PIDS[@]}"
STATUS=$?
set -e
exit "${STATUS}"
