#!/bin/bash
# Controller-only Gazebo comparison suite (bvr loop_comparison mode).
set -e
SH_DIR=$(dirname "$0")
WS_DIR=$(realpath "${SH_DIR}/../..")
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${WS_DIR}/devel/setup.bash"

# One camera profile for truth projection, controllers, RViz and Gazebo.
CAMERA_PARAM_FILE="${WS_DIR}/src/camera_driver/config/camera/camera_params_usb_imx577_30degree.yaml"
SOURCE_MOTION_FILE="${WS_DIR}/src/target_register/config/balloon_motion/balloon_motion_random_walk_radius_1m.yaml"
MOTION_FILE="${WS_DIR}/log/loop_comparison/active_motion.yaml"
COMPARISON_METHODS="${COMPARISON_METHODS:-isb_nmpc,backstepping,pslos,rotate_highspeed_attack}"
LOOP_COUNT="${LOOP_COUNT:-30}"
LOOP_TIMEOUT="${LOOP_TIMEOUT:-100.0}"
COMPARISON_TRAJECTORY_SEED="${COMPARISON_TRAJECTORY_SEED:-1000}"
LOOP_OUTPUT_DIR="${LOOP_OUTPUT_DIR:-${WS_DIR}/results/loop_comparison}"
ENABLE_RVIZ="${ENABLE_RVIZ:-1}"
ENABLE_LAUNCH_TERMINALS="${ENABLE_LAUNCH_TERMINALS:-1}"

# Resolve all required packages before starting the simulation.
rospack find interception_baseline >/dev/null
rospack find loop_comparison >/dev/null
python3 "${WS_DIR}/src/holydrone_ros/scripts/sync_gazebo_camera_model.py" \
    --config "${CAMERA_PARAM_FILE}"
rosrun loop_comparison prepare_comparison_motion.py \
    --source-motion-file "${SOURCE_MOTION_FILE}" --motion-file "${MOTION_FILE}" \
    --balloon-radius 1.0 --uav-collision-radius 0.34

USE_LAUNCH_TERMINALS=0
if [ "${ENABLE_LAUNCH_TERMINALS}" = "1" ]; then
    if command -v gnome-terminal >/dev/null 2>&1 &&
       { [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; }; then
        USE_LAUNCH_TERMINALS=1
    else
        echo "[loop_comparison] No graphical terminal session detected; using the current terminal." >&2
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
    local title="COMPARISON | ${package}/${launch_file}"
    echo "[loop_comparison] roslaunch $*"
    start_process "${title}" roslaunch "$@"
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


start_launch holydrone sitl.launch px4_sim_speed_factor:="${PX4_SIM_SPEED_FACTOR:-4.0}"
wait_for_master
roslaunch camera_driver load_camera_param.launch camera_param_file:="${CAMERA_PARAM_FILE}"
start_launch holydrone mavros_sim.launch
start_launch target_register gazebo_balloon.launch \
    balloon_motion_file:="${MOTION_FILE}" balloon_radius:=1.0 uav_collision_radius:=0.34 \
    spawn_on_start:=false spawn_relative_to_mav:=true auto_advance_on_idle:=false \
    id_marker_text_height:=2.5
start_launch target_manager target_manager.launch \
    param_file:="${WS_DIR}/src/target_manager/config/target_comparison.yaml" \
    raw_masspoint_topic:=/loop_comparison/target_masspoint_map mav_vel:=14.0
start_launch target_estimate target_estimate.launch \
    target_param_file:="${WS_DIR}/src/target_manager/config/target_comparison.yaml"
start_launch plan_manage fsm_loopcomparison.launch
[ "${ENABLE_RVIZ}" = "1" ] && start_launch interception_visualization rviz_loopcomparison.launch
# The runner starts/stops each controller itself. No YOLO or EKF in truth comparison.
start_launch loop_comparison loop_comparison.launch \
    controllers:="${COMPARISON_METHODS}" count:="${LOOP_COUNT}" timeout:="${LOOP_TIMEOUT}" \
    trajectory_seed:="${COMPARISON_TRAJECTORY_SEED}" \
    source_motion_file:="${SOURCE_MOTION_FILE}" motion_file:="${MOTION_FILE}" \
    output_dir:="${LOOP_OUTPUT_DIR}"

if [ "${USE_LAUNCH_TERMINALS}" = "1" ]; then
    echo "Comparison suite started. Results: ${LOOP_OUTPUT_DIR}"
    echo "Use _scripts/run/kill.sh to stop the stack."
    exit 0
fi
# Keep the simulator alive through the entire managed comparison suite.
set +e
wait "${CHILD_PIDS[${#CHILD_PIDS[@]}-1]}"
STATUS=$?
set -e
exit "${STATUS}"
