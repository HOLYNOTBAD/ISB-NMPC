#!/bin/bash

set +e

SH_DIR=$(dirname "$0")
WS_DIR=$(realpath "${SH_DIR}/../..")
TERM_TTY_FILE="${WS_DIR}/.all_sh_ttys"
TERM_ENV_FILE="${WS_DIR}/.interception_terminal_env"

echo "Workspace Directory: ${WS_DIR}"
echo "Cleaning interception, Gazebo SITL, and ROS processes..."

SCREEN_SESSIONS=(
    gazebo_px4_sitl
    gazebo_mavros
    gazebo_camera_relay
    gazebo_yolo_detection
    gazebo_gazebo_balloon
    gazebo_roscore
    gazebo_load_camera_params
    gazebo_rviz
    gazebo_rosbag
)

PROCESS_PATTERNS=(
    roscore
    rosmaster
    roslaunch
    rosbag
    mavros_node
    fsm_node
    target_estimate_cam
    target_manager
    so3_mpc_bridge
    loop_simulation_node.py
    loop_comparison_node.py
    interception_baseline_node.py
    gazebo_balloon_node
    gzserver
    gzclient
    "/build/px4_sitl_default/bin/px4"
    "topic_tools/relay"
    "/target_detection_yolo_node"
    bbox_s2_delay_ekf_node
    rviz
    analyze_isb_nmpc_online.py
)

join_patterns() {
    local pattern=""
    local item
    for item in "${PROCESS_PATTERNS[@]}"; do
        if [ -z "${pattern}" ]; then
            pattern="${item}"
        else
            pattern="${pattern}|${item}"
        fi
    done
    echo "${pattern}"
}

TARGET_PATTERN=$(join_patterns)

collect_target_pids() {
    ps -ef \
        | grep -E "${TARGET_PATTERN}" \
        | grep -v grep \
        | awk -v self="$$" '$2 != self {print $2}' \
        | sort -u
}

if [ -f "${WS_DIR}/devel/setup.bash" ]; then
    source "${WS_DIR}/devel/setup.bash"
fi

if command -v screen >/dev/null 2>&1; then
    for session in "${SCREEN_SESSIONS[@]}"; do
        screen -S "${session}" -X quit >/dev/null 2>&1 || true
    done
fi

if rostopic list >/dev/null 2>&1; then
    rosnode list 2>/dev/null | while read -r node; do
        if [ -n "${node}" ] && [ "${node}" != "/rosout" ]; then
            echo "Killing ROS node: ${node}"
            rosnode kill "${node}" >/dev/null 2>&1
        fi
    done
else
    echo "ROS master is not reachable, skipping rosnode kill."
fi

sleep 1

SELF_TTY=$(ps -o tty= -p $$ | tr -d ' ')
PIDS=$(collect_target_pids)
TARGET_TTYS=$(echo "${PIDS}" | xargs -r -n1 ps -o tty= -p | tr -d ' ' | grep -v '^?$' | sort -u)

if [ -f "${TERM_TTY_FILE}" ]; then
    FILE_TTYS=$(grep -E '^/dev/pts/[0-9]+$' "${TERM_TTY_FILE}" | sed 's#^/dev/##' | sort -u)
    TARGET_TTYS=$(printf "%s\n%s\n" "${TARGET_TTYS}" "${FILE_TTYS}" | sed '/^$/d' | sort -u)
fi

if [ -n "${PIDS}" ]; then
    echo "Sending SIGTERM to remaining processes..."
    echo "${PIDS}" | xargs -r kill
    sleep 2
fi

PIDS=$(collect_target_pids)
if [ -n "${PIDS}" ]; then
    echo "Sending SIGKILL to stubborn processes..."
    echo "${PIDS}" | xargs -r kill -9
fi

if [ -n "${TARGET_TTYS}" ]; then
    echo "Closing matched terminal sessions..."
    echo "${TARGET_TTYS}" | while read -r tty; do
        [ -z "${tty}" ] && continue
        [ "${tty}" = "${SELF_TTY}" ] && continue
        pkill -TERM -t "${tty}" >/dev/null 2>&1
    done
    sleep 1
    echo "${TARGET_TTYS}" | while read -r tty; do
        [ -z "${tty}" ] && continue
        [ "${tty}" = "${SELF_TTY}" ] && continue
        pkill -KILL -t "${tty}" >/dev/null 2>&1
    done
fi

rm -f "${TERM_TTY_FILE}"
rm -f "${TERM_ENV_FILE}"

pkill -9 -f 'roscore|rosmaster|roslaunch|rosout|rosbag|rosnode|rostopic|rosservice|rosparam' >/dev/null 2>&1
pkill -9 -f 'gzserver|gzclient|/build/px4_sitl_default/bin/px4' >/dev/null 2>&1

echo "Done."
