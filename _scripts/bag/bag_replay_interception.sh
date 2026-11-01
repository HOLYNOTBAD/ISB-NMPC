#!/bin/bash

set -e

usage() {
    cat <<EOF
Usage: $0 [bag_file]

Replay an interception rosbag with RViz and rqt_bag.

If bag_file is omitted, the newest rosbag/interception*.bag is used.
EOF
}

SH_DIR=$(dirname "$0")
WS_DIR=$(realpath "${SH_DIR}/..")
BAG_DIR="${WS_DIR}/rosbag"
RVIZ_CONFIG="${WS_DIR}/src/interception_visualization/rviz/interception_no_tpv.rviz"
RQT_BAG_PLAY_RATE="${RQT_BAG_PLAY_RATE:-0.5}"
AUTO_PLAY="${AUTO_PLAY:-1}"
AUTO_REINDEX="${AUTO_REINDEX:-1}"

BAG_FILE=""
EXTRACTED_BAG=""
AUTO_SELECTED_BAG=""
REINDEX_DIR=""
BAG_INFO_FILE=""

if [ "$#" -gt 1 ]; then
    echo "Unexpected extra argument: $2"
    usage
    exit 1
fi

if [ "$#" -eq 1 ]; then
    BAG_FILE="$1"
fi

if [ -z "${BAG_FILE}" ]; then
    # Prefer the newest bag that contains HIL balloon truth.  Cropped bags
    # often contain the camera stream only and cannot produce virtual balloons.
    while IFS= read -r candidate; do
        if rosbag info --yaml "${candidate}" 2>/dev/null \
            | awk -F': ' '/^[[:space:]]*- topic: / {print $2}' \
            | grep -Fxq -- "/hil/targets/masspoint"; then
            AUTO_SELECTED_BAG="${candidate}"
            break
        fi
    done < <(find "${BAG_DIR}" -type f -name 'interception*.bag' -printf '%T@ %p\n' \
        | sort -nr | cut -d' ' -f2-)

    if [ -n "${AUTO_SELECTED_BAG}" ]; then
        BAG_FILE="${AUTO_SELECTED_BAG}"
    else
        BAG_FILE=$(find "${BAG_DIR}" -type f -name 'interception*.bag' -printf '%T@ %p\n' \
            | sort -nr | head -n 1 | cut -d' ' -f2-)
    fi
fi

if [ -z "${BAG_FILE}" ] || [ ! -f "${BAG_FILE}" ]; then
    echo "No interception bag file found."
    usage
    exit 1
fi

if [ ! -f "${RVIZ_CONFIG}" ]; then
    echo "RViz config not found: ${RVIZ_CONFIG}"
    exit 1
fi

echo "Workspace Directory: ${WS_DIR}"
echo "Replay bag: ${BAG_FILE}"
echo "RViz config: ${RVIZ_CONFIG}"

# shellcheck disable=SC1090
source "${WS_DIR}/devel/setup.bash"

cleanup() {
    if [ -n "${RVIZ_PID}" ]; then
        kill "${RVIZ_PID}" 2>/dev/null || true
    fi
    if [ -n "${ROSCORE_PID}" ]; then
        kill "${ROSCORE_PID}" 2>/dev/null || true
    fi
    if [ -n "${HIL_MARKERS_PID}" ]; then
        kill "${HIL_MARKERS_PID}" 2>/dev/null || true
    fi
    if [ -n "${EXTRACTED_BAG}" ]; then
        rm -f "${EXTRACTED_BAG}"
    fi
    if [ -n "${BAG_INFO_FILE}" ]; then
        rm -f "${BAG_INFO_FILE}"
    fi
    if [ -n "${REINDEX_DIR}" ]; then
        rm -rf -- "${REINDEX_DIR}"
    fi
}
trap cleanup EXIT

# rosbag records can be left as a complete data file without an index after
# an interrupted copy/recording.  Probe once, recover to a temporary sibling,
# and cache the YAML metadata so every topic check does not repeat the error.
BAG_INFO_FILE="$(mktemp)"
if ! rosbag info --yaml "${BAG_FILE}" >"${BAG_INFO_FILE}" 2>/dev/null; then
    if [ "${AUTO_REINDEX}" != "1" ]; then
        echo "ERROR bag unindexed: ${BAG_FILE}. Run rosbag reindex or set AUTO_REINDEX=1." >&2
        exit 1
    fi
    REINDEX_DIR="$(mktemp -d "$(dirname -- "${BAG_FILE}")/.bag_replay_reindex.XXXXXX")"
    echo "Bag has no usable index; reindexing a temporary copy in ${REINDEX_DIR}..."
    if ! rosbag reindex --output-dir "${REINDEX_DIR}" "${BAG_FILE}"; then
        echo "ERROR failed to reindex bag: ${BAG_FILE}" >&2
        exit 1
    fi
    RECOVERED_BAG="${REINDEX_DIR}/$(basename -- "${BAG_FILE}")"
    if ! rosbag info --yaml "${RECOVERED_BAG}" >"${BAG_INFO_FILE}" 2>/dev/null; then
        echo "ERROR recovered bag still cannot be read: ${RECOVERED_BAG}" >&2
        exit 1
    fi
    BAG_FILE="${RECOVERED_BAG}"
    echo "Reindex complete; recovered bag is ready for replay."
    echo "Using reindexed temporary bag: ${BAG_FILE}"
fi

bag_has_topic() {
    local topic="$1"
    awk -F': ' '/^[[:space:]]*- topic: / {print $2}' "${BAG_INFO_FILE}" \
        | grep -Fxq -- "${topic}"
}

if bag_has_topic "/camera/image_raw/mjpeg" || bag_has_topic "/camera/image_raw"; then
    if [[ "${BAG_FILE}" == *_cropped.bag ]]; then
        # Cropped bags can be replayed directly. The overlay decodes MJPEG
        # itself, so avoid a full-bag decode/re-encode pass here.
        echo "Input is already cropped; replaying compressed images directly."
    else
        EXTRACTED_BAG="$(mktemp --suffix=_image_raw.bag)"
        "${WS_DIR}/_scripts/bag_image_extract.sh" "${BAG_FILE}" "${EXTRACTED_BAG}"
        BAG_FILE="${EXTRACTED_BAG}"
    fi
fi

REPLAY_IMAGE_ARGS=""
if [[ "${BAG_FILE}" == *_cropped.bag ]] && bag_has_topic "/camera/image_raw/mjpeg"; then
    REPLAY_IMAGE_ARGS="fpv_compressed_image_topic:=/camera/image_raw/mjpeg"
elif bag_has_topic "/camera/image_raw"; then
    REPLAY_IMAGE_ARGS="fpv_los_input_topic:=/camera/image_raw fpv_raw_image_topic:=/camera/image_raw"
elif bag_has_topic "/camera/fpv_debug"; then
    REPLAY_IMAGE_ARGS="fpv_los_input_topic:=/camera/fpv_debug"
else
    echo "Warning: bag has no usable image topic for FPV replay."
fi

ROSCORE_PID=""
RVIZ_PID=""
HIL_MARKERS_PID=""

if ! rosparam list >/dev/null 2>&1; then
    roscore >/tmp/bvr_bag_replay_interception_roscore.log 2>&1 &
    ROSCORE_PID=$!
    for _ in $(seq 1 50); do
        if rosparam list >/dev/null 2>&1; then
            break
        fi
        sleep 0.1
    done
fi

rosparam set use_sim_time true

FPV_BALLOON_ARG=""
if bag_has_topic "/hil/targets/masspoint"; then
    FPV_BALLOON_ARG="fpv_draw_virtual_balloon:=true"
fi

roslaunch interception_visualization rviz_view.launch \
    config_path:="${RVIZ_CONFIG}" \
    show_fpv_los_overlay:=true \
    ${REPLAY_IMAGE_ARGS} \
    ${FPV_BALLOON_ARG} \
    show_baylands_mesh:=false \
    >/tmp/bvr_bag_replay_interception_rviz.log 2>&1 &
RVIZ_PID=$!
sleep 1.0

if bag_has_topic "/hil/targets/masspoint"; then
    echo "Starting HIL balloon marker bridge"
    python3 "${WS_DIR}/_scripts/hil_bag_markers.py" >/tmp/bvr_bag_replay_hil_markers.log 2>&1 &
    HIL_MARKERS_PID=$!
fi

if [ "${AUTO_PLAY}" = "1" ]; then
    echo "Running rqt_bag with Publish All and Play at ${RQT_BAG_PLAY_RATE}x: ${BAG_FILE}"
    RQT_BAG_PLAY_RATE="${RQT_BAG_PLAY_RATE}" \
        python3 "${WS_DIR}/_scripts/rqt_bag_auto_publish.py" --clock "${BAG_FILE}"
else
    echo "Running rqt_bag; use Publish All and Play manually: ${BAG_FILE}"
    rqt_bag --clock "${BAG_FILE}"
fi
