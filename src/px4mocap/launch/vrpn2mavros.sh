#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)/../devel/setup.bash"

VRPN_SERVER="${VRPN_SERVER:-127.0.0.1}"
MARKER_NAME="${MARKER_NAME:-holydrone}"
MOCAP_UNIT="${MOCAP_UNIT:-m}"
ZERO_POSITION_ON_FIRST_POSE="${ZERO_POSITION_ON_FIRST_POSE:-true}"
POSITION_OFFSET_X="${POSITION_OFFSET_X:-0.0}"
POSITION_OFFSET_Y="${POSITION_OFFSET_Y:-0.0}"
POSITION_OFFSET_Z="${POSITION_OFFSET_Z:-0.0}"
MAX_ABS_POSITION_M="${MAX_ABS_POSITION_M:-900.0}"
LOCAL_ORIGIN_TOPIC="${LOCAL_ORIGIN_TOPIC:-/local_frame/origin}"

roslaunch px4mocap vrpn2mavros.launch \
  vrpn_server:="${VRPN_SERVER}" \
  marker_name:="${MARKER_NAME}" \
  mocap_unit:="${MOCAP_UNIT}" \
  zero_position_on_first_pose:="${ZERO_POSITION_ON_FIRST_POSE}" \
  position_offset_x:="${POSITION_OFFSET_X}" \
  position_offset_y:="${POSITION_OFFSET_Y}" \
  position_offset_z:="${POSITION_OFFSET_Z}" \
  max_abs_position_m:="${MAX_ABS_POSITION_M}" \
  local_origin_topic:="${LOCAL_ORIGIN_TOPIC}"
