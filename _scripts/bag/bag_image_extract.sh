#!/bin/bash
set -euo pipefail

if [ "$#" -gt 2 ]; then
  echo "Usage: $0 [INPUT.bag [OUTPUT.bag]]" >&2
  exit 1
fi

if [ "$#" -eq 0 ]; then
  WS_DIR="$(cd -- "$(dirname -- "$0")/.." && pwd)"
  found=0
  while IFS= read -r -d '' input_bag; do
    found=1
    output_bag="${input_bag%.bag}_cropped.bag"
    echo "Processing: ${input_bag} -> ${output_bag}"
    "$0" "$input_bag" "$output_bag"
  done < <(find "${WS_DIR}/rosbag" -type f -name '*.bag' \
    ! -name '*_cropped.bag' ! -name '*_image_raw.bag' -print0)

  if [ "$found" -eq 0 ]; then
    echo "No .bag files found under ${WS_DIR}/rosbag" >&2
    exit 1
  fi
  exit 0
fi

INPUT=$(realpath "$1")
OUTPUT=${2:-${INPUT%.bag}_image_raw.bag}
source "$(dirname "$0")/../devel/setup.bash"

python3 - "$INPUT" "$OUTPUT" <<'PY'
import os
import sys
import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import CompressedImage, Image

src, dst = sys.argv[1:]
compressed_topic = "/camera/image_raw/mjpeg"
raw_topic = "/camera/image_raw"
fsm_topic = "/fsm/fsm_state"
trigger_state = "EXEC_INTERCEPTION"

with rosbag.Bag(src, "r") as input_bag, rosbag.Bag(dst, "w") as output_bag:
    trigger_time = None
    last_exit_time = None
    interception_active = False
    if os.environ.get("BAG_IMAGE_EXTRACT_NO_CROP", "0") != "1":
        for topic, msg, bag_time in input_bag.read_messages(topics=[fsm_topic]):
            state = str(getattr(msg, "state_name", "")).strip()
            if state == trigger_state:
                if trigger_time is None:
                    trigger_time = bag_time
                interception_active = True
            elif interception_active:
                last_exit_time = bag_time
                interception_active = False
    else:
        print("crop disabled: input is already cropped", file=sys.stderr)

    if trigger_time is None:
        print("warning: no EXEC_INTERCEPTION state; keeping all images",
              file=sys.stderr)
    else:
        print("keeping images from first EXEC_INTERCEPTION at %.6f" %
              trigger_time.to_sec())
        if last_exit_time is None:
            print("warning: no EXEC_INTERCEPTION exit; keeping until bag end",
                  file=sys.stderr)
        else:
            end_time = last_exit_time + rospy.Duration.from_sec(4.0)
            print("cropping messages after last EXEC_INTERCEPTION exit at "
                  "%.6f (+4.0s => %.6f)" %
                  (last_exit_time.to_sec(), end_time.to_sec()))

    for topic, msg, bag_time in input_bag.read_messages():
        if trigger_time is not None and bag_time < trigger_time:
            continue
        if trigger_time is not None and last_exit_time is not None and \
                bag_time > last_exit_time + rospy.Duration.from_sec(4.0):
            continue

        if topic == raw_topic:
            output_bag.write(topic, msg, bag_time)
            continue
        if topic != compressed_topic:
            output_bag.write(topic, msg, bag_time)
            continue

        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            print("warning: skipped undecodable JPEG at %.6f" % bag_time.to_sec(),
                  file=sys.stderr)
            continue

        # Match camera_params_usb_imx577_30degree.yaml rotation_angle: 180.
        image = cv2.rotate(image, cv2.ROTATE_180)
        out = Image()
        out.header = msg.header
        out.height, out.width = image.shape[:2]
        out.encoding = "bgr8"
        out.is_bigendian = False
        out.step = out.width * 3
        out.data = image.tobytes()
        output_bag.write(raw_topic, out, bag_time)

print("wrote", dst)
PY
