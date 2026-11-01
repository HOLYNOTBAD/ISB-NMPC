#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEIGHTS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORK_DIR="${WEIGHTS_DIR}/_work"
YOLOV5_REPO="${WORK_DIR}/yolov5"
TENSORRTX_REPO="${WORK_DIR}/tensorrtx"
TENSORRTX_BUILD="${TENSORRTX_REPO}/yolov5/build"
YOLO_PKG_DIR="$(cd "${WEIGHTS_DIR}/.." && pwd)"

usage() {
  cat <<'USAGE'
Usage:
  make_engine.sh [model.wts|model.pt] [output.engine] [model-size] [width] [height]

Defaults:
  model.wts      ../weights/best_640x480.wts
  output.engine  ../balloon_640x480.engine
  model-size     n
  width          640
  height         480

Example:
  ./make_engine.sh ../weights/best_640x480.wts ../balloon.engine n 640 480
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

MODEL_PATH="$(realpath "${1:-${WEIGHTS_DIR}/weights/best_640x480.wts}")"
ENGINE_PATH="$(realpath -m "${2:-${WEIGHTS_DIR}/balloon_640x480.engine}")"
MODEL_SIZE="${3:-n}"
INPUT_W="${4:-640}"
INPUT_H="${5:-480}"

if (( INPUT_W <= 0 || INPUT_H <= 0 || INPUT_W % 32 != 0 || INPUT_H % 32 != 0 )); then
  echo "Input width and height must be positive multiples of 32: ${INPUT_W}x${INPUT_H}" >&2
  exit 1
fi

[[ -f "${MODEL_PATH}" ]] || { echo "Missing weight file: ${MODEL_PATH}" >&2; exit 1; }
command -v git >/dev/null || { echo "git is required" >&2; exit 1; }
command -v cmake >/dev/null || { echo "cmake is required" >&2; exit 1; }
command -v nvcc >/dev/null || { echo "nvcc is required" >&2; exit 1; }

mkdir -p "${WORK_DIR}"

if [[ ! -d "${TENSORRTX_REPO}/.git" ]]; then
  git clone https://github.com/wang-xinyu/tensorrtx.git "${TENSORRTX_REPO}"
fi

NUM_CLASS="$(sed -n 's/.*kNumClass *= *\\([0-9][0-9]*\\).*/\\1/p' "${YOLO_PKG_DIR}/src/src/config.h" | head -1)"
NUM_CLASS="${NUM_CLASS:-1}"
sed -i "s/constexpr static int kNumClass = [0-9][0-9]*;/constexpr static int kNumClass = ${NUM_CLASS};/" \
  "${TENSORRTX_REPO}/yolov5/src/config.h"

case "${MODEL_PATH}" in
  *.wts)
    WTS_PATH="${MODEL_PATH}"
    ;;
  *.pt)
    if [[ ! -d "${YOLOV5_REPO}/.git" ]]; then
      git clone --branch v7.0 --depth 1 https://github.com/ultralytics/yolov5.git "${YOLOV5_REPO}"
    fi
    export PYTHONPATH="${YOLOV5_REPO}:${PYTHONPATH:-}"
    if [[ -f /lib/aarch64-linux-gnu/libgomp.so.1 ]]; then
      export LD_PRELOAD="/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:${LD_PRELOAD}}"
    fi
    WTS_PATH="${MODEL_PATH%.pt}_${INPUT_W}x${INPUT_H}.wts"
    PYTHON_BIN="${PYTHON_BIN:-python3}"
    "${PYTHON_BIN}" "${SCRIPT_DIR}/export_wts.py" "${MODEL_PATH}" -o "${WTS_PATH}"
    ;;
  *)
    echo "Weight file must end in .wts or .pt: ${MODEL_PATH}" >&2
    exit 1
    ;;
esac

SERIALIZER_SOURCE="${TENSORRTX_REPO}/yolov5"
if [[ -d "${SERIALIZER_SOURCE}/yolov5_trt10" ]]; then
  SERIALIZER_SOURCE="${SERIALIZER_SOURCE}/yolov5_trt10"
  TENSORRTX_BUILD="${SERIALIZER_SOURCE}/build"
fi

SERIALIZER_CONFIG="${SERIALIZER_SOURCE}/src/config.h"
sed -i "s/constexpr static int kInputW = [0-9][0-9]*;/constexpr static int kInputW = ${INPUT_W};/" \
  "${SERIALIZER_CONFIG}"
sed -i "s/constexpr static int kInputH = [0-9][0-9]*;/constexpr static int kInputH = ${INPUT_H};/" \
  "${SERIALIZER_CONFIG}"
sed -i "s/constexpr static int kNumClass = [0-9][0-9]*;/constexpr static int kNumClass = ${NUM_CLASS};/" \
  "${SERIALIZER_CONFIG}"

cmake -S "${SERIALIZER_SOURCE}" -B "${TENSORRTX_BUILD}"
cmake --build "${TENSORRTX_BUILD}" --parallel "$(nproc)"

mkdir -p "$(dirname "${ENGINE_PATH}")"
if [[ -x "${TENSORRTX_BUILD}/yolov5_det" ]]; then
  SERIALIZER="${TENSORRTX_BUILD}/yolov5_det"
else
  SERIALIZER="${TENSORRTX_BUILD}/yolov5"
fi

"${SERIALIZER}" -s "${WTS_PATH}" "${ENGINE_PATH}" "${MODEL_SIZE}"

echo
echo "Wrote ${ENGINE_PATH}"
echo "Input shape: 1x3x${INPUT_H}x${INPUT_W}"
