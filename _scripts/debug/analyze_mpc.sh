#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(realpath "${SCRIPT_DIR}/..")"
CONTROLLER=""
CSV_FILE=""
BAG_FILE=""
PARAM_FILE=""
LOG_DIR=""
ONLY="all"

usage() {
  cat <<'EOF'
用法：_scripts/analyze_mpc.sh --controller isb_nmpc [选项]

选项：
  --csv FILE          指定控制器 CSV；缺省时使用日志目录中的最新文件
  --bag FILE          指定 SO3 轨迹图使用的 rosbag
  --param-file FILE   指定控制器参数 YAML
  --log-dir DIR       指定 CSV 日志目录
  --only TYPE         all、costs、trajectory 或 tuning
  -h, --help          显示帮助
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --controller) CONTROLLER="$2"; shift 2 ;;
    --csv) CSV_FILE="$2"; shift 2 ;;
    --bag) BAG_FILE="$2"; shift 2 ;;
    --param-file) PARAM_FILE="$2"; shift 2 ;;
    --log-dir) LOG_DIR="$2"; shift 2 ;;
    --only) ONLY="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ "${CONTROLLER}" != "isb_nmpc" ]; then
  echo "--controller 必须是 isb_nmpc" >&2
  exit 2
fi
case "${ONLY}" in all|costs|trajectory|tuning) ;; *) echo "无效 --only：${ONLY}" >&2; exit 2 ;; esac

latest_file() {
  local directory="$1"
  local pattern="$2"
  find "${directory}" -maxdepth 1 -type f -name "${pattern}" -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr | head -n 1 | cut -d' ' -f2-
}

LOG_DIR="${LOG_DIR:-${WORKSPACE}/log/interception}"
PARAM_FILE="${PARAM_FILE:-${WORKSPACE}/src/isb_nmpc/config/holydrone/simulation/isb_nmpc_sim_outdoor_5ms_0degree.yaml}"
if [ -z "${CSV_FILE}" ]; then CSV_FILE="$(latest_file "${LOG_DIR}" 'isb_nmpc_*.csv')"; fi

  if [ "${ONLY}" = all ] || [ "${ONLY}" = costs ]; then
    if [ -z "${CSV_FILE}" ]; then echo "未找到 ISB-NMPC CSV" >&2; exit 1; fi
    python3 "${WORKSPACE}/src/debug/analyze_mpc/analyze_isb_nmpc_costs.py" \
      "${CSV_FILE}" --param-file "${PARAM_FILE}"
  fi
  if [ "${ONLY}" = all ] || [ "${ONLY}" = tuning ]; then
    if [ -z "${CSV_FILE}" ]; then echo "未找到 ISB-NMPC CSV" >&2; exit 1; fi
    python3 "${WORKSPACE}/src/debug/analyze_mpc/analyze_isb_nmpc_tuning.py" \
      "${CSV_FILE}" --param-file "${PARAM_FILE}"
  fi
  if [ "${ONLY}" = all ] || [ "${ONLY}" = trajectory ]; then
    if [ -z "${BAG_FILE}" ]; then
      BAG_FILE="$(find "${WORKSPACE}/rosbag" -type f -name 'interception_*.bag' -printf '%T@ %p\n' 2>/dev/null \
        | sort -nr | head -n 1 | cut -d' ' -f2-)"
    fi
    if [ -n "${BAG_FILE}" ]; then
      TRAJ_DIR="$(dirname "${BAG_FILE}")/traj_plots"
      if ! python3 "${WORKSPACE}/src/debug/analyze_interception/interception_sim_traj_plot.py" \
        "${BAG_FILE}" --output-dir "${TRAJ_DIR}"; then
        echo "警告：bag 缺少目标轨迹话题，已跳过轨迹图；CSV 分析仍已完成。" >&2
      fi
    elif [ "${ONLY}" = trajectory ]; then
      echo "SO3 轨迹分析需要 --bag" >&2; exit 1
    else
      echo "未找到 rosbag，跳过 SO3 轨迹图。" >&2
    fi
fi
