#!/usr/bin/env bash

# Recover interrupted ROS1 recordings (*.bag.active) without modifying the
# source until the recovered bag has passed `rosbag info`.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BAG_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)/rosbag"

dry_run=0
keep_active=0
quiet=0
work_dir=""
work_bag=""

usage() {
  cat <<EOF
用法: $(basename -- "$0") [选项] [文件或目录...]

恢复 ROS1 意外中断后留下的 *.bag.active。未指定路径时搜索:
  ${DEFAULT_BAG_ROOT}

选项:
  -n, --dry-run      只列出待处理文件，不执行恢复
  -k, --keep-active  成功后保留原始 .bag.active 文件
  -q, --quiet        隐藏 rosbag reindex 的进度输出
  -h, --help         显示帮助

示例:
  $(basename -- "$0")
  $(basename -- "$0") --dry-run rosbag/real_flight
  $(basename -- "$0") --keep-active path/to/run.bag.active
EOF
}

die() {
  echo "错误: $*" >&2
  exit 1
}

cleanup() {
  # work_dir is always created by mktemp below and never supplied by the user.
  if [[ -n "${work_dir}" && -d "${work_dir}" ]]; then
    [[ -z "${work_bag}" ]] || rm -f -- "${work_bag}"
    rmdir -- "${work_dir}" 2>/dev/null || true
  fi
}

on_signal() {
  echo >&2
  echo "已中断；临时文件已清理，原始 .bag.active 保持不变。" >&2
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM HUP

declare -a search_paths=()
while (($#)); do
  case "$1" in
    -n | --dry-run)
      dry_run=1
      ;;
    -k | --keep-active)
      keep_active=1
      ;;
    -q | --quiet)
      quiet=1
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    --)
      shift
      search_paths+=("$@")
      break
      ;;
    -*)
      die "未知选项: $1（使用 --help 查看帮助）"
      ;;
    *)
      search_paths+=("$1")
      ;;
  esac
  shift
done

((${#search_paths[@]} > 0)) || search_paths=("${DEFAULT_BAG_ROOT}")

declare -A seen=()
declare -a active_bags=()
add_bag() {
  local bag="$1"
  local absolute_bag
  absolute_bag="$(readlink -f -- "${bag}")" || return 0
  if [[ -z "${seen["${absolute_bag}"]+x}" ]]; then
    seen["${absolute_bag}"]=1
    active_bags+=("${absolute_bag}")
  fi
}

for path in "${search_paths[@]}"; do
  [[ -e "${path}" ]] || die "路径不存在: ${path}"
  if [[ -d "${path}" ]]; then
    while IFS= read -r -d '' bag; do
      add_bag "${bag}"
    done < <(find "${path}" -type f -name '*.bag.active' -print0)
  elif [[ "${path}" == *.bag.active ]]; then
    add_bag "${path}"
  else
    die "文件不是 *.bag.active: ${path}"
  fi
done

if ((${#active_bags[@]} == 0)); then
  echo "未找到需要恢复的 *.bag.active 文件。"
  exit 0
fi

# Stable output order, including paths containing whitespace.
mapfile -d '' -t active_bags < <(printf '%s\0' "${active_bags[@]}" | sort -z)

echo "找到 ${#active_bags[@]} 个待恢复文件:"
printf '  %s\n' "${active_bags[@]}"
if ((dry_run)); then
  echo "预览完成，未修改任何文件。"
  exit 0
fi

if ! command -v rosbag >/dev/null 2>&1; then
  if [[ -r /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
  fi
fi
command -v rosbag >/dev/null 2>&1 || die "未找到 rosbag，请先安装或加载 ROS1 环境"

recovered=0
skipped=0
failed=0

for active_bag in "${active_bags[@]}"; do
  recovered_bag="${active_bag%.active}"
  bag_dir="$(dirname -- "${active_bag}")"
  work_dir=""
  work_bag=""

  echo
  echo "处理: ${active_bag}"

  if [[ -e "${recovered_bag}" ]]; then
    echo "跳过: 目标已存在: ${recovered_bag}" >&2
    skipped=$((skipped + 1))
    continue
  fi

  # Never touch a recording that may still be receiving data.
  if command -v fuser >/dev/null 2>&1; then
    if fuser "${active_bag}" >/dev/null 2>&1; then
      echo "跳过: 文件仍被进程占用，请先停止 rosbag record" >&2
      skipped=$((skipped + 1))
      continue
    fi
  elif command -v lsof >/dev/null 2>&1 \
    && lsof -- "${active_bag}" >/dev/null 2>&1; then
    echo "跳过: 文件仍被进程占用，请先停止 rosbag record" >&2
    skipped=$((skipped + 1))
    continue
  fi

  # rosbag reindex --output-dir first copies ROS bag v2 files, so ensure the
  # destination filesystem can hold another complete copy.
  bag_size="$(stat -c '%s' -- "${active_bag}")"
  available_kib="$(df -Pk -- "${bag_dir}" | awk 'END {print $4}')"
  required_kib=$(((bag_size + 1023) / 1024 + 16384))
  if ((available_kib < required_kib)); then
    echo "失败: 可用空间不足（需要约 ${required_kib} KiB，可用 ${available_kib} KiB）" >&2
    failed=$((failed + 1))
    continue
  fi

  work_dir="$(mktemp -d "${bag_dir}/.bag_recover.XXXXXX")"
  work_bag="${work_dir}/$(basename -- "${active_bag}")"
  reindex_args=(reindex --output-dir "${work_dir}")
  ((quiet)) && reindex_args+=(--quiet)
  reindex_args+=("${active_bag}")

  if rosbag "${reindex_args[@]}" \
    && [[ -s "${work_bag}" ]] \
    && rosbag info "${work_bag}" >/dev/null; then
    mv -- "${work_bag}" "${recovered_bag}"
    if ((keep_active)); then
      echo "成功: ${recovered_bag}（已保留原始 .active）"
    else
      rm -- "${active_bag}"
      echo "成功: ${recovered_bag}（验证通过，原始 .active 已删除）"
    fi
    rmdir -- "${work_dir}" 2>/dev/null || true
    work_dir=""
    work_bag=""
    recovered=$((recovered + 1))
  else
    echo "失败: 恢复结果未通过验证，原始 .active 已保留" >&2
    cleanup
    work_dir=""
    work_bag=""
    failed=$((failed + 1))
  fi
done

echo
echo "处理完成: 共 ${#active_bags[@]} 个，成功 ${recovered} 个，跳过 ${skipped} 个，失败 ${failed} 个。"
((failed == 0))
