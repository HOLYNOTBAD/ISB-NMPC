#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(realpath "${SCRIPT_DIR}/..")"

cd "${WORKSPACE}"

source /opt/ros/noetic/setup.bash
if [ -f "${WORKSPACE}/devel/setup.bash" ]; then
  source "${WORKSPACE}/devel/setup.bash"
fi

# Rebuild the SO3 ACADO code generator and refresh generated solver code.
# Set ACADO_ENV_FILE=/path/to/acado_env.sh if ACADO is installed elsewhere.
# Set SKIP_ACADO_CODEGEN=1 to rebuild the ROS package using existing generated
# solver sources without requiring ACADO on this machine.
if [ "${SKIP_ACADO_CODEGEN:-0}" != "1" ]; then
  ACADO_ENV_FILE="${ACADO_ENV_FILE:-}"

  if [ -n "${ACADO_ENV_FILE}" ] && [ ! -f "${ACADO_ENV_FILE}" ]; then
    echo "Error: ACADO_ENV_FILE does not exist: ${ACADO_ENV_FILE}" >&2
    exit 1
  fi

  if [ -z "${ACADO_ENV_FILE}" ]; then
    candidate_roots=()
    if [ -n "${ACADO_ROOT:-}" ]; then
      candidate_roots+=("${ACADO_ROOT}")
    fi
    candidate_roots+=(
      "${HOME}/acado"
      "${HOME}/ACADOtoolkit"
      "${HOME}/acado_toolkit"
      "${WORKSPACE}/../acado"
      "${WORKSPACE}/third_party/acado"
      /opt/acado
      /tmp/acado
    )

    for root in "${candidate_roots[@]}"; do
      for candidate in "${root}/build/acado_env.sh" "${root}/acado_env.sh"; do
        if [ -f "${candidate}" ]; then
          ACADO_ENV_FILE="${candidate}"
          break 2
        fi
      done
    done
  fi

  if [ -z "${ACADO_ENV_FILE}" ]; then
    for search_root in "${HOME}" "${WORKSPACE}/.."; do
      if [ -d "${search_root}" ]; then
        ACADO_ENV_FILE="$(find "${search_root}" -maxdepth 4 -path '*/build/acado_env.sh' -type f -print -quit 2>/dev/null || true)"
        if [ -n "${ACADO_ENV_FILE}" ]; then
          break
        fi
      fi
    done
  fi

  if [ -n "${ACADO_ENV_FILE}" ]; then
    echo "Using ACADO environment: ${ACADO_ENV_FILE}"
    source "${ACADO_ENV_FILE}"
  fi

  if [ -z "${ACADO_ENV_INCLUDE_DIRS:-}" ] || \
     [ -z "${ACADO_ENV_LIBRARY_DIRS:-}" ] || \
     [ -z "${ACADO_ENV_SHARED_LIBRARIES:-}" ]; then
    echo "Error: cannot find ACADO environment." >&2
    echo "Install/build ACADO, then rerun with:" >&2
    echo "  ACADO_ENV_FILE=/path/to/acado/build/acado_env.sh ./_scripts/rebuild_isb_nmpc.sh" >&2
    echo "or, if ACADO is already built, use:" >&2
    echo "  ACADO_ROOT=/path/to/acado ./_scripts/rebuild_isb_nmpc.sh" >&2
    echo "To only rebuild the ROS package from existing generated solver code:" >&2
    echo "  SKIP_ACADO_CODEGEN=1 ./_scripts/rebuild_isb_nmpc.sh" >&2
    echo "Typical ACADO build:" >&2
    echo "  git clone https://github.com/acado/acado.git ~/acado" >&2
    echo "  cmake -S ~/acado -B ~/acado/build && cmake --build ~/acado/build -j\$(nproc)" >&2
    exit 1
  fi

  cd "${WORKSPACE}/src/isb_nmpc/model"
  rm -rf CMakeCache.txt CMakeFiles Makefile cmake_install.cmake
  cmake .
  make -j"$(nproc)"
  ./so3_quadrotor_model_codegen
else
  echo "Skipping ACADO code generation; rebuilding isb_nmpc with existing generated solver code."
fi

cd "${WORKSPACE}"

# If build/ was previously configured against a nested package CMakeLists.txt,
# catkin_make will reject the workspace source directory. Clear that cache while
# keeping devel/ intact so local package configs remain available.
if [ -f "${WORKSPACE}/build/CMakeCache.txt" ] &&
   ! grep -q "^CMAKE_HOME_DIRECTORY:INTERNAL=${WORKSPACE}/src$" "${WORKSPACE}/build/CMakeCache.txt"; then
  echo "Clearing stale workspace build cache in ${WORKSPACE}/build"
  rm -rf "${WORKSPACE}/build"
fi

catkin_make \
  -DCATKIN_WHITELIST_PACKAGES=isb_nmpc \
  -DCMAKE_PREFIX_PATH="${WORKSPACE}/devel;/opt/ros/noetic" \
  -j"$(nproc)"
