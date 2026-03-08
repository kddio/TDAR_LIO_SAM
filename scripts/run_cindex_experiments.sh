#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "Usage: $0 <input.bag> <gt_topic> <catkin_ws> [result_dir]"
  echo "Example: $0 ~/bags/high_dynamic.bag /ground_truth/odometry ~/catkin_ws ./results/cindex_$(date +%Y%m%d_%H%M%S)"
  exit 1
fi

INPUT_BAG="$1"
GT_TOPIC="$2"
CATKIN_WS="$3"
RESULT_ROOT="${4:-$(pwd)/results/cindex_$(date +%Y%m%d_%H%M%S)}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BASE_CONFIG="${PKG_ROOT}/config/params.yaml"
TDAR_DISABLED="${PKG_ROOT}/config/tdar/disabled.yaml"
CINDEX_DISABLED="${PKG_ROOT}/config/cindex/disabled.yaml"
CINDEX_ENABLED="${PKG_ROOT}/config/cindex/enabled.yaml"

if [[ ! -f "${INPUT_BAG}" ]]; then
  echo "Bag not found: ${INPUT_BAG}"
  exit 1
fi

if [[ ! -f "${CATKIN_WS}/devel/setup.bash" ]]; then
  echo "catkin workspace not ready: ${CATKIN_WS}/devel/setup.bash"
  exit 1
fi

source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
source "${CATKIN_WS}/devel/setup.bash"

mkdir -p "${RESULT_ROOT}"

run_case() {
  local case_name="$1"
  local cindex_cfg="$2"
  local case_dir="${RESULT_ROOT}/${case_name}"

  echo "[CINDEX] Running case: ${case_name}"
  mkdir -p "${case_dir}"

  local roscore_pid=""
  local launch_pid=""
  local record_pid=""

  cleanup_case() {
    [[ -n "${record_pid}" ]] && kill "${record_pid}" 2>/dev/null || true
    [[ -n "${launch_pid}" ]] && kill "${launch_pid}" 2>/dev/null || true
    [[ -n "${roscore_pid}" ]] && kill "${roscore_pid}" 2>/dev/null || true
    wait "${record_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
    wait "${roscore_pid}" 2>/dev/null || true
  }

  roscore >"${case_dir}/roscore.log" 2>&1 &
  roscore_pid=$!
  sleep 2

  rosparam set use_sim_time true

  roslaunch tdar_lio_sam run.launch \
    config_file:="${BASE_CONFIG}" \
    tdar_config:="${TDAR_DISABLED}" \
    cindex_config:="${cindex_cfg}" >"${case_dir}/launch.log" 2>&1 &
  launch_pid=$!
  sleep 8

  rosbag record -O "${case_dir}/output.bag" \
    /tdar_lio_sam/mapping/odometry \
    /tdar_lio_sam/mapping/path \
    /tdar_lio_sam/tdar/imu_preintegration_diag \
    /tdar_lio_sam/tdar/image_projection_diag \
    /tdar_lio_sam/cindex/search_diag \
    "${GT_TOPIC}" >"${case_dir}/record.log" 2>&1 &
  record_pid=$!
  sleep 2

  rosbag play "${INPUT_BAG}" --clock -r 1.0 >"${case_dir}/play.log" 2>&1
  sleep 5

  cleanup_case

  if command -v evo_ape >/dev/null 2>&1; then
    evo_ape bag "${case_dir}/output.bag" "${GT_TOPIC}" /tdar_lio_sam/mapping/odometry -va >"${case_dir}/ape.txt" 2>&1 || true
    evo_rpe bag "${case_dir}/output.bag" "${GT_TOPIC}" /tdar_lio_sam/mapping/odometry -va >"${case_dir}/rpe.txt" 2>&1 || true
  else
    echo "evo not found, skip ATE/RPE." >"${case_dir}/ape.txt"
    echo "evo not found, skip ATE/RPE." >"${case_dir}/rpe.txt"
  fi

  python3 "${PKG_ROOT}/scripts/export_tdar_diag_stats.py" \
    --bag "${case_dir}/output.bag" \
    --output "${case_dir}/tdar_diag_stats.json"

  python3 "${PKG_ROOT}/scripts/export_cindex_diag_stats.py" \
    --bag "${case_dir}/output.bag" \
    --output "${case_dir}/cindex_diag_stats.json"
}

run_case "baseline" "${CINDEX_DISABLED}"
run_case "cindex_enabled" "${CINDEX_ENABLED}"

python3 "${PKG_ROOT}/scripts/export_experiment_table.py" \
  --root "${RESULT_ROOT}" \
  --cases baseline cindex_enabled \
  --output_csv "${RESULT_ROOT}/summary.csv" \
  --output_md "${RESULT_ROOT}/summary.md"

echo "[CINDEX] Experiment done. Summary: ${RESULT_ROOT}/summary.md"
