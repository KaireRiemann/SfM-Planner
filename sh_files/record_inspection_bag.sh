#!/usr/bin/env bash
# Record the information needed to replay and diagnose one inspection mission.
#
# Run this in the ROS1 Noetic Docker container after the simulator/planner is
# running and before calling /inspection/start.  Stop it with Ctrl-C after the
# drone has returned home; rosbag will then close the bag cleanly.
#
# Usage:
#   ./sh_files/record_inspection_bag.sh [output_dir] [bag_prefix]
#
# Examples:
#   ./sh_files/record_inspection_bag.sh
#   ./sh_files/record_inspection_bag.sh /root/ws/SfM-Planner/bags cave1_inspection

set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'EOF'
Usage:
  ./sh_files/record_inspection_bag.sh [output_dir] [bag_prefix]

Examples:
  ./sh_files/record_inspection_bag.sh
  ./sh_files/record_inspection_bag.sh /root/ws/SfM-Planner/bags cave1_inspection
EOF
    exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"
output_dir="${1:-${workspace_dir}/bags}"
bag_prefix="${2:-inspection_mission}"
timestamp="$(date +%Y%m%d_%H%M%S)"
bag_base="${output_dir}/${bag_prefix}_${timestamp}"

source /opt/ros/noetic/setup.bash
if [[ -f "${workspace_dir}/devel/setup.bash" ]]; then
    source "${workspace_dir}/devel/setup.bash"
fi

if ! rostopic list >/dev/null 2>&1; then
    echo "ROS master is unavailable. Start roscore/the inspection launch first." >&2
    exit 1
fi

mkdir -p "${output_dir}"

# Keep raw perception data for detector replay, planning products for RViz,
# and the mission protocol for debugging recognition/capture transitions.
topics=(
    /tf
    /tf_static
    /rosout_agg
    /lidar_slam/odom
    /lidar_slam/pose
    /global_pc
    /cloud_registered
    /robot
    /perfect_drone/vel_text
    /planning/pos_cmd
    /planning_cmd/poly_traj
    /planning/diagnostics/events
    /fsm_node/fsm/path
    /fsm_node/rog_map/occ
    /fsm_node/rog_map/inf_occ
    /fsm_node/map_manager/topology
    /fsm_node/visualization/goal
    /fsm_node/visualization/exp_traj
    /fsm_node/visualization/backup_traj
    /fsm_node/visualization/receding_traj
    /fsm_node/visualization/yaw_traj
    /inspection/status
    /inspection/face/request
    /inspection/face/result
    /inspection/face/debug
    /inspection/capture/request
    /inspection/capture/result
)

echo "Recording inspection mission bag to: ${bag_base}.bag"
echo "Press Ctrl-C only after the mission has completed/returned home."
printf 'Topics:\n'
printf '  %s\n' "${topics[@]}"

# Split only when a run is unusually large; normal cave1 missions remain one
# bag file. LZ4 keeps PointCloud2 storage and write latency manageable.
exec rosbag record --lz4 --tcpnodelay --split --size=4096 -O "${bag_base}" "${topics[@]}"
