#!/usr/bin/env bash
# Build GUI-side packages on this PC (ROS 2 Jazzy). Does NOT build nimo_arm_ros.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon build --symlink-install --packages-select \
  nimo_arm_msgs nimo_arm_description nimo_arm_gui nimo_arm_bringup

echo "Local GUI packages built. source $ROOT/install/setup.bash"
