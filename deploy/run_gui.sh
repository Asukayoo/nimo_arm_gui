#!/usr/bin/env bash
# Start PyQt HMI + RViz on this PC. Backend must already be running on nimo.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DOMAIN="${ROS_DOMAIN_ID:-0}"
# WiFi AP often blocks multicast; use FastDDS unicast peer to Jetson.
PEER="${NIMO_DDS_PEER:-192.168.0.12}"
DDS_XML="$ROOT/deploy/dds/fastdds_unicast_gui.xml"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -f "$DDS_XML" ]]; then
  # Keep XML in sync with PEER (rewrite address if env overrides).
  sed "s#<address>[^<]*</address>#<address>${PEER}</address>#" "$DDS_XML" > /tmp/fastdds_unicast_gui.xml
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_unicast_gui.xml
fi
set -u

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID peer=$PEER (must match nimo backend)"
ros2 launch nimo_arm_bringup gui.launch.py "$@"
