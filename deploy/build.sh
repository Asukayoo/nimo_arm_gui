#!/usr/bin/env bash
# Build robot packages inside teleop container under /home/mundane/nimo_robot.
set -euo pipefail
REMOTE_HOST="${REMOTE_HOST:-nimo-012}"
PASSWORD="${NIMO_SSH_PASS:-123}"
V1_ROOT="/home/nimo/V1_Hardware_Control"

sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_HOST" bash -s <<EOF
set -euo pipefail
cd '$V1_ROOT'
if [[ ! -f nimo_arm/build/libnimo_arm.a ]]; then
  echo "Building nimo_arm library first..."
  bash deploy/exec.sh 'cmake -S /home/mundane/nimo_arm -B /home/mundane/nimo_arm/build -DCMAKE_BUILD_TYPE=Release && cmake --build /home/mundane/nimo_arm/build -j\$(nproc)'
fi
bash deploy/ensure_rt.sh || true
bash deploy/exec.sh '
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
cd /home/mundane/nimo_robot
export NIMO_ARM_ROOT=/home/mundane/nimo_arm
colcon build --symlink-install --packages-select \
  nimo_arm_msgs nimo_arm_ros nimo_arm_bringup \
  --cmake-args -DNIMO_ARM_ROOT=/home/mundane/nimo_arm
BIN=build/nimo_arm_ros/dual_arm_node
if [[ -x "\$BIN" ]]; then
  if command -v sudo >/dev/null 2>&1; then
    sudo setcap cap_sys_nice,cap_ipc_lock+ep "\$BIN" || true
  else
    setcap cap_sys_nice,cap_ipc_lock+ep "\$BIN" || true
  fi
  getcap "\$BIN" || true
fi
'
EOF
echo "Build finished (nimo_robot)."
