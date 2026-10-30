#!/usr/bin/env bash
# Thoroughly stop nimo backend inside teleop on nimo-012 and clear DDS leftovers.
#
#   bash deploy/stop_backend.sh            # INT→TERM→KILL + wipe FastDDS SHM
#   bash deploy/stop_backend.sh --restart  # also docker restart teleop (reaps zombies)
set -uo pipefail

REMOTE_HOST="${REMOTE_HOST:-nimo-012}"
PASSWORD="${NIMO_SSH_PASS:-123}"
RESTART=0
for a in "$@"; do
  [[ "$a" == "--restart" ]] && RESTART=1
done

echo "Stop backend on ${REMOTE_HOST} (restart_teleop=${RESTART})"

sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_HOST" \
  env RESTART="$RESTART" bash -s <<'REMOTE'
set -uo pipefail

echo "== before =="
docker exec teleop ps -eo pid,ppid,stat,etime,cmd 2>/dev/null \
  | grep -E 'dual_arm_node|nimo_arm_bringup backend|ros2-daemon' \
  | grep -v grep || echo "(none)"

# Kill by exact cmdline match inside container (one docker exec).
docker exec teleop bash -lc '
set +e
kill_pat() {
  sig="$1"; pat="$2"
  pgrep -af "$pat" 2>/dev/null | while read -r line; do
    pid="${line%% *}"
    # skip our helper
    case "$line" in *pgrep*|*kill_pat*|*bash -lc*) continue ;; esac
    echo "  kill -$sig $pid  ${line#"$pid "}"
    kill -"$sig" "$pid" 2>/dev/null
  done
}

echo "== SIGINT =="
kill_pat INT "nimo_arm_bringup backend.launch"
kill_pat INT "/lib/nimo_arm_ros/dual_arm_node"
sleep 3

echo "== SIGTERM =="
kill_pat TERM "nimo_arm_bringup backend.launch"
kill_pat TERM "/lib/nimo_arm_ros/dual_arm_node"
kill_pat TERM "ros2-daemon --ros-domain-id 0"
sleep 2

echo "== SIGKILL =="
kill_pat 9 "nimo_arm_bringup backend.launch"
kill_pat 9 "/lib/nimo_arm_ros/dual_arm_node"
kill_pat 9 "ros2-daemon --ros-domain-id 0"
# wrapper bash that only hosts the launch
kill_pat 9 "backend.launch.py use_sim"
sleep 1

echo "== wipe FastDDS SHM =="
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* /dev/shm/fast_rtps* 2>/dev/null
ls /dev/shm 2>/dev/null | grep -iE "fast|dds|fastrtps" || echo "(no fastdds shm)"
'

if [[ "${RESTART:-0}" == "1" ]]; then
  echo "== docker restart teleop =="
  docker restart teleop
  sleep 5
fi

echo "== after =="
docker exec teleop ps -eo pid,ppid,stat,etime,cmd 2>/dev/null \
  | grep -E 'dual_arm_node|nimo_arm_bringup backend|ros2-daemon' \
  | grep -v grep || echo "(clean)"
if docker exec teleop ps -eo stat,cmd 2>/dev/null | grep -q '^Z.*dual_arm'; then
  echo "WARN: dual_arm zombie remains → re-run with --restart"
fi
echo "Done."
REMOTE
