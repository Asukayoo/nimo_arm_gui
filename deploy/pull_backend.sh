#!/usr/bin/env bash
# Pull backend sources FROM nimo-12 (msgs + ros only). Does NOT overwrite local GUI.
# Opposite of sync.sh (which pushes local → remote).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REMOTE_HOST="${REMOTE_HOST:-nimo-012}"
REMOTE_DIR="${REMOTE_DIR:-/home/nimo/V1_Hardware_Control/nimo_robot}"
PASSWORD="${NIMO_SSH_PASS:-123}"

echo "Pull backend ${REMOTE_HOST}:${REMOTE_DIR} -> $ROOT (msgs + ros)"
sshpass -p "$PASSWORD" rsync -az \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '*/__pycache__/' \
  --exclude '*.pyc' \
  "${REMOTE_HOST}:${REMOTE_DIR}/src/nimo_arm_msgs/" \
  "$ROOT/src/nimo_arm_msgs/"

sshpass -p "$PASSWORD" rsync -az \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '*/__pycache__/' \
  --exclude '*.pyc' \
  "${REMOTE_HOST}:${REMOTE_DIR}/src/nimo_arm_ros/" \
  "$ROOT/src/nimo_arm_ros/"

echo "Done. Pulled nimo_arm_msgs + nimo_arm_ros."
