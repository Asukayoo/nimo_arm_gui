#!/usr/bin/env bash
# Sync robot-side sources → remote nimo_robot (NOT nimo_arm_gui).
# Local HMI repo may still be named nimo_arm_gui; remote install path is decoupled.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REMOTE_HOST="${REMOTE_HOST:-nimo-012}"
REMOTE_DIR="${REMOTE_DIR:-/home/nimo/V1_Hardware_Control/nimo_robot}"
PASSWORD="${NIMO_SSH_PASS:-123}"

echo "Sync robot $ROOT -> ${REMOTE_HOST}:${REMOTE_DIR}"
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_HOST" "mkdir -p '$REMOTE_DIR'"
sshpass -p "$PASSWORD" rsync -az --delete \
  --exclude '.git' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '*/__pycache__/' \
  --exclude '*.pyc' \
  --exclude 'src/nimo_arm_gui/' \
  --exclude 'src/nimo_arm_description/' \
  --exclude 'src/deploy/' \
  --exclude 'canvases/' \
  --exclude 'deploy/run_gui.sh' \
  --exclude 'deploy/build_local.sh' \
  --exclude 'deploy/pull_backend.sh' \
  "$ROOT/" "${REMOTE_HOST}:${REMOTE_DIR}/"

sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_HOST" bash -s <<EOF
set -euo pipefail
cd '$REMOTE_DIR'
rm -rf src/nimo_arm_gui src/nimo_arm_description src/deploy
rm -f deploy/run_gui.sh deploy/build_local.sh deploy/pull_backend.sh \
      deploy/run_backend.sh deploy/run_hmi.sh deploy/run_pico_arms.sh
# drop legacy path if still present
if [[ -d /home/nimo/V1_Hardware_Control/nimo_arm_gui ]]; then
  echo "removing legacy /home/nimo/V1_Hardware_Control/nimo_arm_gui"
  rm -rf /home/nimo/V1_Hardware_Control/nimo_arm_gui
fi
find src deploy -type f -exec touch -d 'now' {} + 2>/dev/null || true
echo "remote src:"; ls -la src
echo "remote deploy:"; ls -la deploy
EOF
echo "Done."
