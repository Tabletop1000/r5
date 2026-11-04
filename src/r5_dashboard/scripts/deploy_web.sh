#!/usr/bin/env bash
# Builds the web app on the laptop and copies it to the robot, which has no Node.js 22.
# Usage: scripts/deploy_web.sh <robot>   for example: scripts/deploy_web.sh jdutra@r4
# Then on the robot: colcon build --packages-select r5_dashboard_server
set -euo pipefail

robot="${1:?Usage: $0 <robot>, for example jdutra@r4}"
# Where this repository is on the robot, relative to the robot user's home.
repo="${R5_ROBOT_REPO:-git/r5}"

cd "$(dirname "$0")/../web"
npm ci
npm run build
rsync -a --delete dist/ "$robot:$repo/src/r5_dashboard/web/dist/"
echo "Copied. On the robot, run: colcon build --packages-select r5_dashboard_server"
