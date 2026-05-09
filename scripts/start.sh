#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${COLCON_TRACE:=}"
export AMENT_TRACE_SETUP_FILES COLCON_TRACE
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$PKG/rbnx-build/ws/install/setup.bash"
fi

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
fi

exec python3 -m ranger_chassis.main
