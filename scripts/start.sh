#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Start the atlas bridge. NO ROS spawn here — ranger_bringup is launched
# by atlas_bridge inside Driver(CMD_INIT).
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$PKG/rbnx-build/ws/install/setup.bash"
else
    echo "[ranger_chassis/start] ERROR: rbnx-build/ws/install missing — run rbnx build first" >&2
    exit 1
fi

export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
if ROBONIX_PY="$(rbnx path robonix-py 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_PY:$PYTHONPATH"
fi

exec python3 -m ranger_chassis.atlas_bridge
