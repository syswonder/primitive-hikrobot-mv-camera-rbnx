#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -eo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
source "${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
cd "$PKG"
CODEGEN="$PKG/rbnx-build/codegen"
export PYTHONPATH="$(rbnx path robonix-api):$PKG:$CODEGEN/proto_gen:$CODEGEN/robonix_mcp_types:${PYTHONPATH:-}"
exec python3 -m mv_camera.main
