#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# MCP types for the snapshot tool. The ROS message types (sensor_msgs) are the
# stock ROS 2 packages from the runtime image.
rbnx codegen -p "$PKG" --mcp
python3 -m compileall -q "$PKG/mv_camera"
