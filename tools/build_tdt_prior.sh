#!/usr/bin/env bash
# Build a Z-Localization prior bundle from the unchanged OriginSim arena PCD.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
input="${1:-$repo_root/src/bringup/map/rmuc_2026.pcd}"
output="${2:-$repo_root/src/bringup/map/tdt_prior}"

if [[ -e "$output" ]]; then
  echo "Output exists: $output (choose an empty path)" >&2
  exit 1
fi

ros2 run backend build_pcd_map_bundle \
  --input "$input" --out-bundle "$output" \
  --x-min 0 --x-max 29 --y-min 0 --y-max 16 \
  --sensor-z 0.88 --bundle-step 1.5 \
  --map-voxel 0.15 --scan-voxel 0.4 \
  --descriptor-type scan_context --sc-lidar-height 0.88
