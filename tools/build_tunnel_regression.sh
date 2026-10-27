#!/usr/bin/env bash
# Build the existing tunnel stack from two checkouts without duplicate packages.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: build_tunnel_regression.sh [options]

  --nav-root DIR       nav checkout (default: this script's parent directory)
  --gazebo-root DIR    gazebo checkout (default: sibling OriginSim-gazebo)
  --workspace DIR      dedicated build workspace (default: sibling OriginSim-tunnel-ws)
  --prepare-only       validate sources and create package links; no ROS required
  --test               also run tests in the seven relevant algorithm/plugin packages
  -h, --help           show this help

Source /opt/ros/jazzy/setup.bash before building. This script never installs
dependencies, edits source files, or replaces an existing unrelated directory.
See docs/tunnel_regression.md for dependency installation and execution commands.
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }
need_value() { [[ $# -ge 2 && -n $2 ]] || fail "Missing value for $1"; }

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
nav_root="$script_dir/.."
gazebo_root=''
workspace=''
prepare_only=false
run_tests=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --nav-root) need_value "$@"; nav_root=$2; shift 2 ;;
    --gazebo-root) need_value "$@"; gazebo_root=$2; shift 2 ;;
    --workspace) need_value "$@"; workspace=$2; shift 2 ;;
    --prepare-only) prepare_only=true; shift ;;
    --test) run_tests=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown argument: $1" ;;
  esac
done

command -v python3 >/dev/null || fail 'python3 is required'
[[ -d $nav_root ]] || fail "nav checkout does not exist: $nav_root"
nav_root=$(cd -- "$nav_root" && pwd -P)
gazebo_root=${gazebo_root:-"$nav_root/../OriginSim-gazebo"}
workspace=${workspace:-"$nav_root/../OriginSim-tunnel-ws"}
[[ -d $gazebo_root ]] || fail "gazebo checkout does not exist: $gazebo_root"
gazebo_root=$(cd -- "$gazebo_root" && pwd -P)
# Resolve a not-yet-created path without depending on GNU realpath.
workspace=$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve())' "$workspace")
[[ $nav_root != "$gazebo_root" ]] || fail 'The two branch checkouts must be separate'
case "$workspace/" in
  "$nav_root/"*|"$gazebo_root/"*)
    fail 'Use a dedicated workspace outside both source trees' ;;
esac

names=(
  bringup map_server referee_msg nav_executor fake_baselink landmark_alignment
  minco_controller topology_global_planner minimum_jerk_smoother
  rmoss_interfaces rmoss_gz_resources rmoss_gz_plugins rmoss_gz_base
  sdformat_tools pb2025_robot_description ign_sim_pointcloud_tool rmu_gazebo_simulator
)
sources=(
  "$nav_root/src/bringup"
  "$nav_root/src/map_server"
  "$nav_root/src/serial/referee_msg"
  "$nav_root/src/nav_executor"
  "$nav_root/src/navigation/fake_baselink"
  "$nav_root/src/localization/landmark_alignment"
  "$nav_root/src/navigation/minco_controller"
  "$nav_root/src/navigation/topology_global_planner"
  "$nav_root/src/navigation/minimum_jerk_smoother"
  "$gazebo_root/src/rmoss_interfaces"
  "$gazebo_root/src/rmoss_gz_resources"
  "$gazebo_root/src/rmoss_gazebo/rmoss_gz_plugins"
  "$gazebo_root/src/rmoss_gazebo/rmoss_gz_base"
  "$gazebo_root/src/sdformat_tools"
  "$gazebo_root/src/pb2025_robot_description"
  "$gazebo_root/src/ign_sim_pointcloud_tool"
  "$gazebo_root/src/rmu_gazebo_simulator"
)

# Validate every source before creating any links.
for i in "${!names[@]}"; do
  [[ -f ${sources[$i]}/package.xml ]] || fail "Missing package: ${sources[$i]}"
  actual_name=$(python3 -c 'import sys,xml.etree.ElementTree as E; print(E.parse(sys.argv[1]).findtext("name"))' "${sources[$i]}/package.xml")
  [[ $actual_name == "${names[$i]}" ]] || fail "Unexpected package name in ${sources[$i]}"
done
[[ -f $nav_root/src/bringup/launch/no_smoother_tunnels.launch.py ]] || fail 'nav tunnel launch is missing'
[[ -f $gazebo_root/src/rmu_gazebo_simulator/launch/tunnel_sim.launch.py ]] || fail 'Gazebo tunnel launch is missing'

# A previous invocation may be reused; an unrelated src entry is not absorbed.
if [[ -d $workspace/src ]]; then
  shopt -s nullglob dotglob
  for existing in "$workspace/src/"*; do
    matched=false
    for i in "${!names[@]}"; do
      if [[ ${existing##*/} == "${names[$i]}" ]]; then
        [[ -L $existing && $(readlink -- "$existing") == "${sources[$i]}" ]] ||
          fail "Refusing to replace existing workspace entry: $existing"
        matched=true
        break
      fi
    done
    $matched || fail "Unrelated workspace entry: $existing"
  done
  shopt -u nullglob dotglob
fi

mkdir -p -- "$workspace/src"
for i in "${!names[@]}"; do
  link="$workspace/src/${names[$i]}"
  [[ -L $link ]] || ln -s -- "${sources[$i]}" "$link"
done
python3 - "$workspace" "$nav_root" "$gazebo_root" "${names[@]}" <<'PY'
import json
from pathlib import Path
import subprocess
import sys

workspace, nav, gazebo, *packages = sys.argv[1:]
def revision(path):
    result = subprocess.run(['git', '-C', path, 'rev-parse', 'HEAD'],
                            capture_output=True, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else None
manifest = {
    'purpose': 'Dedicated OriginSim tunnel regression build workspace',
    'nav_root': nav, 'gazebo_root': gazebo,
    'nav_revision': revision(nav), 'gazebo_revision': revision(gazebo),
    'packages': packages,
    'built': False,
}
Path(workspace, 'source-manifest.json').write_text(
    json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
PY
printf 'Prepared %d source packages in %s\n' "${#names[@]}" "$workspace"
if $prepare_only; then
  printf 'No build was attempted. Install dependencies using docs/tunnel_regression.md.\n'
  exit 0
fi

[[ ${ROS_DISTRO:-} == jazzy ]] || fail 'Source /opt/ros/jazzy/setup.bash before building'
for executable in ros2 colcon cmake git; do
  command -v "$executable" >/dev/null || fail "Missing executable: $executable"
done
python3 - <<'PY' || fail 'Missing Python runtime dependencies; see docs/tunnel_regression.md'
import importlib
for name in ('rclpy', 'launch', 'launch_ros', 'yaml', 'numpy', 'scipy', 'xmacro'):
    importlib.import_module(name)
PY

cd -- "$workspace"
# Explicit package selection avoids optional LIO/front-end dependencies in bringup.
# Only the selected package directories are discoverable, so duplicate referee_msg
# packages from the two branches cannot silently change the message ABI.
colcon build --base-paths "$workspace/src" --symlink-install \
  --packages-select "${names[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

if $run_tests; then
  # Source the completed overlay before tests that import installed ROS interfaces.
  set +u
  source "$workspace/install/setup.bash"
  set -u
  colcon test --base-paths "$workspace/src" --return-code-on-test-failure \
    --packages-select map_server nav_executor minco_controller fake_baselink \
      landmark_alignment rmoss_gz_plugins rmu_gazebo_simulator \
    --event-handlers console_direct+
  colcon test-result --verbose
fi

python3 - "$workspace/source-manifest.json" "$run_tests" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
manifest = json.loads(path.read_text(encoding='utf-8'))
manifest['built'] = True
manifest['selected_package_tests_passed'] = sys.argv[2] == 'true'
path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
PY
printf 'Build complete. Source this overlay before regression runs:\nsource %q\n' "$workspace/install/setup.bash"
