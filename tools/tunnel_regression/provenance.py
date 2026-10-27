"""Source/install provenance checks. No ROS imports are needed for planning."""

import hashlib
import json
from pathlib import Path
import subprocess


NAV_ROOT = Path(__file__).resolve().parents[2]
BASELINES = {
    'nav': '38702f56af3f4a8c1ecc21c077f5f7392a2e0cb1',
    'gazebo': '3c52828f203ae5d1d4c344f46f492366ef76e68c',
}
SCENE_FILES = (
    'resource/worlds/rmuc_2026_world.sdf',
    'resource/models/rmuc_2026/model.sdf',
    'resource/models/rmuc_2026/meshes/rmuc_2026.stl',
    'resource/models/speed_bump/model.sdf',
    'resource/models/speed_bump/meshes/speed_bump.stl',
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, data):
    """Replace only our named report atomically; reject NaN/Infinity."""
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    temporary.replace(path)


def git_state(root):
    def git(*args):
        result = subprocess.run(['git', '-C', str(root), *args],
                                capture_output=True, text=True, check=False,
                                timeout=15)
        return result.stdout.strip() if result.returncode == 0 else None
    return {'head': git('rev-parse', 'HEAD'),
            'branch': git('symbolic-ref', '--quiet', '--short', 'HEAD'),
            'dirty': git('status', '--porcelain', '--untracked-files=normal')}


def source_manifest(nav_root, gazebo_root):
    """Refuse altered accepted inputs; record hooks and implementations separately.

    Historical validation is a baseline for immutable inputs, not evidence that
    this invocation passed. Its private artifact paths are never opened.
    """
    nav_root, gazebo_root = Path(nav_root).resolve(), Path(gazebo_root).resolve()
    historical = json.loads((nav_root / 'docs/no_smoother_vehicle_validation.json')
                            .read_text(encoding='utf-8'))
    simulator = gazebo_root / 'src/rmu_gazebo_simulator'
    protected = {}

    def check(label, path, expected):
        actual = sha256(path)
        protected[label] = {'sha256': actual, 'expected_sha256': expected,
                            'matches': actual == expected}
        if actual != expected:
            raise ValueError('Accepted input changed: ' + str(path))

    check('nav/map_pgm', nav_root / 'src/bringup/map/rmuc_2026.pgm',
          historical['original_pgm_sha256'])
    check('nav/map_pcd', nav_root / 'src/bringup/map/rmuc_2026.pcd',
          historical['original_pcd_sha256'])
    for rel in SCENE_FILES:
        matches = [value for key, value in
                   historical['active_scene_resources_unchanged'].items()
                   if key.endswith('/' + rel)]
        if len(matches) != 1:
            raise ValueError('Ambiguous historical scene entry: ' + rel)
        check('gazebo/' + rel, simulator / rel, matches[0])
    for branch, root, rel in (
        ('nav', nav_root, 'src/bringup/params/no_smoother_tunnels.yaml'),
        ('nav', nav_root, 'src/bringup/launch/no_smoother_tunnels.launch.py'),
        ('nav', nav_root, 'src/bringup/map/rmuc_2026.yaml'),
        ('gazebo', gazebo_root,
         'src/rmu_gazebo_simulator/config/tunnel_vehicle_physics.json'),
        ('gazebo', gazebo_root,
         'src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro'),
    ):
        key = 'originsim-' + branch + '/' + rel
        check(branch + '/' + rel, root / rel, historical['source_sha256'][key])

    # Launch/config/model/Python inputs, including test hooks, must match the
    # sourced install. This does not by itself prove C++ build provenance.
    installed = []
    def remember(package, relative, path, location='share'):
        installed.append({'package': package, 'relative': relative,
                          'location': location, 'sha256': sha256(path)})
    for rel in ('map/rmuc_2026.pgm', 'map/rmuc_2026.pcd', 'map/rmuc_2026.yaml',
                'params/no_smoother_tunnels.yaml', 'launch/no_smoother_tunnels.launch.py'):
        remember('bringup', rel, nav_root / 'src/bringup' / rel)
    for rel in (*SCENE_FILES, 'config/tunnel_vehicle_physics.json',
                'config/ros_gz_bridge.yaml', 'config/base_params.yaml',
                'launch/tunnel_sim.launch.py', 'launch/spawn_robots.launch.py',
                'launch/vehicle_physics_profile.py', 'launch/regression_contacts.py'):
        remember('rmu_gazebo_simulator', rel, simulator / rel)
    description = gazebo_root / 'src/pb2025_robot_description'
    # Included XMacros define wheels, collisions and sensors as well as top SDF.
    for path in sorted((description / 'resource').rglob('*')):
        if path.is_file() and path.suffix in ('.xmacro', '.sdf', '.config'):
            remember('pb2025_robot_description',
                     path.relative_to(description).as_posix(), path)
    resources = gazebo_root / 'src/rmoss_gz_resources'
    for path in sorted((resources / 'resource').rglob('*')):
        if path.is_file() and path.suffix in ('.xmacro', '.sdf'):
            remember('rmoss_gz_resources', path.relative_to(resources).as_posix(), path)
    for name in ('wheel_lidar_odometry.py', 'wheel_lidar_math.py', 'landmark_alignment.py'):
        remember('landmark_alignment', name,
                 nav_root / 'src/localization/landmark_alignment' / name, 'lib')
    cpp_sources = {}
    for branch, root, directories in (
        ('nav', nav_root, ('src/nav_executor', 'src/map_server',
                          'src/navigation/minco_controller', 'src/navigation/fake_baselink',
                          'src/navigation/topology_global_planner',
                          'src/navigation/minimum_jerk_smoother')),
        ('gazebo', gazebo_root, ('src/rmoss_gazebo/rmoss_gz_plugins',
                                'src/rmoss_gazebo/rmoss_gz_base',
                                'src/ign_sim_pointcloud_tool')),
    ):
        for relative in directories:
            for path in sorted((root / relative).rglob('*')):
                if path.is_file() and path.suffix in ('.cpp', '.cc', '.hpp', '.hh', '.h'):
                    cpp_sources[branch + '/' + path.relative_to(root).as_posix()] = sha256(path)
    return {
        'baseline_commits': BASELINES,
        'nav': git_state(nav_root), 'gazebo': git_state(gazebo_root),
        'protected_inputs': protected, 'installed_files_expected': installed,
        'historical_results_sha256': sha256(nav_root / 'docs/no_smoother_vehicle_results.json'),
        'cases_sha256': sha256(Path(__file__).with_name('cases.json')),
        'tool_checkout': git_state(NAV_ROOT),
        'tool_source_sha256': {
            path.relative_to(NAV_ROOT).as_posix(): sha256(path)
            for path in sorted(Path(__file__).parent.rglob('*'))
            if path.is_file() and path.suffix in ('.py', '.json')
        },
        'cpp_source_sha256': cpp_sources,
        'verification_scope': 'Launch/config/model/Python source-install byte matching; '
                              'C++ source and binary fingerprints are provenance, not proof of a clean build.',
    }


def verify_install(manifest):
    """Called on a sourced ROS host, before launching or publishing a goal."""
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    actual = []
    for item in manifest['installed_files_expected']:
        package = item['package']
        base = (Path(get_package_share_directory(package)) if item['location'] == 'share'
                else Path(get_package_prefix(package)) / 'lib' / package)
        path = base / item['relative']
        digest = sha256(path)
        if digest != item['sha256']:
            raise ValueError('Installed file differs from selected checkout; rebuild/source overlay: '
                             + str(path))
        actual.append({**item, 'resolved_path': str(path)})
    return actual


def binary_fingerprints():
    """Record installed C++ artifacts without claiming they were rebuilt here."""
    from ament_index_python.packages import get_package_prefix
    paths = {}
    for package in ('nav_executor', 'map_server', 'minco_controller', 'fake_baselink',
                    'topology_global_planner', 'minimum_jerk_smoother',
                    'rmoss_gz_plugins', 'rmoss_gz_base', 'ign_sim_pointcloud_tool'):
        prefix = Path(get_package_prefix(package))
        candidates = list((prefix / 'lib' / package).glob('*'))
        candidates += list((prefix / 'lib').glob('*.so*'))
        candidates += list((prefix / 'plugins').glob('*.so*'))
        for path in candidates:
            if path.is_file():
                resolved = path.resolve()
                if resolved not in paths:
                    paths[resolved] = {'path': str(path), 'resolved_path': str(resolved),
                                       'sha256': sha256(path)}
    return list(paths.values())
