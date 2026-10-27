"""Exercise source-workspace preparation without ROS or a second real checkout."""

import json
from pathlib import Path
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / 'build_tunnel_regression.sh'
NAV_PACKAGES = {
    'bringup': 'src/bringup',
    'map_server': 'src/map_server',
    'referee_msg': 'src/serial/referee_msg',
    'nav_executor': 'src/nav_executor',
    'fake_baselink': 'src/navigation/fake_baselink',
    'landmark_alignment': 'src/localization/landmark_alignment',
    'minco_controller': 'src/navigation/minco_controller',
    'topology_global_planner': 'src/navigation/topology_global_planner',
    'minimum_jerk_smoother': 'src/navigation/minimum_jerk_smoother',
}
GAZEBO_PACKAGES = {
    'rmoss_interfaces': 'src/rmoss_interfaces',
    'rmoss_gz_resources': 'src/rmoss_gz_resources',
    'rmoss_gz_plugins': 'src/rmoss_gazebo/rmoss_gz_plugins',
    'rmoss_gz_base': 'src/rmoss_gazebo/rmoss_gz_base',
    'sdformat_tools': 'src/sdformat_tools',
    'pb2025_robot_description': 'src/pb2025_robot_description',
    'ign_sim_pointcloud_tool': 'src/ign_sim_pointcloud_tool',
    'rmu_gazebo_simulator': 'src/rmu_gazebo_simulator',
}


class BuildHelperTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='OriginSim build test ')
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.nav = root / 'nav checkout'
        self.gazebo = root / 'gazebo checkout'
        self.workspace = root / 'build workspace'
        for checkout, packages in ((self.nav, NAV_PACKAGES),
                                   (self.gazebo, GAZEBO_PACKAGES)):
            for name, relative in packages.items():
                package = checkout / relative
                package.mkdir(parents=True)
                (package / 'package.xml').write_text(
                    '<package><name>' + name + '</name></package>', encoding='utf-8')
        for checkout, relative in (
            (self.nav, 'src/bringup/launch/no_smoother_tunnels.launch.py'),
            (self.gazebo, 'src/rmu_gazebo_simulator/launch/tunnel_sim.launch.py'),
        ):
            path = checkout / relative
            path.parent.mkdir(parents=True)
            path.write_text('# fixture only; not executed\n', encoding='utf-8')

    def prepare(self, workspace=None):
        return subprocess.run(
            ['bash', str(SCRIPT), '--nav-root', str(self.nav),
             '--gazebo-root', str(self.gazebo),
             '--workspace', str(workspace or self.workspace), '--prepare-only'],
            capture_output=True, text=True, timeout=40, check=False)

    def test_curated_packages_and_repeatable_paths_with_spaces(self):
        duplicate = self.gazebo / 'src/referee_system/referee_msg'
        duplicate.mkdir(parents=True)
        (duplicate / 'package.xml').write_text(
            '<package><name>referee_msg</name></package>', encoding='utf-8')
        first = self.prepare()
        self.assertEqual(first.returncode, 0, first.stderr)
        actual = {path.name: path for path in (self.workspace / 'src').iterdir()}
        self.assertEqual(set(actual), set(NAV_PACKAGES) | set(GAZEBO_PACKAGES))
        self.assertEqual(len(actual), 17)
        for checkout, packages in ((self.nav, NAV_PACKAGES),
                                   (self.gazebo, GAZEBO_PACKAGES)):
            for name, relative in packages.items():
                self.assertTrue(actual[name].is_symlink())
                self.assertEqual(actual[name].resolve(), checkout / relative)
        manifest = json.loads((self.workspace / 'source-manifest.json').read_text())
        self.assertEqual(set(manifest['packages']), set(actual))
        self.assertFalse(manifest['built'])
        self.assertFalse((self.workspace / 'install').exists())
        second = self.prepare()
        self.assertEqual(second.returncode, 0, second.stderr)

    def test_unrelated_workspace_entry_is_preserved_and_rejected(self):
        unrelated = self.workspace / 'src/user_package'
        unrelated.mkdir(parents=True)
        note = unrelated / 'keep.txt'
        note.write_text('developer work', encoding='utf-8')
        result = self.prepare()
        self.assertEqual(result.returncode, 2)
        self.assertIn('Unrelated workspace entry', result.stderr)
        self.assertEqual(note.read_text(), 'developer work')
        self.assertEqual(list((self.workspace / 'src').iterdir()), [unrelated])

    def test_conflicting_package_link_is_not_replaced(self):
        source = self.workspace.parent / 'another bringup'
        source.mkdir()
        src = self.workspace / 'src'
        src.mkdir(parents=True)
        conflict = src / 'bringup'
        conflict.symlink_to(source, target_is_directory=True)
        result = self.prepare()
        self.assertEqual(result.returncode, 2)
        self.assertIn('Refusing to replace', result.stderr)
        self.assertEqual(conflict.resolve(), source)

    def test_missing_source_does_not_create_partial_workspace(self):
        (self.nav / 'src/nav_executor/package.xml').unlink()
        result = self.prepare()
        self.assertEqual(result.returncode, 2)
        self.assertIn('Missing package', result.stderr)
        self.assertFalse(self.workspace.exists())

    def test_workspace_inside_source_checkout_is_rejected(self):
        for checkout in (self.nav, self.gazebo):
            destination = checkout / 'new-workspace'
            result = self.prepare(destination)
            self.assertEqual(result.returncode, 2)
            self.assertIn('dedicated workspace', result.stderr)
            self.assertFalse(destination.exists())


if __name__ == '__main__':
    unittest.main()
