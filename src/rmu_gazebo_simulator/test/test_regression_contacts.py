"""ROS/Gazebo-independent tests for test-only SDF instrumentation invariants."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


HELPER = Path(__file__).resolve().parents[1] / 'launch/regression_contacts.py'
SPEC = importlib.util.spec_from_file_location('regression_contacts', HELPER)
contacts = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(contacts)

ROBOT = '''<sdf version="1.7"><model name="test_robot">
 <pose>11.1 1.025 1.09 0 0 0</pose>
 <plugin filename="SwerveDrive" name="drive"><wheelbase>0.66</wheelbase>
  <body_angular_damping>500 1500 2</body_angular_damping></plugin>
 <link name="chassis"><inertial><mass>15</mass><pose>0 0 .02 0 0 0</pose></inertial>
  <collision name="body"><pose>0 0 .12 0 0 0</pose><geometry><box><size>.5 .5 .025</size></box></geometry>
   <surface><friction><ode><mu>.15</mu></ode></friction></surface></collision>
  <sensor name="imu" type="imu"><update_rate>200</update_rate></sensor></link>
 <link name="front_left_wheel"><collision name="tire"><geometry><cylinder>
  <radius>.0758</radius><length>.024</length></cylinder></geometry></collision>
  <collision name="hub"><geometry><sphere><radius>.01</radius></sphere></geometry></collision></link>
 <link name="base_link"/>
 <joint name="mount" type="fixed"><pose>.33 .25 0 0 0 0</pose><parent>chassis</parent><child>front_left_wheel</child></joint>
</model></sdf>'''


class ContactInstrumentationTest(unittest.TestCase):
    def test_removing_observation_restores_every_original_element(self):
        doc = ET.fromstring(ROBOT)
        original = ET.tostring(doc)
        observed = contacts.instrument_robot(doc)
        self.assertEqual(observed, ['chassis::body', 'front_left_wheel::tire',
                                    'front_left_wheel::hub'])
        model = doc.find('model')
        self.assertIsNone(model.find("plugin[@filename='gz-sim-contact-system']"),
                          'Contact must be preloaded in world, before robot spawn')
        for link in model.iter('link'):
            sensor = link.find("sensor[@name='tunnel_regression_contacts']")
            if sensor is not None:
                self.assertEqual(sensor.findtext('contact/topic'), contacts.CONTACT_TOPIC)
                self.assertIsNone(sensor.find('topic'))
                self.assertEqual([c.text for c in sensor.findall('contact/collision')],
                                 [c.get('name') for c in link.findall('collision')])
                link.remove(sensor)
        self.assertEqual(ET.tostring(doc), original)

    def test_empty_geometry_fails_without_mutation(self):
        doc = ET.fromstring('<sdf><model name="r"><link name="a"/></model></sdf>')
        original = ET.tostring(doc)
        with self.assertRaises(ValueError):
            contacts.instrument_robot(doc)
        self.assertEqual(ET.tostring(doc), original)

    def test_double_instrumentation_fails_without_mutation(self):
        doc = ET.fromstring(ROBOT)
        contacts.instrument_robot(doc)
        original = ET.tostring(doc)
        with self.assertRaises(ValueError):
            contacts.instrument_robot(doc)
        self.assertEqual(ET.tostring(doc), original)

    def test_malformed_collision_and_topic_fail_without_mutation(self):
        for topic in ['relative', '/has a space', '', '/a;touch']:
            doc = ET.fromstring(ROBOT)
            original = ET.tostring(doc)
            with self.assertRaises(ValueError):
                contacts.instrument_robot(doc, topic)
            self.assertEqual(ET.tostring(doc), original)
        doc = ET.fromstring(ROBOT)
        del doc.find('model/link/collision').attrib['name']
        original = ET.tostring(doc)
        with self.assertRaises(ValueError):
            contacts.instrument_robot(doc)
        self.assertEqual(ET.tostring(doc), original)

    def test_world_copy_changes_only_contact_system(self):
        source = HELPER.parents[1] / 'resource/worlds/rmuc_2026_world.sdf'
        original = source.read_bytes()
        with tempfile.TemporaryDirectory() as folder:
            target = contacts.write_contact_world(source, Path(folder) / 'world.sdf')
            doc = ET.fromstring(target.read_bytes())
            world = doc.find('world')
            plugin = world.find("plugin[@filename='gz-sim-contact-system']")
            self.assertIsNotNone(plugin)
            world.remove(plugin)
            self.assertEqual(ET.tostring(doc), ET.tostring(ET.fromstring(original)))
            self.assertEqual(source.read_bytes(), original)
            with self.assertRaises(FileExistsError):
                contacts.write_contact_world(source, target)
            with self.assertRaises(ValueError):
                contacts.write_contact_world(source, source)


if __name__ == '__main__':
    unittest.main()
