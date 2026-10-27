"""Opt-in contact observation for the tunnel regression's generated robot SDF.

This adds observation systems and sensors only. It never edits a collision,
joint, mass, surface, vehicle profile, or source asset. The Contact system MUST
be loaded with the world before spawning the robot: a plugin first loaded with
the robot can miss EachNew<ContactSensor> on its first active simulation step.

The Contact system publishes nonempty contacts only. A silent topic is NOT
proof that no collision occurred. The regression runner must observe fresh
wheel support contacts before starting and enforce its contact freshness limit.

Sources: gazebosim/gz-sim, gz-sim8/src/systems/contact/Contact.cc and
gz-sim8/examples/worlds/contact_sensor.sdf. The explicit topic is a child of
<contact>, not <sensor>. No reliance is placed on contact update_rate throttling.
"""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


CONTACT_TOPIC = '/tunnel_regression/contacts'
SENSOR_NAME = 'tunnel_regression_contacts'
PLUGIN_NAME = 'gz::sim::systems::Contact'


def write_contact_world(source, destination):
    """Copy the world with one observation plugin; never overwrite source files.

    Removing that one plugin from the generated DOM must recover the complete
    original DOM. All geometry, includes, gravity, solver settings and surfaces
    are retained. Both files are available for the run's provenance audit.
    """
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source == destination:
        raise ValueError('Contact instrumentation must use a separate world copy')
    doc = ET.fromstring(source.read_bytes())
    world = doc.find('world')
    if world is None or world.get('name') != 'default':
        raise ValueError('Expected the original default world')
    if any(p.get('name') == PLUGIN_NAME or
           p.get('filename') == 'gz-sim-contact-system'
           for p in world.findall('plugin')):
        raise ValueError('Source world already contains a Contact system')
    ET.SubElement(world, 'plugin', {
        'filename': 'gz-sim-contact-system', 'name': PLUGIN_NAME,
    })
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open('xb') as stream:
        stream.write(ET.tostring(doc, encoding='utf-8', xml_declaration=True))
    return destination


def instrument_robot(doc, topic=CONTACT_TOPIC):
    """Append a contact sensor to every collision-bearing link and return names.

    The caller must opt in explicitly. Reject malformed and already instrumented
    input before making any mutation, so accidental double application cannot
    silently duplicate Gazebo systems or contact events.
    """
    model = doc.find('model')
    if model is None:
        raise ValueError('Expected one generated robot <model> inside <sdf>')
    if not re.fullmatch(r'/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*', topic):
        raise ValueError('Expected an absolute Gazebo contact topic')
    if any(p.get('name') == PLUGIN_NAME or
           p.get('filename') == 'gz-sim-contact-system'
           for p in model.findall('plugin')):
        raise ValueError('Generated robot already contains a Contact system')

    links = []
    observed = []
    for link in model.iter('link'):
        collisions = link.findall('collision')
        if not collisions:
            continue
        if link.find("sensor[@name='%s']" % SENSOR_NAME) is not None:
            raise ValueError('Generated robot is already instrumented')
        names = [c.get('name') for c in collisions]
        if not link.get('name') or any(not name for name in names):
            raise ValueError('Contact observation requires named links/collisions')
        if len(set(names)) != len(names):
            raise ValueError('Duplicate collision names in generated robot link')
        links.append((link, names))
        observed.extend('%s::%s' % (link.get('name'), name) for name in names)
    if not links:
        raise ValueError('Generated robot has no observable collision geometry')

    for link, names in links:
        sensor = ET.SubElement(link, 'sensor', {
            'name': SENSOR_NAME, 'type': 'contact',
        })
        ET.SubElement(sensor, 'always_on').text = 'true'
        contact = ET.SubElement(sensor, 'contact')
        for name in names:
            ET.SubElement(contact, 'collision').text = name
        ET.SubElement(contact, 'topic').text = topic
    return observed
