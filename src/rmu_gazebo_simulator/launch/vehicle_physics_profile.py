"""Apply robot-only physical properties; do not alter any geometry or field."""
import math
import xml.etree.ElementTree as ET


def validate_profile(profile):
    required = {'chassis_mass', 'chassis_com_z', 'chassis_inertia',
                'reference_link_mass', 'upper_assembly_mass', 'chassis_friction',
                'chassis_angular_decay', 'chassis_linear_decay'}
    allowed = required | {'name', 'chassis_com_x', 'wheelbase',
                          'body_angular_damping', 'wheel_friction'}
    if required - profile.keys() or profile.keys() - allowed:
        raise ValueError('Incomplete or unknown vehicle physics profile fields')
    for key in required - {'chassis_inertia'} | {'chassis_com_x', 'wheelbase'} & profile.keys():
        if not math.isfinite(float(profile[key])):
            raise ValueError('Vehicle physics profile must contain finite values')
    for key in ['chassis_mass', 'reference_link_mass', 'upper_assembly_mass']:
        if profile[key] <= 0:
            raise ValueError('Vehicle masses must be positive')
    inertia = profile['chassis_inertia']
    if len(inertia) != 3 or any(not math.isfinite(x) or x <= 0 for x in inertia):
        raise ValueError('Chassis inertia must contain three finite positive moments')
    if 2 * max(inertia) > sum(inertia):
        raise ValueError('Principal inertia moments violate the triangle inequality')
    for key in ['chassis_friction', 'chassis_angular_decay', 'chassis_linear_decay']:
        if profile[key] < 0:
            raise ValueError('Friction and decay must be nonnegative')
    if 'wheelbase' in profile and profile['wheelbase'] <= 0:
        raise ValueError('Wheelbase must be positive')
    damping = profile.get('body_angular_damping', [0, 0, 0])
    if len(damping) != 3 or any(not math.isfinite(x) or x < 0 or x > 2000 for x in damping):
        raise ValueError('Passive angular damping must be in [0, 2000] Nms/rad')
    if 'wheel_friction' in profile:
        if set(profile['wheel_friction']) != {'front', 'rear'} or any(
                not math.isfinite(x) or x < 0 for x in profile['wheel_friction'].values()):
            raise ValueError('Front and rear wheel friction must be finite and nonnegative')

def set_text(parent, path, value):
    node = parent
    for name in path.split('/'):
        child = node.find(name)
        if child is None:
            child = ET.SubElement(node, name)
        node = child
    node.text = str(value)

def apply_profile(doc, p):
    validate_profile(p)
    model = doc.find('model')
    links = {x.get('name'): x for x in model.findall('link')}
    if 'wheelbase' in p:
        for joint in model.findall('joint'):
            if joint.get('name', '').endswith('_mount_joint'):
                node = joint.find('pose')
                values = node.text.split()
                values[0] = str((1 if joint.get('name').startswith('front_') else -1) * p['wheelbase'] / 2)
                node.text = ' '.join(values)
        for plugin in model.findall('plugin'):
            if plugin.get('filename') == 'SwerveDrive':
                set_text(plugin, 'wheelbase', p['wheelbase'])
    if 'body_angular_damping' in p:
        for plugin in model.findall('plugin'):
            if plugin.get('filename') == 'SwerveDrive':
                set_text(plugin, 'body_angular_damping', ' '.join(map(str, p['body_angular_damping'])))
    for name in ['base_footprint', 'base_link']:
        link = links[name]
        set_text(link, 'inertial/pose', '0 0 0 0 0 0')
        set_text(link, 'inertial/mass', p['reference_link_mass'])
        for tag in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']:
            set_text(link, 'inertial/inertia/' + tag, 1e-06 if tag in ['ixx', 'iyy', 'izz'] else 0)
    link = links['chassis']
    set_text(link, 'inertial/mass', p['chassis_mass'])
    set_text(link, 'inertial/pose', f"{p.get('chassis_com_x', 0)} 0 {p['chassis_com_z']} 0 0 0")
    for tag, value in zip(['ixx', 'iyy', 'izz'], p['chassis_inertia']):
        set_text(link, 'inertial/inertia/' + tag, value)
    for tag in ['ixy', 'ixz', 'iyz']:
        set_text(link, 'inertial/inertia/' + tag, 0)
    set_text(link, 'velocity_decay/linear', p['chassis_linear_decay'])
    set_text(link, 'velocity_decay/angular', p['chassis_angular_decay'])
    for collision in link.findall('collision'):
        for tag in ['mu', 'mu2']:
            set_text(collision, 'surface/friction/ode/' + tag, p['chassis_friction'])
        set_text(collision, 'surface/bounce/restitution_coefficient', 0)
    for name, link in links.items():
        if name.endswith('_wheel') and 'wheel_friction' in p:
            value = p['wheel_friction']['front' if name.startswith('front_') else 'rear']
            for collision in link.findall('collision'):
                for tag in ['mu', 'mu2']:
                    set_text(collision, 'surface/friction/ode/' + tag, value)
                for tag in ['friction', 'friction2']:
                    set_text(collision, 'surface/friction/bullet/' + tag, value)
    upper = links['aft_mapped']
    old = float(upper.findtext('inertial/mass'))
    factor = p['upper_assembly_mass'] / old
    set_text(upper, 'inertial/mass', p['upper_assembly_mass'])
    for tag in ['ixx', 'iyy', 'izz']:
        set_text(upper, 'inertial/inertia/' + tag, float(upper.findtext('inertial/inertia/' + tag)) * factor)
