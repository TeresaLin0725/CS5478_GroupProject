"""Reproducible furnishings for the real Gibson Rs scan, imported by iGibson.

URDF furniture has separate collision parts: chair legs, cushions, back slats,
cart wheels and shelves. These supplement the existing textured scan.
"""
from math import pi
from pathlib import Path
from xml.sax.saxutils import escape
import xml.etree.ElementTree as ET
import hashlib
import os

PROFILES = {
    'apartment': {'title': 'Original scanned apartment', 'description': 'Four rooms, doorways, furniture occlusion; the kitchen target is invisible from the start. ', 'props': ()},
    'clutter': {'title': 'Cluttered apartment · Difficult', 'description': 'Added chairs and ottomans narrow the living-room passage, navigate around furniture into the kitchen, then approach the countertop target. ',
                'props': (('ottoman', (.25, -1.10, 0), 0), ('chair', (-.80, .70, 0), .35),
                          ('chair', (-2.70, 1.0, 0), -pi / 2))},
    'dynamic': {'title': 'Temporary obstacles · Very difficult', 'description': 'A cart crosses the living-room passage in the cluttered layout; depth updates trigger replanning, the target requires close-range verification. ',
                'props': (('ottoman', (.25, -1.10, 0), 0), ('chair', (-2.70, 1.0, 0), -pi / 2),
                          ('cart', (1.40, -.10, 0), 0))},
}


def furniture_urdf(kind, bottle_mesh=None):
    parts = []
    def part(name, xyz, geometry, color, rpy=(0, 0, 0)):
        position = ' '.join(str(v) for v in xyz)
        rotation = ' '.join(str(v) for v in rpy)
        parts.append('<link name="%s"><inertial><mass value="0"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>'
                     '<visual><geometry>%s</geometry><material name="%s"><color rgba="%s 1"/></material></visual>'
                     '<collision><geometry>%s</geometry></collision></link>'
                     '<joint name="joint_%s" type="fixed"><parent link="root"/><child link="%s"/><origin xyz="%s" rpy="%s"/></joint>'
                     % (name, geometry, name, color, geometry, name, name, position, rotation))
    def box(name, xyz, size, color):
        part(name, xyz, '<box size="%s"/>' % ' '.join(str(v) for v in size), color)
    def cylinder(name, xyz, radius, length, color, rpy=(0, 0, 0)):
        part(name, xyz, '<cylinder radius="%s" length="%s"/>' % (radius, length), color, rpy)
    wood, fabric, steel = '.42 .24 .12', '.24 .40 .43', '.16 .19 .21'
    if kind == 'chair':
        box('seat', (0, 0, .44), (.44, .44, .08), fabric)
        for i, (x, y) in enumerate(((-.17, -.17), (-.17, .17), (.17, -.17), (.17, .17))):
            cylinder('leg%d' % i, (x, y, .21), .023, .42, wood)
        for i, y in enumerate((-.18, .18)):
            cylinder('back_post%d' % i, (-.19, y, .69), .023, .50, wood)
        for i, z in enumerate((.60, .73, .86)):
            box('back_slat%d' % i, (-.19, 0, z), (.035, .39, .075), wood)
    elif kind == 'ottoman':
        cylinder('upholstery', (0, 0, .25), .25, .35, '.56 .38 .23')
        cylinder('cushion', (0, 0, .44), .255, .065, '.64 .46 .30')
        for i, (x, y) in enumerate(((-.15, -.15), (-.15, .15), (.15, -.15), (.15, .15))):
            cylinder('foot%d' % i, (x, y, .04), .035, .08, wood)
    elif kind == 'cart':
        for i, z in enumerate((.19, .55)):
            box('shelf%d' % i, (0, 0, z), (.48, .40, .035), wood)
        for i, (x, y) in enumerate(((-.21, -.16), (-.21, .16), (.21, -.16), (.21, .16))):
            cylinder('upright%d' % i, (x, y, .35), .015, .54, steel)
            cylinder('wheel%d' % i, (x, y, .055), .055, .032, '.08 .08 .09', (pi / 2, 0, 0))
        box('handle', (-.21, 0, .65), (.035, .40, .035), steel)
        for i, x in enumerate((-.09, .09)):
            # A straight, uncapped cylinder looks like a tall cup to RGB-only
            # detection. Model the declared bottle with a narrow neck and cap;
            # keep its complete geometry inside the cart's existing envelope.
            cylinder('bottle%d' % i, (x, 0, .635), .046, .13, '.57 .68 .46')
            cylinder('bottle_label%d' % i, (x, 0, .635), .047, .05, '.85 .88 .78')
            cylinder('bottle_neck%d' % i, (x, 0, .720), .016, .04, '.57 .68 .46')
            cylinder('bottle_cap%d' % i, (x, 0, .7475), .019, .015, '.12 .22 .34')
    else:
        raise ValueError('Unknown furniture: ' + kind)
    urdf = ('<?xml version="1.0"?><robot name="%s"><link name="root"><inertial><mass value="0"/>'
            '<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>%s</robot>') % (escape(kind), ''.join(parts))
    return textured_cart_bottles(urdf, bottle_mesh) if kind == 'cart' and bottle_mesh else urdf


def textured_cart_bottles(urdf, mesh):
    """Use the installed YCB bottle appearance inside the original cart envelope.

    Only visual geometry changes. All analytic collision parts stay intact.
    Normalization uses this asset's own vertices, never scene or target truth.
    """
    mesh = Path(mesh).resolve()
    vertices = [tuple(map(float, line.split()[1:4])) for line in mesh.read_text(encoding='utf-8').splitlines()
                if line.startswith('v ')]
    if not vertices:
        raise ValueError('Bottle visual mesh has no vertices: ' + str(mesh))
    low = [min(v[i] for v in vertices) for i in range(3)]
    high = [max(v[i] for v in vertices) for i in range(3)]
    scale = min(.18/(high[2]-low[2]), .09/max(high[i]-low[i] for i in (0,1)))
    centre = [(a+b)/2 for a,b in zip(low,high)]
    tree = ET.fromstring(urdf)
    for link in tree.findall('link'):
        if not link.attrib['name'].startswith('bottle'):
            continue
        for visual in link.findall('visual'):
            link.remove(visual)
        if link.attrib['name'] in ('bottle0', 'bottle1'):
            visual = ET.SubElement(link, 'visual')
            # Body link sits at z=.635; normalized mesh spans z=.57..=.75.
            ET.SubElement(visual, 'origin', xyz='%.9f %.9f %.9f' % (
                -centre[0]*scale, -centre[1]*scale, .025-centre[2]*scale))
            ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh',
                          filename=mesh.as_posix(), scale=' '.join([str(scale)]*3))
    # Bullet's bundled XML reader fails on this long, compact serialized
    # document even though ElementTree accepts it. Keep element lines short.
    return ET.tostring(tree, encoding='unicode').replace('><', '>\n<')


def profile_props(profile,scene_layout='legacy'):
    """The 48-case layout keeps the bathroom route open for every target task."""
    if scene_layout not in ('legacy','bowl48_accessible'):
        raise ValueError('Unknown scene layout: '+scene_layout)
    props=PROFILES[profile]['props']
    if scene_layout=='bowl48_accessible':
        return tuple((kind,(-3.35,.85,0) if kind=='chair' and position==(-2.70,1.0,0) else position,yaw)
                     for kind,position,yaw in props)
    return props


def import_furniture(world, profile, directory,scene_layout='legacy'):
    import igibson
    from igibson.objects.articulated_object import ArticulatedObject
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    records = []
    for index, (kind, position, yaw) in enumerate(profile_props(profile,scene_layout)):
        bottle_mesh = (Path(igibson.assets_path)/'models/ycb/006_mustard_bottle/textured_simple.obj'
                       if kind == 'cart' else None)
        urdf = furniture_urdf(kind, bottle_mesh)
        # Isolate simultaneous test/episode initialization, avoiding rewrites
        # of a shared generated resource while another process imports it.
        digest = hashlib.sha256(urdf.encode('utf-8')).hexdigest()[:12]
        filename = directory / ('%s-%d-%s.urdf' % (kind, os.getpid(), digest))
        if not filename.exists():
            filename.write_text(urdf, encoding='utf-8')
        obj = ArticulatedObject(str(filename), name='challenge_%s_%d' % (kind, index),
                                rendering_params={'use_pbr': False, 'use_pbr_mapping': False})
        world.sim.import_object(obj)
        obj.set_position_orientation(position, world.p.getQuaternionFromEuler((0, 0, yaw)))
        for body in obj.get_body_ids():
            world.p.changeDynamics(body, -1, mass=0)
        records.append({'kind': kind, 'position': list(position), 'yaw': yaw, 'object': obj})
    return records
