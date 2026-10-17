"""Reproducible furnishings for the real Gibson Rs scan, imported by iGibson.

URDF furniture has separate collision parts: chair legs, cushions, back slats,
cart wheels and shelves. These supplement the existing textured scan.
"""
from math import pi
from pathlib import Path
from xml.sax.saxutils import escape

PROFILES = {
    'apartment': {'title': '原始扫描公寓', 'description': '四个房间、门洞、家具遮挡；起点看不到厨房目标。', 'props': ()},
    'clutter': {'title': '拥挤公寓 · 困难', 'description': '新增椅子和软凳收窄客厅通道，绕家具进入厨房，再靠近台面目标。',
                'props': (('ottoman', (.25, -1.10, 0), 0), ('chair', (-.80, .70, 0), .35),
                          ('chair', (-2.70, 1.0, 0), -pi / 2))},
    'dynamic': {'title': '临时障碍 · 高难度', 'description': '拥挤布局中推车横穿客厅通道；深度更新触发重规划，目标需近距离确认。',
                'props': (('ottoman', (.25, -1.10, 0), 0), ('chair', (-2.70, 1.0, 0), -pi / 2),
                          ('cart', (1.40, -.10, 0), 0))},
}


def furniture_urdf(kind):
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
            cylinder('bottle%d' % i, (x, 0, .665), .046, .20, '.57 .68 .46')
    else:
        raise ValueError('Unknown furniture: ' + kind)
    return ('<?xml version="1.0"?><robot name="%s"><link name="root"><inertial><mass value="0"/>'
            '<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>%s</robot>') % (escape(kind), ''.join(parts))


def import_furniture(world, profile, directory):
    from igibson.objects.articulated_object import ArticulatedObject
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    records = []
    for index, (kind, position, yaw) in enumerate(PROFILES[profile]['props']):
        filename = directory / (kind + '.urdf')
        filename.write_text(furniture_urdf(kind), encoding='utf-8')
        obj = ArticulatedObject(str(filename), name='challenge_%s_%d' % (kind, index),
                                rendering_params={'use_pbr': False, 'use_pbr_mapping': False})
        world.sim.import_object(obj)
        obj.set_position_orientation(position, world.p.getQuaternionFromEuler((0, 0, yaw)))
        for body in obj.get_body_ids():
            world.p.changeDynamics(body, -1, mass=0)
        records.append({'kind': kind, 'position': list(position), 'yaw': yaw, 'object': obj})
    return records
