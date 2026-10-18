"""Check local dependencies/assets without a model API call; optionally render Rs."""
import argparse
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'runs/environment.json')
    parser.add_argument('--render', action='store_true', help='Open native dynamic Rs, check cart sweep, render one image.')
    args = parser.parse_args()
    os.environ.setdefault('YOLO_CONFIG_DIR', str(ROOT / 'data/yolo-config'))
    Path(os.environ['YOLO_CONFIG_DIR']).mkdir(parents=True, exist_ok=True)
    report = dict(python=platform.python_version(), executable=sys.executable, system=platform.platform(),
                  checks={}, packages={}, render_requested=args.render)
    report['checks']['python_38'] = sys.version_info[:2] == (3, 8)
    expected = {'numpy': ('1.24.4',), 'torch': ('2.4.1+cu121', '2.4.1+cpu'),
        'torchvision': ('0.19.1+cu121', '0.19.1+cpu'), 'ultralytics': ('8.3.203',),
        'opencv-python': ('4.8.1.78',), 'Pillow': ('10.4.0',),
        'pybullet-svl': ('3.1.6.4',), 'imageio-ffmpeg': ('0.5.1',)}
    for name, module in (('numpy', 'numpy'), ('torch', 'torch'), ('torchvision', 'torchvision'),
                         ('ultralytics', 'ultralytics'), ('opencv-python', 'cv2'), ('Pillow', 'PIL'),
                         ('pybullet-svl', 'pybullet'), ('imageio-ffmpeg', 'imageio_ffmpeg')):
        try:
            report['packages'][name] = importlib.metadata.version(name)
            report['checks']['version_' + module] = report['packages'][name] in expected[name]
            importlib.import_module(module)
            report['checks']['import_' + module] = True
        except Exception as exc:
            report['checks']['import_' + module] = False
            report.setdefault('errors', {})[module] = type(exc).__name__ + ': ' + str(exc)
    from frontier_search.simulation.igibson_rs import configure_paths
    configure_paths()
    try:
        import igibson
        from igibson.render.mesh_renderer import mesh_renderer_cpu
        report['igibson_version'] = igibson.__version__
        report['checks']['igibson_222'] = igibson.__version__ == '2.2.2'
        report['igibson_path'] = igibson.__file__
        report['checks']['native_renderer_import'] = True
    except Exception as exc:
        report['checks']['native_renderer_import'] = False
        report.setdefault('errors', {})['renderer'] = type(exc).__name__ + ': ' + str(exc)
    assets = ROOT / 'data/igibson-data'
    for name, path in (
        ('scene', assets / 'g_dataset/Rs/mesh_z_up.obj'),
        ('scene_texture', assets / 'g_dataset/Rs/combined.jpg'),
        ('robot', assets / 'robot_assets/assets/models/locobot/locobot.urdf'),
        ('target', assets / 'robot_assets/assets/models/ycb/025_mug/textured_simple.obj'),
        ('cart_bottle', assets / 'robot_assets/assets/models/ycb/006_mustard_bottle/textured_simple.obj'),
        ('cart_bottle_texture', assets / 'robot_assets/assets/models/ycb/006_mustard_bottle/texture_map.png')):
        report['checks'][name] = path.is_file()
    model = ROOT / 'weights/yolo11/yolo11s.pt'
    report['model_sha256'] = hashlib.sha256(model.read_bytes()).hexdigest() if model.is_file() else None
    report['checks']['model_checksum'] = report['model_sha256'] == '85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5'
    if report['checks']['import_torch']:
        import torch
        report['detector'] = dict(cuda_available=torch.cuda.is_available(), cuda=torch.version.cuda,
            gpu=torch.cuda.get_device_name(0) if torch.cuda.is_available() else None)
    if args.render and all(report['checks'].values()):
        from frontier_search.simulation.igibson_rs import IGibsonWorld
        world = None
        try:
            world = IGibsonWorld(profile='dynamic', target_model='025_mug', target_position=(-1.4, 2.95, .9213))
            motion = world.cart_motion
            report['cart_route'] = dict(start=motion.start, end=motion.end,
                sweep_clear=motion.clear_sweep(motion.start, motion.end),
                floor_supported=motion.floor_supported(motion.start, motion.end))
            report['checks']['cart_route_verified'] = all(report['cart_route'][k] for k in ('sweep_clear', 'floor_supported'))
            from PIL import Image
            args.output.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(world.spectator()).save(args.output.with_suffix('.png'))
            report['checks']['native_render'] = True
        except Exception as exc:
            report['checks']['native_render'] = False
            report.setdefault('errors', {})['render'] = type(exc).__name__ + ': ' + str(exc)
        finally:
            if world is not None:
                world.close()
    report['passed'] = all(report['checks'].values())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=True, indent=2))
    return int(not report['passed'])


if __name__ == '__main__':
    raise SystemExit(main())
