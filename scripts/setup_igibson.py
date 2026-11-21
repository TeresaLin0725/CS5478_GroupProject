"""Build iGibson 2.2.2 from source on Windows into data/, with two MSVC fixes.

Run with any Python from the repository root:  python scripts/setup_igibson.py
Requires git, uv, Visual Studio C++ build tools and network access. Idempotent.

Patches applied to the upstream checkout (both are upstream bugs, not tuning):
1. Bundled Crypto++ calls stdext::make_(un)checked_array_iterator, removed from
   the modern MSVC STL; use the portable plain-pointer branch instead.
2. MeshRendererContext::loadTexture decodes textures as 3-channel RGB but
   resized them with the file's channel count (4 for RGBA PNGs), reading past
   the buffer and crashing when a robot with RGBA textures loads.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'data'
SOURCE = DATA / 'igibson-src'
VENV = DATA / 'venv-igibson'
PYTHON = VENV / 'Scripts' / 'python.exe'
COMMIT = '7d9598278b95ec1039e1f4366c66b45346521e44'  # iGibson 2.2.2

PATCHES = [
    ('igibson/render/cryptopp/integer.cpp',
     '#if (_MSC_VER >= 1500)\n\t\tstd::reverse_copy(encodedInteger, encodedInteger+byteCount,\n\t\t\tstdext::',
     '#if 0 // stdext checked iterators removed from modern MSVC STL\n\t\tstd::reverse_copy(encodedInteger, encodedInteger+byteCount,\n\t\t\tstdext::'),
    ('igibson/render/cryptopp/zdeflate.cpp',
     '#if _MSC_VER >= 1600\n\t\t\t\t(stdext::make_unchecked_array_iterator',
     '#if 0 // stdext unchecked iterators removed from modern MSVC STL\n\t\t\t\t(stdext::make_unchecked_array_iterator'),
    ('igibson/render/cpp/mesh_renderer.cpp',
     'unsigned char *resized_image = (unsigned char *)malloc(w*h*comp);\n'
     '    stbir_resize_uint8(image, w, h, 0, resized_image, new_w, new_h, 0, comp);',
     '// Decoded with STBI_rgb: always 3 channels, whatever the file stores (comp).\n'
     '    unsigned char *resized_image = (unsigned char *)malloc(new_w*new_h*3);\n'
     '    stbir_resize_uint8(image, w, h, 0, resized_image, new_w, new_h, 0, 3);'),
]


def run(*command, **kwargs):
    print('>', ' '.join(str(c) for c in command), flush=True)
    subprocess.check_call([str(c) for c in command], **kwargs)


def patch(relative, old, new):
    path = SOURCE / relative
    raw = path.read_bytes().decode('utf-8', errors='surrogateescape')
    crlf = '\r\n' in raw
    text = raw.replace('\r\n', '\n')
    if new in text:
        return
    if text.count(old) != 1:
        raise RuntimeError('Unexpected upstream source in ' + relative)
    text = text.replace(old, new)
    path.write_bytes((text.replace('\n', '\r\n') if crlf else text).encode('utf-8', errors='surrogateescape'))
    print('patched', relative)


def main():
    if os.name != 'nt':
        raise RuntimeError('This source-build helper is for Windows; see Documentation/running.md for Linux.')
    DATA.mkdir(exist_ok=True)
    os.environ.setdefault('UV_CACHE_DIR', str(DATA / 'uv-cache'))
    if not SOURCE.exists():
        run('git', 'clone', 'https://github.com/StanfordVL/iGibson.git', SOURCE)
        run('git', '-C', SOURCE, 'checkout', COMMIT)
        run('git', '-C', SOURCE, 'submodule', 'update', '--init', '--recursive')
    revision = subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD'], encoding='utf-8').strip()
    if revision != COMMIT:
        raise RuntimeError('Existing iGibson source is not the pinned revision; preserve it and use a fresh data/igibson-src.')
    if not PYTHON.exists():
        run('uv', 'venv', '--python', '3.8.20', VENV)
    run('uv', 'pip', 'install', '--python', PYTHON, '-r', ROOT / 'requirements-igibson.txt',
        '-c', ROOT / 'constraints-windows-py38.txt')
    for relative, old, new in PATCHES:
        patch(relative, old, new)
    renderer = SOURCE / 'igibson' / 'render' / 'mesh_renderer'
    if not list(renderer.glob('VRRendererContext*.pyd')):
        env = dict(os.environ, CMAKE_POLICY_VERSION_MINIMUM='3.5',
                   PATH=str(PYTHON.parent) + os.pathsep + os.environ['PATH'])
        # setup.py copies OpenVR from a fixed path that CMake's FetchContent does
        # not populate; the first build fetches it, then the copy is supplied.
        build = subprocess.call([str(PYTHON), 'setup.py', 'build_ext', '--inplace'], cwd=SOURCE, env=env)
        fetched = next(SOURCE.glob('build/*/Release/_deps/openvr-src/bin/win64/openvr_api.dll'), None)
        if fetched is None:
            raise RuntimeError('Renderer build failed before OpenVR was fetched')
        expected = SOURCE / 'igibson' / 'render' / 'openvr' / 'bin' / 'win64'
        expected.mkdir(parents=True, exist_ok=True)
        shutil.copy(fetched, expected)
        if build:
            run(PYTHON, 'setup.py', 'build_ext', '--inplace', cwd=SOURCE, env=env)
    (VENV / 'Lib' / 'site-packages' / 'igibson-src.pth').write_text(str(SOURCE) + '\n', encoding='utf-8')
    run(PYTHON, ROOT / 'scripts' / 'download_igibson_demo.py', '--root', DATA / 'igibson-data', cwd=ROOT)
    print('iGibson ready:', PYTHON)


if __name__ == '__main__':
    sys.exit(main())
