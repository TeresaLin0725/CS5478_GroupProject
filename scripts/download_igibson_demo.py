"""Download official public Rs/demo robot assets, safely into project data/."""
import argparse
from pathlib import Path
import tarfile
import urllib.request


def download(root, name):
    archive = root / name
    root.mkdir(parents=True, exist_ok=True)
    if not archive.exists():
        url = 'https://storage.googleapis.com/gibson_scenes/' + name
        print('Downloading', url, flush=True)
        partial = archive.with_suffix(archive.suffix + '.part')
        with urllib.request.urlopen(url, timeout=60) as response, partial.open('wb') as output:
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                output.write(chunk)
        partial.replace(archive)
    destination = root / ('g_dataset' if name == 'Rs.tar.gz' else 'robot_assets')
    destination.mkdir(exist_ok=True)
    destination = destination.resolve()
    with tarfile.open(archive) as stream:
        for member in stream.getmembers():
            target = (destination / member.name).resolve()
            if destination != target and destination not in target.parents:
                raise ValueError('Unsafe archive path')
            if member.issym() or member.islnk() or not (member.isfile() or member.isdir()):
                raise ValueError('Unsupported archive member')
        stream.extractall(destination)
    print('Extracted', destination, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('data/igibson-data'))
    args = parser.parse_args()
    download(args.root, 'Rs.tar.gz')
    download(args.root, 'assets_igibson.tar.gz')
