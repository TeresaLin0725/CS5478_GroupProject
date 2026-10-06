"""Run C's unified episode lifecycle with explicitly authored A test data."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

from ..experiments.logging import dumps
from ..fixtures.w2_episode import mock_manifest
from .manifest import load_manifest
from .supervisor import run_supervised


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('progress', 'blocked', 'no_progress', 'no_frontiers'), default='progress')
    parser.add_argument('--policy', choices=('nearest', 'random', 'information_gain', 'llm_guided'), default=None)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--write-manifest', type=Path)
    parser.add_argument('--log', type=Path)
    parser.add_argument('--factory', default='frontier_search.fixtures.w2_episode:build_mock_runner',
                        help='Importable module:function; real factory supplied by A/B integration')
    args = parser.parse_args()
    manifest = load_manifest(args.manifest) if args.manifest else mock_manifest(args.scenario)
    if args.policy:
        manifest = replace(manifest, policy_name=args.policy)
    manifest.validate()
    if args.write_manifest:
        args.write_manifest.parent.mkdir(parents=True, exist_ok=True)
        with args.write_manifest.open('x', encoding='utf-8') as output:
            json.dump(asdict(manifest), output, indent=2, allow_nan=False)
    if args.log is None:
        if args.write_manifest:
            return 0
        parser.error('--log is required to run an episode')
    summary = run_supervised(manifest, args.log, args.factory)
    print(dumps(summary))
    return 1 if summary['termination'] in ('pipeline_error', 'initialization_failed', 'evaluation_error') else 0


if __name__ == '__main__':
    raise SystemExit(main())
