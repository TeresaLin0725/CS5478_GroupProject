"""Profile fusion on a frozen RGB-D sequence and save exact per-frame evidence."""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from frontier_search.fixtures.sequence import load_sequence
from frontier_search.perception.profiling import run_diagnostic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True, help='Package containing sequence.json')
    parser.add_argument('--output', type=Path, required=True, help='New directory; never overwritten')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--warmups', type=int, default=1, help='Complete unmeasured sequence passes')
    parser.add_argument('--max-frames', type=int, help='Use the first N frames, preserving their history')
    parser.add_argument('--baseline', type=Path, help='Previous baseline.json.gz with identical inputs/config')
    args = parser.parse_args()
    report, equal = run_diagnostic(load_sequence(args.dataset), args.output,
                                   args.repeats, args.warmups, args.max_frames, args.baseline)
    summary = report['unprofiled_summary']
    print('Fusion median %.6fs; P95 %.6fs (%d frame measurements)' % (
        summary['median_s'], summary['p95_s'], summary['count']))
    print('Profiler wall-time ratio: %.2fx (do not use profiled times as native latency)' %
          report['profiler_wall_ratio'])
    stages = report['profiled_frames']
    total = sum(frame['instrumented_total_s'] for frame in stages)
    for name in stages[0]['stages']:
        seconds = sum(frame['stages'][name] for frame in stages)
        print('  %s: %.6fs/frame, %.1f%% of profiled time' % (
            name, seconds / len(stages), 100. * seconds / total if total else 0.))
    print('Frame-by-frame evidence checks: %s' % ('PASS' if equal else 'FAIL'))
    print('Report: %s' % (args.output / 'report.json'))
    return 0 if equal else 1


if __name__ == '__main__':
    sys.exit(main())
