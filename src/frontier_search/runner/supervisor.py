"""Spawn one simulator episode in a killable child; parent alone owns JSONL.

Hard termination is suitable for an in-process simulator. Physical hardware
would require an independent motor watchdog, outside this project's scope.
"""
import importlib
import hashlib
import multiprocessing
from pathlib import Path
import sys
from time import monotonic

from ..experiments.logging import JsonlLogger


class _PipeLogger:
    def __init__(self, pipe):
        self.pipe = pipe

    def write(self, event, episode_id, **payload):
        self.pipe.send(('event', event, episode_id, payload))


def _worker(pipe, factory_path, manifest):
    try:
        module, name = factory_path.split(':')
        factory = getattr(importlib.import_module(module), name)
        runner = factory(manifest, logger=_PipeLogger(pipe))
        runner.run(manifest)
        pipe.send(('result', runner.summary))
    except BaseException as exc:
        pipe.send(('error', type(exc).__name__))
    finally:
        pipe.close()


def run_supervised(manifest, log_path, factory_path='frontier_search.fixtures.w2_episode:build_mock_runner'):
    """Enforce a hard total deadline and every runner-reported module deadline.

    Factories must be top-level importable functions. No live renderer, lock,
    or API key is serialized from the parent. Child startup is also bounded.
    """
    manifest.validate()
    ctx = multiprocessing.get_context('spawn')
    receive, send = ctx.Pipe(duplex=False)
    process = ctx.Process(target=_worker, args=(send, factory_path, manifest))
    started = monotonic()
    pending = ('factory', started+manifest.module_deadline_s)
    summary = None
    partial = dict(decisions=0, attempts=0, steps=0, actual_distance_m=0., simulated_time_s=0.)
    reason, detail = 'pipeline_error', 'worker_exit_without_result'
    with JsonlLogger(log_path) as logger:
        digest = hashlib.sha256()
        source_root = Path(__file__).resolve().parents[1]
        for path in sorted(source_root.rglob('*.py')):
            digest.update(path.relative_to(source_root).as_posix().encode('utf-8'))
            digest.update(path.read_bytes())
        logger.write('supervisor_start', manifest.episode_id, source=manifest.source, manifest=manifest,
                     factory=factory_path, python_version=sys.version,
                     source_sha256=digest.hexdigest())
        process.start()
        send.close()
        try:
            while True:
                now = monotonic()
                if now-started >= manifest.limits.max_wall_time_s:
                    reason, detail = 'search_limit', 'hard_wall_deadline'
                    break
                if pending and now >= pending[1]:
                    detail = 'module_timeout:' + pending[0]
                    break
                if receive.poll(.02):
                    try:
                        message = receive.recv()
                    except EOFError:
                        break
                    if message[0] == 'result':
                        summary = message[1]
                        break
                    if message[0] == 'error':
                        detail = 'worker_error:' + message[1]
                        break
                    _, event, episode, payload = message
                    if event == 'motion_sample':
                        partial.update({k: payload[k] for k in ('steps', 'actual_distance_m', 'simulated_time_s', 'collisions')})
                    elif event == 'decision':
                        partial['decisions'] += 1
                    elif event == 'dispatch':
                        partial['attempts'] += 1
                    if event == 'episode_terminal':
                        # Keep exactly one terminal event; publish after cleanup/join.
                        summary = {k: v for k, v in payload.items() if k != 'source'}
                    else:
                        logger.write(event, episode, **payload)
                    if event == 'module_begin':
                        pending = (payload['module'], monotonic()+payload['deadline_s'])
                    elif event == 'module_end' or event == 'episode_start':
                        pending = None
                elif not process.is_alive():
                    break
        except KeyboardInterrupt:
            reason, detail = 'externally_cancelled', 'keyboard_interrupt'
            summary = None
        finally:
            process.join(timeout=.1)
            if process.is_alive():
                process.terminate()
                process.join(timeout=2.)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=2.)
            receive.close()
        if summary is None:
            summary = dict(termination=reason, detail=detail,
                           wall_time_s=monotonic()-started, partial_metrics=True, **partial)
        logger.write('episode_terminal', manifest.episode_id, source=manifest.source, **summary)
    return summary
