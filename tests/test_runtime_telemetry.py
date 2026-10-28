"""Runtime telemetry must preserve evidence without stalling robot execution."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from threading import Event,Thread
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from benchmark_watchdog import WatchdogChannel,AsyncEventWriter,CAPACITY
from run_paired_benchmark import module_deadline_expired

class RuntimeTelemetryTests(unittest.TestCase):
    def test_old_disk_completion_cannot_hide_a_current_shared_memory_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory,'events.jsonl').write_text(json.dumps(
                dict(event='module_end',module='mapper.update',elapsed_s=2.))+'\n')
            watch=dict(pending_module='mapper.update',deadline_at=30.)
            self.assertTrue(module_deadline_expired(watch,directory,
                dict(watchdog_transport='shared_memory_v1'),33.))
            self.assertFalse(module_deadline_expired(watch,directory,{},33.))
            self.assertFalse(module_deadline_expired(watch,directory,
                dict(watchdog_transport='shared_memory_v1'),31.))

    def test_shared_watchdog_is_visible_to_another_process(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=WatchdogChannel(directory,create=True)
            try:
                writer.publish(dict(pending_module='mapper.update',steps=123))
                code=('import sys,json;sys.path.insert(0,sys.argv[1]);'
                      'from benchmark_watchdog import WatchdogChannel;'
                      'c=WatchdogChannel(sys.argv[2]);print(json.dumps(c.read()));c.close()')
                result=subprocess.run([sys.executable,'-B','-c',code,
                    str(Path(__file__).resolve().parents[1]/'scripts'),directory],
                    capture_output=True,text=True,timeout=60)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertEqual(json.loads(result.stdout),dict(pending_module='mapper.update',steps=123))
            finally:writer.close()

    def test_incomplete_and_oversized_updates_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            channel=WatchdogChannel(directory,create=True)
            try:
                self.assertIsNone(channel.read())
                channel.publish({'steps':1})
                with self.assertRaises(ValueError):channel.publish({'data':'x'*CAPACITY})
                self.assertEqual(channel.read(),{'steps':1})
                struct.pack_into('<Q',channel.memory.buf,0,3)
                self.assertIsNone(channel.read())
            finally:channel.close()

    def test_blocked_disk_writer_cannot_block_execution_or_watchdog(self):
        entered,release,submitted=Event(),Event(),Event()
        records=[]
        def blocked(value):
            entered.set();release.wait(5);records.append(value)
        sink=AsyncEventWriter(blocked)
        with tempfile.TemporaryDirectory() as directory:
            channel=WatchdogChannel(directory,create=True)
            try:
                sink.submit(0);self.assertTrue(entered.wait(1))
                def execute():
                    for i in range(1,101):
                        sink.submit(i);channel.publish({'steps':i})
                    submitted.set()
                thread=Thread(target=execute);thread.start()
                self.assertTrue(submitted.wait(1),'execution stalled behind disk logger')
                thread.join(1)
                self.assertEqual(channel.read(),{'steps':100})
            finally:
                release.set();sink.close();channel.close()
        self.assertEqual(records,list(range(101)))

    def test_evidence_write_errors_are_not_silently_ignored(self):
        def failed():raise OSError('disk unavailable')
        sink=AsyncEventWriter(failed);sink.submit()
        with self.assertRaisesRegex(RuntimeError,'evidence writer failed'):sink.close()
