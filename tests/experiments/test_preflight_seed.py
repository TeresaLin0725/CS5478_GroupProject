"""Preflight must open the same seeded task as the native episode."""
from contextlib import redirect_stderr,redirect_stdout
from dataclasses import replace
from io import StringIO
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

try:
    import numpy
    import PIL
    PREFLIGHT_DEPENDENCIES=True
except ImportError:
    PREFLIGHT_DEPENDENCIES=False

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from benchmark_episode import preflight
from run_paired_benchmark import benchmark_manifest


@unittest.skipUnless(PREFLIGHT_DEPENDENCIES,'native preflight logic requires NumPy and Pillow')
class PreflightSeedTests(unittest.TestCase):
    def test_native_reset_uses_manifest_seed_and_closes_on_failure(self):
        for seed in (17,43):
            manifest=replace(benchmark_manifest(),environment_seed=seed)
            opened=[]
            closed=[]
            class Adapter:
                def __init__(self,*args):pass
                def reset(self,*args):
                    opened.append(args)
                    raise RuntimeError('stop before physical queries')
                def close(self):closed.append(True)
            with tempfile.TemporaryDirectory() as directory:
                out=Path(directory)
                with patch('benchmark_episode.preflight_manifest',return_value=manifest), \
                     patch('frontier_search.simulation.integrated_rs.RetainingAdapter',Adapter), \
                     redirect_stdout(StringIO()),redirect_stderr(StringIO()):
                    self.assertEqual(preflight({},'dynamic',out),1)
                self.assertEqual(opened,[(manifest.episode_id,'Rs',manifest.start_pose,seed)])
                self.assertEqual(closed,[True])
                result=json.loads((out/'preflight.json').read_text(encoding='utf-8'))
                self.assertEqual(result['environment_seed'],seed)
                self.assertEqual(result['start_pose']['yaw_rad'],manifest.start_pose.yaw_rad)
                self.assertFalse(result['passed'])


if __name__=='__main__':unittest.main()
