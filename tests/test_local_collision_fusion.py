import copy,pickle,random,sys,unittest
from dataclasses import replace
from pathlib import Path
from struct import pack
OUT=Path(__file__).resolve().parent;ROOT=OUT.parent
BASE=ROOT/'runs/local-refinement-20261103-01'
sys.path[:0]=[str(ROOT/'src'),str(ROOT)]
try:
    import numpy as np
except ImportError:
    raise unittest.SkipTest('NumPy required for experimental local fusion')
from frontier_search.experimental.collision_refinement import vector_fusion as batch
from frontier_search.config import MappingConfig
from frontier_search.perception.fusion import fuse_observation as scalar,_ray_voxels
from frontier_search.fixtures.analytic_rgbd import render_frame

NAMES=('occupancy','coverage','free_masks','floor_seen','clear_counts')

class VectorFusionTests(unittest.TestCase):
    def test_6000_random_boundary_parallel_and_degenerate_rays(self):
        rng=random.Random(5478)
        for group in range(60):
            bounds=rng.choice(((3.,4.,2.4),(7.,6.,19.99999999999),(3.,3.,63.99)))
            def coordinate(upper):
                if rng.random()<.6:return rng.choice((0.,upper,1.,-1.))+rng.choice((0.,1e-13,-1e-13,1e-10,-1e-10))
                return rng.uniform(-2.,upper+2.)
            start=tuple(coordinate(b) for b in bounds);ends=[]
            for index in range(100):
                end=list(start if index%17==0 else (coordinate(b) for b in bounds))
                if index%7==0:end[0]=start[0]
                if index%11==0:end[1]=start[1]
                if index%13==0:end[2]=start[2]
                ends.append(tuple(end))
            h,w=int(bounds[1]),int(bounds[0])
            expected_masks=[[rng.randrange(4) for _ in range(w)] for _ in range(h)]
            expected_seen=[[False]*w for _ in range(h)];expected_fresh=[[0]*w for _ in range(h)]
            seen=np.asarray(expected_seen);masks=np.asarray(expected_masks,dtype=np.uint64);fresh=np.asarray(expected_fresh,dtype=np.uint64)
            batch.accumulate(start,ends,bounds,seen,masks,fresh)
            for end in ends:
                for c,r,z in _ray_voxels(start,end,bounds):
                    expected_seen[r][c]=True;expected_masks[r][c]|=1<<z;expected_fresh[r][c]|=1<<z
            with self.subTest(group=group,start=start,bounds=bounds):
                self.assertEqual(seen.tolist(),expected_seen)
                self.assertEqual(masks.tolist(),expected_masks)
                self.assertEqual(fresh.tolist(),expected_fresh)

    def test_multiframe_static_dynamic_invalid_depth_and_large_mask_fallback(self):
        box=(((.8,-.3,0.),(1.2,.3,1.)),)
        frames=[render_frame(sequence=i+1,width=23,height=17,x=.1 if i==5 else 0.,yaw=.2 if i==5 else 0.,
            boxes=box if i in (0,3) else ()) for i in range(6)]
        invalid=replace(frames[-1].depth_image,data=pack('<391f',*([float('nan')]*391)))
        frames.append(replace(frames[-1],sequence=7,depth_image=invalid))
        for dynamic in (0,2):
            for stride in (1,4):
                for bin_height in (.05,.001):
                    cfg=MappingConfig(resolution_m=.05,width_cells=60,height_cells=60,origin_x_m=-1.5,origin_y_m=-1.5,
                        depth_pixel_stride=stride,dynamic_clear_frames=dynamic,clearance_bin_height_m=bin_height)
                    buffers=dict(zip(NAMES,([[[-1]*60 for _ in range(60)],[[False]*60 for _ in range(60)],
                        [[0]*60 for _ in range(60)],[[False]*60 for _ in range(60)],[[0]*60 for _ in range(60)]])))
                    expected=copy.deepcopy(buffers)
                    for index,frame in enumerate(frames):
                        for fn,target in ((scalar,expected),(batch.fuse_observation,buffers)):
                            fn(frame,cfg,(.04,.923),*(target[n] for n in NAMES[:4]),
                                clear_counts=target['clear_counts'] if dynamic else None,increment_clear_count=index!=2)
                        with self.subTest(dynamic=dynamic,stride=stride,bin_height=bin_height,frame=index):self.assertEqual(buffers,expected)


if __name__=='__main__':unittest.main()
