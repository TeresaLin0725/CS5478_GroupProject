import sys,unittest,importlib.util
from dataclasses import replace
from pathlib import Path
from struct import pack
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT)]
from frontier_search.config import MappingConfig
from frontier_search.contracts import GridCell
from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.navigation.geometry import world_to_cell
from frontier_search.experimental.collision_refinement.rolling_local_map import RollingLocalMap

def frame(sequence=1,x=0.):
    obs=render_frame(sequence=sequence,x=x,width=9,height=7)
    return replace(obs,depth_image=replace(obs.depth_image,data=pack('<63f',*([float('nan')]*63))))

@unittest.skipUnless(importlib.util.find_spec('numpy'), 'NumPy required for experimental local fusion')
class LocalMapTests(unittest.TestCase):
    def subject(self):
        request=SimpleNamespace(episode_id='synthetic-rgbd',mapping_config=MappingConfig(
            width_cells=140,height_cells=100,origin_x_m=-4.5,origin_y_m=-4.5,dynamic_clear_frames=2),
            initial_clearance_cells=(GridCell(45,45),))
        return RollingLocalMap(request)
    def test_only_verified_spawn_subcells_are_seeded(self):
        local=self.subject();grid=local.update(frame(),(.04,.9),1)
        self.assertEqual(sum(v==0 for row in grid.data for v in row),4)
        self.assertFalse(any(v for row in local.buffers['floor_seen'] for v in row))
        self.assertFalse(any(v for row in local.buffers['free_masks'] for v in row))
    def test_rolling_preserves_world_cells_and_new_cells_are_unknown(self):
        local=self.subject();grid=local.update(frame(),(.04,.9),1)
        cell=world_to_cell(grid,.025,.025)
        local.buffers['occupancy'][cell.row][cell.column]=100
        local.buffers['free_masks'][cell.row][cell.column]=17
        after=local.update(frame(2,.6),(.04,.9),2)
        moved=world_to_cell(after,.025,.025)
        self.assertEqual(after.data[moved.row][moved.column],100)
        self.assertEqual(local.buffers['free_masks'][moved.row][moved.column],17)
        self.assertTrue(all(row[-1]==-1 for row in after.data))
        self.assertEqual(local.shifts,1)
    def test_forgotten_spawn_area_is_not_reseeded_on_return(self):
        local=self.subject();local.update(frame(),(.04,.9),1)
        local.update(frame(2,4.),(.04,.9),2)
        back=local.update(frame(3),(.04,.9),3)
        self.assertTrue(all(v==-1 for row in back.data for v in row))
    def test_snapshots_are_owned_and_failed_sequence_does_not_mutate(self):
        local=self.subject();grid=local.update(frame(),(.04,.9),1)
        original=grid.data
        local.update(frame(2,.6),(.04,.9),2)
        self.assertEqual(grid.data,original)
        current=local.grid
        with self.assertRaises(ValueError):local.update(frame(2,.6),(.04,.9),3)
        self.assertIs(local.grid,current)
    def test_patch_cannot_publish_free_space_outside_global_bounds(self):
        local=self.subject()
        local.base=replace(local.base,origin_x_m=0.,origin_y_m=0.,width_cells=2,height_cells=2)
        local.seed_cells={(0,0)}
        grid=local.update(frame(),(.04,.9),1)
        for r,row in enumerate(grid.data):
            for c,value in enumerate(row):
                x=grid.origin_x_m+(c+.5)*grid.resolution_m;y=grid.origin_y_m+(r+.5)*grid.resolution_m
                if not (0<=x<.2 and 0<=y<.2):self.assertEqual(value,-1)

if __name__=='__main__':unittest.main()
