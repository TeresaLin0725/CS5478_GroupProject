"""Obstacle feedback stops the old command and reaches the strategy boundary."""
import importlib.util
from math import hypot
from time import monotonic
from types import SimpleNamespace
import unittest


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'optional numpy unavailable')
class DemoFeedbackTests(unittest.TestCase):
    def episode(self, reachable):
        import numpy as np
        from frontier_search.contracts import Pose
        from frontier_search.runner.igibson_demo import Episode
        from frontier_search.simulation.igibson_rs import HEIGHT, WIDTH, CONTROL_DT
        class World:
            steps, command = 14, (0,0)
            position, planned = Pose(.05,.05,0,0), False
            @property
            def sim_time(self): return self.steps*CONTROL_DT
            def pose(self): return self.position
            def drive(self,v,w):
                self.command=(v,w);self.steps+=1
                if self.planned and v>0:self.position=Pose(1.05,.05,0,0)
                return False
            def stop(self):self.command=(0,0);self.steps+=3
        world=World()
        class Mapper:
            claim, revision = None, 1
            safe=np.zeros((HEIGHT,WIDTH),dtype=bool)
            def mark_footprint(self):pass
            def observe(self):self.revision+=1
            def traversable(self):return self.safe
            def plan(self,goal):
                assert world.command==(0,0), 'Planning began with an active wheel command'
                world.planned=True
                if reachable:self.safe[:]=True
                return {},{goal:None} if reachable else {},self.safe
            def route(self,parent,goal):return [(.05,.05),(1.05,.05)]
        e=Episode.__new__(Episode);e.world=world;e.mapper=Mapper();e.started=monotonic()
        e.active_goal='F1';e.feedback_events=[];e.event_log=None;e.caption='test'
        e.confirm_target=lambda:None;e.frame=lambda:None
        return e

    def test_no_safe_route_requests_upper_replan_after_stopping(self):
        from frontier_search.contracts import FailureReason
        e=self.episode(False)
        status,failure,count=e.follow([(.05,.05),(1.05,.05)],(45,55))
        self.assertEqual((status,failure,count),('failed',FailureReason.BLOCKED,1))
        self.assertEqual(e.world.command,(0,0))
        self.assertEqual([r['kind'] for r in e.feedback_events],
            ['A_to_B_map_update','A_to_B_route_invalidated','B_local_replan','B_to_C_replan_request'])
        self.assertEqual(e.feedback_events[-1]['detail'],'updated_map_no_path')
        self.assertEqual(e.feedback_events[-1]['goal_id'],'F1')

    def test_local_detour_keeps_goal_when_reachable(self):
        e=self.episode(True)
        status,failure,count=e.follow([(.05,.05),(1.05,.05)],(45,55))
        self.assertEqual((status,failure,count),('reached',None,1))
        self.assertEqual(e.active_goal,'F1')
        self.assertTrue(any(r['kind']=='B_local_replan' and r['reachable'] for r in e.feedback_events))
        self.assertFalse(any(r['kind']=='B_to_C_replan_request' for r in e.feedback_events))

    def test_spectator_keeps_clear_side_and_bounds_camera_translation(self):
        import numpy as np
        from frontier_search.contracts import Pose
        from frontier_search.simulation.igibson_rs import IGibsonWorld
        class Physics:
            block=False
            def rayTestBatch(self,starts,ends):
                occluded=self.block and starts[0][1]>.2
                return [(2 if occluded else -1,0,.1 if occluded else 1)]*len(ends)
        class Renderer:
            lightpos,lightV=[0,0,2],None
            def set_fov(self,value):pass
            def set_camera(self,eye,target,up):self.eye=eye.copy()
            def set_light_position_direction(self,position,target):self.lightpos=position
            def render(self,**kwargs):return [np.zeros((4,4,4))]
        world=IGibsonWorld.__new__(IGibsonWorld);world.p=Physics()
        world.sim=SimpleNamespace(renderer=Renderer());world.robot_bodies={1};world.robot_height=.65
        world.robot=SimpleNamespace(renderer_instances=[])
        world.pose=lambda:Pose(0,0,0,0);world.steps=0
        world.spectator_eye=world.spectator_target=None
        world.spectator_offset=2;world.spectator_last_s=0.
        world.spectator();self.assertEqual(world.spectator_offset,2)
        before=world.spectator_eye.copy();world.steps=3;world.p.block=True
        world.spectator()
        self.assertNotEqual(world.spectator_offset,2)
        self.assertLessEqual(float(np.linalg.norm(world.spectator_eye-before)),.180001)

    def test_spectator_restores_sensor_materials_and_lighting_even_on_render_failure(self):
        import numpy as np
        from frontier_search.contracts import Pose
        from frontier_search.simulation.igibson_rs import IGibsonWorld
        for fail in (False,True):
            with self.subTest(render_failure=fail):
                material=SimpleNamespace(kd=[0.,0.,0.],is_texture=lambda:False)
                original=material.kd
                class Renderer:
                    lightpos,lightV=[0,0,2],np.eye(4)
                    shape_material_idx=[0]
                    material_idx_to_material_instance_mapping={0:material}
                    def set_fov(self,value):pass
                    def set_camera(self,*args):pass
                    def set_light_position_direction(self,position,target):
                        self.lightpos,self.lightV=position,np.zeros((4,4))
                    def render(inner,**kwargs):
                        self.assertGreater(max(material.kd),0)
                        if fail:raise RuntimeError('render failed')
                        return [np.zeros((4,4,4))]
                renderer=Renderer();lightpos,lightview=renderer.lightpos,renderer.lightV
                world=IGibsonWorld.__new__(IGibsonWorld)
                world.sim=SimpleNamespace(renderer=renderer)
                world.robot=SimpleNamespace(renderer_instances=[SimpleNamespace(objects=[SimpleNamespace(VAO_ids=[0])])])
                world.robot_bodies={1};world.robot_height=.65;world.steps=0
                world.p=SimpleNamespace(rayTestBatch=lambda starts,ends:[(-1,0,1)]*len(ends))
                world.pose=lambda:Pose(0,0,0,0)
                world.spectator_eye=world.spectator_target=None
                world.spectator_offset=0;world.spectator_last_s=0.
                if fail:
                    with self.assertRaises(RuntimeError):world.spectator()
                else:world.spectator()
                self.assertIs(material.kd,original)
                self.assertIs(renderer.lightpos,lightpos)
                self.assertIs(renderer.lightV,lightview)
