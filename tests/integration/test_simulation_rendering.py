"""Spectator rendering must not disturb the native camera or sensor materials."""
import importlib.util
from types import SimpleNamespace
import unittest


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'optional numpy unavailable')
class SimulationRenderingTests(unittest.TestCase):
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
