"""A-side rolling fusion with optional off-window history of observed evidence."""
from dataclasses import replace
from math import floor,hypot,isfinite
from frontier_search.contracts import OccupancyGrid

def fuse_observation(*args, **kwargs):
    # NumPy is required only when this experiment actually processes a frame.
    from .vector_fusion import fuse_observation as batch
    return batch(*args, **kwargs)

NAMES=('occupancy','coverage','free_masks','floor_seen','clear_counts')
DEFAULTS=(-1,False,0,False,0)

class RollingLocalMap:
    def __init__(self,request,resolution_m=.05,extent_m=3.,shift_distance_m=.4,*,retain_history=False):
        if type(retain_history) is not bool:
            raise ValueError('retain_history must be bool')
        self.request=request;self.base=request.mapping_config
        ratio=self.base.resolution_m/resolution_m
        if not isfinite(ratio) or ratio<1 or abs(ratio-round(ratio))>1e-9:
            raise ValueError('Local resolution must divide coarse resolution')
        cells=extent_m/self.base.resolution_m
        if abs(cells-round(cells))>1e-9 or int(round(cells))%2 or cells<2:
            raise ValueError('Extent requires an even positive number of coarse cells')
        if not 0<shift_distance_m<extent_m/2:
            raise ValueError('Shift threshold must lie inside local window')
        self.factor=int(round(ratio));self.half=int(round(cells))//2
        self.width=2*self.half*self.factor;self.resolution=resolution_m
        self.shift_distance=shift_distance_m;self.origin=None;self.buffers=None
        self.sequence=None;self.timestamp=None;self.grid=None;self.shifts=0
        self.seed_cells={(c.row,c.column) for c in request.initial_clearance_cells}
        self.retain_history=retain_history;self._history={};self._history_geometry=None
        self.history_restored_cells=0

    def _shift_history(self, origin, buffers):
        # Integer world fine-cell indices avoid accumulating floating origin
        # errors. Cache only evicted, non-default evidence within the map.
        history=self._history.copy();restored=0
        old_c,old_r=(v*self.factor for v in self.origin)
        new_c,new_r=(v*self.factor for v in origin)
        size=self.width;base=self.base
        for r in range(size):
            rr=old_r+r
            if not 0<=rr<base.height_cells*self.factor:continue
            for c in range(size):
                cc=old_c+c
                if not 0<=cc<base.width_cells*self.factor:continue
                if new_r<=rr<new_r+size and new_c<=cc<new_c+size:continue
                evidence=tuple(self.buffers[name][r][c] for name in NAMES[:4])
                if evidence!=DEFAULTS[:4]:history[rr,cc]=evidence
        for r in range(size):
            rr=new_r+r
            for c in range(size):
                cc=new_c+c
                if old_r<=rr<old_r+size and old_c<=cc<old_c+size:continue
                evidence=history.pop((rr,cc),None)
                if evidence is not None:
                    for name,value in zip(NAMES[:4],evidence):buffers[name][r][c]=value
                    # Absence breaks a sequence of fresh clearing frames.
                    # clear_counts stays zero, even when occupancy is restored.
                    restored+=1
        return history,restored

    def update(self,observation,collision_z,revision):
        if observation.episode_id!=self.request.episode_id:
            raise ValueError('Local evidence belongs to another episode')
        if self.sequence is not None and (observation.sequence<=self.sequence or observation.timestamp_s<self.timestamp):
            raise ValueError('Local evidence requires fresh sequence and nondecreasing time')
        geometry=(self.base,self.resolution,self.width,tuple(collision_z))
        if self.retain_history and self._history_geometry is not None and geometry!=self._history_geometry:
            raise ValueError('Local history geometry changed; reset required')
        pose=observation.robot_pose;base=self.base
        origin=self.origin
        if origin is None or hypot(pose.x_m-(base.origin_x_m+(origin[0]+self.half)*base.resolution_m),
                                  pose.y_m-(base.origin_y_m+(origin[1]+self.half)*base.resolution_m))>self.shift_distance:
            origin=(floor((pose.x_m-base.origin_x_m)/base.resolution_m)-self.half,
                    floor((pose.y_m-base.origin_y_m)/base.resolution_m)-self.half)
        cfg=replace(base,resolution_m=self.resolution,width_cells=self.width,height_cells=self.width,
            origin_x_m=base.origin_x_m+origin[0]*base.resolution_m,
            origin_y_m=base.origin_y_m+origin[1]*base.resolution_m)
        # Work on owned copies; a failed fusion cannot corrupt the previous patch.
        buffers={name:[[default]*self.width for _ in range(self.width)] for name,default in zip(NAMES,DEFAULTS)}
        history,restored=self._history,0
        if self.buffers is not None:
            dx=(origin[0]-self.origin[0])*self.factor;dy=(origin[1]-self.origin[1])*self.factor
            for r in range(max(0,-dy),min(self.width,self.width-dy)):
                left=max(0,-dx);right=min(self.width,self.width-dx)
                if left<right:
                    for name in NAMES:buffers[name][r][left:right]=self.buffers[name][r+dy][left+dx:right+dx]
            if self.retain_history and origin!=self.origin:
                history,restored=self._shift_history(origin,buffers)
        else:
            # Only the original attested spawn cells may seed fine free space.
            # Later coarse FREE cells, new window cells and occluded space do not.
            for r in range(self.width):
                for c in range(self.width):
                    parent=(origin[1]+r//self.factor,origin[0]+c//self.factor)
                    if parent in self.seed_cells:buffers['occupancy'][r][c]=0
        fuse_observation(observation,cfg,collision_z,*(buffers[name] for name in NAMES[:4]),
            clear_counts=buffers['clear_counts'] if cfg.dynamic_clear_frames else None,
            increment_clear_count=self.timestamp is None or observation.timestamp_s>self.timestamp)
        # Local evidence cannot enlarge the episode's declared map bounds.
        for r in range(self.width):
            for c in range(self.width):
                pr,pc=origin[1]+r//self.factor,origin[0]+c//self.factor
                if not (0<=pr<base.height_cells and 0<=pc<base.width_cells):
                    for name,default in zip(NAMES,DEFAULTS):buffers[name][r][c]=default
        grid=OccupancyGrid('world',revision,cfg.resolution_m,cfg.origin_x_m,cfg.origin_y_m,
            cfg.width_cells,cfg.height_cells,tuple(tuple(row) for row in buffers['occupancy']))
        if self.origin is not None and origin!=self.origin:self.shifts+=1
        self.origin=origin;self.buffers=buffers;self.sequence=observation.sequence
        self.timestamp=observation.timestamp_s;self.grid=grid
        self._history=history;self.history_restored_cells+=restored
        if self.retain_history:self._history_geometry=geometry
        return grid
