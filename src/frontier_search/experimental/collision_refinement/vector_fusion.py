"""Experimental batch ray evidence. No change to production fusion or policies."""
from math import ceil
import numpy as np
from frontier_search.perception.fusion import fuse_observation as scalar_fuse,_finalize_free_space
from frontier_search.perception.projection import iter_depth_world

def accumulate(start,ends,bounds,coverage,masks,fresh=None):
    """Batch the scalar positive-length DDA, preserving arithmetic and tolerances."""
    ends=np.asarray(ends,dtype=np.float64).reshape(-1,3)
    if not len(ends):return
    start=np.asarray(start,dtype=np.float64)
    delta=ends-start
    enter=np.zeros(len(ends));leave=np.ones(len(ends));valid=np.ones(len(ends),dtype=bool)
    for axis,upper in enumerate(bounds):
        a=start[axis];d=delta[:,axis];nonzero=d!=0
        valid &= nonzero | (0<=a<upper)
        first=np.zeros(len(ends));last=np.ones(len(ends))
        np.divide(-a,d,out=first,where=nonzero)
        np.divide(upper-a,d,out=last,where=nonzero)
        lower=np.minimum(first,last);higher=np.maximum(first,last)
        enter=np.maximum(enter,lower);leave=np.minimum(leave,higher)
    valid &= (enter<leave)&np.any(delta!=0,axis=1)
    delta=delta[valid];current=enter[valid];leave=leave[valid]
    if not len(delta):return
    value=start+current[:,None]*delta
    rounded=np.rint(value)
    cells=np.floor(value)
    cells=np.where((delta<0)&(np.abs(value-rounded)<1e-10),rounded-1,cells)
    limits=np.asarray([ceil(b) for b in bounds],dtype=np.int64)
    cells=np.maximum(0,np.minimum(limits-1,cells)).astype(np.int64)
    crossings=np.full(delta.shape,np.inf)
    np.divide(cells+(delta>0)-start,delta,out=crossings,where=delta!=0)
    increments=np.full(delta.shape,np.inf)
    np.divide(1.,delta,out=increments,where=delta!=0);increments=np.abs(increments)
    steps=np.sign(delta).astype(np.int64)
    frame_masks=np.zeros(masks.shape,dtype=np.uint64);flat_masks=frame_masks.ravel()
    while len(cells):
        following=leave.copy()
        for axis in range(3):following=np.minimum(following,crossings[:,axis])
        positive=following-current>1e-12
        c,r,z=cells[positive].T
        bits=np.left_shift(np.uint64(1),z.astype(np.uint64))
        coverage[r,c]=True
        np.bitwise_or.at(flat_masks,r*masks.shape[1]+c,bits)
        keep=following<leave
        cells=cells[keep];crossings=crossings[keep];increments=increments[keep];steps=steps[keep]
        leave=leave[keep];following=following[keep]
        moving=crossings<=following[:,None]+1e-12
        cells[moving]+=steps[moving];crossings[moving]+=increments[moving]
        keep=np.all((cells>=0)&(cells<limits),axis=1)
        cells=cells[keep];crossings=crossings[keep];increments=increments[keep];steps=steps[keep]
        leave=leave[keep];current=following[keep]
    np.bitwise_or(masks,frame_masks,out=masks)
    if fresh is not None:np.bitwise_or(fresh,frame_masks,out=fresh)

def fuse_observation(observation,cfg,collision_z,occupancy,coverage,free_masks,floor_seen,
                     clear_counts=None,increment_clear_count=True):
    low,high=collision_z;extent=(high-low)/cfg.clearance_bin_height_m
    nearest=round(extent)
    if nearest>0 and abs(extent-nearest)<1e-12:extent=float(nearest)
    bins=ceil(extent)
    # Keep the existing arbitrary-width Python integer model for tall columns.
    if bins>64 or any(v<0 or v>2**64-1 for row in free_masks for v in row):
        return scalar_fuse(observation,cfg,collision_z,occupancy,coverage,free_masks,floor_seen,
            clear_counts=clear_counts,increment_clear_count=increment_clear_count)
    points=np.asarray(tuple(iter_depth_world(observation,cfg.depth_pixel_stride)),dtype=np.float64).reshape(-1,6)
    points=points[(points[:,2]>=cfg.depth_min_m)&(points[:,2]<=cfg.depth_max_m)]
    occ=np.asarray(occupancy,dtype=np.int64);seen=np.asarray(coverage,dtype=bool)
    masks=np.asarray(free_masks,dtype=np.uint64);floor_seen_array=np.asarray(floor_seen,dtype=bool)
    fresh=np.zeros(masks.shape,dtype=np.uint64) if clear_counts is not None else None
    fresh_floor=set();hits=set()
    if len(points):
        world=points[:,3:];depth=points[:,2]
        column=(world[:,0]-cfg.origin_x_m)/cfg.resolution_m
        row=(world[:,1]-cfg.origin_y_m)/cfg.resolution_m
        if not (np.all(np.isfinite(column)) and np.all(np.isfinite(row)) and
                np.all(np.abs(column)<2**62) and np.all(np.abs(row)<2**62)):
            return scalar_fuse(observation,cfg,collision_z,occupancy,coverage,free_masks,floor_seen,
                clear_counts=clear_counts,increment_clear_count=increment_clear_count)
        c=np.floor(column).astype(np.int64);r=np.floor(row).astype(np.int64)
        inside=(r>=0)&(r<cfg.height_cells)&(c>=0)&(c<cfg.width_cells)
        seen[r[inside],c[inside]]=True
        floor_hit=np.abs(world[:,2]-cfg.floor_height_m)<=cfg.floor_tolerance_m
        floor_selected=inside&floor_hit
        floor_seen_array[r[floor_selected],c[floor_selected]]=True
        fresh_floor=set(zip(r[floor_selected].tolist(),c[floor_selected].tolist()))
        obstacle=inside&~floor_hit&(world[:,2]>=low)&(world[:,2]<=high)
        occ[r[obstacle],c[obstacle]]=100
        hits=set(zip(r[obstacle].tolist(),c[obstacle].tolist()))
        camera=np.asarray([a[3] for a in observation.camera_to_world.matrix[:3]])
        fraction=(depth-cfg.depth_surface_margin_m)/depth
        endpoint=camera+fraction[:,None]*(world-camera)
        offsets=np.asarray((cfg.origin_x_m,cfg.origin_y_m,low))
        scales=np.asarray((cfg.resolution_m,cfg.resolution_m,cfg.clearance_bin_height_m))
        start=(camera-offsets)/scales;ends=(endpoint-offsets)/scales
        if not np.all(np.isfinite(ends-start)):
            return scalar_fuse(observation,cfg,collision_z,occupancy,coverage,free_masks,floor_seen,
                clear_counts=clear_counts,increment_clear_count=increment_clear_count)
        accumulate(start,ends,(cfg.width_cells,cfg.height_cells,extent),seen,masks,fresh)
    occupancy[:]=occ.tolist();coverage[:]=seen.tolist();free_masks[:]=masks.tolist();floor_seen[:]=floor_seen_array.tolist()
    _finalize_free_space(cfg,occupancy,free_masks,floor_seen,clear_counts,increment_clear_count,
        fresh.tolist() if fresh is not None else None,fresh_floor,hits,(1<<bins)-1)
