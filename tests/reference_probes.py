"""Frozen navigation-probe selector before the free-cell collision optimization."""
from math import atan2,hypot,pi
from frontier_search.contracts import Frontier,GridCell,Pose
from frontier_search.navigation.geometry import cells_overlapping_footprint


def navigation_probes(grid,observation,robot):
    p=observation.robot_pose;sectors={}
    for r,row in enumerate(grid.data):
        for c,value in enumerate(row):
            if value!=0:continue
            x=grid.origin_x_m+(c+.5)*grid.resolution_m
            y=grid.origin_y_m+(r+.5)*grid.resolution_m
            distance=hypot(x-p.x_m,y-p.y_m)
            if not .75<=distance<=1.05:continue
            yaw=atan2(p.y_m-y,p.x_m-x)
            pose=Pose(x,y,p.z_m,yaw)
            cells=tuple(cells_overlapping_footprint(grid,robot.footprint_base,pose,robot.clearance_margin_m))
            if not cells or any(not(0<=rr<grid.height_cells and 0<=cc<grid.width_cells)
                                or grid.data[rr][cc]!=0 for rr,cc in cells):continue
            sector=int((atan2(y-p.y_m,x-p.x_m)+pi)/(pi/4))%8
            score=(abs(distance-.85),r,c)
            if sector not in sectors or score<sectors[sector][0]:sectors[sector]=(score,pose,r,c)
    return tuple(Frontier('probe-%03d-%03d'%(r,c),(),(GridCell(r,c),),pose,None,0.,
        hypot(pose.x_m-p.x_m,pose.y_m-p.y_m),'navigation_probe',None)
        for _,pose,r,c in (value for _,value in sorted(sectors.items())))
