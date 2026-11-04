"""B's opt-in coarse planning lattice with consistent coarse/fine collision tests."""
from math import floor
from ...contracts import Pose
from ...navigation.costmap import Costmap
from ...navigation.geometry import cell_center, cell_value, footprint_in_world, polygon_intersects_square


class RefinedCostmap(Costmap):
    def __init__(self, coarse, patch, is_current):
        super().__init__(coarse.grid, coarse.footprint_base, coarse.margin_m,
            sample_step_m=min(coarse.sample_step_m, patch.grid.resolution_m/2),
            planning_radius_m=coarse.planning_radius_m, start_margin_m=coarse.start_margin_m)
        self.coarse, self.patch, self.is_current = coarse, patch, is_current
        self._rectangles = {}
        g, fine = self.grid, patch.grid
        permitted = None if patch.release_cells is None else {(cell.row,cell.column) for cell in patch.release_cells}
        factor = round(g.resolution_m/fine.resolution_m)
        first_c = round((fine.origin_x_m-g.origin_x_m)/g.resolution_m)
        first_r = round((fine.origin_y_m-g.origin_y_m)/g.resolution_m)
        for r0 in range(0, fine.height_cells, factor):
            for c0 in range(0, fine.width_cells, factor):
                r, c = first_r+r0//factor, first_c+c0//factor
                value = cell_value(g, r, c)
                if value == -1:  # coarse unknown and outside-map are never released
                    continue
                if value == 100 and permitted is not None and (r,c) not in permitted:
                    continue  # keep this whole coarse obstacle; fine FREE has no release authority here
                blocked = []
                for dr in range(factor):
                    for dc in range(factor):
                        rr, cc = r0+dr, c0+dc
                        child = fine.data[rr][cc]
                        if child == 100 or (value == 100 and child != 0):
                            x = fine.origin_x_m+cc*fine.resolution_m
                            y = fine.origin_y_m+rr*fine.resolution_m
                            blocked.append((x,y,x+fine.resolution_m,y+fine.resolution_m))
                self._rectangles[r,c] = tuple(blocked)

    def prepare(self):
        self.coarse.prepare()

    def _collides(self, footprint, pose, margin):
        polygon = footprint_in_world(footprint, pose.x_m, pose.y_m, pose.yaw_rad)
        xs, ys = [p[0] for p in polygon], [p[1] for p in polygon]
        grid = self.grid; res = grid.resolution_m
        for r in range(floor((min(ys)-margin-grid.origin_y_m)/res),
                       floor((max(ys)+margin-grid.origin_y_m)/res)+1):
            for c in range(floor((min(xs)-margin-grid.origin_x_m)/res),
                           floor((max(xs)+margin-grid.origin_x_m)/res)+1):
                rectangles = self._rectangles.get((r,c))
                if rectangles is None:
                    if cell_value(grid,r,c) == 0:
                        continue
                    x, y = grid.origin_x_m+c*res, grid.origin_y_m+r*res
                    rectangles = ((x,y,x+res,y+res),)
                for x0,y0,x1,y1 in rectangles:
                    if polygon_intersects_square(polygon,x0-margin,y0-margin,x1+margin,y1+margin):
                        return True
        return False

    def cell_traversable(self, row, column):
        if not self.is_current():
            return self.coarse.cell_traversable(row,column)
        if not 0 <= row < self.grid.height_cells or not 0 <= column < self.grid.width_cells:
            return False
        key = row,column
        if key not in self._cell_cache:
            p = cell_center(self.grid,row,column)
            self._cell_cache[key] = not self._collides(self._disc,Pose(p.x_m,p.y_m,0.,0.),self.margin_m)
        return self._cell_cache[key]

    def pose_valid(self, pose, margin_m=None):
        if not self.is_current():
            return self.coarse.pose_valid(pose,margin_m)
        return not self._collides(self.footprint_base,pose,self.margin_m if margin_m is None else margin_m)

    def start_pose_valid(self, pose):
        return self.pose_valid(pose,self.start_margin_m)

    def near_obstacle(self, pose, extra_m):
        return not self.pose_valid(pose,self.margin_m+extra_m)

    def _sweep_valid(self, pose, start_yaw, total_rad, step_rad, margin_m=None):
        return super()._sweep_valid(pose,start_yaw,total_rad,
            min(step_rad,self.sample_step_m/max(self.radius_m,1e-9)),margin_m)
