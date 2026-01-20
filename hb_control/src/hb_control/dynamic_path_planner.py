import math
import heapq


class AStarPlanner:
    def __init__(self,
                 arena_size_mm=2438.4,
                 grid_res=80.0,
                 robot_radius=60.0,
                 obstacle_radius=50.0,
                 safety_margin=30.0):

        self.resolution = grid_res
        self.size_mm = arena_size_mm
        self.width = int(arena_size_mm / grid_res)
        self.height = int(arena_size_mm / grid_res)
        self.N = self.width

        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.safety_margin = safety_margin

        self.block_radius = robot_radius + obstacle_radius + safety_margin

    # =====================================================
    # Coordinate transforms
    # =====================================================
    def world_to_grid(self, x, y):
        gx = int(max(0, min(self.N - 1, x / self.resolution)))
        gy = int(max(0, min(self.N - 1, y / self.resolution)))
        return gx, gy

    def grid_to_world(self, gx, gy):
        return (
            (gx + 0.5) * self.resolution,
            (gy + 0.5) * self.resolution
        )

    # =====================================================
    # Grid construction
    # =====================================================
    def build_grid(self,
                   crate_poses,
                   robot_poses=None,
                   ignore_id=None):

        grid = [[0 for _ in range(self.N)] for _ in range(self.N)]

        def mark_obstacle(cx, cy):
            for i in range(self.N):
                for j in range(self.N):
                    wx, wy = self.grid_to_world(i, j)
                    d = math.hypot(wx - cx, wy - cy)
                    if d <= self.block_radius:
                        grid[i][j] = 100
                    elif d <= self.block_radius + self.resolution:
                        # soft cost ring
                        grid[i][j] = max(grid[i][j], 20)

        # -------- Crates --------
        for c in crate_poses:
            if ignore_id is not None and c.id == ignore_id:
                continue
            mark_obstacle(c.x, c.y)

        # -------- Other robots --------
        if robot_poses:
            for r in robot_poses:
                mark_obstacle(r.x, r.y)

        return grid

    # =====================================================
    # A* search
    # =====================================================
    def plan(self, start, goal, grid):

        sx, sy = self.world_to_grid(*start)
        gx, gy = self.world_to_grid(*goal)

        open_q = []
        heapq.heappush(open_q, (0.0, (sx, sy)))

        came_from = {}
        g_cost = {(sx, sy): 0.0}

        moves = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        while open_q:
            _, cur = heapq.heappop(open_q)

            if cur == (gx, gy):
                break

            for dx, dy in moves:
                nx, ny = cur[0] + dx, cur[1] + dy

                if not (0 <= nx < self.N and 0 <= ny < self.N):
                    continue
                if grid[nx][ny] >= 100:
                    continue

                step_cost = math.hypot(dx, dy)
                soft_cost = grid[nx][ny] * 0.02
                new_cost = g_cost[cur] + step_cost + soft_cost

                if (nx, ny) not in g_cost or new_cost < g_cost[(nx, ny)]:
                    g_cost[(nx, ny)] = new_cost
                    h = math.hypot(gx - nx, gy - ny)
                    f = new_cost + h
                    heapq.heappush(open_q, (f, (nx, ny)))
                    came_from[(nx, ny)] = cur

        # -------- reconstruct path --------
        if (gx, gy) not in came_from:
            return []

        path = []
        cur = (gx, gy)
        while cur != (sx, sy):
            path.append(self.grid_to_world(*cur))
            cur = came_from[cur]

        path.reverse()
        return path
