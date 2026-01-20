import math
import heapq


class AStarPlanner:
    def __init__(self, arena_size_mm=2438.4, grid_res=50.0):
        self.resolution = grid_res
        self.width = int(arena_size_mm / grid_res)
        self.height = int(arena_size_mm / grid_res)
        self.N = self.width  # square grid

        # -------- D1 NO-ENTRY ZONE (WORLD MM) --------
        self.D1_X_MIN = 994.0
        self.D1_X_MAX = 1448.0
        self.D1_Y_MIN = 1054.0
        self.D1_Y_MAX = 1371.0

    # ---------------- Coordinate transforms ----------------
    def world_to_grid(self, x, y):
        return round(x / self.resolution), round(y / self.resolution)

    def grid_to_world(self, gx, gy):
        return (gx + 0.5) * self.resolution, (gy + 0.5) * self.resolution

    # ---------------- Grid building ----------------
    def build_grid(self, crate_poses, ignore_id, allow_d1=False):
        grid = [[0] * self.N for _ in range(self.N)]

        ROBOT_R = 30
        CRATE_R = 30
        SAFETY = 0
        BLOCK_R = ROBOT_R + CRATE_R + SAFETY

        # -------- Block crates --------
        for c in crate_poses:
            if ignore_id is not None and c.id == ignore_id:
                continue

            cx, cy = c.x, c.y
            for i in range(self.N):
                for j in range(self.N):
                    wx, wy = self.grid_to_world(i, j)
                    if math.hypot(wx - cx, wy - cy) < BLOCK_R:
                        grid[i][j] = 100

        # -------- Block D1 NO-ENTRY ZONE (only if not allowed) --------
        if not allow_d1:
            gx_min, gy_min = self.world_to_grid(self.D1_X_MIN, self.D1_Y_MIN)
            gx_max, gy_max = self.world_to_grid(self.D1_X_MAX, self.D1_Y_MAX)

            for gx in range(gx_min + 1, gx_max):
                for gy in range(gy_min + 1, gy_max):
                    if 0 <= gx < self.N and 0 <= gy < self.N:
                        grid[gx][gy] = 100

        return grid


    # ---------------- A* search ----------------
    def plan(self, start, goal, grid):
        sx, sy = self.world_to_grid(*start)
        gx, gy = self.world_to_grid(*goal)

        # --- FIX 3: explicitly allow goal cell ---
        if 0 <= gx < self.N and 0 <= gy < self.N:
            grid[gx][gy] = 0


        openq = []
        heapq.heappush(openq, (0, (sx, sy)))

        came_from = {}
        cost = {(sx, sy): 0}

        moves = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        while openq:
            _, cur = heapq.heappop(openq)

            if cur == (gx, gy):
                break

            for dx, dy in moves:
                nx, ny = cur[0] + dx, cur[1] + dy

                if not (0 <= nx < self.N and 0 <= ny < self.N):
                    continue
                if grid[nx][ny]:
                    continue

                new_cost = cost[cur] + math.hypot(dx, dy)

                if (nx, ny) not in cost or new_cost < cost[(nx, ny)]:
                    cost[(nx, ny)] = new_cost
                    priority = new_cost + math.hypot(gx - nx, gy - ny)
                    heapq.heappush(openq, (priority, (nx, ny)))
                    came_from[(nx, ny)] = cur

        # -------- reconstruct path --------
        path = []
        cur = (gx, gy)

        if cur not in came_from:
            return []

        while cur != (sx, sy):
            path.append(self.grid_to_world(*cur))
            cur = came_from[cur]

        path.reverse()
        return path
