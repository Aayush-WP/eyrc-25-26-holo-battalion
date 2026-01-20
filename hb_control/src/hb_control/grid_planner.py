#!/usr/bin/env python3
"""
FULL GRID PLANNER — ORIGINAL + /planned_paths publisher
"""

import math
import heapq
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from hb_interfaces.msg import Poses2D, WorldPath


# ============================================================
# ---------------- CONFIG -----------------------------------
# ============================================================

ARENA_SIZE_MM = 2438.4
GRID_N = 24
CELL_MM = ARENA_SIZE_MM / GRID_N
WARP_SIZE_PX = 900

CORNER_IDS = [1, 3, 5, 7]

BOT_IDS = [4, 0, 2]
ASSIGNMENTS_PICK = {4: 14, 0: 12, 2: 30}

D3_DROP_CELL = (16, 22)
D1_LEFT_DROP_CELL = (11, 12)
D1_RIGHT_DROP_CELL = (13, 12)

BOT_RADIUS = 1  # 3x3 footprint


# ============================================================
# ---------------- GRID HELPERS ------------------------------
# ============================================================

def world_to_grid(x, y):
    gx = int(max(0, min(GRID_N - 1, x / CELL_MM)))
    gy = int(max(0, min(GRID_N - 1, y / CELL_MM)))
    return gx, gy


def grid_to_world(cell):
    gx, gy = cell
    return (gx + 0.5) * CELL_MM, (gy + 0.5) * CELL_MM


def footprint(cell):
    cx, cy = cell
    out = []
    for dx in range(-BOT_RADIUS, BOT_RADIUS + 1):
        for dy in range(-BOT_RADIUS, BOT_RADIUS + 1):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < GRID_N and 0 <= ny < GRID_N:
                out.append((nx, ny))
    return out


def neighbors(cell):
    x, y = cell
    for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
        nx, ny = x+dx, y+dy
        if 0 <= nx < GRID_N and 0 <= ny < GRID_N:
            yield (nx, ny)


def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])


# ============================================================
# ---------------- RESERVATION TABLE -------------------------
# ============================================================

class ReservationTable:
    def __init__(self):
        self.cell_time = {}
        self.edge_time = {}

    def free(self, cell, t):
        return (cell, t) not in self.cell_time

    def edge_free(self, a, b, t):
        return ((b, a), t) not in self.edge_time

    def reserve(self, bot_id, path):
        for t, cell in enumerate(path):
            for fc in footprint(cell):
                self.cell_time[(fc, t)] = bot_id
            if t > 0:
                self.edge_time[((path[t-1], cell), t)] = bot_id


# ============================================================
# ---------------- MAIN PLANNER NODE --------------------------
# ============================================================

class GridPathPlanner(Node):

    def __init__(self):
        super().__init__("grid_path_planner")

        self.bridge = CvBridge()
        self.latest_frame = None

        self.bots = {}
        self.crates = {}

        self.H = None

        # ROS
        self.create_subscription(Image, "/camera/image_raw", self.image_cb, 10)
        self.create_subscription(Poses2D, "/bot_pose", self.bot_cb, 10)
        self.create_subscription(Poses2D, "/crate_pose", self.crate_cb, 10)

        self.path_pub = self.create_publisher(WorldPath, "/planned_paths", 10)

        # ArUco
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters()
        )

        self.win = "GRID PLANNER"
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win, WARP_SIZE_PX, WARP_SIZE_PX)

        self.get_logger().info("Grid planner running")

    # ============================================================
    # ---------------- CALLBACKS ------------------------------
    # ============================================================

    def bot_cb(self, msg):
        for p in msg.poses:
            self.bots[p.id] = (p.x, p.y)

    def crate_cb(self, msg):
        for p in msg.poses:
            self.crates[p.id] = (p.x, p.y)

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.latest_frame = frame
        self.render()

    # ============================================================
    # ---------------- A* PLANNING -----------------------------
    # ============================================================

    def astar(self, start, goal, reservations):
        openq = []
        heapq.heappush(openq, (0, 0, start, 0))
        came = {}
        cost = {(start,0):0}

        while openq:
            _, g, cell, t = heapq.heappop(openq)

            if cell == goal:
                path = [cell]
                cur = (cell,t)
                while cur in came:
                    cur = came[cur]
                    path.append(cur[0])
                return list(reversed(path))

            for nxt in [cell] + list(neighbors(cell)):
                nt = t+1

                if not all(reservations.free(fc,nt) for fc in footprint(nxt)):
                    continue
                if nxt != cell and not reservations.edge_free(cell,nxt,nt):
                    continue

                ng = g+1
                state = (nxt,nt)
                if state not in cost or ng < cost[state]:
                    cost[state]=ng
                    came[state]=(cell,t)
                    heapq.heappush(openq,(ng+manhattan(nxt,goal),ng,nxt,nt))

        return None

    def plan_all(self):
        for bid,cid in ASSIGNMENTS_PICK.items():
            if bid not in self.bots or cid not in self.crates:
                return None

        reservations = ReservationTable()
        paths = {}

        for bid in BOT_IDS:
            start = world_to_grid(*self.bots[bid])
            pick = world_to_grid(*self.crates[ASSIGNMENTS_PICK[bid]])

            path1 = self.astar(start,pick,reservations)
            if not path1:
                continue
            reservations.reserve(bid,path1)

            if bid == 4:
                goal = D3_DROP_CELL
            elif bid == 0:
                goal = D1_LEFT_DROP_CELL
            else:
                goal = D1_RIGHT_DROP_CELL

            path2 = self.astar(path1[-1],goal,reservations)
            full = path1 + (path2[1:] if path2 else [])
            reservations.reserve(bid,full)

            paths[bid]=full

        return paths

    # ============================================================
    # ---------------- PUBLISH PATHS ----------------------------
    # ============================================================

    def publish_paths(self, planned):
        for bid,grid_path in planned.items():
            msg = WorldPath()
            msg.bot_id = bid
            for cell in grid_path:
                x,y = grid_to_world(cell)
                msg.xs.append(float(x))
                msg.ys.append(float(y))
            self.path_pub.publish(msg)

    # ============================================================
    # ---------------- DRAW ------------------------------------
    # ============================================================

    def render(self):
        if self.latest_frame is None:
            return

        planned = self.plan_all()
        if planned:
            self.publish_paths(planned)

        vis = np.zeros((WARP_SIZE_PX,WARP_SIZE_PX,3),dtype=np.uint8)

        for i in range(GRID_N):
            x = int(i*WARP_SIZE_PX/GRID_N)
            cv2.line(vis,(x,0),(x,WARP_SIZE_PX),(50,50,50),1)
            cv2.line(vis,(0,x),(WARP_SIZE_PX,x),(50,50,50),1)

        if planned:
            for bid,path in planned.items():
                color = (255,0,255) if bid==4 else (0,255,255) if bid==0 else (0,255,0)
                pts=[]
                for c in path:
                    x=int((c[0]+0.5)*WARP_SIZE_PX/GRID_N)
                    y=int((c[1]+0.5)*WARP_SIZE_PX/GRID_N)
                    pts.append((x,y))
                for i in range(1,len(pts)):
                    cv2.line(vis,pts[i-1],pts[i],color,3)

        cv2.putText(vis,"GRID PLANNER RUNNING",(20,40),
                    cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)

        cv2.imshow(self.win,vis)
        cv2.waitKey(1)


# ============================================================
# ---------------- MAIN --------------------------------------
# ============================================================

def main():
    rclpy.init()
    node = GridPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
