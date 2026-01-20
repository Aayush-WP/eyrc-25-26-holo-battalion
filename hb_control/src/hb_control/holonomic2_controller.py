#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hb_interfaces.srv import Attach
from hb_interfaces.msg import Poses2D, BotCmd, BotCmdArray
import math

from hb_control.path_planner import AStarPlanner

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray



def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.i = 0.0
        self.prev = 0.0
        self.prev_time = None

    def reset(self):
        self.i = 0.0
        self.prev = 0.0
        self.prev_time = None

    def step(self, error, t):
        dt = 1e-3 if self.prev_time is None else max(t - self.prev_time, 1e-3)
        self.i += error * dt
        d = (error - self.prev) / dt
        self.prev = error
        self.prev_time = t
        u = self.kp * error + self.ki * self.i + self.kd * d
        return max(-self.limit, min(self.limit, u))


class HolonomicController(Node):
    def __init__(self):
        super().__init__('holonomic_controller')

        # ---------------- IDs ----------------
        self.BOT_ID = 2
        self.BOX_ID = 30
        

        # ---------------- Drop Zone ----------------
        self.D1_X = 1250.0
        self.D1_Y = 1072.2

        # ---------------- Limits ----------------
        # self.stop_radius = 20.0

        # ---------------- State ----------------
        self.state = "START"

        # ---------------- Poses ----------------
        self.bot_pose = None
        self.crate_pose = None
        self.start_x = None
        self.start_y = None

        self.tx = None
        self.ty = None

        self.default_elbow = 25.0
        self.pickup_elbow = 5.0
        self.default_base = 10.0
        self.drop_base =  10.0


        # ---------------- PID ----------------
        self.pid_x = PID(6.0, 0.0, 0.0, 60.0)
        self.pid_y = PID(6.0, 0.0, 0.0, 60.0)


        # ---------------- Geometry ----------------
        self.MARKER_SIZE = 40.0
        self.STOP_RADIUS = 110.0
        self.BRAKE_RADIUS = 1.0   # start slowing down here

        self.pickup_start_time = None
        self.pickup_start_time_2 = None
        self.PICKUP_DELAY = 3.0  # seconds (tune this)



        # ---------------- ROS ----------------
        self.create_subscription(Poses2D, '/bot_pose', self.bot_cb, 10)
        self.create_subscription(Poses2D, '/crate_pose', self.crate_cb, 10)

        self.cmd_pub = self.create_publisher(
            BotCmdArray,
            '/bot_cmd',
            10
        )

        self.grid_pub = self.create_publisher(OccupancyGrid, '/planner/grid', 1)
        self.path_pub = self.create_publisher(Path, '/planner/path', 1)
        self.sg_pub = self.create_publisher(PoseArray, '/planner/start_goal', 1)


        self.planner = AStarPlanner()
        self.path = []
        self.wp_idx = 0
        self.last_crates = []



        # ---------------- Attach Service Client ----------------
        self.attach_cli = self.create_client(Attach, '/attach')
        while not self.attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /attach service...")

        # ---------------- Timer ----------------
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("✅ Holonomic Controller Ready")

    # =========================================================
    def bot_cb(self, msg):
        for p in msg.poses:
            if p.id == self.BOT_ID:
                self.bot_pose = p
                if self.start_x is None:
                    self.start_x = 1601.0
                    self.start_y = 147.0

    def crate_cb(self, msg):
        self.last_crates = msg.poses
        for p in msg.poses:
            if p.id == self.BOX_ID:
                self.crate_pose = p

    def compute_pickup_point(self, crate):
        yaw = math.radians(crate.w)

        MARKER_SIZE = self.MARKER_SIZE      # 40.0
        OFFSET_X = 45.5                      # pickup distance
        OFFSET_Y = 45.5
        # ---- shift from corner to center ----
        dx = MARKER_SIZE / 2.0
        dy = MARKER_SIZE / 2.0

        cx = crate.x + dx * math.cos(yaw) - dy * math.sin(yaw)
        cy = crate.y + dx * math.sin(yaw) + dy * math.cos(yaw)

        # ---- apply offset along marker normal ----
        px = cx - OFFSET_X 
        py = cy - OFFSET_Y
        return px, py
    
    def publish_planner_debug(self, grid, path, start, goal):
        # -------- GRID --------
        og = OccupancyGrid()
        og.info.resolution = self.planner.resolution
        og.info.width = self.planner.width
        og.info.height = self.planner.height
        og.data = [cell for row in grid for cell in row]
        self.grid_pub.publish(og)

        # -------- PATH --------
        p = Path()
        for x, y in path:
            ps = PoseStamped()
            ps.pose.position.x = x
            ps.pose.position.y = y
            p.poses.append(ps)
        self.path_pub.publish(p)

        # -------- START + GOAL --------
        pa = PoseArray()
        for x, y in [start, goal]:
            pose = PoseStamped().pose
            pose.position.x = x
            pose.position.y = y
            pa.poses.append(pose)
        self.sg_pub.publish(pa)



    # =========================================================
    def publish_cmd(self, m1, m2, m3, a1, a2):
        cmd = BotCmd()
        cmd.id = self.BOT_ID
        cmd.m1 = float(m1)
        cmd.m2 = float(m2)
        cmd.m3 = float(m3)
        cmd.base = float(a1)
        cmd.elbow = float(a2)

        msg = BotCmdArray()
        msg.cmds.append(cmd)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0, 0.0, self.drop_base, self.default_elbow)

    # =========================================================
    def drive_to(self, tx, ty, a1, a2):
        bx, by = self.bot_pose.x, self.bot_pose.y
        dx, dy = tx - bx, ty - by

        center_dist = math.hypot(dx, dy)
        edge_dist = center_dist - self.MARKER_SIZE

        # ---------- STOP ZONE ----------
        if edge_dist <= self.STOP_RADIUS:
            self.stop_robot()
            return True

        # ---------- BRAKE SCALE ----------
        if edge_dist < self.BRAKE_RADIUS:
            scale = edge_dist / self.BRAKE_RADIUS
            scale = max(0.2, min(1.0, scale))  # prevent stall
        else:
            scale = 1.0

        # ---------- PID ----------
        now = self.get_clock().now().nanoseconds / 1e9
        vx = self.pid_x.step(dx, now) * scale
        vy = self.pid_y.step(dy, now) * scale

        yaw = math.radians(self.bot_pose.w)
        c, s = math.cos(yaw), math.sin(yaw)

        vx_r = c * vx + s * vy
        vy_r = -s * vx + c * vy

        m1 = -math.sin(math.radians(90))  * vx_r + math.cos(math.radians(90))  * vy_r
        m2 = -math.sin(math.radians(210)) * vx_r + math.cos(math.radians(210)) * vy_r
        m3 = -math.sin(math.radians(330)) * vx_r + math.cos(math.radians(330)) * vy_r

        self.publish_cmd(m1, m2, m3, a1, a2)
        return False


    # =========================================================
    def call_attach(self, value: bool):
        req = Attach.Request()
        req.bot_id = self.BOT_ID
        req.attach = value
        self.attach_cli.call_async(req)
        self.get_logger().info(f"{'ATTACH' if value else 'DETACH'} command sent")

    # =========================================================
    def control_loop(self):
        if self.bot_pose is None:
            return

        if self.state == "START":
            if self.crate_pose:
                self.pid_x.reset()
                self.pid_y.reset()
                self.state = "MOVE_TO_CRATE"
                self.get_logger().info("START  STATE DONE")
                self.get_logger().info("MOVE_TO_CRATE STATE START")

        elif self.state == "MOVE_TO_CRATE":

            # ---------- compute left-facing pickup point ----------
            pickup = self.compute_pickup_point(self.crate_pose)

            # ---------- build obstacle grid (ignore target crate) ----------
            grid = self.planner.build_grid(
                self.last_crates,
                ignore_id=self.BOX_ID
            )

            # ---------- plan A* path ----------
            self.path = self.planner.plan(
                start=(self.bot_pose.x, self.bot_pose.y),
                goal=pickup,
                grid=grid
            )

            # ---------- publish debug info ----------
            self.publish_planner_debug(
                grid=grid,
                path=self.path,
                start=(self.bot_pose.x, self.bot_pose.y),
                goal=pickup
            )

            # ---------- safety check ----------
            if not self.path:
                self.get_logger().error("❌ A* failed to find path to crate")
                self.stop_robot()
                return

            self.wp_idx = 0


            self.get_logger().info(
                f"Planner debug: "
                f"start=({self.bot_pose.x:.1f},{self.bot_pose.y:.1f}) "
                f"goal=({pickup[0]:.1f},{pickup[1]:.1f}) "
                f"path_len={len(self.path)}"
            )

            self.state = "FOLLOW_PATH"

            self.get_logger().info("MOVE_TO_CRATE → FOLLOW_PATH")


        elif self.state == "FOLLOW_PATH":
            wx, wy = self.path[self.wp_idx]
            if self.drive_to(wx, wy, self.default_base, self.default_elbow):
                self.wp_idx += 1
                if self.wp_idx >= len(self.path):
                    self.tx, self.ty = self.crate_pose.x, self.crate_pose.y
                    self.get_logger().info(f"WP {self.wp_idx}: dist={math.hypot(wx-self.bot_pose.x, wy-self.bot_pose.y):.1f}")
                    self.stop_robot()
                    self.state = "ALIGN_TO_CRATE"
                    self.get_logger().info("FOLLOW_PATH STATE DONE")
                    self.get_logger().info("ALIGN_TO_CRATE STATE START")

        elif self.state == "ALIGN_TO_CRATE":
            bx, by = self.bot_pose.x, self.bot_pose.y
            tx, ty = self.tx, self.ty
            byaw = math.radians(self.bot_pose.w)

            vec = math.atan2(ty - by, tx - bx)
            desired = wrap_angle(vec - math.pi / 2)
            err = wrap_angle(desired - byaw)

            if abs(err) < 0.5:
                self.publish_cmd(0.0, 0.0, 0.0, self.default_base, self.default_elbow)
                self.pid_x.reset()
                self.pid_y.reset()
                print("desired error range achieved", err)
                self.state = "PICKUP"
                self.get_logger().info("ALIGN_TO_CRATE STATE DONE")
                self.get_logger().info("PICKUP STATE START")
            else:
                K = 20.0
                MIN_ROT = 20.0   # tune experimentally

                rot = err * K

                if abs(rot) < MIN_ROT:
                    rot = math.copysign(MIN_ROT, rot)
                self.publish_cmd(rot, rot, rot, self.default_base, self.default_elbow)
                print(err)

        elif self.state == "PICKUP":

            

            if self.pickup_start_time is None:
                # first entry into PICKUP
                self.pickup_start_time = self.get_clock().now().nanoseconds / 1e9
                self.publish_cmd(
                    0, 0, 0,
                    self.default_base,
                    self.pickup_elbow
                )
                
                return

            now = self.get_clock().now().nanoseconds / 1e9

            if now - self.pickup_start_time < self.PICKUP_DELAY:
                # keep holding position
                self.publish_cmd(
                    0, 0, 0,
                    self.default_base,
                    self.pickup_elbow
                )
                return
            
            self.call_attach(True)
            
            

            # delay done → move on
            self.pickup_start_time = None
            self.pid_x.reset()
            self.pid_y.reset()
            self.STOP_RADIUS = 20
            self.state = "MOVE_TO_D1"
            self.get_logger().info("PICKUP STATE DONE")
            self.get_logger().info("MOVE_TO_D1 STATE START")



        elif self.state == "MOVE_TO_D1":

            # ---------- build obstacle grid (crate is attached, ignore all) ----------
            grid = self.planner.build_grid(
                self.last_crates,
                ignore_id=None
            )

            # ---------- plan A* path to drop zone ----------
            self.path = self.planner.plan(
                start=(self.bot_pose.x, self.bot_pose.y),
                goal=(self.D1_X, self.D1_Y),
                grid=grid
            )

            # ---------- publish debug info ----------
            self.publish_planner_debug(
                grid=grid,
                path=self.path,
                start=(self.bot_pose.x, self.bot_pose.y),
                goal=(self.D1_X, self.D1_Y)
            )

            # ---------- safety check ----------
            if not self.path:
                self.get_logger().error("❌ A* failed to find path to D1")
                self.stop_robot()
                return

            self.wp_idx = 0
            self.state = "FOLLOW_PATH_D1"

            self.get_logger().info("MOVE_TO_D1 → FOLLOW_PATH_D1")


        elif self.state == "FOLLOW_PATH_D1":
            wx, wy = self.path[self.wp_idx]
            if self.drive_to(wx, wy, self.drop_base, self.default_elbow):
                print("waypoint reached")
                self.wp_idx += 1
                if self.wp_idx >= len(self.path):
                    
                    self.STOP_RADIUS = 0.0
                    self.state = "ALIGN_AT_DROP"
                    self.get_logger().info("FOLLOW_PATH_D1 STATE DONE")
                    self.get_logger().info("ALIGN_AT_DROP STATE START")


        elif self.state == "ALIGN_AT_DROP":
            byaw = math.radians(self.bot_pose.w)

            # target yaw = 0 rad
            err = wrap_angle(0.0 - byaw)

            if abs(err) < 0.5:  # ~2 deg tolerance
                self.publish_cmd(0.0, 0.0, 0.0, self.default_base, self.default_elbow)
                self.pid_x.reset()
                self.pid_y.reset()

                self.state = "DROP"
                self.get_logger().info("ALIGN_AT_DROP DONE")
                self.get_logger().info("DROP START")
                return

            K = 20.0
            rot = K * err
            print(err)

            # minimum angular speed
            MIN_ROT = math.radians(20)
            if abs(rot) < MIN_ROT:
                rot = math.copysign(MIN_ROT, rot)
                

            self.publish_cmd(rot, rot, rot,
                            self.default_base,
                            self.default_elbow)
            

        elif self.state == "DROP":

            if self.pickup_start_time is None:
                # first entry into PICKUP
                self.pickup_start_time = self.get_clock().now().nanoseconds / 1e9
                self.publish_cmd(
                    0, 0, 0,
                    self.drop_base,
                    self.pickup_elbow
                )
                return

            now = self.get_clock().now().nanoseconds / 1e9

            if now - self.pickup_start_time < self.PICKUP_DELAY:
                # keep holding position
                self.publish_cmd(
                    0, 0, 0,
                    self.drop_base,
                    self.pickup_elbow
                )
                return
            self.call_attach(False)
            # delay done → move on
            self.pickup_start_time = None
            self.pid_x.reset()
            self.pid_y.reset()
            self.STOP_RADIUS = 0
            self.state = "PLAN_RETURN_HOME"
            self.get_logger().info("DROP STATE DONE")
            self.get_logger().info("PLAN_RETURN_HOME STATE START")
            


        # -------- PLAN RETURN HOME (NEW) --------
        elif self.state == "PLAN_RETURN_HOME":
            # ---------- build obstacle grid (crate is attached, ignore all) ----------
            grid = self.planner.build_grid(
                self.last_crates,
                ignore_id=None,
                allow_d1=True
            )

            # ---------- plan A* path to drop zone ----------
            self.path = self.planner.plan(
                start=(self.bot_pose.x, self.bot_pose.y),
                goal=(self.start_x, self.start_y),
                grid=grid
            )

            # ---------- publish debug info ----------
            self.publish_planner_debug(
                grid=grid,
                path=self.path,
                start=(self.bot_pose.x, self.bot_pose.y),
                goal=(self.start_x, self.start_y)
            )

            # ---------- safety check ----------
            if not self.path:
                self.get_logger().error("❌ A* failed to find path to D1")
                self.stop_robot()
                return

            self.wp_idx = 0
            self.state = "MOVE_TO_START"
            self.get_logger().info("PLAN_RETURN_HOME STATE DONE")
            self.get_logger().info("MOVE_TO_START STATE START")


        elif self.state == "MOVE_TO_START":
            # self.call_attach(False)
            wx, wy = self.path[self.wp_idx]
            if self.drive_to(wx, wy, self.default_base, self.default_elbow):
                self.wp_idx += 1
                if self.wp_idx >= len(self.path):
                    self.state = "STOP_AT_START"
                    self.get_logger().info("MOVE_TO_START STATE DONE")
                    self.get_logger().info("STOP_AT_START STATE START")

        elif self.state == "STOP_AT_START":
            self.stop_robot()
            self.state = "ALIGN_AT_START"
            self.get_logger().info("STOP_AT_START STATE DONE")
            self.get_logger().info("ALIGN_AT_START STATE START")

        elif self.state == "ALIGN_AT_START":
            byaw = math.radians(self.bot_pose.w)

            # target yaw = 0 rad
            err = wrap_angle(0.0 - byaw)

            if abs(err) < math.radians(2.0):  # ~2 deg tolerance
                self.publish_cmd(0.0, 0.0, 0.0,
                                self.default_base,
                                self.default_elbow)
                self.state = "COMPLETE"
                self.get_logger().info("ALIGN_AT_START DONE")
                return

            K = 20.0
            rot = K * err
            print(err)

            # minimum angular speed
            MIN_ROT = math.radians(20)
            if abs(rot) < MIN_ROT:
                rot = math.copysign(MIN_ROT, rot)
                

            self.publish_cmd(rot, rot, rot,
                            self.default_base,
                            self.default_elbow)


        elif self.state == "COMPLETE":
            self.stop_robot()
            self.get_logger().info("COMPLETED")



        


def main():
    rclpy.init()
    node = HolonomicController()
    rclpy.spin(node)
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
