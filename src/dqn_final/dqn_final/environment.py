import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import numpy as np
from typing import Tuple
import math
import cv2


class TurtleBot3Env(Node):

    def __init__(self):
        super().__init__('turtlebot3_env')

        self.map_path = '/home/grace/Documents/Diplomado_Robotica/16_01_ws/src/stage_ros2/world/bitmaps/solid_cave.png'
        self.map_img = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)

        if self.map_img is None:
            self.get_logger().error(f"COULD NOT LOAD MAP FROM: {self.map_path}")
            self.img_height, self.img_width = 800, 800
        else:
            self.map_img = np.flipud(self.map_img)
            self.img_height, self.img_width = self.map_img.shape
            self.get_logger().info(f"Map loaded successfully: {self.img_width}x{self.img_height}")

        self.map_size_meters = 16.0
        self.resolution = self.img_width / self.map_size_meters

        self.spawn_x = -7.0
        self.spawn_y = -7.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 100)

        self.scan_sub = self.create_subscription(
            LaserScan, '/base_scan', self.scan_callback, 100
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom/sim', self.odom_callback, 100
        )

        self.reset_stage_client = self.create_client(Empty, '/reset_sim')

        self.scan_data = None
        self.position = (0.0, 0.0)
        self.yaw = 0.0

        self.goal_position = (4.0, 4.0)

        self.actions = {
            0: (0.16, 0.0),
            1: (0.0, 0.23),
            2: (0.0, -0.23),
            3: (0.085, 0.025),
            4: (0.085, -0.025),
        }

        self.collision_threshold = 0.2
        self.goal_threshold = 0.3
        self.goal_reached_flag = False

    def scan_callback(self, msg: LaserScan):
        self.scan_data = list(msg.ranges)

    def odom_callback(self, msg: Odometry):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        if self.distance_to_goal() < self.goal_threshold:
            self.goal_reached_flag = True

    def step(self, action: int) -> Tuple[np.ndarray, float, bool]:
        v, w = self.actions[action]
        self.send_velocity(v, w)

        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=0.1)
        while (self.get_clock().now() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)

        done = False
        reward = 0.0

        if self.is_collision():
            reward = -100.0
            done = True
            self.get_logger().info("Collision detected!")
        elif self.goal_reached_flag or self.distance_to_goal() < self.goal_threshold:
            reward = 200.0
            done = True
            self.send_velocity(0.0, 0.0)
            self.goal_reached_flag = True
            self.get_logger().info("Goal reached!")
        else:
            reward = self.compute_reward(action)

        return self.get_state(), reward, done

    def compute_reward(self, action: int) -> float:
        d = self.distance_to_goal()

        if hasattr(self, 'last_distance'):
            prog = (self.last_distance - d) * 18.0
        else:
            prog = 0.0

        self.last_distance = d

        if self.scan_data:
            obs = [x for x in self.scan_data if x < 4.9]
            min_obs = np.min(obs) if obs else 3.5
        else:
            min_obs = 3.5

        obs_pen = -5.0 * (0.5 - min_obs) if min_obs < 0.5 else 0.0
        act_pen = -0.01 if action in [1, 2] else 0.0
        time_pen = -0.25

        return prog + obs_pen + time_pen + act_pen

    def is_collision(self) -> bool:
        if self.scan_data is None:
            return False
        close = [r for r in self.scan_data if r < 4.9]
        return np.min(close) < self.collision_threshold if close else False

    def is_goal_reached(self) -> bool:
        return self.distance_to_goal() < self.goal_threshold

    def distance_to_goal(self) -> float:
        dx = self.goal_position[0] - self.position[0]
        dy = self.goal_position[1] - self.position[1]
        return math.hypot(dx, dy)

    def send_velocity(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def is_valid_point(self, x: float, y: float) -> bool:
        if self.map_img is None:
            return True

        px = int((x + self.map_size_meters / 2) * self.resolution)
        py = int((y + self.map_size_meters / 2) * self.resolution)

        if px < 0 or px >= self.img_width or py < 0 or py >= self.img_height:
            return False

        return self.map_img[py, px] > 127

    def reset(self, random_goal: bool = True) -> np.ndarray:
        self.send_velocity(0.0, 0.0)
        self.reset_world()
        self.goal_reached_flag = False

        if random_goal:
            for _ in range(100):
                rx = np.random.uniform(0.0, 7.0)
                ry = np.random.uniform(0.0, 7.0)

                wx = rx + self.spawn_x
                wy = ry + self.spawn_y

                if self.is_valid_point(wx, wy):
                    self.goal_position = (rx, ry)
                    self.get_logger().info(f"Goal: ({rx:.1f}, {ry:.1f})")
                    break
            else:
                self.goal_position = (2.0, 2.0)
                self.get_logger().warn("Goal Failed! Defaulting.")

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.02)

        self.last_distance = self.distance_to_goal()
        return self.get_state()

    def reset_world(self):
        if not self.reset_stage_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/reset_sim service not available')
            return
        req = Empty.Request()
        fut = self.reset_stage_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def get_state(self) -> np.ndarray:
        return None
