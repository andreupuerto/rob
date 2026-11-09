import csv
import itertools
import math
import subprocess
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class autonavigation(Node):
    def __init__(self):
        super().__init__('autonavigation_node')
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qos_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_r)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_b)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_b)

        self.tf_buffer = tf2_ros.Buffer()  
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.points = {
            "A" : (0.7,0.92),
            "B" : (3.102,1.722),
            "C" : (2.078, 2.517),
            "D" : (5.285, 2.517),
            "E" : (5.082, 6.122),
            "F" : (5.679, 8.528),
            "G" : (5.482, 10.930),
            "Door" : (6.490, 11.682),
            "P_Base" : (3.340, 15.700),
            "Q" : (8.970, 14.497),
            "R" : (7.500, 16.170),
            "S" : (3.540, 14.497),
            "T" : (2.136, 15.300)
        }

        self.declare_parameter("start_point", "A")
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("path", ["B", "C", "D", "E", "F", "G", "Door", "Q"])
        self.start_point = self.get_parameter("start_point").value
        self.initial_x, self.initial_y = self.points[self.start_point]
        self.initial_yaw = self.get_parameter("start_yaw").value
        self.path = list(self.get_parameter("path").value)

        self.declare_parameter("final_yaw", math.pi)
        self.declare_parameter("start_delay", 5.0)
        self.declare_parameter("door_yaw", 0.0)
        self.declare_parameter("door_length", 0.6)
        self.final_yaw = self.get_parameter("final_yaw").value
        self.start_delay = self.get_parameter("start_delay").value
        self.door_yaw = self.get_parameter("door_yaw").value
        self.door_length = self.get_parameter("door_length").value

        self.max_linear_speed = 0.2
        self.wall_follow_speed = 0.1
        self.max_angular_speed = 0.8

        self.safe_distance = 0.25
        self.wall_distance = 0.20
        self.goal_tolerance = 0.08
        self.mline_tolerance = 0.05
        self.leave_margin = 0.15
        self.yaw_tol = 0.3
        self.away_dist = 0.5
        self.back_dist = 0.2
        self.emergency_distance = 0.13
        self.wait_time = 3.0
        self.turn_tolerance = 0.08
        self.door_speed = 0.1

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.x0 = None
        self.y0 = None
        self.yaw0 = None

        self.x = None
        self.y = None
        self.yaw = None
        self.start_x = None
        self.start_y = None
        self.goal_x = None
        self.goal_y = None
        self.goal_distance = None

        self.front = None
        self.right = None
        self.left = None
        self.closest = None
        self.goal_free = 0.0

        self.following_wall = False
        self.hit_distance = 0.0
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.left_hit = False
        self.unreachable = False
        self.wait_since = None


    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def scan_callback(self, msg):
        front = float('inf')
        right = float('inf')
        left = float('inf')
        closest = float('inf')
        goal_free = float('inf')

        goal_direction = 0.0
        if self.x is not None and self.goal_x is not None:
            goal_angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            goal_direction = math.degrees(normalize_angle(goal_angle - self.yaw))

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                continue
            closest = min(closest, r)
            angle = math.degrees(
                normalize_angle(msg.angle_min + i * msg.angle_increment))
            if -30 <= angle <= 30:
                front = min(front, r)
            elif -110 <= angle < -30:
                right = min(right, r)
            elif 30 < angle <= 110:
                left = min(left, r)
            if abs((angle - goal_direction + 180) % 360 - 180) <= 45:
                goal_free = min(goal_free, r)

        self.front = front
        self.right = right
        self.left = left
        self.closest = closest
        self.goal_free = goal_free

    def detect_pillars(self, msg):
        pass

    def update_landmarks(self):
        pass

    def log_row(self):
        pass

    def slam_pose(self):
        pass

    def update_pose(self):
        if self.odom_x is None:
            return False
        if self.x0 is None:
            self.x0 = self.odom_x
            self.y0 = self.odom_y
            self.yaw0 = self.odom_yaw
        a = self.initial_yaw - self.yaw0
        dx = self.odom_x - self.x0
        dy = self.odom_y - self.y0
        self.x = self.initial_x + dx * math.cos(a) - dy * math.sin(a)
        self.y = self.initial_y + dx * math.sin(a) + dy * math.cos(a)
        self.yaw = normalize_angle(self.odom_yaw - self.yaw0 + self.initial_yaw)
        return True

    def publish_velocity(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.publisher.publish(msg)

    def stop(self):
        self.publish_velocity(0.0, 0.0)
        rclpy.spin_once(self, timeout_sec=0.1)

    def set_goal(self, name):
        self.goal_x, self.goal_y = self.points[name]
        self.start_x = self.x
        self.start_y = self.y
        self.following_wall = False
        self.left_hit = False
        self.unreachable = False
        self.wait_since = None


    def follow_line(self):
        goal_angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        error = normalize_angle(goal_angle - self.yaw)
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, 2.0 * error))

        if abs(error) > self.yaw_tol:
            self.publish_velocity(0.0, angular)
        elif self.front < self.safe_distance and self.front < self.goal_distance:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.wait_since is None:
                self.wait_since = now
                self.get_logger().info(f'Obstaculo a {self.front:.2f} m: espero')
            if now - self.wait_since < self.wait_time:
                self.publish_velocity(0.0, 0.0)
                return
            self.wait_since = None
            self.following_wall = True
            self.hit_distance = self.goal_distance
            self.hit_x = self.x
            self.hit_y = self.y
            self.left_hit = False
            self.get_logger().info(f'Obstaculo a {self.front:.2f} m: rodeo la pared')
            self.stop()
        else:
            self.wait_since = None
            self.publish_velocity(self.max_linear_speed, angular)


    def follow_wall(self):
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        mline_distance = abs(dx * (self.y - self.start_y) - dy * (self.x - self.start_x)) \
            / math.hypot(dx, dy)

        hit_dist = math.hypot(self.x - self.hit_x, self.y - self.hit_y)
        if hit_dist > self.away_dist:
            self.left_hit = True

        if mline_distance < self.mline_tolerance \
                and self.goal_distance < self.hit_distance - self.leave_margin:
            self.following_wall = False
            self.stop()
        elif self.goal_free > min(self.goal_distance, 1.0) \
                and self.goal_distance < self.hit_distance:
            self.following_wall = False
            self.stop()
        elif self.left_hit and hit_dist < self.back_dist:
            self.unreachable = True
        elif self.front < self.safe_distance:
            self.publish_velocity(0.0, self.max_angular_speed)
        else:
            error = self.wall_distance - min(self.right, self.wall_distance + 0.3)
            angular = max(-self.max_angular_speed, min(self.max_angular_speed, 2.5 * error))
            self.publish_velocity(self.wall_follow_speed, angular)


    def go_to_goal(self):
        self.goal_distance = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if self.goal_distance < self.goal_tolerance:
            self.stop()
            return True
        if self.unreachable:
            self.stop()
            return True
        if self.closest < self.emergency_distance:
            self.publish_velocity(0.0, 0.0)
            return False
        if self.following_wall:
            self.follow_wall()
        else:
            self.follow_line()
        return False


    def go_path(self, names):
        for name in names:
            self.set_goal(name)
            self.get_logger().info(f'Voy a {name} {self.points[name]}')
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if not self.update_pose():
                    continue
                if self.go_to_goal():
                    break
            if not rclpy.ok():
                return False
            if self.unreachable:
                self.get_logger().warn(f'{name} inalcanzable: abandono la ruta')
                return False
            self.get_logger().info(f'Llegado a {name}: x={self.x:.2f} y={self.y:.2f}')
            if name == "Door":
                self.cross_door()
        return True

    def cross_door(self):
        self.turn_to_yaw(self.door_yaw)
        door_x = self.x
        door_y = self.y
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.update_pose():
                continue
            if math.hypot(self.x - door_x, self.y - door_y) >= self.door_length:
                break
            if self.closest < self.emergency_distance or self.front < self.safe_distance:
                self.publish_velocity(0.0, 0.0)
                continue
            error = min(self.left, 1.0) - min(self.right, 1.0)
            angular = max(-0.5, min(0.5, 1.5 * error))
            self.publish_velocity(self.door_speed, angular)
        self.stop()


    def turn_to_yaw(self, target_yaw):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.update_pose():
                continue
            error = normalize_angle(target_yaw - self.yaw)
            if abs(error) <= self.turn_tolerance:
                break
            if self.closest < self.emergency_distance:
                self.publish_velocity(0.0, 0.0)
                continue
            angular = max(-self.max_angular_speed, min(self.max_angular_speed, 2.0 * error))
            angular = math.copysign(max(abs(angular), 0.15), error)
            self.publish_velocity(0.0, angular)
        self.stop()


    def countdown(self, seconds):
        end = self.get_clock().now().nanoseconds / 1e9 + seconds
        last = None
        while rclpy.ok():
            remaining = end - self.get_clock().now().nanoseconds / 1e9
            if remaining <= 0.0:
                break
            if math.ceil(remaining) != last:
                last = math.ceil(remaining)
                self.get_logger().info(f'Empiezo en {last} s')
            rclpy.spin_once(self, timeout_sec=0.05)
            self.update_pose()


    def look_around(self, seconds):
        pass


    def save_map(self):
        pass


    def square_from_pair(self, points, min_side=0.28, max_side=0.50):
        pass


    def dock(self):
        pass


    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.update_pose() and self.front is not None:
                break
        self.get_logger().info(
            f'Salida {self.start_point}: x={self.x:.2f} y={self.y:.2f} '
            f'yaw={math.degrees(self.yaw):.1f} grados')
        self.countdown(self.start_delay)

        if self.go_path(self.path):
            self.turn_to_yaw(self.final_yaw)
            self.get_logger().info('Fase I completada')
        else:
            self.get_logger().warn('Fase I interrumpida')
       

    def close_log(self):
        pass
    


def main(args=None):
    print('Hi from autonavigation_11j.')
    rclpy.init(args=args)
    node = autonavigation()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(TwistStamped())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
