import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class SquareOdom(Node):
    def __init__(self):
        super().__init__('square_odom_node')
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qos_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_r)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_b)

        self.x = None
        self.y = None
        self.yaw = None

        self.side_length = 1.0        # m

        # --- Velocidades lineales ---
        self.fast_linear = 0.20       # m/s  (zona lejos del objetivo)
        self.slow_linear = 0.05       # m/s  (zona cerca del objetivo)
        self.slow_distance = 0.20     # m    (a partir de aquí se ralentiza)

        # --- Velocidades angulares ---
        self.fast_angular = 0.8       # rad/s
        self.slow_angular = 0.15      # rad/s
        self.slow_angle = 0.35        # rad (~20°) (a partir de aquí se ralentiza)

        # --- Tolerancias ---
        self.dist_tol = 0.005         # m
        self.yaw_tol = 0.015          # rad (~0.9°)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def publish_velocity(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.publisher.publish(msg)

    def wait(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.publish_velocity(0.0, 0.0)
        self.wait(0.5)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def move_forward(self, distance):
        x0, y0 = self.x, self.y
        travelled = 0.0
        while rclpy.ok():
            remaining = distance - travelled
            if remaining <= self.dist_tol:
                break
            # Dos velocidades: rápida lejos, lenta cerca
            if remaining > self.slow_distance:
                v = self.fast_linear
            else:
                v = self.slow_linear
            self.publish_velocity(v, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            travelled = math.hypot(self.x - x0, self.y - y0)
        self.stop()
        self.get_logger().info(f'Recorrido: {travelled:.3f} m')

    def turn_to(self, target_yaw):
        while rclpy.ok():
            error = self.normalize_angle(target_yaw - self.yaw)
            if abs(error) <= self.yaw_tol:
                break
            # Dos velocidades: rápida lejos, lenta cerca
            if abs(error) > self.slow_angle:
                w = self.fast_angular
            else:
                w = self.slow_angular
            self.publish_velocity(0.0, math.copysign(w, error))
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()
        self.get_logger().info(
            f'Yaw actual: {math.degrees(self.yaw):.1f} grados')

    def run_square(self):
        while rclpy.ok() and self.x is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        start_x, start_y, start_yaw = self.x, self.y, self.yaw

        for side in range(4):
            self.get_logger().info(f'Lado {side + 1}')
            self.move_forward(self.side_length)
            target = self.normalize_angle(start_yaw + (side + 1) * math.pi / 2)
            self.turn_to(target)
        self.get_logger().info('Cuadrado completado')

        error_pos = math.hypot(self.x - start_x, self.y - start_y)
        error_yaw = math.degrees(self.normalize_angle(self.yaw - start_yaw))
        self.get_logger().info(
            f'Error posición: {error_pos:.3f} m, error orientación: {error_yaw:.1f} grados')


def main(args=None):
    print('Hi from package_square_odom.')
    rclpy.init(args=args)
    node = SquareOdom()
    try:
        node.run_square()
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(TwistStamped())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
