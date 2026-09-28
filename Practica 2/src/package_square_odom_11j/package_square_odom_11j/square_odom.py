import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.clock import Clock
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class SquareOdom(Node):
    def __init__(self):
        super().__init__('square_odom_node')
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)


        # Publicador para mover el robot
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_b)

        self.x = None
        self.y = None
        self.yaw = None
        
        self.linear_speed = 0.15    # m/s
        self.angular_speed = 0.3    # rad/s
        self.side_length = 1.0      # 1 meter

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


    def publish_velocity(self, linear, angular):  
        move_msg = TwistStamped()
        move_msg.header.stamp = Clock().now().to_msg()
        move_msg.header.frame_id = ''
        move_msg.twist.linear.x = linear
        move_msg.twist.angular.z = angular
        self.publisher.publish(move_msg)

    def wait(self, seconds):
        start = time.time()
        while time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.publish_velocity(0.0, 0.0)
        self.wait(0.5)

    @staticmethod
    def normalize_angle(angle):
        # Deja el ángulo entre -pi y pi (evita el salto de 180 a -180)
        return math.atan2(math.sin(angle), math.cos(angle))


    def move_forward(self, distance):
        x0, y0 = self.x, self.y
        travelled = 0.0
        while travelled < distance:
            self.publish_velocity(self.linear_speed, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            travelled = math.hypot(self.x - x0, self.y - y0)
        self.stop()
        self.get_logger().info(f'Recorrido: {travelled:.3f} m')

    def turn(self, angle):
        direction = self.angular_speed if angle > 0 else -self.angular_speed
        previous_yaw = self.yaw
        turned = 0.0
        while abs(turned) < abs(angle):
            self.publish_velocity(0.0, direction)
            rclpy.spin_once(self, timeout_sec=0.05)
            turned += self.normalize_angle(self.yaw - previous_yaw)
            previous_yaw = self.yaw
        self.stop()
        self.get_logger().info(f'Girado: {math.degrees(turned):.1f} grados')

    def run_square(self):

        while self.x is None:
            self.wait(0.1)
        start_x, start_y, start_yaw = self.x, self.y, self.yaw

        for side in range(4):
            self.get_logger().info(f'Lado {side + 1}')
            self.move_forward(self.side_length)
            self.turn(math.pi/2)  # Girar 90 grados
        self.get_logger().info('Cuadrado completado')


        error_pos = math.hypot(self.x - start_x, self.y - start_y)
        error_yaw = math.degrees(self.normalize_angle(self.yaw - start_yaw))
        self.get_logger().info(f'Error posición: {error_pos:.3f} m, error orientación: {error_yaw:.1f} grados')



def main(args=None):
    print('Hi from package_square_odom.')
    rclpy.init(args=args)
    node = SquareOdom()
    try:
        node.run_square()
    except KeyboardInterrupt:
        pass
    finally:
        # Siempre es bueno mandar un último mensaje de parada
        node.publisher.publish(TwistStamped())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
