import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.clock import Clock
from geometry_msgs.msg import TwistStamped

class SquareTimed(Node):
    def __init__(self):
        super().__init__('square_timed_node')
        qos_profile_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # Publicador para mover el robot
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile_r)
        
        self.linear_speed = 0.15    # m/s
        self.angular_speed = 0.3    # rad/s
        self.side_length = 1.0      # 1 meter

    def publish_velocity(self, linear, angular):  
        move_msg = TwistStamped()
        move_msg.header.stamp = Clock().now().to_msg()
        move_msg.header.frame_id = ''
        move_msg.twist.linear.x = linear
        move_msg.twist.angular.z = angular
        self.publisher.publish(move_msg)

    def stop(self):
        self.publish_velocity(0.0, 0.0)
        time.sleep(0.5)

    def move_during(self, linear, angular, duration):
        start = time.time()
        while time.time() - start < duration:
            self.publish_velocity(linear, angular)
            time.sleep(0.05)
        self.stop()

    def move_forward(self, distance):
        duration = distance / self.linear_speed
        self.move_during(self.linear_speed, 0.0, duration)

    def turn(self, angle):
        duration = abs(angle) / self.angular_speed
        angular_direction = self.angular_speed if angle > 0 else -self.angular_speed
        self.move_during(0.0, angular_direction, duration)

    def run_square(self):
        for side in range(4):
            self.get_logger().info(f'Lado {side + 1}')
            self.move_forward(self.side_length)
            self.turn(math.pi/2)  # Girar 90 grados
        self.get_logger().info('Cuadrado completado')


def main(args=None):
    print('Hi from package_square_timed.')
    rclpy.init(args=args)
    node = SquareTimed()
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
