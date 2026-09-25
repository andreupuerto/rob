import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.clock import Clock
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class ObstacleStop(Node):
    def __init__(self):
        super().__init__('obstacle_stop_node')
        # 1. Definimos el perfil de QoS compatible con el LiDAR del TB3
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_profile_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        # Publicador para mover el robot
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile_r)
        # Subscriptor al LiDAR
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_b)
        
        self.safe_distance = 0.25  # 25 cm
        self.linear_speed = 0.15    # m/s
        #move_msg = TwistStamped()
        #move_msg.linear.x = self.linear_speed
        #self.publisher.publish(move_msg)

    def scan_callback(self, msg):
        # El LiDAR del TB3 tiene 360 puntos. El índice 0 es el frente.
        # Comprobamos un rango pequeño al frente (de -15 a 15 grados)
        front_ranges = msg.ranges[0:15] + msg.ranges[345:359]
        
        # Filtramos valores infinitos o erróneos (0.0)
        valid_ranges = [r for r in front_ranges if r > msg.range_min]
        
        min_distance = min(valid_ranges) if valid_ranges else float('inf')

        move_msg = TwistStamped()
        move_msg.header.stamp = Clock().now().to_msg()
        move_msg.header.frame_id = ''
        move_msg.twist.linear.x = 0.0
        move_msg.twist.linear.y = 0.0
        move_msg.twist.linear.z = 0.0
        move_msg.twist.angular.x = 0.0
        move_msg.twist.angular.y = 0.0
        move_msg.twist.angular.z = 0.0

        if min_distance > self.safe_distance:
            move_msg.twist.linear.x = self.linear_speed
            self.get_logger().info(f'Camino despejado. Distancia: {min_distance:.2f}m')
        else:
            move_msg.twist.linear.x = 0.0
            self.get_logger().warn(f'¡OBSTÁCULO DETECTADO! Parando a {min_distance:.2f}m')

        self.publisher.publish(move_msg)
       

def main(args=None):
    print('Hi from package_go_stop.')
    rclpy.init(args=args)
    node = ObstacleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Siempre es bueno mandar un último mensaje de parada
        node.publisher.publish(TwistStamped())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
