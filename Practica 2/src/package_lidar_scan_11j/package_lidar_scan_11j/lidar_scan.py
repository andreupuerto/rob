import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class LidarScan(Node):
    def __init__(self):
        super().__init__('lidar_scan_node')
        # Perfil de QoS compatible con el LiDAR del TB3
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        # Subscriptor al LiDAR
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_b)

        self.num_readings = 10     # N lecturas
        self.readings = []         # una lista de distancias por lectura
        self.done = False

    def scan_callback(self, msg):
        if self.done:
            return
        self.readings.append(list(msg.ranges))
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.get_logger().info(f'Lectura {len(self.readings)}/{self.num_readings}')
        if len(self.readings) == self.num_readings:
            self.analyze()
            self.done = True

    def analyze(self):
        print(f'{"Ángulo":>7} | {"Mín":>6} | {"Máx":>6} | {"Media":>6} | Válidas')
        for i in range(len(self.readings[0])):
            angle = math.degrees(self.angle_min + i * self.angle_increment)
            values = [reading[i] for reading in self.readings]
            valid = [r for r in values
                     if math.isfinite(r) and self.range_min <= r <= self.range_max]
            if valid:
                print(f'{angle:7.1f} | {min(valid):6.3f} | {max(valid):6.3f} | '
                      f'{sum(valid) / len(valid):6.3f} | {len(valid)}/{len(values)}')
            else:
                print(f'{angle:7.1f} | {"-":>6} | {"-":>6} | {"-":>6} | 0/{len(values)}')

def main(args=None):
    print('Hi from package_lidar_scan.')
    rclpy.init(args=args)
    node = LidarScan()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
