import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class LidarScan(Node):
    def __init__(self):
        super().__init__('lidar_scan_node')
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_b)

        self.num_readings = 10
        self.readings = []
        self.done = False

    def scan_callback(self,msg):
        if len(self.readings) < self.num_readings:
            self.readings.append(list(msg.ranges))
            self.range_min = msg.range_min
            self.range_max = msg.range_max
            self.get_logger().info(f'Lectura {len(self.readings)}/{self.num_readings}')
            if len(self.readings) == self.num_readings:
                self.get_logger().info('Ya hay 10 lecturas')
                self.analyze_readings()
                self.done = True

    def analyze_readings(self):
        print("Análisis de las lecturas:\n")
        print("Ángulo\tMínimo\tMáximo\tPromedio\n")
        for i in range(360):
            values = [readings[i] for readings in self.readings]
            valid = [v for v in values if self.range_min <= v <= self.range_max]
            if valid:
                minimum = min(valid)
                maximum = max(valid)
                average = sum(valid) / len(valid)
                print(f'{i}\t{minimum:.3f}\t{maximum:.3f}\t{average:.3f}')
            else:
                print(f'{i}\tsin datos válidos')        

def main(args=None):
    rclpy.init(args=args)
    node = LidarScan()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()