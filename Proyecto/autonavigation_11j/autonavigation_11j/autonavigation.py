import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
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

        # parametros
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 10.0)
        self.declare_parameter("goal_yaw", 0.0)

        # velocidades
        self.max_linear_speed = 0.25 # m/s
        self.min_linear_speed = 0.1
        self.max_angular_speed = 0.8 # rad/s
        self.min_angular_speed = 0.2

        # estado actual
        self.x = None
        self.y = None
        self.yaw = None
        self.following = None # punto q esta siguiendo el robot
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        # distancias
        self.detected_wall = False # si ha detectado un obstaculo o no
        self.safe_distance = 0.25
        self.wall_distance = 0.2

        # LiDAR
        self.front = None
        self.right = None

        # Odometry


        # nodes

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.start_yaw = self.yaw
            c = math.cos(self.start_yaw)
            s = math.sin(self.start_yaw)
            self.goal_x = self.start_x + self.goal_dx * c - self.goal_dy * s
            self.goal_y = self.start_y + self.goal_dx * s + self.goal_dy * c

    def scan_callback(self, msg):
        front = float('inf')
        right = float('inf')
        goal_free = float('inf')

        goal_direction = 0.0
        if self.x is not None:
            goal_angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            goal_direction = math.degrees(normalize_angle(goal_angle - self.yaw))

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                continue
            angle = math.degrees(
                normalize_angle(msg.angle_min + i * msg.angle_increment))
            if -30 <= angle <= 30:
                front = min(front, r)
            elif -110 <= angle < -30:
                right = min(right, r)
            if abs((angle - goal_direction + 180) % 360 - 180) <= 45:
                goal_free = min(goal_free, r)

        self.front = front
        self.right = right
        self.goal_free = goal_free

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


    def go_to_goal(self):
        distance = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if distance < self.goal_tolerance:
            self.stop()
            return True

        goal_angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        error = self.normalize_angle(goal_angle - self.yaw)
        angular = max(-self.max_angular_speed,
                      min(self.max_angular_speed, self.k_angular * error))

        if abs(error) > self.turn_in_place_angle:
            angular = math.copysign(max(abs(angular), self.min_angular_speed), error)
            self.publish_velocity(0.0, angular)
        elif self.front < self.safe_distance:
            self.publish_velocity(0.0, 0.0)
        else:
            linear = max(self.min_linear_speed,
                         min(self.max_linear_speed, self.k_linear * distance))
            self.publish_velocity(linear, angular)

        return False

    # --- PENDIENTE para que funcione la primera version (A -> B con odometria) ---
    # 1. __init__: leer los parametros con get_parameter(...).value (start_x/y/yaw y
    #    goal_yaw). goal_x/goal_y ya NO son parametros: salen de self.points.
    # 2. __init__: self.points (dict con los puntos del enunciado), self.path
    #    (lista de nombres, ej. ['A', 'B']) y self.path_index = 0.
    # 3. __init__: constantes de control que usa go_to_goal:
    #    goal_tolerance = 0.08 m, yaw_tol = 0.05 rad, turn_in_place_angle = 0.3 rad,
    #    k_angular = 2.0, k_linear = 0.5 (ajustar probando) y self.goal_free = None.
    # 4. normalize_angle: esta fuera de la clase (linea 9) pero se llama con self.;
    #    ponerla dentro como @staticmethod.
    # 5. odom_callback: quitar goal_dx/goal_dy (ya no existen); guardar la primera
    #    lectura (x0, y0, yaw0) y calcular self.x, self.y, self.yaw en MUNDO con
    #    la pose inicial start_x/y/yaw.
    # 6. set_goal(): poner self.goal_x/goal_y con self.points[self.path[self.path_index]].
    # 7. turn_to_yaw(): giro final hasta goal_yaw con yaw_tol (como turn_to de P2).
    # 8. run(): esperar pose y scan; bucle spin_once -> go_to_goal; si llega,
    #    path_index += 1 y set_goal(); al acabar la ruta, turn_to_yaw y fin.
    # 9. main(): crear el nodo, llamar a run() y en el finally mandar una parada
    #    y cerrar rclpy, como en Bug2. Imprimir al final el error de posicion y yaw.



def main():
    print('Hi from autonavigation_11j.')


if __name__ == '__main__':
    main()

