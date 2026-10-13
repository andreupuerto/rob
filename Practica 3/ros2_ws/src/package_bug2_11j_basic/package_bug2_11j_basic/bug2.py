import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Bug2(Node):
    def __init__(self):
        super().__init__('bug2_node')
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qos_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_r)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_b)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_b)

        # --- Parámetros (los pasa el launch) ---
        # El objetivo es relativo a la pose inicial del robot: x hacia delante, y hacia la izquierda
        self.declare_parameter('goal_x', 2.7)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('advanced', False)

        # --- Puntos ---
        self.start_x = None           # primera posición de /odom (define la m-line)
        self.start_y = None
        self.start_yaw = None
        self.goal_dx = self.get_parameter('goal_x').value     # objetivo relativo al inicio
        self.goal_dy = self.get_parameter('goal_y').value
        self.goal_x = None            # objetivo en el frame odom (se calcula con la 1ª odometría)
        self.goal_y = None

        # --- Velocidades ---
        self.linear_speed = 0.2       # m/s  (yendo hacia el objetivo)
        self.wall_speed = 0.1         # m/s  (siguiendo la pared)
        self.angular_speed = 0.8      # rad/s (máxima)

        # --- Distancias ---
        self.safe_distance = 0.25     # m, a partir de aquí hay un obstáculo delante
        self.wall_distance = 0.20     # m, separación con la pared al rodear

        # --- Tolerancias ---
        self.goal_tol = 0.1           # m, objetivo alcanzado
        self.mline_tol = 0.05         # m, "estoy en la m-line"
        self.leave_margin = 0.15      # m, hay que estar tanto más cerca que en el hit point para salir
        self.yaw_tol = 0.3            # rad (~17°), por debajo avanzamos y corregimos a la vez

        # --- Detección de objetivo inalcanzable ---
        self.away_dist = 0.5          # m, a partir de aquí ya nos hemos alejado del hit point
        self.back_dist = 0.2          # m, si volvemos a menos de esto, dimos la vuelta entera

        # --- Modo ---
        self.advanced_mode = self.get_parameter('advanced').value

        # --- Pose (odometría) ---
        self.x = None
        self.y = None
        self.yaw = None

        # --- LiDAR (mínimos por sector) ---
        self.front = None             # delante (-30° a 30°)
        self.right = None             # derecha (-110° a -30°)
        self.goal_free = 0.0          # hacia el objetivo (±45°)

        # --- Estado de Bug2 ---
        self.last_linear = 0.0        # último comando enviado (para el log)
        self.last_angular = 0.0
        self.goal_distance = None
        self.hit_distance = None      # None: voy al objetivo. Con valor: rodeando una pared
        self.hit_x = None             # dónde choqué
        self.hit_y = None
        self.left_hit = False         # ¿ya me he alejado del hit point?
        self.unreachable = False

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.start_x is None:
            # Primera odometría: fija el inicio y pasa el objetivo al frame odom
            self.start_x = self.x
            self.start_y = self.y
            self.start_yaw = self.yaw
            self.goal_x = self.start_x + self.goal_dx * math.cos(self.start_yaw) \
                - self.goal_dy * math.sin(self.start_yaw)
            self.goal_y = self.start_y + self.goal_dx * math.sin(self.start_yaw) \
                + self.goal_dy * math.cos(self.start_yaw)

    def scan_callback(self, msg):
        # El ángulo 0 del LiDAR es el frente y crece hacia la izquierda.
        # Calculamos el ángulo de cada medida en vez de usar índices fijos.
        front = float('inf')
        right = float('inf')
        goal_free = float('inf')

        # Dirección del objetivo respecto al robot (grados)
        goal_direction = 0.0
        if self.x is not None:
            goal_angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            goal_direction = math.degrees(self.normalize_angle(goal_angle - self.yaw))

        for i, r in enumerate(msg.ranges):
            # Filtramos valores infinitos o erróneos (0.0)
            if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                continue
            angle = math.degrees(
                self.normalize_angle(msg.angle_min + i * msg.angle_increment))
            if -30 <= angle <= 30:
                front = min(front, r)
            elif -110 <= angle < -30:
                right = min(right, r)
            # Cono de ±45° alrededor de la dirección del objetivo
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
        self.last_linear = linear
        self.last_angular = angular

    def stop(self):
        self.publish_velocity(0.0, 0.0)
        rclpy.spin_once(self, timeout_sec=0.1)

    @staticmethod
    def normalize_angle(angle):
        # Devuelve el ángulo en el rango [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))

    def follow_line(self):
        # Ir hacia el objetivo por la m-line
        goal_angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        error = self.normalize_angle(goal_angle - self.yaw)
        angular = max(-self.angular_speed, min(self.angular_speed, 2.0 * error))

        if abs(error) > self.yaw_tol:
            # Muy desviado: giramos en el sitio
            self.publish_velocity(0.0, angular)
        elif self.front < self.safe_distance and self.front < self.goal_distance:
            # Obstáculo delante: apuntamos dónde y a qué distancia del objetivo chocamos
            self.hit_distance = self.goal_distance
            self.hit_x = self.x
            self.hit_y = self.y
            self.left_hit = False
            self.stop()
            self.get_logger().warn(
                f'¡OBSTÁCULO DETECTADO a {self.front:.2f}m! Siguiendo la pared')
        else:
            self.publish_velocity(self.linear_speed, angular)

    def follow_wall(self):
        # Distancia a la m-line (recta inicio -> objetivo)
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        mline_distance = abs(dx * (self.y - self.start_y) - dy * (self.x - self.start_x)) \
            / math.hypot(dx, dy)

        # Distancia al punto donde chocamos
        hit_dist = math.hypot(self.x - self.hit_x, self.y - self.hit_y)
        if hit_dist > self.away_dist:
            self.left_hit = True

        if mline_distance < self.mline_tol \
                and self.goal_distance < self.hit_distance - self.leave_margin:
            # Estoy en la m-line y claramente más cerca del objetivo que al chocar
            self.hit_distance = None
            self.stop()
            self.get_logger().info(
                f'm-line recuperada en ({self.x:.2f}, {self.y:.2f}). Hacia el objetivo')
        elif self.advanced_mode and self.goal_free > min(self.goal_distance, 1.0) \
                and self.goal_distance < self.hit_distance:
            # Avanzado: camino libre hacia el objetivo, no hace falta volver a la m-line
            self.hit_distance = None
            self.stop()
            self.get_logger().info(
                f'Camino libre hacia el objetivo en ({self.x:.2f}, {self.y:.2f}). Atajo')
        elif self.left_hit and hit_dist < self.back_dist:
            # He dado la vuelta entera sin encontrar salida
            self.unreachable = True
        elif self.front < self.safe_distance:
            # Pared delante: giro a la izquierda (la pared queda a la derecha)
            self.publish_velocity(0.0, self.angular_speed)
        else:
            # Sigo la pared manteniendo wall_distance
            error = self.wall_distance - min(self.right, self.wall_distance + 0.3)
            angular = max(-self.angular_speed, min(self.angular_speed, 2.5 * error))
            self.publish_velocity(self.wall_speed, angular)

    def run_bug2(self):
        # Esperamos a tener odometría y LiDAR
        while rclpy.ok() and (self.x is None or self.front is None):
            rclpy.spin_once(self, timeout_sec=0.1)

        modo = 'avanzado' if self.advanced_mode else 'básico'
        self.get_logger().info(
            f'Inicio ({self.start_x:.2f}, {self.start_y:.2f}, '
            f'{math.degrees(self.start_yaw):.0f}°) -> '
            f'objetivo odom ({self.goal_x:.2f}, {self.goal_y:.2f}) '
            f'[relativo ({self.goal_dx:.2f}, {self.goal_dy:.2f})]. Modo {modo}')

        # Diagnóstico: más de un publicador en estos topics significa que hay otra fuente
        # (Gazebo a la vez, otro nodo moviendo el robot, otro grupo en el mismo dominio...)
        for topic in ('/cmd_vel', '/odom', '/scan'):
            # En /cmd_vel el nuestro ya cuenta como 1; en /odom y /scan solo debe publicar el robot
            n_pub = self.count_publishers(topic)
            if n_pub > 1:
                self.get_logger().warn(
                    f'{topic} tiene {n_pub} publicadores (esperado 1). '
                    f'Revisa con: ros2 topic info {topic} -v')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            self.goal_distance = math.hypot(self.goal_x - self.x, self.goal_y - self.y)

            estado = 'LINEA' if self.hit_distance is None else 'PARED'
            self.get_logger().info(
                f'[{estado}] pos=({self.x:.2f}, {self.y:.2f}) '
                f'yaw={math.degrees(self.yaw):.0f}° dist_obj={self.goal_distance:.2f} '
                f'front={self.front:.2f} right={self.right:.2f} libre_obj={self.goal_free:.2f} '
                f'cmd=({self.last_linear:.2f}, {self.last_angular:.2f})',
                throttle_duration_sec=1.0)

            if self.goal_distance < self.goal_tol:
                self.stop()
                self.get_logger().info('¡OBJETIVO ALCANZADO!')
                break
            elif self.unreachable:
                self.stop()
                self.get_logger().error(
                    f'OBJETIVO INALCANZABLE: vuelta completa al obstáculo desde '
                    f'({self.hit_x:.2f}, {self.hit_y:.2f})')
                break
            elif self.hit_distance is None:
                self.follow_line()
            else:
                self.follow_wall()


def main(args=None):
    print('Hi from package_bug2_11j_basic.')
    rclpy.init(args=args)
    node = Bug2()
    try:
        node.run_bug2()
    except KeyboardInterrupt:
        pass
    finally:
        # Siempre es bueno mandar un último mensaje de parada
        if rclpy.ok():
            node.publisher.publish(TwistStamped())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
