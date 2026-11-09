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

        self.declare_parameter("pose_source", "slam")
        self.declare_parameter("explore_path", ["R", "S", "T", "P_Base"])
        self.declare_parameter("log_file", "mission_log.csv")
        self.declare_parameter("map_file", "mapa_robot")
        self.declare_parameter("min_sightings", 15)
        self.declare_parameter("pillar_max_range", 2.5)
        self.pose_source = self.get_parameter("pose_source").value
        self.explore_path = list(self.get_parameter("explore_path").value)
        self.map_file = self.get_parameter("map_file").value
        self.min_sightings = self.get_parameter("min_sightings").value
        self.pillar_max_range = self.get_parameter("pillar_max_range").value

        self.max_linear_speed = 0.2
        self.min_linear_speed = 0.08
        self.k_linear = 0.5
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
        self.explore_speed = 0.15

        self.pillar_gap = 0.1
        self.pillar_max_width = 0.12
        self.pillar_radius = 0.025
        self.merge_radius = 0.15

        self.stage_distance = 0.8
        self.dock_speed = 0.05
        self.dock_tolerance = 0.03
        self.dock_emergency = 0.09

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

        self.phase = 'I'
        self.pillars = []
        self.scan_id = 0
        self.last_scan_id = 0
        self.landmarks = []
        self.station = None
        self.station_pillars = []
        self.map_saved = False

        self.log_file = open(self.get_parameter("log_file").value, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['timestamp', 'current_phase', 'robot_x', 'robot_y',
                                  'robot_yaw', 'obstacle_front_m', 'pillars_detected',
                                  'station_x', 'station_y'])
        self.last_log = 0.0
        self.log_period = 0.5


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

        if self.phase == 'II':
            self.pillars = self.detect_pillars(msg)
        else:
            self.pillars = []
        self.scan_id += 1

    def detect_pillars(self, msg):
        ranges = msg.ranges
        n = len(ranges)
        groups = []
        current = []
        for i in range(n):
            r = ranges[i]
            valid = math.isfinite(r) and msg.range_min < r < self.pillar_max_range
            if valid and current and abs(r - ranges[current[-1]]) < self.pillar_gap:
                current.append(i)
            else:
                if current:
                    groups.append(current)
                current = [i] if valid else []
        if current:
            groups.append(current)
        if len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == n - 1 \
                and abs(ranges[n - 1] - ranges[0]) < self.pillar_gap:
            groups[0] = groups[-1] + groups[0]
            groups.pop()

        pillars = []
        for g in groups:
            r_mean = sum(ranges[i] for i in g) / len(g)
            width = r_mean * len(g) * msg.angle_increment
            if width > self.pillar_max_width:
                continue
            sin_sum = 0.0
            cos_sum = 0.0
            for i in g:
                a = msg.angle_min + i * msg.angle_increment
                sin_sum += math.sin(a)
                cos_sum += math.cos(a)
            theta = math.atan2(sin_sum, cos_sum)
            r = r_mean + self.pillar_radius
            pillars.append((r * math.cos(theta), r * math.sin(theta)))
        return pillars

    def update_landmarks(self):
        if self.scan_id == self.last_scan_id:
            return
        self.last_scan_id = self.scan_id
        for px, py in self.pillars:
            wx = self.x + px * math.cos(self.yaw) - py * math.sin(self.yaw)
            wy = self.y + px * math.sin(self.yaw) + py * math.cos(self.yaw)
            for lm in self.landmarks:
                if math.hypot(lm[0] - wx, lm[1] - wy) < self.merge_radius:
                    lm[0] = (lm[0] * lm[2] + wx) / (lm[2] + 1)
                    lm[1] = (lm[1] * lm[2] + wy) / (lm[2] + 1)
                    lm[2] += 1
                    break
            else:
                self.landmarks.append([wx, wy, 1])
        self.find_station()

    def find_station(self):
        if self.station is not None:
            return
        good = [lm for lm in self.landmarks if lm[2] >= self.min_sightings]
        for seed in good:
            near = sorted(good, key=lambda lm: math.hypot(lm[0] - seed[0], lm[1] - seed[1]))[:4]
            if len(near) < 4:
                continue
            dists = [math.hypot(a[0] - b[0], a[1] - b[1])
                     for i, a in enumerate(near) for b in near[i + 1:]]
            if max(dists) < 0.7 and min(dists) > 0.25:
                cx = sum(lm[0] for lm in near) / 4
                cy = sum(lm[1] for lm in near) / 4
                self.station = (cx, cy)
                self.station_pillars = [(lm[0], lm[1]) for lm in near]
                self.get_logger().info(f'Estacion encontrada: centro=({cx:.2f}, {cy:.2f})')
                return

    def log_row(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log < self.log_period:
            return
        self.last_log = now
        obstacle = f'{self.front:.2f}' if self.front is not None and self.front < 3.0 else ''
        pillars = ';'.join(f'{p[0]:.2f}:{p[1]:.2f}' for p in self.station_pillars)
        station_x = ''
        station_y = ''
        if self.station is not None:
            station_x = f'{self.station[0]:.3f}'
            station_y = f'{self.station[1]:.3f}'
        self.log_writer.writerow([f'{now:.2f}', self.phase, f'{self.x:.3f}', f'{self.y:.3f}',
                                  f'{self.yaw:.3f}', obstacle, pillars, station_x, station_y])
        self.log_file.flush()

    def slam_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def update_pose(self):
        if self.pose_source == "slam":
            raw = self.slam_pose()
        elif self.odom_x is not None:
            raw = (self.odom_x, self.odom_y, self.odom_yaw)
        else:
            raw = None
        if raw is None:
            return False
        x, y, yaw = raw
        if self.x0 is None:
            self.x0 = x
            self.y0 = y
            self.yaw0 = yaw
        a = self.initial_yaw - self.yaw0
        dx = x - self.x0
        dy = y - self.y0
        self.x = self.initial_x + dx * math.cos(a) - dy * math.sin(a)
        self.y = self.initial_y + dx * math.sin(a) + dy * math.cos(a)
        self.yaw = normalize_angle(yaw - self.yaw0 + self.initial_yaw)
        if self.phase == 'II':
            self.update_landmarks()
        self.log_row()
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
            linear = max(self.min_linear_speed,
                         min(self.max_linear_speed, self.k_linear * self.goal_distance))
            self.publish_velocity(linear, angular)


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


    def save_map(self):
        self.get_logger().info(f'Guardando el mapa en {self.map_file}.yaml / .pgm')
        for attempt in range(2):
            try:
                result = subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.map_file],
                    capture_output=True, text=True, timeout=30)
                if result.returncode == 0:
                    self.map_saved = True
                    self.get_logger().info('Mapa guardado')
                    return
                self.get_logger().warn(f'map_saver_cli ha fallado: {result.stderr[-300:]}')
            except (subprocess.TimeoutExpired, OSError) as error:
                self.get_logger().warn(f'map_saver_cli ha fallado: {error}')
        self.get_logger().error('No se ha podido guardar el mapa')


    def dock(self):
        cx, cy = self.station
        p0 = self.station_pillars[0]
        others = sorted(self.station_pillars[1:],
                        key=lambda p: math.hypot(p[0] - p0[0], p[1] - p0[1]))
        p1 = others[0]
        side_angle = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

        to_robot = math.atan2(self.y - cy, self.x - cx)
        normal = side_angle
        best_diff = 10.0
        for k in range(4):
            candidate = normalize_angle(side_angle + k * math.pi / 2)
            diff = abs(normalize_angle(to_robot - candidate))
            if diff < best_diff:
                best_diff = diff
                normal = candidate

        self.points["Stage"] = (cx + self.stage_distance * math.cos(normal),
                                cy + self.stage_distance * math.sin(normal))
        if not self.go_path(["Stage"]):
            self.get_logger().warn('No he podido llegar al punto de aproximacion')
            return False
        self.turn_to_yaw(normalize_angle(normal + math.pi))

        ux = -math.cos(normal)
        uy = -math.sin(normal)
        along = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.update_pose():
                continue
            along = (cx - self.x) * ux + (cy - self.y) * uy
            if along < self.dock_tolerance:
                break
            if self.closest < self.dock_emergency:
                self.publish_velocity(0.0, 0.0)
                self.get_logger().warn(f'Algo a {self.closest:.2f} m: parado',
                                       throttle_duration_sec=2.0)
                continue
            axis_x = cx - ux * along
            axis_y = cy - uy * along
            ahead = min(0.15, along)
            target_x = axis_x + ux * ahead
            target_y = axis_y + uy * ahead
            error = normalize_angle(math.atan2(target_y - self.y, target_x - self.x) - self.yaw)
            angular = 2.0 * error
            if self.left < 0.3 and self.right < 0.3:
                angular += 1.0 * (self.left - self.right)
            angular = max(-0.5, min(0.5, angular))
            linear = self.dock_speed if abs(error) < 0.3 else 0.0
            self.publish_velocity(linear, angular)
        self.stop()
        self.get_logger().info(f'Acoplado: faltan {max(along, 0.0):.3f} m hasta el centro')
        return True


    def run(self):
        waited = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.update_pose() and self.front is not None:
                break
            waited += 1
            if waited % 50 == 0:
                self.get_logger().warn('Sin pose o sin LiDAR todavia: esta el SLAM en marcha? '
                                       '(o prueba con -p pose_source:=odom)')
        self.get_logger().info(
            f'Salida {self.start_point}: x={self.x:.2f} y={self.y:.2f} '
            f'yaw={math.degrees(self.yaw):.1f} grados')
        self.countdown(self.start_delay)

        if not self.go_path(self.path):
            self.get_logger().warn('Fase I interrumpida')
            return
        self.turn_to_yaw(self.final_yaw)
        self.get_logger().info('Fase I completada')

        self.phase = 'II'
        self.max_linear_speed = self.explore_speed
        if not self.go_path(self.explore_path):
            self.get_logger().warn('Fase II interrumpida')
            return
        self.save_map()
        if self.station is None:
            self.get_logger().error('No se ha encontrado la estacion')
            return
        self.get_logger().info('Fase II completada')

        self.phase = 'III'
        if self.dock():
            self.phase = 'FIN'
            self.get_logger().info('Mision completada')
       

    def close_log(self):
        if not self.log_file.closed:
            self.log_file.close()
    


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
        if not node.map_saved and node.x is not None and node.pose_source == "slam":
            try:
                node.save_map()
            except Exception as error:
                print(f'No se ha podido guardar el mapa: {error}')
        node.close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
