import cmath
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

def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def clamp(value, low, high):
    return max(low, min(high, value))

def find_square(points, min_side=0.28, max_side=0.50, allow_three=False):
    best = None
    for combo in itertools.combinations(points[:14], 4):
        d = sorted(math.hypot(a[0] - b[0], a[1] - b[1])
                   for a, b in itertools.combinations(combo, 2))
        side = sum(d[:4]) / 4
        if not min_side <= side <= max_side:
            continue
        side_dev = max(abs(v - side) for v in d[:4])
        diag_dev = max(abs(v - side * math.sqrt(2)) for v in d[4:])
        if side_dev > 0.06 or diag_dev > 0.08:
            continue
        if best is None or side_dev + diag_dev < best[0]:
            best = (side_dev + diag_dev, combo, side)
    if best is None and allow_three:
        return find_corner(points, min_side, max_side)
    if best is None:
        return None
    combo, side = best[1], best[2]
    cx = sum(p[0] for p in combo) / 4
    cy = sum(p[1] for p in combo) / 4
    total = sum(cmath.exp(4j * math.atan2(p[1] - cy, p[0] - cx)) for p in combo)
    theta = (cmath.phase(total) - math.pi) / 4
    return cx, cy, side, theta

def find_corner(points, min_side, max_side):
    best = None
    for combo in itertools.combinations(points[:14], 3):
        for i in range(3):
            vertex = combo[i]
            p, q = [combo[j] for j in range(3) if j != i]
            d1 = math.hypot(p[0] - vertex[0], p[1] - vertex[1])
            d2 = math.hypot(q[0] - vertex[0], q[1] - vertex[1])
            side = (d1 + d2) / 2
            if not min_side <= side <= max_side:
                continue
            diag = math.hypot(p[0] - q[0], p[1] - q[1])
            error = abs(d1 - d2) + abs(diag - side * math.sqrt(2))
            if abs(d1 - d2) > 0.06 or abs(diag - side * math.sqrt(2)) > 0.08:
                continue
            if best is None or error < best[0]:
                best = (error, vertex, p, q, side)
    if best is None:
        return None
    _, vertex, p, q, side = best
    cx = (p[0] + q[0]) / 2
    cy = (p[1] + q[1]) / 2
    theta = math.atan2(p[1] - vertex[1], p[0] - vertex[0])
    return cx, cy, side, theta

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
            "T" : (2.136, 15.300),
            "Door_in" : (5.9, 11.75),
            "Door_out" : (7.0, 11.75),
            "Via_Q" : (8.2, 14.3),
            "Cor_1" : (8.2, 15.45),
            "Cor_2" : (6.5, 15.45),
            "Cor_3" : (4.8, 15.45),
            "Cor_4" : (2.2, 15.45)
        }

        self.declare_parameter("start_point", "A")
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("path", ["B", "C", "D", "E", "F", "G",
                                        "Door_in", "Door", "Door_out", "Via_Q", "Q"])
        self.declare_parameter("final_yaw", math.pi)
        self.declare_parameter("explore_path", ["Cor_1", "Cor_2", "Cor_3", "Cor_4"])
        self.declare_parameter("pose_source", "slam")
        self.declare_parameter("log_file", "mission_log.csv")
        self.declare_parameter("map_file", "mapa_robot")
        self.declare_parameter("start_delay", 5.0)
        self.declare_parameter("pillar_max_range", 2.5)
        self.declare_parameter("min_sightings", 15)

        self.max_linear_speed = 0.25
        self.min_linear_speed = 0.08
        self.max_angular_speed = 0.8
        self.min_angular_speed = 0.05

        self.k_angular = 2.0
        self.k_linear = 0.5

        self.x = None
        self.y = None
        self.yaw = None
        self.phase = 'INIT'
        self.following = None
        self.start_point = self.get_parameter("start_point").value
        self.start_x, self.start_y = self.points[self.start_point]
        self.start_yaw = self.get_parameter("start_yaw").value
        self.path = list(self.get_parameter("path").value)
        self.final_yaw = self.get_parameter("final_yaw").value
        self.explore_path = list(self.get_parameter("explore_path").value)
        for name in self.path + self.explore_path:
            if name not in self.points:
                raise ValueError(f'Punto desconocido: {name}')
        self.goal_x = None
        self.goal_y = None

        self.safe_distance = 0.25
        self.emergency_distance = 0.13
        self.wait_time = 3.0
        self.wait_since = None
        self.wall_distance = 0.2
        self.goal_tolerance = 0.12
        self.yaw_tol = 0.08
        self.turn_in_place_angle = 0.3

        self.front = None
        self.right = None
        self.closest = None
        self.goal_free = float('inf')
        self.lidar_offset = -0.032
        self.scan_id = 0

        self.wall_follow_speed = 0.1
        self.mline_tolerance = 0.05
        self.leave_margin = 0.15
        self.away_dist = 0.5
        self.back_dist = 0.2
        self.following_wall = False
        self.hit_distance = 0.0
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.left_hit = False
        self.unreachable = False
        self.goal_distance = None
        self.leg_start_x = None
        self.leg_start_y = None

        self.pillar_radius = 0.025
        self.pillar_max_range = self.get_parameter("pillar_max_range").value
        self.min_sightings = self.get_parameter("min_sightings").value
        self.pillars = []
        self.landmarks = []
        self.last_scan_used = -1
        self.station = None
        self.staging_distance = 0.9
        self.dock_tolerance = 0.03
        self.lateral_tolerance = 0.04

        self.pose_source = self.get_parameter("pose_source").value

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None

        self.x0 = None
        self.y0 = None
        self.yaw0 = None

        self.log_file = open(self.get_parameter("log_file").value, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['timestamp', 'current_phase', 'robot_x', 'robot_y',
                                  'robot_yaw', 'obstacle_front_m', 'pillars_detected',
                                  'station_x', 'station_y'])
        self.last_log = -1.0
        self.log_period = 0.5
        self.map_saved = False

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        front = float('inf')
        right = float('inf')
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
            if abs((angle - goal_direction + 180) % 360 - 180) <= 45:
                goal_free = min(goal_free, r)

        self.front = front
        self.right = right
        self.closest = closest
        self.goal_free = goal_free
        self.pillars = self.detect_pillars(msg)
        self.scan_id += 1

    def detect_pillars(self, msg):
        n = len(msg.ranges)
        valid = [(not math.isinf(r) and not math.isnan(r) and r >= msg.range_min)
                 for r in msg.ranges]

        clusters = []
        current = []
        for i in range(n):
            if valid[i] and current and abs(msg.ranges[i] - msg.ranges[current[-1]]) < 0.08:
                current.append(i)
            else:
                if current:
                    clusters.append(current)
                current = [i] if valid[i] else []
        if current:
            clusters.append(current)
        if len(clusters) > 1 and clusters[0][0] == 0 and clusters[-1][-1] == n - 1 \
                and abs(msg.ranges[0] - msg.ranges[n - 1]) < 0.08:
            clusters[0] = clusters.pop() + clusters[0]

        pillars = []
        for cluster in clusters:
            ranges = [msg.ranges[i] for i in cluster]
            mean_range = sum(ranges) / len(ranges)
            if mean_range > self.pillar_max_range:
                continue
            first = msg.angle_min + cluster[0] * msg.angle_increment
            last = msg.angle_min + cluster[-1] * msg.angle_increment
            width = math.hypot(ranges[0] * math.cos(first) - ranges[-1] * math.cos(last),
                               ranges[0] * math.sin(first) - ranges[-1] * math.sin(last))
            if width > 0.12:
                continue
            before = msg.ranges[(cluster[0] - 1) % n] if valid[(cluster[0] - 1) % n] else float('inf')
            after = msg.ranges[(cluster[-1] + 1) % n] if valid[(cluster[-1] + 1) % n] else float('inf')
            if abs(before - mean_range) < 0.12 or abs(after - mean_range) < 0.12:
                continue
            angles = [msg.angle_min + i * msg.angle_increment for i in cluster]
            angle = math.atan2(sum(math.sin(a) for a in angles),
                               sum(math.cos(a) for a in angles))
            r = mean_range + self.pillar_radius
            pillars.append((r * math.cos(angle) + self.lidar_offset, r * math.sin(angle)))
        return pillars

    def update_landmarks(self):
        if self.scan_id == self.last_scan_used:
            return
        self.last_scan_used = self.scan_id
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        for px, py in self.pillars:
            wx = self.x + px * c - py * s
            wy = self.y + px * s + py * c
            for lm in self.landmarks:
                if math.hypot(lm[0] - wx, lm[1] - wy) < 0.15:
                    n = lm[2]
                    lm[0] = (lm[0] * n + wx) / (n + 1)
                    lm[1] = (lm[1] * n + wy) / (n + 1)
                    lm[2] = n + 1
                    break
            else:
                self.landmarks.append([wx, wy, 1])
        confirmed = [(lm[0], lm[1]) for lm in sorted(self.landmarks, key=lambda lm: -lm[2])
                     if lm[2] >= self.min_sightings]
        square = find_square(confirmed)
        if square is not None:
            if self.station is None:
                self.get_logger().info(
                    f'Estacion encontrada: centro=({square[0]:.2f}, {square[1]:.2f}) '
                    f'lado={square[2]:.2f} m')
            self.station = square

    def log_row(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log < self.log_period:
            return
        self.last_log = now
        pillars = ''
        if self.station is not None:
            pillars = ';'.join(
                f'{lm[0]:.2f}:{lm[1]:.2f}' for lm in self.landmarks
                if lm[2] >= self.min_sightings
                and math.hypot(lm[0] - self.station[0], lm[1] - self.station[1]) < 0.5)
        obstacle = f'{self.front:.2f}' if self.front is not None and self.front < 3.0 else ''
        station = ('', '') if self.station is None \
            else (f'{self.station[0]:.3f}', f'{self.station[1]:.3f}')
        self.log_writer.writerow([f'{now:.2f}', self.phase, f'{self.x:.3f}', f'{self.y:.3f}',
                                  f'{self.yaw:.3f}', obstacle, pillars, station[0], station[1]])
        self.log_file.flush()

    def slam_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x,
                t.transform.translation.y,
                yaw_from_quaternion(t.transform.rotation))

    def to_world(self, x, y, yaw):
        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = x, y, yaw
        a = self.start_yaw - self.yaw0
        dx = x - self.x0
        dy = y - self.y0
        x_w = self.start_x + dx * math.cos(a) - dy * math.sin(a)
        y_w = self.start_y + dx * math.sin(a) + dy * math.cos(a)
        yaw_w = normalize_angle(yaw - self.yaw0 + self.start_yaw)
        return x_w, y_w, yaw_w

    def get_pose(self):
        if self.pose_source == "slam":
            raw = self.slam_pose()
        else:
            if self.odom_x is None:
                return None
            raw = (self.odom_x, self.odom_y, self.odom_yaw)
        if raw is None:
            return None
        return self.to_world(*raw)

    def update_pose(self):
        pose = self.get_pose()
        if pose is None:
            return False
        self.x, self.y, self.yaw = pose
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
        self.following = name
        self.goal_x, self.goal_y = self.points[name]
        self.leg_start_x = self.x
        self.leg_start_y = self.y
        self.following_wall = False
        self.left_hit = False
        self.unreachable = False
        self.wait_since = None

    def follow_line(self):
        goal_angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        error = normalize_angle(goal_angle - self.yaw)
        angular = clamp(self.k_angular * error, -self.max_angular_speed, self.max_angular_speed)

        if abs(error) > self.turn_in_place_angle:
            angular = math.copysign(max(abs(angular), self.min_angular_speed), error)
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
            linear = clamp(self.k_linear * self.goal_distance,
                           self.min_linear_speed, self.max_linear_speed)
            self.publish_velocity(linear, angular)

    def follow_wall(self):
        dx = self.goal_x - self.leg_start_x
        dy = self.goal_y - self.leg_start_y
        leg_length = math.hypot(dx, dy)
        if leg_length > 0.0:
            mline_distance = abs(dx * (self.y - self.leg_start_y)
                                 - dy * (self.x - self.leg_start_x)) / leg_length
        else:
            mline_distance = 0.0

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
            angular = clamp(2.5 * error, -self.max_angular_speed, self.max_angular_speed)
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
            self.get_logger().warn(f'Algo a {self.closest:.2f} m: parado',
                                   throttle_duration_sec=2.0)
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
            error = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
            self.get_logger().info(
                f'Llegado a {name}: x={self.x:.2f} y={self.y:.2f} '
                f'error={error:.3f} m')
        return True

    def turn_to_yaw(self, target_yaw):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.update_pose():
                continue
            error = normalize_angle(target_yaw - self.yaw)
            if abs(error) <= self.yaw_tol:
                break
            if self.closest < self.emergency_distance:
                self.publish_velocity(0.0, 0.0)
                continue
            angular = clamp(self.k_angular * error, -self.max_angular_speed,
                            self.max_angular_speed)
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
        self.stop()
        end = self.get_clock().now().nanoseconds / 1e9 + seconds
        while rclpy.ok() and self.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.update_pose()

    def save_map(self):
        name = self.get_parameter("map_file").value
        sim = self.get_parameter("use_sim_time").value
        self.get_logger().info(f'Guardando el mapa en {name}.yaml / {name}.pgm')
        for attempt in range(1, 4):
            try:
                result = subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', name,
                     '--ros-args', '-p', f'use_sim_time:={str(sim).lower()}'],
                    timeout=40, check=False, capture_output=True, text=True)
            except (subprocess.TimeoutExpired, OSError) as error:
                self.get_logger().error(f'No se ha podido guardar el mapa: {error}')
                continue
            if result.returncode == 0:
                self.get_logger().info('Mapa guardado')
                self.map_saved = True
                return True
            self.get_logger().warn(
                f'map_saver_cli ha fallado (intento {attempt}/3): '
                f'{(result.stderr or result.stdout)[-300:]}')
            self.look_around(2.0)
        return False

    def square_from_pair(self, points, min_side=0.28, max_side=0.50):
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        best = None
        for p, q in itertools.combinations(points, 2):
            side = math.hypot(p[0] - q[0], p[1] - q[1])
            if not min_side <= side <= max_side:
                continue
            mx = (p[0] + q[0]) / 2
            my = (p[1] + q[1]) / 2
            nx = -(q[1] - p[1]) / side
            ny = (q[0] - p[0]) / side
            if nx * mx + ny * my < 0:
                nx, ny = -nx, -ny
            cx = mx + nx * side / 2
            cy = my + ny * side / 2
            wx = self.x + cx * c - cy * s
            wy = self.y + cx * s + cy * c
            deviation = math.hypot(wx - self.dock_centre[0], wy - self.dock_centre[1])
            if deviation < 0.6 and (best is None or deviation < best[0]):
                best = (deviation, (cx, cy, side, math.atan2(q[1] - p[1], q[0] - p[0])))
        return None if best is None else best[1]

    def dock(self):
        cx, cy, side, theta = self.station
        self.dock_centre = (cx, cy)

        best = None
        for k in range(4):
            a = theta + k * math.pi / 2
            score = math.cos(a) * (self.x - cx) + math.sin(a) * (self.y - cy)
            if best is None or score > best[0]:
                best = (score, a)
        angle = best[1]
        self.points['Stage'] = (cx + self.staging_distance * math.cos(angle),
                                cy + self.staging_distance * math.sin(angle))
        if not self.go_path(['Stage']):
            return False
        self.turn_to_yaw(normalize_angle(angle + math.pi))

        lost_since = None
        start = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.update_pose():
                continue
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > 150.0:
                self.get_logger().error('Tiempo agotado en el acoplamiento')
                break

            near = [p for p in self.pillars if math.hypot(p[0], p[1]) < 1.6]
            square = find_square(near, allow_three=True)
            if square is not None:
                c = math.cos(self.yaw)
                s = math.sin(self.yaw)
                self.dock_centre = (self.x + square[0] * c - square[1] * s,
                                    self.y + square[0] * s + square[1] * c)
                self.station = (self.dock_centre[0], self.dock_centre[1],
                                square[2], self.station[3])
            elif self.dock_centre is not None:
                square = self.square_from_pair(near)
            if square is None:
                self.publish_velocity(0.0, 0.0)
                if lost_since is None:
                    lost_since = now
                elif now - lost_since > 3.0:
                    self.get_logger().error(
                        f'No veo los pilares ({len(near)} detectados): aborto el acoplamiento')
                    break
                continue
            lost_since = None
            sx, sy, side, theta = square

            distance = math.hypot(sx, sy)
            if distance < self.dock_tolerance:
                self.stop()
                self.get_logger().info(
                    f'Acoplado: el centro de la estacion esta a {distance:.3f} m del robot')
                return True

            best = None
            for k in range(4):
                a = theta + k * math.pi / 2
                score = -(math.cos(a) * sx + math.sin(a) * sy)
                if best is None or score > best[0]:
                    best = (score, a)
            along = best[0]
            ux, uy = math.cos(best[1]), math.sin(best[1])
            lateral = -sx * (-uy) - sy * ux
            lateral = abs(lateral)

            if lateral > self.lateral_tolerance:
                along_target = max(along - 0.25, 0.55)
            else:
                along_target = max(along - 0.25, 0.0)
            tx = sx + ux * along_target
            ty = sy + uy * along_target
            error = math.atan2(ty, tx)
            if abs(error) > 0.2:
                self.publish_velocity(0.0, math.copysign(
                    clamp(1.5 * abs(error), 0.1, 0.5), error))
            else:
                self.publish_velocity(
                    clamp(0.5 * math.hypot(tx, ty), 0.03, 0.07),
                    clamp(1.5 * error, -0.4, 0.4))
        self.stop()
        return False

    def run(self):
        while rclpy.ok() and not (self.update_pose() and self.front is not None):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f'Salida {self.start_point}: x={self.x:.2f} y={self.y:.2f} '
            f'yaw={math.degrees(self.yaw):.1f} grados ({self.pose_source})')
        self.countdown(self.get_parameter("start_delay").value)

        self.phase = 'I'
        if not self.go_path(self.path):
            return
        self.turn_to_yaw(self.final_yaw)
        self.get_logger().info('Fase I completada')

        self.phase = 'II'
        self.max_linear_speed = 0.15
        if not self.go_path(self.explore_path + ['P_Base']):
            return
        if self.station is None:
            self.get_logger().warn('Estacion sin confirmar: miro parado desde P_Base')
            self.look_around(5.0)
        if self.station is None:
            self.get_logger().warn('Estacion sin confirmar: segunda pasada del pasillo')
            if not self.go_path(list(reversed(self.explore_path)) + ['P_Base']):
                return
            self.look_around(5.0)
        self.save_map()
        if self.station is None:
            self.get_logger().error(
                f'Fase II: no he encontrado la estacion ({len(self.landmarks)} objetos '
                f'pequenos, {sum(1 for lm in self.landmarks if lm[2] >= self.min_sightings)} '
                f'confirmados)')
            return
        self.get_logger().info('Fase II completada')

        self.phase = 'III'
        if self.dock():
            self.phase = 'FIN'
            self.get_logger().info('Mision completada')

    def close_log(self):
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
        if not node.map_saved and node.x is not None:
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
