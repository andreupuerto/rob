# MOBILE ROBOTICS PROJECT

## INTRODUCTION

This report describes a ROS 2 node that drives a TurtleBot3 Burger through a three-phase mission without collisions, building its own map along the way. As in the previous laboratory assignments, the robot has no external help: it relies only on its LiDAR, its odometry and the map that it builds itself.

The mission has three phases:

1. **Global navigation.** Go from an assigned start pose in Zone 1 (points A to D) through E and the Door to the final pose Q in Zone 2, avoiding unknown static or dynamic obstacles.
2. **Exploration.** Explore the corridor to find the charging station (a 40x40 cm structure with four pillars of about 5 cm each) and return to P_Base, while SLAM builds the map.
3. **Docking.** Dock with precision in the centre of the charging station.

The node that does this mission is `autonavigation_node`. It uses the LiDAR for obstacle avoidance and pillar detection, and `slam_toolbox` for mapping and localisation. Obstacle avoidance is based on the Bug2 algorithm from Practice 3, extended with a waiting step for dynamic obstacles. The rest of the report is organised as follows: Architecture describes how the code is structured, the next three sections explain each phase in detail, and the last two sections present the results and the conclusions.

## ARCHITECTURE

The code consists of a single ROS 2 node, written in Python, that runs the whole mission. It works together with `slam_toolbox`, which provides the map and the localisation. This section gives the overall structure of the code; each phase is described in detail in its own section.

### NODES

*(Insert here the `rqt_graph` screenshot or the block diagram.)*

| Node | Inputs | Outputs |
|---|---|---|
| `autonavigation_node` | `/scan` (LaserScan), `/odom` (Odometry), TF `map -> base_footprint` | `/cmd_vel` (TwistStamped), `mission_log.csv` |
| `slam_toolbox` | `/scan`, `/odom` | `/map` (OccupancyGrid), TF `map -> odom` |
| `map_saver_cli` (`nav2_map_server`) | `/map` | `mapa_robot.pgm` and `mapa_robot.yaml` |

Our node never talks to `slam_toolbox` directly: it only reads the robot pose from the TF tree. `map_saver_cli` is launched by our node as a subprocess at the end of phase II.

### STRUCTURE

The node is a single class organised in four layers:

- **Sensing.** `scan_callback` reduces every LiDAR scan to a few numbers, using the same sectors as in Practice 3: the minimum distance in the front sector (-30° to +30°), in the right sector (-110° to -30°), in a ±45° cone around the direction of the goal, and in any direction. It also calls `detect_pillars`, which finds small isolated objects (Phase II). `odom_callback` stores the raw odometry.
- **Pose.** `get_pose` reads the SLAM pose (or the odometry, as a fallback chosen with the `pose_source` parameter). `to_world` converts it into the frame of the assignment map, where all the mission points are defined.
- **Control.** `go_to_goal` runs one step of Bug2 towards the current goal. `follow_line` goes straight to the goal and `follow_wall` goes around an obstacle. `turn_to_yaw` rotates in place to a given orientation.
- **Mission sequencer.** `run` executes phases I, II and III in order. `go_path` visits a list of points one after another.

The mission points and the tuning values (path, speeds, thresholds) are ROS parameters or class attributes, so they can be changed without modifying the logic.

| Phase | Main functions |
|---|---|
| I: global navigation | `go_path`, `go_to_goal`, `follow_line`, `follow_wall` |
| II: exploration | `go_path`, `detect_pillars`, `update_landmarks`, `find_square`, `save_map` |
| III: docking | `dock`, `find_square`, `square_from_pair` |

### SLAM

We use `slam_toolbox` in online asynchronous mode with its default configuration. It builds the map from `/scan` and `/odom` and, at the same time, estimates where the robot is inside that map. Our node reads this estimate through the TF tree (`map -> base_footprint`) and uses it as the robot pose. Unlike the raw odometry, this pose is corrected by the map, so it does not accumulate drift.

The SLAM map frame starts where the robot is when SLAM is launched, so it does not coincide with the frame of the assignment. `to_world` aligns both frames using the known start point and start yaw: it takes the first pose read as the origin and applies a rotation and a translation to every later pose. The map is saved to disk at the end of phase II and also if the mission is interrupted.

## PHASE 1: GLOBAL NAVIGATION

The robot has to go from the assigned start point in Zone 1 to the final pose in Zone 2, passing through E, the Door and Q, without hitting any obstacle.

### Route

The route is a list of waypoints that the robot visits in order. Between two consecutive waypoints it moves in a straight line, so the waypoints must keep it away from the walls:

`A -> B -> C -> D -> E -> F -> G -> Door_in -> Door -> Door_out -> Via_Q -> Q`

B, C, D, F and G are points of the assignment. `Door_in`, `Door_out` and `Via_Q` are extra points that we added: the first two make the robot cross the door in a straight line, and `Via_Q` makes it go around the wall before reaching Q. Their coordinates are approximate and were chosen on the map. At Q the robot turns in place to the final orientation `final_yaw`.

### Navigation to a waypoint

Each leg is solved with the Bug2 algorithm of Practice 3, as a two-state machine:

1. **Go to goal (`follow_line`).** The robot computes the angle to the goal and steers towards it. If the heading error is larger than 0.3 rad it rotates in place; otherwise it drives forward with a proportional angular correction (gain 2.0). The linear speed is proportional to the distance to the goal (gain 0.5), limited between 0.08 and 0.25 m/s. The straight line from the start of the leg to the goal is the m-line.
2. **Wall following (`follow_wall`).** The robot goes around the obstacle keeping it on its right side, at a target distance of 0.20 m, with a proportional controller (gain 2.5) on the right LiDAR sector, at 0.10 m/s. If something appears in front it turns left in place.

A leg ends when the robot is within 0.12 m of the waypoint. The robot then logs the final error and starts the next leg.

### Obstacle avoidance strategy

The LiDAR overrides the predefined path in four ways, from the mildest to the strongest:

1. **Detection.** If the front sector reads less than the safety distance (0.25 m) and the obstacle is closer than the goal, there is an obstacle on the path.
2. **Wait.** The robot first stops for 3 s. If the obstacle is dynamic it will probably move away, and the robot resumes its path without any detour.
3. **Detour.** If the obstacle is still there after 3 s, the robot saves the hit point and its distance to the goal, and switches to wall following. It leaves the wall (back to go to goal) when one of these conditions is met:
   - it is back on the m-line (within 0.05 m) and closer to the goal than at the hit point by at least 0.15 m (standard Bug2);
   - the LiDAR sees free space in the ±45° cone around the goal direction (farther than the distance to the goal or 1 m, whichever is smaller) and the robot is closer to the goal than at the hit point (the shortcut of the advanced version of Practice 3).
4. **Emergency stop.** If anything is closer than 0.13 m in any direction, including the sides, the robot stops and does not move until the space is free again. This also applies when it turns in place.

If the robot completes a loop around an obstacle (it moves more than 0.5 m away from the hit point and comes back to within 0.2 m of it), the goal is declared unreachable and the mission stops.

### Localisation

The pose used for the control is the SLAM pose converted to the frame of the assignment (see SLAM). All the thresholds above are compared with this pose, so the drift of the odometry does not affect the m-line.

## PHASE 2: EXPLORATION AND MAPPING

Once at Q, the robot has to explore the corridor, find the charging station and return to P_Base. SLAM keeps running during the whole phase, so the map is built while the robot moves.

### Exploration

The robot follows a predefined sweep along the centre line of the corridor, `Cor_1 -> Cor_2 -> Cor_3 -> Cor_4`, and then goes to P_Base. It uses the same Bug2 navigation as in phase I, so the obstacle avoidance still works. The maximum speed is reduced from 0.25 to 0.15 m/s so that the LiDAR sees the pillars better.

### Pillar detection

For every scan, `detect_pillars` looks for objects that are small and isolated:

1. It groups consecutive rays whose distances differ by less than 0.08 m. The first and last rays are neighbours, because the scan is circular.
2. It discards the groups farther than `pillar_max_range` (2.5 m), where a 5 cm pillar is hit by almost no rays.
3. It discards the groups wider than 0.12 m (walls and large objects).
4. It discards the groups that are not isolated: a pillar has empty space (or a far wall) on both sides, while the end of a wall continues at a similar distance.
5. The LiDAR only sees the face of the pillar, so the centre is placed one pillar radius (2.5 cm) farther along the same direction. The offset of the LiDAR with respect to the centre of the robot (-3.2 cm) is also corrected.

### Accumulating landmarks

A single scan also contains corners of walls that look like pillars, so the detections are accumulated in the world frame (`update_landmarks`). Each detection is compared with the known landmarks: if one is closer than 0.15 m it is merged with it (running average of the position and a counter), otherwise a new landmark is created. A landmark is confirmed only after being seen at least `min_sightings` times (15). Wall corners are seen only a few times, while the real pillars are seen from many positions.

### Finding the station

With the confirmed landmarks (the 14 most seen), `find_square` tests every group of four and keeps the one that looks most like a square:

- the four short distances (sides) average between 0.28 and 0.50 m and differ from the average by at most 0.06 m;
- the two long distances (diagonals) equal the side times sqrt(2), with a tolerance of 0.08 m.

The centre of the station is the average of the four pillars. Its orientation is obtained by averaging the angles of the corners: since the corners are 90° apart, multiplying each angle by 4 makes them all equal, which avoids the ambiguity of the four sides. When the station is found it is stored (centre, side and orientation) for phase III.

### Fallbacks and map

If the station has not been confirmed when the robot reaches P_Base, it looks around for 5 s while standing still. If it still does not appear, it repeats the sweep in the opposite direction, so the pillars are seen from the other side. Finally the map is saved with `map_saver_cli` (up to three attempts) and the phase ends. If the station was not found the mission stops with an error, but the map is still saved.

### Mission log

While the robot moves, the node writes a CSV row every 0.5 s with: timestamp, phase, robot pose (x, y, yaw), distance to the closest obstacle in front, positions of the pillars of the station and position of the station.

## PHASE 3: PRECISION DOCKING

From P_Base the robot has to park in the exact centre of the station. In the phase II measurement the station position is only as accurate as the SLAM pose and the distance at which the pillars were seen, so in phase III the robot refines it with the LiDAR from close.

### Approach

1. **Entry side.** The station is a square, so it can be entered from four sides. We choose the one that faces the robot, i.e. the side whose normal gives the largest dot product with the vector from the station to the robot.
2. **Staging point.** A point is placed 0.9 m in front of that side, on its normal. The robot goes there with the same Bug2 navigation (so it avoids obstacles) and turns in place to look at the centre of the station.

### Closed-loop alignment

From the staging point, the robot controls its motion directly with the pillars it sees in each scan (only those closer than 1.6 m):

1. **Measurement.** `find_square` is run again, now accepting only 3 corners (`allow_three`): when the robot enters, the pillars at the back may be hidden by the ones in front. With 3 pillars, two sides of equal length at a right angle and the diagonal are enough to compute the centre (half of the diagonal). The measurement replaces the estimate of phase II, which is less precise.
2. **Only two pillars.** When the robot is inside and only the two front pillars are visible, `square_from_pair` computes the centre as the midpoint of the pair moved half a side inwards. It is accepted only if it is within 0.6 m of the centre known from the last good measurement, to avoid confusing it with other objects.
3. **Control.** The entry axis is the normal of the square that points to the robot. The lateral error is the distance of the centre to that axis.
   - If the lateral error is larger than 0.04 m, the robot targets a point of the axis at 0.55 m from the centre, so it first aligns itself outside the structure.
   - Otherwise the target is a point of the axis 0.25 m closer than the robot, so it advances.
   - If the heading error to the target is larger than 0.2 rad it rotates in place; otherwise it advances at 0.03 to 0.07 m/s while correcting the heading (gain 1.5).
4. **End.** The robot stops when the centre of the station is closer than 0.03 m to the centre of the robot.

### Safety

If no valid measurement is available for 3 s, the robot stops and aborts the docking instead of moving blindly. There is also a global timeout of 150 s. The emergency stop of phase I is active only during the approach to the staging point; the final alignment relies on the very low speed (at most 0.07 m/s).
