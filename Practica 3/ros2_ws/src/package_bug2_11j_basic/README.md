# package_bug2_11j_basic

Bug2 (versión básica) para el TurtleBot3 burger en ROS 2 Jazzy. Solo usa el
LiDAR (`/scan`) y la odometría (`/odom`); no usa ningún mapa.

## Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select package_bug2_11j_basic
source install/setup.bash
```

## Ejecutar (un solo comando)

Simulación (arranca Gazebo, el robot y el nodo Bug2):

```bash
ros2 launch package_bug2_11j_basic bug2.launch.py world:=box
```

Robot real (con `robot.launch.py` ya en marcha en la Raspberry del TB3):

```bash
ros2 launch package_bug2_11j_basic bug2.launch.py sim:=false
```

Argumentos del launch:

| Argumento | Por defecto | Qué es |
|-----------|-------------|--------|
| `sim` | `true` | `true` arranca Gazebo; `false` solo el nodo (robot real) |
| `world` | `box` | Escenario de Gazebo: `empty`, `box`, `u_shape`, `multi` o `enclosed` |
| `goal_x` | `2.7` | Objetivo X en metros (frame `odom`, relativo a la pose inicial del robot) |
| `goal_y` | `0.0` | Objetivo Y en metros (frame `odom`) |
| `advanced` | `false` | `true`: versión avanzada, sale de la pared en cuanto el LiDAR ve camino libre hacia el objetivo |

La m-line se calcula con la primera posición recibida por `/odom` y el objetivo. No hace
falta recompilar para cambiar el objetivo ni el modo.

Escenarios del enunciado:

```bash
ros2 launch package_bug2_11j_basic bug2.launch.py world:=empty                # mundo vacío
ros2 launch package_bug2_11j_basic bug2.launch.py world:=box                  # obstáculo convexo
ros2 launch package_bug2_11j_basic bug2.launch.py world:=u_shape              # obstáculo cóncavo (U)
ros2 launch package_bug2_11j_basic bug2.launch.py world:=multi goal_x:=5.0    # varios obstáculos
ros2 launch package_bug2_11j_basic bug2.launch.py world:=u_shape advanced:=true   # versión avanzada
```

## Cómo funciona

La **m-line** es la recta que une la posición inicial del robot con el objetivo.
El nodo (`package_bug2_11j_basic/bug2.py`) alterna entre dos conductas:

1. `follow_line`: se orienta hacia el objetivo y avanza recto por la m-line.
   Si el LiDAR ve un obstáculo delante a menos de `safe_distance`, guarda la distancia
   al objetivo en ese punto (*hit point*) y pasa a `follow_wall`.
2. `follow_wall`: rodea el obstáculo dejando la pared a su derecha, a `wall_distance`.
   Cuando vuelve a estar sobre la m-line **y** más cerca del objetivo que en el
   hit point, deja la pared y vuelve a `follow_line`.

El robot se para cuando está a menos de 8 cm del objetivo.

Funciones del nodo:

- `odom_callback`: guarda la posición (x, y) y la orientación (yaw) del robot.
- `scan_callback`: guarda la distancia mínima delante (±30º), a la derecha (−110º a −30º) y
  en el cono de ±45º hacia el objetivo.
- `run_bug2`: bucle principal con `rclpy.spin_once`; en cada vuelta decide si toca
  `follow_line`, `follow_wall` o parar.
- `follow_line` y `follow_wall`: las dos conductas descritas arriba.
- `publish_velocity` / `stop`: publican la velocidad en `/cmd_vel` (`TwistStamped`).

## Objetivo inalcanzable

Al chocar se guarda el hit point. Mientras rodea la pared, el robot marca `left_hit` cuando
se aleja más de 0.5 m de él. Si después vuelve a menos de 0.2 m sin haber podido salir
(ni por la m-line ni por el atajo avanzado), ha dado la vuelta completa: se detiene y
avisa con `OBJETIVO INALCANZABLE`. Se prueba con:

```bash
ros2 launch package_bug2_11j_basic bug2.launch.py world:=enclosed
```

(el objetivo por defecto `(2.7, 0)` queda dentro de un recinto cerrado).
