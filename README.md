# WATonomous ASD Admission Assignment

A ROS2-based autonomous navigation system for a simulated differential-drive robot. The robot uses a laser scanner to detect obstacles, builds a map of its environment, plans a path using A*, and follows it using Pure Pursuit control.

---

## Architecture

```
/lidar (LaserScan)
     │
     ▼
┌─────────────┐     /costmap (OccupancyGrid)     ┌──────────────┐
│ costmap_node│ ──────────────────────────────── ▶│map_memory_no │
└─────────────┘                                   │     de       │
                                                  └──────┬───────┘
/odom/filtered (Odometry) ──────────────────────────────┘
     │                         /map (OccupancyGrid)
     │                              │
     ▼                              ▼
┌─────────────┐ ◀────────────────────────────────┌──────────────┐
│ planner_node│                                  │              │
└──────┬──────┘     /path (Path)                 │              │
       │ ──────────────────────────────────────▶ │ control_node │
       │                                         │              │
/goal_point (PointStamped) ──────────────────────▶              │
                                                 └──────┬───────┘
                                                        │ /cmd_vel (Twist)
                                                        ▼
                                                     🤖 Robot
```

---

## Nodes

### 1. `costmap_node` — Perception
Subscribes to `/lidar` (LaserScan) and converts each scan into a local occupancy grid centred on the robot. Obstacles are marked at cost 100 and inflated outward using a linear decay over a 1 metre radius.

- **Subscribes:** `/lidar` → `sensor_msgs/LaserScan`
- **Publishes:** `/costmap` → `nav_msgs/OccupancyGrid`
- **Frame:** `robot/chassis/lidar`
- **Grid:** 20 m × 20 m at 0.1 m/cell resolution

### 2. `map_memory_node` — World Modeling & Memory
Fuses local costmaps into a persistent global map. Only updates when the robot has moved at least 1.5 m, keeping computation low. Each costmap is transformed from robot frame into the global `sim_world` frame using odometry.

- **Subscribes:** `/costmap` → `nav_msgs/OccupancyGrid`, `/odom/filtered` → `nav_msgs/Odometry`
- **Publishes:** `/map` → `nav_msgs/OccupancyGrid`
- **Frame:** `sim_world`
- **Grid:** 40 m × 40 m at 0.1 m/cell resolution
- **Update trigger:** robot moves ≥ 1.5 m, checked every 1 second

### 3. `planner_node` — Planning
Implements A* pathfinding on the global map. Operates as a two-state machine: waiting for a goal, then navigating. Replans whenever the map updates. Publishes an empty path when the goal is reached.

- **Subscribes:** `/map`, `/odom/filtered`, `/goal_point` → `geometry_msgs/PointStamped`
- **Publishes:** `/path` → `nav_msgs/Path`
- **Frame:** `sim_world`
- **Algorithm:** A* with 8-connected grid, Euclidean heuristic, inflation-weighted cost
- **Obstacle threshold:** cells with cost ≥ 50 are treated as blocked

### 4. `control_node` — Action
Follows the planned path using Pure Pursuit control. Selects a lookahead point on the path 1 m ahead of the robot and computes the angular velocity needed to steer toward it. Stops when within 0.4 m of the final waypoint.

- **Subscribes:** `/path`, `/odom/filtered`
- **Publishes:** `/cmd_vel` → `geometry_msgs/Twist`
- **Algorithm:** Pure Pursuit
- **Lookahead distance:** 1.0 m
- **Linear speed:** 0.4 m/s
- **Control rate:** 10 Hz

---

## File Structure

```
src/robot/
├── costmap/
│   ├── include/
│   │   ├── costmap_core.hpp      ← scaffold (provided by repo)
│   │   └── costmap_node.hpp      ← our node header
│   ├── src/
│   │   ├── costmap_core.cpp      ← scaffold (provided by repo)
│   │   └── costmap_node.cpp      ← our implementation
│   ├── CMakeLists.txt
│   └── package.xml
├── map_memory/
│   ├── include/
│   │   └── map_memory_node.hpp
│   ├── src/
│   │   ├── map_memory_core.cpp   ← empty placeholder
│   │   └── map_memory_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── planner/
│   ├── include/
│   │   └── planner_node.hpp
│   ├── src/
│   │   ├── planner_core.cpp      ← empty placeholder
│   │   └── planner_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
└── control/
    ├── include/
    │   └── control_node.hpp
    ├── src/
    │   ├── control_core.cpp      ← empty placeholder
    │   └── control_node.cpp
    ├── CMakeLists.txt
    └── package.xml
```

---

## Setup & Running

### Prerequisites
- Docker Engine (or Docker Desktop)
- Git

### Clone and build

```bash
git clone git@github.com:WATonomous/wato_asd_training.git
cd wato_asd_training
```

Set active modules in `watod-config.sh`:
```bash
ACTIVE_MODULES="robot gazebo vis_tools"
```

Build and start everything:
```bash
./watod build
./watod up
```

### Useful commands

```bash
# Rebuild only the robot after code changes
./watod down robot && ./watod build robot && ./watod up robot

# View robot logs
./watod logs robot

# View gazebo logs
./watod logs gazeboserver

# Check container status
./watod ps
```

---

## Foxglove Visualization

1. Open [Foxglove](https://app.foxglove.dev) and connect to `ws://localhost:20000`
2. Import the layout from `config/wato_asd_training_foxglove_config.json`
3. In the 3D panel, set **Fixed frame** to `sim_world`
4. Import the pre-made layout to see the robot, costmap, and path

### Sending a goal

In Foxglove, open a **Publish** panel, set the topic to `/goal_point` with type `geometry_msgs/PointStamped`, and publish:

```json
{
  "header": {
    "stamp": { "sec": 0, "nanosec": 0 },
    "frame_id": "sim_world"
  },
  "point": {
    "x": 0,
    "y": 0,
    "z": 0
  }
}
```

The robot will plan a path and start driving.

---

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/lidar` | `sensor_msgs/LaserScan` | Raw laser scan from Gazebo |
| `/odom/filtered` | `nav_msgs/Odometry` | Robot position in `sim_world` frame |
| `/costmap` | `nav_msgs/OccupancyGrid` | Local obstacle map (robot frame) |
| `/map` | `nav_msgs/OccupancyGrid` | Global fused map (sim_world frame) |
| `/goal_point` | `geometry_msgs/PointStamped` | Target position input |
| `/path` | `nav_msgs/Path` | A* planned path |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to robot |

---

## TF Frame Tree

```
sim_world
└── robot
    ├── robot/left_wheel
    ├── robot/right_wheel
    ├── robot/caster
    └── robot/chassis
        ├── robot/chassis/lidar
        ├── robot/chassis/camera
        └── robot/chassis/imu_sensor
```

---

## Debugging

Check if a topic is publishing:
```bash
docker exec -it watod_ads_armaan-robot-1 bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic echo /cmd_vel"
```

Check the TF tree:
```bash
docker exec -it watod_ads_armaan-robot-1 bash -c \
  "source /opt/ros/humble/setup.bash && ros2 run tf2_tools view_frames"
```

Check all active topics:
```bash
docker exec -it watod_ads_armaan-robot-1 bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list"
```

---

## Dependencies

| Package | Used for |
|---|---|
| `rclcpp` | ROS2 C++ client library |
| `sensor_msgs` | LaserScan message type |
| `nav_msgs` | OccupancyGrid, Odometry, Path message types |
| `geometry_msgs` | Twist, PointStamped message types |
| `tf2` | Transform utilities |

---

## How it works — the short version

1. The laser scanner sees obstacles and `costmap_node` marks them on a local grid with an inflation radius around each one.
2. As the robot moves, `map_memory_node` stitches those local grids into one big global map by transforming each costmap into the world frame using odometry.
3. When you publish a goal, `planner_node` runs A* on the global map and outputs a list of waypoints avoiding all known obstacles.
4. `control_node` follows those waypoints using Pure Pursuit — it picks a point 1 m ahead on the path and steers toward it continuously until the robot arrives.
