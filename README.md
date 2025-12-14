# F1Tenth Autonomous Racing Nodes

ROS2 packages for autonomous racing on the [F1Tenth](https://f1tenth.org/) platform: a 1/10th scale autonomous racing competition.

## Packages

### `gap_node` — Follow The Gap

Reactive obstacle avoidance using the Follow The Gap algorithm.

**How it works:**

1. Preprocesses LiDAR data (caps ranges, limits to ±60° forward cone)
2. Creates a "safety bubble" around the closest obstacle
3. Applies disparity extension to avoid clipping corners near depth discontinuities
4. Finds the largest contiguous gap in free space
5. Steers toward the furthest point in that gap

**Topics:**
| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Publish |
| `/scan_proc` | `sensor_msgs/LaserScan` | Publish (debug) |

---

### `wall_node` — Wall Following

PID-controlled wall following using two-point distance estimation.

**How it works:**

1. Measures distance to wall at two angles (125° and 180°)
2. Calculates wall angle using trigonometry
3. Projects future position to estimate lookahead error
4. PID controller adjusts steering to maintain desired wall distance
5. Speed scales with steering angle (slower in tight turns)

**Topics:**
| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Publish |

---

### `safety_node` — Automatic Emergency Braking

Safety system that triggers emergency braking when collision is imminent.

**How it works:**

1. Monitors LiDAR ranges and vehicle velocity
2. Calculates instantaneous Time-To-Collision (iTTC) for each beam
3. Triggers emergency stop when iTTC < 1 second
4. Automatically resets when obstacle clears

**Topics:**
| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Subscribe |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Publish |

---

## Dependencies

- ROS2 (tested on Foxy/Humble)
- `sensor_msgs`
- `ackermann_msgs`
- `nav_msgs`
- `numpy`

## Build

```bash
cd ~/f1tenth_ws  # or your workspace
colcon build --packages-select gap_node safety_node wall_node
source install/setup.bash
```

## Run

```bash
# Follow The Gap
ros2 run gap_node gap_node

# Wall Following
ros2 run wall_node wall_node

# Safety Node (run alongside other nodes)
ros2 run safety_node safety_node
```
