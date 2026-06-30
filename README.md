# vrpn_client_ros2

`vrpn_client_ros2` is a ROS 2 Jazzy client for publishing tracker data from a
VRPN server, such as an OptiTrack motion capture setup that exposes tracker
poses through VRPN.

The package publishes pose, twist, accel, and optional TF data using standard
ROS 2 parameters and launch arguments. It targets Ubuntu 24.04 with ROS 2
Jazzy.

## Dependencies

Install ROS 2 Jazzy and the VRPN packages:

```bash
sudo apt update
sudo apt install ros-jazzy-vrpn
```

The ROS dependencies are declared in `package.xml` and are resolved by
`rosdep` in a normal ROS workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

Use a normal ROS workspace layout:

```text
ros2_ws/
  src/
    vrpn_client_ros2/
```

Then build from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vrpn_client_ros2
source install/setup.bash
```

Avoid building repeatedly from inside the package directory. If `build/`,
`install/`, or `log/` are created inside this package, remove them before
running lint tests so generated files are not scanned as source files.

## Launch

Start the client with the default parameter file:

```bash
ros2 launch vrpn_client_ros2 launch_vrpn_client.launch.py
```

Common launch overrides:

```bash
ros2 launch vrpn_client_ros2 launch_vrpn_client.launch.py \
  server:=127.0.0.1 \
  port:=3883 \
  node_namespace:=/vrpn \
  trackers:=FirstTracker,SecondTracker
```

Show all launch arguments:

```bash
ros2 launch vrpn_client_ros2 launch_vrpn_client.launch.py --show-args
```

Most launch arguments default to `__use_yaml__`. That sentinel means the value
from `parameters_file` is used. A parameter is overridden only when the launch
argument is explicitly set, for example `server:=127.0.0.1`.

## Parameters

The default parameter file is `config/param.yaml`. It uses `/**` so the same
file works even when `node_name` is changed through launch.

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `server` | string | `127.0.0.1` | VRPN server host or IP address. |
| `port` | int | `3883` | VRPN server port. |
| `update_frequency` | double | `100.0` | VRPN mainloop polling frequency in Hz. |
| `refresh_tracker_frequency` | double | `1.0` | Tracker discovery frequency in Hz. Use `0.0` to disable discovery. |
| `frame_id` | string | `world` | Parent frame id for messages and TF. |
| `use_server_time` | bool | `false` | Use VRPN server timestamps instead of ROS time. |
| `broadcast_tf` | bool | `true` | Publish TF transforms for tracker poses. |
| `process_sensor_id` | bool | `false` | Add VRPN sensor id to topic paths and TF child frames. |
| `trackers` | string array | `[""]` | Fixed tracker list. Empty string uses VRPN sender discovery. |
| `topic_prefix` | string | empty | Optional relative prefix before tracker topic names. |
| `pose_topic` | string | `pose` | Pose topic basename under each tracker. |
| `twist_topic` | string | `twist` | Twist topic basename under each tracker. |
| `accel_topic` | string | `accel` | Accel topic basename under each tracker. |

## Topics and TF

With `node_namespace:=/vrpn`, `topic_prefix:=`, and tracker `Tracker1`, the
default output topics are:

```text
/vrpn/Tracker1/pose
/vrpn/Tracker1/twist
/vrpn/Tracker1/accel
```

With `topic_prefix:=mocap`, the topics become:

```text
/vrpn/mocap/Tracker1/pose
/vrpn/mocap/Tracker1/twist
/vrpn/mocap/Tracker1/accel
```

When `process_sensor_id` is true and VRPN reports sensor `0`, the topics become:

```text
/vrpn/Tracker1/0/pose
/vrpn/Tracker1/0/twist
/vrpn/Tracker1/0/accel
```

If `broadcast_tf` is true, each pose update also publishes a transform from
`frame_id` to the tracker name. With `process_sensor_id` enabled, the TF child
frame is `Tracker1/0`.

Tracker names are sanitized before being used as ROS topic or frame names.
Invalid characters are replaced with underscores, and duplicate sanitized names
receive numeric suffixes.

## Fixed Trackers vs Discovery

By default, `trackers` contains an empty string and `refresh_tracker_frequency`
is `1.0`, so the node discovers senders from the VRPN connection. The empty
string keeps the ROS 2 YAML parser aware that this parameter is a string array.

To create only known trackers, set a fixed list and disable discovery:

```yaml
/**:
  ros__parameters:
    refresh_tracker_frequency: 0.0
    trackers:
      - FirstTracker
      - SecondTracker
```

The same can be done from launch:

```bash
ros2 launch vrpn_client_ros2 launch_vrpn_client.launch.py \
  trackers:=FirstTracker,SecondTracker \
  refresh_tracker_frequency:=0.0
```

## Verification

Build:

```bash
colcon build --packages-select vrpn_client_ros2 --event-handlers console_direct+
```

Test:

```bash
rm -rf build install log
colcon test --packages-select vrpn_client_ros2 --event-handlers console_direct+
colcon test-result --verbose
```

Launch argument check:

```bash
ros2 launch vrpn_client_ros2 launch_vrpn_client.launch.py --show-args
```

## Troubleshooting

- If the node starts before the VRPN server is reachable, it stays alive and
  keeps polling. Connection warnings are throttled.
- If no tracker topics appear, confirm the VRPN server exposes tracker senders
  and check whether `trackers` is empty or matches the exact VRPN sender names.
- If topic paths are not where expected, check `node_namespace`,
  `topic_prefix`, and `process_sensor_id`.
- If timestamps look old or inconsistent, leave `use_server_time` as `false`
  unless the VRPN server clock is synchronized with the ROS system.
- If TF is not desired, launch with `broadcast_tf:=false`.
