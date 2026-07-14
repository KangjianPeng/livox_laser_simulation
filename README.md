# Livox LiDAR Simulation

A ROS 2 system plugin that simulates non-repetitive Livox scan patterns in
Ignition Gazebo / Gazebo Sim. It is based on the original
[Livox Gazebo Classic plugin](https://github.com/Livox-SDK/livox_laser_simulation)
and retains per-point timing and Livox-compatible output formats without a
dependency on the Livox SDK or driver.

This branch targets ROS 2 Humble and Ignition Gazebo 6 (Fortress). Gazebo
Classic 11 and ROS 1 sources have been removed.

## Features

- Includes measured scan patterns for Avia, Horizon, Mid-40, Mid-70,
  Mid-360, and Tele-15.
- Publishes ROS 2 `sensor_msgs/msg/PointCloud2` directly from the simulator.
- Supports XYZ, Livox-style PointCloud2, and Livox custom message output.
- Uses simulation time for message headers and per-point timestamps.
- Supports GUI and server-only headless rendering.
- Resolves scan patterns from the installed package, so SDF files are portable.

![Undistorted point cloud comparison](./images/undistort.png)

## Scan Patterns

| Model | Pattern file | `line_count` | `point_rate` |
| --- | --- | ---: | ---: |
| Livox Avia | `avia.csv` | 6 | 240000 |
| Livox Horizon | `horizon.csv` | 6 | 240000 |
| Livox Mid-40 | `mid40.csv` | 1 | 100000 |
| Livox Mid-70 | `mid70.csv` | 1 | 100000 |
| Livox Mid-360 | `mid360.csv` | 4 | 200000 |
| Livox Tele-15 | `tele.csv` | 6 | 240000 |

The packaged SDF model is a Mid-360 example. The plugin itself is model
independent: select another pattern file and its matching line count and point
rate when embedding it in a robot model.

## Supported Environment

| Component | Tested version |
| --- | --- |
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Ignition Gazebo | Fortress / Gazebo Sim 6.18 |
| Compiler | GCC 11 |

## Dependencies

Install ROS 2 Humble, Gazebo Fortress, and the ROS / Gazebo integration:

```bash
sudo apt update
sudo apt install \
  ros-humble-ros-gz \
  ros-humble-rviz2 \
  ros-humble-tf2-ros
```

## Build

Clone the repository into a ROS 2 workspace and build it with `colcon`:

```bash
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/fratopa/Mid360_simulation_plugin.git livox_laser_simulation
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run The Example

Start Gazebo, spawn the Mid-360 model, bridge `/clock`, and open RViz:

```bash
ros2 launch livox_laser_simulation simulation.launch.py
```

The example keeps the lidar static at the center of a four-wall, open-top test
room. The walls are translucent in Gazebo, and the ground is collision-only, so
RViz shows wall returns without unrelated ground points while the sensor model
remains visible from above.

For SSH, CI, or machines without a display:

```bash
ros2 launch livox_laser_simulation simulation.launch.py \
  headless:=true rviz:=false
```

Verify the output:

```bash
ros2 topic info /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --field fields
```

## Add The Plugin To A Model

The plugin must be attached to an SDF sensor. The `gpu_lidar` sensor causes
Gazebo's Sensors system to create and maintain the server rendering scene used
for ray intersection queries.

```xml
<sensor type="gpu_lidar" name="livox_mid360">
  <pose>0 0 0.05 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <visualize>false</visualize>
  <topic>/livox/lidar/scan</topic>
  <ray>
    <scan>
      <horizontal>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>40.0</max>
      <resolution>1</resolution>
    </range>
  </ray>
  <plugin filename="liblivox_laser_simulation.so"
          name="livox_laser_simulation::LivoxLidarSystem">
    <csv_file_name>mid360.csv</csv_file_name>
    <samples>20000</samples>
    <downsample>10</downsample>
    <line_count>4</line_count>
    <point_rate>200000</point_rate>
    <min_range>0.1</min_range>
    <max_range>40.0</max_range>
    <update_rate>10.0</update_rate>
    <publish_pointcloud_type>2</publish_pointcloud_type>
    <ros_topic>/livox/lidar</ros_topic>
    <frame_name>livox_frame</frame_name>
    <use_inf>false</use_inf>
  </plugin>
</sensor>
```

The built-in `gpu_lidar` scan is intentionally 1x1 and is not published to
ROS. It only keeps Gazebo's rendering scene active; the plugin performs the
actual Livox ray queries from the selected CSV pattern.

The world must load the Sensors system with a rendering engine:

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

Ensure the package's `lib` directory is in `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`
and the parent of its share directory is in `IGN_GAZEBO_RESOURCE_PATH`. The
provided launch file configures both variables automatically.

## Plugin Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `csv_file_name` | required | Absolute path or a filename under the package `scan_patterns` directory. |
| `samples` | `20000` | Number of consecutive pattern entries advanced per publication. |
| `downsample` | `1` | Reduce ray queries by N while balancing the configured interleaved lines. |
| `line_count` | `4` | Number of interleaved laser lines in the selected pattern. |
| `point_rate` | `200000` | Device point rate in points per second, used for per-point timestamps. |
| `min_range` | `0.1` | Minimum accepted range in metres. |
| `max_range` | `40.0` | Maximum accepted range in metres. |
| `update_rate` | `10.0` | Requested publication rate in Hz. Expensive ray queries may limit the achieved rate. |
| `ros_topic` | `/livox/lidar` | ROS 2 output topic. |
| `frame_name` | sensor name | Point cloud child frame. |
| `use_inf` | `false` | Emit max-range points when no object is hit. |
| `publish_pointcloud_type` | `2` | Output message layout, described below. |

Output types:

- `1`: PointCloud2 fields `x`, `y`, `z`.
- `2`: PointCloud2 fields `x`, `y`, `z`, `intensity`, `tag`, `line`, `timestamp`.
- `3`: `livox_laser_simulation/msg/CustomMsg` with per-point `offset_time`.

`samples / downsample` is the number of rendering ray queries per update. The
Mid-360 example advances the full 20,000 pattern entries at 10 Hz so scan timing
and pattern progression match the 200,000 point/s device rate. It queries one
ray from each block of 10 entries, resulting in 2,000 rendering queries per
update and 20,000 per second. Each block contributes a ray from the least
represented scan line, preventing downsampling from repeatedly selecting only a
subset of the four lines.

Gazebo Rendering evaluates these ray queries individually. On the tested host,
the example produced about 1,460 valid wall returns per frame at 9.4 Hz wall
time while preserving a 10 Hz simulation-time rate. Set `downsample` closer to
`1` only when the host can absorb the additional rendering cost; `1` performs
the full 200,000 ray queries per second.

## Acknoledgements
https://github.com/fratopa/Mid360_simulation_plugin
https://github.com/Livox-SDK/livox_laser_simulation