# Sonar 3D-15 ROS Driver

## Overview

This package provides a ROS 2 driver for the **Water Linked Sonar 3D-15**, a real-time multibeam imaging sonar. The sonar streams data over UDP multicast, which is decoded and published as standard ROS messages.

Range Image Protocol parsing and sonar configuration are handled by the official [`wlsonar`](https://pypi.org/project/wlsonar/) Python package ([source](https://github.com/waterlinked/wlsonar)), so both **RIP1** and **RIP2** packets are supported.

---

## Features

- Receives and decodes **RIP1** and **RIP2** packets via UDP multicast (RIP2 packets are snappy-compressed)
- Publishes point clouds, range images, and signal-strength images
- Automatically enables sonar acoustics and UDP multicast output on startup
- Replays recorded `.sonar` files into ROS 2 (for recording to a bag)
- Compatible with ROS 2 (tested on **Jazzy**, should also work on **Humble**)

---

## Topics

The live driver (`sonar_publisher`) and the file player (`sonar_to_bag`) publish the same topics:

| Topic                            | Message Type              | Source message          | Description                                  |
|----------------------------------|---------------------------|-------------------------|----------------------------------------------|
| `/sonar3d/point_cloud`           | `sensor_msgs/PointCloud2` | `RangeImage`            | 3D point cloud (xyz, meters)                 |
| `/sonar3d/range_image`           | `sensor_msgs/Image`       | `RangeImage`            | Range image, `32FC1`, distance in meters     |
| `/sonar3d/signal_strength_image` | `sensor_msgs/Image`       | `BitmapImageGreyscale8` | Signal-strength image, `mono8`               |

The point cloud coordinate convention follows `wlsonar.range_image_to_xyz`: `x` forward (range), `y` horizontal, `z` vertical.

---

## Installation

### 1. Clone the repository into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/waterlinked/Sonar-3D-15-ROS-driver.git
```

### 2. Install the Python requirements

The driver depends on the [`wlsonar`](https://pypi.org/project/wlsonar/) package:

```bash
pip install -r src/sonar3d/requirements.txt
```

### 3. Build the package

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sonar3d
source install/local_setup.bash
```

---

## Usage

### Live driver

Set your sonar's IP in [`launch/sonar3d.launch.py`](src/sonar3d/launch/sonar3d.launch.py):

```python
{'IP': '192.168.194.96'},  # Change to your sonar IP. '192.168.194.96' is the fallback IP.
```

Then launch:

```bash
ros2 launch sonar3d sonar3d.launch.py
```

Parameters:

| Parameter        | Type     | Default           | Description                                                                 |
|------------------|----------|-------------------|-----------------------------------------------------------------------------|
| `IP`             | `string` | `192.168.194.96`  | IP address of the sonar.                                                    |
| `speed_of_sound` | `double` | `0.0`             | Speed of sound in m/s. `0.0` leaves the sonar setting unchanged. Setting a value can take ~20 s. |
| `frame_id`       | `string` | `sonar3d`         | `frame_id` used in published message headers.                               |

### Playback of `.sonar` files and recording to a ROS 2 bag

A recorded `.sonar` file (RIP1 or RIP2) can be replayed as ROS 2 messages and recorded to a bag.

1. Start recording in one terminal:

```bash
ros2 bag record -o <output_bag_dir> \
    /sonar3d/point_cloud /sonar3d/range_image /sonar3d/signal_strength_image
```

2. In another terminal, replay the file (paced to the original timestamps):

```bash
ros2 run sonar3d sonar_to_bag --file <recording.sonar> --realtime-factor 1.0
```

Options: `--realtime-factor` (playback speed, `1.0` = real time), `--frame-id` (default `sonar3d`).

The `sonar_to_bag` playback was originally contributed by Marios Xanthidis of SINTEF Ocean, with acknowledgements:

 - Supported by the Research Council of Norway (EchoNav: NO-359447)
 - Filtering and name conventions adapted from Alberto Quattrini Li @ Dartmouth.
   His repository for ROS1 integration of the Sonar 3D-15 can be found at:
   https://github.com/quattrinili/Sonar-3D-15-api-example/tree/ros1

---

## License

This package is distributed under the MIT License.
