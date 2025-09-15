# Sonar 3D-15 ROS Driver

## Overview

This package provides a ROS 2 driver for the **Water Linked Sonar 3D-15**, a real-time multibeam imaging sonar. The sonar streams 3D range images over UDP multicast, which are decoded and published as standard ROS messages:

- 3D point clouds: `sensor_msgs/PointCloud2` on `/sonar_point_cloud`
- Raw range images: `sensor_msgs/Image` on `/sonar_range_image`

The driver listens to RIP1 multicast packets, extracts and parses `RangeImage` protobuf messages, and converts the sonar data into formats usable by standard ROS visualization and processing tools.

---

## Features

- Receives and decodes **RIP1** packets via UDP multicast
- Publishes point clouds and range images at real-time rates
- Automatically enables sonar acoustics and udp multicast on startup
- Compatible with ROS 2 (tested on **Jazzy**)

---

## Topics

| Topic               | Message Type              | Description                        |
|--------------------|---------------------------|------------------------------------|
| `/sonar_point_cloud` | `sensor_msgs/PointCloud2` | 3D point cloud in ROS frame        |
| `/sonar_range_image` | `sensor_msgs/Image`       | Raw float32 range image (in meters) |

---

## Usage

### 1. Clone the repository into your ros2 workspace

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/waterlinked/Sonar-3D-15-ROS-driver.git
```

### 2. Install requirements

Install requirements:

```bash
pip install -r requirements.txt
```


### 3. Set IP-address of Sonar 3D-15

Change to your sonars IP-address in the sonar3d.launch.py file:

```python
    {'IP': '192.168.194.96'},  # Change to your sonar IP, '192.168.194.96' is the fallback ip.
```
Alternatively, you can modify the default parameter in `multicast_listener.py` directly.

```python
    self.declare_parameter('IP', '192.168.194.96')#  <-- your sonar's IP here, '192.168.194.96' is the fallback ip.
```

### 4. Build the package from the root of your ros project

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sonar3d
source install/local_setup.bash
```

### 5. Run the package

```bash
ros2 launch sonar3d sonar3d.launch.py
```

### 6. Playback of .sonar files and recording to ROS2 bagfile

A script has been included, which onverts Sonar 3D-15 .sonar files to ROS2 bagfiles.

Requirements:
- Tested with ROS2 Humble and Jazzy
- numpy, protobuf, sonar_3d_15_protcol_pb2.py
- sensor_msgs, std_msgs, builtin_interfaces
- cv_bridge (for ROS2)

Usage:
1. Start recording in one terminal:
```
ros2 bag record -o <output_bag_dir> /sonar3d/range_image /sonar3d/point_cloud
```
2. In another terminal, run:
```
python3 sonar_to_bag.py --file <sonar_file.sonar> --realtime-factor 1.0
```

The sonar_to_bag.py has been shared by Marios Xanthidis of SINTEF Ocean, with acknowledgements:

 - Supported by the Research Council of Norway (EchoNav: NO-359447)
 - Filtering and name conventions adapted from Alberto Quattrini Li @ Dartmouth
   His repository for ROS1 integration of the Sonar 3D-15 can be found in:
   https://github.com/quattrinili/Sonar-3D-15-api-example/tree/ros1

### 7. License

This package is distributed under the MIT License.


