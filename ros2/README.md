# ROS2 Interface for MagiClaw

This package provides a ROS2 interface for the MagiClaw robotic system, allowing users to control it through standard ROS messaging protocols.

## Installation

Install `magiclaw_msgs` package:

```bash
cd magiclaw-ros/magiclaw_msgs
rosdep install --from-paths src --ignore-src -r -y
colcon build --package-select magiclaw_msgs
source install/setup.bash
```

Compile the `proto` files:

```bash
cd magiclaw-ros/ros2/modules/protobuf
protoc --proto_path=. --python_out=. *.proto
```

Install `magiclaw_ros2` package:

```bash
cd magiclaw-ros/ros2
rosdep install --from-paths src --ignore-src -r -y
pip install .
```

## Usage

Start the ROS2 node:

```bash
ros2 run magiclaw magiclaw_pub --host <host> --port <port>
```

where `<host>` is the IP address of the MagiClaw and `<port>` is the port number.
