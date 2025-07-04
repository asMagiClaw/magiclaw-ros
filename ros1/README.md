# ROS1 Interface for MagiClaw

This package provides a ROS1 interface for the MagiClaw robotic system, allowing users to control it through standard ROS messaging protocols.

## Installation

Compile the `proto` files:

```bash
cd magiclaw-ros/ros1/magiclaw/modules/protobuf
protoc --proto_path=. --python_out=. *.proto
```

Install dependencies:

```bash
cd magiclaw-ros/ros1
rosdep install --from-paths src --ignore-src -r -y
```

Install the `magiclaw_msgs` package:

```bash
cd magiclaw-ros/ros1/magiclaw_msgs
catkin_make
source devel/setup.bash
```

Install `magiclaw_ros1` package:

```bash
cd magiclaw-ros/ros1/magiclaw
catkin_make
source devel/setup.bash
```

## Usage

Start the ROS1 node:

```bash
rosrun magiclaw magiclaw_pub.py --host <host> --port <port>
```

where `<host>` is the IP address of the MagiClaw and `<port>` is the port number.
