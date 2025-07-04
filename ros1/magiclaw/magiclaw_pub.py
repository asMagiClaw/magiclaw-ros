#!/usr/bin/env python

"""
MagiClaw Publisher for ROS
========================

This script publishes the state of the MagiClaw robot to a ROS topic.
It subscribes to messages from a ZMQ server and converts them into ROS messages.
It uses the `magiclaw_msgs` package for message definitions and `cv_bridge` for image conversion.

Usage:
    rosrun magiclaw magiclaw_pub.py --host <host> --port <port>

where `<host>` is the address of the magiclaw and `<port>` is the port number.
"""

import argparse
import numpy as np
import rospy
from magiclaw_msgs.msg import MagiClaw, Claw, Motor, Finger, Phone
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from magiclaw.modules.zmq import MagiClawSubscriber

# Initialize CvBridge for converting OpenCV images to ROS Image messages
bridge = CvBridge()


def np_to_imgmsg(np_arr: np.ndarray, encoding: str = "mono8") -> Image:
    """
    Convert a NumPy array to a ROS Image message.

    Args:
        np_arr (np.ndarray): The NumPy array to convert.
        encoding (str): The encoding of the image (default is "mono8").

    Returns:
        Image: The converted ROS Image message.
    """

    try:
        return bridge.cv2_to_imgmsg(np_arr, encoding=encoding)
    except Exception:
        # Return empty if conversion fails
        return Image()


def main():
    """
    Main function to initialize the ROS node and start publishing messages.
    """

    # Argument parser for command line arguments
    parser = argparse.ArgumentParser(description="MagiClaw Publisher")
    parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        help="ZMQ host address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=6300,
        help="ZMQ port number",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=30,
        help="Publishing rate in Hz",
    )
    args, _ = parser.parse_known_args()

    # Initialize the ROS node and publisher
    rospy.loginfo("Starting MagiClaw Publisher to /magiclaw/state")
    rospy.init_node("magiclaw_publisher")
    pub = rospy.Publisher("/magiclaw/state", MagiClaw, queue_size=10)

    # Create a subscriber to the MagiClaw messages
    rospy.loginfo("Subscribing from %s:%d", args.host, args.port)
    sub = MagiClawSubscriber(host=args.host, port=args.port)

    # Set up the rate for publishing messages
    rate = rospy.Rate(args.rate)

    # Main loop to continuously receive and publish messages
    while not rospy.is_shutdown():
        try:
            # Receive a message from the MagiClaw subscriber
            (
                claw_angle,
                motor_angle,
                motor_speed,
                motor_iq,
                motor_temperature,
                finger_0_img,
                finger_0_pose,
                finger_0_force,
                finger_0_node,
                finger_1_img,
                finger_1_pose,
                finger_1_force,
                finger_1_node,
                phone_color_img,
                phone_depth_img,
                phone_local_pose,
                phone_global_pose,
                magiclaw_pose,
            ) = sub.subscribeMessage()

            # Create a MagiClaw message
            msg = MagiClaw()
            msg.timestamp = rospy.Time.now().to_sec()

            # Create a Claw message and populate its fields
            msg.claw = Claw()
            msg.claw.angle = claw_angle
            msg.claw.motor = Motor()
            msg.claw.motor.angle = motor_angle
            msg.claw.motor.speed = motor_speed
            msg.claw.motor.iq = motor_iq
            msg.claw.motor.temperature = motor_temperature

            # Create Finger messages for both fingers and populate their fields
            msg.finger_0 = Finger()
            msg.finger_0.pose = finger_0_pose.tolist()
            msg.finger_0.force = finger_0_force.tolist()
            msg.finger_0.node = finger_0_node.flatten().tolist()
            msg.finger_0.img = np_to_imgmsg(np.frombuffer(finger_0_img, dtype=np.uint8))

            msg.finger_1 = Finger()
            msg.finger_1.pose = finger_1_pose.tolist()
            msg.finger_1.force = finger_1_force.tolist()
            msg.finger_1.node = finger_1_node.flatten().tolist()
            msg.finger_1.img = np_to_imgmsg(np.frombuffer(finger_1_img, dtype=np.uint8))

            # Create a Phone message and populate its fields
            msg.phone = Phone()
            msg.phone.local_pose = phone_local_pose.tolist()
            msg.phone.global_pose = phone_global_pose.tolist()
            msg.phone.color_img = np_to_imgmsg(
                np.frombuffer(phone_color_img, dtype=np.uint8)
            )
            msg.phone.depth_img = np_to_imgmsg(
                np.frombuffer(phone_depth_img, dtype=np.uint8), encoding="mono16"
            )

            # Populate the magiclaw_pose field
            msg.pose = magiclaw_pose.tolist()

            # Publish the received message
            pub.publish(msg)

        except Exception as e:
            # Log any exceptions that occur during message processing
            rospy.logwarn("Failed to receive/convert message: %s", str(e))

        # Sleep to maintain the specified publishing rate
        rate.sleep()


if __name__ == "__main__":
    main()
