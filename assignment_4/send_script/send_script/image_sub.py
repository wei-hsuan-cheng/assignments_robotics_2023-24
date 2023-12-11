#!/usr/bin/env python
import rclpy

from rclpy.node import Node

import sys
# sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages') # robot 2 (right)
# sys.path.append('/home/robotics/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2 as cv

import os

import time

DIR = "/home/robotics/workspace2/team5_ws"
# DIR = "/home/robot/workspace2/team5_ws" # Robot 2 (right)

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)

        self.subscription = self.create_subscription(
            Image, 'techman_image', self.image_callback, 10)

    def image_callback(self, data):
        self.get_logger().info('Received image')

        bridge = CvBridge()

        img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        try:
            print(f"type(data) = {type(data)}")
            print(f"type(img) = {type(img)}")
            print(f"img.shape = {img.shape}")
            print(f"{DIR}")

            # dst_path = f"{DIR}/brick_images/img-{int(time.time() * 1000)}.png"
            dst_path = f"{DIR}/cap_img.png"

            print(f"dst_path = {dst_path}")

            cv.imwrite(dst_path, img)
        except:
            pass

        # TODO (write your code here)

def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

def main(args=None):
    print(f"adkoawd")
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    print(f"abc")
    rclpy.spin(node)
    print(f"bed")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()