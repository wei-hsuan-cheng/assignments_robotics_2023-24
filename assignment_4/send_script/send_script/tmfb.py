#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from tm_msgs.msg import FeedbackState

r2d = 180 / np.pi

class ArmFeedbackSubscriberNode(Node):

    def __init__(self):
        super().__init__('arm_feedback_sub')
        self.arm_feedback_sub = self.create_subscription(
            FeedbackState, 
            'feedback_states', 
            self.feedback_states_callback, 
            10)

    def feedback_states_callback(self, msg):
        ##### Joint feedback
        joint_angles, joint_velocities = np.zeros(6), np.zeros(6)
        for i in range(0,6):
            joint_angles[i] = round(msg.joint_pos[i] * r2d, 3)
            joint_velocities[i] = round(msg.joint_vel[i] * r2d, 3)
            
        self.get_logger().info(
            "\n"
            # + f"joint_angles [deg] = ({joint_angles[0]}, {joint_angles[1]}, {joint_angles[2]}, {joint_angles[3]}, {joint_angles[4]}, {joint_angles[5]}) \n"
            # + f"joint_velocities [deg/s] = ({joint_velocities[0]}, {joint_velocities[1]}, {joint_velocities[2]}, {joint_velocities[3]}, {joint_velocities[4]}, {joint_velocities[5]}) \n"
            # + f"joint_tor [] = ({msg.joint_tor[0]}, {msg.joint_tor[1]}, {msg.joint_tor[2]}, {msg.joint_tor[3]}, {msg.joint_tor[4]}, {msg.joint_tor[5]}) \n"
            )
        
        # ##### Cartesian feedback 
        self.get_logger().info(
            f"Cartesian feedback [m] = ({msg.tool_pose[0]}, {msg.tool_pose[1]}, {msg.tool_pose[2]}, {msg.tool_pose[3]}, {msg.tool_pose[4]}, {msg.tool_pose[5]})")

def main(args=None):
    rclpy.init(args=args)

    node = ArmFeedbackSubscriberNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()