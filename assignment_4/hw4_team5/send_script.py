#!/usr/bin/env python
from rclpy.node import Node
import numpy as np
import rclpy
import time
import sys
import cv2
# sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

import threading

sys.path.append("/home/robotics/workspace2/team5_ws/src/send_script/send_script") # robot 1 (left)
import brick_cv

import cv2 as cv

r2d = 180 / np.pi

CUR_POSE = None

DIR = "/home/robotics/workspace2/team5_ws" # robot 1 (left)

class ArmFeedbackSubscriberNode(Node):

    def __init__(self):
        super().__init__('arm_feedback_sub')
        self.arm_feedback_sub = self.create_subscription(
            FeedbackState,
            'feedback_states',
            self.feedback_states_callback,
            10)

    def feedback_states_callback(self, msg):
        global cur_pose
        ##### Cartesian feedback
        cur_pose = msg.tool_pose

# arm client
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    print(f"script = {script}")

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    # arm_cli.call(move_cmd)

    print(f"time.time() a = {time.time()}")

    time.sleep(2)

    print(f"time.time() b = {time.time()}")

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        print(f"send_script after wait")
        arm_node.get_logger().info('service not availabe, waiting again...')

    arm_node.destroy_node()

# gripper client
def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        gripper_node.get_logger().info('service not availabe, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    # gripper_cli.call(io_cmd)

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        print(f"set_io after wait")
        gripper_node.get_logger().info('service not availabe, waiting again...')

    gripper_node.destroy_node()

def ToStr(l):
    return ", ".join(str(x) for x in l)

def Wait(timeout):
    # return lambda : cv2.waitKey(10000)
    return lambda : time.sleep(timeout)

def CaptureImage():
    def Func():
        send_script("Vision_DoJob(job1)")
        cv2.waitKey(1)

    return Func

def Move(pose):
    def Func():
        global CUR_POSE
        send_script("Line(\"CPP\","+ToStr(pose)+", 500, 100, 0, false)")
        time.sleep(3)
        CUR_POSE = pose

    return Func

def GripperOpen():
    def Func():
        set_io(0.0)
        time.sleep(1)

    return Func

def GripperClose():
    def Func():
        set_io(1.0)
        time.sleep(1)

    return Func

def LocObjs():
    def Func():
        CaptureImage()()

        frame = cv.imread(f"{DIR}/cap_img.png")
        img = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        return brick_cv.Inference(img, CUR_POSE)

    return Func

def main(args=None):

    rclpy.init(args=args)

    #--- move command by joint angle ---#
    # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#
    # targetP1 = "398.97, -122.27, 748.26, -179.62, 0.25, 90.12"s

    # Initial camera position for taking image (Please do not change the values)
    # For right arm: targetP1 = "230.00, 230, 730, 180.00, 0.0, 135.00"
    # For left  arm: targetP1 = "350.00, 350, 730, 180.00, 0.0, 135.00"
    # targetP1 = "230.00, 230, 730, 180.00, 0.0, 135.00"
    # targetP2 = "350.00, 350, 730, 180.00, 0.0, 135.00"
    # script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    # script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
    # send_script(script1)
    # send_script(script2)

    node = ArmFeedbackSubscriberNode()

    # t = threading.Thread(target=lambda : rclpy.spin(node))
    # t.start()

    above_z = 500

    red_pose = [348, 158, 110, 180, 0, 120]
    blue_rot = [3.13 * r2d, 0, 2.25 * r2d]
    blue_pose = [360, 319, 135] + blue_rot
    yellow_rot = [-3.09 * r2d, 0, 2.29 * r2d]
    yellow_pose = [171, 374, 110] + yellow_rot

    red_above = red_pose[:]
    red_above[2] = above_z

    blue_above = blue_pose[:]
    blue_above[2] = above_z

    yellow_above = yellow_pose[:]
    yellow_above[2] = above_z

    origin_pose = [250, 300, above_z, 180, 0, 135]

    extr_pose0 = [250, 300, above_z, 180, 0, 135]
    extr_pose1 = [250 + 50, 300, above_z, 180, 0, 135]
    extr_pose2 = [250 + 100, 300, above_z, 180, 0, 135]

    print(f"ToStr(origin_pose) = {ToStr(origin_pose)}")
    print(f"ToStr(blue_pose) = {ToStr(blue_pose)}")
    print(f"ToStr(yellow_pos) = {ToStr(yellow_pose)}")
    print(f"ToStr(red_pose) = {ToStr(red_pose)}")

    cap_mesh = list() # [N, 6]

    '''
    ToStr(origin_pose) = 348, 158, 400, 180, 0, 135
    ToStr(blue_pose) = 360, 319, 110, 179.33578987594765, 0, 128.91550390443524
    ToStr(yellow_pos) = 171, 374, 110, -177.04395869542438, 0, 131.20733508495852
    ToStr(red_pose) = 348, 158, 110, 180, 0, 120

    '''

    # -177.6169164905552, -1.8621128341751756, 112.87268564077218

    stride = 10

    x_start = 200
    y_start = 250

    for x in range(x_start, x_start + stride * 4, stride):
        for y in range(y_start, y_start + stride * 4, stride):
            for z in [above_z - stride, above_z]:
                cap_mesh.append([x, y, z, 180, 0, 135])

    '''
    motions = [
        GripperOpen(),
        Wait(0.5),
        Move(origin_pose),
        Wait(0.5),
        Move(yellow_above),
        Wait(0.5),
        Move(yellow_pose),
        GripperClose(),
        Wait(0.5),
        Move(yellow_above),
        Move(origin_pose),
    ]
    '''

    '''
    motions = [
        GripperOpen(),

        Move(red_above),
        Move(red_pose),

        GripperClose(),
        Move(origin_pose),
    ]'''

    recover_motions = [
        GripperOpen(),
        Move(origin_pose),
    ]

    extr_motions = [
        GripperOpen(),
        Move(extr_pose0),
        Move(extr_pose1),
        Move(extr_pose2),
    ]

    motions = [
        Move(red_above),
        Move(red_pose),

        GripperOpen(),
        GripperClose(),

        Move(red_above),
        Move(origin_pose),
    ]

    place_red_motions = [
        GripperClose(),
        Move(red_above),
        Move(red_pose),
        GripperOpen(),
        Move(red_above),
        Move(origin_pose)
    ]

    place_blue_motions = [
        GripperClose(),
        Move(blue_above),
        Move(blue_pose),
        GripperOpen(),
        Move(blue_above),
        Move(origin_pose)
    ]

    place_yellow_motions = [
        GripperClose(),
        Move(yellow_above),
        Move(yellow_pose),
        GripperOpen(),
        Move(yellow_above),
        Move(origin_pose)
    ]

    replace_yellow_motions = [
        GripperOpen(),
        Move(yellow_above),
        Move(yellow_pose),

        GripperClose(),

        Move(yellow_above),
        Move(origin_pose),

        Move(yellow_above),
        Move(yellow_pose),

        GripperOpen(),
    ]

    # for cap_idx in range(len(cap_mesh)):
    #     print(f"ToStr(cap_mesh[cap_idx]) = {ToStr(cap_mesh[cap_idx])}")

    cap_motions = list()

    for cap_idx in range(len(cap_mesh)):
        cap_motions.append(Move(cap_mesh[cap_idx]))
        cap_motions.append(CaptureImage())

    for motion in recover_motions:
        motion()

    # -----

    global CUR_POSE

    print(f"CUR_POSE = {CUR_POSE}")

    obj_poses = LocObjs()()

    print(f"obj_poses")
    print(obj_poses)




    # ########## Grasphing mode ##########
    # # 110 -> baseline, 50 -> air gap, 135 for blue, 110 for red and yellow

    # ##### grasp red
    # print(f"obj_poses")
    # print(obj_poses)

    # cur_brick_above = obj_poses[0][:]
    # cur_brick_above[2] = 110 + 50
    # Move(cur_brick_above)()

    # cur_brick_above[2] = 110 # 110 for red and yellow
    # Move(cur_brick_above)()

    # # print(f"CUR_POSE = {CUR_POSE}")
    # print(f"RED_POSE = {CUR_POSE}")

    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    # set_io(0.0)
    # Move(cur_brick_above)()
    # Move(origin_pose)()

    # ##### grasp yellow
    # print(f"obj_poses")
    # print(obj_poses)

    # cur_brick_above = obj_poses[1][:]
    # cur_brick_above[2] = 110 + 50
    # Move(cur_brick_above)()

    # cur_brick_above[2] = 110 # 110 for red and yellow
    # Move(cur_brick_above)()

    # print(f"YELLOW_POSE = {CUR_POSE}")

    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    # set_io(0.0)
    # Move(cur_brick_above)()
    # Move(origin_pose)()

    # ##### grasp blue
    # print(f"obj_poses")
    # print(obj_poses)

    # cur_brick_above = obj_poses[2][:]
    # cur_brick_above[2] = 110 + 50
    # Move(cur_brick_above)()

    # cur_brick_above[2] = 135 # 135 for blue
    # Move(cur_brick_above)()

    # print(f"CUR_POSE = {CUR_POSE}")

    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    # set_io(0.0)
    # Move(cur_brick_above)()
    # Move(origin_pose)()



    ########## Stacking mode (Not yet done) ##########
    # 110 -> baseline, 50 -> air gap, 135 for blue, 110 for red and yellow



    # ------

    if False:
        obj_poses = LocObjs()()

        print(f"CUR_POSE = {CUR_POSE}")

        cur_brick_above = obj_poses[0][:]
        cur_brick_above[2] = 110 + 50

        print(obj_poses)

        Move(cur_brick_above)()

        print(f"CUR_POSE = {CUR_POSE}")

    # ---



# What does Vision_DoJob do? Try to use it...
# -------------------------------------------------

#--------------------------------------------------


    rclpy.shutdown()

if __name__ == '__main__':
    main()




