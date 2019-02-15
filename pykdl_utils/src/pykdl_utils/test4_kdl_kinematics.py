#!/usr/bin/env python
#
# tests for PyKDL kinematics.
#


import numpy as np
import sys
import PyKDL as kdl
import rospy
from sensor_msgs.msg import JointState
import hrl_geom.transformations as trans
from hrl_geom.pose_converter import PoseConv
from kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import Robot
from kdl_kinematics import KDLKinematics

# for live joint xyz positions
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import geometry_msgs.msg

# for live joint angular positions
from sheldon_servos.srv import ReturnJointStates
#TODO from servo_joint_list import *

leg_joints = [
    'knee_bend_joint',
    'hip_bend_joint',
    ]

right_arm_joints = [
    'right_arm_shoulder_rotate_joint',
    'right_arm_shoulder_lift_joint',
    'right_arm_elbow_rotate_joint',
    'right_arm_elbow_bend_joint',
    'right_arm_wrist_bend_joint',
    'right_arm_wrist_rotate_joint',
    ]

left_arm_joints = [
    'left_arm_shoulder_rotate_joint',
    'left_arm_shoulder_lift_joint',
    'left_arm_elbow_rotate_joint',
    'left_arm_elbow_bend_joint',
    'left_arm_wrist_bend_joint',
    'left_arm_wrist_rotate_joint',
    ]


right_arm_kinematic_joints =  leg_joints + right_arm_joints
left_arm_kinematic_joints =  leg_joints + left_arm_joints

def get_joint_states_from_service(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return (resp.position, resp.velocity, resp.effort)


def do_right_arm():

    print "   do_right_arm starting..."
    rospy.init_node('arm_kinematics')
    tf_listener = TransformListener()

    print "   getting robot URDF from Parameter Server..."
    robot = Robot.from_parameter_server()

    print "   getting base_link..."
    base_link = robot.get_root()
    
    end_link = 'right_arm_gripper_link' # 8 joints
    print "   set end_link to " + end_link


    print "   getting current arm XYZ position from TF..."
    # Uses TF to directly get the pose of right_arm_gripper_link

    print "   waiting for transform"
    tf_listener.waitForTransform (end_link, base_link, rospy.Time(), rospy.Duration(4.0))
    print "   done waiting"

    if not tf_listener.frameExists("base_link"):
        print "ERROR NO FRAME base_link"
        return
    
    if not tf_listener.frameExists(end_link):
        print "ERROR NO FRAME" +  end_link
        return
    
    t = tf_listener.getLatestCommonTime("/base_link", end_link)
    joint_pose = geometry_msgs.msg.PoseStamped()
    joint_pose.header.frame_id = end_link
    joint_pose.pose.orientation.w = 1.0    # Neutral orientation
    pose_from_base_msg = tf_listener.transformPose("/base_link", joint_pose)
    
    #print "Position from " + base_link + " to " + end_link
    #print pose_from_base_msg

    print "================================"
    print "Current actual gripper position: "
    physical_xyz = pose_from_base_msg.pose.position
    print physical_xyz
    print "================================"

    # print " starting calculations..."
 
    # joint angles:
    # 1 "knee_bend_joint" 
    # 2 "hip_bend_joint" 
    # 3 "right_arm_shoulder_rotate_joint" 
    # 4 "right_arm_shoulder_lift_joint" 
    # 5 "right_arm_elbow_rotate_joint" 
    # 6 "right_arm_elbow_bend_joint" 
    # 7 "right_arm_wrist_bend_joint" 
    # 8 "right_arm_wrist_rotate_joint" 

    # print ">> Root link: %s,   End link: %s" % (base_link, end_link)
    kdl_kin = KDLKinematics(robot, base_link, end_link)

    ##DAVES - Set q to whatever joint angles you want
    #joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Get live joint angular values 
    (position, velocity, effort) = get_joint_states_from_service(right_arm_kinematic_joints)
    #print "raw position: ", position

    joint_angles = np.array(position)
    #print("Data type of the array x is:",joint_angles.dtype)
    #print "NP Joint Angles:", joint_angles
    
    pose = kdl_kin.forward(joint_angles)
    print "================================"
    print "Calculated FK:"
    #print pose
    print "x = " + str(pose[0,3])
    print "y = " + str(pose[1,3])
    print "z = " + str(pose[2,3])
    print "================================"







if __name__ == "__main__":
    do_right_arm()
