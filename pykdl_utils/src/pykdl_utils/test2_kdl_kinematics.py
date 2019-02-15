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

# for live joint positions
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import geometry_msgs.msg



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


    print "   getting current arm position..."
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

    print " starting calculations..."
    

    # joint angles:
    # 1 "knee_joint" 
    # 2 "waist_joint" 
    # 3 "right_arm_shoulder_rotate_joint" 
    # 4 "right_arm_shoulder_lift_joint" 
    # 5 "right_arm_elbow_rotate_joint" 
    # 6 "right_arm_elbow_bend_joint" 
    # 7 "right_arm_wrist_bend_joint" 
    # 8 "right_arm_wrist_rotate_joint" 

    print ">> Root link: %s,   End link: %s" % (base_link, end_link)
    kdl_kin = KDLKinematics(robot, base_link, end_link)

    ##DAVES - Set q to whatever joint angles you want
    joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    print "Joint Angles:", joint_angles
    pose = kdl_kin.forward(joint_angles)
    print "FK:"
    #print pose
    print "x = " + str(pose[0,3])
    print "y = " + str(pose[1,3])
    print "z = " + str(pose[2,3])
    print "===================================="


    q_new = kdl_kin.inverse(pose)
    print "IK (not necessarily the same):", q_new
    if q_new is not None:
        pose_new = kdl_kin.forward(q_new)
        print "FK on IK:", pose_new
        print "Error:", np.linalg.norm(pose_new * pose**-1 - np.mat(np.eye(4)))
    else:
        print "IK failure"


if __name__ == "__main__":
    do_right_arm()
