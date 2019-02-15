#!/usr/bin/env python
#
# Provides wrappers for PyKDL kinematics.
#
# Copyright (c) 2012, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Kelsey Hawkins

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



def main():
    import sys
    def usage():
        print("Tests for kdl_parser:\n")
        print("kdl_parser <urdf file>")
        print("\tLoad the URDF from file.")
        print("kdl_parser")
        print("\tLoad the URDF from the parameter server.")
        sys.exit(1)

    if len(sys.argv) > 2:
        usage()
    if len(sys.argv) == 2 and (sys.argv[1] == "-h" or sys.argv[1] == "--help"):
        usage()
    if (len(sys.argv) == 1):
        robot = Robot.from_parameter_server()
    else:
        f = file(sys.argv[1], 'r')
        robot = Robot.from_xml_string(f.read())
        f.close()

    if True:
        import random
        base_link = robot.get_root()
        #end_link = 'torso_link'#2
        #end_link = 'right_arm_torso_link'#2
        #end_link = 'right_arm_shoulder_rotate_servo_link' #2
        #q = np.array([0.0, 0.0])
        #q = np.array([0.0, 0.1])

        #end_link = 'right_arm_shoulder_rotate_link' #3
        #q = np.array([0.0, 0.0, 0.0])

        #end_link = 'right_arm_shoulder_lift_servo_link' #4
        #end_link = 'right_arm_upper_link'#4
        #end_link = 'right_arm_elbow_rotate_servo_link' #4
        #q = np.array([0.0, 0.0, 0.0, 0.0])

        #end_link = 'right_arm_elbow_bend_servo_link' #5
        #q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        #end_link = 'right_arm_elbow_bend_link' #6
        #end_link = 'right_arm_lower_link' #6
        #end_link = 'right_arm_wrist_bend_servo_link' #6
        #q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        #end_link = 'right_arm_wrist_rotate_servo_link' #7
        #q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #end_link = 'right_arm_wrist_rotate_link' #8
        #q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        #end_link = 'right_arm_gripper_body_link' #8
        #---------------------------------------
        end_link = 'right_arm_gripper_link' #8
        #---------------------------------------

        q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #q = np.array([-0.0095, 0.0095, -0.0460, -0.0383, 0.0261, 1.7196, -0.0184, 0.0322])
        #q = np.array([-0.0095, 0.0095, 0.1887, -0.1319, -1.846, 0.1795, 0.5031, -0.6550])

        #q = np.array([-0.849942445755, 0.849942445755, 1.2870, -0.1043, -0.0399, 0.5430, -0.8836, -0.0077])

        # joint angles:
        # 1 "knee_bend_joint" 
        # 2 "hip_bend_joint" 
        # 3 "right_arm_shoulder_rotate_joint" 
        # 4 "right_arm_shoulder_lift_joint" 
        # 5 "right_arm_elbow_rotate_joint" 
        # 6 "right_arm_elbow_bend_joint" 
        # 7 "right_arm_wrist_bend_joint" 
        # 8 "right_arm_wrist_rotate_joint" 

        #end_link = robot.link_map.keys()[random.randint(0, len(robot.link_map)-1)]
        print "Root link: %s,   End link: %s" % (base_link, end_link)
        kdl_kin = KDLKinematics(robot, base_link, end_link)
        #q = kdl_kin.random_joint_angles()
        ##DAVES - Set q to whatever joint angles you want

        print "Joint Angles:", q
        pose = kdl_kin.forward(q)
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
    main()
