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

#===============================================================================
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


#===============================================================================
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


#===============================================================================
def PprintAnglesRight(joint_angles):

    for idx, val in enumerate(joint_angles):
        print '% 1.6f'% (joint_angles[idx]), "  ", right_arm_kinematic_joints[idx]


#===============================================================================
def CalculateXYZ(kdl_kin, joint_angles):

    pose = kdl_kin.forward(joint_angles)
    return(pose[0,3], pose[1,3], pose[2,3])

#===============================================================================
def CalculateArmMoveToXYZ(kdl_kin, starting_joint_angles, target_x, target_y, target_z):

    # Constants
    TOLERANCE_X =               0.050      # mm
    TOLERANCE_Y =               0.050      
    TOLERANCE_Z =               0.050      
    MIN_Z =                     0.020 
    MAX_Z =                     0.100 
    ANGLE_STEP  =               np.radians(0.5)  # Degrees converted to radians
    LARGE_ANGLE_STEP =          np.radians(2.0)  

    # keep values in reasonable limits
    ANGLE_LIMIT_SHOULDER_MIN =      np.radians(-30.0)     # Degrees try angles = numpy.radians(angles)
    ANGLE_LIMIT_SHOULDER_MAX =      np.radians(30.0) 
    ANGLE_LIMIT_ELBOW_BEND_MIN =    np.radians(0.0)
    ANGLE_LIMIT_ELBOW_BEND_MAX =    np.radians(40.0)
    ANGLE_LIMIT_ELBOW_ROTATE_MIN =  np.radians(-40.0)
    ANGLE_LIMIT_ELBOW_ROTATE_MAX =  np.radians(40.0)

    # array positions
    # knee and hip joints are considered fixed (not changed in this function) 
    SHOULDER_ROTATE =  2
    SHOULDER_LIFT = 3
    ELBOW_ROTATE = 4
    ELBOW_BEND = 5
    WRIST_BEND = 6
    WRIST_ROTATE =  7
    

    # initialize
    print "CalculateArmMoveToXYZ:  target_xyz = ", target_x, target_y, target_z
    
    target_angles =  np.copy(starting_joint_angles)

    pose = kdl_kin.forward(target_angles)
    arm_x = pose[0,3] # set starting positions for imaginary arm
    arm_y = pose[1,3]
    arm_z = pose[2,3]
    
    print "================================"
    print "CalculateArmMoveToXYZ calculated starting xyz:"
    #print pose
    print "x = " + '% 1.6f'% (arm_x)
    print "y = " + '% 1.6f'% (arm_y)
    print "z = " + '% 1.6f'% (arm_z)
    print "================================"

    if target_z > 0.400: #mm
        print "WARNING - Target Z too high!  TODO!"
        # return starting joint positions, to avoid breaking arm
        # return (false, target_angles) 

    # Tune for overshoot or undershoot the object
    target_y -= 0.0; #mm


    # Begin Calculations
    
    # Find a good initial X position, with finger just above ground
    print "knee   hip   sh_rot    sh_lift   el_rot   el_bend   wr_bend   wr_rot: "
    print target_angles

    loop_count = 0
    while (arm_x > target_x) or (arm_z < MIN_Z) or (arm_z > MAX_Z):

        print "Loop ", loop_count

        # Arm too far forward, ahead of the object, or claw below the ground
        # move arm back until it meets requirements
        if arm_x > target_x:
            print "arm too far"
            target_angles[SHOULDER_ROTATE] -= LARGE_ANGLE_STEP
            if target_angles[SHOULDER_ROTATE] < ANGLE_LIMIT_SHOULDER_MIN: 
                target_angles[SHOULDER_ROTATE] = ANGLE_LIMIT_SHOULDER_MIN

        if arm_z < MIN_Z:
            print "arm too Low"            
            target_angles[ELBOW_BEND] += LARGE_ANGLE_STEP  # Keep arm above ground
            #if target_angles[ELBOW_BEND] > ANGLE_LIMIT_ELBOW_BEND_MAX: 
            #    target_angles[ELBOW_BEND] = ANGLE_LIMIT_ELBOW_BEND_MAX

        if arm_z > MAX_Z:
            print "arm too High"            
            target_angles[ELBOW_BEND] -= LARGE_ANGLE_STEP  # Keep arm close to ground
            if target_angles[ELBOW_BEND] < ANGLE_LIMIT_ELBOW_BEND_MIN: 
                target_angles[ELBOW_BEND] = ANGLE_LIMIT_ELBOW_BEND_MIN

        (arm_x, arm_y, arm_z) = CalculateXYZ(kdl_kin, target_angles)

        print target_angles
        
        loop_count += 1        
        if loop_count > 100:
            print 
            print "****** FIRST POSITION loop stuck, bailing out! ******"
            return (False, starting_joint_angles) 


    print
    print " ******* ARM IN FIRST POSITION after ", loop_count, " loops"
    print "ARM in FIRST Position:"
    PprintAnglesRight(target_angles)


    print
    print "find SECOND POSITION..."
    print "target_x = ", target_x, "target_y = ", target_y, "target_z = ", target_z

     # OK, arm in reasonable starting position. Find pick up position
    nLoopCount = 0
    #FPOINT3D_T LastArmXYZ;
    Lastarm_x = -1000.0;
    Lastarm_y = -1000.0;
    Lastarm_z = -1000.0;
    FoundX = FoundY = FoundZ = False
    for loop_count in range(0,100):
    
        # Iterate X
        FoundX = False;
        if arm_x < (target_x - TOLERANCE_X):
            target_angles[SHOULDER_ROTATE] += ANGLE_STEP
            if target_angles[SHOULDER_ROTATE] > ANGLE_LIMIT_SHOULDER_MAX: 
                target_angles[SHOULDER_ROTATE] = ANGLE_LIMIT_SHOULDER_MAX

        elif arm_x > (target_x + TOLERANCE_X):
            target_angles[SHOULDER_ROTATE] -= ANGLE_STEP # Overshot
            if target_angles[SHOULDER_ROTATE] < ANGLE_LIMIT_SHOULDER_MIN: 
                target_angles[SHOULDER_ROTATE] = ANGLE_LIMIT_SHOULDER_MIN
        else:
            FoundX = True;

    
         # Iterate Z
        FoundZ = False;
        if arm_z < (target_z - TOLERANCE_Z):
            target_angles[ELBOW_BEND] += ANGLE_STEP # UP
            if target_angles[ELBOW_BEND] > ANGLE_LIMIT_ELBOW_BEND_MAX: 
                target_angles[ELBOW_BEND] = ANGLE_LIMIT_ELBOW_BEND_MAX

        elif arm_z > (target_z + TOLERANCE_Z):
            target_angles[ELBOW_BEND] -= ANGLE_STEP # DOWN
            if target_angles[ELBOW_BEND] < ANGLE_LIMIT_ELBOW_BEND_MIN: 
                target_angles[ELBOW_BEND] = ANGLE_LIMIT_ELBOW_BEND_MIN
        else:
            FoundZ = True;


         # Iterate Y
        FoundY = False;
        if arm_y < (target_y - TOLERANCE_Y):
            target_angles[ELBOW_ROTATE] += ANGLE_STEP #
            if target_angles[ELBOW_ROTATE] > ANGLE_LIMIT_ELBOW_ROTATE_MAX: 
                target_angles[ELBOW_ROTATE] = ANGLE_LIMIT_ELBOW_ROTATE_MAX

        elif arm_y > (target_y + TOLERANCE_Y):
            target_angles[ELBOW_ROTATE] -= ANGLE_STEP #
            if target_angles[ELBOW_ROTATE] < ANGLE_LIMIT_ELBOW_ROTATE_MIN: 
                target_angles[ELBOW_ROTATE] = ANGLE_LIMIT_ELBOW_ROTATE_MIN
        else:
            FoundY = True;


        # Check progress
        if FoundX and FoundY and FoundZ:
            print
            print " ******* ARM IN SECOND POSITION after ", loop_count, " loops"
            return (True, target_angles) 

        (arm_x, arm_y, arm_z) = CalculateXYZ(kdl_kin, target_angles)
        print "FoundX = ", FoundX, "FoundY = ", FoundY, "FoundZ = ", FoundZ
        print "arm_x = ", arm_x, "arm_y = ", arm_y, "arm_z = ", arm_z
        print target_angles
 
        if (Lastarm_x == arm_x)  and (Lastarm_y == arm_y) and (Lastarm_z == arm_z):
            print "********* NO PROGRESS IN ITERATION LOOP, Bailing out! ************"
            return (False, starting_joint_angles) 

        Lastarm_x = arm_x;
        Lastarm_y = arm_y;
        Lastarm_z = arm_z;
 

    # dropped out of loop
    print 
    print "****** exceeded max loop count, NO SOLUTION FOUND ******"
    return (False, starting_joint_angles) 


#===============================================================================
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

    print "  Current actual gripper position: "
    physical_xyz = pose_from_base_msg.pose.position
    print physical_xyz

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
    #start_joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Get live joint angular values 
    (position, velocity, effort) = get_joint_states_from_service(right_arm_kinematic_joints)
    #print "raw position: ", position

    start_joint_angles = np.array(position)
    #print("Data type of the array x is:",start_joint_angles.dtype)
    #print "NP Joint Angles:", start_joint_angles
    
    pose = kdl_kin.forward(start_joint_angles)
    print "Calculated FK:"
    #print pose
    print "x = " + str(pose[0,3])
    print "y = " + str(pose[1,3])
    print "z = " + str(pose[2,3])

    print "================================"
    print "Calling CalculateArmMoveToXYZ() "
    print
    target_x = 0.40 #TODO set these to reasonable values!
    target_y = -0.252
    target_z = 0.030  
    (success, new_joint_angles) = CalculateArmMoveToXYZ(kdl_kin, start_joint_angles, target_x, target_y, target_z)
    print
    print "================================="
    print "RETURNED VALUES:"
    print "   Success:         ", success
    #print "   New Joint Angles:", new_joint_angles
    
    print "======== COMPARISON =============" 
    print " out:          in:          Difference:"
    for idx, val in enumerate(start_joint_angles):
        print '% 1.6f'% (new_joint_angles[idx]), " - ", '% 1.6f'% (start_joint_angles[idx]), \
            " = " '% 1.6f'% (new_joint_angles[idx] - start_joint_angles[idx]), \
            "  ", right_arm_kinematic_joints[idx]
            
    (final_x, final_y, final_z) = CalculateXYZ(kdl_kin, new_joint_angles)

    print '% 1.6f'% (final_x), " - ", '% 1.6f'% (target_x), " =", '% 1.3f'% (final_x - target_x), "      X mm"
    print '% 1.6f'% (final_y), " - ", '% 1.6f'% (target_y), " =", '% 1.3f'% (final_y - target_y), "      Y mm"
    print '% 1.6f'% (final_z), " - ", '% 1.6f'% (target_z), " =", '% 1.3f'% (final_z - target_z), "      Z mm"






            
    print "================================="  



#===============================================================================
if __name__ == "__main__":
    do_right_arm()
    
