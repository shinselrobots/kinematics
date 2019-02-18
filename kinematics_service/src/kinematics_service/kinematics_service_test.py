#!/usr/bin/env python
# test client for kinematics service

import roslib
import rospy
from kinematics_service.srv import ReturnIKSolution
import time
import sys
# -------------------------------------------------------------------------
#from servo_joint_list import *


def call_return_ik_solution(target_arm, target_x, target_y, target_z):
    rospy.wait_for_service("return_ik_solution")
    try:
        s = rospy.ServiceProxy("return_ik_solution", ReturnIKSolution)
        resp = s(target_arm, target_x, target_y, target_z)
        
    except rospy.ServiceException, e:
        print "error when calling return_ik_solution: %s"%e
        sys.exit(1)
        
    return (resp.success, resp.joints)


#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":

    while(1):
        print '------------------------------'
        
        #(success, joints) = call_return_ik_solution('right_arm', 0.40, -0.252, 0.030)
        (success, joints) = call_return_ik_solution('left_arm', 0.40, -0.252, 0.030)
        print "success = ", success, "joints = ", pplist(joints)

        time.sleep(1)
