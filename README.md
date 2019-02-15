# hrl-kdl
Kinematics and geometry utilities, cloned from: https://github.com/gt-ros-pkg/hrl-kdl

Modified for Sheldon robot use.

For more info, see:  http://wiki.ros.org/hrl_kdl  (NOT hrl-kdl)

NOTE:  
This will be used for calculating inverse kinematics (using successive approximation)
It supplies the XYZ gripper positon for *imaginary* servo positions.

TF provides the ACTUAL XYZ of the REAL servos.  
See sheldon_servos --> get_gripper_xyz.py for example of using TF to get actual gripper XYZ.



