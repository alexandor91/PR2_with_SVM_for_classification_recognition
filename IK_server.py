#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        #Define DH param symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  #link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')   #link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        #twist angle
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  #joint angles    

        # Define Modified DH Transform matrix 
        DH_Table = {alpha0:     0, a0:      0, d1:   0.75, q1:         q1,
                    alpha1: -pi/2, a1:   0.35, d2:      0, q2: -pi/2 + q2,
                    alpha2:     0, a2:   1.25, d3:      0, q3:         q3,
                    alpha3: -pi/2, a3: -0.054, d4:    1.5, q4:         q4,
                    alpha4:  pi/2, a4:      0, d5:      0, q5:         q5,
                    alpha5: -pi/2, a5:      0, d6:      0, q6:         q6,
                    alpha6:     0, a6:      0, d7:  0.303, q7:          0}
        # Define modified DH transform matrix
        def DH_Transform(alpha, a, d, q):
             Transform = Matrix([[            cos(q),           -sin(q),           0,             a],
                                  [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                                  [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                                  [                 0,                 0,           0,             1]])
             return Transform
        #Define individual transform matrix
        TF0_1 = DH_Transform(alpha0, a0, d1, q1).subs(DH_Table)
        TF1_2 = DH_Transform(alpha1, a1, d2, q2).subs(DH_Table)
        TF2_3 = DH_Transform(alpha2, a2, d3, q3).subs(DH_Table)
        TF3_4 = DH_Transform(alpha3, a3, d4, q4).subs(DH_Table)
        TF4_5 = DH_Transform(alpha4, a4, d5, q5).subs(DH_Table)
        TF5_6 = DH_Transform(alpha5, a5, d6, q6).subs(DH_Table)
        TF6_7 = DH_Transform(alpha6, a6, d7, q7).subs(DH_Table)
        #Define Roll Pitch Yaw matrice
        r, p, y = symbols('r p y')
        Roll_x = Matrix([[1,      0,       0],
                         [0, cos(r),  -sin(r)],
                         [0, sin(r),   cos(r)]])  #Roll     
        Pitch_y = Matrix([[cos(p),  0, sin(p)],
                          [0,       1,      0],
                          [-sin(p), 0,  cos(p)]]) #Pitch    
        Yaw_z = Matrix([[cos(y), -sin(y),   0],
                        [sin(y),  cos(y),   0],
                        [0,            0,   1]]) #Yaw

        Rotation_corr = Yaw_z.subs(y, radians(180))*Pitch_y.subs(p, radians(-90))
        TF0_3 = TF0_1[0:3, 0:3] * TF1_2[0:3, 0:3] * TF2_3[0:3, 0:3]    

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            Rotation_7 = Yaw_z.subs({'y':yaw}) * Pitch_y.subs({'p':pitch}) * Roll_x.subs({'r':roll})
            Rotation_7 = Rotation_7 * Rotation_corr
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    # Calculate joint angles using Geometric IK method
            ###
            End = Matrix([[px],
                          [py],
                          [pz]])
            Wrist = End - (0.303)*Rotation_7[:,2]
        
            # Calculate theta1 theta2 theta3, the first three angles
            theta1 = atan2(Wrist[1], Wrist[0])
            side_a = 1.5
            side_c = 1.25
            side_b = sqrt(pow((sqrt(Wrist[0] * Wrist[0] + Wrist[1] * Wrist[1]) - 0.35), 2) + pow((Wrist[2] - 0.75), 2))

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b *side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a *side_c))
            angle_c = acos((side_b * side_b + side_a * side_a - side_c * side_c) / (2 * side_a *side_b))

            theta2 = pi/2 - angle_a - atan2(Wrist[2] - 0.75, sqrt(Wrist[0] * Wrist[0] + Wrist[1] * Wrist[1]) - 0.35)
            theta3 = pi/2 - (angle_b + 0.054/1.5)  #there is a minor angle offset to link4
            
            TF0_3 = TF0_1[0:3, 0:3].evalf(subs = {q1: theta1}) * TF1_2[0:3, 0:3].evalf(subs = {q2: theta2})  * TF2_3[0:3, 0:3].evalf(subs = {q3: theta3})     
            #TF0_3 = TF0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})



            TF3_6 = TF0_3.inv("LU") * Rotation_7
            #Extract the euler angles from the rotation matrix, take into account the constraint at the initial positons and end possitions
            if x<1 or x>(len(req.poses)-1):   #the gripper should be rotated to the desired angle in order to avoid collision.
               theta4 = atan2(TF3_6[2,2], -TF3_6[0,2])
               theta5 = atan2(sqrt(TF3_6[0,2] * TF3_6[0,2] + TF3_6[2,2] * TF3_6[2,2]), TF3_6[1,2])
               theta6 = atan2(-TF3_6[1,1], TF3_6[1,0])
            else:              # other positions along the long trajectory  should be the configured to zero to speed up the movement
               theta4 = 0
               theta5 = 0
               theta6 = 0

		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
