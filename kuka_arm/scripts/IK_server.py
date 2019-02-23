#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Modified on Feb 22, 2019 by Shehab Attia
# 

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
        # Create symbols
        d = symbols('d1:8') #link offsets
        a = symbols('a0:7') #link lengths
        alpha = symbols('alpha0:7') #twist angles
        theta = symbols('theta1:8') #joint angles
        #Vals from kr210.urdf.xacro
        num_links = 7
        d_vals = [0.75, 0,0,1.5,0,0,0.303]
        a_vals = [0,0.35,1.25,-0.054,0,0,0]
        alpha_vals = [0, -pi/2,0,-pi/2,pi/2,-pi/2,0]
        theta_vals = [0, -pi/2,0,0,0,0,0]
        theta_vals = [theta[i] + theta_vals[i] for i in range(0,num_links) ] #Add symbolic theta to each rotation
        #Combine DH-Parameters into one dictionary:
        dhParams = {}
        dhParams.update(dict(zip(a,a_vals)))
        dhParams.update(dict(zip(d,d_vals)))
        dhParams.update(dict(zip(alpha,alpha_vals)))
        dhParams.update(dict(zip(theta,theta_vals)))

        # Define Modified DH Transformation matrix
        def TF_Matrix(alpha,a,d,theta): #references Kinematics walkthrough video
            transformMatrix = Matrix([ #Create a symbolic matrix to transform adjacent joints - taken from lesson.
                [cos(theta), -sin(theta), 0, a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                [0, 0, 0, 1]
            ])
            return transformMatrix
        # Create individual transformation matrices
        transformMatricies = [] #List of all num_links (7) transformation matricies between every joint and the next
        for syms in zip(alpha,a,d,theta): #zip the alpha, a, d, theta lists and use them to create transformation matricies
            transformMatricies.append(TF_Matrix(syms[0],syms[1],syms[2],syms[3]).subs(dhParams)) #Substitute values from dhParams dictionary into symbolic matrix
        transformation_between_baselink_and_endEffector = 1 #initialize this with 1
        for tfm in transformMatricies: #Multiply all the matricies between base link and end effector to get final transform matrix:
            transformation_between_baselink_and_endEffector = transformation_between_baselink_and_endEffector * tfm 

        # Extract rotation matrices from the transformation matrices

	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    r, p, y = symbols('r p y')

        #Textbook X,Y,Z 3D transformation matricies:

        rot_x = Matrix([
            [1, 0, 0],
            [0, cos(r), -sin(r)],
            [0, sin(r), cos(r)]
        ]) #Roll

        rot_y = Matrix([
            [cos(p), 0, sin(p)],
            [0, 1, 0],
            [-sin(p), 0, cos(p)]
        ]) #Pitch

        rot_z = Matrix([
            [cos(y), -sin(y), 0],
            [sin(y), cos(y), 0],
            [0, 0 ,1]
        ]) #Yaw

        rot_endEffector_sym = rot_x * rot_y * rot_z


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


            #Correct for the rotation error between URDF file and DH-Params:
            rot_error = rot_z.copy().subs(y, pi) * rot_y.copy().subs(p, -pi/2)

            rot_endEffector = rot_endEffector_sym.copy() * rot_error

            rot_endEffector = rot_endEffector.subs({'r':roll, 'p':pitch,'y':yaw})

            #End effector position and wrist center:
            endEffector = Matrix([
                [px], [py], [pz]
            ])
            #Wrist center is the homogenous transform between base and end effector, end effector position - displacement * rotation of effector about z-axis.
            wristCenter = endEffector - 0.303 * rot_endEffector[:,2]

            #theta 1 is the joint angle about the z-axis from gripper to shares O4,5,6
            theta1 = atan2(wristCenter[1],wristCenter[0])

            #From IK simplified drawing of links 0-3 and 3-6:
            side_a = 1.5
            side_b = sqrt(pow((sqrt(wristCenter[0]*wristCenter[0] + wristCenter[1]*wristCenter[1])-0.35),2)+pow((wristCenter[2]-0.75),2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c*side_c - side_a*side_a)/(2* side_b * side_c))
            angle_b = acos((side_a * side_a + side_c*side_c - side_b*side_b)/(2* side_a * side_c))
            angle_c = acos((side_b * side_b + side_a*side_a - side_c*side_c)/(2* side_b * side_a))
            theta2 = pi/2 - angle_a - atan2(wristCenter[2]-0.75, sqrt(wristCenter[0]*wristCenter[0]+wristCenter[1]*wristCenter[1])-0.35)
            theta3 = pi/2 - (angle_b +0.036)

            
            R0_3 = transformMatricies[0][0:3,0:3]*transformMatricies[1][0:3,0:3]*transformMatricies[2][0:3,0:3]
            R0_3 = R0_3.evalf(subs={theta[0]: theta1,theta[1]: theta2,theta[2]: theta3})
            R3_6 = R0_3.inv("LU") * rot_endEffector
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])
            theta_dict = dict(zip(theta,[theta1,theta2,theta3,theta4,theta5,theta6]))
            FK = transformation_between_baselink_and_endEffector.evalf(subs = theta_dict )
            your_wc = wristCenter
            your_ee = [FK[0,3],FK[1,3],FK[2,3]]

        



            #
            # Calculate joint angles using Geometric IK method
            #
            #
            ###

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
