#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.
# Author: Harsh Pandya


####################################################
# Name: Shehab Attia, Udacity RoboND 
# Modified on Feb 22, 2019 by Shehab Attia
# The script calculates joint angles given poses from a pre-determined path plan for the Kuka KR210 robot arm.

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from time import time

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Begin pose estimation by calculating transformation matricies from DH-Params
        # Create symbols
        d = symbols('d1:8') #link offsets
        a = symbols('a0:7') #link lengths
        alpha = symbols('alpha0:7') #twist angles
        theta = symbols('theta1:8') #joint angles
        num_links = 7 #includes EE
        #Vals from kr210.urdf.xacro
        d_vals = [0.75, 0,0,1.5,0,0,0.303] #link offsets
        a_vals = [0,0.35,1.25,-0.054,0,0,0] #link lengths
        alpha_vals = [0, -pi/2.0,0,-pi/2.0,pi/2.0,-pi/2.0,0] #twist angles
        theta_vals = [0, -pi/2.0,0,0,0,0,-theta[6]] #joint angles (usually q, I'm calling it theta)
        theta_vals = [theta[i] + theta_vals[i] for i in range(0,num_links) ] #Add symbolic theta to each rotation
        #Combine DH-Parameters into one dictionary:
        dhParams = {}
        dhParams.update(dict(zip(a,a_vals)))
        dhParams.update(dict(zip(d,d_vals)))
        dhParams.update(dict(zip(alpha,alpha_vals)))
        dhParams.update(dict(zip(theta,theta_vals)))
        
        # Define Modified DH Transformation matrix
        def TF_Matrix(alpha_tfm,a_tfm,d_tfm,theta_tfm): #references Kinematics walkthrough video
            transformMatrix = Matrix([ #Create a symbolic matrix to transform adjacent joints - taken from lesson.
                [cos(theta_tfm), -sin(theta_tfm), 0, a_tfm],
                [sin(theta_tfm)*cos(alpha_tfm), cos(theta_tfm)*cos(alpha_tfm), -sin(alpha_tfm), -sin(alpha_tfm)*d_tfm],
                [sin(theta_tfm)*sin(alpha_tfm), cos(theta_tfm)*sin(alpha_tfm), cos(alpha_tfm), cos(alpha_tfm)*d_tfm],
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
        print("Finished calculating DH-Params and transformations")

        ### Compensate for rotation discrepancy between DH parameters and Gazebo
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
        #Create a symbolic XYZ rotational transformation matrix:
        rot_endEffector_sym = rot_x * rot_y * rot_z

        #Log the poses calculated by MoveIt:
        with open('./poses_log.txt','a') as f:
            print >> f,str(time())
            for pose in req.poses:
                print >> f, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

        ### Iterate through all the given poses:
        joint_trajectory_list = [] # Initialize service response
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            ### Begin pose estimation 
            px = req.poses[x].position.x #EE x
            py = req.poses[x].position.y #EE y
            pz = req.poses[x].position.z #EE z
            #Orientation (r,p,y):
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            start_time = time()
            print("Received pose:", px, py, pz)
            #Correct for the rotation error between URDF file and DH-Params:
            rot_error = rot_z.copy().subs(y, radians(90)) * rot_y.copy().subs(p, radians(-90)) * rot_x.copy().subs(r, radians(90)) 
            rot_endEffector = rot_endEffector_sym.copy() * rot_error
            rot_endEffector = rot_endEffector.subs({'r':roll, 'p':pitch,'y':yaw})

            #End effector position and wrist center:
            endEffector = Matrix([
                [px], [py], [pz]
            ])
            #Wrist center is the homogenous transform between base and end effector, end effector position - displacement * rotation of effector about z-axis.
            # endEffector =  rot_endEffector * endEffector
            print("Calculated wrist center")
            posedEE_on_wristCenter = (0.303 * rot_endEffector) #Calculate the rotation of ee on wrist center 
            wristCenter = endEffector #initialize wristCenter before translating it down the arm
            #Translate wrist center after rotation of EE:
            wristCenter[0] = wristCenter[0] - posedEE_on_wristCenter[0,2]
            wristCenter[1] = wristCenter[1] - posedEE_on_wristCenter[0,1]
            wristCenter[2] = wristCenter[2] + posedEE_on_wristCenter[0,0]
            
            #theta 1 is the joint angle about the z-axis from base link to WC:
            theta1 = atan2(wristCenter[1],wristCenter[0])
            print("Theta 1 calculated") 
            #From IK simplified drawing of links 0-3 and 3-6. References sample code
            side_a = 1.5
            side_b = sqrt(pow((sqrt(wristCenter[0]*wristCenter[0] + wristCenter[1]*wristCenter[1])-0.35),2)+pow((wristCenter[2]-0.75),2))
            side_c = 1.25
            angle_a = acos((side_b * side_b + side_c*side_c - side_a*side_a)/(2* side_b * side_c))
            angle_b = acos((side_a * side_a + side_c*side_c - side_b*side_b)/(2* side_a * side_c))
            angle_c = acos((side_b * side_b + side_a*side_a - side_c*side_c)/(2* side_b * side_a))
            theta2 = pi/2 - angle_a - atan2(wristCenter[2]-0.75, sqrt(wristCenter[0]*wristCenter[0]+wristCenter[1]*wristCenter[1])-0.35)
            print("Theta 2 calculated")
            theta3 = pi/2 - (angle_b +0.036)
            print("Theta 3 calculated")
            #Construct Forward Kinematics transformation for the first three joints:
            R0_3 = transformMatricies[0]*transformMatricies[1]*transformMatricies[2]
            R0_3 = R0_3.evalf(subs={theta[0]: theta1,theta[1]: theta2,theta[2]: theta3})
            #Calculate transformation from Joint 3 to WC:
            R3_6 = R0_3.inv("LU")[0:3,0:3] * rot_endEffector
            #Calculate the orientations of the last three joints:
            #Sometimes we need to add or subtract pi, but we'll not worry about that right now
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            print("Theta 4 calculated")
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            print("Theta 5 calculated")
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])
            print("Theta 6 calculated")

            #Display time
            print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))
                
            #Append theta's to joint trajectory:
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
