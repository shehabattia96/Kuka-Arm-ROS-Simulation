from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
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
    theta_vals = [0, -pi/2,0,0,0,0,-theta[6]]
    theta_vals = [theta[i] + theta_vals[i] for i in range(0,num_links) ] #Add symbolic theta to each rotation
    #Combine DH-Parameters into one dictionary:
    dhParams = {}
    dhParams.update(dict(zip(a,a_vals)))
    dhParams.update(dict(zip(d,d_vals)))
    dhParams.update(dict(zip(alpha,alpha_vals)))
    dhParams.update(dict(zip(theta,theta_vals)))
    print(dhParams)

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

    O = [
        [0, 0, 0.33],
        [0.35, 0, 0.75],
        [0.35, 0, 2],
        [1.31, 0, 1.946],
        [1.85, 0, 1.946],
        [2.043, 0, 1.946],
        [2.153, 0, 1.946],
    ]

    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    # theta1 = 0
    # theta2 = 0
    # theta3 = 0
    # theta4 = 0
    # theta5 = 0
    # theta6 = 0

    # #Correct for the rotation error between URDF file and DH-Params:
    rot_error = rot_z.copy().subs(y, pi) * rot_y.copy().subs(p, -pi/2)

    rot_endEffector = rot_endEffector_sym.copy() * rot_error

    rot_endEffector = rot_endEffector.subs({'r':roll, 'p':pitch,'y':yaw})

    # #End effector position and wrist center:
    endEffector = Matrix([
        [px], [py], [pz]
    ])
    #Wrist center is the homogenous transform between base and end effector, end effector position - displacement * rotation of effector about z-axis.
    wristCenter = endEffector - 0.303 * rot_endEffector[:,2]
    print("endEffector", endEffector)
    print("wrist center", wristCenter)

    theta6 = atan2(wristCenter[2]-O[6][2],wristCenter[1]-O[6][1])
    O[5] = rot_x.copy().subs(r,theta6)*Matrix(O[5])
    theta5 = atan2(O[5][2]-O[4][2],O[5][0]-O[4][0])

    O[4] = rot_y.copy().subs(p,theta5)*Matrix(O[4])
    theta4 = atan2(O[4][2]-O[3][2],O[4][1]-O[3][1]) 
    
    O[3] = rot_x.copy().subs(r,theta4)*Matrix(O[3])
    theta3 = atan2(O[3][2]-O[2][2],O[3][0]-O[2][0])
    
    O[2] = rot_y.copy().subs(p,theta3)*Matrix(O[2])
    theta2 = atan2(O[2][2]-O[1][2],O[2][0]-O[1][0])
    
    O[1] = rot_y.copy().subs(p,theta2)*Matrix(O[1])
    theta1 = atan2(O[1][1]-O[0][1],O[1][0]-O[0][0])
    
    O[0] = rot_z.copy().subs(y,theta1)*Matrix(O[0])

    print("O: ", O) 
    #theta 1 is the joint angle about the z-axis from gripper to shares O4,5,6
    theta1 = atan2(wristCenter[1],wristCenter[0])

    # #From IK simplified drawing of links 0-3 and 3-6:
    # side_a = 1.5
    # side_b = sqrt(pow((sqrt(wristCenter[0]*wristCenter[0] + wristCenter[1]*wristCenter[1])-0.35),2)+pow((wristCenter[2]-0.75),2))
    # side_c = 1.25

    # angle_a = acos((side_b * side_b + side_c*side_c - side_a*side_a)/(2* side_b * side_c))
    # angle_b = acos((side_a * side_a + side_c*side_c - side_b*side_b)/(2* side_a * side_c))
    # angle_c = acos((side_b * side_b + side_a*side_a - side_c*side_c)/(2* side_b * side_a))
    # theta2 = pi/2 - angle_a - atan2(wristCenter[2]-0.75, sqrt(wristCenter[0]*wristCenter[0]+wristCenter[1]*wristCenter[1])-0.35)
    # theta3 = pi/2 - (angle_b +0.036)

    # R0_3 = transformMatricies[0][0:3,0:3]*transformMatricies[1][0:3,0:3]*transformMatricies[2][0:3,0:3]
    # R0_3 = R0_3.evalf(subs={theta[0]: theta1,theta[1]: theta2,theta[2]: theta3})
    # R3_6 = R0_3.inv("LU") * rot_endEffector
    # theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    # theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    # theta6 = atan2(-R3_6[1,1],R3_6[1,0])

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    theta_dict = dict(zip(theta,[theta1,theta2,theta3,theta4,theta5,theta6]))
    print(theta_dict)
    FK = transformation_between_baselink_and_endEffector.evalf(subs = theta_dict )
    your_wc = wristCenter
    your_ee = [FK[0,3],FK[1,3],FK[2,3]]
    print("FK")
    print(FK)
    ########################################################################################

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
