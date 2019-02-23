from mpmath import *
from sympy import *

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
theta_vals = [theta[i] +theta_vals[i] for i in range(0,num_links) ] #Add symbolic theta to each rotation
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
transformation_between_joint0_and_endEffector = 1 #initialize this with 1
for x in transformMatricies: #Multiply all the matricies between joint 0 and end effector to get final transform matrix:
    transformation_between_joint0_and_endEffector = transformation_between_joint0_and_endEffector * x 
#print(transformation_between_joint0_and_endEffector)
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
print(rot_endEffector_sym)