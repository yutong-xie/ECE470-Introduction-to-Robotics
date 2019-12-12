#!/usr/bin/env python
import numpy as np
import math
from scipy.linalg import expm
# PI = 3.1415926535
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def skew(vector):
    return np.array([[0.0, -vector[2], vector[1]],[vector[2], 0.0, -vector[0]],[-vector[1], vector[0], 0.0]])

def SBracket(vector,theta):
	new_vector = np.ones((6,1))
	w = np.ones((3,1))
	v = np.ones((3,1))

	for i in range(3):
		w[i,0] = vector[i]
		v[i,0] = vector[3 + i]

	w_bracket = skew(w)
	S_theta = np.hstack([w_bracket*theta, v*theta])
	S_theta = np.vstack([S_theta,np.array([0.0, 0.0, 0.0, 0.0])])
	return S_theta

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	pos_1 = np.array([[0.0],[0.0],[0.152]])
	pos_2 = np.array([[0.0],[0.12],[0.152]])
	pos_3 = np.array([[0.244],[0.12],[0.152]])
	pos_4 = np.array([[0.457],[0.027],[0.152]])
	pos_5 = np.array([[0.457],[0.11],[0.152]])
	pos_6 = np.array([[0.54],[0.11],[0.152]])

	pos_W = np.array([[0.0],[0.0],[0.0]])
	pos_E = np.array([[0.54],[0.243],[0.212]])

	P = pos_E - pos_W
	R = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
	M = np.hstack([R, P])
	M = np.vstack([M,np.array([0.0,0.0,0.0,1.0])])

	joint_positions = []
	joint_positions.append(pos_1)
	joint_positions.append(pos_2)
	joint_positions.append(pos_3)
	joint_positions.append(pos_4)
	joint_positions.append(pos_5)
	joint_positions.append(pos_6)

	q = []
	for i in range(0,len(joint_positions)):
		q.append(joint_positions[i]-pos_W)

	#joint rotation axes for the UR3
	w1 = np.array([[0.0],[0.0],[1.0]])
	w2 = np.array([[0.0],[1.0],[0.0]])
	w3 = np.array([[0.0],[1.0],[0.0]])
	w4 = np.array([[0.0],[1.0],[0.0]])
	w5 = np.array([[1.0],[0.0],[0.0]])
	w6 = np.array([[0.0],[1.0],[0.0]])

	w = []
	w.append(w1)
	w.append(w2)
	w.append(w3)
	w.append(w4)
	w.append(w5)
	w.append(w6)

	v = []
	for i in range(0,len(joint_positions)):
		v.append(-np.cross(w[i],q[i],axis=0))


	# screw axes for each joint on the UR3
	SA = []
	for i in range(0,len(joint_positions)):
		# w_bracket = skew(w[i])
		Si = np.vstack([w[i], v[i]])
		# Si = np.vstack([Si, np.array([0.0, 0.0, 0.0, 0.0])])
		SA.append(Si)

	S = np.hstack([SA[0],SA[1],SA[2],SA[3],SA[4],SA[5]])

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	e =[]

	for i in range(6):
		exps = expm(SBracket(S[:,i], theta[i]))
		e.append(exps)

	T = M
	for i in range(5, -1, -1):
		T = e[i].dot(T)


	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.083
	l07 = 0.083
	l08 = 0.082
	l09 = 0.0535
	l10 = 0.059   # thickness of aluminum plate is around 0.006

	yaw = yaw_WgripDegree * PI / 180

	xgrip = xWgrip + 0.15
	ygrip = yWgrip - 0.15
	zgrip = zWgrip - 0.008

	xcen = xgrip - l09*math.cos(yaw)
	ycen = ygrip - l09*math.sin(yaw)
	zcen = zgrip

	# theta1
	thetal = np.arctan2(ycen,xcen)
	l = np.sqrt(xcen**2+ycen**2)
	theta111 = np.arcsin((l02-l04+l06)/l)
	thetas[0] = thetal - theta111          # Default value Need to Change

	# theta6
	thetas[5] = PI/2 - yaw + thetas[0]     # Default value Need to Change

	l67 = np.sqrt(l07**2+(l06+0.027)**2)
	theta67 = np.arctan2(l07,(l06+0.027))

	x3end = xcen - l67*np.sin(theta67-thetas[0])
	y3end = ycen - l67*np.cos(theta67-thetas[0])
	z3end = zcen + l10 + l08

	xy3end = np.sqrt(x3end**2 + y3end**2)
	z = z3end - l01
	xyz3end = np.sqrt(xy3end**2 + z**2)

	theta21 = np.arctan2(z,xy3end)
	print("theta21 is: " + str(theta21))
	theta41 = PI/2 - theta21
	print("theta41 is: " + str(theta41))

	theta22 = np.arccos((l03**2 + xyz3end**2 - l05**2) / (2*l03*xyz3end))
	xxxx = (l03**2 + xyz3end**2 - l05**2) / (2*l03*xyz3end)
	print(xxxx)
	theta42 = np.arccos((l05**2 + xyz3end**2 - l03**2) / (2*l05*xyz3end))
	yyyy = (l05**2 + xyz3end**2 - l03**2) / (2*l05*xyz3end)
	print(yyyy)

	thetas[1]= - (theta21 + theta22)        # Default value Need to Change
	thetas[2]= theta22 + theta42            # Default value Need to Change
	thetas[3]= - (theta41 + theta42 - PI/2) # Default value Need to Change, need + (0.5*PI) for compensation

	thetas[4]=-PI/2                         # Default value Need to Change

	print("theta1 to theta6: " + str(thetas) + "\n")

	return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
		          float(thetas[3]), float(thetas[4]), float(thetas[5]) )
