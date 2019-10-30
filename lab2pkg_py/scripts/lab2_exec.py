#!/usr/bin/env python

import copy
import time
import rospy
import numpy as np
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)


############## Your Code Start Here ##############

# Hanoi tower location 1

Q11 = np.radians([138.26, -63.64, 111.73, -141.15, -88.27, 41.54])
Q12 = np.radians([138.25, -57.19, 109.93, -142.03, -88.22, 41.54])
Q13 = np.radians([138.38, -50.63, 110.53, -149.17, -88.46,41.54])
Q1High = np.radians([139.04, -71.2, 105.8, -126.54, -90.63, 41.98])

# Hanoi tower location 2

Q21 = np.radians([153.91, -69.41, 120.22, -142.22, -87.18, 68.3])
Q22 = np.radians([153.14, -61.57, 123.88, -155.92, -87.55, 62.27])
Q23 = np.radians([153.2, -53.84, 122.72, -160.81, -87.57, 62.27])
Q2High = np.radians([152.74, -79.17, 116.32, -130.13, -89.79, 51.71])

# Hanoi tower location 3

Q31 = np.radians([171.51, -69.17, 119.02, -140.89, -90.66, 80.11])
Q32 = np.radians([171.62, -62.25, 119.02, -145.9, -90.66, 80.11])
Q33 = np.radians([171.58, -52.71, 124.22, -166.43, -90.92, 80.11])
Q3High = np.radians([166.71,-80.37,122.44,-138.77,-86.37,57.68])

Q = [ [Q11, Q12, Q13, Q1High], \
      [Q21, Q22, Q23, Q2High], \
      [Q31, Q32, Q33, Q3High] ]


gripped = False

startPoint = 0
middlePosition = 1
endPoint = 2


############### Your Code End Here ###############


thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False


############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_input_callback(data):
	global gripped

	gripped = data.DIGIN


############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1

		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	if rospy.is_shutdown():
		rospy.signal_shutdown("SIGINT Called")
		exit()

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")

		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate):

	global home
	global Q
	global SPIN_RATE
	global startPoint
	global endPoint

	error = 0

	# Move to Start Point High

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move Down to Start Point Poistion 1 and turn on gripper move back up

	move_arm(pub_cmd, loop_rate, Q[startPoint][0], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_on)

	if not check_if_gripped():
		print_error(pub_cmd, loop_rate)

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move to Destination Point High

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move down to Desitination 3 and turn off gripper and move back up

	move_arm(pub_cmd, loop_rate, Q[endPoint][2], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_off)

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move to StartPoint High

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Pick up Block 2

	move_arm(pub_cmd, loop_rate, Q[startPoint][1], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_on)

	if not check_if_gripped():
		print_error(pub_cmd, loop_rate)

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move to middle position High

	move_arm(pub_cmd, loop_rate, Q[middlePosition][3], 4.0, 4.0)

	# Drop off block to middle 3 and come back up

	move_arm(pub_cmd, loop_rate, Q[middlePosition][2], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_off)

	move_arm(pub_cmd, loop_rate, Q[middlePosition][3], 4.0, 4.0)

	# Move to Endpoint High

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move Down to Endpoint 3 pick up and come back up

	move_arm(pub_cmd, loop_rate, Q[endPoint][2], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_on)

	if not check_if_gripped():
		print_error(pub_cmd, loop_rate)

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move to middlePoint High

	move_arm(pub_cmd, loop_rate, Q[middlePosition][3], 4.0, 4.0)

	# Move Down to Middle point 2 drop off and come back

	move_arm(pub_cmd, loop_rate, Q[middlePosition][1], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_off)

	move_arm(pub_cmd, loop_rate, Q[middlePosition][3], 4.0, 4.0)

	# Move to StartPoint High

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move Down to Startpoint 3 pick up and come back

	move_arm(pub_cmd, loop_rate, Q[startPoint][2], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_on)

	if not check_if_gripped():
		print_error(pub_cmd, loop_rate)

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move to End Point High

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move down to Endpoint 3 and turn off gripper and move back up

	move_arm(pub_cmd, loop_rate, Q[endPoint][2], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_off)

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move to middle position High

	move_arm(pub_cmd, loop_rate, Q[middlePosition][3], 4.0, 4.0)

	# Move Down to Middle point 2 pick up and come back

	move_arm(pub_cmd, loop_rate, Q[middlePosition][1], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_on)

	if not check_if_gripped():
		print_error(pub_cmd, loop_rate)

	move_arm(pub_cmd, loop_rate, Q[middlePosition][3], 4.0, 4.0)

	# Move to StartPoint High

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move Down to Startpoint 3 drop off and come back

	move_arm(pub_cmd, loop_rate, Q[startPoint][2], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_off)

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move to middle position High

	move_arm(pub_cmd, loop_rate, Q[middlePosition][3], 4.0, 4.0)

	# Move Down to middle 3 pick up and come back up

	move_arm(pub_cmd, loop_rate, Q[middlePosition][2], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_on)

	if not check_if_gripped():
		print_error(pub_cmd, loop_rate)

	move_arm(pub_cmd, loop_rate, Q[middlePosition][3], 4.0, 4.0)

	# Move to End Point High

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move down to Endpoint 2 and turn off gripper and move back up

	move_arm(pub_cmd, loop_rate, Q[endPoint][1], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_off)

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move to StartPoint High

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move Down to Startpoint 3 pick up and come back

	move_arm(pub_cmd, loop_rate, Q[startPoint][2], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_on)

	if not check_if_gripped():
		print_error(pub_cmd, loop_rate)

	move_arm(pub_cmd, loop_rate, Q[startPoint][3], 4.0, 4.0)

	# Move to End Point High

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# Move down to Endpoint 1 and turn off gripper and move back up

	move_arm(pub_cmd, loop_rate, Q[endPoint][0], 4.0, 4.0)

	gripper(pub_cmd, loop_rate, suction_off)

	move_arm(pub_cmd, loop_rate, Q[endPoint][3], 4.0, 4.0)

	# DONE

	return error

def check_if_gripped():
	rospy.sleep(0.2)
	global gripped

	if not gripped:
		return False
	else:
		return True

def print_error(pub_cmd, loop_rate):
	global home
	gripper(pub_cmd, loop_rate, suction_off)
	move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
	rospy.signal_shutdown("Gripper is not holding any block")
	rospy.loginfo("Gripper is not holding any block, Exiting Program")
	exit()


############### Your Code End Here ###############


def main():

	global home
	global Q
	global SPIN_RATE

	# Initialize ROS node
	rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)

	############## Your Code Start Here ##############
	# TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

	sub = rospy.Subscriber("/ur3/gripper_input", gripper_input, gripper_input_callback)




	############### Your Code End Here ###############


	############## Your Code Start Here ##############
	# TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0
    input_position = 0

    global startPoint
    global middlePosition
    global endPoint

    while(True):
    	if loop_count == 2:
    		break
    	input_done = 0

    	input_string = raw_input("Enter number of loops <Either 1 2 or 3> ")
    	print("You entered " + input_string + "\n")

    	if(int(input_string) == 1):
    		input_position = 1
    		input_done = 1
    	elif (int(input_string) == 2):
    		input_done = 1
    		input_position = 2
    	elif (int(input_string) == 3):
    		input_done = 1
    		input_position = 3
    	else:
    		print("Please just enter the character 1 2 or 3 \n\n")

    	if input_done:
    		if loop_count == 0:
    			startPoint = input_position - 1
    		elif loop_count == 1:
    			endPoint = input_position - 1
    		else:
    			break

    		loop_count += 1

    middlePosition = 3 - startPoint - endPoint

	############### Your Code End Here ###############

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	rospy.loginfo("Sending Goals ...")

	loop_rate = rospy.Rate(SPIN_RATE)

	############## Your Code Start Here ##############
	# TODO: modify the code so that UR3 can move tower accordingly from user input

		# MOVE 1

	move_arm(pub_command, loop_rate, home, 4.0, 4.0)

	move_block(pub_command, loop_rate)

	move_arm(pub_command, loop_rate, home, 4.0, 4.0)


	############### Your Code End Here ###############



if __name__ == '__main__':

	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
