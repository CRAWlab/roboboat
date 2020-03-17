#! /usr/bin/env python

#####################################################################################
#
# ctrl_allocation.py
#
# This script is to take in the computed "raw" control in the form of:
#
# [[Tx]
#  [Ty]
#  [Mz]
#
# in a 3 DOF (surge, sway, yaw) reference frame for an ASV and output how these
# can be resolved into the actuator configuration of the RoboBoat.
#
# The RoboBoat is equipped with an "X" configuration, capable of holonomic motion. The
# actuators do not azimuth. Because the hull is imprecise, bias factors are included to
# manually tune where motors may not be properly aligned, creating additional, unintentional
# moments.
#
# Created: 03/17/2020
#   - Benjamin Armentor
#   - benjamin.armentor1@louisiana.edu
#
# Modified:
#   * 
#
# TODO:
# 1) Add custom messages used here to the make files.
# 2) Possibly adjust the publish rate
# 3) Get physical measurements from RoboBoat
# 4) Determine what types of values should be published -- RPMs directly to Pi or in-between step?
#####################################################################################

import rospy
import numpy as np
from roboboat_msgs.msg import motor_msg, ctrl_msg

def shutdown_hook():

	# Cease all commands to motors
	global motors
	motors = motor_msg()

	motors.Tpb = 0.0
	motors.Tps = 0.0
	motors.Tsb = 0.0
	motors.Tss = 0.0

	motor_pub.publish(motors)

# Callback function for the raw control message
def raw_ctrl_callback(msg):

	global raw_ctrl_msg
	raw_ctrl_msg = msg

# Initialize the ROS node
rospy.init_node('CtrlAllocation')

# Define the publish rate
rate = rospy.Rate(10) # (Hz)

# Set up the publishers and subscribers
raw_ctrl_sub = rospy.Subscriber('/ctrl_msg/raw', ctrl_msg, raw_ctrl_callback)
motor_pub = rospy.Publisher('/ctrl_msg/assigned', motor_msg, queue_size=1)

# Define message containers
raw_ctrl_msg = ctrl_msg()
motors = motor_msg()

# Initialize the sequence counter
seq = 0

# TODO: Add physical RoboBoat measurements here

# Give ROS instructions when this node is shut down
rospy.on_shutdown(shutdown_hook)

start_time = rospy.get_rostime()

# Main loop
while not rospy.is_shutdown():

	# Extract the current values from the raw_ctrl_msg
	Tx = raw_ctrl_msg.Tx
	Ty = raw_ctrl_msg.Ty
	Mz = raw_ctrl_msg.Mz

	# Initial plan for how to solve
	B_mat = np.array([[Tx],
					  [Ty],
					  [Mz]])

	# TODO: Construct an A matrix based on the physical measurements
	
	# I think we need a higher-level system understanding
	# Should this be an RPM command? Or should we manually fit the RPMs from the graphs
	# available at https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/

	# Assign the message header
	motors.header.seq = seq
	motors.header.stamp = rospy.get_rostime() - start_time

	# Assign the motor control values
	motors.Tpb = 
	motors.Tps = 
	motors.Tsb = 
	motors.Tss = 

	# Publish the resolved motor message
	motor_pub.publish(motors)

	# Update the sequence counter
	seq += 1

	# Avoid CPU saturation
	rate.sleep()