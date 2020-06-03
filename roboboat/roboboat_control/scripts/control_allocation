#! /usr/bin/env python

###############################################################################
# RoboBoat_thrust.py
#
# Mapping ROS cmd_vel messages to thruster inputs for the RoboBoat 2019 
# X-configuration of BlueRobotics T-200 thrusters.
#
# The script is set up to control the ESC that is connected to a 
# Adafruit 16-channel PWM hat on a Raspberry Pi. This requires installation of 
# the libraries to communicate with that board.
#
# You should hear three beeps once the ESC is powered on. Then, two more once
# this script initializes it. Then, this script will cycle through forward and 
# reverse throttle.
#
# Relevant Links:
#   T200 - https://www.bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/
#   Basic ESC - https://www.bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/
#   Adafruit 16-channel PWM Hat - https://www.adafruit.com/product/2327
#   Adafruit software HowTo - https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/
#
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 05/02/19
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#    * 04/20/19 - JEV - Fix throttle offset issue
#    * 04/27/19 - JEV - Check thruster positioning
#    * 05/02/19 - JEV - clean up thrust and velocities to match the actual limits
#    * 05/02/19 - JEV - Vectorize thruster definitions
###############################################################################

import numpy as np
import time

# from adafruit_servokit import ServoKit

# ROS related imports
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


HEARTBEAT_MAX_MISSED = 5 # maximum number of allowable timesteps to miss before stopping

# TODO: 04/27/19 - JEV - Check these values
WIDTH = 11 * 2.54/100         # Approx width to thruster (m)
LENGTH = 13 * 2.54/100        # Approx length to thruster (m)

ESC_STARTUP_DELAY = 1.0         # Time to sleep during initialization

# TODO: 04/20/19 - JEV 
#       * 0.4 throttle is being interpreted as zero. 
#       * Likely due to:
#           - PWM timing in Adafruit library or
#           - range of pulse width set
THROTTLE_OFFSET = 0.4           # Hack for now


# NOTE: This is a *very* rough approximation based on the forward and reverse
#       thrust numbers reported in the BlueRobotics T-200 datasheet. 
#
# TODO: 05/02/19 - JEV - clean up this to match the actual limits
MAX_FORWARD_PER_THRUSTER = 100    # defined as 100 (max) 
MAX_REVERSE_PER_THRUSTER = -80    # Reverse thrust is ~80% of forward

# Define a thrust ratio to use in scaling for cases needing reverse thrust
thrust_ratio = np.abs(MAX_REVERSE_PER_THRUSTER / MAX_FORWARD_PER_THRUSTER)

# The 16-channel PWM hat has (surprise!!) 16 channels
# kit = ServoKit(channels=16)


class RoboBoat_ThrustMapper():
    """
    This class contains all the necessary code to map ROS cmb_vel messages
    to the thrust/throttle commands for the X-config.
    """
    
    
    def __init__(self, thruster_angle=np.deg2rad(45), max_throttle=100):
        """
        Arguments:
            thruster_angle : the angle of the thrusters away from surge (rad)
                             Defaults to 45 deg
            max_throttle : maximum throttle command to issue
                           assumed symmetric about zero
                           Defaults to 100
        """
        
        self.thruster_angle = thruster_angle
        self.max_throttle = max_throttle
        
        # TODO: 05/02/19 - JEV - We should probably vectorize these definiations
        #                        to make accessing thruster info and assigning 
        #                        speeds for them more efficient.
        #
        # The ESC is connected to channel 0. We can map a name to that channel to make
        # the code easier to understand
#         self.port_stern = kit.continuous_servo[0]
#         self.port_bow = kit.continuous_servo[4]
#         self.stdb_stern = kit.continuous_servo[8]
#         self.stbd_bow = kit.continuous_servo[12]
# 
#         # The BlueRobotics ESC expects pulses in the range 1100-1900us, so we modify
#         # the range of pulses for the pin connected to the ESC
#         self.port_stern.set_pulse_width_range(1100, 1900)
#         self.port_bow.set_pulse_width_range(1100, 1900)
#         self.stbd_stern.set_pulse_width_range(1100, 1900)
#         self.stbd_bow.set_pulse_width_range(1100, 1900)
#         
#         # Finally, initialize the Thrusters
#         # To initialize the thruster, we need to send zero speed for some time
#         # You should hear two beeps once the ESC is initialized
#         rospy.loginfo('Initializing...')
#         self.port_stern.throttle = 0.0 + THROTTLE_OFFSET
#         self.port_bow.throttle = 0.0 + THROTTLE_OFFSET
#         self.stbd_stern.throttle = 0.0 + THROTTLE_OFFSET
#         self.stbd_bow.throttle = 0.0 + THROTTLE_OFFSET
#         time.sleep(ESC_STARTUP_DELAY)
        
        # Define the matrix needed for the mapping. 
        # Defining it here instead of the mapping function, means it should 
        # only get defined once.
        self.A = np.array([[np.cos(thruster_angle), np.cos(thruster_angle), np.cos(thruster_angle), np.cos(thruster_angle)],
                           [-np.sin(thruster_angle), np.sin(thruster_angle), np.sin(thruster_angle), -np.sin(thruster_angle)],
                           [LENGTH * np.sin(thruster_angle) + WIDTH * np.cos(thruster_angle), LENGTH * np.sin(thruster_angle) + WIDTH * np.cos(thruster_angle), 
                            -LENGTH * np.sin(thruster_angle) - WIDTH * np.cos(thruster_angle), -LENGTH * np.sin(thruster_angle) - WIDTH * np.cos(thruster_angle)]])

        # Calculate the maximum posdible net forces/torques in each direction
        # We'll use these to scale the velocity according to the range we predict 
        # is its min/max
        self.MAX_SURGE_FORCE = np.dot(self.A[0,:], thrust_ratio * MAX_FORWARD_PER_THRUSTER * np.ones(4))
        self.MAX_SWAY_FORCE = np.dot(self.A[1,:], np.array([MAX_REVERSE_PER_THRUSTER, 
                                                       thrust_ratio * MAX_FORWARD_PER_THRUSTER, 
                                                       thrust_ratio * MAX_FORWARD_PER_THRUSTER, 
                                                       MAX_REVERSE_PER_THRUSTER]) * np.sin(thruster_angle))
        self.MAX_YAW_TORQUE = (WIDTH * 2 * thrust_ratio * MAX_FORWARD_PER_THRUSTER) - (WIDTH * 2 * MAX_REVERSE_PER_THRUSTER)
        
        # Define the maximum velocities in each direction
        # TODO: 05/02/19 - JEV - Assumed to be symmetric. Are they?
        self.MAX_SURGE_VEL = 6.643  # From calculations in final presentation (m/s)
        self.MAX_SWAY_VEL = 3.0     # 05/02/19 - JEV - Total guess (m/s)
        self.MAX_YAW_VEL = 0.5      # 05/02/19 - JEV - Total guess (rad/s)
        
        # Now, Set up the ROS node and subscribe to the cmd_vel topic
        # We'll also set up publishing to Chatter for monitoring
        
        # Initialize the node
        rospy.init_node('thrust_mapper', anonymous=True)
        
        # Create the publisher
        self.thrust_mapper_chatter = rospy.Publisher('chatter', String, queue_size=10)
        
        # Set up the cmd_vel subscriber and register the callback
        rospy.Subscriber("/cmd_vel", Twist, self.calculate_and_assign_thrust) 
        
        self.rate = rospy.Rate(10) # We'll run the heartbeat count at 10Hz
        
        # We also set up a heartbeat counter so that we shut off the thrusters
        # if we don't receive a cmd_vel command after some time
        self.heartbeat_counter = 0


    def calculate_and_assign_thrust(self, twist):
        """
        This is called every time a TWist message is received. It should run
        quickly.
        """
        
        # First, reset the heartbeat_counter to zero since we received a 
        # cmd_vel message
        self.heartbeat_counter = 0
        
        # Then, grab the parts of the Twist message we care about
        self.surge_vel_command = twist.linear.x
        self.sway_vel_command = twist.linear.y
        self.yaw_vel_command = twist.angular.z
        
        # Now, represent those desired velocities as a percentage of the maximum
        # in each direction
        desired_input_percentage = 100* np.array([self.surge_vel_command / self.MAX_SURGE_VEL,
                                                  self.sway_vel_command / self.MAX_SWAY_VEL,
                                                  self.yaw_vel_command / self.MAX_YAW_VEL])

        # Then, map that to a percentage of the maximum force/torque in the 
        # corresponding axis.
        #
        # Note; This is a very rough way to do this. We'd do better to 
        # incorporate some knowledge of the boat's dynamics
        desired_net_inputs = desired_input_percentage / 100 * np.array([self.MAX_SURGE_FORCE,
                                                                        self.MAX_SWAY_FORCE,
                                                                        self.MAX_YAW_TORQUE])

        # Finally, solve for the thruster inputs to generate those
        thrusts, residuals, rank, s = np.linalg.lstsq(self.A, desired_net_inputs)

        rospy.loginfo('Raw thruster solution = {} &'.format(thrusts))

        # TODO: 04/27/19 - JEV - Be more elegant here. We can do this without scaling
        #                        twice, as often happens here.
        #
        # If max needed thrust for any thruster is greater than capable, scale all
        # of the trusters equally
        # 
        # Here (as of 05/02/19), we're doing this by percentage. We probably 
        # should change
        if np.max(thrusts) > 100: 
            rospy.loginfo('Scaling due to forward thrust')
            thrusts = thrusts / (np.max(thrusts) / 100)
            rospy.loginfo('Scaled solution = {} %'.format(thrusts))

        # If max reverse thrust needed for any thruster is greater than capable, scale
        # all of the trusters equally
        if np.min(thrusts) < -100:
            rospy.loginfo('Scaling due to reverse thrust')
            thrusts = thrusts / (np.min(thrusts) / -100)
            rospy.loginfo('Scaled solution = {} %'.format(thrusts))
        
        # TODO: 05/02/19 - JEV - Do we need to clip throttle commands to +/-1 to account for the OFFSET
        rospy.loginfo('Throttle: {}'.format(thrusts / 100.0 + THROTTLE_OFFSET))
        # Finally, we can send the throttle commands to the thrusters
#         self.port_stern.throttle = thrusts[0] / 100.0 + THROTTLE_OFFSET
#         self.port_bow.throttle = thrusts[1] / 100.0 + THROTTLE_OFFSET
#         self.stbd_stern.throttle = thrusts[2] / 100.0 + THROTTLE_OFFSET
#         self.stbd_bow.throttle = thrusts[3] / 100.0 + THROTTLE_OFFSET

        
    def wait_for_cmd_vel(self):
        """
        This is what will get called if this script is run directly, rather
        than as an import. It just loops, keeping track of the heartbeat.
        
        The cmd_vel messages are processed by the callback.
        """
        
        try: 
            while not rospy.is_shutdown():
                # Increment the heartbeak counter by 1
                self.heartbeat_counter = self.heartbeat_counter + 1
        
                if self.heartbeat_counter >= HEARTBEAT_MAX_MISSED:
                    # Log the error the first time
                    if self.heartbeat_counter == HEARTBEAT_MAX_MISSED:
                        rospy.logerr("Heartbeat not reset for {} steps. Stopping.".format(HEARTBEAT_MAX_MISSED))
        
#                 self.port_stern.throttle = 0.0 + THROTTLE_OFFSET
#                 self.port_bow.throttle = 0.0 + THROTTLE_OFFSET
#                 self.stbd_stern.throttle = 0.0 + THROTTLE_OFFSET
#                 self.stbd_bow.throttle = 0.0 + THROTTLE_OFFSET

                self.rate.sleep()

        except (KeyboardInterrupt, SystemExit):
#             self.port_stern.throttle = 0.0 + THROTTLE_OFFSET
#             self.port_bow.throttle = 0.0 + THROTTLE_OFFSET
#             self.stbd_stern.throttle = 0.0 + THROTTLE_OFFSET
#             self.stbd_bow.throttle = 0.0 + THROTTLE_OFFSET
            raise

if __name__ == "__main__":
    thrust_mapper = RoboBoat_ThrustMapper()
    
    try:
        thrust_mapper.wait_for_cmd_vel()
    
    except (KeyboardInterrupt, SystemExit):
        rospy.logerr("Thrustmapper node quit.")
        raise