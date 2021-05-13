#! /usr/bin/env python

###############################################################################
# roboboat_thrust_mapping.py
#
# Package that provides basic thrust mapping for the RoboBoat. 
# It maps cmd_vel messages to raw thruster commands.
#
# Created: 05/13/21
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - @doc_vaughan
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#   * 
###############################################################################

import numpy as np

# ROS related imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mavros.msg import OverrideRCIn, State

class ThrustMapper():
    '''
    Class to contain all functionality and data for thrust mapping
    '''
    
    def __init__(self, thruster_angle=np.deg2rad(45), max_throttle=100):
        """
        Arguments:
            thruster_angle : the angle of the thrusters away from surge (rad)
                             Defaults to 45 deg
            max_throttle : maximum throttle command to issue
                           assumed symmetric about zero
                           Defaults to 100%
        """
        
        self.thruster_angle = thruster_angle
        self.max_throttle = max_throttle

        # Initialize the node
        rospy.init_node('roboboat_thrust_mapper', anonymous=True)
        
        # TODO: 05/13/21 - JEV - Change to rosparmss, read from config file
        self.HEARTBEAT_MAX_MISSED = 5 # maximum number of allowable timesteps to miss before stopping

        # TODO: 05/13/21 - JEV - Change these to rosparmss, read from config file
        self.WIDTH = 11 * 2.54/100         # Approx width to thruster (m)
        self.LENGTH = 13 * 2.54/100        # Approx length to thruster (m)

        self.state = State()  # We will keep track of the system state
        rospy.Subscriber('mavros/state', State, self.mavros_state_callback)
    
        # Define the matrix needed for the mapping. 
        # Defining it here instead of the mapping function, means it should 
        # only get defined once.
        self.A = np.array([[np.cos(self.thruster_angle), np.cos(self.thruster_angle), np.cos(self.thruster_angle), np.cos(self.thruster_angle)],
                           [-np.sin(self.thruster_angle), np.sin(self.thruster_angle), np.sin(self.thruster_angle), -np.sin(self.thruster_angle)],
                           [self.LENGTH * np.sin(self.thruster_angle) + self.WIDTH * np.cos(self.thruster_angle), self.LENGTH * np.sin(self.thruster_angle) + self.WIDTH * np.cos(self.thruster_angle), 
                            -self.LENGTH * np.sin(self.thruster_angle) - self.WIDTH * np.cos(self.thruster_angle), -self.LENGTH * np.sin(self.thruster_angle) - self.WIDTH * np.cos(self.thruster_angle)]])

        # Calculate the maximum posdible net forces/torques in each direction
        # We'll use these to scale the velocity according to the range we predict 
        # is its min/max
        self.max_surge_force = np.dot(self.A[0,:], thrust_ratio * MAX_FORWARD_PER_THRUSTER * np.ones(4))
        self.max_sway_force = np.dot(self.A[1,:], np.array([MAX_REVERSE_PER_THRUSTER, 
                                                       thrust_ratio * MAX_FORWARD_PER_THRUSTER, 
                                                       thrust_ratio * MAX_FORWARD_PER_THRUSTER, 
                                                       MAX_REVERSE_PER_THRUSTER]) * np.sin(thruster_angle))
        self.max_yaw_torque = (WIDTH * 2 * thrust_ratio * MAX_FORWARD_PER_THRUSTER) - (WIDTH * 2 * MAX_REVERSE_PER_THRUSTER)
        
        # Define the maximum velocities in each direction
        # TODO: 05/02/19 - JEV - Assumed to be symmetric. Are they?
        self.MAX_SURGE_VEL = 6.643  # From calculations in final presentation (m/s)
        self.MAX_SWAY_VEL = 3.0     # 05/02/19 - JEV - Total guess (m/s)
        self.MAX_YAW_VEL = 0.5      # 05/02/19 - JEV - Total guess (rad/s)
        
        # Now, Set up the ROS node and subscribe to the cmd_vel topic
        
        # Set up the cmd_vel subscriber and register the callback
        rospy.Subscriber("/cmd_vel", Twist, self.calculate_and_assign_thrust) 
        
        # Set up the OverrideRCIn publisher
        self.thruster_pub = rospy.Publisher('/mavros/rc/override', 
                                            OverrideRCIn, 
                                            queue_size=10)
        
        self.rate = rospy.Rate(10) # We'll run the heartbeat count at 10Hz
        
        # We also set up a heartbeat counter so that we shut off the thrusters
        # if we don't receive a cmd_vel command after some time
        self.heartbeat_counter = 0
    
    def mavros_state_callback(self, msg):
        self.state = msg
      
    def map_thrust_percentate_to_pwm(self, thrust_percentage):
        return 5.0 * thrust_percentage + 1500
        
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
        # in each direction.
        # TODO: 05/13/21 - JEV 
        #   WE assume a linear mapping here. I'm pretty sure that it is not linear.
        desired_input_percentage = 100 * np.array([self.surge_vel_command / self.MAX_SURGE_VEL,
                                                   self.sway_vel_command / self.MAX_SWAY_VEL,
                                                   self.yaw_vel_command / self.MAX_YAW_VEL])

        # Then, map that to a percentage of the maximum force/torque in the 
        # corresponding axis.
        #
        # Note; This is a very rough way to do this. We'd do better to 
        # incorporate some knowledge of the boat's dynamics
        #
        # TODO: 05/13/21 - JEV 
        #   WE assume a linear mapping here. I'm pretty sure that it is not linear.
        desired_net_inputs = desired_input_percentage / 100 * np.array([self.max_surge_force,
                                                                        self.max_sway_force,
                                                                        self.max_yaw_torque])

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
        rospy.loginfo('Throttle Percentages: {}'.format(thrusts / 100.0))
        
        # Calculate the necessary pwm command for the desired thrust_percentage
        pwm = map_thrust_percentate_to_pwm(self.desired_thurst_percentage)
        
        # Create the message
        rc_in_override = OverrideRCIn()
        rc_in_override.channels = [pwm, pwm, pwm, pwm, 0, 0, 0, 0]
        
        # And publish it
        self.thruster_pub.publish(rc_in_override)


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
        
                if self.heartbeat_counter >= self.HEARTBEAT_MAX_MISSED:
                    # Log the error the first time
                    if self.heartbeat_counter == self.HEARTBEAT_MAX_MISSED:
                        rospy.logerr("Heartbeat not reset for {} steps. Stopping.".format(self.HEARTBEAT_MAX_MISSED))

                self.rate.sleep()

        except (KeyboardInterrupt, SystemExit):
            raise


if __name__ == "__main__":
    thrust_mapper = ThrustMapper()
    thrust_mapper.wait_for_cmd_vel()
    