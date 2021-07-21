#! /usr/bin/env python

###############################################################################
# roboboat_raw_throttle_test.py
#
# Script to test sending raw thruster commands via the mavros OverrideRCIn 
# message. Script implements the procedure suggested here:
#  https://discuss.bluerobotics.com/t/simulating-manual-control-using-mavros/1745/63
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

# import numpy as np

# ROS related imports
import rospy
from std_msgs.msg import String
from mavros.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool

class ThrottleTest():
    def __init__(self, desired_thurst_percentage=25, duration=3):
        '''
        Arguments:
          desired_thurst_percentage : desired thrust as percentage of max
          duration : the number of seconds to throttle up to that speed
        '''
          
        self.desired_thurst_percentage = desired_thurst_percentage
        self.duration = duration
        
        # Initialize the node
        rospy.init_node('roboboat_thrust_mapper', anonymous=True)
        
        self.state = State()  # We will keep track of the system state
        rospy.Subscriber('mavros/state', State, self.mavros_state_callback)
        
        # Set up the OverrideRCIn publisher
        self.thruster_pub = rospy.Publisher('/mavros/rc/override', 
                                            OverrideRCIn, 
                                            queue_size=10)
        
        # We'll publish commands at 20Hz
        self.rate = rospy.Rate(20.0)


    def mavros_state_callback(self, msg):
        self.state = msg


    def arm_the_system(self):
        rospy.wait_for_service('mavros/cmd/arming')
        
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)

        except rospy.ServiceException, e:
            rospy.logwarn("Service arming call failed: {}".format(e))


    def map_thrust_percentate_to_pwm(self, thrust_percentage):
        return 5.0 * thrust_percentage + 1500


    def run_throttle_test(self):
        ''' We'll spin up the thrusters for one second after the system is armed '''
        
        # First, make sure the drone is armed
        rospy.loginfo('Arming the System')
        
        while not self.state.armed:
            self.arm_the_system()
            self.rate.sleep()

        rospy.loginfo('Armed. Starting thrusters.')
        
        # First publish a neutral pwm command (should be 0 throtle)
        rc_in_override = OverrideRCIn()
        rc_in_override.channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0]
        
        self.thruster_pub.publish(rc_in_override)
        
        # Calculate the necessary pwm command for the desired thrust_percentage
        pwm = map_thrust_percentate_to_pwm(self.desired_thurst_percentage)
        rc_in_override.channels = [pwm, pwm, pwm, pwm, 0, 0, 0, 0]
        
        # Now, throttle up to the desired thrust percentage for 3s
        start_time = rospy.get_time()
        
        while rospy.get_time() - start_time < 3.0:
            self.thruster_pub.publish(rc_in_override)
            
            self.rate.sleep()
        
        rospy.loginfo('Stopping thrusters.')
        
        # Finally, stop the thrusters
        rc_in_override.channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0]
        
        self.thruster_pub.publish(rc_in_override)


if __name__ = '__main__':
    # TODO: 05/13/21 - JEV - Set up thrust percentage and duration as rosparameters
    test = ThrottleTest(desired_thurst_percentage=25, duration=3)
    
    test.run_throttle_test()