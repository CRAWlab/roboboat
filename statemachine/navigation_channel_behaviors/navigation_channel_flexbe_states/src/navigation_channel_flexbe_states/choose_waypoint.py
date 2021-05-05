#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

class ChooseWaypoint(EventState):
	'''
	Determine the waypoint to send to the nav stack to get through the buoys of the navigation channel

	<= found_waypoint             The waypoint has been calculated

	'''
    def __init__(self):
    	# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ChooseWaypoint, self).__init__(outcomes = ['found_waypoint'])

    def execute(self, userdata):
        Logger.loginfo('Executing state CHOOSE_WAYPOINT')
        return 'found_waypoint'