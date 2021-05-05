#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

class MoveToWaypoint(EventState):
    '''
    Send messages to the nav stack to move the roboboat towards the waypoint

    <= reached_waypoint1             Arrived at waypoint1
    <= reached_waypoint2             Arrived at waypoint2

    '''

    def __init__(self):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(MoveToWaypoint, self).__init__(outcomes = ['reached_waypoint1', 'reached_waypoint2'])
        self.counter = 0
        self.waypoint_counter = 0
    def execute(self, userdata):
        Logger.loginfo('Executing state MoveToWaypoint')
        if self.counter < 5:
        	self.counter += 1

        elif self.counter >= 5 and self.waypoint_counter < 1:
        	self.waypoint_counter += 1
        	self.counter = 0
        	return 'reached_waypoint1'
        else:
        	return 'reached_waypoint2'