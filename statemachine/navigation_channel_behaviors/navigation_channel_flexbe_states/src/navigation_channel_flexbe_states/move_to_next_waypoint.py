#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

class MoveToNextWaypoint(EventState):
    '''
    Send messages to the nav stack to move the roboboat towards the waypoint

    <= reached_waypoint            Arrived at waypoint

    '''

    def __init__(self):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(MoveToNextWaypoint, self).__init__(outcomes = ['reached_waypoint'])
        self.counter = 0

    def execute(self, userdata):
        Logger.loginfo('Executing state MoveToNextWaypoint')
        if self.counter < 5:
        	self.counter += 1

        else:
        	self.counter = 0
        	return 'reached_waypoint'