#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger

class CircumNavPillBuoy(EventState):
    '''
    Look for opening into obstacle field by circumnavigating

    <= found_opening          the opening has been located

    '''

    def __init__(self):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(CircumNavPillBuoy, self).__init__(outcomes = ['circled_buoy'])


    def execute(self, userdata):
        # Logger.loginfo('Executing state SEARCH_BUOYS')
        return 'circled_buoy'
        

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # The following code is just for illustrating how the behavior logger works.
        # Text logged by the behavior logger is sent to the operator and displayed in the GUI.

        # # time_to_wait = (self._target_time - (rospy.Time.now() - self._start_time)).to_sec()

        # if time_to_wait > 0:
        #     Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)
        pass

    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        pass # Nothing to do in this example.


    def on_start(self):
        # This method is called when the behavior is started.
        # If possible, it is generally better to initialize used resources in the constructor
        # because if anything failed, the behavior would not even be started.

        # In this example, we use this event to set the correct start time.
        # self._start_time = rospy.Time.now()
        pass


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        pass # Nothing to do in this example.
        


