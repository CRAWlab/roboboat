#!/usr/bin/env python
# StateMachine for the ASV and UAV for teh CRAWLABs ASV and UAV. 

import rospy
import smach
import smach_ros

from smach_ros import SimpleActionState
from smach import Concurrence

## State finding navigation channel  ###############################################
class find_NavChan_start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['found', 'not_found'])
        self.counter = 0

    def execute(self, userdata): #This is where all the code for finding the navigation channel goes looking for GPS coordinate and trying to find the two larger buoys
            rospy.loginfo('Executing state find Navigation Channel')
            if self.counter < 3:
                    self.counter += 1
                    rospy.loginfo('Navigation Channel not found')
                    return 'not_found'
            else:
                    return 'found'
####################################################################################


## State that could not find the navigation channel still looking ##################
class cannot_find_NavChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_find_NavChan'])

    def execute(self, userdata):
            rospy.loginfo('Cannot find Navigation Channel')
            return 'back_to_find_NavChan'
####################################################################################


# State for finding the middle of the navigation channel ###########################
class find_middle_NavChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_middle_NavChan', 'not_found'])
        self.counter = 0 

    def execute(self, userdata):
        if self.counter < 3:
                self.counter += 1
                rospy.loginfo('Did not find middle still computing')
                return 'not_found'
        else:
                return 'found_middle_NavChan'
####################################################################################

## State that could not find the middle of the NavCahn still looking ############### 
class cannot_find_middle_NavChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_find_middle'])

    def execute(self, userdata):
            rospy.loginfo('Cannot find the middle of Navigation Channel')
            return 'back_to_find_middle'
####################################################################################


# ## State to stay straight through the navigation channel #########################
# class passthrough(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes= ['passing'])

#     def execute(self, userdata): #All code for passing through goes here
#             rospy.loginfo('Passing through')
#             return 'passing'
# ##################################################################################

## State for checking distance to  buoy  ###########################################
class NavChan_distance_checking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['go_starboard', 'go_port', 'go_straight'])
        self.counter = 0
    def execute(self, userdata): #All code for checking distance 
        if self.counter < 1:
            self.counter += 1
            rospy.loginfo('Boat too close to buoy on port side. Going toward starboard')
            return 'go_starboard'
        elif self.counter  == 2:
            self.counter += 1 
            rospy.loginfo('Boat too close to buoy on starboard side. Going toward port')
            return 'go_port'
        else:
            return 'go_straight'
####################################################################################

## State to return to checking distance ############################################
class back_to_distance_checking_NavChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_distance_checking_NavChan'])

    def execute(self, userdata): #All code for returnign to distance checking
        rospy.loginfo('Rechecking Distance')
        return 'back_to_distance_checking_NavChan'
####################################################################################


## State to stay straight through the navigation channel ###########################
class passing_through_NavChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['passing_through_NavChan'])

    def execute(self, userdata): #All code for passing through goes here
            rospy.loginfo('Passing through')
            return 'passing_through_NavhChan'
####################################################################################

#################################    Obstacle channel    ###########################
####################################################################################
####################################################################################

## State to find the Obstacle Channel 
class find_ObChan_start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['found', 'not_found'])
        self.counter = 0

    def execute(self, userdata): #This is where all the code for finding the obstacle channel goes looking for GPS coordinate and trying to find the two larger buoys
            rospy.loginfo('Executing state find Obstacle Channel')
            if self.counter < 3:
                    self.counter += 1
                    rospy.loginfo('Obstacle Channel not found')
                    return 'not_found'
            else:
                    return 'found'
####################################################################################


## State that could not find the Obstacle channel still looking ####################
class cannot_find_ObChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_find_ObChan'])

    def execute(self, userdata):
            rospy.loginfo('Cannot find Obstacle Channel')
            return 'back_to_find_ObChan'
####################################################################################


# State for finding the middle of the Obstacle channel ###########################
class find_middle_ObChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_middle_ObChan', 'not_found'])
        self.counter = 0 

    def execute(self, userdata):
        if self.counter < 3:
                self.counter += 1
                rospy.loginfo('Did not find middle still computing')
                return 'not_found'
        else:
                return 'found_middle_ObChan'
####################################################################################

## State that could not find the middle of the Obstacle still looking ###############
class cannot_find_middle_ObChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_find_middle_ObChan'])

    def execute(self, userdata):
            rospy.loginfo('Cannot find the middle of Obstacle Channel')
            return 'back_to_find_middle_ObChan'
####################################################################################

## State for checking distance to  buoy  ###########################################
class ObChan_distance_checking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['go_starboard', 'go_port', 'go_straight'])
        self.counter = 0
    def execute(self, userdata): #All code for checking distance 
        if self.counter < 1:
            self.counter += 1
            rospy.loginfo('Boat too close to buoy on port side. Going toward starboard')
            return 'go_starboard'
        elif self.counter  == 2:
            self.counter += 1 
            rospy.loginfo('Boat too close to buoy on starboard side. Going toward port')
            return 'go_port'
        else:
            return 'go_straight'
####################################################################################

## State to return to checking distance ############################################
class back_to_distance_checking_ObChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_distance_checking_ObChan'])

    def execute(self, userdata): #All code for returnign to distance checking
        rospy.loginfo('Rechecking Distance')
        return 'back_to_distance_checking_ObChan'
####################################################################################


## State to stay straight through the Obstacle channel ###########################
class passing_through_ObChan_buoy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['passing_through_ObChan_buoy'])

    def execute(self, userdata): #All code for passing through goes here
            rospy.loginfo('Passing through')
            return 'passing_through_ObChan_buoy'
####################################################################################


def main():
    rospy.init_node('roboboatStateMach')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm:
        # Navigation Containers
        smach.StateMachine.add('find_NavChan', find_NavChan_start(), transitions= {'not_found':'cannot_find_NavChan', 'found':'find_middle_NavChan'})

        smach.StateMachine.add('cannot_find_NavChan', cannot_find_NavChan(), transitions={'back_to_find_NavChan':'find_NavChan'})

        smach.StateMachine.add('find_middle_NavChan', find_middle_NavChan(), transitions= {'not_found':'cannot_find_middle_NavChan', 'found_middle_NavChan':'NavChan_distance_checking'})

        smach.StateMachine.add('cannot_find_middle_NavChan', cannot_find_middle_NavChan(), transitions= {'back_to_find_middle':'find_middle_NavChan'})

        smach.StateMachine.add('NavChan_distance_checking', NavChan_distance_checking(), transitions= { 'go_starboard':'back_to_distance_checking_NavChan', 'go_port':'back_to_distance_checking_NavChan', 'go_straight':'passing_through_NavChan'})

        smach.StateMachine.add('back_to_distance_checking_NavChan', back_to_distance_checking_NavChan(), transitions= {'back_to_distance_checking_NavChan':'NavChan_distance_checking'})

        smach.StateMachine.add('passing_through_NavChan', passing_through_NavChan(), {'passing_through_NavChan':'find_ObChan_start'})

        ####################### Starting the Obstacle channel Containers ########################################
        ########################################################################################################

        smach.StateMachine.add('find_ObChan_start', find_ObChan_start(), transitions= {'not_found':'cannot_find_ObChan', 'found':'find_middle_ObChan'})

        smach.StateMachine.add('cannot_find_ObChan', cannot_find_ObChan(), transitions={'back_to_find_ObChan':'find_ObChan_start'})

        smach.StateMachine.add('find_middle_ObChan', find_middle_ObChan(), transitions= {'not_found':'cannot_find_middle_ObChan', 'found_middle_ObChan':'ObChan_distance_checking'})

        smach.StateMachine.add('cannot_find_middle_ObChan', cannot_find_middle_ObChan(), transitions= {'back_to_find_middle_ObChan':'find_middle_ObChan'})

        smach.StateMachine.add('ObChan_distance_checking', ObChan_distance_checking(), transitions= { 'go_starboard':'back_to_distance_checking_ObChan', 'go_port':'back_to_distance_checking_ObChan', 'go_straight':'passing_through_ObChan_buoy'})

        smach.StateMachine.add('back_to_distance_checking_ObChan', back_to_distance_checking_ObChan(), transitions= {'back_to_distance_checking_ObChan':'ObChan_distance_checking'})

        smach.StateMachine.add('passing_through_ObChan_buoy', passing_through_ObChan_buoy(), {'passing_through_ObChan_buoy':'finished'})

# This allows you to view the state machine in a graph format 
    sis = smach_ros.IntrospectionServer('statemach_viewer', sm, '/Start Competition Run')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()  ## Allows for the node to stay active until clt+c is pressed
    sis.stop()    ## 


if __name__ == '__main__':
    main()