#!/usr/bin/env python
# StateMachine for the ASV and UAV for the CRAWLAB

import rospy
import smach
import smach_ros

from smach_ros import SimpleActionState
from smach import Concurrence

## State finding navigation channel  ################################################
class find_NavChan_start(smach.State):  # Looking for the red and green buoy and the gps coordinate 
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
            return 'passing_through_NavChan'
####################################################################################

#################################    Obstacle channel    ###########################
####################################################################################
####################################################################################

## State to find the Obstacle Channel 
class find_ObChan_start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['found', 'not_found'])
        self.counter = 0

    def execute(self, userdata): #This is where all the code for finding the obstacle channel goes looking for GPS coordinate and proper buoys.
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

## State that could not find the middle of the Obstacle still looking ##############
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
class passing_through_ObChan_buoys(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['passing_through_ObChan_buoys'])

    def execute(self, userdata): #All code for passing through goes here
            rospy.loginfo('Passing through')
            return 'passing_through_ObChan_buoys'
####################################################################################

## State to exit the obstacle channel   ############################################
class exiting_ObChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['exiting_ObChan'])

    def execute(self, userdata): #All code for passing through goes here
            rospy.loginfo('Exiting the Obstacle Channel')
            return 'exiting_ObChan'
####################################################################################

#################################   Obstacle Field   ###############################
####################################################################################
####################################################################################

## State to find the obstacle field ################################################
class find_ObField(smach.State): ##Looking for the pill buoy
    def __init__(self):
        smach.State.__init__(self, outcomes = ['found', 'not_found'])
        self.counter = 0

    def execute(self, userdata): #This is where all the code for finding the navigation channel goes looking for GPS coordinate and trying to find the two proper buoys
            rospy.loginfo('Finding Obstacle Field')
            if self.counter < 3:
                    self.counter += 1
                    rospy.loginfo('Obstacle Field not found')
                    return 'not_found'
            else:
                    return 'found'
####################################################################################

## State that could not find the obstacle field still looking ######################
class cannot_find_ObField(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_find_ObField'])

    def execute(self, userdata):
            rospy.loginfo('Cannot find Obstacle Field')
            return 'back_to_find_ObField'
####################################################################################

## State to find the opening in the obstacle field #################################
class locate_ObField_opening(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_opening', 'no_opening'])
        self.counter = 0

    def execute(self, userdata):
            rospy.loginfo('Still circling obstacle field')
            if self.counter < 2:
                    self.counter += 1
                    rospy.loginfo('Obstacle Field entrance not found')
                    return 'no_opening'
            else:
                    return 'found_opening'
####################################################################################

## State to return to locating the opening in the obstacle field####################
class back_to_locate_ObField_opening(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_locate_ObField_opening'])

    def execute(self, userdata): 
        rospy.loginfo('Searching for opening')
        return 'back_to_locate_ObField_opening'
####################################################################################

## State to find the opening in the obstacle field #################################
class locate_pill_buoy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_pill_buoy'])

    def execute(self, userdata):
            rospy.loginfo('Searching for pill buoy')
            return 'found_pill_buoy'
####################################################################################

## State to find the opening in the obstacle field #################################
class circle_pill_buoy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['circling'])

    def execute(self, userdata):
            rospy.loginfo('Searching for pill buoy')
            return 'circling'
####################################################################################

## State to exit the obstacle field   ############################################
class exiting_ObField(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['exiting_ObField'])

    def execute(self, userdata): #All code for passing through goes here
            rospy.loginfo('Exiting the Obstacle Field')
            return 'exiting_ObField'
####################################################################################

################################ Launch UAV   ######################################
####################################################################################
####################################################################################

## State to for UAV to launch ######################################################
class launching(smach.State): ##Launching
    def __init__(self):
        smach.State.__init__(self, outcomes = ['launching', 'cannot_launch'])
        self.counter = 0

    def execute(self, userdata): #This is where all the code for launching 
            rospy.loginfo('Launching Now')
            if self.counter < 3:
                    self.counter += 1
                    rospy.loginfo('Cannot launch. It is unsafe')
                    return 'cannot_launch'
            else:
                    return 'launching'
####################################################################################

## State that could not launch because it was unsafe ###############################
class back_to_launching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_launching'])

    def execute(self, userdata):
            rospy.loginfo('It is unsafe to launch now, but re-checking and trying again')
            return 'back_to_launching'
####################################################################################

####################################################################################
class UAV_hovering(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['UAV_hovering'])
        self.counter = 0
    
    def execute(self, userdata):
            rospy.loginfo('UAV is hovering to ensure stability')
            
            return('UAV_hovering')
####################################################################################

## State for UAV to find the obstacle channel ######################################
class UAV_finding_ObChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['UAV_found_ObChan', 'UAV_ObChan_not_found'])
        self.counter = 0

    def execute(self, userdata):
            rospy.loginfo('Still looking for Obstacle Channel Start')
            if self.counter < 2:
                    self.counter += 1
                    rospy.loginfo('Obstacle Field entrance not found')
                    return 'UAV_ObChan_not_found'
            else:
                    return 'UAV_found_ObChan'
####################################################################################

## State to return to searching for the obstacle channel ###########################
class UAV_still_searching_ObChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_UAV_finding_ObChan'])

    def execute(self, userdata): 
        rospy.loginfo('Searching for Obstacle Channel')
        return 'back_to_UAV_finding_ObChan'
####################################################################################

## State to follow the the obstacle channel ########################################
class UAV_following_ObChan_buoys(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['UAV_following_ObChan_buoys'])

    def execute(self, userdata):
            rospy.loginfo('Following buoys in obstacle channel')
            return 'UAV_following_ObChan_buoys'
####################################################################################

## State to "exit" the the obstacle channel ########################################
class UAV_exiting_ObChan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['UAV_exiting_ObChan'])

    def execute(self, userdata):
            rospy.loginfo('UAV is exiting Obstacle Channel')
            return 'UAV_exiting_ObChan'
####################################################################################

## State for UAV to find the obstacle field ########################################
class UAV_finding_ObField(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['UAV_found_ObField', 'UAV_ObField_not_found'])
        self.counter = 0

    def execute(self, userdata):
            rospy.loginfo('Still looking for Obstacle Field')
            if self.counter < 2:
                    self.counter += 1
                    rospy.loginfo('Obstacle Field entrance not found')
                    return 'UAV_ObField_not_found'
            else:
                    return 'UAV_found_ObField'
####################################################################################

## State to return to searching for the obstacle field ###########################
class UAV_still_searching_ObField(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['back_to_UAV_finding_ObField'])

    def execute(self, userdata): 
        rospy.loginfo('Searching for Obstacle Field')
        return 'back_to_UAV_finding_ObField'
####################################################################################

## State to circle the obstacle field ##############################################
class circling_ObField(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['circling_ObField'])

    def execute(self, userdata):
            rospy.loginfo('UAV is exiting Obstacle Channel')
            return 'circling_ObField'
####################################################################################




def main():
    rospy.init_node('roboboatStateMachine')

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['finished'])
    # uav_sm = smach.StateMachine(outcomes=['uav_finished'])
    # Open the container
    with sm_top:
       # rospy.init_node('smach_roboboat_state_machine')
        
        # Navigation Containers
        smach.StateMachine.add('find_NavChan', find_NavChan_start(), transitions= {'not_found':'cannot_find_NavChan', 'found':'find_middle_NavChan'})

        smach.StateMachine.add('cannot_find_NavChan', cannot_find_NavChan(), transitions={'back_to_find_NavChan':'find_NavChan'})

        smach.StateMachine.add('find_middle_NavChan', find_middle_NavChan(), transitions= {'not_found':'cannot_find_middle_NavChan', 'found_middle_NavChan':'NavChan_distance_checking'})

        smach.StateMachine.add('cannot_find_middle_NavChan', cannot_find_middle_NavChan(), transitions= {'back_to_find_middle':'find_middle_NavChan'})

        smach.StateMachine.add('NavChan_distance_checking', NavChan_distance_checking(), transitions= { 'go_starboard':'back_to_distance_checking_NavChan', 'go_port':'back_to_distance_checking_NavChan', 'go_straight':'passing_through_NavChan'})

        smach.StateMachine.add('back_to_distance_checking_NavChan', back_to_distance_checking_NavChan(), transitions= {'back_to_distance_checking_NavChan':'NavChan_distance_checking'})

        smach.StateMachine.add('passing_through_NavChan', passing_through_NavChan(), {'passing_through_NavChan':'find_ObChan_start'})

        ####################### Starting the Obstacle channel Containers #######################################
        ########################################################################################################

        smach.StateMachine.add('find_ObChan_start', find_ObChan_start(), transitions= {'not_found':'cannot_find_ObChan', 'found':'find_middle_ObChan'})

        smach.StateMachine.add('cannot_find_ObChan', cannot_find_ObChan(), transitions={'back_to_find_ObChan':'find_ObChan_start'})

        smach.StateMachine.add('find_middle_ObChan', find_middle_ObChan(), transitions= {'not_found':'cannot_find_middle_ObChan', 'found_middle_ObChan':'ObChan_distance_checking'})

        smach.StateMachine.add('cannot_find_middle_ObChan', cannot_find_middle_ObChan(), transitions= {'back_to_find_middle_ObChan':'find_middle_ObChan'})

        smach.StateMachine.add('ObChan_distance_checking', ObChan_distance_checking(), transitions= { 'go_starboard':'back_to_distance_checking_ObChan', 'go_port':'back_to_distance_checking_ObChan', 'go_straight':'passing_through_ObChan_buoys'})

        smach.StateMachine.add('back_to_distance_checking_ObChan', back_to_distance_checking_ObChan(), transitions= {'back_to_distance_checking_ObChan':'ObChan_distance_checking'})

        smach.StateMachine.add('passing_through_ObChan_buoys', passing_through_ObChan_buoys(), transitions={'passing_through_ObChan_buoys':'exiting_ObChan'})

        smach.StateMachine.add('exiting_ObChan', exiting_ObChan(), transitions= {'exiting_ObChan':'find_ObField'})

        ####################### Starting the Obstacle Field Containers #######################################
        ######################################################################################################

        smach.StateMachine.add('find_ObField', find_ObField(), transitions= {'not_found':'cannot_find_ObField', 'found':'locate_ObField_opening'})

        smach.StateMachine.add('cannot_find_ObField', cannot_find_ObField(), transitions={'back_to_find_ObField':'find_ObField'})

        smach.StateMachine.add('locate_ObField_opening', locate_ObField_opening(), transitions={'found_opening':'locate_pill_buoy', 'no_opening':'back_to_locate_ObField_opening'})

        smach.StateMachine.add('back_to_locate_ObField_opening', back_to_locate_ObField_opening(), transitions={'back_to_locate_ObField_opening':'locate_ObField_opening'})

        smach.StateMachine.add('locate_pill_buoy', locate_pill_buoy(), transitions={'found_pill_buoy':'circle_pill_buoy'})

        smach.StateMachine.add('circle_pill_buoy', circle_pill_buoy(), transitions={'circling':'exiting_ObField'})

        smach.StateMachine.add('exiting_ObField', exiting_ObField(), transitions={'exiting_ObField':'finished'})
       
        ####################### Starting UAV SUB statemachine ################################################
        ######################################################################################################

        sm_sub = smach.StateMachine(outcomes=['uav_finished'])

        with sm_sub:

            smach.StateMachine.add('launching', launching(), transitions= {'cannot_launch':'back_to_launching', 'launching':'UAV_hovering'})

            smach.StateMachine.add('back_to_launching', back_to_launching(), transitions= {'back_to_launching':'launching'})            

            smach.StateMachine.add('UAV_hovering', UAV_hovering(), transitions={'UAV_hovering':'UAV_finding_ObChan'})  

            smach.StateMachine.add('UAV_finding_ObChan', UAV_finding_ObChan(), transitions={'UAV_ObChan_not_found':'UAV_still_searching_ObChan', 'UAV_found_ObChan':'UAV_following_ObChan_buoys'})

            smach.StateMachine.add('UAV_still_searching_ObChan', UAV_still_searching_ObChan(), transitions={'back_to_UAV_finding_ObChan':'UAV_finding_ObChan'})

            smach.StateMachine.add('UAV_following_ObChan_buoys', UAV_following_ObChan_buoys(), transitions={'UAV_following_ObChan_buoys':'UAV_exiting_ObChan'})

            smach.StateMachine.add('UAV_exiting_ObChan', UAV_exiting_ObChan(), transitions={'UAV_exiting_ObChan':'UAV_finding_ObField'})

            smach.StateMachine.add('UAV_finding_ObField', UAV_finding_ObField(), transitions={'UAV_found_ObField':'circling_ObField', 'UAV_ObField_not_found':'UAV_still_searching_ObField'})

            smach.StateMachine.add('UAV_still_searching_ObField', UAV_still_searching_ObField(), transitions={'back_to_UAV_finding_ObField':'UAV_finding_ObField'})

            smach.StateMachine.add('circling_ObField', circling_ObField(), transitions={'circling_ObField':'uav_finished'})

# This allows you to view the state machine in a graph format 
    top = smach_ros.IntrospectionServer('ASV_viewer', sm_top, '/Start Competition Run')
    top.start()
    
    sub = smach_ros.IntrospectionServer('UAV_Viewer', sm_sub, '/View UAV States')
    sub.start()

    # Execute SMACH plan
    outcome = sm_top.execute()
    outcome = sm_sub.execute()
    rospy.spin()  ## Allows for the node to stay active until clt+c is pressed
    top.stop()    ## 
    sub.stop()


if __name__ == '__main__':
    main()