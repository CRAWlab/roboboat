#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from navigation_channel_flexbe_states.search_buoys_obs_channel import SearchBuoysObsChannel
from navigation_channel_flexbe_states.move_to_next_waypoint import MoveToNextWaypoint as navigation_channel_flexbe_states__MoveToNextWaypoint
from navigation_channel_flexbe_states.choose_waypoint import ChooseWaypoint
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 18 2020
@author: Gerald
'''
class Obstacle_channelSM(Behavior):
	'''
	Navigate roboboat through the obstacle channel while avoiding collisions
	'''


	def __init__(self):
		super(Obstacle_channelSM, self).__init__()
		self.name = 'Obstacle_channel'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:456 y:41
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:112 y:50
			OperatableStateMachine.add('Search for buoys',
										SearchBuoysObsChannel(),
										transitions={'found_buoys': 'Choose waypoint', 'finished_channel': 'finished'},
										autonomy={'found_buoys': Autonomy.Off, 'finished_channel': Autonomy.Off})

			# x:111 y:241
			OperatableStateMachine.add('Move to next waypoint',
										navigation_channel_flexbe_states__MoveToNextWaypoint(),
										transitions={'reached_waypoint': 'Search for buoys'},
										autonomy={'reached_waypoint': Autonomy.Off})

			# x:383 y:144
			OperatableStateMachine.add('Choose waypoint',
										ChooseWaypoint(),
										transitions={'found_waypoint': 'Move to next waypoint'},
										autonomy={'found_waypoint': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
