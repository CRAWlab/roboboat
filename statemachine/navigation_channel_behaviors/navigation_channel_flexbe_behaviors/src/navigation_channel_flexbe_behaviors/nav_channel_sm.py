#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from navigation_channel_flexbe_states.search_buoys import SearchBuoys
from navigation_channel_flexbe_states.choose_waypoint import ChooseWaypoint
from navigation_channel_flexbe_states.move_to_waypoint import MoveToWaypoint
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 15 2020
@author: Gerald
'''
class Nav_channelSM(Behavior):
	'''
	Navigate the roboboat through the navigation channel
	'''


	def __init__(self):
		super(Nav_channelSM, self).__init__()
		self.name = 'Nav_channel'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:21 y:335
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:185 y:27
			OperatableStateMachine.add('Search for buoys',
										SearchBuoys(),
										transitions={'found_buoys': 'Determine waypoint'},
										autonomy={'found_buoys': Autonomy.Off})

			# x:440 y:153
			OperatableStateMachine.add('Determine waypoint',
										ChooseWaypoint(),
										transitions={'found_waypoint': 'Move to waypoint'},
										autonomy={'found_waypoint': Autonomy.Off})

			# x:184 y:324
			OperatableStateMachine.add('Move to waypoint',
										MoveToWaypoint(),
										transitions={'reached_waypoint1': 'Search for buoys', 'reached_waypoint2': 'finished'},
										autonomy={'reached_waypoint1': Autonomy.Off, 'reached_waypoint2': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
