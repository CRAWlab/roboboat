#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from navigation_channel_flexbe_states.find_pill_buoy import FindPillBuoy
from navigation_channel_flexbe_states.circum_nav_obs_field import CircumNavObsField
from navigation_channel_flexbe_states.go_through_opening import GoThroughOpening
from navigation_channel_flexbe_states.circum_nav_pill_buoy import CircumNavPillBuoy
from navigation_channel_flexbe_states.exit_obs_field import ExitObsField
from navigation_channel_flexbe_states.move_to_opening import MoveToOpening
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 20 2020
@author: Gerald
'''
class Obstacle_fieldSM(Behavior):
	'''
	Complete the obstacle field task
	'''


	def __init__(self):
		super(Obstacle_fieldSM, self).__init__()
		self.name = 'Obstacle_field'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:647 y:278
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:83 y:50
			OperatableStateMachine.add('Find pill buoy',
										FindPillBuoy(),
										transitions={'outside_field': 'Look for opening', 'inside_field': 'Circle pill buoy'},
										autonomy={'outside_field': Autonomy.Off, 'inside_field': Autonomy.Off})

			# x:373 y:131
			OperatableStateMachine.add('Look for opening',
										CircumNavObsField(),
										transitions={'found_opening': 'Move to opening'},
										autonomy={'found_opening': Autonomy.Off})

			# x:940 y:49
			OperatableStateMachine.add('Go through opening',
										GoThroughOpening(),
										transitions={'passed_opening': 'Find pill buoy'},
										autonomy={'passed_opening': Autonomy.Off})

			# x:72 y:234
			OperatableStateMachine.add('Circle pill buoy',
										CircumNavPillBuoy(),
										transitions={'circled_buoy': 'Exit obstacle field'},
										autonomy={'circled_buoy': Autonomy.Off})

			# x:354 y:250
			OperatableStateMachine.add('Exit obstacle field',
										ExitObsField(),
										transitions={'exit_obs_field': 'finished'},
										autonomy={'exit_obs_field': Autonomy.Off})

			# x:698 y:131
			OperatableStateMachine.add('Move to opening',
										MoveToOpening(),
										transitions={'reached_opening': 'Go through opening'},
										autonomy={'reached_opening': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
