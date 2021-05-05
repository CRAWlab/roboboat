#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from navigation_channel_flexbe_states.deploy_hydrophones import DeployHydrophones as navigation_channel_flexbe_states__DeployHydrophones
from navigation_channel_flexbe_states.localize_signal import LocalizeSignal as navigation_channel_flexbe_states__LocalizeSignal
from navigation_channel_flexbe_states.record_docking_bay_signal import RecordDockingBaySymbol as navigation_channel_flexbe_states__RecordDockingBaySymbol
from navigation_channel_flexbe_states.enter_docking_bay import EnterDockingBay as navigation_channel_flexbe_states__EnterDockingBay
from navigation_channel_flexbe_states.wait import Wait as navigation_channel_flexbe_states__Wait
from navigation_channel_flexbe_states.exit_bay import ExitDockingBay
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 20 2020
@author: Gerald
'''
class AcousticdockingSM(Behavior):
	'''
	Dock in the correct bay corresponding to a broadcast acoustic signal
	'''


	def __init__(self):
		super(AcousticdockingSM, self).__init__()
		self.name = 'Acoustic docking'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:671 y:249
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:96 y:32
			OperatableStateMachine.add('Deploy hydrophones',
										navigation_channel_flexbe_states__DeployHydrophones(),
										transitions={'hydrophones_deployed': 'Localize signal'},
										autonomy={'hydrophones_deployed': Autonomy.Off})

			# x:94 y:138
			OperatableStateMachine.add('Localize signal',
										navigation_channel_flexbe_states__LocalizeSignal(),
										transitions={'signal_localized': 'Record bay signal'},
										autonomy={'signal_localized': Autonomy.Off})

			# x:93 y:239
			OperatableStateMachine.add('Record bay signal',
										navigation_channel_flexbe_states__RecordDockingBaySymbol(),
										transitions={'symbol_stored': 'Enter docking bay'},
										autonomy={'symbol_stored': Autonomy.Off})

			# x:388 y:29
			OperatableStateMachine.add('Enter docking bay',
										navigation_channel_flexbe_states__EnterDockingBay(),
										transitions={'entered_bay': 'Wait'},
										autonomy={'entered_bay': Autonomy.Off})

			# x:389 y:140
			OperatableStateMachine.add('Wait',
										navigation_channel_flexbe_states__Wait(),
										transitions={'finished_waiting': 'Exit docking bay'},
										autonomy={'finished_waiting': Autonomy.Off})

			# x:388 y:239
			OperatableStateMachine.add('Exit docking bay',
										ExitDockingBay(),
										transitions={'exited_bay': 'finished'},
										autonomy={'exited_bay': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
