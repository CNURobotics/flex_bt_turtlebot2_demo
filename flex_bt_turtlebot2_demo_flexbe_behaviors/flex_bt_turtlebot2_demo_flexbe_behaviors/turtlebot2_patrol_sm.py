#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flex_bt_flexbe_states.bt_execute_data_state import BtExecuteDataState
from flex_bt_flexbe_states.bt_execute_goal_state import BtExecuteGoalState
from flex_bt_flexbe_states.bt_execute_state import BtExecuteState
from flex_bt_flexbe_states.bt_loader_state import BtLoaderState
from flex_nav_flexbe_states.charging_state import ChargingState
from flex_nav_flexbe_states.check_battery_life_state import CheckBatteryLifeState
from flex_nav_flexbe_states.get_pose_state import GetPoseState
from flex_nav_flexbe_states.get_waypoints_state import GetWaypointsState
from flex_nav_flexbe_states.log_path_state import LogPathState
from flex_nav_flexbe_states.send_waypoints_state import SendWaypointsState
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Feb 04 2022
@author: Josh Zutell
'''
class Turtlebot2PatrolSM(Behavior):
	'''
	Using Flexible Navigation with Behavior Trees to control the Turtlebot2
	'''


	def __init__(self, node):
		super(Turtlebot2PatrolSM, self).__init__()
		self.name = 'Turtlebot2 Patrol'

		# parameters of this behavior

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		BtLoaderState.initialize_ros(node)
		BtExecuteDataState.initialize_ros(node)
		BtExecuteGoalState.initialize_ros(node)
		BtExecuteState.initialize_ros(node)
		ChargingState.initialize_ros(node)
		CheckBatteryLifeState.initialize_ros(node)
		GetPoseState.initialize_ros(node)
		GetWaypointsState.initialize_ros(node)
		LogPathState.initialize_ros(node)
		OperatorDecisionState.initialize_ros(node)
		SendWaypointsState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:642 y:164
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.min_battery = 0.20

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]

		# x:623 y:85, x:296 y:227
		_sm_setupbehavior_0 = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['charging_location'])

		with _sm_setupbehavior_0:
			# x:96 y:67
			OperatableStateMachine.add('LoadBTs',
										BtLoaderState(bt_topic="bt_loader", filepaths=["flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_plan_path.xml", "flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_follow_path_w_battery_life.xml", "flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_recovery_fallback.xml", "flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_navigate_to_pose.xml"], timeout=5.0),
										transitions={'done': 'GetChargingLocation', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:388 y:68
			OperatableStateMachine.add('GetChargingLocation',
										GetPoseState(topic='move_base_simple/goal'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low},
										remapping={'goal': 'charging_location'})


		# x:1239 y:362, x:795 y:470, x:233 y:37
		_sm_patrol_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'canceled'], input_keys=['waypoints', 'min_battery'], output_keys=['waypoints'])

		with _sm_patrol_1:
			# x:191 y:181
			OperatableStateMachine.add('ContinuePatrolling',
										OperatorDecisionState(outcomes=["yes", "no"], hint=None, suggestion=None),
										transitions={'yes': 'SendWaypoint', 'no': 'canceled'},
										autonomy={'yes': Autonomy.Low, 'no': Autonomy.Full})

			# x:1023 y:185
			OperatableStateMachine.add('FollowPath',
										BtExecuteGoalState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_follow_path_w_battery_life.xml", goal_id="min_battery", goal_msg_type="double", request_id="battery_percentage", request_msg_pkg="", request_msg_type="double"),
										transitions={'done': 'ContinuePatrolling', 'canceled': 'ContinuePatrolling', 'failed': 'IsBatteryLow'},
										autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'goal': 'min_battery', 'data': 'battery_life'})

			# x:1018 y:349
			OperatableStateMachine.add('IsBatteryLow',
										CheckBatteryLifeState(),
										transitions={'true': 'finished', 'false': 'Recover'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'battery_life': 'battery_life'})

			# x:1020 y:28
			OperatableStateMachine.add('LogPath',
										LogPathState(),
										transitions={'done': 'FollowPath'},
										autonomy={'done': Autonomy.Low},
										remapping={'plan': 'plan'})

			# x:740 y:28
			OperatableStateMachine.add('Plan',
										BtExecuteGoalState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_plan_path.xml", goal_id="goal", goal_msg_type="PoseStamped", request_id="path", request_msg_pkg="nav_msgs", request_msg_type="Path"),
										transitions={'done': 'LogPath', 'canceled': 'ContinuePatrolling', 'failed': 'Recover'},
										autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'goal': 'goal', 'data': 'plan'})

			# x:746 y:354
			OperatableStateMachine.add('Recover',
										BtExecuteState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_recovery_fallback.xml"),
										transitions={'done': 'ContinuePatrolling', 'canceled': 'ContinuePatrolling', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off})

			# x:418 y:28
			OperatableStateMachine.add('SendWaypoint',
										SendWaypointsState(),
										transitions={'done': 'Plan', 'canceled': 'canceled'},
										autonomy={'done': Autonomy.Low, 'canceled': Autonomy.Low},
										remapping={'waypoints': 'waypoints', 'goal': 'goal'})


		# x:703 y:76, x:397 y:224
		_sm_charge_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['charging_location'])

		with _sm_charge_2:
			# x:219 y:79
			OperatableStateMachine.add('NavigateToCharger',
										BtExecuteDataState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_navigate_to_pose.xml", goal_id="goal", goal_msg_type="PoseStamped", request_id="", request_msg_pkg="", request_msg_type=""),
										transitions={'done': 'Charge', 'canceled': 'Charge', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'goal': 'charging_location', 'data': 'data'})

			# x:480 y:71
			OperatableStateMachine.add('Charge',
										ChargingState(charger_topic="robot_charger", battery_topic="battery_status", battery_threshold=0.95),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Off})



		with _state_machine:
			# x:78 y:145
			OperatableStateMachine.add('SetupBehavior',
										_sm_setupbehavior_0,
										transitions={'finished': 'GetWaypoints', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'charging_location': 'charging_location'})

			# x:286 y:65
			OperatableStateMachine.add('GetWaypoints',
										GetWaypointsState(timeout=5.0, topic='move_base_simple/goal'),
										transitions={'done': 'Patrol'},
										autonomy={'done': Autonomy.Low},
										remapping={'waypoints': 'waypoints'})

			# x:599 y:8
			OperatableStateMachine.add('Patrol',
										_sm_patrol_1,
										transitions={'finished': 'Charge', 'failed': 'failed', 'canceled': 'GetWaypoints'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'canceled': Autonomy.Inherit},
										remapping={'waypoints': 'waypoints', 'min_battery': 'min_battery'})

			# x:1037 y:139
			OperatableStateMachine.add('Charge',
										_sm_charge_2,
										transitions={'finished': 'Patrol', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'charging_location': 'charging_location'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
