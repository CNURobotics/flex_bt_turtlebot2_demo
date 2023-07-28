#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2022 Josh Zutell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Turtlebot2 Patrol And Investigate.

Created on Fri Feb 04 2022
@author: Josh Zutell
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from ball_detector_flexbe_states.ball_detection_state import BallDetectionState
from ball_detector_flexbe_states.check_ball_color_state import CheckBallColorState
from flex_bt_flexbe_states.bt_execute_goal_state import BtExecuteGoalState
from flex_bt_flexbe_states.bt_execute_state import BtExecuteState
from flex_bt_flexbe_states.bt_loader_state import BtLoaderState
from flex_bt_turtlebot2_demo_flexbe_states.charging_state import ChargingState as flex_bt_turtlebot2_demo_flexbe_states__ChargingState
from flex_bt_turtlebot2_demo_flexbe_states.check_battery_life_state import CheckBatteryLifeState as flex_bt_turtlebot2_demo_flexbe_states__CheckBatteryLifeState
from flex_bt_turtlebot2_demo_flexbe_states.get_waypoints_state import GetWaypointsState as flex_bt_turtlebot2_demo_flexbe_states__GetWaypointsState
from flex_bt_turtlebot2_demo_flexbe_states.send_waypoints_state import SendWaypointsState as flex_bt_turtlebot2_demo_flexbe_states__SendWaypointsState
from flex_nav_flexbe_states.get_pose_state import GetPoseState
from flex_nav_flexbe_states.log_path_state import LogPathState
from flexbe_states.operator_decision_state import OperatorDecisionState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class Turtlebot2PatrolAndInvestigateSM(Behavior):
    """
    Define Turtlebot2 Patrol And Investigate.

    Using Flexible Navigation with Navigation 2 Behavior Trees to control the Turtlebot2

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Turtlebot2 Patrol And Investigate'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        BallDetectionState.initialize_ros(node)
        BtExecuteGoalState.initialize_ros(node)
        BtExecuteState.initialize_ros(node)
        BtLoaderState.initialize_ros(node)
        CheckBallColorState.initialize_ros(node)
        GetPoseState.initialize_ros(node)
        LogPathState.initialize_ros(node)
        OperatorDecisionState.initialize_ros(node)
        flex_bt_turtlebot2_demo_flexbe_states__ChargingState.initialize_ros(node)
        flex_bt_turtlebot2_demo_flexbe_states__CheckBatteryLifeState.initialize_ros(node)
        flex_bt_turtlebot2_demo_flexbe_states__GetWaypointsState.initialize_ros(node)
        flex_bt_turtlebot2_demo_flexbe_states__SendWaypointsState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        # x:459 y:327, x:468 y:472
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
                                       BtLoaderState(bt_topic="bt_loader",
                                                     filepaths=["flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_plan_path.xml",
                                                                "flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_navigate_w_detection_and_battery.xml",
                                                                "flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_recovery_fallback.xml",
                                                                "flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_navigate_to_pose.xml",
                                                                "flex_bt_turtlebot2_demo_bringup/behavior_trees/red_ball_behavior.xml",
                                                                "flex_bt_turtlebot2_demo_bringup/behavior_trees/green_ball_behavior.xml",
                                                                "flex_bt_turtlebot2_demo_bringup/behavior_trees/blue_ball_behavior.xml"],
                                                     timeout=5.0),
                                       transitions={'done': 'GetChargingLocation', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:388 y:68
            OperatableStateMachine.add('GetChargingLocation',
                                       GetPoseState(topic='move_base_simple/goal'),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Low},
                                       remapping={'goal': 'charging_location'})

        # x:241 y:262, x:934 y:33, x:506 y:257, x:927 y:176
        _sm_navigate_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'low_battery', 'ball_detected'], input_keys=['min_battery'], output_keys=['ball_location', 'ball_label'])

        with _sm_navigate_1:
            # x:159 y:128
            OperatableStateMachine.add('FollowPath',
                                       BtExecuteGoalState(bt_topic="bt_executor",
                                                          bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_navigate_w_detection_and_battery.xml",
                                                          goal_id="min_battery", goal_msg_type="double", request_id="battery_percentage",
                                                          request_msg_pkg="", request_msg_type="double"),
                                       transitions={'done': 'finished', 'canceled': 'finished', 'failed': 'IsBatteryLow'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'min_battery', 'data': 'battery_life'})

            # x:364 y:128
            OperatableStateMachine.add('IsBatteryLow',
                                       flex_bt_turtlebot2_demo_flexbe_states__CheckBatteryLifeState(battery_threshold=0.2),
                                       transitions={'true': 'low_battery', 'false': 'BallDetection'},
                                       autonomy={'true': Autonomy.Low, 'false': Autonomy.Low},
                                       remapping={'battery_life': 'battery_life'})

            # x:639 y:122
            OperatableStateMachine.add('BallDetection',
                                       BallDetectionState(balls_topic='/ball_detector/balls', min_radius_pixels=-1.0),
                                       transitions={'unavailable': 'failed', 'invalid': 'failed', 'done': 'ball_detected'},
                                       autonomy={'unavailable': Autonomy.Off, 'invalid': Autonomy.Off, 'done': Autonomy.High},
                                       remapping={'ball_detected': 'ball_detected', 'goal': 'ball_location', 'ball_label': 'ball_label'})

        # x:789 y:162, x:243 y:37, x:1284 y:192, x:1278 y:328
        _sm_patrol_2 = OperatableStateMachine(outcomes=['failed', 'canceled', 'low_battery', 'finished'],
                                              input_keys=['waypoints', 'min_battery'],
                                              output_keys=['waypoints', 'ball_location', 'ball_label'])

        with _sm_patrol_2:
            # x:203 y:251
            OperatableStateMachine.add('ContinuePatrolling',
                                       OperatorDecisionState(outcomes=["yes", "no"], hint=None, suggestion="yes"),
                                       transitions={'yes': 'SendWaypoint', 'no': 'canceled'},
                                       autonomy={'yes': Autonomy.High, 'no': Autonomy.High})

            # x:1026 y:28
            OperatableStateMachine.add('LogPath',
                                       LogPathState(),
                                       transitions={'done': 'Navigate'},
                                       autonomy={'done': Autonomy.Low},
                                       remapping={'plan': 'plan'})

            # x:1041 y:250
            OperatableStateMachine.add('Navigate',
                                       _sm_navigate_1,
                                       transitions={'finished': 'ContinuePatrolling', 'failed': 'failed', 'low_battery': 'low_battery',
                                                    'ball_detected': 'finished'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit,
                                                 'low_battery': Autonomy.Inherit, 'ball_detected': Autonomy.Inherit},
                                       remapping={'min_battery': 'min_battery', 'ball_location': 'ball_location',
                                                  'ball_label': 'ball_label'})

            # x:740 y:28
            OperatableStateMachine.add('Plan',
                                       BtExecuteGoalState(bt_topic="bt_executor",
                                                          bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_plan_path.xml",
                                                          goal_id="goal", goal_msg_type="PoseStamped", request_id="path",
                                                          request_msg_pkg="nav_msgs", request_msg_type="Path"),
                                       transitions={'done': 'LogPath', 'canceled': 'ContinuePatrolling', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal', 'data': 'plan'})

            # x:385 y:27
            OperatableStateMachine.add('SendWaypoint',
                                       flex_bt_turtlebot2_demo_flexbe_states__SendWaypointsState(),
                                       transitions={'done': 'Plan', 'canceled': 'canceled'},
                                       autonomy={'done': Autonomy.Low, 'canceled': Autonomy.Low},
                                       remapping={'waypoints': 'waypoints', 'goal': 'goal'})

        # x:1131 y:33, x:1118 y:464, x:1124 y:228
        _sm_performballaction_3 = OperatableStateMachine(outcomes=['finished', 'failed', 'canceled'], input_keys=['ball_label'])

        with _sm_performballaction_3:
            # x:237 y:237
            OperatableStateMachine.add('CheckBallColor',
                                       CheckBallColorState(),
                                       transitions={'red_ball': 'RedBallAction', 'green_ball': 'GreenBallAction',
                                                    'blue_ball': 'BlueBallAction', 'failed': 'failed'},
                                       autonomy={'red_ball': Autonomy.High, 'green_ball': Autonomy.High,
                                                 'blue_ball': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'ball_label': 'ball_label'})

            # x:669 y:240
            OperatableStateMachine.add('GreenBallAction',
                                       BtExecuteState(bt_topic="bt_executor",
                                                      bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/green_ball_behavior.xml"),
                                       transitions={'done': 'finished', 'canceled': 'canceled', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off})

            # x:669 y:27
            OperatableStateMachine.add('RedBallAction',
                                       BtExecuteState(bt_topic="bt_executor",
                                                      bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/red_ball_behavior.xml"),
                                       transitions={'done': 'finished', 'canceled': 'canceled', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off})

            # x:673 y:452
            OperatableStateMachine.add('BlueBallAction',
                                       BtExecuteState(bt_topic="bt_executor",
                                                      bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/blue_ball_behavior.xml"),
                                       transitions={'done': 'finished', 'canceled': 'canceled', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off})

        # x:1021 y:81, x:1014 y:246, x:609 y:286, x:1011 y:395
        _sm_investigate_4 = OperatableStateMachine(outcomes=['finished', 'failed', 'canceled', 'low_battery'],
                                                   input_keys=['ball_location', 'ball_label'])

        with _sm_investigate_4:
            # x:163 y:180
            OperatableStateMachine.add('NavigateToBall',
                                       BtExecuteGoalState(bt_topic="bt_executor",
                                                          bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_navigate_ball_w_battery_life.xml",
                                                          goal_id="goal", goal_msg_type="PoseStamped", request_id="battery_percentage",
                                                          request_msg_pkg="", request_msg_type="double"),
                                       transitions={'done': 'PerformBallAction', 'canceled': 'canceled', 'failed': 'IsBatteryLow'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'ball_location', 'data': 'battery_life'})

            # x:556 y:69
            OperatableStateMachine.add('PerformBallAction',
                                       _sm_performballaction_3,
                                       transitions={'finished': 'finished', 'failed': 'failed', 'canceled': 'canceled'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'canceled': Autonomy.Inherit},
                                       remapping={'ball_label': 'ball_label'})

            # x:513 y:379
            OperatableStateMachine.add('IsBatteryLow',
                                       flex_bt_turtlebot2_demo_flexbe_states__CheckBatteryLifeState(battery_threshold=0.2),
                                       transitions={'true': 'low_battery', 'false': 'failed'},
                                       autonomy={'true': Autonomy.Low, 'false': Autonomy.Low},
                                       remapping={'battery_life': 'battery_life'})

        # x:775 y:84, x:397 y:224
        _sm_charge_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['charging_location'])

        with _sm_charge_5:
            # x:136 y:79
            OperatableStateMachine.add('NavigateToCharger',
                                       BtExecuteGoalState(bt_topic="bt_executor",
                                                          bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_navigate_to_pose.xml",
                                                          goal_id="goal", goal_msg_type="PoseStamped", request_id="",
                                                          request_msg_pkg="", request_msg_type=""),
                                       transitions={'done': 'Charge', 'canceled': 'Charge', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'charging_location', 'data': 'data'})

            # x:492 y:79
            OperatableStateMachine.add('Charge',
                                       flex_bt_turtlebot2_demo_flexbe_states__ChargingState(charger_topic="robot_charger",
                                                                                            battery_topic="battery_status",
                                                                                            battery_threshold=0.95),
                                       transitions={'done': 'finished', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Off})

        with _state_machine:
            # x:62 y:257
            OperatableStateMachine.add('SetupBehavior',
                                       _sm_setupbehavior_0,
                                       transitions={'finished': 'GetWaypoints', 'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                       remapping={'charging_location': 'charging_location'})

            # x:311 y:206
            OperatableStateMachine.add('GetWaypoints',
                                       flex_bt_turtlebot2_demo_flexbe_states__GetWaypointsState(timeout=5.0,
                                                                                                topic='move_base_simple/goal'),
                                       transitions={'done': 'Patrol', 'canceled': 'finished'},
                                       autonomy={'done': Autonomy.Low, 'canceled': Autonomy.Off},
                                       remapping={'waypoints': 'waypoints'})

            # x:963 y:31
            OperatableStateMachine.add('Investigate',
                                       _sm_investigate_4,
                                       transitions={'finished': 'Patrol', 'failed': 'Recovery', 'canceled': 'Patrol',
                                                    'low_battery': 'Charge'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit,
                                                 'canceled': Autonomy.Inherit, 'low_battery': Autonomy.Inherit},
                                       remapping={'ball_location': 'ball_location', 'ball_label': 'ball_label'})

            # x:594 y:200
            OperatableStateMachine.add('Patrol',
                                       _sm_patrol_2,
                                       transitions={'failed': 'Recovery', 'canceled': 'GetWaypoints', 'low_battery': 'Charge',
                                                    'finished': 'Investigate'},
                                       autonomy={'failed': Autonomy.Inherit, 'canceled': Autonomy.Inherit,
                                                 'low_battery': Autonomy.Inherit, 'finished': Autonomy.Inherit},
                                       remapping={'waypoints': 'waypoints', 'min_battery': 'min_battery',
                                                  'ball_location': 'ball_location', 'ball_label': 'ball_label'})

            # x:959 y:462
            OperatableStateMachine.add('Recovery',
                                       BtExecuteState(bt_topic="bt_executor",
                                                      bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_recovery_fallback.xml"),
                                       transitions={'done': 'Patrol', 'canceled': 'Patrol', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off})

            # x:1216 y:271
            OperatableStateMachine.add('Charge',
                                       _sm_charge_5,
                                       transitions={'finished': 'Patrol', 'failed': 'Recovery'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                       remapping={'charging_location': 'charging_location'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
