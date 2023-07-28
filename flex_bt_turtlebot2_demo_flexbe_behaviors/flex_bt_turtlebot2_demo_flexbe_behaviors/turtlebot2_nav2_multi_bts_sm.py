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
Define Turtlebot2 Nav2 Multi-BTs.

Created on Fri Feb 04 2022
@author: Josh Zutell
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flex_bt_flexbe_states.bt_execute_goal_state import BtExecuteGoalState
from flex_bt_flexbe_states.bt_execute_state import BtExecuteState
from flex_bt_flexbe_states.bt_loader_state import BtLoaderState
from flex_nav_flexbe_states.get_pose_state import GetPoseState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class Turtlebot2Nav2MultiBTsSM(Behavior):
    """
    Define Turtlebot2 Nav2 Multi-BTs.

    Using Flexible Navigation with Behavior Trees to control the Turtlebot2

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Turtlebot2 Nav2 Multi-BTs'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        BtExecuteGoalState.initialize_ros(node)
        BtExecuteState.initialize_ros(node)
        BtLoaderState.initialize_ros(node)
        GetPoseState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        # x:30 y:365, x:325 y:326
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:106 y:145
            OperatableStateMachine.add('Loader',
                                       BtLoaderState(bt_topic="bt_loader",
                                                     filepaths=["flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_plan_path.xml",
                                                                "flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_follow_path.xml",
                                                                "flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_recovery_fallback.xml"],
                                                     timeout=5.0),
                                       transitions={'done': 'GetGoal', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:351 y:146
            OperatableStateMachine.add('GetGoal',
                                       GetPoseState(topic='move_base_simple/goal'),
                                       transitions={'done': 'PlanPath'},
                                       autonomy={'done': Autonomy.Low},
                                       remapping={'goal': 'goal'})

            # x:596 y:18
            OperatableStateMachine.add('PlanPath',
                                       BtExecuteGoalState(bt_topic="bt_executor",
                                                          bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_plan_path.xml",
                                                          goal_id="goal", goal_msg_type="PoseStamped",
                                                          request_id="", request_msg_pkg="", request_msg_type=""),
                                       transitions={'done': 'FollowPath', 'canceled': 'GetGoal', 'failed': 'Recovery'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
                                       remapping={'goal': 'goal', 'data': 'data'})

            # x:602 y:317
            OperatableStateMachine.add('Recovery',
                                       BtExecuteState(bt_topic="bt_executor",
                                                      bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_recovery_fallback.xml"),
                                       transitions={'done': 'GetGoal', 'canceled': 'GetGoal', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off})

            # x:860 y:145
            OperatableStateMachine.add('FollowPath',
                                       BtExecuteState(bt_topic="bt_executor",
                                                      bt_file="flex_bt_turtlebot2_demo_bringup/behavior_trees/nav2_follow_path.xml"),
                                       transitions={'done': 'GetGoal', 'canceled': 'GetGoal', 'failed': 'Recovery'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
