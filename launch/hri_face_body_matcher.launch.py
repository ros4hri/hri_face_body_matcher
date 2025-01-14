# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    face_body_matcher_node = LifecycleNode(
        package='hri_face_body_matcher', executable='hri_face_body_matcher', namespace='',
        name='hri_face_body_matcher')

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(face_body_matcher_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=face_body_matcher_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(face_body_matcher_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
        face_body_matcher_node,
        configure_event,
        activate_event])
