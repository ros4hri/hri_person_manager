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
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    param_args = [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in [
        ('match_threshold', '0.5', "Minimum likelihood to associate a face/body/voice to a given "
                                   "person"),
        ('reference_frame', 'map', "This should usually be a static frame wrt to the robot"),
        ('robot_reference_frame', 'base_link', "Reference frame for persons' distance "
                                               "computation"),
        ('personal_distance', '1.2', "Person upper distance threshold for personal zone "
                                     "(the nearest one) (m)"),
        ('social_distance', '3.6', "Person distance threshold between personal (nearer) and "
                                   "social (farther) zone (m)"),
        ('public_distance', '20.', "Person distance threshold between social (nearer) and public "
                                   "(farther) zone (m)")]]

    person_manager_node = LifecycleNode(
        package='hri_person_manager', executable='hri_person_manager', namespace='',
        name='hri_person_manager',
        parameters=[{arg.name: LaunchConfiguration(arg.name)} for arg in param_args])

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(person_manager_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=person_manager_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(person_manager_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription(param_args + [
        person_manager_node,
        configure_event,
        activate_event])
