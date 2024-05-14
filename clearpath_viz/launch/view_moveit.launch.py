#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
from pprint import pprint
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from clearpath_config.common.utils.yaml import read_yaml, write_yaml

MARKER = 'rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/'

MOVEIT_TOPICS = [
    'attached_collision_object',
    'compute_cartesian_path',
    'display_planned_path',
    'get_planner_params',
    'monitored_planning_scene',
    'move_action',
    'query_planner_interface',
    'set_planner_params',
    'trajectory_execution_event',
    MARKER + 'feedback',
    MARKER + 'get_interactive_markers',
    MARKER + 'update',
]


def launch_setup(context, *args, **kwargs):
    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    arm_count = LaunchConfiguration('arm_count')

    # Apply Namespace to Rviz Configuration
    context_namespace = namespace.perform(context)

    # RViz Configuration
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')
    default_config = PathJoinSubstitution(
        [pkg_clearpath_viz, 'rviz', 'moveit.rviz']
    )

    context_rviz = default_config.perform(context)
    content_rviz = read_yaml(context_rviz)

    content_rviz[
        'Visualization Manager'][
            'Displays'][1][
                'Move Group Namespace'] = '/' + context_namespace

    namespaced_config = '/tmp/moveit.rviz'
    write_yaml(namespaced_config, content_rviz)

    # Remappings
    remappings = [
        # Standard
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # Remappings for MoveIt!
    for topic in MOVEIT_TOPICS:
        # Standard Topics
        remappings.append(('/%s' % topic, topic))
        # Doubled Topics
        remappings.append(('/%s/%s/' % (context_namespace, context_namespace) + topic, topic))

    # Arm Kinematics
    parameters = {}
    parameters['use_sim_time'] = use_sim_time
    parameters['robot_description_kinematics'] = {}
    for i in range(int(arm_count.perform(context))):
        parameters['robot_description_kinematics'].update(
            {'arm_%s' % i: {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.005,
                'kinematics_solver_timeout': 0.005,
            }}
        )

    return [
        GroupAction([
            PushRosNamespace(namespace),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', namespaced_config],
                parameters=[parameters],
                remappings=remappings,
                output='screen'
            )
        ])
    ]


def generate_launch_description():
    # Launch Arguments
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    arg_arm_count = DeclareLaunchArgument(
        'arm_count',
        default_value='1',
        description='Number of arms to add kinematics for.'
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_namespace)
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_arm_count)
    # Nodes
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
