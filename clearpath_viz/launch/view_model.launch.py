from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/'
    )

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

    arg_rviz_config = DeclareLaunchArgument(
        name='config',
        default_value='model.rviz',
    )

    # Launch Configurations
    rviz_config = LaunchConfiguration('config')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')

    # Get Package Paths
    pkg_clearpath_platform_description = FindPackageShare('clearpath_platform_description')
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    config_rviz = PathJoinSubstitution(
        [pkg_clearpath_viz, 'rviz', rviz_config]
    )

    group_view_model = GroupAction([
        PushRosNamespace(namespace),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_rviz],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
               ('/tf', 'tf'),
               ('/tf_static', 'tf_static')
            ],
            output='screen'),
        # Joint State Publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            remappings=[
                ("joint_states", "platform/joint_states")
            ]
        ),
        # Load Robot Description
        IncludeLaunchDescription(
            PathJoinSubstitution([
                pkg_clearpath_platform_description,
                'launch',
                'description.launch.py'])
        ),
        # Live Updater
        Node(
            package='clearpath_config_live',
            executable='clearpath_config_live',
            parameters=[{'setup_path': setup_path}]
        ),
    ])

    # Generate Initial Description
    node_generate_description = Node(
        package='clearpath_generator_common',
        executable='generate_description',
        name='generate_description',
        output='screen',
        arguments=['-s', setup_path],
    )

    event_generate_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_description,
            on_exit=[group_view_model]
        )
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_setup_path)
    ld.add_action(arg_namespace)
    ld.add_action(arg_rviz_config)
    ld.add_action(arg_use_sim_time)
    # Nodes
    ld.add_action(node_generate_description)
    ld.add_action(event_generate_description)

    return ld
