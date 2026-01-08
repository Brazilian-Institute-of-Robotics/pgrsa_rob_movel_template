# bringup_nav2_map_with_amcl.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Args
    log_level = LaunchConfiguration('log_level')
    declare_log = DeclareLaunchArgument('log_level', default_value='INFO')

    # Pacotes e caminhos
    gz_pkg_share = get_package_share_directory('scout_gazebo_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    pgrsa_scout_pkg_share = get_package_share_directory('pgrsa_scout')

    nav2_params = os.path.join(pgrsa_scout_pkg_share, 'config', 'scout_nav2_params.yaml')
    map_yaml    = os.path.join(gz_pkg_share, 'map', 'powerstation.yaml')
    
    # 1) Display (RViz + RSP/JSP)
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, 'launch', 'display.launch.py')
        ),
        launch_arguments={'gui': 'true'}.items()
    )

    # 3) Map Server (fornece /map do arquivo salvo)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml,
            'frame_id': 'map',
            'topic_name': 'map'
        }]
    )

    # 4) AMCL (publica map->odom)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params, {'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', log_level],
    )
        # --- Nav2 core servers ---
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
        remappings=[
        ('/cmd_vel', '/cmd_vel_nav'),
    ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
    )


    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/skid_drive_controller/cmd_vel_unstamped'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
            ]
        }],
    )
 
    
    ekf_yaml = os.path.join(pgrsa_scout_pkg_share, 'config', 'ekf_localization.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_log,
        display,
        ekf_node,
        amcl,
        map_server,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
    ])
