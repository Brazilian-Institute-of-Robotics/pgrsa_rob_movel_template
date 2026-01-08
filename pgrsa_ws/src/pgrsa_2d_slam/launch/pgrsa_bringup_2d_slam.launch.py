import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    # STRING REAL (Python)
    pkg_share = get_package_share_directory("pgrsa_2d_slam")
    worlds_dir = os.path.join(pkg_share, "worlds")
    ekf_yaml = os.path.join(pkg_share, 'config', 'ekf_localization.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz.rviz')
    
    rviz_config_arg = DeclareLaunchArgument(
    'rviz_config_file',
    default_value=rviz_config_file,
    description='RViz config file'
    )

    append_ign_resource = AppendEnvironmentVariable(
    name="IGN_GAZEBO_RESOURCE_PATH",
    value=worlds_dir
    )

    world_file = os.path.join(worlds_dir, "warehouse_world.sdf")

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r -v4 {world_file}",
            "on_exit_shutdown": "true"
        }.items()
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scout_gazebo_sim"),
                "launch",
                "gz_bridge.launch.py"
            )
        )
    )
    
    car_sim_options = {
        'start_x': '0.0',
        'start_y': '7.0',
        'start_z': '0.84',
        'start_yaw': '0.0',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false'
    }

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                         get_package_share_directory('scout_gazebo_sim'),
                         'launch', 'scout_v2_spawn.launch.py')
        ]),
        launch_arguments=car_sim_options.items()
    )
    
    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{
                # Mapeamento típico (ajuste se seu controle for diferente)
                'axis_linear.x': 1,        # analógico esquerdo vertical
                'scale_linear.x': 1.5,     # m/s

                'axis_angular.yaw': 0,     # analógico esquerdo horizontal
                'scale_angular.yaw': 1.0,  # rad/s

                # "deadman switch" (segurar para habilitar)
                'enable_button': 8,        # LB (Xbox)
                'enable_turbo_button': 5,  # RB (Xbox)

                # opcional: turbo
                'scale_linear_turbo.x': 1.2,
                'scale_angular_turbo.yaw': 3.0,
            }],
            remappings=[
                ('/cmd_vel', '/skid_drive_controller/cmd_vel_unstamped'),
                # Se você quiser mudar a origem do joy, descomente:
                # ('/joy', '/joy'),
            ],)
        
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',   # ajuste se necessário
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],)

    localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': True}],)
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )
    

    return LaunchDescription([
        use_sim_time,
        rviz_config_arg,
        append_ign_resource,
        gazebo_simulator,
        bridge,
        spawn_car,
        localization,
        joy_node,
        teleop_twist_joy_node,
        rviz_node,
    ])

