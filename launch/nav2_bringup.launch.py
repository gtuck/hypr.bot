from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    remaps = [
        ('/scan', '/sensors/lidar'),
        ('/cmd_vel', '/cmd_vel'),
    ]

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=['params/nav2_params.yaml'],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=['params/nav2_params.yaml'],
            remappings=remaps
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=['params/nav2_params.yaml']
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['params/nav2_params.yaml'],
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=['params/nav2_params.yaml']
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=['params/nav2_params.yaml']
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=['params/nav2_params.yaml']
        ),
    ])
