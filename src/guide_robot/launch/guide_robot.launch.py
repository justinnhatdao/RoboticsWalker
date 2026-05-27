"""
BEFORE running, paste this into terminal (required every session):

export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch guide_robot guide_robot.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_guide_robot = get_package_share_directory('guide_robot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # In launch file
   
    x_pose = LaunchConfiguration('x_pose', default='3.0')
    y_pose = LaunchConfiguration('y_pose', default='3.0')

    world = os.path.join(pkg_guide_robot, 'worlds', 'guide_house.world')
    # HOME PC: map_file = '/home/justin/turtlebot3_ws/src/guide_robot/maps/map.yaml'
    # LAB/SEED: map_file = os.path.join(pkg_guide_robot, 'maps', 'map.yaml')
    map_file = os.path.join(pkg_guide_robot, 'maps', 'map.yaml')

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world]}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'launch',
                'navigation2.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file,
            'rviz_config': os.path.join(pkg_guide_robot, 'rviz', 'guide_robot.rviz'),
        }.items()
    )

    teleop_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run guide_robot teleop_game'],
        output='screen',
    )

    waypoint_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run guide_robot waypoint_navigator'],
        output='screen',
    )

    delayed_nav2 = TimerAction(
        period=10.0,
        actions=[nav2_launch]
    )

    delayed_teleop = TimerAction(
        period=4.0,
        actions=[teleop_node]
    )

    delayed_waypoint = TimerAction(
        period=12.0,
        actions=[waypoint_node]
    )

    return LaunchDescription([
        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        delayed_nav2,
        delayed_teleop,
        delayed_waypoint,
    ])