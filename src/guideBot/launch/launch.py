"""
PROMPT TO RUN LAUNCH FILE:

export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch guideBot launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the install paths for the packages I need to reference launch files and assets
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_guide_robot = get_package_share_directory('guideBot')

    # use_sim_time tells all nodes to use the Gazebo simulation clock instead of wall time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Spawn the robot slightly offset from the origin so it starts in an open area of my world
    x_pose = LaunchConfiguration('x_pose', default='3.0')
    y_pose = LaunchConfiguration('y_pose', default='3.0')

    # Path to my custom Gazebo world that I built for the guide robot scenario
    world = os.path.join(pkg_guide_robot, 'worlds', 'guide_house.world')

    # Path to the map I generated with SLAM — Nav2 uses this for localization and path planning
    # HOME PC: map_file = '/home/justin/turtlebot3_ws/src/guideBot/maps/map.yaml'
    # LAB/SEED: map_file = os.path.join(pkg_guide_robot, 'maps', 'map.yaml')
    map_file = os.path.join(pkg_guide_robot, 'maps', 'map.yaml')

    # Gazebo needs to know where to find the TurtleBot3 mesh/model files so they render correctly
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    )

    # Start the Gazebo physics server in headless mode (-s) with my custom world loaded
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world]}.items()
    )

    # Start the Gazebo GUI client (-g) so I can see the simulation visually
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    # Publishes the robot's URDF so all nodes know the physical structure of the TurtleBot3
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawns the TurtleBot3 model into Gazebo at my chosen starting position
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Launch Nav2 with my pre-built map and my custom RViz config that shows the robot,
    # the costmaps, and the navigation goal markers laid out the way I want them
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

    # Open my custom teleop node in its own xterm window so it has a dedicated terminal
    # for reading keyboard input without interfering with the rest of the launch output
    teleop_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run guideBot teleop_game'],
        output='screen',
    )

    # Open the waypoint navigator in its own xterm window for the same reason —
    # it needs raw keyboard access (tty) which requires an independent terminal
    waypoint_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run guideBot waypoint_navigator'],
        output='screen',
    )

    # Open the guide announcer in its own xterm so it is a separately visible node
    # that prints destination and arrival messages from the /guide_status topic
    announcer_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run guideBot guide_announcer'],
        output='screen',
    )

    # Delay Nav2 by 10 seconds to give Gazebo time to fully load the world and
    # spawn the robot before the navigation stack tries to initialize
    delayed_nav2 = TimerAction(
        period=10.0,
        actions=[nav2_launch]
    )

    # Delay teleop by 4 seconds so the robot is spawned before I try to send velocity commands
    delayed_teleop = TimerAction(
        period=4.0,
        actions=[teleop_node]
    )

    # Delay the waypoint navigator by 12 seconds so Nav2 has time to come up before
    # the node tries to connect to the navigate_to_pose action server
    delayed_waypoint = TimerAction(
        period=12.0,
        actions=[waypoint_node]
    )

    # Delay the announcer by the same amount so it is ready before any status messages arrive
    delayed_announcer = TimerAction(
        period=12.0,
        actions=[announcer_node]
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
        delayed_announcer,
    ])
