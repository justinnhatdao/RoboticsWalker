"""
BEFORE running, paste this into your terminal (required every session):

  export LIBGL_ALWAYS_SOFTWARE=1
  source /opt/ros/humble/setup.bash
  source ~/turtlebot3_ws/install/setup.bash
  export TURTLEBOT3_MODEL=waffle
  ros2 launch guide_robot guide_robot.launch.py

If you changed any source files, rebuild first:
  cd ~/turtlebot3_ws && colcon build --packages-select guide_robot
  source install/setup.bash
"""

import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_guide_robot = get_package_share_directory('guide_robot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(pkg_guide_robot, 'worlds', 'guide_house.world')

    # ----------------------------------------------------------------
    # 1. GAZEBO — launches your custom guide_house world with TurtleBot3
    # ----------------------------------------------------------------
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

    # ----------------------------------------------------------------
    # 2. CARTOGRAPHER SLAM — builds a live map in RViz as you drive.
    #    Drive around with teleop to scan the whole environment.
    #    Once done, save the map with:
    #      ros2 run nav2_map_server map_saver_cli -f ~/map
    #
    #    PHASE 2: Replace this with Nav2 navigation (see bottom of file).
    # ----------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_cartographer'),
                'launch',
                'cartographer.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'use_rviz': 'False',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/ros/humble/share/turtlebot3_cartographer/rviz/tb3_cartographer.rviz'],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen',
    )

    # ----------------------------------------------------------------
    # 3. TELEOP GAMEPAD/KEYBOARD — lets you drive the robot manually.
    #    This is equivalent to what you ran in Terminal 3.
    #    Used during the SLAM mapping phase to explore the environment.
    #
    #    NOTE: Remove or comment this out once you switch to autonomous
    #    Nav2 navigation — you won't need manual control anymore.
    # ----------------------------------------------------------------
    # teleop_node = ExecuteProcess(
    #     cmd=['xterm', '-e', 'ros2 run guide_robot teleop_game'],
    #     output='screen',
    # )

    # ----------------------------------------------------------------
    # 4. DEAD MAN'S SWITCH NODE — publishes a Boolean ROS2 topic.
    #    True = user is pressing Space (simulating contact with robot).
    #    False = user released Space (robot should stop for safety).
    #    This is one of your original contributions for the instructor.
    #
    #    You will write this node yourself in guide_robot/dead_mans_switch.py
    #    Uncomment this section once you've created that node.
    # ----------------------------------------------------------------
    # dead_mans_switch_node = Node(
    #     package='guide_robot',
    #     executable='dead_mans_switch',
    #     name='dead_mans_switch',
    #     output='screen',
    # )

    # ----------------------------------------------------------------
    # 5. WAYPOINT NAV NODE — your main original contribution.
    #    Reads user keyboard input (room name or number key),
    #    looks up the matching coordinates, sends Nav2 a goal.
    #    Also publishes status messages like "Heading to kitchen".
    #
    #    Uncomment once you've written guide_robot/waypoint_navigator.py
    # ----------------------------------------------------------------
    # waypoint_node = Node(
    #     package='guide_robot',
    #     executable='waypoint_navigator',
    #     name='waypoint_navigator',
    #     output='screen',
    # )

    # ----------------------------------------------------------------
    # TIMING NOTE:
    # Gazebo needs a few seconds to start before SLAM can connect to it.
    # TimerAction delays SLAM by 5 seconds so Gazebo is ready first.
    # ----------------------------------------------------------------
    delayed_slam = TimerAction(
        period=3.0,
        actions=[slam_launch]
    )

    delayed_rviz = TimerAction(
        period=4.0,
        actions=[rviz_node]
    )

    wanderer_node = Node(
        package='guide_robot',
        executable='wanderer',
        name='wanderer',
        output='screen',
    )

    delayed_wanderer = TimerAction(
        period=4.0,
        actions=[wanderer_node]
    )

    # ----------------------------------------------------------------
    # RETURN — this list is what actually gets launched.
    # Add/uncomment items above and include them here as you build.
    # ----------------------------------------------------------------
    return LaunchDescription([
        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        delayed_slam,
        delayed_rviz,
        delayed_wanderer,

        # Uncomment these once your nodes exist:
        # dead_mans_switch_node,
        # waypoint_node,
    ])


# ================================================================
# PHASE 2 — Once you have a saved map, replace slam_launch above
# with this Nav2 navigation launch instead:
#
#   nav2_launch = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource(
#           os.path.join(
#               get_package_share_directory('turtlebot3_navigation2'),
#               'launch',
#               'navigation2.launch.py'
#           )
#       ),
#       launch_arguments={
#           'use_sim_time': 'True',
#           'map': '/home/seed/map.yaml',   # path to your saved map
#       }.items()
#   )
#
# Then swap delayed_slam for a delayed_nav2 in the LaunchDescription.
# ================================================================