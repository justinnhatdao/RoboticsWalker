import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── 1. TurtleBot3 in Gazebo (Ignition Fortress) ──────────────────────────
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    # ── 2. Teleop keyboard node ───────────────────────────────────────────────
    # Publishes velocity commands to /cmd_vel based on keyboard input.
    # Launched in a new xterm window so it can receive keystrokes.
    teleop_node = ExecuteProcess(
        cmd=[
            'xterm', '-e',
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            'source ~/turtlebot3_ws/install/setup.bash && '
            'export TURTLEBOT3_MODEL=waffle && '
            'ros2 run turtlebot3_teleop teleop_keyboard'
        ],
        output='screen'
    )

    # ── 3. Pose node (myPose.py) ──────────────────────────────────────────────
    # Subscribes to /odom and logs the robot's current (x, y, heading).
    pose_node = Node(
        package='walkerproj',
        executable='my_pose_node',
        name='my_pose_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        turtlebot3_gazebo,
        teleop_node,
        pose_node,
    ])