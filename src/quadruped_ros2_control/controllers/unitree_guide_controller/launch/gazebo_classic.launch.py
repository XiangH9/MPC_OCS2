import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations["pkg_description"]
    init_height = context.launch_configurations["height"]
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    robot_description = xacro.process_file(
        xacro_file,
        mappings={"GAZEBO": "true", "CLASSIC": "true"},
    ).toxml()

    rviz_config_file = os.path.join(
        get_package_share_directory(package_description),
        "config",
        "visualize_urdf.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_ocs2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(context.launch_configurations["rviz"]),
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])]
        ),
        launch_arguments={
            "verbose": "false",
            "pause": context.launch_configurations["pause"],
        }.items(),
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(context.launch_configurations["gui"]),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", init_height,
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "publish_frequency": 20.0,
                "use_tf_static": True,
                "robot_description": robot_description,
                "ignore_timestamp": True,
            }
        ],
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "leg_pd_controller",
            "--controller-manager", "/controller_manager",
        ],
    )

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "unitree_guide_controller",
            "--controller-manager", "/controller_manager",
        ],
    )

    unpause_gazebo = ExecuteProcess(
        cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
    )

    return [
        rviz,
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[unpause_gazebo, leg_pd_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[imu_sensor_broadcaster, joint_state_publisher, unitree_guide_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        "pkg_description",
        default_value="go1_description",
        description="package for robot description",
    )

    height = DeclareLaunchArgument(
        "height",
        default_value="0.43",
        description="Init height in simulation",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Whether to start rviz2",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Whether to start gazebo client",
    )

    pause_arg = DeclareLaunchArgument(
        "pause",
        default_value="true",
        description="Whether to start gazebo paused",
    )

    return LaunchDescription([
        pkg_description,
        height,
        rviz_arg,
        gui_arg,
        pause_arg,
        OpaqueFunction(function=launch_setup),
    ])
