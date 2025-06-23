"""
Launch file for yahboomcar ROS 2 robot.

This module defines the launch description for the yahboomcar robot, including configuration
for robot description, state publisher, joint state publisher (with or without GUI), RViz,
driver node, base node, IMU filter, and EKF localization node.
"""

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Generate the launch description for the yahboomcar robot.

    This function sets up the launch configuration for the yahboomcar robot, including
    robot description, state publisher, joint state publisher (with or without GUI),
    RViz, driver node, base node, IMU filter, and EKF localization node.
    """
    yahboomcar_share_path = get_package_share_path("yahboomcar")
    default_model_path = yahboomcar_share_path / "urdf/yahboomcar_R2.urdf.xacro"
    default_rviz_config_path = yahboomcar_share_path / "rviz/yahboomcar.rviz"

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to robot urdf file",
    )

    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(default_rviz_config_path),
        description="Absolute path to rviz config file",
    )

    pub_odom_tf_arg = DeclareLaunchArgument(
        "pub_odom_tf",
        default_value="true",
        description="Whether to publish the tf from the original odom to the base_footprint",
    )

    rviz_enabled_arg = DeclareLaunchArgument(
        name="rviz_enabled",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable RViz visualization",
    )

    use_joint_state_publisher_arg = DeclareLaunchArgument(
        name="use_joint_state_publisher",
        default_value="true",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher (disable when driver publishes joint states)",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="Logging level for all nodes."
    )

    imu_filter_enabled_arg = DeclareLaunchArgument(
        name="imu_filter_enabled",
        default_value="false",
        choices=["true", "false"],
        description="Enable the IMU filter node",
    )

    ekf_enabled_arg = DeclareLaunchArgument(
        name="ekf_enabled",
        default_value="true",
        choices=["true", "false"],
        description="Enable the EKF localization node",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="r2",
        parameters=[{"robot_description": robot_description}],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # Note: joint_state_publisher is disabled by default since Ackermann_Driver_R2 publishes joint states
    # Enable with use_joint_state_publisher:=true if needed
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace="r2",
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher")),
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            LaunchConfiguration("rvizconfig"),
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz_enabled")),
    )

    driver_node = Node(
        package="yahboomcar",
        executable="Ackermann_Driver_R2",
        name="Ackermann_Driver_R2",
        namespace="r2",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    base_node = Node(
        package="yahboomcar",
        executable="base_node_R2",
        name="base_node_r2",
        namespace="r2",
        parameters=[{"pub_odom_tf": LaunchConfiguration("pub_odom_tf")}],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    imu_filter_config = str(
        Path(get_package_share_directory("yahboomcar"))
        / "param"
        / "imu_filter_param.yaml"
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        namespace="r2",
        parameters=[imu_filter_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        condition=IfCondition(LaunchConfiguration("imu_filter_enabled")),
    )
    ekf_config = str(
        Path(get_package_share_directory("yahboomcar")) / "param" / "ekf.yaml"
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
        condition=IfCondition(LaunchConfiguration("ekf_enabled")),
        remappings=[("/odometry/filtered", "/odom")],  # Remap filtered odometry topic
    )

    joystick_param_path = str(
        Path(get_package_share_directory("yahboomcar")) / "param" / "joystick.yaml"
    )

    joystick_joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        parameters=[joystick_param_path],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    joystick_control_node = Node(
        package="yahboomcar",
        executable="joystick",
        name="joystick_node",
        parameters=[joystick_param_path],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # Use consistent and clear argument order in LaunchDescription
    return LaunchDescription(
        [
            gui_arg,
            model_arg,
            rviz_arg,
            pub_odom_tf_arg,
            rviz_enabled_arg,
            use_joint_state_publisher_arg,
            log_level_arg,
            imu_filter_enabled_arg,
            ekf_enabled_arg,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
            driver_node,
            base_node,
            imu_filter_node,
            ekf_node,
            joystick_joy_node,
            joystick_control_node,
        ]
    )
