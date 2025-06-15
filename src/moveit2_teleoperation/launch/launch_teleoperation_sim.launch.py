import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkgShareMoveitConfig = FindPackageShare(package="moveit2_teleoperation").find("moveit2_teleoperation")
    robotSRDFPath = os.path.join(pkgShareMoveitConfig,"config/dual_lbr_iisy3_r760.srdf")

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_config = (
        MoveItConfigsBuilder("iisy", package_name = "moveit2_teleoperation")
        .robot_description(
            file_path="config/dual_lbr_iisy3_r760.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path=robotSRDFPath)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("moveit2_teleoperation"), "config"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit2_teleoperation"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot1_arm_controller", "-c", "/controller_manager"],
    )

    arm2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot2_arm_controller", "-c", "/controller_manager"],
    )

    gripper1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot1_gripper_controller", "-c", "/controller_manager"],
    )

    gripper2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot2_gripper_controller", "-c", "/controller_manager"],
    )

    gesture_to_pose_node = Node(
        package='gesture_to_pose',  # Replace with your package name
        executable='gesture_node',
        name='gesture_to_pose_node',
        output='screen',
    )

    joint_angle_publisher = Node(
        package='moveit2_teleoperation',  # Replace with your package name
        executable='joint_angle_publisher',
        name='joint_angle_publisher',
        output='screen',
    )

    teleoperation_node = Node(
        package='moveit2_teleoperation',  # Replace with your package name
        executable='teleoperator_node',
        name='teleoperation_node',
        output='screen',
        parameters=[{'use_sim_time': True}]  # Set to True if using simulation
    )


    return LaunchDescription(
        [
            ros2_control_hardware_type,
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm1_controller_spawner,
            gripper1_controller_spawner,
            arm2_controller_spawner,
            gripper2_controller_spawner,
            gesture_to_pose_node,
            joint_angle_publisher,
            teleoperation_node,
        ]
    )


