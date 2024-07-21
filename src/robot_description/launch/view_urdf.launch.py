import os

import xacro
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def robot_state_publisher_spawner(context: LaunchContext, arm_id, load_gripper):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_xacro_filepath = os.path.join(
        "/home/vboxuser/ros2_ws/src/robot_description",
        "urdf",
        "fr3.urdf.xacro"
    )
    robot_description = xacro.process_file(
        franka_xacro_filepath
    ).toprettyxml(indent="  ")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():
    load_gripper_parameter_name = "load_gripper"
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)

    ee_id_parameter_name = "ee_id"
    ee_id = LaunchConfiguration(ee_id_parameter_name)

    arm_id_parameter_name = "arm_id"
    arm_id = LaunchConfiguration(arm_id_parameter_name)

    robot_state_publisher_spawner_opaque_function = OpaqueFunction(
        function=robot_state_publisher_spawner, args=[arm_id, load_gripper]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value="false",
                description="Use end-effector if true. Default value is franka hand. "
                "Robot is loaded without end-effector otherwise",
            ),

            DeclareLaunchArgument(
                arm_id_parameter_name,
                default_value="fr3",
                description="ID of the type of arm used. Supporter values: "
                "fer, fr3, fp3",
            ),
            robot_state_publisher_spawner_opaque_function,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", "/home/vboxuser/franka_ros2_ws/src/franka_description/rviz/visualize_franka.rviz"],
            ),
        ]
    )