from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_path = get_package_share_directory('my_robot_descripction')
    model_path = os.path.join(description_path, 'urdf', 'Robot_1.urdf')
    rviz_conf_path = os.path.join(description_path, 'rviz', 'rviz_config.rviz')

    with open(model_path, 'r') as f:
        robot_description_content = f.read()

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    controller_manager_node = Node(
        package='robot1_controler',
        executable='controller_manager',
        output='screen'
    )

    manipulator_controller_node = Node(
        package='robot1_controler',
        executable='manipulator_controller',
        output='screen'
    )

    hardware_interface_node = Node(
        package='robot1_controler',
        executable='hardware_interface',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_conf_path],
        output='screen'
    )

    """joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher'
    )"""


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    return LaunchDescription([
        controller_manager_node,
        manipulator_controller_node,
        hardware_interface_node,
        rviz_node,
        robot_state_publisher_node,
    ])