from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    balance_controller_parameters = os.path.join(
        get_package_share_directory('control'), 'config', 'controller_params.yaml'
    )

    start_balance_contro_node = Node(
        package='control',
        executable='balance_controller',
        name='wip_balance_controller',
        parameters=[balance_controller_parameters],
        output='screen'
    )

    return LaunchDescription(
        [
        start_balance_contro_node
        ]
    )