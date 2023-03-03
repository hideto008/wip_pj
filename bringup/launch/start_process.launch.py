
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    # launch wip model in gazebo
    spawn_wip_model_into_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            ThisLaunchFileDir(), '/spawn_wip_model.launch.py'
        ])
    )

    # start balance controller
    execute_balance_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            ThisLaunchFileDir(), '/execute_balance_control.launch.py'
        ])
    )

    return LaunchDescription([
        execute_balance_controller,
        spawn_wip_model_into_gazebo,
    ])


