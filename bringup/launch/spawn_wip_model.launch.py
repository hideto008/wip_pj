import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro
import yaml

def generate_launch_description():

    world_sdf_file = os.path.join(
        get_package_share_directory('model'),'world','modified_ode_engin.sdf')

    # launch gzserver with pause
    launch_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'pause' : 'true',
            'world' : world_sdf_file,
        }.items()
    )

    # launch gzclient
    launch_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'
        ))
    )

    # read tire radius from mode_params.yaml
    tire_radius = 0.0

    model_param_yaml_file = os.path.join(get_package_share_directory('model'), 'config', 'model_params.yaml')

    with open(model_param_yaml_file, 'r') as params:
        link_params = yaml.safe_load(params)

        tire_radius = link_params['tire_left_link']['radius']
    
    # spawn position
    spawn_base_x = 0.0
    spawn_base_y = 0.0
    spawn_base_z = tire_radius

    # spawn entry point
    # check more argument with 'ros2 run gazebo_ros spawn_engity.py -h'
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'wip',
            '-topic' , 'robot_description', # subsribe parameter
            '-x', str(spawn_base_x),
            '-y', str(spawn_base_y),
            '-z', str(spawn_base_z)
        ]
    )

    # xacro of model file path
    xacro_file = os.path.join(
        get_package_share_directory('model'), 'model', 'model.xacro.urdf')

    # generate xml urdf from xacro
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # spawn robot with urdf
    spawn_robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_xml # publish this string as topic to entry point 
        }]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_effort_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controllers'],
        output='screen'
    )


    return LaunchDescription([
        launch_gzserver,
        launch_gzclient,
        spawn_entity,
        spawn_robot,
        load_joint_state_broadcaster,
        load_effort_controllers
    ])


    

