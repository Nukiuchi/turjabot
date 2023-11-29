import os
 
from ament_index_python.packages import get_package_share_directory
 
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
 
from launch_ros.actions import Node
import launch_ros
 
 
def generate_launch_description():
 
 
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
 
    package_name='turjabot' #<--- CHANGE ME
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')
                ]), 
                launch_arguments={'use_sim_time': 'false'}.items()
    )
 
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
 
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters = [
            {'robot_description': robot_description}, 
            os.path.join(get_package_share_directory('turjabot'), 'config', 'my_controllers.yaml')
        ],
    )
 
    delayed_controller_manager = TimerAction(period=15.0,actions=[controller_manager])
 
    # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo_params_path = os.path.join(
    #               get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # gazebo = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([os.path.join(
    #             get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #             launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
    #      )
 
    # # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', package_name],
    #                     output='screen')
 
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["acker_cont"],
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )
    
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_broad'],
        output='screen'
    )
    load_move_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robot_move_controller'],
        output='screen'
    )
    load_camera_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robot_cam_controller'],
        output='screen'
    )
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(get_package_share_directory(package_name), 'config/ekf.yaml'), {'use_sim_time': True}]
    )
    
    delayed_joint_state_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[load_joint_state_controller],
        )
    )
    delayed_move_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[load_move_controller],
        )
    )
    delayed_camera_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[load_camera_controller],
        )
    )
 
    # Launch them all!
    return LaunchDescription([
        rsp,
        #gazebo,
        #spawn_entity,
        #load_joint_state_controller,
        #load_move_controller,
        #load_camera_controller
        delayed_controller_manager,
        delayed_joint_state_controller,
        delayed_move_controller,
        delayed_camera_controller
    ])