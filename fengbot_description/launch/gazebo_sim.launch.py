import launch
import launch.event_handlers
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
import launch_ros.parameter_descriptions

def generate_launch_description():

    #获取功能包的share路径
    urdf_package_path = get_package_share_directory('fengbot_description')

    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'fengbot/fengbot.urdf.xacro')
    #default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'feng_home.world')

    #声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=str(default_xacro_path), 
        description='加载的模型文件路径'
    )


    #通过文件路径，获取内容，并转换成参数值对象，以供传入 robot_state_publisher

    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    

    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)


    action_robot_state_publisher = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher', 
    parameters=[{'robot_description': robot_description_value}]
    )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(
        [get_package_share_directory('gazebo_ros'), '/launch','/gazebo.launch.py']
    ),
    
    launch_arguments=[
    ('world', default_gazebo_world_path),
    ('verbose', 'true'),
    ('debug', 'true')]
    )


    action_spawn_entity =launch_ros.actions.Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','fengbot']
    )


    #   加载joint_state_broadcaster
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fengbot_joint_state_broadcaster", "-c", "/controller_manager"],
    )


       
   


    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fengbot_diff_drive_controller'],
    output='screen'

)
          
#  在实体生成后加载控制器
    spawn_entity_event = RegisterEventHandler(
         event_handler =OnProcessExit(
             target_action=action_spawn_entity,
             on_exit=[joint_state_broadcaster_spawner],
         )
     )


    return launch.LaunchDescription([
    action_declare_arg_mode_path, 
    action_robot_state_publisher, 
    action_launch_gazebo,
    action_spawn_entity,
    spawn_entity_event,
    launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[action_load_diff_drive_controller],
        )
    ),

  
    ]) 