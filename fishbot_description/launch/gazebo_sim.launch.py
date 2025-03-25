import launch
import launch.event_handlers
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory # 找目录的函数
import os

import launch_ros.parameter_descriptions #用于拼接目录


#固定的函数名字
def generate_launch_description():
    """产生launch描述"""
    
#文件路径
    # 获取功能包的share路径
    urdf_package_path = get_package_share_directory('fishbot_description')
    # 获取默认的xacro路径 拼接
    default_xacro_path = os.path.join(urdf_package_path,'urdf','fishbot/fishbot.urdf.xacro')

    # 获取默认的rviz路径 拼接  后***************
    #default_rviz_config_path = os.path.join(urdf_package_path,'config','display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path,'world1','custom_room.world')

    # 声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_xacro_path),description='加载的模型文件路径'
    )
#通过文件路径，获取内容,并转换成参数值对象，以供传入 robot_state_publisher 
    #用 cat 命令来通过目录获取内容  用替换来获取路径
    substitutions_command_result = launch.substitutions.Command(
        ['xacro ',launch.substitutions.LaunchConfiguration('model')])
    #转换成参数值的对象
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result,value_type=str)
    
    # robot_state_publisher 节点配置
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    #  启动 gazebo 配置
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        #在一个launch文件中启动另一个 launch 文件
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',default_gazebo_world_path),('verbose','ture')]
    )

    # 将sdf加载到gazebo中  (加载机器人)
    acion_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','fishbot']
    )

    # 加载关节状态控制器
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'fishbot_joint_state_broadcaster'],
        output='screen'

    )

    # 加载力控制器
    # action_load_effort_controller = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_effort_controller'], 
    #     output='screen'
    # )

    # 加载两轮差速控制器
    action_load_diff_driver_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_diff_drive_controller --set-state active'.split(' '), # 数组
        output='screen',
    )

    # rviz 
    # action_rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d',default_rviz_config_path]
    # )
    return launch.LaunchDescription([
            #action动作
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        acion_spawn_entity,
        # 一个事件（顺序事件）
        launch.actions.RegisterEventHandler( # 当acion_spawn_entity进程退出后运行action_load_joint_state_controller
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=acion_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),

        # 一个事件（顺序事件）
        launch.actions.RegisterEventHandler( # 当action_load_joint_state_controller程退出后运行action_load_effort_controller
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_driver_controller],
            )
        ),
        # action_rviz_node,

    ])