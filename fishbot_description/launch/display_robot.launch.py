import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory # 找目录的函数
import os

import launch_ros.parameter_descriptions #用于拼接目录


#固定的函数名字
def generate_launch_description():
    """产生launch描述"""
    
#文件路径
    # 获取功能包的目录
    urdf_package_path = get_package_share_directory('fishbot_description')
    # 获取默认的urdf路径 拼接
    default_urdf_path = os.path.join(urdf_package_path,'urdf','first_robot.urdf')

    # 获取默认的rviz路径 拼接  后***************
    default_rviz_config_path = os.path.join(urdf_package_path,'config','display_robot_model.rviz')

    # 声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_urdf_path),description='加载的模型文件路径'
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
    # joint_state_publisher 节点配置
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # rviz 
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',default_rviz_config_path]
    )
    return launch.LaunchDescription([
            #action动作
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node,

    ])