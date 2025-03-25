import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory # 用于拼接目录
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 两个动作 （多个目标点，播放服务端）
def generate_launch_description():
    # 获取与拼接默认路径
    # 获取功能包的名字
    autopartol_robot_path = get_package_share_directory('autopartol_robot')
    default_partol_config_path = autopartol_robot_path + '/config/partol_config.yaml'

    # 动作
    action_partol_node = launch_ros.actions.Node(
        package='autopartol_robot',
        executable='partol_node',
        output='screen',
        parameters=[default_partol_config_path],
        
    )
    action_speaker_node = launch_ros.actions.Node(
        package='autopartol_robot',
        executable='speaker',
        output='screen'
    )

    return launch.LaunchDescription([
        action_partol_node,
        action_speaker_node,
    ])
