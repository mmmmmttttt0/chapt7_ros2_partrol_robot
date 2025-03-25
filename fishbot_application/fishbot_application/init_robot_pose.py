from geometry_msgs.msg import PoseStamped # 用于发布机器人位姿节点的消息接口
from nav2_simple_commander.robot_navigator import BasicNavigator # 转换消息接口为能用的类型(这里面有导航的操作)
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator() # 节点对象
    # 对消息接口进行处理
    init_pose = PoseStamped() # 节点对象
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = nav.get_clock().now().to_msg() # 将 nav 这个节点的当前时间赋值
    init_pose.pose.position.x = 0.0 # 位置初始化
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.w = 1.0 # 朝向初始化

    nav.setInitialPose(init_pose) # 对导航节点进行初始化
    nav.waitUntilNav2Active() # 等待导航激活
    rclpy.spin(nav) # 轮询
    rclpy.shutdown()