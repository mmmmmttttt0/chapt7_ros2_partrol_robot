from geometry_msgs.msg import PoseStamped # 用于发布机器人位姿节点的消息接口
from nav2_simple_commander.robot_navigator import BasicNavigator # 转换消息接口为能用的类型(这里面有导航的操作)
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator() # 节点对象
    nav.waitUntilNav2Active() # 等待导航激活
    # 对消息接口进行处理
    goal_pose = PoseStamped() # 节点对象
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = nav.get_clock().now().to_msg() # 将 nav 这个节点的当前时间赋值
    goal_pose.pose.position.x = 2.0 # 位置初始化
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0 # 朝向初始化

    nav.goToPose(goal_pose) # 单点导航  这个函数中有循环
    while not nav.isTaskComplete(): # 当前的认为是否完成  
        feedback = nav.getFeedback() # 没有完成则获取反馈
        nav.get_logger().info(f'剩余距离:{feedback.distance_remaining}')
        # nav.cancelTask()
    result = nav.getResult() 
    nav.get_logger().info(f'导航结果:{result}')
    # rclpy.spin(nav) # 轮询
    # rclpy.shutdown()