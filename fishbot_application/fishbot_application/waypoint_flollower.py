from geometry_msgs.msg import PoseStamped # 用于发布机器人位姿节点的消息接口
from nav2_simple_commander.robot_navigator import BasicNavigator # 转换消息接口为能用的类型(这里面有导航的操作)
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator() # 节点对象
    nav.waitUntilNav2Active() # 等待导航激活
    # 目标点数组
    goal_poses = []
    # 对消息接口进行处理
    goal_pose = PoseStamped() # 节点对象
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = nav.get_clock().now().to_msg() # 将 nav 这个节点的当前时间赋值
    goal_pose.pose.position.x = 0.0 # 位置初始化
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0 # 朝向初始化
    goal_poses.append(goal_pose)

    goal_pose1 = PoseStamped() # 节点对象
    goal_pose1.header.frame_id = "map"
    goal_pose1.header.stamp = nav.get_clock().now().to_msg() # 将 nav 这个节点的当前时间赋值
    goal_pose1.pose.position.x = 1.0 # 位置初始化
    goal_pose1.pose.position.y = 1.0
    goal_pose1.pose.orientation.w = 1.0 # 朝向初始化
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped() # 节点对象
    goal_pose2.header.frame_id = "map"
    goal_pose2.header.stamp = nav.get_clock().now().to_msg() # 将 nav 这个节点的当前时间赋值
    goal_pose2.pose.position.x = 0.0 # 位置初始化
    goal_pose2.pose.position.y = 0.0
    goal_pose2.pose.orientation.w = 1.0 # 朝向初始化
    goal_poses.append(goal_pose2)

    nav.followWaypoints(goal_poses) # 路点导航  这个函数中有循环
    while not nav.isTaskComplete(): # 当前的认为是否完成  
        feedback = nav.getFeedback() # 没有完成则获取反馈
        nav.get_logger().info(f'路点编号:{feedback.current_waypoint}')
        # nav.cancelTask()
    result = nav.getResult() 
    nav.get_logger().info(f'导航结果:{result}')
    # rclpy.spin(nav) # 轮询
    # rclpy.shutdown()