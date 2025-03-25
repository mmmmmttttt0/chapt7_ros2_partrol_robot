from geometry_msgs.msg import PoseStamped,Pose # 用于发布机器人位姿节点的消息接口
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult # 导航，对结果进行判断
import rclpy

from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener,Buffer# 坐标监听器
from tf_transformations import euler_from_quaternion,quaternion_from_euler # 四元数转欧拉角 欧拉转四元数
import math # 用里面的角度转弧度

from sensor_msgs.msg import Image # 相机图像的消息接口
from cv_bridge import CvBridge # 相机格式转换的库
import cv2 # 保存图像

from autopatol_interfaces.srv import SpeechText # 消息接口的服务 语音

# 实现所有功能的主类
class PartolNode(BasicNavigator):
    # 节点的名字
    def __init__(self, node_name='partol_node'):
        super().__init__(node_name)
        # 声明相关参数 (x,y,w(朝向))
        self.declare_parameter('initial_point',[0.0,0.0,0.0])
        self.declare_parameter('target_point',[0.0,0.0,0.0,1.0,1.0,1.57]) # 1.57是90度
        self.declare_parameter('img_save_path','') # 用户可以自定义图像保存路径
        # 获取初始点和目标点的值 (设置一个状态变量)
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_point = self.get_parameter('target_point').value
        self.img_save_path = self.get_parameter('img_save_path').value

        self.buffer_ = Buffer() # 创建一个缓存区1
        #将监听到的数据放入缓存区2
        self.listener_= TransformListener(self.buffer_ ,self) # 创建坐标监听器

        # 创建语音客户端 （消息接口类型，消息接口名字）
        self.speech_client_ = self.create_client(SpeechText,'speech_text')

        # 创建cv桥
        self.cv_bridge_ = CvBridge()
        # 创建一个成员变量，实时的保存图像
        self.latest_img_ = None
        # 创建一个订阅者 (消息接口，话题类型，回调函数，1个数据)
        self.img_sub_ = self.create_subscription(Image,'/camera_sensor/image_raw',self.img_callback,1)



        try:#尝试
        #查询最新的 base_link 与 bottle_link 的关系；超时时间为1.0s
        # result 为查询的结果
            result = self.buffer_.lookup_transform('map','base_footprint',
                rclpy.time.Time(seconds=0.0),rclpy.time.Duration(seconds=1.0))
            tranfrom = result.transform # 提取结果
            self.get_logger().info(f'平移:{tranfrom.translation}')
            self.get_logger().info(f'旋转:{tranfrom.rotation}')
            # 四元数转欧拉角
            rotation_euler = euler_from_quaternion(
                [tranfrom.rotation.x,
                tranfrom.rotation.y,
                tranfrom.rotation.z,
                tranfrom.rotation.w,]
            )
            self.get_logger().info(f'旋转RPY:{rotation_euler}')
        
        except Exception as e:#捕获异常
            self.get_logger().warn(f'获取坐标变换异常失败原因：{str(e)}')


    def img_callback(self,msg):
        # 将最新的数据一直往里面放
        self.latest_img_ = msg

    # 保存图像 将 latest_img_ 中的数据保存到指定的目录中去
    def record_img(self):
        if self.latest_img_ is not None:
            # 获取当前文件的位置
            pose = self.get_current_pose()
            # ros2 类型将图像转换为 opencv 格式
            cv_image = self.cv_bridge_.imgmsg_to_cv2(self.latest_img_)
            # 保存图像 路径 名字 图像
            cv2.imwrite(
                f'{self.img_save_path}img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png',
                cv_image
            )



    def get_pose_by_xyyaw(self,x,y,yaw):
        """
        return PoseStamped (消息接口)对象，也就是把值搞到消息接口中
        """
        # 对消息接口进行处理
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        # 返回顺序 xyzw ,有的是 wxyz
        quat = quaternion_from_euler(0,0,yaw) # 欧拉转四元数
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    def init_robot_pose(self):
        """
        初始化机器人的位姿
        """
        # 获取初始化的点
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0],self.initial_point_[1],self.initial_point_[2])
        
        self.setInitialPose(init_pose) # 对导航节点进行初始化
        self.waitUntilNav2Active() # 等待导航激活

    def get_target_points(self):
        """
        通过参数值获取目标点的集合
        """
        # 目标点集合
        points = []
        # 获取目标点
        self.target_points_ = self.get_parameter('target_point').value
        # 8/3 = 2  依次取出的每一个元素
        for index in range(int(len(self.target_points_)/3)):
            # 对目标点进行提取
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            # 将提取的点放进目标点数组中
            points.append([x,y,yaw])
            # 打印获取到的目标点
            self.get_logger().info(f"获取到目标点{index}->{x},{y},{yaw}")
        # 返回目标点集合
        return points
            


    def nav_to_pose(self,target_point):
        """
        导航到目标点
        """
        self.goToPose(target_point) # 单点导航  这个函数中有循环
        while not self.isTaskComplete(): # 当前的认为是否完成  
            feedback = self.getFeedback() # 没有完成则获取反馈
            self.get_logger().info(f'剩余距离:{feedback.distance_remaining}')
            # nav.cancelTask()
        # 获取结果
        result = self.getResult() 
        self.get_logger().info(f'导航结果:{result}')

    def get_current_pose(self):
        """
        获取机器人当前位姿
        """
        while rclpy.ok(): # 死循环 获取失败后可再次尝试获取当前位姿
            try:#尝试
            #查询最新的 base_link 与 bottle_link 的关系；超时时间为1.0s
            # result 为查询的结果
                result = self.buffer_.lookup_transform('map','base_footprint',
                    rclpy.time.Time(seconds=0.0),rclpy.time.Duration(seconds=1.0))
                tranfrom = result.transform # 提取结果
                self.get_logger().info(f'平移:{tranfrom.translation}')
                # self.get_logger().info(f'旋转:{tranfrom.rotation}')
                # # 四元数转欧拉角
                # rotation_euler = euler_from_quaternion(
                #     [tranfrom.rotation.x,
                #     tranfrom.rotation.y,
                #     tranfrom.rotation.z,
                #     tranfrom.rotation.w,]
                # )
                # self.get_logger().info(f'旋转RPY:{rotation_euler}')
                return tranfrom
            except Exception as e:#捕获异常
                self.get_logger().warn(f'获取坐标变换异常失败原因：{str(e)}')
    
    def speech_text(self,text):
        """
        调用服务合成语音
        """
        # 检测服务端是否上线 (timeout 1s) /循环（没有上线就一直等）
        while not self.speech_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音合成服务未上线,等待中...')
        # 消息接口
        request = SpeechText.Request()
        request.text = text
        # 发送请求，并获取结果
        future = self.speech_client_.call_async(request)
        # 等待服务端处理完成
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None: # 如果有结果
            response = future.result() # 提取结果
            if response.result == True: # 结果是Ture
                self.get_logger().info(f'语音合成成功{text}')
            else:
                self.get_logger().warn(f'语音合成失败{text}')
        else:
            self.get_logger().warn(f'语音响应失败')


        

def main():
    rclpy.init()
    partol = PartolNode() # 节点对象
    # # 为了创建yaml文件
    # rclpy.spin(partol)
    # 初始化机器人的位置
    partol.speech_text('正在准备初始化位置')
    partol.init_robot_pose()
    partol.speech_text('位置初始化完成')


    while rclpy.ok():
        # 获取目标点
        points = partol.get_target_points()
        # 循环导航
        for point in points:
            x,y,yaw = point[0],point[1],point[2]
            # 将目标点放入集合
            target_pose = partol.get_pose_by_xyyaw(x,y,yaw)
            partol.speech_text(f'正在准备前往{x},{y}目标点')
            # 进行单点导航
            partol.nav_to_pose(target_pose)
            # 保存图像
            partol.speech_text(f'已经到达目标点{x},{y}，正在准备记录图像')
            partol.record_img()
            partol.speech_text(f'图像记录完成')




    # rclpy.spin(nav) # 轮询
    rclpy.shutdown()