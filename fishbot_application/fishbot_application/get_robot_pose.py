import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener,Buffer# 坐标监听器
from tf_transformations import euler_from_quaternion # 四元数转欧拉角函数
import math # 用里面的角度转弧度

class TFBroadcaster(Node):#节点类
    def __init__(self):#自己初始化
        super().__init__('tf_broadcaster')#父类初始化
        self.buffer_ = Buffer() # 创建一个缓存区1
        #将监听到的数据放入缓存区2
        self.listener_= TransformListener(self.buffer_ ,self) # 创建坐标监听器
        #定时调用1.0s
        self.timer_ = self.create_timer(1.0,self.get_transfrom)

    def get_transfrom(self):#发布静态坐标发布器的函数
        """
        实时查询坐标关系
        """
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

def main():
    rclpy.init()#初始一下上下文
    node = TFBroadcaster()#创建一个节点（也是一个对象）
    rclpy.spin(node)
    rclpy.shutdown()

