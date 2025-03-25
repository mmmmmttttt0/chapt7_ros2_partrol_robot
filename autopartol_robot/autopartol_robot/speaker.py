import rclpy
from rclpy.node import Node
from autopatol_interfaces.srv import SpeechText # 消息接口的服务
import espeakng # 语音合成的库

class Speaker(Node):
    def __init__(self):  
        super().__init__('speaker')
        # 创建一个服务端 (消息接口，服务的名字,服务的回调函数)
        self.speech_service_ = self.create_service(SpeechText,'speech_text',
                self.speech_text_callback)
        self.speaker_ = espeakng.Speaker() # 定义一个语音对象
        self.speaker_ .voice = 'zh'
    
    # 回调函数
    def speech_text_callback(self,request,response):
        self.get_logger().info(f'正在准备朗读{request.text}')
        self.speaker_.say(request.text) # 说
        self.speaker_.wait() # 等待说完
        response.result = True # 响应的结果
        return response # 返回响应
    
def main():
    rclpy.init()
    node = Speaker()
    rclpy.spin(node)
    rclpy.shutdown()