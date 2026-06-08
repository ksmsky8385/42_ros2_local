import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddTwoInts

class AddServiceServer(Node):
    def __init__(self):
        super().__init__('add_service_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_tow_int', self.set_add_callback)
        self.get_logger().info('ADD Two int 서비스 서버가 시작되었습니다.')

    def set_add_callback(self, request, response):
        response.sum = request.a + request.b
        return response  # 반드시 response 반환!

def main():
    rclpy.init()
    node = AddServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

