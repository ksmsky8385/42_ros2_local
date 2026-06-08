import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddTwoInts

class AddServiceClient(Node):
    def __init__(self):
        super().__init__('add_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_tow_int')
        # 서버가 준비될 때까지 1초마다 확인하며 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서버 대기 중...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)       # 비동기 요청
        rclpy.spin_until_future_complete(self, self.future)  # 블로킹 대기
        return self.future.result()

def main():
    rclpy.init()
    client = AddServiceClient()
    response = client.send_request(10, 20)
    client.get_logger().info(
        f'결과: {response.sum}')
    client.destroy_node()
    rclpy.shutdown()

