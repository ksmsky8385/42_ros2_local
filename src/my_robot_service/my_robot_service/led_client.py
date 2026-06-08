import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import LedControl

class LedServiceClient(Node):
    def __init__(self):
        super().__init__('led_service_client')
        self.cli = self.create_client(LedControl, 'set_led')
        # 서버가 준비될 때까지 1초마다 확인하며 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서버 대기 중...')
        self.req = LedControl.Request()

    def send_request(self, state):
        self.req.state = state
        self.future = self.cli.call_async(self.req)       # 비동기 요청
        rclpy.spin_until_future_complete(self, self.future)  # 블로킹 대기
        return self.future.result()

def main():
    rclpy.init()
    client = LedServiceClient()
    response = client.send_request(True)
    client.get_logger().info(
        f'결과: {response.success}, 메시지: {response.message}')
    client.destroy_node()
    rclpy.shutdown()

