import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        # Publisher와 동일한 토픽 이름('hello_topic')으로 구독
        self.subscription = self.create_subscription(
            String, 'hello_topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        # 메시지를 받았을 때 실행되는 함수
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
