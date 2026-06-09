import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from turtlebot3_msgs.srv import Sound

KEY_MAP = {
    '1': 0,   # 죠스
    '2': 1,   # 스타워즈
    '3': 2,   # 배터리 부족
    '4': 3,   # 에러
    '5': 4,   # BUTTON1
    '6': 5,   # BUTTON2
    'a': 6,   # 도
    's': 7,   # 레
    'd': 8,   # 미
    'f': 9,   # 파
    'g': 10,  # 솔
    'h': 11,  # 라
    'j': 12,  # 시
    'k': 13,  # 도(높은)
}

NOTE_NAME = {
    0: '죠스', 1: '스타워즈', 2: '배터리부족', 3: '에러',
    4: 'BUTTON1', 5: 'BUTTON2',
    6: '도', 7: '레', 8: '미', 9: '파',
    10: '솔', 11: '라', 12: '시', 13: '도↑',
}


class KeyboardBuzzer(Node):
    def __init__(self):
        super().__init__('keyboard_buzzer')
        self._client = self.create_client(Sound, 'sound')
        self.get_logger().info('keyboard_buzzer 시작 — q로 종료')
        self.get_logger().info('1=죠스 2=스타워즈 3=배터리 4=에러')
        self.get_logger().info('a=도 s=레 d=미 f=파 g=솔 h=라 j=시 k=도↑')

    def call_sound(self, value: int):
        if not self._client.service_is_ready():
            self.get_logger().warn('sound 서비스 없음')
            return
        req = Sound.Request()
        req.value = value
        self._client.call_async(req)
        self.get_logger().info(f'♪ {NOTE_NAME.get(value, value)}')


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardBuzzer()
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == 'q':
                break
            if key in KEY_MAP:
                node.call_sound(KEY_MAP[key])
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
