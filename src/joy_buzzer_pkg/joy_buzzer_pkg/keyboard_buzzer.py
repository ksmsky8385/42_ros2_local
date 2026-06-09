import sys
import os
import tty
import termios
import threading
import time
import rclpy
from rclpy.node import Node
from turtlebot3_msgs.srv import Sound

os.environ['RCUTILS_LOGGING_SEVERITY_THRESHOLD'] = '40'

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

MIDI_MAP = {
    'a': 100,  # 도
    'w': 101,  # 도#
    's': 102,  # 레
    'e': 103,  # 레#
    'd': 104,  # 미
    'f': 105,  # 파
    't': 106,  # 파#
    'g': 107,  # 솔
    'y': 108,  # 솔#
    'h': 109,  # 라
    'u': 110,  # 라#
    'j': 111,  # 시
    'k': 112,  # 도(높은)
}

STOP_VALUE = 255

NOTE_NAME = {
    0: '죠스', 1: '스타워즈', 2: '배터리부족', 3: '에러',
    4: 'BUTTON1', 5: 'BUTTON2',
    6: '도', 7: '레', 8: '미', 9: '파',
    10: '솔', 11: '라', 12: '시', 13: '도↑',
    100: '도', 101: '도#', 102: '레', 103: '레#',
    104: '미', 105: '파', 106: '파#', 107: '솔',
    108: '솔#', 109: '라', 110: '라#', 111: '시', 112: '도↑',
}


class KeyboardBuzzer(Node):
    def __init__(self):
        super().__init__('keyboard_buzzer')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        self._client = self.create_client(Sound, 'sound')
        self._stop_timer = None
        self._current_key = None
        self._key_press_time = 0.0

    def call_sound(self, value: int):
        if not self._client.service_is_ready():
            return
        req = Sound.Request()
        req.value = value
        self._client.call_async(req)

    def play_midi(self, key: str):
        value = MIDI_MAP[key]
        now = time.monotonic()
        if self._current_key != key:
            self._current_key = key
            self._key_press_time = now
            self.call_sound(value)
            print(f'♪ {NOTE_NAME.get(value)} ON ')
        # 반복 시작 전(~500ms)엔 긴 타이머, 이후엔 짧은 타이머
        elapsed = now - self._key_press_time
        timeout = 0.15 if elapsed > 0.4 else 0.7
        self._reset_stop_timer(timeout)

    def _reset_stop_timer(self, timeout: float = 0.15):
        if self._stop_timer:
            self._stop_timer.cancel()
        self._stop_timer = threading.Timer(timeout, self._stop_sound)
        self._stop_timer.start()

    def _stop_sound(self):
        if self._current_key:
            print(f'  {NOTE_NAME.get(MIDI_MAP.get(self._current_key, 0))} OFF')
        self._current_key = None
        self.call_sound(STOP_VALUE)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def run_midi_mode(node, settings):
    print("MIDI 모드 — 누르는 동안 소리 유지, q 로 종료")
    print("  흰건반: a s d f g h j k")
    print("  검은건반: w e t y u\n")
    while rclpy.ok():
        key = get_key(settings)
        if key == 'q':
            break
        if key in MIDI_MAP:
            node.play_midi(key)


def run_normal_mode(node, settings):
    print("일반 모드 — 1~6: 효과음, a~k: 도레미, q: 종료\n")
    while rclpy.ok():
        key = get_key(settings)
        if key == 'q':
            break
        if key in KEY_MAP:
            value = KEY_MAP[key]
            node.call_sound(value)
            print(f'♪ {NOTE_NAME.get(value)}')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardBuzzer()
    settings = termios.tcgetattr(sys.stdin)

    print("모드 선택 — [1] 일반모드  [2] MIDI모드(누르는동안 유지): ", end='', flush=True)
    mode = sys.stdin.readline().strip()

    try:
        if mode == '2':
            run_midi_mode(node, settings)
        else:
            run_normal_mode(node, settings)
    finally:
        if node._stop_timer:
            node._stop_timer.cancel()
        node.call_sound(STOP_VALUE)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    node.destroy_node()
    rclpy.shutdown()
