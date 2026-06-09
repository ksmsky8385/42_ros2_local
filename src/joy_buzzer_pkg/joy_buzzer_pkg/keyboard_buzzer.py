import sys
import os
import tty
import termios
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

    def call_sound(self, value: int):
        if not self._client.service_is_ready():
            return
        req = Sound.Request()
        req.value = value
        self._client.call_async(req)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def find_keyboard():
    try:
        from evdev import InputDevice, ecodes, list_devices
    except ImportError:
        return None
    for path in list_devices():
        try:
            dev = InputDevice(path)
            caps = dev.capabilities()
            if ecodes.EV_KEY in caps:
                keys = caps[ecodes.EV_KEY]
                if ecodes.KEY_A in keys and ecodes.KEY_Z in keys:
                    return dev
        except Exception:
            pass
    return None


def run_midi_mode(node):
    try:
        from evdev import ecodes
    except ImportError:
        print("evdev 미설치 — pip install evdev")
        return

    dev = find_keyboard()
    if dev is None:
        print("키보드 장치를 찾지 못했습니다")
        return

    EVDEV_MIDI = {
        ecodes.KEY_A: 100, ecodes.KEY_W: 101, ecodes.KEY_S: 102,
        ecodes.KEY_E: 103, ecodes.KEY_D: 104, ecodes.KEY_F: 105,
        ecodes.KEY_T: 106, ecodes.KEY_G: 107, ecodes.KEY_Y: 108,
        ecodes.KEY_H: 109, ecodes.KEY_U: 110, ecodes.KEY_J: 111,
        ecodes.KEY_K: 112,
    }

    print(f"MIDI 모드 [{dev.name}] — 누르는 동안 소리 유지, q 종료")
    print("  흰건반: a s d f g h j k")
    print("  검은건반: w e t y u\n")

    dev.grab()
    try:
        for event in dev.read_loop():
            if event.type != ecodes.EV_KEY:
                continue
            if event.code == ecodes.KEY_Q and event.value == 1:
                break
            if event.code not in EVDEV_MIDI:
                continue
            if event.value == 1:  # key down
                value = EVDEV_MIDI[event.code]
                node.call_sound(value)
                print(f'♪ {NOTE_NAME.get(value)} ON')
            elif event.value == 0:  # key up
                node.call_sound(STOP_VALUE)
                print(f'  OFF')
    finally:
        try:
            dev.ungrab()
        except Exception:
            pass
        node.call_sound(STOP_VALUE)


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
            run_midi_mode(node)
        else:
            run_normal_mode(node, settings)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    node.destroy_node()
    rclpy.shutdown()
