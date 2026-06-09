import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from turtlebot3_msgs.srv import Sound


class JoyBuzzer(Node):
    BUTTON_A = 0
    BUTTON_B = 1

    SOUND_BUTTON1 = 4
    SOUND_BUTTON2 = 5

    def __init__(self):
        super().__init__('joy_buzzer')
        self._prev_buttons = []
        self._client = self.create_client(Sound, 'sound')
        self.create_subscription(Joy, 'joy', self._joy_callback, 10)
        self.get_logger().info('joy_buzzer node started')

    def _joy_callback(self, msg: Joy):
        buttons = msg.buttons
        self.get_logger().info(f'joy received: buttons={list(buttons[:4])}')

        a_pressed = self._edge(buttons, self.BUTTON_A)
        b_pressed = self._edge(buttons, self.BUTTON_B)

        if a_pressed:
            self.get_logger().info('A button pressed → calling sound BUTTON1')
            self._call_sound(self.SOUND_BUTTON1)
        if b_pressed:
            self.get_logger().info('B button pressed → calling sound BUTTON2')
            self._call_sound(self.SOUND_BUTTON2)

        self._prev_buttons = list(buttons)

    def _edge(self, buttons, index):
        curr = index < len(buttons) and buttons[index] == 1
        prev = index < len(self._prev_buttons) and self._prev_buttons[index] == 1
        return curr and not prev

    def _call_sound(self, value: int):
        if not self._client.service_is_ready():
            self.get_logger().warn('sound service not available')
            return
        req = Sound.Request()
        req.value = value
        self._client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = JoyBuzzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
