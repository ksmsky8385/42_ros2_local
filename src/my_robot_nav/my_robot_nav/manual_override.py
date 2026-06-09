import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class ManualOverride(Node):
    def __init__(self):
        super().__init__('manual_override')

        self.declare_parameter('toggle_button', 9)
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_angular_yaw', 0)
        self.declare_parameter('scale_linear_x', 0.20)
        self.declare_parameter('scale_angular_yaw', 2.0)
        self.declare_parameter('deadman_timeout', 0.2)

        self.toggle_button = self.get_parameter('toggle_button').value
        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_angular_yaw = self.get_parameter('axis_angular_yaw').value
        self.scale_linear_x = self.get_parameter('scale_linear_x').value
        self.scale_angular_yaw = self.get_parameter('scale_angular_yaw').value
        self.deadman_timeout = self.get_parameter('deadman_timeout').value

        self.manual_mode = False
        self.prev_toggle_pressed = False
        self.last_joy_time = self.get_clock().now()
        self.latest_twist = Twist()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.lock_pub = self.create_publisher(Bool, '/manual_override_lock', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.publish_lock()

    def joy_callback(self, msg):
        toggle_pressed = (
            0 <= self.toggle_button < len(msg.buttons)
            and msg.buttons[self.toggle_button] == 1
        )

        if toggle_pressed and not self.prev_toggle_pressed:
            self.manual_mode = not self.manual_mode
            self.publish_lock()
            mode = 'manual' if self.manual_mode else 'autonomous'
            self.get_logger().info(f'mode={mode}')

            if not self.manual_mode:
                self.cmd_pub.publish(Twist())

        self.prev_toggle_pressed = toggle_pressed
        self.last_joy_time = self.get_clock().now()

        twist = Twist()
        if self.axis_linear_x < len(msg.axes):
            twist.linear.x = msg.axes[self.axis_linear_x] * self.scale_linear_x
        if self.axis_angular_yaw < len(msg.axes):
            twist.angular.z = msg.axes[self.axis_angular_yaw] * self.scale_angular_yaw
        self.latest_twist = twist

    def timer_callback(self):
        self.publish_lock()

        if not self.manual_mode:
            return

        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > self.deadman_timeout:
            self.cmd_pub.publish(Twist())
            return

        self.cmd_pub.publish(self.latest_twist)

    def publish_lock(self):
        lock_msg = Bool()
        lock_msg.data = self.manual_mode
        self.lock_pub.publish(lock_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManualOverride()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.manual_mode = False
            node.publish_lock()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
