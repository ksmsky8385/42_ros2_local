import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.declare_parameter('vel_x', 1.0)
        self.declare_parameter('angle_z', math.pi / 2)
        self.vel_x = self.get_parameter('vel_x').value
        self.angle_z = self.get_parameter('angle_z').value
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.step = 0                              # 0~7: 전진·회전 교대 (4변 × 2동작)
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        if self.step % 2 == 0:    # 짝수 스텝: 1초 전진
            msg.linear.x = self.vel_x
        else:                      # 홀수 스텝: 1초 동안 90° 회전
            msg.angular.z = self.angle_z
        self.pub.publish(msg)
        self.step += 1
        if self.step >= 8:         # 4변 완료 → 처음부터 반복
            self.step = 0
            self.get_logger().info('Square complete! Restarting...')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TurtleSquare())