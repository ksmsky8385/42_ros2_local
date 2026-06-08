import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped


class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',       # target_frame
                'camera_link',     # source_frame
                rclpy.time.Time()) # 가장 최근 시간
            t = trans.transform.translation
            self.get_logger().info(
                f'camera_link in base_link: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}'
            )
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF 조회 실패: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'시간 불일치: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TfListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
