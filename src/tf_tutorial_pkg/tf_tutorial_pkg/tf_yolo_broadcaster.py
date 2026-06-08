import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import ObjectDetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TfYoloBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_yolo_broadcaster')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(
            ObjectDetectionArray, '/image_yolo', self.callback, 10)

    def callback(self, msg):
        now = self.get_clock().now().to_msg()
        for i, det in enumerate(msg.detections):
            if det.class_name != 'person':
                continue
            # bbox 중심을 3D 공간에 배치 (간단 가정)
            cx, cy, w, h = det.bbox
            # 픽셀 좌표를 임의의 3D 좌표로 매핑 (z=1m 고정)
            x = (cx - 160) / 320.0  # 정규화
            y = (cy - 120) / 240.0
            z = 1.0

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'camera_link'
            t.child_frame_id = f'object_{det.class_name}_{0}'
            t.transform.translation.x = z
            t.transform.translation.y = -x
            t.transform.translation.z = -y
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)
            # self.get_logger().info(f'TF 발행: {t.child_frame_id} at ({x:.2f}, {y:.2f}, {z:.2f})')
            break


def main(args=None):
    rclpy.init(args=args)
    node = TfYoloBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
