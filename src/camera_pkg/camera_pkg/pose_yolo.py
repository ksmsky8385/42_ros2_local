#!/usr/bin/env python3

import os
import sys
import numpy as np
import cv2
from ultralytics import YOLO

# ROS2 관련 라이브러리
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from turtlebot3_msgs.srv import Sound  # 📢 사운드 서비스 추가

# Wayland/X11 호환성 설정
os.environ["QT_QPA_PLATFORM"] = "xcb"

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        # 1. 파라미터 및 상수 설정
        self.CONFIDENCE_THRESHOLD = 0.6
        self.KEYPOINT_THRESHOLD = 0.5
        
        self.SKELETON_CONNECTIONS = [
            (0, 1), (0, 2), (1, 3), (2, 4),
            (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
            (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
            (5, 11), (6, 12)
        ]
        
        self.COLORS = {
            'skeleton': (0, 255, 255),
            'keypoint': (0, 0, 255),
            'bbox': (0, 255, 0),
            'text': (0, 255, 0),
            'fall_bbox': (0, 0, 255),
            'fall_text': (0, 0, 255),
        }

        self.FALL_HISTORY = []
        self.FALL_HISTORY_SIZE = 5

        # 📢 [추가] 연속 사운드 호출 방지를 위한 타이머 변수
        self.last_sound_time = self.get_clock().now()
        self.SOUND_COOL_DOWN = 2.0  # 한 번 소리 내면 2초 동안은 중복 호출 방지

        # 2. YOLOv8 포즈 모델 로드
        self.get_logger().info("YOLOv8 포즈 추정 모델을 로드 중...")
        self.model = YOLO('yolov8n-pose.pt')

        # 3. ROS2 구독(Subscriber), 발행(Publisher), 클라이언트(Client) 설정
        self.img_sub = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.cmd_pub = self.create_publisher(String, 'robot_command', 10)
        self.img_pub = self.create_publisher(CompressedImage, 'camera/image_pose/compressed', 10)

        # 📢 [추가] 터틀봇 소리 서비스 클라이언트 생성 및 대기
        self.sound_cli = self.create_client(Sound, 'sound')
        while not self.sound_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('터틀봇 소리 서비스(/sound)를 기다리는 중...')
        
        self.sound_req = Sound.Request()

        self.get_logger().info("포즈 추정 노드가 성공적으로 시작되었습니다. 'image_raw' 대기 중...")

    # 📢 [추가] 소리를 제어하는 헬퍼 함수
    def trigger_fall_sound(self):
        """설정한 쿨다운 시간이 지나면 터틀봇에게 경고음(value: 2)을 요청합니다."""
        now = self.get_clock().now()
        elapsed = (now - self.last_sound_time).nanoseconds / 1e9
        
        if elapsed >= self.SOUND_COOL_DOWN:
            self.get_logger().warn("🚨 낙상 감지! 터틀봇 삐뽀 경고음 발송 🚨")
            self.sound_req.value = 2  # 삐빅 경고음
            self.sound_cli.call_async(self.sound_req)
            self.last_sound_time = now

    def detect_fall(self, kpts, kp_conf, box):
        """3가지 조건의 낙상 점수를 합산해 2점 이상이면 낙상으로 판정."""
        score = 0
        reasons = []

        # 조건 1: 바운딩박스 가로/세로 비율
        xmin, ymin, xmax, ymax = box.xyxy[0].tolist()
        w = xmax - xmin
        h = ymax - ymin
        if h < 1:
            return False, "invalid", 0.0

        bbox_ratio = w / h
        if bbox_ratio > 1.2:
            score += 1
            reasons.append(f"bbox={bbox_ratio:.2f}")

        # 조건 2: 척추 각도
        spine_angle = 90.0
        l_sh, r_sh, l_hip, r_hip = 5, 6, 11, 12
        if (kp_conf is not None and
                kp_conf[l_sh] >= self.KEYPOINT_THRESHOLD and
                kp_conf[r_sh] >= self.KEYPOINT_THRESHOLD and
                kp_conf[l_hip] >= self.KEYPOINT_THRESHOLD and
                kp_conf[r_hip] >= self.KEYPOINT_THRESHOLD):

            sh_mid = np.array([(kpts[l_sh][0] + kpts[r_sh][0]) / 2,
                                (kpts[l_sh][1] + kpts[r_sh][1]) / 2])
            hip_mid = np.array([(kpts[l_hip][0] + kpts[r_hip][0]) / 2,
                                 (kpts[l_hip][1] + kpts[r_hip][1]) / 2])
            dx = hip_mid[0] - sh_mid[0]
            dy = hip_mid[1] - sh_mid[1]
            spine_angle = np.degrees(np.arctan2(abs(dy), abs(dx)))

            if spine_angle < 30.0:
                score += 1
                reasons.append(f"angle={spine_angle:.0f}°")

        # 조건 3: 어깨-엉덩이 수직 거리
        if (kp_conf is not None and
                kp_conf[l_sh] >= self.KEYPOINT_THRESHOLD and
                kp_conf[l_hip] >= self.KEYPOINT_THRESHOLD):
            v_dist = abs(kpts[l_hip][1] - kpts[l_sh][1]) / h
            if v_dist < 0.25:
                score += 1
                reasons.append(f"vdist={v_dist:.2f}")

        return score >= 2, ", ".join(reasons) if reasons else "normal", spine_angle

    def update_fall_state(self, is_fall_frame):
        """오탐 방지: 최근 N프레임 중 과반수 이상이면 낙상 확정"""
        self.FALL_HISTORY.append(is_fall_frame)
        if len(self.FALL_HISTORY) > self.FALL_HISTORY_SIZE:
            self.FALL_HISTORY.pop(0)
        return sum(self.FALL_HISTORY) >= 3

    def image_callback(self, msg):
        start_time = self.get_clock().now()
        
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("imdecode 결과가 None")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.model(frame_rgb, verbose=False)[0]
        keypoints = results.keypoints.xy.cpu().numpy()
        kpt_confs = results.keypoints.conf.cpu().numpy() if results.keypoints.conf is not None else None
        boxes = results.boxes

        current_command = "STOP"

        for i, (box, kpts) in enumerate(zip(boxes, keypoints)):
            kp_conf = kpt_confs[i] if kpt_confs is not None else None
            confidence = box.conf[0].item()
            if confidence < self.CONFIDENCE_THRESHOLD:
                continue

            xmin, ymin, xmax, ymax = map(int, box.xyxy[0].tolist())
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), self.COLORS['bbox'], 2)
            
            text = f"Person {confidence * 100:.1f}%"
            cv2.putText(frame, text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.COLORS['text'], 2)

            for j, kp in enumerate(kpts):
                x, y = kp
                conf = kp_conf[j] if kp_conf is not None else 1.0
                if x > 0 and y > 0 and conf >= self.KEYPOINT_THRESHOLD:
                    cv2.circle(frame, (int(x), int(y)), 5, self.COLORS['keypoint'], -1)

            for connection in self.SKELETON_CONNECTIONS:
                start_idx, end_idx = connection
                s_conf = kp_conf[start_idx] if kp_conf is not None else 1.0
                e_conf = kp_conf[end_idx] if kp_conf is not None else 1.0
                if (len(kpts) > start_idx and len(kpts) > end_idx and
                        kpts[start_idx][0] > 0 and kpts[start_idx][1] > 0 and
                        kpts[end_idx][0] > 0 and kpts[end_idx][1] > 0 and
                        s_conf >= self.KEYPOINT_THRESHOLD and e_conf >= self.KEYPOINT_THRESHOLD):
                    start_x, start_y = int(kpts[start_idx][0]), int(kpts[start_idx][1])
                    end_x, end_y = int(kpts[end_idx][0]), int(kpts[end_idx][1])
                    cv2.line(frame, (start_x, start_y), (end_x, end_y), self.COLORS['skeleton'], 2)

            # ── 낙상 감지 ──────────────────────────────────────────────
            is_fall_frame, reason, angle = self.detect_fall(kpts, kp_conf, box)
            is_fall = self.update_fall_state(is_fall_frame)

            if is_fall:
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), self.COLORS['fall_bbox'], 3)

                overlay = frame.copy()
                cv2.rectangle(overlay, (xmin, ymin - 40), (xmax, ymin), (0, 0, 180), -1)
                cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

                cv2.putText(frame, "FALL DETECTED!", (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.COLORS['fall_text'], 2)
                cv2.putText(frame, f"[{reason}]", (xmin, ymax + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 100, 255), 1)

                l_sh, r_sh, l_hip, r_hip = 5, 6, 11, 12
                if (kp_conf is not None and
                        all(kp_conf[j] >= self.KEYPOINT_THRESHOLD for j in [l_sh, r_sh, l_hip, r_hip])):
                    smx = int((kpts[l_sh][0] + kpts[r_sh][0]) / 2)
                    smy = int((kpts[l_sh][1] + kpts[r_sh][1]) / 2)
                    hmx = int((kpts[l_hip][0] + kpts[r_hip][0]) / 2)
                    hmy = int((kpts[l_hip][1] + kpts[r_hip][1]) / 2)
                    cv2.line(frame, (smx, smy), (hmx, hmy), (0, 0, 255), 3)
                    cv2.putText(frame, f"{angle:.0f}deg", (smx + 5, smy - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                current_command = "FALL_ALERT"
                
                # 📢 [추가] 낙상 확정 시 사운드 함수 발동!
                self.trigger_fall_sound()

            else:
                if len(kpts) > 10 and kp_conf is not None:
                    if (kp_conf[5] >= self.KEYPOINT_THRESHOLD and kp_conf[6] >= self.KEYPOINT_THRESHOLD and
                            kp_conf[9] >= self.KEYPOINT_THRESHOLD and kp_conf[10] >= self.KEYPOINT_THRESHOLD):
                        if kpts[9][1] < kpts[5][1] and kpts[10][1] < kpts[6][1]:
                            current_command = "FORWARD"
                        elif kpts[9][1] < kpts[5][1]:
                            current_command = "LEFT"
                        elif kpts[10][1] < kpts[6][1]:
                            current_command = "RIGHT"

        cmd_msg = String()
        cmd_msg.data = current_command
        self.cmd_pub.publish(cmd_msg)

        cmd_color = self.COLORS['fall_text'] if current_command == "FALL_ALERT" else (255, 0, 0)
        cv2.putText(frame, f"CMD: {current_command}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, cmd_color, 2)

        end_time = self.get_clock().now()
        processing_time = (end_time - start_time).nanoseconds / 1e9
        fps_text = f"FPS: {1.0 / processing_time:.2f}" if processing_time > 0 else "FPS: --"
        cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.COLORS['text'], 2)

        try:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            msg.data = buf.tobytes()
            self.img_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"이미지 발행 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드가 사용자에 의해 종료되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()