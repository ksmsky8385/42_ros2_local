#!/usr/bin/env python3
"""
TurtleBot3 직진/후진 명령 시 왼쪽/오른쪽 바퀴 회전량 비교 및 시각화

사용법:
    ros2 run <package> wheel_rotation_visualizer.py
    또는
    python3 wheel_rotation_visualizer.py

기본 동작:
    - 전진(linear.x = 0.1)으로 5초간 주행
    - /joint_states 토픽으로부터 wheel_left_joint, wheel_right_joint position 수집
    - 주행 종료 후 matplotlib으로 회전량 및 차이 시각화
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import threading
import time


class WheelRotationVisualizer(Node):
    def __init__(self, wheel_radius: float = 0.033):
        super().__init__('wheel_rotation_visualizer')
        self.wheel_radius = wheel_radius

        # Publisher / Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # 데이터 저장
        self.times = []
        self.left_positions = []
        self.right_positions = []
        self.diff_positions = []

        self.initial_left = None
        self.initial_right = None
        self.start_time = None
        self.is_collecting = False

        self.lock = threading.Lock()

    def joint_states_callback(self, msg: JointState):
        if not self.is_collecting:
            return

        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            # joint 이름이 없으면 무시
            return

        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        with self.lock:
            if self.initial_left is None:
                self.initial_left = left_pos
                self.initial_right = right_pos
                self.start_time = time.time()

            elapsed = time.time() - self.start_time
            self.times.append(elapsed)
            self.left_positions.append(left_pos - self.initial_left)
            self.right_positions.append(right_pos - self.initial_right)
            self.diff_positions.append(
                (left_pos - self.initial_left) - (right_pos - self.initial_right)
            )

    def run_drive(self, direction: str = 'forward', speed: float = 0.1, duration: float = 5.0):
        """
        direction: 'forward' (전진) 또는 'backward' (후진)
        speed: 선속도 (m/s), 양수 값 사용 후 방향에 따라 부호 변경
        duration: 주행 시간 (초)
        """
        twist = Twist()
        linear_x = speed if direction == 'forward' else -speed
        twist.linear.x = linear_x

        self.get_logger().info(
            f"주행 시작: direction={direction}, speed={abs(linear_x):.2f} m/s, duration={duration:.1f}s"
        )

        # 데이터 수집 시작
        self.is_collecting = True

        # 주행 루프
        start = time.time()
        rate = self.create_rate(20)  # 20Hz
        while rclpy.ok() and (time.time() - start) < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # 정지
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.is_collecting = False

        self.get_logger().info("주행 종료 및 정지 명령 발행")

        # 약간의 여유를 두어 마지막 데이터를 수집
        time.sleep(0.5)

    def plot_results(self):
        with self.lock:
            if not self.times:
                self.get_logger().warn("수집된 데이터가 없습니다.")
                return

            t = self.times[:]
            left = self.left_positions[:]
            right = self.right_positions[:]
            diff = self.diff_positions[:]

        final_diff = diff[-1]
        distance_diff = final_diff * self.wheel_radius
        left_dist = left[-1] * self.wheel_radius
        right_dist = right[-1] * self.wheel_radius

        # 터미널 요약 먼저 출력
        print("\n===== 결과 요약 =====")
        print(f"왼쪽 바퀴 총 회전량:  {left[-1]:.4f} rad")
        print(f"오른쪽 바퀴 총 회전량: {right[-1]:.4f} rad")
        print(f"회전량 차이 (L - R):   {final_diff:.4f} rad")
        print(f"왼쪽 바퀴 이동거리:    {left_dist:.4f} m")
        print(f"오른쪽 바퀴 이동거리:  {right_dist:.4f} m")
        print(f"이동거리 차이 (L - R): {distance_diff:.4f} m")
        print("=====================\n")

        # 시각화
        fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # 상단: 왼쪽/오른쪽 바퀴 누적 회전량
        ax1 = axes[0]
        ax1.plot(t, left, label='Left Wheel', color='blue', linewidth=2)
        ax1.plot(t, right, label='Right Wheel', color='red', linewidth=2)
        ax1.set_ylabel('Cumulative Rotation (rad)')
        ax1.set_title('Wheel Rotation Comparison')
        ax1.legend(loc='upper left')
        ax1.grid(True)

        # 하단: 양쪽 차이
        ax2 = axes[1]
        ax2.plot(t, diff, label='Left - Right', color='green', linewidth=2)
        ax2.axhline(0, color='black', linestyle='--', linewidth=0.8)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Difference (rad)')
        ax2.set_title('Wheel Rotation Difference (Left - Right)')
        ax2.legend(loc='upper left')
        ax2.grid(True)

        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    # 사용자 입력
    print("\n=== TurtleBot3 Wheel Rotation Visualizer ===")

    direction_input = input("Direction [f]orward / [b]ackward (default: forward): ").strip().lower()
    direction = 'backward' if direction_input.startswith('b') else 'forward'

    speed_input = input("Linear speed (m/s, default: 0.1, range 0.1~0.2): ").strip()
    if speed_input:
        speed = float(speed_input)
        if not (0.1 <= speed <= 0.2):
            print("Speed out of range. Using default 0.1 m/s.")
            speed = 0.1
    else:
        speed = 0.1

    duration_input = input("Duration (seconds, default: 5.0): ").strip()
    duration = float(duration_input) if duration_input else 5.0

    node = WheelRotationVisualizer()

    # 별도 스레드에서 spin (콜백 수신용)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # 주행
        node.run_drive(direction=direction, speed=speed, duration=duration)

        # 결과 요약 출력 후 시각화
        node.plot_results()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()