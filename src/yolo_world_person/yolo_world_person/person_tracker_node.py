#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # --- 파라미터 ---
        self.declare_parameter('camera_fov_deg', 60.0)      # 카메라 수평 FOV
        self.declare_parameter('angle_offset_deg', 0.0)     # 카메라-라이다 yaw 오프셋
        self.declare_parameter('timeout_sec', 1.0)          # detection timeout

        self.fov_deg = self.get_parameter('camera_fov_deg').value
        self.angle_offset = math.radians(
            self.get_parameter('angle_offset_deg').value
        )
        self.timeout = self.get_parameter('timeout_sec').value

        # 최신 데이터 저장용
        self.last_bbox = None        # [cx, cy, w, h, conf, img_w, img_h]
        self.last_bbox_stamp = None
        self.last_scan = None

        # Subscriber 설정
        self.sub_bbox = self.create_subscription(
            Float32MultiArray,
            'person_detector/bbox',
            self.bbox_callback,
            10
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher 설정
        self.pub_state = self.create_publisher(String, '/person_state', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/person_relative_pose', 10)

        # 10Hz 주기로 상태 갱신
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('person_tracker node started ✅')

    def bbox_callback(self, msg: Float32MultiArray):
        # 데이터 길이 확인
        if len(msg.data) < 7:
            self.get_logger().warn(
                'person_detector/bbox length < 7, expected [cx, cy, w, h, conf, img_w, img_h]'
            )
            return

        self.last_bbox = msg.data[:]  # 리스트 복사
        self.last_bbox_stamp = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def update(self):
        # 기본 상태: LOST
        state = String()
        state.data = 'LOST'

        now = self.get_clock().now()

        # bbox/scan이 둘 다 없으면 LOST 유지
        if self.last_bbox is None or self.last_scan is None or self.last_bbox_stamp is None:
            self.pub_state.publish(state)
            return

        # timeout 체크
        dt = (now - self.last_bbox_stamp).nanoseconds * 1e-9
        if dt > self.timeout:
            # 오래 동안 detection 없음
            self.pub_state.publish(state)
            return

        # --- bbox → 각도 계산 ---
        cx, cy, w, h, conf, img_w, img_h = self.last_bbox
        img_w = float(img_w)
        img_h = float(img_h)

        fov_rad = math.radians(self.fov_deg)

        # 이미지 중심 기준 -1~1 정규화
        u_norm = (cx - img_w / 2.0) / (img_w / 2.0)
        theta_cam = u_norm * (fov_rad / 2.0)
        theta = theta_cam + self.angle_offset   # 라이다 기준 각도

        scan = self.last_scan
        idx = int(round((theta - scan.angle_min) / scan.angle_increment))

        if idx < 0 or idx >= len(scan.ranges):
            # 라이다 범위 밖
            self.pub_state.publish(state)
            return

        # 주변 빔 몇 개 같이 사용
        window = 3
        i0 = max(0, idx - window)
        i1 = min(len(scan.ranges) - 1, idx + window)

        valid_ranges = [
            r for r in scan.ranges[i0:i1+1]
            if not math.isinf(r) and not math.isnan(r) and r > 0.05
        ]

        if len(valid_ranges) == 0:
            self.pub_state.publish(state)
            return

        dist = min(valid_ranges)

        # --- 거리/각도 → base_link 좌표 ---
        x = dist * math.cos(theta)
        y = dist * math.sin(theta)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 방향은 일단 사용 X

        state.data = 'DETECTED'

        # Publish
        self.pub_pose.publish(pose)
        self.pub_state.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
