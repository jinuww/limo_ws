import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DabaiCamPublisher(Node):
    def __init__(self):
        super().__init__('dabai_cam_publisher')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('image_topic', '/dabai/camera_raw')

        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.get_logger().info(f"Opening Dabai camera on /dev/video{device_id}")
        self.get_logger().info(f"Resolution: {width}x{height}, FPS: {fps}")
        self.get_logger().info(f"Publishing to: {image_topic}")

        # 퍼블리셔
        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()

        # 카메라 오픈
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error("카메라 열기 실패")
            raise RuntimeError("Failed to open camera")

        # 타이머 콜백 (1/fps 주기)
        self.timer = self.create_timer(1.0 / fps, self.timer_cb)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임 읽기 실패")
            return

        # OpenCV BGR → ROS Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(msg)

        # 원본 디버그용 GUI (원하면 끄거나 켜고)
        cv2.imshow("Dabai camera raw", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("q 입력 감지, 노드 종료")
            rclpy.shutdown()

    def destroy_node(self):
        # 리소스 정리
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DabaiCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
