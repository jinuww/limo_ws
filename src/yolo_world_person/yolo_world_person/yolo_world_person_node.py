import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLOWorld
from collections import deque
import threading
from std_msgs.msg import String

class YoloWorldPersonNode(Node):
    def __init__(self):
        super().__init__('yolo_world_person_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw/compressedDepth')
        self.declare_parameter('conf_thres', 0.75)
        self.declare_parameter('fov', 60.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('show_gui', True)
        
        self.target_labels = []

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
        self.fov = self.get_parameter('fov').get_parameter_value().double_value
        self.img_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.show_gui = self.get_parameter('show_gui').get_parameter_value().bool_value
        
        self.bridge = CvBridge()
        self.model_name = "yolov8s-worldv2.pt"

        print(f"\n[System] Loading Model... ({self.model_name})")
        print(f"[System] Confidence Threshold: {self.conf_thres}")
        self.load_new_model([])

        self.image_sub = self.create_subscription(
            CompressedImage, 
            image_topic, 
            self.image_callback, 
            qos_profile_sensor_data
        )

        self.depth_sub = self.create_subscription(
            CompressedImage, 
            depth_topic, 
            self.depth_callback, 
            qos_profile_sensor_data
        )

        self.cmd_sub = self.create_subscription(
            String,
            '/yolo_world_person/command',
            self.command_callback,
            10
        )

        self.target_pub = self.create_publisher(Point, '/detected_person_target', 10)

        self.latest_depth_img = None
        self.dist_history = deque(maxlen=5)
        self.angle_history = deque(maxlen=5)

        self.input_thread = threading.Thread(target=self.user_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def load_new_model(self, targets):
        try:
            new_model = YOLOWorld(self.model_name)
            if targets:
                new_model.set_classes(targets)
            
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            new_model.predict(dummy_img, verbose=False, device='cpu')
            
            self.model = new_model
            
        except Exception as e:
            print(f"[Error] Model Load Failed: {e}")

    def user_input_loop(self):
        print("\n" + "="*50)
        print("  [YOLO World Interactive Mode]")
        print(f"  Current Confidence: {self.conf_thres}")
        print("  Input target (e.g., person in white clothes, red bottle)")
        print("="*50 + "\n")

        while rclpy.ok():
            try:
                new_target = input("Enter Command: ")
                if new_target.strip():
                    self.target_labels = [new_target.strip()]
                    
                    print(f"   [System] Resetting Model... ('{self.target_labels[0]}')")
                    self.load_new_model(self.target_labels)
                    
                    print(f"Start Detection! Target: '{self.target_labels[0]}'\n")
                else:
                    print(" Please enter text.")
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")
                
    def command_callback(self, msg):
        command_text = msg.data.strip()
        if not command_text:
            return
            
        self.get_logger().info(f"Received Command from Voice: '{command_text}'")
        
        self.target_labels = [command_text]
        self.load_new_model(self.target_labels)
        self.get_logger().info(f"Target Set: {self.target_labels}")

    def depth_callback(self, msg):
        try:
            if not msg.data:
                return

            if 'compressedDepth' in (msg.format or ''):
                header_size = 12
                payload = msg.data[header_size:]
                np_arr = np.frombuffer(payload, np.uint8)
                depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            else:
                np_arr = np.frombuffer(msg.data, np.uint8)
                depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

            if depth is None:
                return

            self.latest_depth_img = depth

        except Exception:
            return

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None: return
        except Exception:
            return

        if not self.target_labels:
            if self.show_gui:
                cv2.putText(frame, "Waiting for command...", (50, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv2.putText(frame, "Check your Terminal", (80, 280), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                cv2.imshow("YOLO-World Tracker", frame)
                cv2.waitKey(1)
            return

        try:
            if hasattr(self, 'model'):
                results = self.model.predict(source=frame, verbose=False, conf=self.conf_thres, device='cpu')
                annotated_frame = results[0].plot()

                boxes = results[0].boxes
                
                if len(boxes) > 0 and self.latest_depth_img is not None:
                    best_box = max(boxes, key=lambda b: (b.xyxy[0][2]-b.xyxy[0][0]) * (b.xyxy[0][3]-b.xyxy[0][1]))
                    x1, y1, x2, y2 = map(int, best_box.xyxy[0].cpu().numpy())
                    
                    h, w = self.latest_depth_img.shape
                    
                    if x2 <= w and y2 <= h:
                        x1_c, y1_c = max(0, x1), max(0, y1)
                        x2_c, y2_c = min(w, x2), min(h, y2)
                        
                        depth_roi = self.latest_depth_img[y1_c:y2_c, x1_c:x2_c]
                        valid_depths = depth_roi[depth_roi > 0]

                        if len(valid_depths) > 0:
                            dist_raw = np.median(valid_depths)
                            dist_meter = dist_raw / 1000.0 if dist_raw > 100 else dist_raw

                            if 0.7 <= dist_meter <= 10.0:
                                center_x = (x1 + x2) / 2
                                angle_raw = ((self.img_width / 2) - center_x) * (self.fov / self.img_width)

                                self.dist_history.append(dist_meter)
                                self.angle_history.append(angle_raw)
                                
                                avg_dist = sum(self.dist_history) / len(self.dist_history)
                                avg_angle = sum(self.angle_history) / len(self.angle_history)

                                target_msg = Point()
                                target_msg.x = float(avg_angle)
                                target_msg.y = float(avg_dist)
                                self.target_pub.publish(target_msg)

                                cv2.putText(annotated_frame, f"Dist: {avg_dist:.2f}m", (x1, y1-25), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            else:
                                cv2.putText(annotated_frame, "Too Close!", (x1, y1-25), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            if self.latest_depth_img is None:
                cv2.putText(annotated_frame, "Waiting for Depth...", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if self.show_gui:
                cv2.putText(annotated_frame, f"Target: {self.target_labels[0]} (Conf: {self.conf_thres})", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("YOLO-World Tracker", annotated_frame)
                cv2.waitKey(1)
                
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = YoloWorldPersonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()