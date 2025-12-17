import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math
import time

class EvacuationCommander(Node):
    def __init__(self):
        super().__init__('evacuation_commander')

        self.EXIT_COORDS = {
            'x': -0.399,   
            'y': 3.03,  
            'w': 1.0    
        }

        # 상태 정의
        self.STATE_SEARCH = 0    
        self.STATE_APPROACH = 1 
        self.STATE_TALK = 2      
        self.STATE_GUIDE = 3     
        self.STATE_FINISH = 4    

        self.current_state = self.STATE_SEARCH
        
        # Nav2 설정
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()   

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.voice_pub = self.create_publisher(String, '/robot_voice', 10)
        self.create_subscription(Point, '/detected_person_target', self.target_callback, 10)
        
        self.create_timer(1.0, self.check_goal_status)

        self.get_logger().info("🚀 Commander 시작: 사람을 찾으면 자동으로 비상구로 안내합니다.")

    def target_callback(self, msg: Point):
        # """ YOLO에서 사람이 감지되면 실행 """
        # # 이미 안내 중이면 사람 감지 무시 (State Machine 역할)
        # if self.current_state >= self.STATE_TALK:
        #     return

        # dist = msg.y
        # angle = msg.x

        # # [상태 1] 사람 발견 -> 접근 모드 전환
        # if self.current_state == self.STATE_SEARCH:
        #     self.current_state = self.STATE_APPROACH
        #     self.get_logger().info("👁️ 사람 발견! 접근 시작")

        # # [상태 2] 접근 중 -> 도착 판단
        # if self.current_state == self.STATE_APPROACH:
        #     if dist <= 1.0: # 1m 앞 도착
        #         self.navigator.cancelTask() # 이동 멈춤
        #         self.run_sequence_talk_and_guide() # 다음 시퀀스 실행
        #     else:
        #         self.send_approach_goal(dist, angle) # 계속 이동
        
        # ✅ GUIDE/FINISH면 추적 goal이 절대 안 나가게 막기
        if self.current_state in (self.STATE_GUIDE, self.STATE_FINISH):
            return

        dist = msg.y
        angle = msg.x

        if self.current_state == self.STATE_SEARCH:
            self.current_state = self.STATE_APPROACH
            self.get_logger().info("👁️ 사람 발견! 접근 시작")

        if self.current_state == self.STATE_APPROACH:
            if dist <= 1.0:
                self.navigator.cancelTask()
                self.run_sequence_talk_and_guide()
            else:
                self.send_approach_goal(dist, angle)

    def run_sequence_talk_and_guide(self):

        self.current_state = self.STATE_GUIDE
        self.get_logger().info(f"🏃 비상구({self.EXIT_COORDS})로 이동 시작")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(self.EXIT_COORDS['x'])
        goal_pose.pose.position.y = float(self.EXIT_COORDS['y'])
        #goal_pose.pose.orientation.w = float(self.EXIT_COORDS['w'])

        # quaternion 정상값
        # goal_pose.pose.orientation.x = 0.0
        # goal_pose.pose.orientation.y = 0.0
        # goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        # 비상구 목표 토픽
        self.get_logger().info(
            f"EXIT GOAL SENT -> x={goal_pose.pose.position.x:.3f}, y={goal_pose.pose.position.y:.3f}"
        )

        # Nav2에게 자동 이동 명령
        self.navigator.goToPose(goal_pose)

    def check_goal_status(self):
        if self.current_state == self.STATE_GUIDE:
            if self.navigator.isTaskComplete():
                self.get_logger().info("비상구 도착")
                
                msg = String()
                msg.data = "비상구입니다. 대피하세요."
                self.voice_pub.publish(msg)
                
                self.current_state = self.STATE_FINISH

    def send_approach_goal(self, dist, angle):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx, ry = trans.transform.translation.x, trans.transform.translation.y
            q = trans.transform.rotation
            (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

            target_angle = yaw + math.radians(angle)
            travel_dist = max(0.0, dist - 1.0) 
            gx = rx + (travel_dist * math.cos(target_angle))
            gy = ry + (travel_dist * math.sin(target_angle))

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = gx
            pose.pose.position.y = gy
            pose.pose.orientation.w = 1.0 
            
            self.navigator.goToPose(pose)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = EvacuationCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()