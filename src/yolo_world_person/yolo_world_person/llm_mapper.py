# llm_mapper.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import os

class LLMLabelMapper(Node):
    def __init__(self):
        super().__init__('llm_label_mapper_node')
        
        # [입력] 운용자가 영어로 명령을 내리는 곳 (예: "a man with black hair")
        self.subscription = self.create_subscription(
            String, '/target_description', self.listener_callback, 10)
        
        # [출력] YOLO에게 전달할 핵심 단어들 (예: "man, black hair")
        self.publisher_ = self.create_publisher(String, '/yoloworld/target_labels', 10)

        # API 키 설정
        self.api_key = os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            self.get_logger().error("API Key가 없습니다! export OPENAI_API_KEY='...' 해주세요.")
        else:
            self.client = OpenAI(api_key=self.api_key)

    def listener_callback(self, msg):
        user_text = msg.data # 영어 문장이 들어옴
        self.get_logger().info(f"[Input] English Command: {user_text}")

        # GPT에게 핵심 단어 추출 요청
        labels = self.call_gpt(user_text)
        
        if labels:
            out_msg = String()
            out_msg.data = labels
            self.publisher_.publish(out_msg)
            self.get_logger().info(f"[Output] To YOLO: {labels}")

    def call_gpt(self, text):
        try:
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {
                        "role": "system", 
                        "content": (
                            "You are an assistant for an object detection system. "
                            "Extract the object class and visual attributes from the user's English description. "
                            "Output ONLY a comma-separated list of English keywords. "
                            "Example: 'Find a black hair man, in black clothes' -> 'man, black hair, black clothes'"
                        )
                    },
                    {"role": "user", "content": text}
                ],
                max_tokens=50
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            self.get_logger().error(f"GPT Error: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = LLMLabelMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
