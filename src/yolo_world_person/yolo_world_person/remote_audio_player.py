import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import time

class RemoteAudioPlayer(Node):
    def __init__(self):
        super().__init__('remote_audio_player')

        # 로봇이 보내는 경고 신호 구독
        self.subscription = self.create_subscription(
            String,
            '/threat_alert',
            self.alert_callback,
            10)
            
        self.MENT = "거기 가지. 더 다가오면 가만두지 않겠다."
        self.mp3_file = "defense_voice.mp3" # 현재 폴더에 저장
        
        # 실행 시 음성 파일 미리 생성
        self.preload_voice()
        self.get_logger().info("🔊 오디오 시스템 대기 중 (노트북)")

    def preload_voice(self):
        if not os.path.exists(self.mp3_file):
            self.get_logger().info(f'음성 생성 중... "{self.MENT}"')
            try:
                tts = gTTS(text=self.MENT, lang='ko')
                tts.save(self.mp3_file)
            except Exception as e:
                self.get_logger().error(f'음성 생성 실패: {e}')
        else:
            self.get_logger().info('기존 음성 파일 로드 완료.')

    def alert_callback(self, msg):
        self.get_logger().info(f"📩 로봇으로부터 신호 수신: {msg.data}")
        
        # 소리 재생 (노트북 스피커)
        # Linux(Ubuntu) 기준 mpg321 사용
        if os.path.exists(self.mp3_file):
            self.get_logger().warn("🔊 경고 방송 송출!")
            os.system(f'mpg321 -q {self.mp3_file} &')
        else:
            self.get_logger().error("재생할 mp3 파일이 없습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = RemoteAudioPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()