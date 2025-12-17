import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
import threading
import math
import struct
import wave

class VoiceOutputNode(Node):
    def __init__(self):
        super().__init__('voice_output_node')
        
        self.sub = self.create_subscription(String, '/voice_cmd', self.callback, 10)
        
        self.script_dir = "/home/wego/wego_ws/src/yolo_world_person/yolo_world_person"
        
        self.mp3_player = "mpg321 -q"
        self.wav_player = "aplay -q" 

        self.check_and_create_files()
        
        self.is_beeping = False
        self.beep_thread = None
        
        self.get_logger().info("oice Node Ready!")

    def get_file_path(self, filename):
        return os.path.join(self.script_dir, filename)

    def create_electronic_beep(self, filename):

        try:
            sample_rate = 44100
            duration = 0.1       
            frequency = 1500    
            volume = 0.5         

            n_samples = int(sample_rate * duration)
            
            with wave.open(filename, 'w') as wav_file:
                wav_file.setnchannels(1)      
                wav_file.setsampwidth(2)      
                wav_file.setframerate(sample_rate)
                
                for i in range(n_samples):

                    value = int(32767.0 * volume * math.sin(2 * math.pi * frequency * i / sample_rate))
                    data = struct.pack('<h', value)
                    wav_file.writeframes(data)
            
            self.get_logger().info(f"비프음 생성 완료: {filename}")
        except Exception as e:
            self.get_logger().error(f"비프음 생성 실패: {e}")

    def check_and_create_files(self):
        mp3_files = ["follow_me.mp3", "arrived_exit.mp3"]
        for f in mp3_files:
            path = self.get_file_path(f)
            if not os.path.exists(path):
                self.get_logger().warn(f"파일 없음: {f}")

        beep_path = self.get_file_path("beep.wav")
        if not os.path.exists(beep_path):
            self.get_logger().info("beep.wav 파일이 없습니다.")
            self.create_electronic_beep(beep_path)
        else:
            self.get_logger().info("beep.wav 확인 완료.")

    def play_beep_loop(self):
        beep_path = self.get_file_path("beep.wav")
        
        if not os.path.exists(beep_path):
            self.get_logger().error(f"파일 없음: {beep_path}")
            return

        while self.is_beeping and rclpy.ok():

            os.system(f"{self.wav_player} {beep_path}")
            time.sleep(0.3)  

    def stop_beeping(self):
        if self.is_beeping:
            self.is_beeping = False
            if self.beep_thread:
                self.beep_thread.join()
                self.beep_thread = None

    def callback(self, msg):
        command = msg.data
        
        # 1. 비프음 시작
        if command == "beep_start":
            if not self.is_beeping:
                self.get_logger().info("비프음")
                self.is_beeping = True
                self.beep_thread = threading.Thread(target=self.play_beep_loop)
                self.beep_thread.daemon = True
                self.beep_thread.start()

        # 2. 비프음 중지
        elif command == "beep_stop":
            if self.is_beeping:
                self.get_logger().info("중지")
                self.stop_beeping()

        elif command in ["follow_me", "arrived_exit"]:
            self.stop_beeping() 
            
            filename = f"{command}.mp3"
            file_path = self.get_file_path(filename)
            
            if os.path.exists(file_path):
                self.get_logger().info(f"음성 출력: {filename}")
                os.system(f"{self.mp3_player} {file_path}")
            else:
                self.get_logger().error(f"파일 없음: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceOutputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_beeping()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()