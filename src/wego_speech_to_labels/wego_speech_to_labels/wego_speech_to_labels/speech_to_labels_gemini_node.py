import io
import json
import time
import wave
import queue
import threading
import os
import sys

import numpy as np
import sounddevice as sd
import webrtcvad

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from google import genai
from google.genai import types


def pcm16_to_wav_bytes(pcm16: np.ndarray, sample_rate: int) -> bytes:
    """int16 PCM mono -> WAV bytes"""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # int16
        wf.setframerate(sample_rate)
        wf.writeframes(pcm16.tobytes())
    return buf.getvalue()


class SpeechToLabelsGemini(Node):

    def __init__(self):
        super().__init__("speech_to_labels_gemini_node")

        # ===== Params =====
        self.declare_parameter("model", "gemini-2.5-flash")
        self.declare_parameter("vad_mode", 2)              
        self.declare_parameter("silence_end_ms", 900)      
        self.declare_parameter("min_utter_sec", 0.7)

        # ===== 종료/조건 파라미터 =====
        self.declare_parameter("require_person_word", True)    
        self.declare_parameter("exit_after_publish", True)     
        self.declare_parameter("exit_delay_sec", 0.6)          
        self.declare_parameter("wait_for_subscribers_sec", 3.0)

        self.model = self.get_parameter("model").value
        self.vad_mode = int(self.get_parameter("vad_mode").value)
        self.silence_end_ms = int(self.get_parameter("silence_end_ms").value)
        self.min_utter_sec = float(self.get_parameter("min_utter_sec").value)

        self.require_person_word = bool(self.get_parameter("require_person_word").value)
        self.exit_after_publish = bool(self.get_parameter("exit_after_publish").value)
        self.exit_delay_sec = float(self.get_parameter("exit_delay_sec").value)
        self.wait_for_subscribers_sec = float(self.get_parameter("wait_for_subscribers_sec").value)

        # ===== State =====
        self._published_once = False
        self._shutdown_requested = False
        self._shutdown_request_time = 0.0

        # ===== QoS =====
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # ===== ROS pubs =====
        self.pub_text = self.create_publisher(String, "/target_description", qos)
        self.pub_labels = self.create_publisher(String, "/yoloworld/target_labels", qos)  # JSON array string
        self.pub_yolo_cmd = self.create_publisher(String, "/yolo_world_person/command", 10)

        # ===== 종료 타이머 =====
        self.create_timer(0.1, self._maybe_shutdown)

        # ===== Audio 설정 =====
        self.sample_rate = 16000
        self.channels = 1
        self.dtype = "int16"

        self.vad = webrtcvad.Vad(self.vad_mode)
        self.frame_ms = 30
        self.frame_size = int(self.sample_rate * self.frame_ms / 1000)  # samples
        self.frame_bytes = self.frame_size * 2                           # int16 -> 2 bytes

        self.stream = None
        self.audio_q = queue.Queue()
        self.utt_q = queue.Queue()

        self.listen_event = threading.Event()

        # ===== Gemini client =====
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise RuntimeError(
                "GEMINI_API_KEY가 설정되지 않았습니다.\n"
                "터미널에서 다음처럼 설정 후 다시 실행하세요:\n"
                "  export GEMINI_API_KEY='YOUR_KEY'\n"
            )
        self.client = genai.Client(api_key=api_key)

        # Workers
        threading.Thread(target=self.keyboard_worker, daemon=True).start()
        threading.Thread(target=self.segment_worker, daemon=True).start()
        threading.Thread(target=self.gemini_worker, daemon=True).start()

        self.get_logger().info(f"Two-step Speech→Labels (model={self.model}, vad_mode={self.vad_mode})")
        self.get_logger().info("준비되면 콘솔에 1을 입력하고 Enter를 치세요.")
        self.get_logger().info("조건 만족 시 1회 publish 후 종료합니다.")


    def _start_stream(self):
        if self.stream is not None:
            return
        self.audio_q = queue.Queue()  
        self.stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype=self.dtype,
            blocksize=self.frame_size,
            callback=self.audio_callback,
        )
        self.stream.start()

    def _stop_stream(self):
        if self.stream is None:
            return
        try:
            self.stream.stop()
            self.stream.close()
        except Exception:
            pass
        self.stream = None

    def audio_callback(self, indata, frames, time_info, status):
        self.audio_q.put(bytes(indata))

    def _maybe_shutdown(self):
        if not self._shutdown_requested:
            return
        if time.time() - self._shutdown_request_time >= self.exit_delay_sec:
            self.get_logger().info("publish 완료 → 노드 종료")
            rclpy.shutdown()

    def keyboard_worker(self):
        while rclpy.ok() and (not self._published_once):
            try:
                s = input("준비됐으면 1을 입력하고 Enter를 치고, 바로 말하세요: ").strip()
            except EOFError:
                time.sleep(0.2)
                continue

            if s == "1":
                self.get_logger().info("지금부터 말하세요...")
                self.listen_event.set()
            else:
                self.get_logger().info("1을 입력하면 녹음을 시작합니다.")

    def segment_worker(self):
        while rclpy.ok():
            if self._published_once:
                time.sleep(0.1)
                continue

            self.listen_event.wait(timeout=0.2)
            if not self.listen_event.is_set():
                continue

            self._start_stream()

            in_speech = False
            voiced = []
            silence_ms = 0

            start_time = time.time()
            self.get_logger().info("말하세요")

            while rclpy.ok() and self.listen_event.is_set() and (not self._published_once):
                try:
                    frame = self.audio_q.get(timeout=0.8)
                except queue.Empty:
                    if time.time() - start_time > 10.0:
                        self.get_logger().info("입력이 없어 녹음을 종료합니다. 다시 1을 입력하세요.")
                        break
                    continue

                if len(frame) != self.frame_bytes:
                    continue

                is_speech = self.vad.is_speech(frame, self.sample_rate)

                if is_speech:
                    if not in_speech:
                        in_speech = True
                        voiced = []
                        silence_ms = 0
                    voiced.append(frame)
                else:
                    if in_speech:
                        silence_ms += self.frame_ms
                        voiced.append(frame)
                        if silence_ms >= self.silence_end_ms:
                            # 발화 끝
                            break

            self._stop_stream()

            self.listen_event.clear()

            if not voiced:
                continue

            audio_bytes = b"".join(voiced)
            pcm16 = np.frombuffer(audio_bytes, dtype=np.int16)
            dur = len(pcm16) / self.sample_rate

            if dur < self.min_utter_sec:
                self.get_logger().info(f"다시 1을 입력하세요.")
                continue

            self.get_logger().info(f"utterance 수집 완료 ({dur:.2f}s)")
            self.utt_q.put(pcm16.copy())

    def gemini_worker(self):
        schema = {
            "type": "object",
            "properties": {
                "transcript": {"type": "string"},
                "labels": {
                    "type": "array",
                    "items": {"type": "string"},
                    "minItems": 1,
                    "maxItems": 4
                }
            },
            "required": ["transcript", "labels"]
        }

        while rclpy.ok():
            if self._published_once:
                time.sleep(0.1)
                continue

            try:
                pcm16 = self.utt_q.get(timeout=0.5)
            except queue.Empty:
                continue

            wav_bytes = pcm16_to_wav_bytes(pcm16, self.sample_rate)

            prompt = (
                "You are an accurate Korean speech recognizer AND a label extractor for YOLO-World.\n"
                "1) Transcribe the Korean speech accurately.\n"
                "2) Convert the intent into a minimal list of SHORT English labels YOLO-World can use.\n"
                "Rules for labels:\n"
                "- If the user asks to find a person, include 'person'.\n"
                "- Add up to 2 attribute phrases like 'black shirt', 'red hat', 'blue jacket'.\n"
                "- Keep labels lowercase.\n"
                "Return ONLY JSON that matches the provided schema."
            )

            try:
                resp = self.client.models.generate_content(
                    model=self.model,
                    contents=[
                        types.Part.from_bytes(data=wav_bytes, mime_type="audio/wav"),
                        prompt,
                    ],
                    config={
                        "temperature": 0,
                        "response_mime_type": "application/json",
                        "response_json_schema": schema,
                    },
                )

                data = json.loads((resp.text or "{}").strip())
                transcript = (data.get("transcript") or "").strip()
                labels = data.get("labels") or []

                if not isinstance(labels, list):
                    labels = []

                # labels 정리/중복 제거
                norm = []
                for x in labels:
                    if isinstance(x, str):
                        x = x.strip().lower()
                        if x and x not in norm:
                            norm.append(x)
                if "person" not in norm:
                    norm.insert(0, "person")

                if self.require_person_word and ("사람" not in transcript):
                    self.get_logger().info(f"'사람' 미포함 publish x: {transcript}")
                    self.get_logger().info("다시 1을 입력하고 말하세요.")
                    continue

                start_wait = time.time()
                while (self.pub_labels.get_subscription_count() == 0) and (time.time() - start_wait < self.wait_for_subscribers_sec):
                    time.sleep(0.05)

                if transcript:
                    m = String()
                    m.data = transcript
                    self.pub_text.publish(m)

                out = String()
                out.data = json.dumps(norm, ensure_ascii=False)
                self.pub_labels.publish(out)
                target_str = "person"
                for lab in norm:
                    if lab != "person":
                        target_str = f"person in {lab}"
                        break

                cmd = String()
                cmd.data = target_str
                self.pub_yolo_cmd.publish(cmd)

                self.get_logger().info(f"transcript: {transcript}")
                self.get_logger().info(f"labels: {out.data}")
                self.get_logger().info(f"yolo command: {cmd.data}")

                self._published_once = True
                if self.exit_after_publish:
                    self._shutdown_requested = True
                    self._shutdown_request_time = time.time()
                    self.get_logger().info("1회 publish 완료 → 종료 예약")

            except Exception as e:
                self.get_logger().warn(f"Gemini call failed: {e}")
                self.get_logger().info("다시 1을 입력하고 말하세요.")

    def destroy_node(self):
        self._stop_stream()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToLabelsGemini()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
