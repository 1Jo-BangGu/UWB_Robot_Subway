import threading
import requests
import cv2
import numpy as np
import collections

class Client:
    def __init__(self, server_ip):
        self.server_ip = server_ip
        self.frame_queue = collections.deque(maxlen=5)
        self.queue_lock = threading.Lock()
        self.running = False
        self.stream_url = f'http://{server_ip}:5000/stream/global'
        self.latest_frame = None
        self.transfer_data_url = f'http://{server_ip}:5000/transfer_data'

    def network_thread(self):
        session = requests.Session()
        stream = session.get(self.stream_url, stream=True, timeout=3)
        bytes_data = b''
        while self.running:
            chunk = stream.raw.read(131072)
            if not chunk:
                continue
            bytes_data += chunk
            while True:
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                if a == -1 or b == -1:
                    break
                jpg = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    with self.queue_lock:
                        self.latest_frame = frame
        session.close()

    def start(self):
        self.running = True
        threading.Thread(target=self.network_thread, daemon=True).start()

    def send_message(self, topic, message):
        """비동기 메시지 전송"""
        try:
            requests.post(
                f'{self.transfer_data_url}',
                json={'topic': topic, 'message': message},
                timeout=1.5
            )
            # print(f"[{topic}] {message} 전송 완료")
        except Exception as e:
            print(f"[{topic} error] 메시지 전송 실패: {e}")

    def upload_frame(self, frame, quality=50):
        """현재 프레임을 서버로 비동기 업로드 (JPEG 압축 후 전송)"""
        def _send(encoded_frame):
            try:
                requests.post(
                    f'http://{self.server_ip}:5000/upload/processed_global',
                    files={'frame': ('frame.jpg', encoded_frame.tobytes(), 'image/jpeg')},
                    timeout=3
                )
            except Exception as e:
                print(f"[frame upload error] {e}")

        success, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if success:
            threading.Thread(target=_send, args=(buffer,), daemon=True).start()