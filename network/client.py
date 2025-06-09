import requests
import threading
import collections
import cv2
import numpy as np

class RaspiVisionClient:
    def __init__(self, server_ip):
        self.server_ip = server_ip
        self.frame_queue = collections.deque(maxlen=5)
        self.queue_lock = threading.Lock()
        self.running = False
        self.stream_url = f'http://{server_ip}:5000/stream/2ce8fa8a-6010-463a-834f-18bcb78d022d'
        self.latest_frame = None

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
