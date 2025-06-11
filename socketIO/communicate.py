# communicate.py

import socketio
import asyncio
import threading
import sys

class ClientSocketIO:
    def __init__(self, server_ip, server_port=6000):
        self.server_ip = server_ip
        self.server_port = server_port
        self.sio = socketio.AsyncClient(reconnection=True, reconnection_attempts=5, reconnection_delay=1)
        self.running = False

        # 이벤트 핸들러 등록
        self.sio.on('connect', self.on_connect)
        self.sio.on('connect_error', self.on_connect_error)
        self.sio.on('patrol_update', self.on_patrol_update)
        self.sio.on('perception_update', self.on_perception_update)
        self.sio.on('disconnect', self.on_disconnect)

        # 필요시 외부에서 접근할 수 있도록 상태값 저장
        self.return_flag = 0
        self.fire = 0
        self.fall = 0

    async def on_connect(self):
        print("[✅] 서버 연결 성공")

    async def on_connect_error(self, error):
        print(f"[⚠️] 연결 실패: {error}")

    async def on_patrol_update(self, data):
        try:
            self.return_flag = 0 if data['patrol'] == True else 1
            print(f"[→] 순찰 상태 수신: {self.return_flag}")
        except KeyError as e:
            print(f"[⚠️] 잘못된 데이터 키: {e}")

    async def on_perception_update(self, data):
        try:
            self.fire = 1 if data['fire'] == True else 0
            self.fall = 1 if data['fall'] == True else 0
            print(f"[→] 인식 상태 수신: fire={self.fire}, fall={self.fall}")
        except KeyError as e:
            print(f"[⚠️] 잘못된 데이터 키: {e}")

    async def on_disconnect(self):
        print("[⚠️] 서버 연결 해제")

    async def connect_and_listen(self):
        try:
            await self.sio.connect(
                f'http://{self.server_ip}:{self.server_port}',
                transports=['websocket'],
                headers={'Device-Type': 'PC_Client'}
            )
            await self.sio.wait()
        except Exception as e:
            print(f"[⚠️] 연결 오류: {str(e)}")
            sys.exit(1)

    def start(self):
        """비동기 메시지 수신을 별도 스레드에서 시작"""
        self.running = True
        threading.Thread(target=self._run_asyncio_loop, daemon=True).start()

    def _run_asyncio_loop(self):
        asyncio.run(self.connect_and_listen())

    def stop(self):
        """클라이언트 연결 종료"""
        self.running = False
        try:
            asyncio.run(self.sio.disconnect())
        except Exception:
            pass