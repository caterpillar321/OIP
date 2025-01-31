import socket
import struct
import json
import time
import threading


class Controller:
    def __init__(self, drone_host, port_command=5001, port_state=5010):
        self.drone_host = drone_host
        self.port_command = port_command
        self.port_state = port_state
        self.command_socket = None
        self.state_socket = None
        self.is_running = False

    def connect(self):
        """드론과 명령 및 상태 소켓 연결 및 핸드셰이크"""
        try:
            # 명령 소켓 연결
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.command_socket.connect((self.drone_host, self.port_command))
            print(f"Connected to command socket on {self.drone_host}:{self.port_command}")

            time.sleep(0.1)
            # 상태 소켓 연결
            self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.state_socket.connect((self.drone_host, self.port_state))
            print(f"Connected to state socket on {self.drone_host}:{self.port_state}")
            time.sleep(0.3)
           

            self.is_running = True
        except Exception as e:
            print(f"Connection error: {e}")

    def send_position_data(self):
        """드론의 recv 함수로 x, y, z 데이터를 전송"""
        try:
            for i in range(10):
                x = 0.0
                y = 0.0
                z = 1.0
                yaw = 0.0 
                message = json.dumps({"x": x, "y": y, "z": z, "yaw" : yaw})
                size = len(message.encode())  
                self.command_socket.sendall(struct.pack(">I", size))
                self.command_socket.sendall(message.encode())
                time.sleep(0.1)
            

            ck = 0
            while self.is_running:
                # x, y, z 데이터 생성
                #x = round(random.uniform(-100.0, 100.0), 2)
                #y = round(random.uniform(-100.0, 100.0), 2)
                #z = round(random.uniform(-100.0, 100.0), 2)
                #yaw = round(random.uniform(-100.0, 100.0), 2)

                # JSON 데이터 생성
                message = json.dumps({"x": x, "y": y, "z": z, "yaw" : yaw})
                size = len(message.encode())

                # 메시지 크기와 데이터 전송
                self.command_socket.sendall(struct.pack(">I", size))
                self.command_socket.sendall(message.encode())
                ck += 1
                if (ck >= 10) :
                    ck = 0
                    x = random.uniform(-1.0, 1.0)
                    y = random.uniform(-1.0, 1.0)
                    #z = round(random.uniform(-1.0, 1.0), 2)
                    yaw = random.uniform(-1.0, 1.0)
                #print(f"Sent position data: {message}")
                time.sleep(0.1)  # 1초에 10번 전송

        except Exception as e:
            print(f"Error in send_position_data: {e}")
            self.is_running = False

    def receive_drone_info(self):
        """드론의 sendInfo 데이터를 수신하고 출력"""
        try:
            while self.is_running:
                # 데이터 크기 수신
                size_buffer = self.state_socket.recv(4)
                size = struct.unpack(">I", size_buffer)[0]

                # 데이터 수신 및 JSON 변환
                data = self.state_socket.recv(size).decode()
                drone_info = json.loads(data)

                # 드론 정보 출력
                #print(f"Received drone info: {drone_info}")
        except Exception as e:
            print(f"Error in receive_drone_info: {e}")
            self.is_running = False

    def start(self):
        """컨트롤러 실행"""
        self.connect()
        if self.is_running:
            # 스레드를 통해 데이터 전송 및 수신 실행
            threading.Thread(target=self.send_position_data, daemon=True).start()
            threading.Thread(target=self.receive_drone_info, daemon=True).start()

    def stop(self):
        """통신 종료"""
        self.is_running = False
        if self.command_socket:
            self.command_socket.close()
        if self.state_socket:
            self.state_socket.close()
        print("Controller stopped")


# 실행 예시
if __name__ == "__main__":
    import random

    controller = Controller(drone_host="192.168.0.13")
    try:
        controller.start()
        while controller.is_running:
            time.sleep(1)  # 메인 루프 유지
    except KeyboardInterrupt:
        controller.stop()
