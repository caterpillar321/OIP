import socket
import time
import struct
import rclpy
from rclpy.node import Node
import json
from drone.Drone import Drone
import threading
#from utility import kill_process_on_port
import os
import signal
import subprocess
import netifaces

class Connection(Node):
    def __init__(self, host="0.0.0.0"):
        super().__init__('connection_node')
        self.host = host
        self.drone = Drone()
        self.sock = None
        self.sock_info = None
        self.is_connected = False
        # 연결 완료를 알리기 위한 이벤트
        self.connection_event = threading.Event()

    def kill_process_on_port(self, port):
    # Find the process ID (PID) using lsof
        try:
            result = subprocess.check_output(["lsof", "-t", f"-i:{port}"])
            pids = result.decode().strip().split("\n")
        
            for pid in pids:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                    print(f"Process {pid} on port {port} has been terminated.")
                except OSError as e:
                    print(f"Error terminating process {pid}: {e}")
        except subprocess.CalledProcessError as e:
            print(f"No process found on port {port}")

    def accept_connection(self, sock, attr_name):
        conn, addr = sock.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        setattr(self, attr_name, conn)
        self.get_logger().info(f"Connection established on {attr_name} with {addr}")
        # 두 소켓 모두 연결되면 이벤트를 set
        if self.sock is not None and self.sock_info is not None:
            self.is_connected = True
            self.connection_event.set()

    def start_server(self):
        
        self.get_logger().info("Starting server...")

        tsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tsock.bind((self.host, 5001))
        tsock.listen(1)

        tsock_info = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tsock_info.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tsock_info.bind((self.host, 5010))
        tsock_info.listen(1)

        # 두 스레드를 생성 (join() 대신 별도 처리)
        threading.Thread(target=self.accept_connection, args=(tsock, 'sock'), daemon=True).start()
        threading.Thread(target=self.accept_connection, args=(tsock_info, 'sock_info'), daemon=True).start()

        # 연결 완료될 때까지 대기 (타임아웃 설정 가능)
        if not self.connection_event.wait(timeout=20):
            self.get_logger().error("서버 연결 타임아웃")
        else:
            self.get_logger().info("Server connection established")

        tsock.close()
        tsock_info.close()

    def recvall(self, sock, n):
        data = b''
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:  # 소켓이 닫혔을 경우
                return None
            data += packet
        return data
    
    def recv(self):
        # 연결이 될 때까지 대기
        self.connection_event.wait()
        print("checkRecv")
        if not self.sock or not self.is_connected:
            self.get_logger().error("Error: Connection not established in recv()")
            return
        
        ck = 0
        while self.is_connected:
            try:
                sizebuffer = self.recvall(self.sock, 4)
                if not sizebuffer:
                    break
                size = struct.unpack('>I', sizebuffer)[0]
                message_bytes = self.recvall(self.sock, size)
                buffer = message_bytes.decode()
                data = json.loads(buffer)

                ck += 1
                if ck % 3 == 0:
                    self.get_logger().info(f"Received data: {data}")
                self.drone.operate(data)
                time.sleep(0.07)

            except Exception as e:
                self.get_logger().error(f"Error in recv: {e}")
                self.is_connected = False
                break

    def send_info(self):
        # 연결이 될 때까지 대기
        self.connection_event.wait()
        self.get_logger().info("Info sending started")
        try:
            while self.is_connected:
                roll, pitch, yaw = self.drone.roll, self.drone.pitch, self.drone.yaw
                # 단순화된 예: 음수 처리
                roll = roll if roll >= 0 else roll + 360
                pitch = pitch if pitch >= 0 else pitch + 360
                yaw = yaw if yaw >= 0 else yaw + 360

                message = json.dumps({
                    "roll": f"{roll:03.0f}",
                    "pitch": f"{pitch:03.0f}",
                    "yaw": f"{yaw:03.0f}",
                })

                size = len(message.encode())
                if self.sock_info.fileno() == -1:
                    break

                self.sock_info.sendall(struct.pack(">L", size))
                self.sock_info.sendall(message.encode())
                time.sleep(0.05)

        except Exception as e:
            self.get_logger().error(f"Error in send_info: {e}")
            self.is_connected = False

    def make_basic_connect(self):
        threading.Thread(target=self.recv, daemon=True).start()
        threading.Thread(target=self.send_info, daemon=True).start()



        

