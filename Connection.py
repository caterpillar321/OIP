import socket
import time
import struct
import rclpy
from rclpy.node import Node
import json
from drone.Drone import Drone
import threading

class Connection(Node):
    def __init__(self, host="0.0.0.0"):
        super().__init__('connection_node')
        self.host = host
        self.drone = Drone()
        self.sock = None
        self.sock_info = None
        self.is_connected = False

    def start_server(self):
        self.get_logger().info("Starting server...")
        tsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tsock.bind((self.host, 5001))
        tsock.listen(1)

        self.sock, addr = tsock.accept()
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        tsock_info = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tsock_info.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tsock_info.bind((self.host, 5010))
        tsock_info.listen(1)

        self.sock_info, addr = tsock_info.accept()
        self.sock_info.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self.is_connected = True
        self.get_logger().info("Server connection established")

        tsock.close()
        tsock_info.close()

    def recv(self):
        if not self.sock or not self.is_connected:
            self.get_logger().error("Error: Connection not established")
            return

        while self.is_connected:
            try:
                sizebuffer = self.sock.recv(4)
                if not sizebuffer:
                    break
                size = struct.unpack('>I', sizebuffer)[0]
                buffer = self.sock.recv(size).decode()
                data = json.loads(buffer)

                self.get_logger().info(f"Received data: {data}")
                self.drone.operate(data)

                time.sleep(0.07)

            except (ConnectionAbortedError, ConnectionResetError, OSError, struct.error) as e:
                self.get_logger().error(f"Error in recv: {e}")
                self.is_connected = False
                break

            except json.JSONDecodeError:
                self.get_logger().error("JSON Decode Error")
                self.sock.recv(1000)

    def send_info(self):
        self.get_logger().info("Info sending started")
        try:
            while self.is_connected:
                roll, pitch, yaw = self.drone.roll, self.drone.pitch, self.drone.yaw

                if roll < 0:
                    roll += 360
                if pitch < 0:
                    pitch += 360
                if yaw < 0:
                    yaw += 360

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

                #self.get_logger().info(f"Sent: {message}")
                time.sleep(0.05)

        except (ConnectionAbortedError, ConnectionResetError, OSError) as e:
            self.get_logger().error(f"Error in send_info: {e}")
            self.is_connected = False
            self.sock_info.close()

    def make_basic_connect(self):
        recv_thread = threading.Thread(target=self.recv)
        send_thread = threading.Thread(target=self.send_info)

        recv_thread.start()
        send_thread.start()



        

