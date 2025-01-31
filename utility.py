import os
import signal
import subprocess
import netifaces
import socket
import time
import select

def kill_process_on_port(port):
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

def internetAvailable():
    try:
        sockt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sockt.settimeout(3)
        sockt.connect(('8.8.8.8', 53))
        return True
    except socket.error:
        return False
    

def get_local_ip():
    exclude_interfaces = ['docker0', 'br-', 'veth', 'vEthernet (WSL)', 'Hyper-V']  # 제외할 인터페이스 이름
    for interface in netifaces.interfaces():
        if any(excluded in interface for excluded in exclude_interfaces):
            continue  # 특정 네트워크 인터페이스는 무시
        addresses = netifaces.ifaddresses(interface)
        if netifaces.AF_INET in addresses:
            for address in addresses[netifaces.AF_INET]:
                ip = address['addr']
                if ip.startswith("192.168.") or ip.startswith("10."):
                    return ip  # 로컬 네트워크 IP만 반환
    return None


def check_existing_socket(sock, interval=5):
    while True:
        try:
            # 소켓에 읽기, 쓰기, 예외 상태 확인
            readable, writable, exceptional = select.select([sock], [sock], [sock], 5)
            if readable:
                data = sock.recv(1024)
                if not data:
                    print("Socket connection closed by the remote host")
                    break
                print("Received data from socket:", data)
            #if writable:
                #print("Socket is writable")
            if exceptional:
                print("Socket has an exceptional condition")
                break

        except socket.error as e:
            print(f"Socket error: {e}")
            with open('status.txt', 'w') as file:
                file.write('Reconnection')
                file.close()
            break
        except ValueError as e:
            print(e)
            break

        time.sleep(interval)