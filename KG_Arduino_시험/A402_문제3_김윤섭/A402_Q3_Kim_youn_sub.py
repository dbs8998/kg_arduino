import socket
import threading
import time
from pynput import keyboard

esp_ip = "172.30.1.11"
port = 1234

sock = None
connected = False
recv_thread = None

key_to_command = {
    keyboard.Key.up: "m",
    keyboard.Key.down: "b",
    keyboard.Key.left: "l",
    keyboard.Key.right: "r",
}

def connect_to_esp():
    global sock, connected
    while not connected:
        try:
            print("ESP32 연결 시도 중...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((esp_ip, port))
            sock.settimeout(None)
            connected = True
            print(" ESP32 연결 성공")
        except Exception as e:
            print(f"연결 실패: {e}, 5초 후 재시도")
            time.sleep(5)

def send(cmd):
    if not connected:
        return
    try:
        sock.sendall((cmd + "\n").encode())
        # print(f"명령 전송: {cmd}")
    except Exception as e:
        print(f"명령 전송 실패: {e}")
        reconnect()

def receive_loop():
    try:
        while connected:
            data = sock.recv(1024).decode('utf-8', errors='ignore')
            for line in data.strip().split('\n'):
                if line.strip() != "":
                    print(f"수신: {line.strip()}")
    except Exception as e:
        print(f"수신 중 연결 끊김: {e}")
        reconnect()

def reconnect():
    global sock, connected, recv_thread
    try:
        sock.close()
    except:
        pass
    connected = False
    print(" 연결 재시도 준비 중...")
    connect_to_esp()
    recv_thread = threading.Thread(target=receive_loop, daemon=True)
    recv_thread.start()

# ⌨️ 방향키 입력 핸들러
def on_press(key):
    if key in key_to_command:
        send(key_to_command[key])
    elif key == keyboard.Key.esc:
        send("s")
        print(" ESC 눌러서 정지 및 종료")
        return False  # stop listener

def on_release(key):
    send("s")  # 방향키 뗐을 때 정지

# 시작
connect_to_esp()
recv_thread = threading.Thread(target=receive_loop, daemon=True)
recv_thread.start()

print("\n▶ 방향키로 제어 (ESC = 종료)\n")

# 방향키 입력 시작
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# 종료
if sock:
    sock.close()
print("프로그램 종료 완료")
