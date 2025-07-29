import socket
import time

esp_ip = "172.30.1.11"
port = 1234

def connect_to_esp():
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((esp_ip, port))
            print("✅ 연결됨!")
            return sock
        except Exception as e:
            print("🔁 재연결 시도 중... (5초 후)", e)
            time.sleep(5)

# 첫 연결 시도
sock = connect_to_esp()

try:
    while True:
        try:
            raw = sock.recv(1024)
            if not raw:
                print("🔌 연결 끊김: 서버에서 종료")
                sock.close()
                sock = connect_to_esp()
                continue

            decoded = raw.decode('utf-8', errors='ignore').strip()
            if decoded:
                print("📥 Arduino → Python:", decoded)

        except socket.timeout:
            print("⏱️ 응답 없음 (타임아웃)")

        except (ConnectionResetError, BrokenPipeError) as e:
            print("❌ 연결 오류 발생:", e)
            sock.close()
            sock = connect_to_esp()
            continue

        # 사용자 입력 → 아두이노 전송
        try:
            msg = input("Arduino에 보낼 명령 입력: ")
            sock.sendall((msg + "\n").encode())
        except (BrokenPipeError, OSError) as e:
            print("⚠️ 전송 중 오류 발생:", e)
            sock.close()
            sock = connect_to_esp()

except KeyboardInterrupt:
    print("⛔ 사용자 종료")
    sock.close()
