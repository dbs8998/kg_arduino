'''
MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com
     
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
     
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
     
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.
'''

import socket
import pygame

# ESP32 서버의 IP 주소 및 포트 번호 설정
server_ip = '172.30.1.40'  # ESP32의 IP 주소 (Wi-Fi 연결 시 확인)
port = 8080                  # ESP32에서 설정한 포트 번호

# 소켓 생성 (IPv4, TCP)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 서버에 연결
try:
    client_socket.connect((server_ip, port))
    print(f"서버 {server_ip}:{port}에 연결 성공")
except Exception as e:
    print(f"서버에 연결 실패: {e}")
    exit()

# Pygame 초기화
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("ESP32 통신")

# 프로그램 실행 플래그
running = True

def handle_keypress():
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        print("forward")
        return "w\n"
    elif keys[pygame.K_s]:
        print("backward")
        return "s\n"
    elif keys[pygame.K_a]:
        print("left")
        return "a\n"
    elif keys[pygame.K_d]:
        print("right")
        return "d\n"
    elif keys[pygame.K_SPACE]:
        print("stop")
        return " \n"
    return None

try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
       
        # 키 상태 확인 및 처리
        message = handle_keypress()
        if message:
            # 메시지 전송
            client_socket.send(message.encode())
            print(f"전송: {message.strip()}")

            # 서버로부터의 응답 수신
            response = client_socket.recv(1024)
            print(f"서버 응답: {response.decode()}")

finally:
    # 소켓 종료 및 pygame 종료
    client_socket.close()
    pygame.quit()
