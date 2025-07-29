
import pygame
import socket
import threading
import sys
import math
import time

# ====== 설정 ======
HOST = "172.30.1.11"
PORT = 1234

# ====== pygame 초기화 & UI 세팅 ======
pygame.init()
SCREEN_W, SCREEN_H = 800, 600
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("RC Car Control")

font = pygame.font.SysFont(None, 36)
button_font = pygame.font.SysFont(None, 24)

path_points = []
draw_mode   = False
wifi_status = False
sock = None

BUTTON_W = 120
BUTTON_H = 40
BUTTON_X = 650
BUTTON_Y_START = 280
BUTTON_GAP = 10

connect_btn = pygame.Rect(BUTTON_X, BUTTON_Y_START + 0*(BUTTON_H + BUTTON_GAP), BUTTON_W, BUTTON_H)
start_btn   = pygame.Rect(BUTTON_X, BUTTON_Y_START + 1*(BUTTON_H + BUTTON_GAP), BUTTON_W, BUTTON_H)
reset_btn   = pygame.Rect(BUTTON_X, BUTTON_Y_START + 2*(BUTTON_H + BUTTON_GAP), BUTTON_W, BUTTON_H)
draw_btn    = pygame.Rect(BUTTON_X, BUTTON_Y_START + 3*(BUTTON_H + BUTTON_GAP), BUTTON_W, BUTTON_H)
clear_btn   = pygame.Rect(BUTTON_X, BUTTON_Y_START + 4*(BUTTON_H + BUTTON_GAP), BUTTON_W, BUTTON_H)
stop_btn    = pygame.Rect(BUTTON_X, BUTTON_Y_START + 5*(BUTTON_H + BUTTON_GAP), BUTTON_W, BUTTON_H)

started = False
PIXEL_TO_CM = 0.5

auto_mode = False
segment_index = 0
segment_dist_cm = 0
current_heading = 90
current_total_dist = 0
current_pos = None

def draw_initial_ui():
    screen.fill((255, 255, 255))
    dist_text = font.render(f"distance: {current_total_dist:.1f} cm", True, (0, 0, 0))
    screen.blit(dist_text, (50, 30))
    for btn, label, color in [
        (connect_btn, "Connect", (50, 100, 200)),
        (start_btn, "Start", (50, 200, 50)),
        (reset_btn, "Reset", (200, 50, 50)),
        (draw_btn, "Draw Path", draw_mode and (100,200,100) or (200,200,200)),
        (clear_btn, "Clear", (200,100,100)),
        (stop_btn, "STOP", (180, 0, 0))
    ]:
        pygame.draw.rect(screen, color, btn)
        screen.blit(button_font.render(label, True, (255,255,255)), (btn.x+20, btn.y+10))
    for px, py in path_points:
        pygame.draw.circle(screen, (0, 0, 0), (px, py), 5)
    if len(path_points) > 1:
        pygame.draw.lines(screen, (0, 0, 0), False, path_points, 2)
    if current_pos:
        pygame.draw.circle(screen, (255, 0, 0), current_pos, 6)
    wifi_color = (0, 200, 0) if wifi_status else (180, 180, 180)
    pygame.draw.circle(screen, wifi_color, (750, 40), 15)
    pygame.display.flip()

def send(cmd, wait=0.0, retry=3):
    global sock
    if not sock:
        print("❌ 통신 실패: 연결되지 않음")
        return
    for i in range(retry):
        try:
            sock.sendall((cmd + "\n").encode())
            print(f"📤 전송: {cmd}")
            break
        except Exception as e:
            print(f"⚠️ send 실패: {e}")
            time.sleep(0.2)
    if wait > 0:
        time.sleep(wait)

def receive_loop():
    global current_total_dist, segment_index, segment_dist_cm
    global auto_mode, current_pos, sock

    buffer = ""
    TOLERANCE_CM = 3.0
    MAX_DIST_JUMP = 50

    while True:
        try:
            data = sock.recv(1024).decode('utf-8', errors='ignore')
            if not data:
                continue
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if line.startswith("dist:"):
                    dist_cm = float(line.split(":")[1])
                    if abs(dist_cm - current_total_dist) > MAX_DIST_JUMP:
                        print(f"⚠️ 거리값 튐 감지: {dist_cm} → 무시")
                        continue
                    current_total_dist = dist_cm
                    if auto_mode and segment_index < len(path_points) - 1:
                        start = path_points[segment_index]
                        end = path_points[segment_index + 1]
                        dx, dy = end[0] - start[0], end[1] - start[1]
                        pixel_dist = math.sqrt(dx**2 + dy**2)
                        total_pixel = min(dist_cm / PIXEL_TO_CM, pixel_dist)
                        cx = start[0] + (1 if dx > 0 else -1) * total_pixel if dx != 0 else start[0]
                        cy = start[1] + (1 if dy > 0 else -1) * total_pixel if dy != 0 else start[1]
                        current_pos = (int(cx), int(cy))
                    draw_initial_ui()
                    if auto_mode and abs(dist_cm - segment_dist_cm) <= TOLERANCE_CM:
                        print(f"📏 도착 판정: 현재거리 {dist_cm:.2f}, 목표거리 {segment_dist_cm:.2f}")
                        send("s")
                        time.sleep(1)
                        segment_index += 1
                        if segment_index >= len(path_points) - 1:
                            print("✅ 경로 종료")
                            auto_mode = False
                            current_pos = None
                            return
                        current_pos = path_points[segment_index]
                        draw_initial_ui()
                        time.sleep(0.5)
                        handle_next_segment()
        except Exception as e:
            print(f"❌ receive_loop 오류: {e}")
            reconnect_to_esp()
            break


def start_receive_thread():
    thread = threading.Thread(target=receive_loop, daemon=True)
    thread.start()

def reconnect_to_esp():
    global sock, wifi_status
    try:
        print("🔁 ESP32 재연결 시도 중...")
        sock.close()
    except:
        pass
    wifi_status = False
    draw_initial_ui()
    time.sleep(1)
    connect_to_esp()

def connect_to_esp(host=HOST, port=PORT, retry=5):
    global wifi_status, sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for _ in range(retry):
        try:
            sock.connect((host, port))
            print("✅ ESP32 connected")
            wifi_status = True
            draw_initial_ui()
            start_receive_thread()
            return
        except socket.error:
            pygame.time.delay(500)
    print("❌ Connection failed")
    wifi_status = False
    draw_initial_ui()

def prepare_segments():
    global segment_index, current_total_dist
    segment_index = 0
    current_total_dist = 0
    if len(path_points) >= 2:
        x1, y1 = path_points[0]
        x2, y2 = path_points[1]
        dx, dy = x2 - x1, y2 - y1
        pixel_dist = math.sqrt(dx**2 + dy**2)
        segment_dist_cm = pixel_dist * PIXEL_TO_CM

def handle_next_segment():
    global segment_dist_cm, current_heading, current_total_dist
    x1, y1 = path_points[segment_index]
    x2, y2 = path_points[segment_index+1]
    dx, dy = x2 - x1, y2 - y1
    if abs(dx) > abs(dy):
        target = 0 if dx > 0 else 180
    else:
        target = 90 if dy > 0 else 270
    diff = (target - current_heading + 360) % 360
    send("s")
    time.sleep(0.5)
    if diff == 90:
        # send("R")
        # time.sleep(0.75)
        send("R", wait=0.75)
    elif diff == 270:
        # send("L")
        # time.sleep(0.75)
        send("L", wait=0.75)
    elif diff == 180:
        send("L"); time.sleep(0.5); send("L")
    current_heading = target
 
    pixel_dist = math.sqrt(dx**2 + dy**2)
    segment_dist_cm = pixel_dist * PIXEL_TO_CM

    # send("s")             # 회전 전에 정지
    # time.sleep(1.5)       # 회전 대기
    send("s", wait=1.5)
    # send("i")             # 거리 초기화
    # time.sleep(0.5)       # ✅ 거리 초기화가 완전히 반영될 시간 확보
    send("i", wait=0.5)
    current_total_dist = 0  # 🔁 명시적 초기화
    send("m")             # 직진 시작

def reset_autonomous_state():
    global auto_mode, segment_index, segment_dist_cm, current_heading
    global current_total_dist, current_pos
    auto_mode = False
    segment_index = 0
    segment_dist_cm = 0
    current_heading = 90
    current_total_dist = 0
    current_pos = None


draw_initial_ui()
clock = pygame.time.Clock()

while True:
    for evt in pygame.event.get():
        if evt.type == pygame.QUIT:
            if sock:
                sock.close()
            pygame.quit()
            sys.exit()
        elif evt.type == pygame.MOUSEBUTTONDOWN:
            if connect_btn.collidepoint(evt.pos):
                connect_to_esp()
            elif reset_btn.collidepoint(evt.pos):
                send("i")
                current_total_dist = 0
                draw_initial_ui()
            elif start_btn.collidepoint(evt.pos):
                if wifi_status and len(path_points) >= 2:
                    reset_autonomous_state()
                    auto_mode = True
                    prepare_segments()
                    handle_next_segment()  # ✅ 첫 경로도 이 함수에서 시작
            elif stop_btn.collidepoint(evt.pos):
                auto_mode = False
                current_pos = None
                send("s")
                draw_initial_ui()
            elif draw_btn.collidepoint(evt.pos):
                draw_mode = not draw_mode
                draw_initial_ui()
            elif clear_btn.collidepoint(evt.pos):
                path_points.clear()
                reset_autonomous_state()
                draw_initial_ui()
            elif draw_mode and not any(btn.collidepoint(evt.pos) for btn in [connect_btn, reset_btn, start_btn, draw_btn, clear_btn, stop_btn]):
                x, y = evt.pos
                if path_points:
                    last_x, last_y = path_points[-1]
                    dx = abs(x - last_x)
                    dy = abs(y - last_y)
                    if dx > dy:
                        y = last_y
                    else:
                        x = last_x
                path_points.append((x, y))
                draw_initial_ui()
    if current_pos:
        draw_initial_ui()
    clock.tick(30)
