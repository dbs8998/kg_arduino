
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
    # 거리 텍스트 표시
    dist_text = font.render(f"distance: {current_total_dist:.1f} cm", True, (0, 0, 0))
    screen.blit(dist_text, (50, 30))

    # 버튼 UI
    pygame.draw.rect(screen, (50, 100, 200), connect_btn)
    screen.blit(button_font.render("Connect", True, (255,255,255)), (connect_btn.x+20, connect_btn.y+10))
    pygame.draw.rect(screen, (50, 200, 50), start_btn)
    screen.blit(button_font.render("Start", True, (255,255,255)), (start_btn.x+30, start_btn.y+10))
    pygame.draw.rect(screen, (200, 50, 50), reset_btn)
    screen.blit(button_font.render("Reset", True, (255,255,255)), (reset_btn.x+30, reset_btn.y+10))
    pygame.draw.rect(screen, draw_mode and (100,200,100) or (200,200,200), draw_btn)
    screen.blit(button_font.render("Draw Path", True, (0,0,0)), (draw_btn.x+10, draw_btn.y+10))
    pygame.draw.rect(screen, (200,100,100), clear_btn)
    screen.blit(button_font.render("Clear", True, (255,255,255)), (clear_btn.x+30, clear_btn.y+10))
    pygame.draw.rect(screen, (180, 0, 0), stop_btn)
    screen.blit(button_font.render("STOP", True, (255,255,255)), (stop_btn.x+35, stop_btn.y+10))

    for px, py in path_points:
        pygame.draw.circle(screen, (0, 0, 0), (px, py), 5)
    if len(path_points) > 1:
        pygame.draw.lines(screen, (0, 0, 0), False, path_points, 2)
    if current_pos:
        pygame.draw.circle(screen, (255, 0, 0), current_pos, 6)

    wifi_color = (0, 200, 0) if wifi_status else (180, 180, 180)
    pygame.draw.circle(screen, wifi_color, (750, 40), 15)
    pygame.display.flip()

def send(cmd):
    global sock
    if not sock:
        print("❌ 통신 실패: 아직 연결되지 않았습니다.")
        return
    try:
        sock.sendall((cmd + "\n").encode())
    except Exception as e:
        print(f"Error sending {cmd}: {e}")

def receive_loop():
    global current_total_dist, segment_index, segment_dist_cm, auto_mode, current_pos
    buffer = ""
    while True:
        try:
            data = sock.recv(1024).decode('utf-8', errors='ignore')
            if not data:
                continue
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if line.startswith("dist:"):
                    dist_cm = float(line.split(":")[1])
                    current_total_dist = dist_cm
                    draw_initial_ui()  # 항상 UI 반영
                    if auto_mode and segment_index < len(path_points) - 1:
                        start = path_points[segment_index]
                        end = path_points[segment_index + 1]
                        dx, dy = end[0] - start[0], end[1] - start[1]
                        total_pixel = dist_cm / PIXEL_TO_CM
                        if abs(dx) > abs(dy):
                            cx = start[0] + (1 if dx > 0 else -1) * total_pixel
                            cy = start[1]
                        else:
                            cx = start[0]
                            cy = start[1] + (1 if dy > 0 else -1) * total_pixel
                        current_pos = (int(cx), int(cy))
                    if auto_mode and dist_cm >= segment_dist_cm:
                        send("s")
                        time.sleep(1)
                        segment_index += 1
                        if segment_index >= len(path_points)-1:
                            print("✅ 경로 종료")
                            auto_mode = False
                            current_pos = None
                            return
                        handle_next_segment()
        except Exception as e:
            print(f"⚠️ Receive error: {e}")
            break

def start_receive_thread():
    thread = threading.Thread(target=receive_loop, daemon=True)
    thread.start()

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
    global segment_dist_cm, segment_index, current_total_dist
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
        send("R")
        time.sleep(0.75)
    elif diff == 270:
        send("L")
        time.sleep(0.75)
    elif diff == 180:
        send("L"); time.sleep(0.5); send("L")
    current_heading = target
 
    pixel_dist = math.sqrt(dx**2 + dy**2)
    segment_dist_cm = pixel_dist * PIXEL_TO_CM

    send("s")             # 회전 전에 정지
    time.sleep(1.5)       # 회전 대기

    send("i")             # 거리 초기화
    time.sleep(0.5)       # ✅ 거리 초기화가 완전히 반영될 시간 확보

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
                    send("i")
                    time.sleep(0.2)
                    send("m")
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
