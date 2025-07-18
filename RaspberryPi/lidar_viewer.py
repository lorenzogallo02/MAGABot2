import math
import pygame
import serial
import struct
import time

# === CONFIGURATION ===
PORT = '/dev/ttyUSB0'
BAUD = 230400
W, H = 800, 800
MAX_RANGE_M = 8.0
POINT_FADE = 100
MIN_VALID_DIST =0.8  # Ignore distances below this (ghost filter)

# === INITIALIZATION ===
pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Enhanced T-mini Pro LIDAR Viewer")
clock = pygame.time.Clock()

ser = serial.Serial(PORT, BAUD, timeout=1)

def start_scan():
    ser.write(b'\xA5\x60')
    time.sleep(0.1)

def stop_scan():
    ser.write(b'\xA5\x65')
    time.sleep(0.1)

def read_packet():
    while True:
        sync1 = ser.read(1)
        if sync1 != b'\xAA':
            continue
        sync2 = ser.read(1)
        if sync2 != b'\x55':
            continue
        header = sync1 + sync2 + ser.read(6)
        if len(header) < 8:
            continue
        count = header[3]
        data_len = count * 3
        data = ser.read(data_len + 2)
        if len(data) < data_len + 2:
            continue
        return header + data

def parse_packet(packet):
    if not packet or len(packet) < 10:
        return []
    count = packet[3]
    angle_start = struct.unpack('<H', packet[4:6])[0] / 100.0
    angle_end = struct.unpack('<H', packet[6:8])[0] / 100.0
    angle_diff = (angle_end - angle_start + 360) % 360
    points = []
    for i in range(count):
        base = 8 + i * 3
        distance = struct.unpack('<H', packet[base:base+2])[0] / 1000.0
        angle = (angle_start + angle_diff * i / count) % 360
        if MIN_VALID_DIST <= distance <= MAX_RANGE_M:
            points.append((angle, distance))
    return points

def polar_to_cartesian(angle_deg, distance_m):
    rad = math.radians(angle_deg)
    x = distance_m * math.cos(rad)
    y = distance_m * math.sin(rad)
    return x, y

def world_to_screen(x, y):
    scale = (W // 2) / MAX_RANGE_M
    screen_x = int(W // 2 + x * scale)
    screen_y = int(H // 2 - y * scale)
    return screen_x, screen_y

def draw_grid():
    cx, cy = W // 2, H // 2
    for r in range(1, int(MAX_RANGE_M) + 1):
        pygame.draw.circle(screen, (40, 40, 40), (cx, cy), int(r * (W // 2) / MAX_RANGE_M), 1)
        font = pygame.font.SysFont(None, 18)
        label = font.render(f"{r}m", True, (100, 100, 100))
        screen.blit(label, (cx + 5, cy - int(r * (W // 2) / MAX_RANGE_M)))

    for a in range(0, 360, 30):
        rad = math.radians(a)
        x = cx + int((W // 2) * math.cos(rad))
        y = cy - int((H // 2) * math.sin(rad))
        pygame.draw.line(screen, (50, 50, 50), (cx, cy), (x, y), 1)
        font = pygame.font.SysFont(None, 18)
        label = font.render(f"{a}°", True, (120, 120, 120))
        lx = cx + int((W // 2 + 10) * math.cos(rad))
        ly = cy - int((H // 2 + 10) * math.sin(rad))
        screen.blit(label, (lx - 10, ly - 10))

# === MEMORY OF RECENT POINTS ===
point_memory = []
sweep_angle = 0

# === MAIN LOOP ===
try:
    start_scan()
    print("✅ Enhanced T-mini Pro scanning... Press Ctrl+C to stop.")

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        packet = read_packet()
        scan_points = parse_packet(packet)

        new_points = []
        for angle, dist in scan_points:
            x, y = polar_to_cartesian(angle, dist)
            new_points.append({'x': x, 'y': y, 'lifetime': POINT_FADE})

        point_memory.extend(new_points)

        for pt in point_memory:
            pt['lifetime'] -= 1
        point_memory = [pt for pt in point_memory if pt['lifetime'] > 0]

        screen.fill((0, 0, 0))
        draw_grid()

        for pt in point_memory:
            sx, sy = world_to_screen(pt['x'], pt['y'])
            alpha = int(255 * pt['lifetime'] / POINT_FADE)
            color = (0, alpha, 255)
            pygame.draw.circle(screen, color, (sx, sy), 2)

        pygame.draw.circle(screen, (255, 0, 0), (W // 2, H // 2), 4)

        # Rotating red sweep line
        sweep_angle = (sweep_angle + 2) % 360
        rad = math.radians(sweep_angle)
        cx, cy = W // 2, H // 2
        ex = cx + int((W // 2) * math.cos(rad))
        ey = cy - int((H // 2) * math.sin(rad))
        pygame.draw.line(screen, (255, 0, 0), (cx, cy), (ex, ey), 1)

        pygame.display.flip()
        clock.tick(30)

except KeyboardInterrupt:
    print("🛑 Stopping enhanced viewer...")
    stop_scan()
    ser.close()
    pygame.quit()
