import socket
import struct
import threading

import pygame

pygame.init()

WIDTH, HEIGHT = 1300, 20
LED_SIZE = 2
GLOW_SIZE = 8
LED_GAP = 10
LED_COUNT = 100
FPS = 60

UDP_IP = "127.0.0.1"
UDP_PORT = 6868

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))


LED_COUNT = 100

led_buffer = [((0, 0, 0), 0)] * LED_COUNT
led_buffer_lock = threading.Lock()


def udp_server():
    while True:
        data, addr = sock.recvfrom(1472)
        with led_buffer_lock:
            for i in range(2, len(data) - 2, 7):
                chunk = data[i : i + 7]
                port, index, r, g, b, w = struct.unpack("<BHBBBB", chunk)
                led_buffer[index] = ((r, g, b), w)


server_thread = threading.Thread(target=udp_server)
server_thread.daemon = True
server_thread.start()


screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.SRCALPHA)

clock = pygame.time.Clock()


running = True
while running:
    screen.fill((0, 0, 0))

    with led_buffer_lock:
        for i, (color, brightness) in enumerate(led_buffer):
            x = 10 + i * (LED_SIZE + LED_GAP)
            y = 10

            brightness = (brightness / 255) ** 0.5

            color = tuple(int(c * brightness) for c in color)

            glow = pygame.Surface((GLOW_SIZE * 2, GLOW_SIZE * 2), pygame.SRCALPHA)
            pygame.draw.circle(glow, (*color, 50), (GLOW_SIZE, GLOW_SIZE), GLOW_SIZE)
            screen.blit(glow, (x - GLOW_SIZE, y - GLOW_SIZE))

            led = pygame.Surface((LED_SIZE * 2, LED_SIZE * 2), pygame.SRCALPHA)
            pygame.draw.circle(led, (*color, 255), (LED_SIZE, LED_SIZE), LED_SIZE)
            screen.blit(led, (x - LED_SIZE, y - LED_SIZE))

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    clock.tick(FPS)

pygame.quit()
