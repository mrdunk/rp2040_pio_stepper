#!/usr/bin/env python3
"""
Display data logged from test_client/udp.c
"""

import pygame
import collections

FIFO_C_TO_PY = '/tmp/fifo_c_to_py'
FIFO_PY_TO_C = '/tmp/fifo_py_to_c'
MAX_AXIS = 4
DATA_LEN = 100
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 710
UINT_MAX = pow(2, 32)

def get_data(data_chunk):
    with open(FIFO_C_TO_PY, mode='r') as fifo:
        new_data = fifo.read()
        if new_data:
            parse_data(new_data, data_chunk)

def clear_data(data_chunk):
    for axis_data in data_chunk:
        for data in axis_data:
            data.clear()

last_count = 0;
def parse_data(new_data, data_chunk):
    global last_count;

    for entry in new_data.split('|'):
        entry = entry.strip('\x00')
        if not entry:
            continue
        count, axis, data_type, value = entry.split(',')[0:4]
        count = int(count)
        axis = int(axis)
        data_type = int(data_type)
        value = int(value)
        if last_count >= count:
            print(f"wat? {last_count} !< {count}")
        elif count - last_count != 1:
            print(f"wat? {last_count} {count}  {count - last_count}")

        if int(axis) < MAX_AXIS:
            #print(f'{count},\t {axis},\t {data_type},\t {value}')
            data_chunk[axis][data_type].append(value)

        last_count = count

y_offset = [-UINT_MAX / 2 - 5000, -UINT_MAX / 2 - 5000, 0]
y_scale = [0.05, 0.05, 10]
x_scale = 0.25
def convert_coord(screen, data_type, x, y):
    width, height = screen.get_size()
    
    y = y + y_offset[data_type]
    y = y * y_scale[data_type]
    y = int(y + height / 2)
    y = max(0, y)
    y = min(height, y)

    x = int(width - x * x_scale)
    return (x, y)

def scroll_x(screen, offset_x):
    width, height = screen.get_size()
    screen_copy = screen.copy()

    if offset_x > 0:
        screen.set_clip((0, 0, offset_x, height))
        screen.fill("black")
        screen.set_clip(None)
        screen.blit(screen_copy, (offset_x, 0), (0, 0, width, height))
    else:
        screen.set_clip((-offset_x, 0, width + offset_x, height))
        screen.fill("black")
        screen.set_clip(None)
        screen.blit(screen_copy, (0, 0), (width + offset_x, 0, width, height))


def data_count(data_chunk, requested_axis):
    shortest = UINT_MAX
    longest = 0

    for axis, axis_data in enumerate(data_chunk):
        if axis != requested_axis:
            continue

        for data in axis_data:
            shortest = min(shortest, len(data))
            longest = max(longest, len(data))

    return shortest, longest

colors = ["purple", "green", "red"]
def draw_graph(screen, data_chunk, requested_axis):
    shortest, longest = data_count(data_chunk, requested_axis)

    x, y = convert_coord(screen, 0, longest, 0)
    scroll_x(screen, -x)

    count = longest
    data_left = True
    while data_left:
        for axis, axis_data in enumerate(data_chunk):
            if axis != requested_axis:
                continue

            data_left = False
            for data_type, data in enumerate(axis_data):
                if data:
                    data_left = True

            if not data_left:
                break

            for data_type, data in enumerate(axis_data):
                if not data:
                    continue
                value = data.popleft()
                x, y = convert_coord(screen, data_type, count, value)
                #print(axis, data_type, value)
                pygame.Surface.set_at(screen, (x, y), colors[data_type])
        count -= 1


def main():
    data_chunk = []
    for axis in range(MAX_AXIS):
        data_chunk.append((
                collections.deque(),
                collections.deque(),
                collections.deque()
                ))

    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    clock = pygame.time.Clock()
    running = True

    screen.fill("black")

    requested_axis = 2

    # Clear FIFO.
    while True:
        get_data(data_chunk)
        longest, shortest = data_count(data_chunk, requested_axis)
        if longest == shortest:
            break
        clear_data(data_chunk)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        get_data(data_chunk)
        draw_graph(screen, data_chunk, requested_axis)

        pygame.display.flip()
        clock.tick(60)  # limits FPS to 60

if __name__ == "__main__":
    main()

