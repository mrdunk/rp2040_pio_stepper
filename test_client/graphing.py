#!/usr/bin/env python3
"""
Display data logged from test_client/udp.c.
Gets and sends data via Linux FIFO. (FIFOs are file like objects.)
"""

import pygame
import collections
import struct

FIFO_C_TO_PY = '/tmp/fifo_c_to_py'
FIFO_PY_TO_C = '/tmp/fifo_py_to_c'
MAX_AXIS = 4
DATA_LEN = 100
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 710
MENU_HEIGHT = 50
UINT_MAX = pow(2, 32)

def get_data(data_chunk):
    """ Get data from FIFO. """
    with open(FIFO_C_TO_PY, mode='rb') as fifo:
        new_data = fifo.read()
        if new_data:
            parse_data(new_data, data_chunk)

def send_data(menu_selected):
    """ Send data to FIFO. """

    if menu_selected == 0:
        return

    serialised = struct.pack('cf', bytes([menu_selected - 1]), menu_data[menu_selected])
    print(serialised)
    print(len(serialised))
    with open(FIFO_PY_TO_C, mode='wb') as fifo:
        new_data = fifo.write(serialised)

def clear_data(data_chunk):
    for axis_data in data_chunk:
        for data in axis_data:
            data.clear()

last_count = 0;
def parse_data(new_data, data_chunk):
    """
    De-serialize data that arrived on the FIFO.
    """
    global last_count;

    for index in range(0, len(new_data), 10):
        count = struct.unpack('I', new_data[index:index + 4])[0]
        axis = int(new_data[index + 4])
        data_type = int(new_data[index + 5])
        if(data_type == 2):
            value = struct.unpack('i', new_data[index + 6:index + 10])[0]
        else:
            value = struct.unpack('I', new_data[index + 6:index + 10])[0]

        if last_count >= count:
            print(f"Out of sequence: {last_count} !< {count}")
        elif count - last_count != 1:
            print(f"Missing updates: {last_count} {count}  {count - last_count}")

        if int(axis) < MAX_AXIS:
            #print(len(new_data[index:]), count, axis, data_type, value)
            data_chunk[axis][data_type].append(value)

        last_count = count

y_offset = [-UINT_MAX / 2 - 5000, -UINT_MAX / 2 - 5000, 0]
y_scale = [0.05, 0.05, 10]
x_scale = 0.25
def convert_coord(screen, data_type, x, y):
    """
    Convert data into something that fits between the graph axis.
    """
    width, height = screen.get_size()
    
    y = y + y_offset[data_type]
    y = y * y_scale[data_type]
    y = int(y + height / 2)
    y = max(0, y)
    y = min(height, y)

    x = int(width - x * x_scale)
    return (x, y)


def data_count(data_chunk, requested_axis):
    """
    Count how much data exists.
    Returns:
        (shortest, longest): shortest: The axis with least data in it.
                             longest: The axis with most data in it.
    """
    shortest = UINT_MAX
    longest = 0

    for axis, axis_data in enumerate(data_chunk):
        if axis != requested_axis:
            continue

        for data in axis_data:
            shortest = min(shortest, len(data))
            longest = max(longest, len(data))

    return shortest, longest

def scroll_x(screen, offset_x):
    """
    Scroll the graph display along the x axis.
    """
    width, height = screen.get_size()
    screen_copy = screen.copy().convert()

    if offset_x > 0:
        screen.set_clip((0, 0, offset_x, height - MENU_HEIGHT))
        screen.fill("black")
        screen.set_clip(None)
        screen.blit(screen_copy, (offset_x, 0), (0, 0, width, height - MENU_HEIGHT))
    else:
        screen.set_clip((-offset_x, 0, width + offset_x, height - MENU_HEIGHT))
        screen.fill("black")
        screen.set_clip(None)
        screen.blit(screen_copy, (0, 0), (width + offset_x, 0, width, height - MENU_HEIGHT))

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


def draw_button(screen, font, index, value, selected):
    y_offset = 10
    colour = "grey"
    if selected:
        colour = "white"

    pygame.draw.rect(
            screen,
            "white",
            (index * MENU_HEIGHT, SCREEN_HEIGHT - MENU_HEIGHT + y_offset,
                MENU_HEIGHT, MENU_HEIGHT)
            )
    pygame.draw.rect(
            screen,
            colour,
            (index * MENU_HEIGHT, SCREEN_HEIGHT - MENU_HEIGHT + y_offset,
                MENU_HEIGHT, MENU_HEIGHT),
            width = 2)

    colour = "green"
    if selected:
        colour = "red"
    text = font.render(f'{value:.3}', True, colour)
    screen.blit(text, (index * MENU_HEIGHT, SCREEN_HEIGHT - MENU_HEIGHT + y_offset))


menu_data = [1.0, 0.1, 0.0, 0.0]
menu_update_size = [1.0, 0.01, 0.001, 0.01]
menu_min_max = [(0.0, float(MAX_AXIS - 1)), None, None, None]
menu_selected = 0

def clamp_menu(menu_selected):
    data = round(menu_data[menu_selected], 4)
    if menu_min_max[menu_selected] is None:
        return data

    data = min(data, menu_min_max[menu_selected][1])
    data = max(data, menu_min_max[menu_selected][0])

    return data

def draw_menu(screen, events):
    global menu_selected
    menu_changed = False

    pygame.key.set_repeat = 0


    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                menu_selected -= 1
                menu_changed = True
            elif event.key == pygame.K_RIGHT:
                menu_selected += 1
                menu_changed = True
            elif event.key == pygame.K_UP:
                menu_data[menu_selected] += menu_update_size[menu_selected]
                menu_changed = True
                menu_data[menu_selected] = clamp_menu(menu_selected)
            elif event.key == pygame.K_DOWN:
                menu_data[menu_selected] -= menu_update_size[menu_selected]
                menu_changed = True
                menu_data[menu_selected] = clamp_menu(menu_selected)
    menu_selected = menu_selected % len(menu_data)


    font = pygame.font.Font(None, int(MENU_HEIGHT / 2))
    for pos, data in enumerate(menu_data):
        draw_button(screen, font, pos, menu_data[pos], menu_selected == pos)

    if menu_changed:
        send_data(menu_selected);

    return menu_data[0]

def main():
    print(pygame.version.ver)

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

    screen.fill("white")

    requested_axis = 1

    # Clear FIFO.
    while True:
        get_data(data_chunk)
        longest, shortest = data_count(data_chunk, requested_axis)
        if longest == shortest:
            break
        clear_data(data_chunk)

    while running:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                running = False

        get_data(data_chunk)
        draw_graph(screen, data_chunk, requested_axis)
        requested_axis = draw_menu(screen, events)

        pygame.display.flip()
        clock.tick(60)  # limits FPS to 60

if __name__ == "__main__":
    main()

