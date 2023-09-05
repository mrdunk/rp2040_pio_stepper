#!/usr/bin/env python3
"""
Display data logged from test_client/udp.c
"""

import pygame
import aiofiles
import asyncio

FIFO_C_TO_PY = '/tmp/fifo_c_to_py'
FIFO_PY_TO_C = '/tmp/fifo_py_to_c'

async def get_data():
    async with aiofiles.open(FIFO_C_TO_PY, mode='r') as f:
        contents = await f.read()
        parse_data(contents)

last_count = 0;
progres = 0
def parse_data(data):
    global last_count;
    global progres

    for entry in data.split('|'):
        entry = entry.strip('\x00')
        if not entry:
            continue
        count, axis, key, value = entry.split(',')[0:4]
        count = int(count)
        if last_count >= count:
            print(f"wat? {last_count} !< {count}")
        elif abs(count - last_count) != 1:
            print(f"wat? {last_count} {count}  {count - last_count}")
        else:
            progres += 1

        if progres > 100:
            print(".")
            progres = 0

        print(f'{count},\t {axis},\t {key},\t {value}')

        last_count = count

    #assert ((len(data) % 4) == 0)
    #count = data[0]
    #axis = data[1]
    #key = data[2]
    #value = data[3]
    #print(f'{count},\t {axis},\t {key},\t {value}')


def main():
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    clock = pygame.time.Clock()
    running = True


    while running:
        #asyncio.run(get_data())

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill("purple")

        with open(FIFO_C_TO_PY, mode='r') as fifo:
            data = fifo.read()
            if len(data) == 0:
               continue
            parse_data(data)
            #print(f'Read: "{data}"')

        pygame.display.flip()
        clock.tick(60)  # limits FPS to 60

if __name__ == "__main__":
    main()

