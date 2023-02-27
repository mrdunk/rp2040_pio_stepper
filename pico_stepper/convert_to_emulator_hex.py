#!/usr/bin/env python3
"""
Convert a rp2040 .pio.h file into a .hex file suitable for the RP2040 PIO Emulator.
https://rp2040pio-docs.readthedocs.io/en/latest/index.html
"""

import argparse
import re

prog_name = ""
wrap_target = -1
wrap = -1

def cb_progname(value):
    global prog_name
    prog_name = value.group(1)
    return f'# .program {prog_name}'

def cb_append(value):
    return value.group(1)

def cb_wrap_target(value):
    global wrap_target
    wrap_target = value.group(1)

def cb_wrap(value):
    global wrap
    wrap = value.group(1)

matches = [
        (re.compile('^static const uint16_t (\S+)_program_instructions\[\] = {'), cb_progname),
        (re.compile('^ +0x([0-9a-fA-F][0-9a-fA-F][0-9a-fA-F][0-9a-fA-F]), '), cb_append),
        (re.compile('^#define [\S]+_wrap_target (\d+)$'), cb_wrap_target),
        (re.compile('^#define [\S]+_wrap (\d+)$'), cb_wrap),
        ]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--file_in", help='.pio.h fle to be parsed.')
    parser.add_argument("-o", "--file_out", help='.hex file to be produced', )
    args = parser.parse_args()

    lines = []

    with open(args.file_in, 'r') as file_in:
        for line in file_in:
            for to_match, callback in matches:
                search = to_match.search(line)
                if search:
                    to_write_line = callback(search)
                    if to_write_line:
                        lines.append(to_write_line)

    if args.file_out:
        with open(args.file_out, 'w') as file_out:
            for line in lines:
                file_out.write(f'{line}\n')
        print(f'Wrote {len(lines)} lines to {args.file_out}')
    else:
        for line in lines:
            print(line)
        print()

    print(f'wrap --pio=0 --sm=0 --wrap={hex(int(wrap))} --target={hex(int(wrap_target))}')

if __name__ == "__main__":
    main()
