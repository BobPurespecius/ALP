#!/usr/bin/env python3

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET


def parse_bool(value):
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise argparse.ArgumentTypeError('expected a boolean value')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--disable-livox-ray', type=parse_bool, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()

    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('missing SDF generator command after --')

    result = subprocess.run(command, stdout=subprocess.PIPE)
    if result.returncode != 0:
        return result.returncode

    if not args.disable_livox_ray:
        sys.stdout.buffer.write(result.stdout)
        return 0

    root = ET.fromstring(result.stdout)
    for parent in root.iter():
        for child in list(parent):
            if child.tag.rsplit('}', 1)[-1] == 'sensor' and child.get('name') == 'laser_livox':
                parent.remove(child)

    sys.stdout.buffer.write(ET.tostring(root, encoding='utf-8', xml_declaration=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
