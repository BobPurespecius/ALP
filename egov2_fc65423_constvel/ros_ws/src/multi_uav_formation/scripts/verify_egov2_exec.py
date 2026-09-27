#!/usr/bin/env python3

import os
import subprocess
import sys


def main():
    if len(sys.argv) < 4:
        print('verify_egov2_exec.py requires EXPECTED_PACKAGE_DIR, EXPECTED_BINARY_DIR and a command', file=sys.stderr)
        return 64

    expected = os.path.realpath(sys.argv[1])
    expected_binary_dir = os.path.realpath(sys.argv[2])
    try:
        resolved = subprocess.check_output(
            ['rospack', 'find', 'ego_planner'],
            stderr=subprocess.STDOUT,
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        print('Cannot resolve ego_planner: {}'.format(exc), file=sys.stderr)
        return 69

    resolved = os.path.realpath(resolved)
    if resolved != expected:
        print('Refusing to start mismatched ego_planner binary.', file=sys.stderr)
        print('Resolved: {}'.format(resolved), file=sys.stderr)
        print('Expected: {}'.format(expected), file=sys.stderr)
        print('Source the EGO-Planner-v2 tracking workspace before launching.', file=sys.stderr)
        return 78

    command = sys.argv[3:]
    resolved_binary_dir = os.path.dirname(os.path.realpath(command[0]))
    if resolved_binary_dir != expected_binary_dir:
        print('Refusing to start ego_planner from a mismatched devel space.', file=sys.stderr)
        print('Resolved binary: {}'.format(os.path.realpath(command[0])), file=sys.stderr)
        print('Expected directory: {}'.format(expected_binary_dir), file=sys.stderr)
        print('Put the EGO-Planner-v2 tracking devel space first in CMAKE_PREFIX_PATH.', file=sys.stderr)
        return 78
    os.execv(command[0], command)


if __name__ == '__main__':
    sys.exit(main())
