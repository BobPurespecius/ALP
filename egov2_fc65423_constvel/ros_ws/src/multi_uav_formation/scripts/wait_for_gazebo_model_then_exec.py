#!/usr/bin/env python3

"""Start a command only after its Gazebo model has finished spawning."""

import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates


def main():
    if len(sys.argv) < 4:
        print(
            "usage: wait_for_gazebo_model_then_exec.py MODEL TIMEOUT COMMAND [ARG ...]",
            file=sys.stderr,
        )
        return 2

    model_name = sys.argv[1]
    try:
        timeout = float(sys.argv[2])
    except ValueError:
        print(f"invalid timeout: {sys.argv[2]}", file=sys.stderr)
        return 2

    command = sys.argv[3:]
    rospy.init_node(
        f"wait_for_{model_name}",
        anonymous=True,
        disable_signals=True,
        argv=[sys.argv[0]],
    )

    deadline = time.monotonic() + timeout
    print(f"waiting up to {timeout:g}s for Gazebo model '{model_name}'", flush=True)
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            print(
                f"timed out waiting for Gazebo model '{model_name}'",
                file=sys.stderr,
                flush=True,
            )
            return 1

        try:
            states = rospy.wait_for_message(
                "/gazebo/model_states",
                ModelStates,
                timeout=min(remaining, 2.0),
            )
        except rospy.ROSException:
            continue

        if model_name in states.name:
            print(f"Gazebo model '{model_name}' is ready; starting PX4", flush=True)
            os.execvp(command[0], command)

    return 1


if __name__ == "__main__":
    sys.exit(main())
