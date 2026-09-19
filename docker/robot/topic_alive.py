#!/usr/bin/env python3
"""Touch a file whenever a message arrives on a topic — so a supervisor can tell a node that is
RUNNING from one that is WORKING.

    topic-alive /mouse/scan /dev/shm/scan.alive

Written for a Kubernetes livenessProbe, after an RPLIDAR A1 hung in its opening handshake: the
driver printed its banner and then nothing, for ever, while the process stayed up and the pod
stayed Ready. No exit code, no log line, nothing to restart on. Message arrival is the only honest
signal, and a probe that starts a ROS node every few seconds costs a Pi real CPU; checking one
file's mtime costs nothing:

    [ $(( $(date +%s) - $(stat -c %Y /dev/shm/scan.alive 2>/dev/null || echo 0) )) -lt 20 ]

The file is absent until the FIRST message, so the same test also catches a node that never
started publishing. Type-agnostic: the message type is discovered from the graph.
"""
import pathlib
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    topic, stamp = sys.argv[1], pathlib.Path(sys.argv[2])
    rclpy.init()
    node = Node('topic_alive_' + re.sub(r'[^A-Za-z0-9]+', '_', topic).strip('_'))

    msg_type = None
    while rclpy.ok() and msg_type is None:  # the publisher may not exist yet; wait for it
        for name, types in node.get_topic_names_and_types():
            if name == topic:
                msg_type = get_message(types[0])
        if msg_type is None:
            rclpy.spin_once(node, timeout_sec=1.0)
    node.get_logger().info(f'watching {topic} -> {stamp}')

    last = 0.0

    def on_message(_):
        nonlocal last
        now = time.monotonic()
        if now - last >= 1.0:  # once a second is plenty, whatever the topic's rate
            stamp.touch()
            last = now

    # Sensor-data QoS (best effort) is compatible with both reliable and best-effort publishers.
    node.create_subscription(msg_type, topic, on_message, qos_profile_sensor_data)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
