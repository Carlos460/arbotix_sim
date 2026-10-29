#!/usr/bin/env python3
"""
    Send fake joint packets to udp_joint_bridge: every joint swings in a slow sine dance.

    Usage: python3 send_test_joints.py [host] [port]   (default 127.0.0.1 15000)
"""
import math
import socket
import struct
import sys
import time

host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 15000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

start = time.time()
while True:
    angle = 0.4 * math.sin(time.time() - start)
    # 18 records of (int32 joint_id, float32 joint_position), native byte order
    packet = b"".join(struct.pack("=if", joint_id, angle) for joint_id in range(1, 19))
    sock.sendto(packet, (host, port))
    time.sleep(0.02)
