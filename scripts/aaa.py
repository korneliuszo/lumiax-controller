#!/usr/bin/env python3


import tlay2_client
import socket

conn = tlay2_client.Tlay2_out(0)

while True:
    packet = conn.recv()
    if packet == b'[SUBSCRIBED]\n':
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.connect(("127.0.0.1",4444))
        s.send(b"reset\n")
        s.close()
