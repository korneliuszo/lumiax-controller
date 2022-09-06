#!/usr/bin/env python3

import tlay2_client
import struct

def send_onoff(state):
    a=tlay2_client.Tlay2_msg(1)
    a.msg(struct.pack("?",state))
    
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Control power')
    
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-0', '--on', action='store_true')
    group.add_argument('-1', '--off', action='store_true')
    
    args = parser.parse_args()

    if args.on:
        send_onoff(True)
    if args.off:
        send_onoff(False)