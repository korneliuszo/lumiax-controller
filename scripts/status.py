#!/usr/bin/env python3

import tlay2_client
import struct

def get_state():
    a=tlay2_client.Tlay2_msg(3)
    keys=("b_soc","b_v","b_a","l_v","l_a","s_v","s_a","on")
    vals=struct.unpack("<HHhHHHH?",a.msg(b""))
    ratio = (1,100,100,100,100,100,100,"I")
    vals = [val/r if r != "I" else val for val,r in zip(vals,ratio)]
    return dict(zip(keys,vals))

if __name__ == "__main__":
    print(get_state())