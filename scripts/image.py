#!/usr/bin/env python3

import tlay2_client
import struct
from PIL import Image

def send_image(img):
    img = img.convert("1")
    w,h = img.size
    if w != 200 or h != 128:
        raise Exception("bad format")


    data = []
    for i in range(h//8):
        data.append(bytearray([255])*w)
    for x in range(w):
        for y in range(h):
            if img.getpixel((x,y)) == 0:
                data[y//8][x] &= ~(1<<(7-y%8))
    
    a=tlay2_client.Tlay2_msg(2)
    for i in range(len(data)):
        line = data[i]
        for j in range(0, len(line), 64):
            chunk = line[j:j+64]
            a.msg(struct.pack("<H",96+j+i*(296//8)*8)+chunk)
    
if __name__ == "__main__":
    import sys
    send_image(Image.open(sys.argv[1]))
