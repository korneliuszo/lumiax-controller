#!/usr/bin/env python3

import sys
from PIL import Image, ImageOps
from pathlib import Path
import bdflib.reader

used_set=set()
def add_string(s,str):
    for c in str:
        s.add(c)

add_string(used_set,"1234567890")
add_string(used_set,"Bat")
add_string(used_set,"PV")
add_string(used_set,"Out")
add_string(used_set,"VA%+- .")
add_string(used_set,"/-\\|")
add_string(used_set,"ONOFF")

if "?" in used_set:
    used_set.remove("?")
used_chars = "?"+"".join(sorted(used_set))

bdf_file = Path(sys.argv[1])
bdf=bdflib.reader.read_bdf(open(bdf_file,'rb'))


source = open("src/font.c","w")
header = open("src/font.h","w")

def gen_hdr(tgt):
    tgt.write('unsigned char my_font_[][16] ')


def gen_char(tgt,ch):
    glyph=bdf.glyphs_by_codepoint[ord(ch)]
    
    if glyph.bbH != 16 or glyph.bbW != 8:
            raise Exception("glyph size not supported")
    
    data_raw = bytes(glyph.data)
    
    data = bytearray([255])*16
    for x in range(8):
        for y in range(16):
            if(data_raw[y]&(1<<(7-x))):
                data[x+(1-(y//8))*8] &= ~(1<<(y%8))
    
    tgt.write(" {\n")

    line_buffer = []
    for p in data:
        line_buffer.append("0x%02x,"%p)
        if len(line_buffer) >= 2:
            tgt.write('    %s\n'%(''.join(line_buffer)))
            line_buffer = []
    if len(line_buffer):
        tgt.write('    %s\n'%(''.join(line_buffer)))
    tgt.write(" },\n")

def gen_map_hdr(tgt):
    tgt.write('char my_font_map_[]')
def gen_map_char(tgt,char):
    if char == "\\":
        tgt.write("   '\\\\',\n")
    else:
        tgt.write("   '%s',\n"%char)


source.write("/*autogenerated file*/\n")
gen_hdr(source)
source.write("=\n{\n")
for char in used_chars:
    gen_char(source,char)
source.write("};\n")

gen_map_hdr(source)
source.write("=\n{\n")
for char in used_chars:
    gen_map_char(source,char)
gen_map_char(source,"\\0")
source.write("};\n")


header.write("/*autogenerated file*/\n")
header.write("#ifndef FONT_H\n")
header.write("#define FONT_H\n")
header.write("extern ")
gen_hdr(header)
header.write(";\n")
header.write("extern ")
gen_map_hdr(header)
header.write(";\n")
header.write("#endif\n")

