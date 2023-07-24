# -*- coding: utf-8 -*-
"""
Roch Schanen
Conversion of "8x15_.txt"
into C++ code
"""

from __future__ import print_function

print("const uint8_t microlib_SSD1306::fu[][8] PROGMEM = {");

n = None;

file = open("./8x15_.txt", "r");

for line in file:

    if line.find("GLYPH") == 0:
        i = 0;
        Bytes = [0]*8;
        n = int(line[6:10], 16);

    elif line.find("ENDGLYPH") == 0:
        print('\t{', end='');
        for j in range(8):
            print('0x', end='');
            print(format(Bytes[j], '02x'), end='');
            if j<7: print(', ', end='');
        print('},');
        n = None;

#    elif n == 1: break;

    elif not n == None:
        if len(line)>8:
            if i<8:
                # horizontal scan (text line 8 chars)
                for j in range(8):
                    # vertical encoding (8 bits)
                    Bytes[j] >>= 1;
                    if line[j+1] == 'X': Bytes[j] += 0x80;
#                    print(line[j+1], end='');
#                print();
            i+=1;

file.close();
print("};");
print("// remove last coma manually");

print();

print("const uint8_t microlib_SSD1306::fl[][8] PROGMEM = {");

n = None;

file = open("./8x15_.txt", "r");

for line in file:

    if line.find("GLYPH") == 0:
        i = 0;
        Bytes = [0]*8;
        n = int(line[6:10], 16);

    elif line.find("ENDGLYPH") == 0:
        print('\t{', end='');
        for j in range(8):
            print('0x', end='');
            print(format(Bytes[j], '02x'), end='');
            if j<7: print(', ', end='');
        print('},');
        n = None;

#    elif n == 1: break;

    elif not n == None:
        if len(line)>8:
            if i>7:
                # horizontal scan (text line 8 chars)
                for j in range(8):
                    # vertical encoding (7 bits)
                    Bytes[j] >>= 1;
                    if line[j+1] == 'X': Bytes[j] += 0x40;
#                    print(line[j+1], end='');
#                print();
            i+=1;

file.close();
print("};");
print("// remove last coma manually");

print();

print("const uint8_t microlib_SSD1306::lu[] PROGMEM = {");

n = None;

file = open("./8x15_.txt", "r");

k = 0;

lu = [0]*96;

for line in file:

    if line.find("GLYPH") == 0:
        n = int(line[6:10], 16)-32;
        lu[n] = k;
        k += 1;

file.close();

for j in range(12):
    for i in range(8):
        if i == 0: print('\t', end='');
        print('0x', end='');
        print(format(lu[j*8+i], '02x'), end='');
        print(', ', end='');
    print();

print("};");
print("// remove last coma manually");
