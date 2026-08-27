#!/usr/bin/env python3
# lwasm's raw (-r) output writer has a code-generation bug on this file: for
# a large chunk of the tail of hdbdos.asm (in testing, roughly the last 1.2K
# of an 8K image), the bytes it writes to the raw binary don't match what it
# resolves internally -- e.g. a live branch instruction lands on stale bytes
# left over from an earlier assembler pass instead of the real target code.
# lwasm's listing output doesn't have this problem (verified against the
# DECB (-b) output format, which is generated differently and is correct),
# so this rebuilds the ROM image from the listing's own byte dump instead of
# trusting the raw file, using the assembler's MAGICDG/ZZLAST symbols (also
# only trustworthy via the listing/symbol dump) as the start/end of real code.
#
# Usage: fixrom8k.py romfile listingfile symdumpfile
import re
import sys

rom_path, lst_path, sym_path = sys.argv[1:4]

start = end = None
with open(sym_path) as f:
    for line in f:
        m = re.match(r'^(MAGICDG|ZZLAST) EQU \$([0-9A-Fa-f]+)', line)
        if not m:
            continue
        if m.group(1) == "MAGICDG":
            start = int(m.group(2), 16)
        else:
            end = int(m.group(2), 16)
if start is None or end is None:
    sys.exit("fixrom8k.py: couldn't find MAGICDG/ZZLAST in " + sym_path)

mem = {}
addr_re = re.compile(r'^([0-9A-Fa-f]{4}) ([0-9A-Fa-f]+)\s')
cont_re = re.compile(r'^\s{5}([0-9A-Fa-f]+)\s*$')
cur_addr = None
with open(lst_path) as f:
    for line in f:
        m = addr_re.match(line)
        if m:
            addr = int(m.group(1), 16)
            hexbytes = m.group(2)
            if len(hexbytes) % 2 == 0:
                for i in range(0, len(hexbytes), 2):
                    mem[addr] = int(hexbytes[i:i + 2], 16)
                    addr += 1
                cur_addr = addr
            continue
        m = cont_re.match(line)
        if m and cur_addr is not None:
            hexbytes = m.group(1)
            if len(hexbytes) % 2 == 0:
                for i in range(0, len(hexbytes), 2):
                    mem[cur_addr] = int(hexbytes[i:i + 2], 16)
                    cur_addr += 1

missing = [a for a in range(start, end + 1) if a not in mem]
if missing:
    sys.exit(f"fixrom8k.py: {len(missing)} bytes missing from listing "
              f"in ${start:04X}-${end:04X}, e.g. ${missing[0]:04X}")

codelen = end - start + 1
if codelen > 8192:
    sys.exit(f"fixrom8k.py: reconstructed code is {codelen} bytes, "
              f"expected <= 8192")

data = bytes(mem[a] for a in range(start, end + 1))
data += bytes([0x39]) * (8192 - codelen)
with open(rom_path, "wb") as f:
    f.write(data)
