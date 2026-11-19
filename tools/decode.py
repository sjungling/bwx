"""Merge the mmiotrace and PCI config logs into one stream with backplane addresses."""
import re, sys

mmio_path, cfg_path, out_path = sys.argv[1:4]
CORES = {0x18000000: "cc", 0x18001000: "d11", 0x18002000: "cr4", 0x18003000: "pcie2",
         0x18004000: "usb", 0x1810a000: "erom"}
WRAP = {0x18100000: "cc", 0x18101000: "d11", 0x18102000: "cr4", 0x18103000: "pcie2", 0x18104000: "usb"}

events = []
maps = {}
seq = 0
for line in open(mmio_path):
    p = line.split()
    if not p:
        continue
    seq += 1
    if p[0] == "MAP":
        maps[p[2]] = int(p[3], 16)
    elif p[0] in ("R", "W"):
        events.append((float(p[2]), seq, "mmio", p[0], int(p[1]), int(p[4], 16) - maps[p[3]], int(p[5], 16), maps[p[3]]))
    elif p[0] == "MARK":
        events.append((float(p[1]), seq, "mark", " ".join(p[3:])))
cfg_re = re.compile(r"\s(\d+\.\d+): (read|write)_(byte|word|dword): .*where=0x([0-9a-f]+) val=0x([0-9a-f]+)")
for line in open(cfg_path):
    m = cfg_re.search(line)
    if m and m.group(2) == "write":
        seq += 1
        events.append((float(m.group(1)), seq, "cfgw", int(m.group(4), 16), int(m.group(5), 16)))
events.sort(key=lambda e: (e[0], e[1]))

win, win2 = None, None
with open(out_path, "w") as out:
    for e in events:
        if e[2] == "mark":
            out.write("# %s\n" % e[3])
        elif e[2] == "cfgw":
            where, val = e[3], e[4]
            if where == 0x80:
                win = val
            elif where == 0x70:
                win2 = val
            else:
                out.write("%.6f CFGW 0x%02x = 0x%x\n" % (e[0], where, val))
        else:
            _, _, _, rw, width, off, val, base = e
            if base != 0x800200000:
                tgt = "bar2+0x%x" % off
            elif off < 0x1000:
                tgt = "%s+0x%03x" % (CORES.get(win, "?%s" % (hex(win) if win is not None else "none")), off)
            elif off < 0x2000:
                tgt = "%s.wrap+0x%03x" % (WRAP.get(win2, "?%s" % (hex(win2) if win2 is not None else "none")), off - 0x1000)
            elif off < 0x3000:
                tgt = "pcie2.fixed+0x%03x" % (off - 0x2000)
            else:
                tgt = "cc.fixed+0x%03x" % (off - 0x3000)
            out.write("%.6f %s%d %s = 0x%x\n" % (e[0], rw, width, tgt, val))
