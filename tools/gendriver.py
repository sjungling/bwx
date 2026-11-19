"""Build the driver's op table and channel segment table from the merged trace."""
import re, sys

mmio_path, cfg_path, out_path = sys.argv[1:4]
# optional: argv[4], argv[5] = a second trace pair holding an 80 MHz join
# optional: argv[6] = a decoded once-a-second maintenance burst at 80 MHz
CFG_KEEP = {0x80, 0x70, 0xd4}
D11 = 0x18001000
SHM_CHANSPEC_SEL = 0x10028

def load(mmio_path, cfg_path):
    events, maps, seq = [], {}, 0
    for line in open(mmio_path):
        p = line.split()
        if not p:
            continue
        seq += 1
        if p[0] == "MAP":
            maps[p[2]] = int(p[3], 16)
        elif p[0] in ("R", "W") and maps[p[3]] == 0x800200000:
            events.append((float(p[2]), seq, p[0], int(p[1]), int(p[4], 16) - maps[p[3]], int(p[5], 16)))
        elif p[0] == "MARK":
            events.append((float(p[1]), seq, "M", " ".join(p[3:])))
    for line in open(cfg_path):
        m = cfg_re.search(line)
        if m and int(m.group(3), 16) in CFG_KEEP:
            seq += 1
            events.append((float(m.group(1)), seq, "C", WIDTH[m.group(2)], int(m.group(3), 16), int(m.group(4), 16)))
    events.sort(key=lambda e: (e[0], e[1]))
    return events

cfg_re = re.compile(r"\s(\d+\.\d+): write_(byte|word|dword): .*where=0x([0-9a-f]+) val=0x([0-9a-f]+)")
WIDTH = {"byte": 1, "word": 2, "dword": 4}
events = load(mmio_path, cfg_path)

KIND = {"C": 0, "W": 1, "R": 2}
ops, marks, prev, win, win2, sel = [], {}, None, None, None, None
suspends, enables, chans, resets = [], [], [], []
D11_WRAP = 0x18101000
MCTL_BCNS_PROMISC = 0x00100000
for e in events:
    if e[2] == "M":
        marks[e[3]] = len(ops)
        continue
    gap = 0 if prev is None else int((e[0] - prev) * 1e6)
    prev = e[0]
    kind, width, off, val = e[2], e[3], e[4], e[5]
    i = len(ops)
    if kind == "C" and off == 0x80:
        win = val
    if kind == "C" and off == 0x70:
        win2 = val
    if win2 == D11_WRAP and kind == "W" and off == 0x1800 and val == 1:
        resets.append(i)
    if win == D11 and kind == "W" and off < 0x1000:
        if width == 4 and off == 0x120 and not val & 1:
            suspends.append(i)
        elif width == 4 and off == 0x120:
            enables.append(i)
        elif width == 4 and off == 0x160:
            sel = val
        elif (width == 2 and off == 0x164 and sel == SHM_CHANSPEC_SEL
              and "scan start" in marks and "scan returned" not in marks):
            chans.append((i, val))
    ops.append([KIND[kind], width, off, val, min(gap, 200000) if gap >= 40 else 0])

scan_end = marks["scan returned"]
starts = []
for i, cs in chans:
    s = max(x for x in suspends if x < i)
    assert i - s < 1500, (i, s)
    if i - s > 100:
        print("note: chanspec 0x%x set %d ops after its MAC suspend" % (cs, i - s))
    starts.append((s, cs))
# After the last channel the vendor driver leaves scan mode and then powers the
# D11 core down. End the last segment at the MAC enable that completes the
# channel change, and keep the "all beacons" receive filter that every other
# segment ends with. The power-down becomes the driver's stop sequence.
last = chans[-1][0]
last_enable = min(x for x in enables if x > last)
for x in suspends + enables:
    if last < x <= last_enable:
        ops[x][3] |= MCTL_BCNS_PROMISC
down_start = max(x for x in suspends if x < min(r for r in resets if r > last))
assert last_enable < down_start < scan_end

down_end = len(ops)
segs = [(0, starts[1][0], starts[0][1])]
for k in range(1, len(starts)):
    end = starts[k + 1][0] if k + 1 < len(starts) else last_enable + 1
    segs.append((starts[k][0], end, starts[k][1]))
# No segment may contain a D11 core reset except the first, which is bring-up.
for s_, e_, cs_ in segs[1:]:
    assert not any(s_ <= r < e_ for r in resets), hex(cs_)

# Optional second trace: the vendor driver joining on an 80 MHz channel. Take
# everything from the MAC suspend that starts the 80 MHz tune up to its first
# transmit, which covers the calibration that follows the tune.
CHANSPEC_80 = 0xe06a
if len(sys.argv) > 5:
    jev = load(sys.argv[4], sys.argv[5])
    jops, jprev, jwin, jsel = [], None, None, None
    jsus, jcs, jtx = [], None, None
    for e in jev:
        if e[2] == "M":
            continue
        gap = 0 if jprev is None else int((e[0] - jprev) * 1e6)
        jprev = e[0]
        kind, width, off, val = e[2], e[3], e[4], e[5]
        i = len(jops)
        if kind == "C" and off == 0x80:
            jwin = val
        if jwin == D11 and kind == "W" and off < 0x1000:
            if width == 4 and off == 0x120:
                if not val & 1 and jcs is None:
                    jsus.append(i)
            elif width == 4 and off == 0x160:
                jsel = val
            elif width == 2 and off == 0x164 and jsel == SHM_CHANSPEC_SEL and val == CHANSPEC_80 and jcs is None:
                jcs = i
            elif width == 4 and off in (0x204, 0x244, 0x284, 0x2c4) and jcs is not None and jtx is None:
                jtx = i
        jops.append([KIND[kind], width, off, val, min(gap, 200000) if gap >= 40 else 0, jwin])
    jstart = max(x for x in jsus if x < jcs)
    seg80 = jops[jstart:jtx]
    for o in seg80:
        if o[5] == D11 and o[0] == KIND["W"] and o[1] == 4:
            # ring addresses must not be reprogrammed inside this stretch
            assert o[2] not in (0x208, 0x248, 0x288, 0x2c8, 0x228), hex(o[2])
            if o[2] == 0x120:
                o[3] |= MCTL_BCNS_PROMISC
    base = len(ops)
    ops.extend([o[:5] for o in seg80])
    segs.append((base, len(ops), CHANSPEC_80))
    print("80 MHz segment: %d ops from the join trace (tune at +%d)" % (len(seg80), jcs - jstart))

# Optional third input: one of the vendor driver's once-a-second maintenance
# bursts at 80 MHz, in the decoded format written by tools/decode.py. Every
# access in it is on the D11 core, so no window changes are needed.
periodic = (0, 0)
if len(sys.argv) > 6:
    FIXED = {"d11": 0x0000, "d11.wrap": 0x1000, "pcie2.fixed": 0x2000, "cc.fixed": 0x3000}
    pstart, pprev = len(ops), None
    for line in open(sys.argv[6]):
        t, op, target, _, val = line.split()
        core, off = target.split("+")
        off, val, t = FIXED[core] + int(off, 16), int(val, 16), float(t)
        gap = 0 if pprev is None else int((t - pprev) * 1e6)
        pprev = t
        if core == "d11" and op == "W4" and off == 0x120:
            val |= MCTL_BCNS_PROMISC
        ops.append([KIND[op[0]], int(op[1]), off, val, min(gap, 200000) if gap >= 40 else 0])
    periodic = (pstart, len(ops))
    print("periodic segment: %d ops" % (periodic[1] - periodic[0]))

with open(out_path, "w") as out:
    out.write("/* Generated by tools/gendriver.py from a register trace. Do not edit. */\n\n")
    out.write("#define\tBWX_DOWN_START\t%d\n" % down_start)
    out.write("#define\tBWX_DOWN_END\t%d\n" % down_end)
    out.write("#define\tBWX_PERIODIC_START\t%d\n" % periodic[0])
    out.write("#define\tBWX_PERIODIC_END\t%d\n\n" % periodic[1])
    out.write("static const struct bwx_seg bwx_segs[] = {\n")
    for s, e, cs in segs:
        out.write("\t{ %d, %d, 0x%04x },\n" % (s, e, cs))
    out.write("};\n\nstatic const struct bwx_op bwx_ops[] = {\n")
    for k, w, off, val, d in ops:
        out.write("\t{ %d, %d, 0x%04x, 0x%08x, %d },\n" % (k, w, off, val, d))
    out.write("};\n")
print(len(ops), "ops,", len(segs), "segments; last segment", segs[-1][:2], "down starts at", down_start, "scan ends at", scan_end)
print("init segment:", segs[0][:2], "sizes:", sorted(set(e - s for s, e, _ in segs[1:]))[:6], "...")
print("2.4 GHz:", sorted(cs & 0xff for _, _, cs in segs if cs >> 12 == 1))
print("5 GHz:  ", sorted(cs & 0xff for _, _, cs in segs if cs >> 12 == 0xd))
print("order:  ", " ".join("%x" % cs for _, _, cs in segs))
