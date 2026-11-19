# Taking the traces

The driver replays what Broadcom's Linux driver, `wl`, does to the card. This page says how those recordings were made. It is a record of one working setup, not a tested recipe. Expect to adapt it.

Keep your traces out of any public repo. They hold Broadcom's microcode as register values. The join trace also holds your network's name, addresses and the start of every frame sent.

## Setup

- A FreeBSD host with the card reserved for bhyve: `pptdevs="2/0/0"` in `/boot/loader.conf`, with your card's bus address.
- An Ubuntu guest with the card passed through, and a kernel that has the `mmiotrace` tracer and kprobes.
- Broadcom's `wl` driver, version 6.30.223.271 (`broadcom-sta`).

bhyve gives a passed-through device no legacy interrupt, only MSI, and `wl` does not ask for MSI. In `src/wl/sys/wl_linux.c`, add one call after `pci_set_master(pdev);` and rebuild:

```c
pci_enable_msi(pdev);
```

A read of a powered-down core resets the host even from inside the guest. Do not poke at the card's registers by hand while `wl` has it down.

## What the generator needs

`tools/gendriver.py` takes five files:

| Argument | What it is |
|---|---|
| Scan trace, `mmiotrace` output | `wl` loading, the interface coming up, one full scan, then down and unload |
| Scan trace, PCI config writes | The kprobe log taken alongside it |
| Join trace, `mmiotrace` output | `wl` joining a network on the 80 MHz channel |
| Join trace, PCI config writes | The kprobe log taken alongside it |
| One maintenance burst | A decoded piece of the join trace |

### The scan trace

The script for this run was not kept. `linux-tools/join-trace.sh` sets up the same tracing: `mmiotrace` as the tracer, and kprobes on `pci_write_config_byte`, `_word` and `_dword` in a second trace instance.

The run was `modprobe wl`, `ip link set up`, one scan, `ip link set down`, `rmmod wl`. The generator finds the scan by two markers written to `trace_marker`: `MARK scan start` before the scan and `MARK scan returned` after it. Each channel the scan visits becomes one replayable channel change.

### The join trace

Run `linux-tools/join-trace.sh` as root in the guest, with a `wpa_supplicant` config at `/root/wpa.conf`. Build the two modules in `linux-tools/` first. It writes `join-mmio.txt`, `join-cfg.txt` and `join-txsnoop.txt`.

The generator only uses the part from the 80 MHz tune to the first frame sent after it. Cut `join-mmio.txt` soon after that point and before the WPA handshake installs keys. The cut file is the third argument.

`join-txsnoop.txt` holds the first 176 bytes of each frame `wl` sent. The transmit header in `if_bwx.c` was worked out from it. The generator does not read it.

### The maintenance burst

While associated at 80 MHz, `wl` repeats a short register sequence once a second. Without it the link is slower. To get it:

1. Decode the full join trace with `tools/decode.py`.
2. Find the burst that repeats each second while the link is idle. Every access in it is on the D11 core.
3. Save one copy as the last argument. Ours is about 1,250 lines.

## Values tied to our run

These are fixed in the code and match one card, one guest and one network. Yours may differ.

| Value | Where | Meaning |
|---|---|---|
| `0x800200000` | `tools/gendriver.py`, `tools/decode.py` | Where the guest mapped BAR0 |
| `0xe06a` | `tools/gendriver.py`, `if_bwx.c` | The chanspec for channel 100 at 80 MHz |
| `BWX_TRACE_RXRING`, `bwx_trace_txring` | `if_bwx.c` | DMA ring addresses `wl` used in the recorded run |
