# bwx: FreeBSD driver for the Broadcom BCM4360

An experimental `net80211` driver for the Broadcom BCM4360 wifi card (`14e4:43a0`), a chip that no other FreeBSD driver supports. It was written for a 2014 Mac mini running FreeBSD 15.1, which is the only machine it has been tested on.

**This repository will not build as it stands.** The driver depends on a table of register writes recorded from Broadcom's own Linux driver, `wl`. That table contains Broadcom's microcode, so it is not included, and you have to record it from your own card by following `docs/TRACING.md`.

## Warnings

- **It can reset or freeze the machine.** Reading a register on a powered-down core will do it, and an ordinary driver bug can too. `docs/NOTES.md` explains the rule that prevents this.
- **Radio regulations are your responsibility.** The driver has no transmit power control of its own and simply replays the values that `wl` programmed. Its behavior on DFS channels has not been tested, so you are responsible for meeting the rules where you live.
- **Only one card has been tested.** The recorded sequences include calibration results, and nobody knows yet how well they carry over to a different card.

## What works

- Scanning, joining a WPA2 network and passing traffic on 2.4 and 5 GHz.
- 802.11n receive on 20 MHz channels, at about 50 Mb/s.
- 802.11ac at 80 MHz, on channel 100 only. Set `hw.bwx.vht="1"` in `/boot/loader.conf` and choose a regulatory domain that has 802.11ac bands on that channel.
- Aggregated transmit on that channel, where uploads run at 100 to 185 Mb/s and downloads at 115 to 140 Mb/s. Setting `hw.bwx.ampdu="0"` turns it off.
- Station mode only, with no roaming, no power saving and no access point mode.

## How it works

The driver does not know how to set up the radio by itself. For bring-up and channel changes it replays register sequences that were recorded from Broadcom's driver. The receive path, the transmit path and the `net80211` glue are native code. The driver polls the card once per tick instead of using an interrupt.

## Build

Put your own traces in the repository root, as described in `docs/TRACING.md`. Then generate the table and build the module against the kernel source in `/usr/src`:

```sh
python3 tools/gendriver.py scan-mmiotrace.txt scan-pcicfg.txt \
    driver/bwx_ops.h join-preassoc-mmio.txt join-cfg.txt join-periodic.txt
cd driver && make
```

Rebuild the module after every kernel update. A module built for another kernel will not load.

## Layout

| Path | What it is |
|---|---|
| `driver/` | The driver, `if_bwx.c` |
| `tools/` | Scripts that decode traces and generate the table |
| `linux-tools/` | Linux modules and a script used to take the traces (GPL-2.0) |
| `docs/NOTES.md` | Chip layout, the transmit path and what is still open |
| `docs/TRACING.md` | How the traces were taken |

## License

The driver and tools are under the BSD 2-Clause license. The code in `linux-tools/` is GPL-2.0, because it builds Linux kernel modules. This project is not affiliated with Broadcom.
