# Notes on the BCM4360

What was learned about the card while writing the driver. The card is a Broadcom BCM4360, PCI ID `14e4:43a0`, chip rev 3, in a 2014 Mac mini.

## The one rule that matters

Never read a register on a core that is in reset or powered down.
The read returns `0xffffffff`, the card logs a Completer Abort, and the host
resets or freezes a few seconds later. This has happened four times:

1. A `/dev/mem` read through the default window (`0x18001000`, the D11 core).
2. A `bus_read_4` of the same address from an early probe module.
3. A read of `rcvstatus0` after the replayed scan had powered the D11 core down.
4. The driver's status poll, after a channel change replayed the power-down
   that ends the recorded scan.

The driver now checks the D11 wrapper before each poll and transmit:
`resetctl` must be 0 and `ioctl` must have bits `0x11` set.

The bhyve guest does not protect the host from this. Wrapper registers and
chipcommon have been safe to read at all times so far.

## How the card is laid out

BAR0 is 32 KB and holds four 4 KB views:

| BAR0 offset | What it shows | Selected by |
|---|---|---|
| `0x0000` | Any core's registers | PCI config `0x80` |
| `0x1000` | Any core's wrapper | PCI config `0x70` |
| `0x2000` | PCIe core, fixed | |
| `0x3000` | Chipcommon, fixed | |

Cores, from the EROM at `0x1810a000`:

| Core | ID | Rev | Registers | Wrapper |
|---|---|---|---|---|
| Chipcommon | `0x800` | 43 | `0x18000000` | `0x18100000` |
| D11 (the MAC) | `0x812` | 42 | `0x18001000` | `0x18101000` |
| ARM CR4 | `0x83e` | 2 | `0x18002000` | `0x18102000` |
| PCIe gen2 | `0x83c` | 1 | `0x18003000` | `0x18103000` |
| USB | | | `0x18004000` | `0x18104000` |

To reset a core, write the wrapper: `ioctl` (`0x408`) = 7, `resetctl` (`0x800`)
= 1 then 0, `ioctl` = 5.

The D11 core uses the standard 64-bit DMA engine. The four transmit FIFOs sit
at `0x200`, `0x240`, `0x280` and `0x2c0`, and receive sits at `0x220`. Every
high address word needs `0x80000000` set. Received frames carry a 40-byte
header, and the 802.11 header starts at byte 48.

## Aggregated transmit

The card builds the aggregates. The driver marks each frame, and the card
groups neighbouring frames in a FIFO, sends them, reads the block ack and
resends what was missed. What the driver has to get right:

- **The header.** Control word `0x40c0`, the sequence number at offset 18,
  four 802.11ac rate entries and the limits at offset 106. All of it is
  copied from the vendor driver's frames. The rate entries are for 80 MHz
  with two streams, so only the 80 MHz channel is aggregated.
- **Sequence numbers.** `net80211` leaves them to the driver once a
  block-ack session runs. The driver assigns them as frames enter the ring
  and never lets more than 64 be outstanding.
- **Status reports.** A report names one frame and gives a count. The count
  is how many ring frames, starting there, the card has finished with. The
  second packet holds one ack bit per sequence number. A missed frame that
  the card is still resending ends the count early. The third word holds the
  number of frames sent and, in the next byte, the number acked. Rate
  selection uses those two numbers.
- **Ring slots.** A slot is free only once its frame has a status report.
  The card's ring pointer moves when a frame is copied into the card, which
  is long before it is sent. Trusting that pointer let new frames overwrite
  the records of old ones, and under load every report then matched the
  wrong frame.
- **One FIFO per access category.** QoS data goes to the FIFO of its class,
  aggregated or not, as in the vendor driver. `ssh` marks bulk data as
  background, so an `scp` runs on FIFO 0 and a plain upload on FIFO 1.

Rate selection is simple. It starts at MCS 7, steps up after two clean
seconds and steps down when a quarter of the frames in a second were lost.
Each failed step up doubles the wait before the next try.
`dev.bwx.0.agg_mcs` shows the current rate, and `agg_auto=0` pins it.

Known gaps:

- The access point ends a new session once or twice before one sticks. It
  sends a DELBA with reason 1 a few frames after accepting. `net80211`
  asks again at once, and the session then stays up, so the cost is a slow
  first second. The cause is not known.
- Aggregates use two streams and the long guard interval. Three streams and
  the short guard interval are untested.
- `dev.bwx.0.tx_debug=N` logs the next N status reports that do not fit,
  with the 40 reports before each.

## bhyve notes

- `pptdevs="2/0/0"` in `/boot/loader.conf` reserves the card.
- bhyve gives passed-through devices no legacy interrupt, only MSI.
  `docs/TRACING.md` shows the small change that makes Broadcom's `wl` use MSI.

## What is still open

1. Its own bring-up code in place of the replayed sequences.
2. An interrupt handler in place of the once-a-tick poll.
3. 80 MHz on channels other than 100, and aggregated transmit on 20 and
   40 MHz channels. Both need more traces of the vendor driver.
4. Hardware encryption. The card can do it; the driver uses `net80211`'s
   software CCMP.
5. Power saving and roaming.

