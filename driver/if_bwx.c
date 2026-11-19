/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Scott Jungling
 *
 * bwx: experimental net80211 driver for the Broadcom BCM4360 (14e4:43a0).
 *
 * Bring-up and channel changes replay register sequences recorded from the
 * vendor driver (bwx_ops.h). Receive, transmit and the net80211 glue are
 * native. On the 80 MHz channel unicast data is marked for aggregation at
 * 802.11ac rates: the card builds the aggregates and resends missed frames,
 * and the driver keeps each frame until the card reports it acked or gives
 * up on it. Everything else goes out one frame at a time at legacy rates.
 *
 * One rule shapes the locking: a D11 register must never be read while that
 * core is in reset or powered down, because the host resets a few seconds
 * later. All D11 access therefore happens under sc_mtx, only while
 * sc_running is set, and only with the BAR0 window on the D11 core.
 */
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/endian.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>
#include <net/ethernet.h>
#include <net/vnet.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp_lro.h>
#include <machine/in_cksum.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_phy.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_ht.h>
#include <net80211/ieee80211_vht.h>

struct bwx_op {
	uint8_t		kind;		/* 0 config write, 1 write, 2 read */
	uint8_t		width;
	uint16_t	off;
	uint32_t	val;
	uint32_t	delay_us;
};
#define	BWX_OP_CFGW	0
#define	BWX_OP_W	1
#define	BWX_OP_R	2

struct bwx_seg {
	uint32_t	start;
	uint32_t	end;
	uint16_t	chanspec;
};

#include "bwx_ops.h"

/* PCI config and BAR0 layout */
#define	BWX_CFG_WIN		0x80	/* core window, shown at BAR0+0x0000 */
#define	BWX_CFG_WIN2		0x70	/* wrapper window, shown at BAR0+0x1000 */
#define	BWX_D11_BASE		0x18001000
#define	BWX_D11_WRAP		0x18101000
#define	BWX_CC_FIXED		0x3000	/* chipcommon is always at BAR0+0x3000 */
#define	BWX_SROM_MACADDR	(BWX_CC_FIXED + 0x890)

/* D11 wrapper registers, through the wrapper window at BAR0+0x1000 */
#define	BWX_WRAP_IOCTL		0x1408
#define	BWX_WRAP_RESETCTL	0x1800
/*
 * Every ioctl value the vendor driver read back while the core was up had
 * these bits set, and every value around a reset or power-down lacked one.
 */
#define	BWX_WRAP_IOCTL_UP	0x0011

/* D11 core registers */
#define	D11_TXSTATUS(n)		(0x170 + (n) * 4)
#define	D11_XMTPTR(f)		(0x204 + (f) * 0x40)
#define	D11_XMTADDRLOW(f)	(0x208 + (f) * 0x40)
#define	D11_XMTSTATUS0(f)	(0x210 + (f) * 0x40)
#define	D11_RCVPTR		0x224
#define	D11_RCVADDRLOW		0x228
#define	D11_RCVSTATUS0		0x230
#define	D11_STATUS_CD_MASK	0x1fff
#define	D11_TXSTATUS_VALID	0x0001
/*
 * A status report is two four-word packets. The first has this bit and the
 * frame ID in its top half; the second carries neither.
 */
#define	D11_TXSTATUS_FIRST	0x0002
/* Set on every acknowledged unicast frame recorded, clear on broadcasts. */
#define	D11_TXSTATUS_ACKED	0x8000
#define	D11_TXSTATUS_NCONS	0x7f00	/* frames this report covers */

/* Ring bases the vendor driver used in the recorded run. */
#define	BWX_TRACE_RXRING	0x0839c000
static const uint32_t bwx_trace_txring[4] = {
	0x0269c000, 0x02694000, 0x03cf0000, 0x03cf4000
};

#define	BWX_NFIFO		4
/*
 * One FIFO per access category. The vendor driver sends management frames on
 * FIFO 3 (voice) and data on FIFO 1 (best effort). Voice contends for the
 * channel far more eagerly, so bulk data sent there collides with the access
 * point's own frames.
 */
#define	BWX_FIFO_DATA		1
#define	BWX_FIFO_MGMT		3
#define	BWX_NTXQ		BWX_NFIFO	/* indexed by FIFO number */
#define	BWX_RING_BYTES		8192
#define	BWX_NDESC		512
/* Room for the largest default A-MSDU (3839 bytes) plus the card's header. */
#define	BWX_BUF_BYTES		4096
#define	BWX_DD_SOF		0x80000000
#define	BWX_DD_EOF		0x40000000
#define	BWX_DD_IOC		0x20000000
#define	BWX_DD_EOT		0x10000000
#define	BWX_DMA_H32		0x80000000

struct bwx_dd {
	uint32_t	ctrl1;
	uint32_t	ctrl2;
	uint32_t	addrlow;
	uint32_t	addrhigh;
};

/* Receive header the card puts in front of every frame. */
#define	BWX_RXHDR_LEN		40
#define	BWX_RXH_FRAMESIZE	0
#define	BWX_RXH_RSSI		9	/* one signed dBm byte per antenna */
#define	BWX_RXH_NANT		3
#define	BWX_RXH_STATUS1		16
#define	BWX_RXH_CHANSPEC	22
#define	BWX_RXS_FCSERR		0x0001
#define	BWX_RXS_PBPRES		0x0004	/* two pad bytes follow the header */
#define	BWX_PLCP_LEN		6
#define	BWX_NOISE_DBM		(-95)	/* assumed; not measured */

/*
 * Transmit header, as observed on probe requests from the vendor driver:
 * a 4-byte prefix, 20 bytes of per-packet info, four 20-byte rate entries
 * and 24 bytes of cache info.
 */
#define	BWX_TXHDR_LEN		128
#define	BWX_TXH_PREFIX		0	/* le16, always 2 */
#define	BWX_TXH_MACCTLLOW	6
#define	BWX_TXH_MACCTLHIGH	8
#define	BWX_TXH_CHANSPEC	10
#define	BWX_TXH_IVOFFSET	12
#define	BWX_TXH_FRAMELEN	14
#define	BWX_TXH_FRAMEID		16
#define	BWX_TXH_PHYCTL0		24
#define	BWX_TXH_PLCP		30
#define	BWX_TXH_TXRATE		38
#define	BWX_TXH_RTSCTS		40
/* Control bits, named for when the vendor driver set them. */
#define	BWX_TXC_BASE		0x4000	/* every frame */
#define	BWX_TXC_HWSEQ		0x0800	/* management frames, not data */
#define	BWX_TXC_IACK		0x0080	/* every unicast frame */
#define	BWX_TXC_AMPDU		0x0040	/* may be aggregated with its neighbours */
#define	BWX_TXH_SEQ		18
#define	BWX_TXH_RATELEN		20
/* Aggregation limits, copied from the vendor driver's frames. */
#define	BWX_TXH_MPDUMAX		106	/* two bytes: primary and fallback */
#define	BWX_TXH_AMPDUDUR	108
#define	BWX_TXH_BAMAX		110
#define	BWX_AGG_MPDUMAX		0x30
#define	BWX_AGG_DUR		0x1525
#define	BWX_AGG_BAMAX		0x3f
#define	BWX_TXH_PHYCTL1		26
#define	BWX_PHYCTL0_CCK		0x01c4
#define	BWX_PHYCTL0_OFDM	0x0045
/* What the vendor driver used for 20 MHz frames inside an 80 MHz channel. */
#define	BWX_PHYCTL0_OFDM_IN80	0x01c5
#define	BWX_PHYCTL1_OFDM_IN80	0x0058
#define	BWX_PHYCTL0_VHT80	0x81c7
/* Aggregates ask for RTS/CTS; the second value marks the last rate entry. */
#define	BWX_RTSCTS_AGG		0x0905
#define	BWX_RTSCTS_AGG_LAST	0x0925

#define	BWX_TXID_STEP		0x200	/* the vendor driver's increment */
#define	BWX_TXID_FIFO		0x0007
#define	BWX_NIDS		(0x10000 / BWX_TXID_STEP)
#define	BWX_NDONE		32
#define	BWX_NEXT(i)		(((i) + 1) % BWX_NDESC)

#define	BWX_NTID		8
#define	BWX_BAWND		64
#define	BWX_AGG_QLEN		256
#define	BWX_AGG_TRIES		10
#define	BWX_VHT_NSS		2
#define	BWX_VHT_MCS_MAX		9

#define	BWX_CHANSPEC_2G		0x1000
#define	BWX_CHANSPEC_5G		0xd000
#define	BWX_CHANSPEC_BW_MASK	0xf000
#define	BWX_CHANSPEC_BW80	0xe000
/*
 * The one 80 MHz channel with a recorded tune: primary channel 100, centre
 * 106. It is the last segment and is entered from channel 1.
 */
#define	BWX_CHANSPEC_100_80	0xe06a
#define	BWX_CHAN_80_PRIMARY	100
#define	BWX_SEG_80		(nitems(bwx_segs) - 1)
#define	BWX_CHANSPEC_CH1	0x1001

/*
 * What the driver remembers about a frame in a transmit ring until its
 * status report arrives. Reports come back in ring order.
 */
struct bwx_txd {
	struct ieee80211_node	*ni;	/* set when rate control wants the result */
	uint16_t		 id;
	uint16_t		 pktlen;
	uint16_t		 seq;
	int8_t			 tid;	/* -1 unless sent for aggregation */
	uint8_t			 rate;
};

/*
 * One block-ack session. A frame stays in its slot, indexed by sequence
 * number, from the first transmission until it is acked or given up on.
 */
struct bwx_agg {
	struct mbufq		 q;	/* waiting for room in the window */
	struct mbuf		*slot[BWX_BAWND];
	uint8_t			 tries[BWX_BAWND];
	uint16_t		 start;	/* oldest sequence number still held */
	int			 held;
	int			 stamp;	/* ticks when a report last arrived */
	bool			 bar;	/* a frame was dropped; move the peer's window */
};

struct bwx_done {
	struct ieee80211_node	*ni;
	struct ieee80211_ratectl_tx_status txs;
};

/* The last few status reports, kept to explain one that makes no sense. */
#define	BWX_NTRACE		64
struct bwx_trace {
	uint32_t		 s0, s1, s2, t0, t1, t2;
	uint16_t		 done, prod, at;
};

struct bwx_softc {
	device_t		 sc_dev;
	struct ieee80211com	 sc_ic;
	struct mtx		 sc_mtx;
	struct callout		 sc_poll;
	struct resource		*sc_mem;
	int			 sc_rid;
	int			 sc_running;
	int			 sc_seg;
	uint32_t		 sc_win;

	struct bwx_dd		*sc_rxring;
	uint8_t			*sc_rxbufs;
	int			 sc_rxnext;
	struct bwx_txq {
		struct bwx_dd	*ring;
		uint8_t		*bufs;
		struct bwx_txd	 txd[BWX_NDESC];
		int		 prod;
		int		 done;	/* oldest frame without a report */
		int		 stamp;	/* ticks at the last report */
		int		 fifo;
	}			 sc_txq[BWX_NTXQ];
	uint16_t		 sc_txid;
	struct bwx_agg		 sc_agg[BWX_NTID];
	struct mbufq		 sc_txok, sc_txfail;	/* to hand back to net80211 */
	int			(*sc_ampdu_enable)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *);
	void			(*sc_addba_stop)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *);
	int			 sc_agg_mcs, sc_agg_auto, sc_agg_ticks;
	int			 sc_agg_ticks_hz;
	int			 sc_agg_good, sc_agg_hold, sc_agg_wait;
	u_int			 sc_ra_ok, sc_ra_bad;
	u_int			 sc_nagg, sc_naggack, sc_naggretry, sc_naggdrop;
	u_int			 sc_ntxstray, sc_ntxstale;
	int			 sc_txdebug, sc_txdebug_all;
	struct bwx_trace	 sc_tr[BWX_NTRACE];
	u_int			 sc_trnext;

	u_int			 sc_nrx, sc_nrxbad, sc_ntx, sc_ntxdrop;
	u_int			 sc_ntxstatus, sc_ntxnoack, sc_ntxtracked;

	/*
	 * Receive coalescing. Each TCP acknowledgement we send costs a turn
	 * on the channel, so merging the segments of a burst before TCP sees
	 * them cuts that return traffic several-fold.
	 */
	struct lro_ctrl		 sc_lro;
	if_input_fn_t		 sc_if_input;
	struct thread		*sc_lro_td;	/* set while the poll delivers */
	bool			 sc_lro_ready, sc_lro_flushing;
	int			 sc_periodic_ticks;
	int			 sc_lro_on;
	u_int			 sc_nlro;
	int			 sc_debug;
};

struct bwx_vap {
	struct ieee80211vap	 bv_vap;
	int			(*bv_newstate)(struct ieee80211vap *,
				    enum ieee80211_state, int);
};

#define	BWX_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	BWX_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define	BWX_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)

#define	BWX_READ_4(sc, o)	bus_read_4((sc)->sc_mem, (o))
#define	BWX_WRITE_4(sc, o, v)	bus_write_4((sc)->sc_mem, (o), (v))

static MALLOC_DEFINE(M_BWX, "bwx", "bwx driver");

/*
 * 802.11ac at 80 MHz is off unless hw.bwx.vht=1 is set in loader.conf. It
 * works on channel 100 only and needs a regulatory domain with 802.11ac
 * bands there (ETSI in FreeBSD's table). With it on under FCC3, net80211
 * falls back to plain 802.11a.
 */
static int bwx_vht = 0;
TUNABLE_INT("hw.bwx.vht", &bwx_vht);

/* Aggregated transmit, 80 MHz channel only. hw.bwx.ampdu=0 turns it off. */
static int bwx_ampdu = 1;
TUNABLE_INT("hw.bwx.ampdu", &bwx_ampdu);

/* Receive coalescing default; dev.bwx.N.lro changes it at run time. */
static int bwx_lro = 1;
TUNABLE_INT("hw.bwx.lro", &bwx_lro);

struct bwx_done;
static void	bwx_tx_status(struct bwx_softc *, const uint32_t *,
		    const uint32_t *, struct bwx_done *, int *);
static void	bwx_agg_tick(struct bwx_softc *);
static void	bwx_agg_fill(struct bwx_softc *, int);
static void	bwx_tx_complete_q(struct mbufq *, int);
static void	bwx_send_bar(struct bwx_softc *, int);

/* Point both BAR0 windows at the D11 core. */
static void
bwx_select_d11(struct bwx_softc *sc)
{
	pci_write_config(sc->sc_dev, BWX_CFG_WIN, BWX_D11_BASE, 4);
	pci_write_config(sc->sc_dev, BWX_CFG_WIN2, BWX_D11_WRAP, 4);
	sc->sc_win = BWX_D11_BASE;
}

/*
 * Backstop for the rule at the top of this file. The wrapper stays readable
 * in every core state, so ask it before touching a D11 register.
 */
static bool
bwx_d11_alive(struct bwx_softc *sc)
{
	BWX_ASSERT_LOCKED(sc);
	if (!sc->sc_running)
		return (false);
	if (BWX_READ_4(sc, BWX_WRAP_RESETCTL) == 0 &&
	    (BWX_READ_4(sc, BWX_WRAP_IOCTL) & BWX_WRAP_IOCTL_UP) ==
	    BWX_WRAP_IOCTL_UP)
		return (true);
	sc->sc_running = 0;
	device_printf(sc->sc_dev, "D11 core is down, stopped\n");
	return (false);
}

/* Give up on every frame a block-ack session holds. */
static void
bwx_agg_flush(struct bwx_softc *sc, struct bwx_agg *agg)
{
	struct mbuf *m;
	int i;

	BWX_ASSERT_LOCKED(sc);
	for (i = 0; i < BWX_BAWND; i++) {
		if (agg->slot[i] != NULL)
			(void)mbufq_enqueue(&sc->sc_txfail, agg->slot[i]);
		agg->slot[i] = NULL;
	}
	while ((m = mbufq_dequeue(&agg->q)) != NULL)
		(void)mbufq_enqueue(&sc->sc_txfail, m);
	agg->held = 0;
}

static void
bwx_agg_flush_all(struct bwx_softc *sc)
{
	for (int tid = 0; tid < BWX_NTID; tid++)
		bwx_agg_flush(sc, &sc->sc_agg[tid]);
}

/* Forget every frame in a ring; its status reports will not come. */
static void
bwx_txq_reset(struct bwx_softc *sc, struct bwx_txq *txq)
{
	for (int i = 0; i < BWX_NDESC; i++) {
		if (txq->txd[i].ni != NULL)
			ieee80211_free_node(txq->txd[i].ni);
		txq->txd[i].ni = NULL;
		txq->txd[i].tid = -1;
	}
	txq->prod = txq->done = 0;
	bwx_agg_flush_all(sc);
}

static uint32_t
bwx_rcvptr(struct bwx_softc *sc)
{
	/* One past the last usable descriptor: everything but rxnext - 1. */
	return (vtophys(sc->sc_rxring) +
	    ((sc->sc_rxnext + BWX_NDESC - 1) % BWX_NDESC) * sizeof(struct bwx_dd));
}

/*
 * Adjust a recorded write for this run: swap in our ring addresses and keep
 * the recorded transmit kicks from firing. Returns false to drop the write.
 */
static bool
bwx_replay_fixup(struct bwx_softc *sc, const struct bwx_op *op, uint32_t *val)
{
	int f;

	if (sc->sc_win != BWX_D11_BASE || op->width != 4)
		return (true);
	for (f = 0; f < BWX_NFIFO; f++) {
		if (op->off == D11_XMTPTR(f))
			return (false);
		if (op->off == D11_XMTADDRLOW(f) &&
		    op->val == bwx_trace_txring[f]) {
			*val = vtophys(sc->sc_txq[f].ring);
			bwx_txq_reset(sc, &sc->sc_txq[f]);
		}
	}
	if (op->off == D11_RCVADDRLOW && op->val == BWX_TRACE_RXRING) {
		*val = vtophys(sc->sc_rxring);
		sc->sc_rxnext = 0;
		pci_enable_busmaster(sc->sc_dev);
	} else if (op->off == D11_RCVPTR)
		*val = bwx_rcvptr(sc);
	return (true);
}

static void
bwx_replay(struct bwx_softc *sc, uint32_t from, uint32_t to)
{
	const struct bwx_op *op;
	uint32_t i, v;

	BWX_ASSERT_LOCKED(sc);
	for (i = from; i < to; i++) {
		op = &bwx_ops[i];
		if (op->delay_us != 0)
			DELAY(op->delay_us);
		switch (op->kind) {
		case BWX_OP_CFGW:
			pci_write_config(sc->sc_dev, op->off, op->val,
			    op->width);
			if (op->off == BWX_CFG_WIN)
				sc->sc_win = op->val;
			break;
		case BWX_OP_W:
			v = op->val;
			if (!bwx_replay_fixup(sc, op, &v))
				break;
			if (op->width == 4)
				bus_write_4(sc->sc_mem, op->off, v);
			else if (op->width == 2)
				bus_write_2(sc->sc_mem, op->off, v);
			else
				bus_write_1(sc->sc_mem, op->off, v);
			break;
		case BWX_OP_R:
			/* Some reads have side effects, so keep them. */
			if (op->width == 4)
				(void)bus_read_4(sc->sc_mem, op->off);
			else if (op->width == 2)
				(void)bus_read_2(sc->sc_mem, op->off);
			else
				(void)bus_read_1(sc->sc_mem, op->off);
			break;
		}
	}
	bwx_select_d11(sc);
}

static uint16_t
bwx_chanspec(const struct ieee80211_channel *c)
{
	if (IEEE80211_IS_CHAN_VHT80(c) && c->ic_ieee == BWX_CHAN_80_PRIMARY)
		return (BWX_CHANSPEC_100_80);
	return ((IEEE80211_IS_CHAN_2GHZ(c) ? BWX_CHANSPEC_2G : BWX_CHANSPEC_5G) |
	    c->ic_ieee);
}

/* The primary 20 MHz channel number a chanspec refers to. */
static int
bwx_chanspec_primary(uint16_t chanspec)
{
	int centre = chanspec & 0xff;

	/* An 80 MHz chanspec holds the centre; bits 8-9 pick the primary. */
	if ((chanspec & BWX_CHANSPEC_BW_MASK) == BWX_CHANSPEC_BW80)
		return (centre - 6 + 4 * ((chanspec >> 8) & 3));
	return (centre);
}

/*
 * The recorded channel changes form a loop in scan order, each one starting
 * from the channel before it. Walk forward around the loop to the target.
 */
static void
bwx_tune(struct bwx_softc *sc, uint16_t chanspec)
{
	int n, next;

	BWX_ASSERT_LOCKED(sc);
	for (n = 0; n < 2 * nitems(bwx_segs) &&
	    bwx_segs[sc->sc_seg].chanspec != chanspec; n++) {
		if (sc->sc_seg == BWX_SEG_80) {
			/* No recorded way out of 80 MHz, so start over. */
			bwx_replay(sc, 0, bwx_segs[0].end);
			sc->sc_seg = 0;
			continue;
		}
		if (chanspec == BWX_CHANSPEC_100_80 &&
		    bwx_segs[sc->sc_seg].chanspec == BWX_CHANSPEC_CH1)
			next = BWX_SEG_80;
		else
			next = sc->sc_seg + 1 >= BWX_SEG_80 ? 1 : sc->sc_seg + 1;
		bwx_replay(sc, bwx_segs[next].start, bwx_segs[next].end);
		sc->sc_seg = next;
	}
	if (bwx_segs[sc->sc_seg].chanspec != chanspec)
		device_printf(sc->sc_dev, "no recorded sequence for 0x%04x\n",
		    chanspec);
}

static void
bwx_rx_frame(struct bwx_softc *sc, const uint8_t *buf, struct mbufq *mq)
{
	struct ieee80211_rx_stats rxs;
	struct mbuf *m;
	uint16_t size, status, chanspec;
	int i, off, len, dbm;

	size = le16dec(buf + BWX_RXH_FRAMESIZE);
	status = le16dec(buf + BWX_RXH_STATUS1);
	off = BWX_RXHDR_LEN + ((status & BWX_RXS_PBPRES) ? 2 : 0) + BWX_PLCP_LEN;
	len = size - (off - BWX_RXHDR_LEN) - IEEE80211_CRC_LEN;
	if ((status & BWX_RXS_FCSERR) || len < IEEE80211_ACK_LEN ||
	    off + len > BWX_BUF_BYTES) {
		sc->sc_nrxbad++;
		return;
	}
	m = m_get2(len, M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL) {
		sc->sc_nrxbad++;
		return;
	}
	m_copyback(m, 0, len, buf + off);
	m->m_pkthdr.len = m->m_len = len;

	/*
	 * Tag the frame with the channel it arrived on. A channel change walks
	 * through other channels, so the current channel is often not it.
	 */
	dbm = BWX_NOISE_DBM;
	for (i = 0; i < BWX_RXH_NANT; i++)
		if ((int8_t)buf[BWX_RXH_RSSI + i] < 0)
			dbm = MAX(dbm, (int8_t)buf[BWX_RXH_RSSI + i]);
	chanspec = le16dec(buf + BWX_RXH_CHANSPEC);
	memset(&rxs, 0, sizeof(rxs));
	rxs.r_flags = IEEE80211_R_NF | IEEE80211_R_RSSI;
	rxs.c_nf = BWX_NOISE_DBM;
	rxs.c_rssi = MIN(2 * (dbm - BWX_NOISE_DBM), 127);
	if ((chanspec & 0xff) != 0) {
		rxs.r_flags |= IEEE80211_R_IEEE | IEEE80211_R_FREQ |
		    IEEE80211_R_BAND;
		rxs.c_ieee = bwx_chanspec_primary(chanspec);
		rxs.c_band = (chanspec & 0xf000) == BWX_CHANSPEC_2G ?
		    IEEE80211_CHAN_2GHZ : IEEE80211_CHAN_5GHZ;
		rxs.c_freq = ieee80211_ieee2mhz(rxs.c_ieee, rxs.c_band);
	}
	if (sc->sc_debug > 0) {
		sc->sc_debug--;
		device_printf(sc->sc_dev,
		    "rx fc=%02x%02x a2=%6D chanspec=0x%04x freq=%u dbm=%d plcp=%3D len=%d\n",
		    buf[off], buf[off + 1], buf + off + 10, ":", chanspec,
		    rxs.c_freq, dbm,
		    buf + off - BWX_PLCP_LEN, "", len);
	}
	(void)ieee80211_add_rx_params(m, &rxs);
	(void)mbufq_enqueue(mq, m);
	sc->sc_nrx++;
}

/*
 * Verify the IPv4 and TCP checksums of a decapsulated frame in software.
 * The kernel's coalescing code only accepts segments marked as verified.
 */
static bool
bwx_tcp_csum_ok(struct mbuf *m)
{
	struct ip *ip;
	int hlen, iplen;
	uint16_t sum;

	if (m->m_len < ETHER_HDR_LEN + sizeof(struct ip) ||
	    mtod(m, struct ether_header *)->ether_type != htons(ETHERTYPE_IP))
		return (false);
	ip = (struct ip *)(mtod(m, char *) + ETHER_HDR_LEN);
	hlen = ip->ip_hl << 2;
	iplen = ntohs(ip->ip_len);
	if (ip->ip_v != IPVERSION || hlen != sizeof(struct ip) ||
	    ip->ip_p != IPPROTO_TCP ||
	    (ip->ip_off & htons(IP_MF | IP_OFFMASK)) != 0 ||
	    iplen < hlen + 20 || m->m_pkthdr.len < ETHER_HDR_LEN + iplen)
		return (false);
	if (in_cksum_hdr(ip) != 0)
		return (false);
	/* The segment's sum plus the pseudo header must come to all ones. */
	sum = ~in_cksum_skip(m, ETHER_HDR_LEN + iplen, ETHER_HDR_LEN + hlen);
	return (in_addword(sum, in_pseudo(ip->ip_src.s_addr, ip->ip_dst.s_addr,
	    htons(IPPROTO_TCP + iplen - hlen))) == 0xffff);
}

/*
 * Sits in front of the vap's input routine. Frames that arrive from the
 * poll loop are offered to the coalescing code; the poll flushes it once
 * per pass. Anything else goes straight up the stack.
 */
static void
bwx_vap_input(if_t ifp, struct mbuf *m)
{
	struct ieee80211vap *vap = if_getsoftc(ifp);
	struct bwx_softc *sc = vap->iv_ic->ic_softc;
	struct epoch_tracker et;
	int error;

	if (sc->sc_lro_on && !sc->sc_lro_flushing &&
	    sc->sc_lro_td == curthread && bwx_tcp_csum_ok(m)) {
		m->m_pkthdr.csum_flags |= CSUM_DATA_VALID | CSUM_PSEUDO_HDR |
		    CSUM_IP_CHECKED | CSUM_IP_VALID;
		m->m_pkthdr.csum_data = 0xffff;
		NET_EPOCH_ENTER(et);
		CURVNET_SET(if_getvnet(ifp));
		error = tcp_lro_rx(&sc->sc_lro, m, 0);
		CURVNET_RESTORE();
		NET_EPOCH_EXIT(et);
		if (error == 0) {
			sc->sc_nlro++;
			return;
		}
	}
	sc->sc_if_input(ifp, m);
}

static void
bwx_lro_flush(struct bwx_softc *sc)
{
	struct epoch_tracker et;

	NET_EPOCH_ENTER(et);
	CURVNET_SET(if_getvnet(sc->sc_lro.ifp));
	sc->sc_lro_flushing = true;
	tcp_lro_flush_all(&sc->sc_lro);
	sc->sc_lro_flushing = false;
	CURVNET_RESTORE();
	NET_EPOCH_EXIT(et);
}

static void
bwx_poll(void *arg)
{
	struct bwx_softc *sc = arg;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct bwx_done done[BWX_NDONE];
	struct mbufq mq, txok, txfail;
	struct mbuf *m;
	uint32_t st, s[4], t[4];
	int cur, i, n, ndone = 0;
	bool bar[BWX_NTID];

	BWX_ASSERT_LOCKED(sc);
	if (!bwx_d11_alive(sc))
		return;
	mbufq_init(&mq, BWX_NDESC);

	st = BWX_READ_4(sc, D11_RCVSTATUS0);
	cur = (st & D11_STATUS_CD_MASK) / sizeof(struct bwx_dd);
	while (sc->sc_rxnext != cur) {
		bwx_rx_frame(sc, sc->sc_rxbufs + sc->sc_rxnext * BWX_BUF_BYTES,
		    &mq);
		sc->sc_rxnext = (sc->sc_rxnext + 1) % BWX_NDESC;
	}
	BWX_WRITE_4(sc, D11_RCVPTR, bwx_rcvptr(sc));

	/*
	 * At 80 MHz the vendor driver repeated one radio maintenance sequence
	 * every second while joined. Do the same: without it a quarter of
	 * the access point's bursts needed a retransmission.
	 */
	if (sc->sc_seg == BWX_SEG_80 && BWX_PERIODIC_END != 0 &&
	    ++sc->sc_periodic_ticks >= hz) {
		sc->sc_periodic_ticks = 0;
		bwx_replay(sc, BWX_PERIODIC_START, BWX_PERIODIC_END);
	}

	/*
	 * Drain transmit status so the card's status queue cannot fill.
	 * Each report is two packets; a stray second packet is skipped.
	 */
	for (n = 0; n < 4 * BWX_NDONE && ndone < BWX_NDONE; n++) {
		s[0] = BWX_READ_4(sc, D11_TXSTATUS(0));
		if ((s[0] & D11_TXSTATUS_VALID) == 0)
			break;
		for (i = 1; i < 4; i++)
			s[i] = BWX_READ_4(sc, D11_TXSTATUS(i));
		sc->sc_ntxstatus++;
		if ((s[0] & D11_TXSTATUS_FIRST) == 0)
			continue;
		memset(t, 0, sizeof(t));
		t[0] = BWX_READ_4(sc, D11_TXSTATUS(0));
		if (t[0] & D11_TXSTATUS_VALID) {
			for (i = 1; i < 4; i++)
				t[i] = BWX_READ_4(sc, D11_TXSTATUS(i));
			sc->sc_ntxstatus++;
		}
		bwx_tx_status(sc, s, t, done, &ndone);
	}
	if (++sc->sc_agg_ticks_hz >= hz) {
		sc->sc_agg_ticks_hz = 0;
		bwx_agg_tick(sc);
	}
	for (i = 0; i < BWX_NTID; i++) {
		bwx_agg_fill(sc, i);
		bar[i] = sc->sc_agg[i].bar;
		sc->sc_agg[i].bar = false;
	}
	mbufq_init(&txok, INT_MAX);
	mbufq_init(&txfail, INT_MAX);
	mbufq_concat(&txok, &sc->sc_txok);
	mbufq_concat(&txfail, &sc->sc_txfail);

	BWX_UNLOCK(sc);
	for (i = 0; i < ndone; i++) {
		if (done[i].txs.flags != 0)
			ieee80211_ratectl_tx_complete(done[i].ni,
			    &done[i].txs);
		ieee80211_free_node(done[i].ni);
	}
	bwx_tx_complete_q(&txok, 0);
	bwx_tx_complete_q(&txfail, 1);
	for (i = 0; i < BWX_NTID; i++)
		if (bar[i])
			bwx_send_bar(sc, i);
	sc->sc_lro_td = curthread;
	while ((m = mbufq_dequeue(&mq)) != NULL) {
		ni = ieee80211_find_rxnode(ic,
		    mtod(m, struct ieee80211_frame_min *));
		if (ni != NULL) {
			/* Lets net80211 reorder frames that came in an aggregate. */
			if (ni->ni_flags & IEEE80211_NODE_HT)
				m->m_flags |= M_AMPDU;
			(void)ieee80211_input_mimo(ni, m);
			ieee80211_free_node(ni);
		} else
			(void)ieee80211_input_mimo_all(ic, m);
	}
	if (sc->sc_lro_ready)
		bwx_lro_flush(sc);
	sc->sc_lro_td = NULL;
	BWX_LOCK(sc);
	if (sc->sc_running)
		callout_reset(&sc->sc_poll, 1, bwx_poll, sc);
}

/* The 4-bit rate code from the 802.11 OFDM PLCP header, or -1. */
static int
bwx_ofdm_code(uint8_t rate)
{
	switch (rate) {
	case 12:	return (0xb);
	case 18:	return (0xf);
	case 24:	return (0xa);
	case 36:	return (0xe);
	case 48:	return (0x9);
	case 72:	return (0xd);
	case 96:	return (0x8);
	case 108:	return (0xc);
	}
	return (-1);
}

static bool
bwx_rate_ok(uint8_t rate, bool is5g)
{
	if (bwx_ofdm_code(rate) >= 0)
		return (true);
	return (!is5g && (rate == 2 || rate == 4 || rate == 11 || rate == 22));
}

/*
 * QoS data uses the FIFO of its access category, aggregated or not, so the
 * frames of one traffic class stay in order when a block-ack session starts.
 * The access point ended sessions whose first frames overtook older ones.
 */
static const uint8_t bwx_tid_fifo[BWX_NTID] = { 1, 0, 0, 1, 2, 2, 3, 3 };

static void
bwx_txd_clear(struct bwx_txd *d)
{
	if (d->ni != NULL)
		ieee80211_free_node(d->ni);
	d->ni = NULL;
	d->tid = -1;
}

/*
 * A slot is free once its frame has a status report. The card's own ring
 * position is not enough: it moves on as soon as a frame is copied into the
 * card, long before the frame is sent.
 */
static bool
bwx_txq_room(struct bwx_softc *sc, struct bwx_txq *txq)
{
	uint32_t st;

	if (BWX_NEXT(txq->prod) == txq->done)
		return (false);
	st = BWX_READ_4(sc, D11_XMTSTATUS0(txq->fifo));
	return (BWX_NEXT(txq->prod) !=
	    (st & D11_STATUS_CD_MASK) / sizeof(struct bwx_dd));
}

/*
 * Start a transmit header in the ring's next buffer and copy the frame in
 * behind it. The caller fills in the rate entries.
 */
static uint8_t *
bwx_txh_start(struct bwx_softc *sc, struct bwx_txq *txq, struct mbuf *m,
    uint16_t macctl, uint16_t macctlhi)
{
	uint8_t *h = txq->bufs + txq->prod * BWX_BUF_BYTES;
	struct bwx_txd *d = &txq->txd[txq->prod];
	int len = m->m_pkthdr.len;

	memset(h, 0, BWX_TXHDR_LEN);
	le16enc(h + BWX_TXH_PREFIX, 2);
	le16enc(h + BWX_TXH_MACCTLLOW, macctl);
	le16enc(h + BWX_TXH_MACCTLHIGH, macctlhi);
	le16enc(h + BWX_TXH_CHANSPEC, bwx_segs[sc->sc_seg].chanspec);
	h[BWX_TXH_IVOFFSET] = ieee80211_hdrsize(mtod(m, void *));
	le16enc(h + BWX_TXH_FRAMELEN, len + IEEE80211_CRC_LEN);
	sc->sc_txid += BWX_TXID_STEP;
	le16enc(h + BWX_TXH_FRAMEID, sc->sc_txid | txq->fifo);
	m_copydata(m, 0, len, h + BWX_TXHDR_LEN);

	bwx_txd_clear(d);
	d->id = sc->sc_txid | txq->fifo;
	d->pktlen = len;
	return (h);
}

/* Hand the buffer started by bwx_txh_start() to the card. */
static void
bwx_txq_push(struct bwx_softc *sc, struct bwx_txq *txq, int len)
{
	struct bwx_dd *dd = &txq->ring[txq->prod];

	dd->ctrl1 = htole32(BWX_DD_SOF | BWX_DD_EOF | BWX_DD_IOC |
	    (txq->prod == BWX_NDESC - 1 ? BWX_DD_EOT : 0));
	dd->ctrl2 = htole32(BWX_TXHDR_LEN + len);
	if (txq->prod == txq->done)
		txq->stamp = ticks;
	txq->prod = BWX_NEXT(txq->prod);
	BWX_WRITE_4(sc, D11_XMTPTR(txq->fifo), vtophys(txq->ring) +
	    txq->prod * sizeof(struct bwx_dd));
	sc->sc_ntx++;
}

static int
bwx_tx(struct bwx_softc *sc, struct ieee80211_node *ni, struct mbuf *m)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	struct bwx_txd *d;
	struct bwx_txq *txq;
	uint8_t *h;
	uint16_t chanspec, macctl, flen;
	uint8_t rate, type;
	int fifo, len, usec;
	bool is5g, isdata, track, countonly = false;

	BWX_ASSERT_LOCKED(sc);
	/* net80211 leaves the numbering of a block-ack session to the driver. */
	if (m->m_flags & M_AMPDU_MPDU)
		ieee80211_output_seqno_assign(ni, -1, m);
	wh = mtod(m, struct ieee80211_frame *);
	if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) {
		if (ieee80211_crypto_encap(ni, m) == NULL)
			return (ENOBUFS);
		wh = mtod(m, struct ieee80211_frame *);
	}
	len = m->m_pkthdr.len;
	if (BWX_TXHDR_LEN + len > BWX_BUF_BYTES || !bwx_d11_alive(sc))
		return (ENETDOWN);

	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	isdata = type == IEEE80211_FC0_TYPE_DATA;
	/* The key handshake rides with management so it is not held up. */
	fifo = isdata && (m->m_flags & M_EAPOL) == 0 ? BWX_FIFO_DATA :
	    BWX_FIFO_MGMT;
	if (fifo == BWX_FIFO_DATA && IEEE80211_QOS_HAS_SEQ(wh) &&
	    !IEEE80211_IS_MULTICAST(wh->i_addr1) &&
	    ieee80211_gettid(wh) < BWX_NTID)
		fifo = bwx_tid_fifo[ieee80211_gettid(wh)];
	txq = &sc->sc_txq[fifo];
	if (!bwx_txq_room(sc, txq))
		return (ENOBUFS);

	chanspec = bwx_segs[sc->sc_seg].chanspec;
	is5g = (chanspec & 0xf000) != BWX_CHANSPEC_2G;

	/*
	 * Rate control picks the rate for unicast data. Everything else,
	 * including the key handshake, goes at the lowest basic rate.
	 */
	rate = is5g ? 12 : 2;
	track = false;
	if (isdata && !IEEE80211_IS_MULTICAST(wh->i_addr1) &&
	    (m->m_flags & M_EAPOL) == 0) {
		ieee80211_ratectl_rate(ni, NULL, 0);
		if (ni->ni_flags & IEEE80211_NODE_HT) {
			/*
			 * Rate control now speaks MCS, which this path
			 * cannot send. Use the top legacy rate, and track
			 * the frame only to count lost ones.
			 */
			rate = 108;
			track = true;
			countonly = true;
		} else if (bwx_rate_ok(ieee80211_node_get_txrate_dot11rate(ni),
		    is5g)) {
			rate = ieee80211_node_get_txrate_dot11rate(ni);
			track = true;
		}
	}

	macctl = BWX_TXC_BASE;
	if (type == IEEE80211_FC0_TYPE_MGT)
		macctl |= BWX_TXC_HWSEQ;
	if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		macctl |= BWX_TXC_IACK;
		le16enc(wh->i_dur, ieee80211_ack_duration(ic->ic_rt, rate, 0));
	}
	flen = len + IEEE80211_CRC_LEN;

	d = &txq->txd[txq->prod];
	h = bwx_txh_start(sc, txq, m, macctl, 2);
	le16enc(h + BWX_TXH_RTSCTS, 0x20);
	le16enc(h + BWX_TXH_TXRATE, rate);
	if (bwx_ofdm_code(rate) >= 0) {
		/* OFDM PLCP: the rate code, then the 12-bit length */
		if ((chanspec & BWX_CHANSPEC_BW_MASK) == BWX_CHANSPEC_BW80) {
			le16enc(h + BWX_TXH_PHYCTL0, BWX_PHYCTL0_OFDM_IN80);
			le16enc(h + BWX_TXH_PHYCTL1, BWX_PHYCTL1_OFDM_IN80);
		} else
			le16enc(h + BWX_TXH_PHYCTL0, BWX_PHYCTL0_OFDM);
		le32enc(h + BWX_TXH_PLCP, bwx_ofdm_code(rate) |
		    ((uint32_t)flen << 5));
	} else {
		/*
		 * DSSS/CCK PLCP: signal in 100 kb/s units, service, then the
		 * duration in usec. At 11 Mb/s a service bit resolves the
		 * rounding of that duration.
		 */
		usec = howmany(flen * 16, rate);
		le16enc(h + BWX_TXH_PHYCTL0, BWX_PHYCTL0_CCK);
		h[BWX_TXH_PLCP] = rate * 5;
		h[BWX_TXH_PLCP + 1] = 0x04 |
		    (rate == 22 && usec * 11 - flen * 8 >= 8 ? 0x80 : 0);
		le16enc(h + BWX_TXH_PLCP + 2, usec);
	}
	if (track) {
		d->ni = ieee80211_ref_node(ni);
		d->rate = countonly ? 0 : rate;
	}
	bwx_txq_push(sc, txq, len);
	return (0);
}

/* One 80 MHz 802.11ac rate entry: two streams, long guard interval, LDPC. */
static void
bwx_vht_rate(uint8_t *r, int mcs, bool last)
{
	/* In 500 kb/s units. */
	static const uint16_t rate[BWX_VHT_MCS_MAX + 1] = {
		117, 234, 351, 468, 702, 936, 1053, 1170, 1404, 1560
	};

	le16enc(r, BWX_PHYCTL0_VHT80);
	le16enc(r + 2, BWX_PHYCTL1_OFDM_IN80);
	le16enc(r + 4, (BWX_VHT_NSS - 1) << 4 | mcs);
	/* VHT-SIG-A; the card adds the checksum. */
	r[6] = 0x06;
	r[7] = (BWX_VHT_NSS - 1) << 2;
	r[8] = 0x80;
	r[9] = mcs << 4 | 0x04;
	r[10] = 0x02;
	le16enc(r + 14, rate[mcs]);
	le16enc(r + 16, last ? BWX_RTSCTS_AGG_LAST : BWX_RTSCTS_AGG);
}

/* Queue one frame of a block-ack session to the card. */
static void
bwx_agg_put(struct bwx_softc *sc, int tid, struct mbuf *m)
{
	static const int8_t step[4] = { 0, 2, 3, 4 };
	struct bwx_txq *txq = &sc->sc_txq[bwx_tid_fifo[tid]];
	struct ieee80211_frame *wh = mtod(m, struct ieee80211_frame *);
	struct bwx_txd *d = &txq->txd[txq->prod];
	uint8_t *h;
	int i;

	h = bwx_txh_start(sc, txq, m, BWX_TXC_BASE | BWX_TXC_IACK |
	    BWX_TXC_AMPDU, 0);
	h[2] = 2;
	le16enc(h + BWX_TXH_SEQ, le16dec(wh->i_seq));
	/* The card steps down this list when a burst gets no block ack. */
	for (i = 0; i < 4; i++)
		bwx_vht_rate(h + BWX_TXH_PHYCTL0 + i * BWX_TXH_RATELEN,
		    MAX(MIN(sc->sc_agg_mcs, BWX_VHT_MCS_MAX) - step[i], 0),
		    i == 3);
	h[BWX_TXH_MPDUMAX] = h[BWX_TXH_MPDUMAX + 1] = BWX_AGG_MPDUMAX;
	le16enc(h + BWX_TXH_AMPDUDUR, BWX_AGG_DUR);
	h[BWX_TXH_BAMAX] = BWX_AGG_BAMAX;
	d->tid = tid;
	d->seq = M_SEQNO_GET(m);
	bwx_txq_push(sc, txq, m->m_pkthdr.len);
	sc->sc_nagg++;
}

/* Move waiting frames into the block-ack window while it has room. */
static void
bwx_agg_fill(struct bwx_softc *sc, int tid)
{
	struct bwx_agg *agg = &sc->sc_agg[tid];
	struct ieee80211_node *ni;
	struct ieee80211_frame *wh;
	struct mbuf *m;
	int i, wnd;

	BWX_ASSERT_LOCKED(sc);
	if (mbufq_first(&agg->q) == NULL || !bwx_d11_alive(sc))
		return;
	while ((m = mbufq_first(&agg->q)) != NULL) {
		ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;
		wnd = ni->ni_tx_ampdu[tid].txa_wnd;
		if (wnd == 0 || wnd > BWX_BAWND)
			wnd = BWX_BAWND;
		if (agg->held == 0) {
			agg->start = ni->ni_txseqs[tid];
			agg->stamp = ticks;
		}
		if (IEEE80211_SEQ_SUB(ni->ni_txseqs[tid], agg->start) >= wnd ||
		    !bwx_txq_room(sc, &sc->sc_txq[bwx_tid_fifo[tid]]))
			break;
		m = mbufq_dequeue(&agg->q);
		ieee80211_output_seqno_assign(ni, -1, m);
		wh = mtod(m, struct ieee80211_frame *);
		i = M_SEQNO_GET(m) % BWX_BAWND;
		if (agg->slot[i] != NULL ||
		    ((wh->i_fc[1] & IEEE80211_FC1_PROTECTED) &&
		    ieee80211_crypto_encap(ni, m) == NULL)) {
			(void)mbufq_enqueue(&sc->sc_txfail, m);
			continue;
		}
		agg->slot[i] = m;
		agg->tries[i] = 0;
		agg->held++;
		bwx_agg_put(sc, tid, m);
	}
}

/* The card's verdict on one frame of a block-ack session. */
static void
bwx_agg_status(struct bwx_softc *sc, const struct bwx_txd *d, bool acked)
{
	struct bwx_agg *agg = &sc->sc_agg[d->tid];
	int i = d->seq % BWX_BAWND;
	struct mbuf *m = agg->slot[i];

	/* The session may have been flushed since the frame was sent. */
	if (m == NULL || M_SEQNO_GET(m) != d->seq)
		return;
	agg->stamp = ticks;
	if (acked) {
		sc->sc_naggack++;
	} else {
		if (++agg->tries[i] < BWX_AGG_TRIES &&
		    bwx_txq_room(sc, &sc->sc_txq[bwx_tid_fifo[d->tid]])) {
			sc->sc_naggretry++;
			mtod(m, struct ieee80211_frame *)->i_fc[1] |=
			    IEEE80211_FC1_RETRY;
			bwx_agg_put(sc, d->tid, m);
			return;
		}
		sc->sc_naggdrop++;
		agg->bar = true;
	}
	agg->slot[i] = NULL;
	agg->held--;
	(void)mbufq_enqueue(acked ? &sc->sc_txok : &sc->sc_txfail, m);
	while (agg->held > 0 && agg->slot[agg->start % BWX_BAWND] == NULL)
		agg->start = IEEE80211_SEQ_INC(agg->start);
}

/*
 * One status report: two packets of four words. The first names a frame and
 * says how many ring frames, starting there, the card is done with. The
 * second holds one ack bit per sequence number, counted from that frame's.
 * A missed frame the card is still resending ends the count early.
 */
static void
bwx_tx_status(struct bwx_softc *sc, const uint32_t *s, const uint32_t *t,
    struct bwx_done *done, int *ndone)
{
	struct bwx_txq *txq;
	struct bwx_txd *d;
	struct bwx_done *dn;
	uint64_t map = (uint64_t)t[2] << 32 | t[1];
	struct bwx_trace *tr;
	uint16_t id = s[0] >> 16, first;
	int i, k, n, ncons, off;

	txq = &sc->sc_txq[(id & BWX_TXID_FIFO) % BWX_NTXQ];
	/* Frame ids repeat after BWX_NIDS frames, so look no further. */
	for (i = txq->done, n = 0; i != txq->prod && n < BWX_NIDS;
	    i = BWX_NEXT(i), n++)
		if (txq->txd[i].id == id)
			break;
	if (n == BWX_NIDS)
		i = txq->prod;
	txq->stamp = ticks;
	tr = &sc->sc_tr[sc->sc_trnext++ % BWX_NTRACE];
	tr->s0 = s[0]; tr->s1 = s[1]; tr->s2 = s[2];
	tr->t0 = t[0]; tr->t1 = t[1]; tr->t2 = t[2];
	tr->done = txq->done; tr->prod = txq->prod; tr->at = i;
	if ((i == txq->prod || i != txq->done) && sc->sc_txdebug > 0) {
		sc->sc_txdebug--;
		device_printf(sc->sc_dev, "txs %s, fifo %d, ids at done:",
		    i == txq->prod ? "stray" : "skip", txq->fifo);
		for (n = 0, k = txq->done; n < 6 && k != txq->prod;
		    n++, k = BWX_NEXT(k))
			printf(" %04x/%d/%u", txq->txd[k].id, txq->txd[k].tid,
			    txq->txd[k].seq);
		printf("\n");
		for (n = 0; n < 40 && n < sc->sc_trnext; n++) {
			tr = &sc->sc_tr[(sc->sc_trnext - 1 - n) % BWX_NTRACE];
			printf(" -%d %08x %08x %08x | %08x %08x %08x done %d "
			    "at %d prod %d\n", n, tr->s0, tr->s1, tr->s2,
			    tr->t0, tr->t1, tr->t2, tr->done, tr->at, tr->prod);
		}
	}
	if (i == txq->prod) {
		sc->sc_ntxstray++;
		return;
	}
	/* Anything passed over lost its report. */
	for (; txq->done != i; txq->done = BWX_NEXT(txq->done)) {
		d = &txq->txd[txq->done];
		if (d->tid >= 0)
			bwx_agg_status(sc, d, false);
		bwx_txd_clear(d);
	}
	ncons = MAX((s[0] & D11_TXSTATUS_NCONS) >> 8, 1);
	/*
	 * For an aggregate the third word counts the frames sent and, in the
	 * next byte, the ones the block ack covered.
	 */
	if (txq->txd[i].tid >= 0 && (s[2] & 0xff) >= (s[2] >> 8 & 0xff)) {
		sc->sc_ra_ok += s[2] >> 8 & 0xff;
		sc->sc_ra_bad += (s[2] & 0xff) - (s[2] >> 8 & 0xff);
	}
	first = txq->txd[i].seq;
	/* tx_debug logs the reports that are not a clean sweep; tx_debug_all, all. */
	n = (s[0] & D11_TXSTATUS_NCONS) >> 8;
	if (sc->sc_txdebug_all > 0 || (sc->sc_txdebug > 0 &&
	    txq->txd[i].tid >= 0 && (n == 0 || n > BWX_BAWND ||
	    map != (n == 64 ? ~0ULL : (1ULL << n) - 1)))) {
		if (sc->sc_txdebug_all > 0)
			sc->sc_txdebug_all--;
		else
			sc->sc_txdebug--;
		device_printf(sc->sc_dev, "txs %08x %08x %08x %08x | %08x "
		    "%08x %08x %08x seq %u tid %d\n", s[0], s[1], s[2], s[3],
		    t[0], t[1], t[2], t[3], first, txq->txd[i].tid);
	}
	for (n = 0; n < ncons && txq->done != txq->prod; n++) {
		d = &txq->txd[txq->done];
		txq->done = BWX_NEXT(txq->done);
		if (d->tid >= 0) {
			off = IEEE80211_SEQ_SUB(d->seq, first);
			if (sc->sc_txdebug_all > 0 && n > 0 && off != n)
				device_printf(sc->sc_dev, "txs   frame %d has "
				    "seq %u\n", n, d->seq);
			bwx_agg_status(sc, d, off < BWX_BAWND &&
			    (map >> off & 1) != 0);
			d->tid = -1;
			continue;
		}
		if (d->ni == NULL)
			continue;
		if (*ndone >= BWX_NDONE) {
			bwx_txd_clear(d);
			continue;
		}
		sc->sc_ntxtracked++;
		if ((s[0] & D11_TXSTATUS_ACKED) == 0)
			sc->sc_ntxnoack++;
		dn = &done[(*ndone)++];
		memset(&dn->txs, 0, sizeof(dn->txs));
		if (d->rate != 0) {
			dn->txs.flags = IEEE80211_RATECTL_STATUS_PKTLEN |
			    IEEE80211_RATECTL_STATUS_FINAL_RATE |
			    IEEE80211_RATECTL_STATUS_LONG_RETRY;
			dn->txs.status = (s[0] & D11_TXSTATUS_ACKED) ?
			    IEEE80211_RATECTL_TX_SUCCESS :
			    IEEE80211_RATECTL_TX_FAIL_LONG;
			dn->txs.pktlen = d->pktlen;
			dn->txs.final_rate = d->rate;
			/* The low byte of the third word counts attempts. */
			dn->txs.long_retries = MAX(s[2] & 0xff, 1) - 1;
		}
		dn->ni = d->ni;
		d->ni = NULL;
	}
}

/*
 * Once a second: pick the aggregate rate from how the last second went, and
 * free a session whose reports stopped coming.
 */
static void
bwx_agg_tick(struct bwx_softc *sc)
{
	struct bwx_agg *agg;
	struct bwx_txq *txq;
	u_int n = sc->sc_ra_ok + sc->sc_ra_bad;
	int q, tid;

	if (sc->sc_agg_hold > 0)
		sc->sc_agg_hold--;
	if (sc->sc_agg_auto && n >= 32) {
		if (sc->sc_ra_bad * 4 > n) {
			/*
			 * A rate that is too fast loses nearly everything, so
			 * each failed step up doubles the wait before the next.
			 */
			if (sc->sc_agg_mcs > 0)
				sc->sc_agg_mcs--;
			sc->sc_agg_good = 0;
			sc->sc_agg_wait = MIN(MAX(sc->sc_agg_wait * 2, 15), 900);
			sc->sc_agg_hold = sc->sc_agg_wait;
		} else if (sc->sc_ra_bad * 20 < n && sc->sc_agg_hold == 0 &&
		    ++sc->sc_agg_good >= 2) {
			if (sc->sc_agg_mcs < BWX_VHT_MCS_MAX)
				sc->sc_agg_mcs++;
			sc->sc_agg_good = 0;
		}
	}
	if (n >= 32 || sc->sc_agg_ticks++ >= 5) {
		sc->sc_ra_ok = sc->sc_ra_bad = 0;
		sc->sc_agg_ticks = 0;
	}
	/* A ring whose reports stopped would otherwise stay full for good. */
	for (q = 0; q < BWX_NTXQ; q++) {
		txq = &sc->sc_txq[q];
		if (txq->done == txq->prod || ticks - txq->stamp <= hz)
			continue;
		sc->sc_ntxstale++;
		for (; txq->done != txq->prod; txq->done = BWX_NEXT(txq->done))
			bwx_txd_clear(&txq->txd[txq->done]);
	}
	for (tid = 0; tid < BWX_NTID; tid++) {
		agg = &sc->sc_agg[tid];
		if (agg->held > 0 && ticks - agg->stamp > hz) {
			if (sc->sc_txdebug > 0)
				device_printf(sc->sc_dev, "agg tid %d: no reports "
				    "for %d frames from seq %u\n", tid, agg->held,
				    agg->start);
			sc->sc_naggdrop += agg->held;
			bwx_agg_flush(sc, agg);
			agg->bar = true;
		}
	}
}

static void
bwx_tx_complete_q(struct mbufq *q, int status)
{
	struct mbuf *m;

	while ((m = mbufq_dequeue(q)) != NULL)
		ieee80211_tx_complete((struct ieee80211_node *)m->m_pkthdr.rcvif,
		    m, status);
}

/* Hand finished frames back to net80211. Call without the lock. */
static void
bwx_tx_reap(struct bwx_softc *sc)
{
	struct mbufq ok, fail;

	mbufq_init(&ok, INT_MAX);
	mbufq_init(&fail, INT_MAX);
	BWX_LOCK(sc);
	mbufq_concat(&ok, &sc->sc_txok);
	mbufq_concat(&fail, &sc->sc_txfail);
	BWX_UNLOCK(sc);
	bwx_tx_complete_q(&ok, 0);
	bwx_tx_complete_q(&fail, 1);
}

static int
bwx_tx_one(struct bwx_softc *sc, struct ieee80211_node *ni, struct mbuf *m)
{
	int error;

	BWX_LOCK(sc);
	error = bwx_tx(sc, ni, m);
	if (error != 0)
		sc->sc_ntxdrop++;
	BWX_UNLOCK(sc);
	if (error != 0)
		return (error);
	/* Rate control hears about the frame later; net80211 need not wait. */
	ieee80211_tx_complete(ni, m, 0);
	return (0);
}

static int
bwx_transmit(struct ieee80211com *ic, struct mbuf *m)
{
	struct bwx_softc *sc = ic->ic_softc;
	struct ieee80211_node *ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;
	int error, tid;

	if (m->m_flags & M_AMPDU_MPDU) {
		tid = ieee80211_gettid(mtod(m, struct ieee80211_frame *));
		BWX_LOCK(sc);
		/* Leave room for what encryption adds. */
		if (sc->sc_running && sc->sc_seg == BWX_SEG_80 &&
		    tid < BWX_NTID && BWX_TXHDR_LEN + m->m_pkthdr.len + 32 <=
		    BWX_BUF_BYTES) {
			error = mbufq_enqueue(&sc->sc_agg[tid].q, m);
			if (error == 0)
				bwx_agg_fill(sc, tid);
			else
				sc->sc_ntxdrop++;
			BWX_UNLOCK(sc);
			return (error);
		}
		BWX_UNLOCK(sc);
	}
	return (bwx_tx_one(sc, ni, m));
}

static int
bwx_raw_xmit(struct ieee80211_node *ni, struct mbuf *m,
    const struct ieee80211_bpf_params *params)
{
	int error;

	error = bwx_tx_one(ni->ni_ic->ic_softc, ni, m);
	if (error != 0)
		m_freem(m);
	return (error);
}

/* Tell the peer to stop waiting for frames we gave up on. */
static void
bwx_send_bar(struct bwx_softc *sc, int tid)
{
	struct ieee80211vap *vap = TAILQ_FIRST(&sc->sc_ic.ic_vaps);
	struct bwx_agg *agg = &sc->sc_agg[tid];
	struct ieee80211_node *ni;
	struct ieee80211_tx_ampdu *tap;
	uint16_t seq;

	if (vap == NULL || vap->iv_state != IEEE80211_S_RUN)
		return;
	ni = ieee80211_ref_node(vap->iv_bss);
	tap = &ni->ni_tx_ampdu[tid];
	if (IEEE80211_AMPDU_RUNNING(tap)) {
		BWX_LOCK(sc);
		seq = agg->held > 0 ? agg->start : ni->ni_txseqs[tid];
		BWX_UNLOCK(sc);
		if (sc->sc_txdebug > 0)
			device_printf(sc->sc_dev, "agg tid %d: BAR to seq %u\n",
			    tid, seq);
		tap->txa_start = seq;
		(void)ieee80211_send_bar(ni, tap, seq);
	}
	ieee80211_free_node(ni);
}

static int
bwx_ampdu_enable(struct ieee80211_node *ni, struct ieee80211_tx_ampdu *tap)
{
	struct bwx_softc *sc = ni->ni_ic->ic_softc;

	/* Only the 80 MHz channel has a known aggregate header. */
	if (sc->sc_seg != BWX_SEG_80 || (ni->ni_flags & IEEE80211_NODE_VHT) == 0)
		return (0);
	return (sc->sc_ampdu_enable(ni, tap));
}

static void
bwx_addba_stop(struct ieee80211_node *ni, struct ieee80211_tx_ampdu *tap)
{
	struct bwx_softc *sc = ni->ni_ic->ic_softc;
	bool locked = mtx_owned(&sc->sc_mtx);

	sc->sc_addba_stop(ni, tap);
	if (tap->txa_tid >= BWX_NTID)
		return;
	if (sc->sc_txdebug > 0)
		device_printf(sc->sc_dev, "agg tid %d stopped, %d held\n",
		    tap->txa_tid, sc->sc_agg[tap->txa_tid].held);
	/* Dropping a node reference under the lock can lead here. */
	if (!locked)
		BWX_LOCK(sc);
	bwx_agg_flush(sc, &sc->sc_agg[tap->txa_tid]);
	if (!locked)
		BWX_UNLOCK(sc);
}

static void
bwx_init_locked(struct bwx_softc *sc)
{
	BWX_ASSERT_LOCKED(sc);
	bwx_replay(sc, 0, bwx_segs[0].end);
	sc->sc_seg = 0;
	sc->sc_running = 1;
	bwx_tune(sc, bwx_chanspec(sc->sc_ic.ic_curchan));
	callout_reset(&sc->sc_poll, 1, bwx_poll, sc);
}

static void
bwx_stop_locked(struct bwx_softc *sc)
{
	BWX_ASSERT_LOCKED(sc);
	if (!sc->sc_running)
		return;
	/* Clear sc_running first: the recorded shutdown powers the core down. */
	sc->sc_running = 0;
	callout_stop(&sc->sc_poll);
	for (int q = 0; q < BWX_NTXQ; q++)
		bwx_txq_reset(sc, &sc->sc_txq[q]);
	bwx_replay(sc, BWX_DOWN_START, BWX_DOWN_END);
	pci_disable_busmaster(sc->sc_dev);
}

static void
bwx_parent(struct ieee80211com *ic)
{
	struct bwx_softc *sc = ic->ic_softc;
	int startall = 0;

	BWX_LOCK(sc);
	if (ic->ic_nrunning > 0) {
		if (!sc->sc_running) {
			bwx_init_locked(sc);
			startall = 1;
		}
	} else
		bwx_stop_locked(sc);
	BWX_UNLOCK(sc);
	bwx_tx_reap(sc);
	if (startall)
		ieee80211_start_all(ic);
}

static void
bwx_set_channel(struct ieee80211com *ic)
{
	struct bwx_softc *sc = ic->ic_softc;

	BWX_LOCK(sc);
	if (sc->sc_running)
		bwx_tune(sc, bwx_chanspec(ic->ic_curchan));
	BWX_UNLOCK(sc);
}

static void
bwx_scan_start(struct ieee80211com *ic)
{
}

static void
bwx_scan_end(struct ieee80211com *ic)
{
}

static void
bwx_update_mcast(struct ieee80211com *ic)
{
}

static int
bwx_wme_update(struct ieee80211com *ic)
{
	/* The card keeps the queue parameters from the recorded bring-up. */
	return (0);
}

static void
bwx_getradiocaps(struct ieee80211com *ic, int maxchans, int *nchans,
    struct ieee80211_channel chans[])
{
	/* Every channel here has a recorded tuning sequence. */
	static const uint8_t chan5[] = {
		36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120,
		124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165
	};
	uint8_t bands[IEEE80211_MODE_BYTES];

	memset(bands, 0, sizeof(bands));
	setbit(bands, IEEE80211_MODE_11B);
	setbit(bands, IEEE80211_MODE_11G);
	setbit(bands, IEEE80211_MODE_11NG);
	ieee80211_add_channels_default_2ghz(chans, maxchans, nchans, bands, 0);
	memset(bands, 0, sizeof(bands));
	setbit(bands, IEEE80211_MODE_11A);
	setbit(bands, IEEE80211_MODE_11NA);
	ieee80211_add_channel_list_5ghz(chans, maxchans, nchans, chan5,
	    nitems(chan5), bands, 0);
	if (!bwx_vht)
		return;
	/* 802.11ac at 80 MHz, only where a tune was recorded. */
	setbit(bands, IEEE80211_MODE_VHT_5GHZ);
	(void)ieee80211_add_channel_cbw(chans, maxchans, nchans,
	    BWX_CHAN_80_PRIMARY,
	    ieee80211_ieee2mhz(BWX_CHAN_80_PRIMARY, IEEE80211_CHAN_A), 0, 0,
	    bands, NET80211_CBW_FLAG_HT40 | NET80211_CBW_FLAG_VHT80);
}

static int
bwx_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct bwx_vap *bv = (struct bwx_vap *)vap;
	struct bwx_softc *sc = vap->iv_ic->ic_softc;

	if (nstate != IEEE80211_S_RUN) {
		BWX_LOCK(sc);
		bwx_agg_flush_all(sc);
		BWX_UNLOCK(sc);
	}
	return (bv->bv_newstate(vap, nstate, arg));
}

static struct ieee80211vap *
bwx_vap_create(struct ieee80211com *ic, const char name[IFNAMSIZ], int unit,
    enum ieee80211_opmode opmode, int flags,
    const uint8_t bssid[IEEE80211_ADDR_LEN],
    const uint8_t mac[IEEE80211_ADDR_LEN])
{
	struct bwx_softc *sc = ic->ic_softc;
	struct bwx_vap *bv;
	struct ieee80211vap *vap;

	if (!TAILQ_EMPTY(&ic->ic_vaps))
		return (NULL);
	bv = malloc(sizeof(*bv), M_80211_VAP, M_WAITOK | M_ZERO);
	vap = &bv->bv_vap;
	/* NOBEACONS makes net80211 count missed beacons itself. */
	if (ieee80211_vap_setup(ic, vap, name, unit, opmode,
	    flags | IEEE80211_CLONE_NOBEACONS, bssid) != 0) {
		free(bv, M_80211_VAP);
		return (NULL);
	}
	bv->bv_newstate = vap->iv_newstate;
	vap->iv_newstate = bwx_newstate;
	if (!bwx_ampdu)
		vap->iv_flags_ht &= ~IEEE80211_FHT_AMPDU_TX;
	/* net80211 defaults to 8 KB bursts, about five full frames. */
	vap->iv_ampdu_rxmax = IEEE80211_HTCAP_MAXRXAMPDU_64K;
	ieee80211_ratectl_init(vap);
	ieee80211_vap_attach(vap, ieee80211_media_change,
	    ieee80211_media_status, mac);
	if (tcp_lro_init(&sc->sc_lro) == 0) {
		sc->sc_lro.ifp = vap->iv_ifp;
		sc->sc_if_input = if_getinputfn(vap->iv_ifp);
		if_setinputfn(vap->iv_ifp, bwx_vap_input);
		sc->sc_lro_ready = true;
	}
	ic->ic_opmode = opmode;
	return (vap);
}

static void
bwx_vap_delete(struct ieee80211vap *vap)
{
	struct bwx_softc *sc = vap->iv_ic->ic_softc;

	if (sc->sc_lro_ready) {
		/* The poll is the only user, and it checks this under sc_mtx. */
		BWX_LOCK(sc);
		sc->sc_lro_ready = false;
		BWX_UNLOCK(sc);
		if_setinputfn(vap->iv_ifp, sc->sc_if_input);
		tcp_lro_free(&sc->sc_lro);
	}
	BWX_LOCK(sc);
	bwx_agg_flush_all(sc);
	BWX_UNLOCK(sc);
	bwx_tx_reap(sc);
	ieee80211_ratectl_deinit(vap);
	ieee80211_vap_detach(vap);
	free(vap, M_80211_VAP);
}

static void *
bwx_contig(size_t size, size_t align)
{
	return (contigmalloc(size, M_BWX, M_WAITOK | M_ZERO, 0,
	    BUS_SPACE_MAXADDR_32BIT, align, 0));
}

static void
bwx_rings_init(struct bwx_softc *sc)
{
	struct bwx_txq *txq;
	int i, q;

	sc->sc_rxring = bwx_contig(BWX_RING_BYTES, BWX_RING_BYTES);
	sc->sc_rxbufs = bwx_contig(BWX_NDESC * BWX_BUF_BYTES, PAGE_SIZE);
	for (q = 0; q < BWX_NTXQ; q++) {
		txq = &sc->sc_txq[q];
		txq->fifo = q;
		txq->ring = bwx_contig(BWX_RING_BYTES, BWX_RING_BYTES);
		txq->bufs = bwx_contig(BWX_NDESC * BWX_BUF_BYTES, PAGE_SIZE);
		for (i = 0; i < BWX_NDESC; i++) {
			txq->txd[i].tid = -1;
			txq->ring[i].addrlow = htole32(vtophys(txq->bufs) +
			    i * BWX_BUF_BYTES);
			txq->ring[i].addrhigh = htole32(BWX_DMA_H32);
		}
	}
	for (i = 0; i < BWX_NDESC; i++) {
		sc->sc_rxring[i].ctrl1 = htole32(i == BWX_NDESC - 1 ?
		    BWX_DD_EOT : 0);
		sc->sc_rxring[i].ctrl2 = htole32(BWX_BUF_BYTES);
		sc->sc_rxring[i].addrlow = htole32(vtophys(sc->sc_rxbufs) +
		    i * BWX_BUF_BYTES);
		sc->sc_rxring[i].addrhigh = htole32(BWX_DMA_H32);
	}
}

static int
bwx_probe(device_t dev)
{
	if (pci_get_vendor(dev) != 0x14e4 || pci_get_device(dev) != 0x43a0)
		return (ENXIO);
	device_set_desc(dev, "Broadcom BCM4360 802.11ac (experimental)");
	return (BUS_PROBE_DEFAULT);
}

static int
bwx_attach(device_t dev)
{
	struct bwx_softc *sc = device_get_softc(dev);
	struct ieee80211com *ic = &sc->sc_ic;
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid_list *tree =
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	uint16_t w;
	int i;

	sc->sc_dev = dev;
	sc->sc_lro_on = bwx_lro;
	sc->sc_agg_mcs = 7;
	sc->sc_agg_auto = 1;
	mbufq_init(&sc->sc_txok, INT_MAX);
	mbufq_init(&sc->sc_txfail, INT_MAX);
	for (i = 0; i < BWX_NTID; i++)
		mbufq_init(&sc->sc_agg[i].q, BWX_AGG_QLEN);
	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	callout_init_mtx(&sc->sc_poll, &sc->sc_mtx, 0);

	/*
	 * bhyve gives a passed-through device no legacy interrupt, and the
	 * driver polls, so keep the card from raising one at all.
	 */
	pci_write_config(dev, PCIR_COMMAND,
	    pci_read_config(dev, PCIR_COMMAND, 2) | PCIM_CMD_INTxDIS, 2);
	pci_disable_busmaster(dev);

	sc->sc_rid = PCIR_BAR(0);
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->sc_rid,
	    RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		mtx_destroy(&sc->sc_mtx);
		return (ENXIO);
	}
	bwx_rings_init(sc);

	/* Chipcommon is always powered, so the SROM is safe to read here. */
	for (i = 0; i < 3; i++) {
		w = bus_read_2(sc->sc_mem, BWX_SROM_MACADDR + i * 2);
		ic->ic_macaddr[i * 2] = w >> 8;
		ic->ic_macaddr[i * 2 + 1] = w & 0xff;
	}

	ic->ic_softc = sc;
	ic->ic_name = device_get_nameunit(dev);
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	/*
	 * C_DFS is here so ifconfig's regdomain code keeps channels 52-144.
	 * As a station the driver only follows an access point onto them.
	 */
	ic->ic_caps = IEEE80211_C_STA | IEEE80211_C_MONITOR | IEEE80211_C_WPA |
	    IEEE80211_C_SHSLOT | IEEE80211_C_SHPREAMBLE | IEEE80211_C_DFS |
	    IEEE80211_C_WME;
	/* 802.11n, receive side only for now. */
	ic->ic_htcaps = IEEE80211_HTC_HT | IEEE80211_HTC_AMPDU |
	    IEEE80211_HTCAP_SMPS_OFF | IEEE80211_HTCAP_SHORTGI20;
	ic->ic_rxstream = 3;
	ic->ic_txstream = 3;
	if (bwx_vht) {
		/*
		 * 802.11ac receive: three streams, MCS 0-9, 1 MB bursts.
		 * net80211 will not pick an 80 MHz channel without the
		 * 40 MHz bit.
		 */
		ic->ic_htcaps |= IEEE80211_HTCAP_CHWIDTH40 |
		    IEEE80211_HTCAP_SHORTGI40;
		ic->ic_flags_ext |= IEEE80211_FEXT_VHT;
		ic->ic_vht_cap.vht_cap_info = IEEE80211_VHTCAP_MAX_MPDU_LENGTH_3895 |
		    IEEE80211_VHTCAP_SHORT_GI_80 |
		    (7 << IEEE80211_VHTCAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT);
		ic->ic_vht_cap.supp_mcs.rx_mcs_map = ic->ic_vht_cap.supp_mcs.tx_mcs_map =
		    htole16(IEEE80211_VHT_MCS_SUPPORT_0_9 << 0 |
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 2 |
		    IEEE80211_VHT_MCS_SUPPORT_0_9 << 4 |
		    IEEE80211_VHT_MCS_NOT_SUPPORTED << 6 |
		    IEEE80211_VHT_MCS_NOT_SUPPORTED << 8 |
		    IEEE80211_VHT_MCS_NOT_SUPPORTED << 10 |
		    IEEE80211_VHT_MCS_NOT_SUPPORTED << 12 |
		    IEEE80211_VHT_MCS_NOT_SUPPORTED << 14);
	}
	bwx_getradiocaps(ic, IEEE80211_CHAN_MAX, &ic->ic_nchans,
	    ic->ic_channels);
	ieee80211_ifattach(ic);
	ic->ic_vap_create = bwx_vap_create;
	ic->ic_vap_delete = bwx_vap_delete;
	ic->ic_parent = bwx_parent;
	ic->ic_transmit = bwx_transmit;
	ic->ic_raw_xmit = bwx_raw_xmit;
	ic->ic_set_channel = bwx_set_channel;
	ic->ic_scan_start = bwx_scan_start;
	ic->ic_scan_end = bwx_scan_end;
	ic->ic_update_mcast = bwx_update_mcast;
	ic->ic_wme.wme_update = bwx_wme_update;
	ic->ic_getradiocaps = bwx_getradiocaps;
	sc->sc_ampdu_enable = ic->ic_ampdu_enable;
	ic->ic_ampdu_enable = bwx_ampdu_enable;
	sc->sc_addba_stop = ic->ic_addba_stop;
	ic->ic_addba_stop = bwx_addba_stop;

	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "rx", CTLFLAG_RD, &sc->sc_nrx, 0,
	    "frames received");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "rx_bad", CTLFLAG_RD,
	    &sc->sc_nrxbad, 0, "received frames dropped");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "tx", CTLFLAG_RD, &sc->sc_ntx, 0,
	    "frames queued to the card");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "tx_drop", CTLFLAG_RD,
	    &sc->sc_ntxdrop, 0, "frames not sent");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "tx_status", CTLFLAG_RD,
	    &sc->sc_ntxstatus, 0, "transmit status reports from the card");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "tx_tracked", CTLFLAG_RD,
	    &sc->sc_ntxtracked, 0, "unicast data frames with a status report");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "tx_noack", CTLFLAG_RD,
	    &sc->sc_ntxnoack, 0, "tracked frames that were not acknowledged");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "tx_stray", CTLFLAG_RD,
	    &sc->sc_ntxstray, 0, "status reports that matched no frame");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "tx_stale", CTLFLAG_RD,
	    &sc->sc_ntxstale, 0, "times a ring's status reports stopped");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "agg_tx", CTLFLAG_RD,
	    &sc->sc_nagg, 0, "frames sent for aggregation, resends included");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "agg_acked", CTLFLAG_RD,
	    &sc->sc_naggack, 0, "aggregated frames acknowledged");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "agg_retry", CTLFLAG_RD,
	    &sc->sc_naggretry, 0, "aggregated frames sent again");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "agg_drop", CTLFLAG_RD,
	    &sc->sc_naggdrop, 0, "aggregated frames given up on");
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "agg_mcs", CTLFLAG_RW,
	    &sc->sc_agg_mcs, 0, "802.11ac rate index for aggregates, 0-9");
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "agg_auto", CTLFLAG_RW,
	    &sc->sc_agg_auto, 0, "adjust agg_mcs from the loss rate");
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "tx_debug", CTLFLAG_RW,
	    &sc->sc_txdebug, 0, "log this many unusual transmit status reports");
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "tx_debug_all", CTLFLAG_RW,
	    &sc->sc_txdebug_all, 0, "log this many transmit status reports");
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "lro", CTLFLAG_RW, &sc->sc_lro_on,
	    0, "coalesce received TCP segments");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "lro_frames", CTLFLAG_RD,
	    &sc->sc_nlro, 0, "frames handed to the coalescing code");
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "debug", CTLFLAG_RW, &sc->sc_debug,
	    0, "log this many received frames");
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "segment", CTLFLAG_RD, &sc->sc_seg,
	    0, "current channel segment");

	if (bootverbose)
		ieee80211_announce(ic);
	return (0);
}

static int
bwx_detach(device_t dev)
{
	struct bwx_softc *sc = device_get_softc(dev);

	ieee80211_ifdetach(&sc->sc_ic);
	BWX_LOCK(sc);
	bwx_stop_locked(sc);
	BWX_UNLOCK(sc);
	callout_drain(&sc->sc_poll);
	bwx_tx_reap(sc);
	pci_disable_busmaster(dev);
	bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_rid, sc->sc_mem);
	for (int q = 0; q < BWX_NTXQ; q++) {
		free(sc->sc_txq[q].bufs, M_BWX);
		free(sc->sc_txq[q].ring, M_BWX);
	}
	free(sc->sc_rxbufs, M_BWX);
	free(sc->sc_rxring, M_BWX);
	mtx_destroy(&sc->sc_mtx);
	return (0);
}

static device_method_t bwx_methods[] = {
	DEVMETHOD(device_probe,		bwx_probe),
	DEVMETHOD(device_attach,	bwx_attach),
	DEVMETHOD(device_detach,	bwx_detach),
	DEVMETHOD_END
};

static driver_t bwx_driver = {
	"bwx", bwx_methods, sizeof(struct bwx_softc)
};

DRIVER_MODULE(bwx, pci, bwx_driver, 0, 0);
MODULE_DEPEND(bwx, pci, 1, 1, 1);
MODULE_DEPEND(bwx, wlan, 1, 1, 1);
