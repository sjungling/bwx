// SPDX-License-Identifier: GPL-2.0
/*
 * Throwaway: dump the buffers a dma64 transmit ring points at, to learn the
 * frame layout the vendor driver hands the card. Load with ring=<phys addr>.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mm.h>

static unsigned long ring;
module_param(ring, ulong, 0444);
static unsigned int ndesc = 512;
module_param(ndesc, uint, 0444);
static unsigned int dumplen = 224;
module_param(dumplen, uint, 0444);

struct dma64dd {
	__le32 ctrl1, ctrl2, addrlow, addrhigh;
};

static int __init ringdump_init(void)
{
	struct dma64dd *dd;
	unsigned int i, len;
	phys_addr_t pa;

	if (!ring || !pfn_valid(PHYS_PFN(ring)))
		return -EINVAL;
	dd = phys_to_virt(ring);
	for (i = 0; i < ndesc; i++) {
		u32 c1 = le32_to_cpu(dd[i].ctrl1), c2 = le32_to_cpu(dd[i].ctrl2);

		pa = le32_to_cpu(dd[i].addrlow) |
		    ((u64)(le32_to_cpu(dd[i].addrhigh) & 0x7fffffff) << 32);
		if (!pa)
			continue;
		len = min(c2 & 0x7fff, dumplen);
		pr_info("ringdump: dd[%u] ctrl1=%08x ctrl2=%08x pa=%pa\n", i, c1,
		    c2, &pa);
		if (!pfn_valid(PHYS_PFN(pa)))
			continue;
		print_hex_dump(KERN_INFO, "ringdump:   ", DUMP_PREFIX_OFFSET, 32,
		    1, phys_to_virt(pa), len, false);
	}
	return 0;
}

static void __exit ringdump_exit(void)
{
}

module_init(ringdump_init);
module_exit(ringdump_exit);
MODULE_LICENSE("GPL");
