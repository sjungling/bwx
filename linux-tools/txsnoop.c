// SPDX-License-Identifier: GPL-2.0
/*
 * Throwaway: log every buffer the wl driver maps for transmit, at the moment
 * it is mapped. The ring cannot be read afterwards, because the driver
 * poisons each descriptor once the frame has gone out.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/mm.h>

static unsigned int dumplen = 256;
module_param(dumplen, uint, 0644);

static int txsnoop_pre(struct kprobe *kp, struct pt_regs *regs)
{
	struct device *dev = (struct device *)regs->di;
	struct page *page = (struct page *)regs->si;
	unsigned long offset = regs->dx;
	size_t size = regs->cx;

	if ((int)regs->r8 != DMA_TO_DEVICE || !dev || !dev->driver ||
	    strcmp(dev->driver->name, "wl") != 0 || PageHighMem(page))
		return 0;
	pr_info("txsnoop: map len=%zu\n", size);
	print_hex_dump(KERN_INFO, "txsnoop:   ", DUMP_PREFIX_OFFSET, 32, 1,
	    page_address(page) + offset, min_t(size_t, size, dumplen), false);
	return 0;
}

static struct kprobe txsnoop_kp = {
	.symbol_name = "dma_map_page_attrs",
	.pre_handler = txsnoop_pre,
};

static int __init txsnoop_init(void)
{
	return register_kprobe(&txsnoop_kp);
}

static void __exit txsnoop_exit(void)
{
	unregister_kprobe(&txsnoop_kp);
}

module_init(txsnoop_init);
module_exit(txsnoop_exit);
MODULE_LICENSE("GPL");
