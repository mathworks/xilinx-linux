#ifndef _MWGENERIC_H_
#define _MWGENERIC_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
/* Open firmware includes */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/kfifo.h>


#define	IP2DEV(x)	(x->pdev->dev)

struct mw_dma_info {
	void				*virt;
	dma_addr_t			phys;
	size_t				size;
};

struct ipcore_info {
    const char 			*name;
    struct resource 		*mem;
    void __iomem 		*regs;
    struct platform_device 	*pdev;
    struct cdev 		cdev;
    dev_t 			dev_id;
    int 			irq;
    struct fasync_struct 	*asyncq;
/*
 * DMA Virtual and physical address
 */
    struct mw_dma_info	dma_info;
};


#define MWGENERIC_MAX_DEVTYPE 32
#define	MWGENERIC_DEVNAME_LEN 32

struct mwgeneric_info {
	char			devname[MWGENERIC_DEVNAME_LEN];
	dev_t			devid;
	int				devcnt;
};

#endif
