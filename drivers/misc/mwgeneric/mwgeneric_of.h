#include "mwgeneric.h"
/* Open firmware includes */
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define ip_to_pdev(x)  (container_of(x->dev, struct platform_device, dev))

#define DRIVER_NAME "mwgeneric_of"
