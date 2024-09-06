// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx DisplayPort Sound PCM support
 *
 *  Copyright (C) 2015 Xilinx, Inc.
 *
 *  Author: Hyun Woo Kwon <hyunk@xilinx.com>
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#define DP_PCM_NAME_0 "zynqmp_dp_snd_pcm0"
#define DP_PCM_NAME_1 "zynqmp_dp_snd_pcm1"

static const struct snd_pcm_hardware xilinx_pcm_hw = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,
	.buffer_bytes_max	= 128 * 1024,
	.period_bytes_min	= 256,
	.period_bytes_max	= 1024 * 1024,
	.periods_min		= 2,
	.periods_max		= 256,
};

static int xilinx_dp_pcm_prepare_slave_config(struct snd_pcm_substream *substream,
					      struct snd_pcm_hw_params *params,
					      struct dma_slave_config *slave_config)
{
	/*
	 * The slave_config is not required to be configured as
	 * the Xilinx DPDMA is hardwired to DP controller. But
	 * this is added to avoid NULL pointer dereference
	 * while configuring DMA DAI data later.
	 */
	return 0;
}

static const struct snd_dmaengine_pcm_config xilinx_dmaengine_pcm_config = {
	.prepare_slave_config = xilinx_dp_pcm_prepare_slave_config,
	.pcm_hardware = &xilinx_pcm_hw,
	.prealloc_buffer_size = 64 * 1024,
};

static int xilinx_dp_pcm_probe(struct platform_device *pdev)
{
	int ret;

	if (of_device_is_compatible(pdev->dev.of_node, "xlnx,dp-snd-pcm0"))
		dev_set_name(&pdev->dev, DP_PCM_NAME_0);
	else if (of_device_is_compatible(pdev->dev.of_node, "xlnx,dp-snd-pcm1"))
		dev_set_name(&pdev->dev, DP_PCM_NAME_1);
	else
		dev_set_name(&pdev->dev, pdev->dev.of_node->name);

	pdev->name = dev_name(&pdev->dev);
	ret = devm_snd_dmaengine_pcm_register(&pdev->dev,
					      &xilinx_dmaengine_pcm_config, 0);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Xilinx DisplayPort Sound PCM probed\n");

	return 0;
}

static const struct of_device_id xilinx_dp_pcm_of_match[] = {
	{ .compatible = "xlnx,dp-snd-pcm", },
	{ .compatible = "xlnx,dp-snd-pcm0", },
	{ .compatible = "xlnx,dp-snd-pcm1", },
	{ /* end of table */ },
};
MODULE_DEVICE_TABLE(of, xilinx_dp_pcm_of_match);

static struct platform_driver xilinx_dp_pcm_driver = {
	.driver	= {
		.name		= "xilinx-dp-snd-pcm",
		.of_match_table	= xilinx_dp_pcm_of_match,
	},
	.probe	= xilinx_dp_pcm_probe,
};
module_platform_driver(xilinx_dp_pcm_driver);

MODULE_DESCRIPTION("Xilinx DisplayPort Sound PCM module");
MODULE_LICENSE("GPL v2");
