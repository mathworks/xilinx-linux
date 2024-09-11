// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx ZynqMP SHA Driver.
 * Copyright (c) 2022 Xilinx Inc.
 * Copyright (C) 2022-2023, Advanced Micro Devices, Inc.
 */
#include <linux/cacheflush.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/sha3.h>
#include <linux/crypto.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware/xlnx-zynqmp.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define CONTINUE_PACKET		BIT(31)
#define FIRST_PACKET		BIT(30)
#define FINAL_PACKET		0
#define RESET			0

#define ZYNQMP_DMA_BIT_MASK		32U
#define ZYNQMP_DMA_ALLOC_FIXED_SIZE	0x1000U

enum zynqmp_sha_op {
	ZYNQMP_SHA3_INIT = 1,
	ZYNQMP_SHA3_UPDATE = 2,
	ZYNQMP_SHA3_FINAL = 4,
};

struct xilinx_sha_drv_ctx {
	struct shash_alg sha3_384;
	struct device *dev;
};

struct zynqmp_sha_tfm_ctx {
	struct device *dev;
	struct crypto_shash *fbk_tfm;
};

struct zynqmp_sha_desc_ctx {
	struct shash_desc fbk_req;
};

static dma_addr_t update_dma_addr, final_dma_addr;
static char *ubuf, *fbuf;

static int zynqmp_sha_init_tfm(struct crypto_shash *hash)
{
	const char *fallback_driver_name = crypto_shash_alg_name(hash);
	struct zynqmp_sha_tfm_ctx *tfm_ctx = crypto_shash_ctx(hash);
	struct shash_alg *alg = crypto_shash_alg(hash);
	struct crypto_shash *fallback_tfm;
	struct xilinx_sha_drv_ctx *drv_ctx;

	drv_ctx = container_of(alg, struct xilinx_sha_drv_ctx, sha3_384);
	tfm_ctx->dev = drv_ctx->dev;

	/* Allocate a fallback and abort if it failed. */
	fallback_tfm = crypto_alloc_shash(fallback_driver_name, 0,
					  CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(fallback_tfm))
		return PTR_ERR(fallback_tfm);

	tfm_ctx->fbk_tfm = fallback_tfm;
	hash->descsize += crypto_shash_descsize(tfm_ctx->fbk_tfm);

	return 0;
}

static void zynqmp_sha_exit_tfm(struct crypto_shash *hash)
{
	struct zynqmp_sha_tfm_ctx *tfm_ctx = crypto_shash_ctx(hash);

	if (tfm_ctx->fbk_tfm) {
		crypto_free_shash(tfm_ctx->fbk_tfm);
		tfm_ctx->fbk_tfm = NULL;
	}

	memzero_explicit(tfm_ctx, sizeof(struct zynqmp_sha_tfm_ctx));
}

static int zynqmp_sha_init(struct shash_desc *desc)
{
	struct zynqmp_sha_desc_ctx *dctx = shash_desc_ctx(desc);
	struct zynqmp_sha_tfm_ctx *tctx = crypto_shash_ctx(desc->tfm);

	dctx->fbk_req.tfm = tctx->fbk_tfm;
	return crypto_shash_init(&dctx->fbk_req);
}

static int zynqmp_sha_update(struct shash_desc *desc, const u8 *data, unsigned int length)
{
	struct zynqmp_sha_desc_ctx *dctx = shash_desc_ctx(desc);

	return crypto_shash_update(&dctx->fbk_req, data, length);
}

static int zynqmp_sha_final(struct shash_desc *desc, u8 *out)
{
	struct zynqmp_sha_desc_ctx *dctx = shash_desc_ctx(desc);

	return crypto_shash_final(&dctx->fbk_req, out);
}

static int zynqmp_sha_finup(struct shash_desc *desc, const u8 *data, unsigned int length, u8 *out)
{
	struct zynqmp_sha_desc_ctx *dctx = shash_desc_ctx(desc);

	return crypto_shash_finup(&dctx->fbk_req, data, length, out);
}

static int zynqmp_sha_import(struct shash_desc *desc, const void *in)
{
	struct zynqmp_sha_desc_ctx *dctx = shash_desc_ctx(desc);
	struct zynqmp_sha_tfm_ctx *tctx = crypto_shash_ctx(desc->tfm);

	dctx->fbk_req.tfm = tctx->fbk_tfm;
	return crypto_shash_import(&dctx->fbk_req, in);
}

static int zynqmp_sha_export(struct shash_desc *desc, void *out)
{
	struct zynqmp_sha_desc_ctx *dctx = shash_desc_ctx(desc);

	return crypto_shash_export(&dctx->fbk_req, out);
}

static int zynqmp_sha_digest(struct shash_desc *desc, const u8 *data, unsigned int len, u8 *out)
{
	unsigned int remaining_len = len;
	int update_size;
	int ret;

	ret = zynqmp_pm_sha_hash(0, 0, ZYNQMP_SHA3_INIT);
	if (ret)
		return ret;

	while (remaining_len != 0) {
		memzero_explicit(ubuf, ZYNQMP_DMA_ALLOC_FIXED_SIZE);
		if (remaining_len >= ZYNQMP_DMA_ALLOC_FIXED_SIZE) {
			update_size = ZYNQMP_DMA_ALLOC_FIXED_SIZE;
			remaining_len -= ZYNQMP_DMA_ALLOC_FIXED_SIZE;
		} else {
			update_size = remaining_len;
			remaining_len = 0;
		}
		memcpy(ubuf, data, update_size);
		flush_icache_range((unsigned long)ubuf, (unsigned long)ubuf + update_size);
		ret = zynqmp_pm_sha_hash(update_dma_addr, update_size, ZYNQMP_SHA3_UPDATE);
		if (ret)
			return ret;

		data += update_size;
	}

	ret = zynqmp_pm_sha_hash(final_dma_addr, SHA3_384_DIGEST_SIZE, ZYNQMP_SHA3_FINAL);
	memcpy(out, fbuf, SHA3_384_DIGEST_SIZE);
	memzero_explicit(fbuf, SHA3_384_DIGEST_SIZE);

	return ret;
}

static int versal_sha_digest(struct shash_desc *desc, const u8 *data,
			     unsigned int len, u8 *out)
{
	int update_size, ret, flag = FIRST_PACKET;
	unsigned int remaining_len = len;

	while (remaining_len != 0) {
		memzero_explicit(ubuf, ZYNQMP_DMA_ALLOC_FIXED_SIZE);
		if (remaining_len >= ZYNQMP_DMA_ALLOC_FIXED_SIZE) {
			update_size = ZYNQMP_DMA_ALLOC_FIXED_SIZE;
			remaining_len -= ZYNQMP_DMA_ALLOC_FIXED_SIZE;
		} else {
			update_size = remaining_len;
			remaining_len = 0;
		}

		memcpy(ubuf, data, update_size);
		flush_icache_range((unsigned long)ubuf,
				   (unsigned long)ubuf + update_size);

		flag |= CONTINUE_PACKET;
		ret = versal_pm_sha_hash(update_dma_addr, 0,
					 update_size | flag);
		if (ret)
			return ret;

		data += update_size;
		flag = RESET;
	}

	flag |= FINAL_PACKET;
	ret = versal_pm_sha_hash(0, final_dma_addr, flag);
	if (ret)
		return ret;

	memcpy(out, fbuf, SHA3_384_DIGEST_SIZE);
	memzero_explicit(fbuf, SHA3_384_DIGEST_SIZE);

	return 0;
}

static struct xilinx_sha_drv_ctx zynqmp_sha3_drv_ctx = {
	.sha3_384 = {
		.init = zynqmp_sha_init,
		.update = zynqmp_sha_update,
		.final = zynqmp_sha_final,
		.finup = zynqmp_sha_finup,
		.digest = zynqmp_sha_digest,
		.export = zynqmp_sha_export,
		.import = zynqmp_sha_import,
		.init_tfm = zynqmp_sha_init_tfm,
		.exit_tfm = zynqmp_sha_exit_tfm,
		.descsize = sizeof(struct zynqmp_sha_desc_ctx),
		.statesize = sizeof(struct sha3_state),
		.digestsize = SHA3_384_DIGEST_SIZE,
		.base = {
			.cra_name = "sha3-384",
			.cra_driver_name = "zynqmp-sha3-384",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY |
				     CRYPTO_ALG_ALLOCATES_MEMORY |
				     CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize = SHA3_384_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct zynqmp_sha_tfm_ctx),
			.cra_alignmask = 3,
			.cra_module = THIS_MODULE,
		}
	}
};

static struct xilinx_sha_drv_ctx versal_sha3_drv_ctx = {
	.sha3_384 = {
		.init = zynqmp_sha_init,
		.update = zynqmp_sha_update,
		.final = zynqmp_sha_final,
		.finup = zynqmp_sha_finup,
		.export = zynqmp_sha_export,
		.import = zynqmp_sha_import,
		.digest = versal_sha_digest,
		.init_tfm = zynqmp_sha_init_tfm,
		.exit_tfm = zynqmp_sha_exit_tfm,
		.descsize = sizeof(struct zynqmp_sha_desc_ctx),
		.statesize = sizeof(struct sha3_state),
		.digestsize = SHA3_384_DIGEST_SIZE,
		.base = {
			.cra_name = "sha3-384",
			.cra_driver_name = "versal-sha3-384",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY |
				CRYPTO_ALG_ALLOCATES_MEMORY |
				CRYPTO_ALG_NEED_FALLBACK,
			.cra_blocksize = SHA3_384_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct zynqmp_sha_tfm_ctx),
			.cra_alignmask = 3,
			.cra_module = THIS_MODULE,
		}
	}
};

static struct xlnx_feature sha_feature_map[] = {
	{
		.family = ZYNQMP_FAMILY_CODE,
		.subfamily = ALL_SUB_FAMILY_CODE,
		.feature_id = PM_SECURE_SHA,
		.data = &zynqmp_sha3_drv_ctx,
	},
	{
		.family = VERSAL_FAMILY_CODE,
		.subfamily = VERSAL_SUB_FAMILY_CODE,
		.feature_id = XSECURE_API_SHA3_UPDATE,
		.data = &versal_sha3_drv_ctx,
	},
	{ /* sentinel */ }
};

static int zynqmp_sha_probe(struct platform_device *pdev)
{
	struct xilinx_sha_drv_ctx *sha3_drv_ctx;
	struct device *dev = &pdev->dev;
	int err;

	/* Verify the hardware is present */
	sha3_drv_ctx = xlnx_get_crypto_dev_data(sha_feature_map);
	if (IS_ERR(sha3_drv_ctx)) {
		dev_err(dev, "SHA is not supported on the platform\n");
		return PTR_ERR(sha3_drv_ctx);
	}

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(ZYNQMP_DMA_BIT_MASK));
	if (err < 0) {
		dev_err(dev, "No usable DMA configuration\n");
		return err;
	}

	sha3_drv_ctx->dev = dev;
	platform_set_drvdata(pdev, sha3_drv_ctx);

	ubuf = dma_alloc_coherent(dev, ZYNQMP_DMA_ALLOC_FIXED_SIZE, &update_dma_addr, GFP_KERNEL);
	if (!ubuf) {
		err = -ENOMEM;
		return err;
	}

	fbuf = dma_alloc_coherent(dev, SHA3_384_DIGEST_SIZE, &final_dma_addr, GFP_KERNEL);
	if (!fbuf) {
		err = -ENOMEM;
		goto err_mem;
	}

	err = crypto_register_shash(&sha3_drv_ctx->sha3_384);
	if (err < 0) {
		dev_err(dev, "Failed to register shash alg.\n");
		goto err_mem1;
	}
	return 0;

err_mem1:
	dma_free_coherent(dev, SHA3_384_DIGEST_SIZE, fbuf, final_dma_addr);

err_mem:
	dma_free_coherent(dev, ZYNQMP_DMA_ALLOC_FIXED_SIZE, ubuf, update_dma_addr);

	return err;
}

static int zynqmp_sha_remove(struct platform_device *pdev)
{
	struct xilinx_sha_drv_ctx *sha3_drv_ctx;

	sha3_drv_ctx = platform_get_drvdata(pdev);

	dma_free_coherent(sha3_drv_ctx->dev,
			  ZYNQMP_DMA_ALLOC_FIXED_SIZE, ubuf, update_dma_addr);
	dma_free_coherent(sha3_drv_ctx->dev,
			  SHA3_384_DIGEST_SIZE, fbuf, final_dma_addr);
	crypto_unregister_shash(&sha3_drv_ctx->sha3_384);

	return 0;
}

static struct platform_driver zynqmp_sha_driver = {
	.probe = zynqmp_sha_probe,
	.remove = zynqmp_sha_remove,
	.driver = {
		.name = "zynqmp-sha3-384",
	},
};

static int __init sha_driver_init(void)
{
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&zynqmp_sha_driver);
	if (ret)
		return ret;

	pdev = platform_device_register_simple(zynqmp_sha_driver.driver.name,
					       0, NULL, 0);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		platform_driver_unregister(&zynqmp_sha_driver);
	}

	return ret;
}

static void __exit sha_driver_exit(void)
{
	platform_driver_unregister(&zynqmp_sha_driver);
}

device_initcall(sha_driver_init);
module_exit(sha_driver_exit);

MODULE_DESCRIPTION("ZynqMP SHA3 hardware acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Harsha <harsha.harsha@xilinx.com>");
