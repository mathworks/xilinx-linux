// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

#define SPINOR_OP_IS_DTR_RD	0xfd	/* Fast Read opcode in DTR mode */
#define SPINOR_OP_IS_RD_ANY_REG	0x85	/* Read volatile register */
#define SPINOR_OP_IS_WR_ANY_REG	0x81	/* Write volatile register */
#define SPINOR_REG_IS_CFR0V	0x00	/* For setting octal DTR mode */
#define SPINOR_REG_IS_CFR1V	0x01	/* For setting dummy cycles */
#define SPINOR_IS_OCT_DTR	0xe7	/* Enable Octal DTR. */
#define SPINOR_IS_EXSPI		0xff	/* Enable Extended SPI (default) */

static int spi_nor_issi_octal_dtr_enable(struct spi_nor *nor, bool enable)
{
	struct spi_mem_op op;
	u8 *buf = nor->bouncebuf;
	int ret;

	if (enable) {
		/* Use 20 dummy cycles for memory array reads. */
		ret = spi_nor_write_enable(nor);
		if (ret)
			return ret;

		*buf = 20;
		op = (struct spi_mem_op)
			SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_IS_WR_ANY_REG, 1),
				   SPI_MEM_OP_ADDR(3, SPINOR_REG_IS_CFR1V, 1),
				   SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_DATA_OUT(1, buf, 1));

		ret = spi_mem_exec_op(nor->spimem, &op);
		if (ret)
			return ret;

		ret = spi_nor_wait_till_ready(nor);
		if (ret)
			return ret;
	}

	ret = spi_nor_write_enable(nor);
	if (ret)
		return ret;

	if (enable)
		*buf = SPINOR_IS_OCT_DTR;
	else
		*buf = SPINOR_IS_EXSPI;

	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_IS_WR_ANY_REG, 1),
			   SPI_MEM_OP_ADDR(enable ? 3 : 4,
					   SPINOR_REG_IS_CFR0V, 1),
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_DATA_OUT(1, buf, 1));

	if (!enable)
		spi_nor_spimem_setup_op(nor, &op, SNOR_PROTO_8_8_8_DTR);

	ret = spi_mem_exec_op(nor->spimem, &op);
	if (ret)
		return ret;

	if ((nor->flags & SNOR_F_HAS_STACKED) && nor->spimem->spi->cs_index_mask == 1)
		return 0;

	/* Read flash ID to make sure the switch was successful. */
	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_RDID, 1),
			   SPI_MEM_OP_NO_ADDR,
			   SPI_MEM_OP_DUMMY(enable ? 8 : 0, 1),
			   SPI_MEM_OP_DATA_IN(round_up(nor->info->id_len, 2),
					      buf, 1));

	if (enable)
		spi_nor_spimem_setup_op(nor, &op, SNOR_PROTO_8_8_8_DTR);

	ret = spi_mem_exec_op(nor->spimem, &op);
	if (ret)
		return ret;

	if (memcmp(buf, nor->info->id, nor->info->id_len))
		return -EINVAL;

	return 0;
}

static int is25wx256_set_4byte_addr_mode(struct spi_nor *nor, bool enable)
{
	int ret;

	ret = spi_nor_write_enable(nor);
	if (ret)
		return ret;

	ret = spi_nor_set_4byte_addr_mode(nor, enable);
	if (ret)
		return ret;

	return spi_nor_write_disable(nor);
}

static void is25wx256_default_init(struct spi_nor *nor)
{
	struct spi_nor_flash_parameter *params = spi_nor_get_params(nor, 0);

	params->set_octal_dtr = spi_nor_issi_octal_dtr_enable;
	params->set_4byte_addr_mode = is25wx256_set_4byte_addr_mode;
}

static int is25wx256_post_sfdp_fixup(struct spi_nor *nor)
{
	struct spi_nor_flash_parameter *params = spi_nor_get_params(nor, 0);

	/* Set the Fast Read settings. */
	params->hwcaps.mask |= SNOR_HWCAPS_READ_8_8_8_DTR;
	spi_nor_set_read_settings(&params->reads[SNOR_CMD_READ_8_8_8_DTR],
				  0, 20, SPINOR_OP_IS_DTR_RD,
				  SNOR_PROTO_8_8_8_DTR);

	nor->cmd_ext_type = SPI_NOR_EXT_REPEAT;
	params->rdsr_dummy = 8;
	params->rdsr_addr_nbytes = 0;

	/*
	 * The BFPT quad enable field is set to a reserved value so the quad
	 * enable function is ignored by spi_nor_parse_bfpt(). Make sure we
	 * disable it.
	 */
	params->quad_enable = NULL;

	return 0;
}

static struct spi_nor_fixups is25wx256_fixups = {
	.default_init = is25wx256_default_init,
	.post_sfdp = is25wx256_post_sfdp_fixup,
};

static int
is25lp256_post_bfpt_fixups(struct spi_nor *nor,
			   const struct sfdp_parameter_header *bfpt_header,
			   const struct sfdp_bfpt *bfpt)
{
	struct spi_nor_flash_parameter *params = spi_nor_get_params(nor, 0);

	/*
	 * IS25LP256 supports 4B opcodes, but the BFPT advertises
	 * BFPT_DWORD1_ADDRESS_BYTES_3_ONLY.
	 * Overwrite the number of address bytes advertised by the BFPT.
	 */
	if ((bfpt->dwords[SFDP_DWORD(1)] & BFPT_DWORD1_ADDRESS_BYTES_MASK) ==
		BFPT_DWORD1_ADDRESS_BYTES_3_ONLY)
		params->addr_nbytes = 4;

	return 0;
}

static const struct spi_nor_fixups is25lp256_fixups = {
	.post_bfpt = is25lp256_post_bfpt_fixups,
};

static int pm25lv_nor_late_init(struct spi_nor *nor)
{
	struct spi_nor_flash_parameter *params = spi_nor_get_params(nor, 0);
	struct spi_nor_erase_map *map = &params->erase_map;
	int i;

	/* The PM25LV series has a different 4k sector erase opcode */
	for (i = 0; i < SNOR_ERASE_TYPE_MAX; i++)
		if (map->erase_type[i].size == 4096)
			map->erase_type[i].opcode = SPINOR_OP_BE_4K_PMC;

	return 0;
}

static const struct spi_nor_fixups pm25lv_nor_fixups = {
	.late_init = pm25lv_nor_late_init,
};

static const struct flash_info issi_nor_parts[] = {
	/* ISSI */
	{ "is25cd512",  INFO(0x7f9d20, 0, 32 * 1024,   2)
		NO_SFDP_FLAGS(SECT_4K) },
	{ "is25lq040b", INFO(0x9d4013, 0, 64 * 1024,   8)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25lp016d", INFO(0x9d6015, 0, 64 * 1024,  32)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25lp080d", INFO(0x9d6014, 0, 64 * 1024,  16)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25lp032",  INFO(0x9d6016, 0, 64 * 1024,  64)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ) },
	{ "is25lp064",  INFO(0x9d6017, 0, 64 * 1024, 128)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ) },
	{ "is25lp128",  INFO(0x9d6018, 0, 64 * 1024, 256)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_BP3_SR_BIT5)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ) },
	{ "is25lp256",  INFO(0x9d6019, 0, 64 * 1024, 512)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES)
		.fixups = &is25lp256_fixups },
	{ "is25wp256d", INFO(0x9d7019, 0, 64 * 1024, 512)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "is25wp032",  INFO(0x9d7016, 0, 64 * 1024,  64)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25wp064",  INFO(0x9d7017, 0, 64 * 1024, 128)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25wp128",  INFO(0x9d7018, 0, 64 * 1024, 256)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_BP3_SR_BIT5)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25wp256", INFO(0x9d7019, 0, 64 * 1024, 512)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES)
		FLAGS(SPI_NOR_QUAD_PP)
		.fixups = &is25lp256_fixups },
	{ "is25lp512m", INFO(0x9d601a, 0, 64 * 1024, 1024)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25wp512m", INFO(0x9d701a, 0, 64 * 1024, 1024)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "is25lp01g", INFO(0x9d601b, 0, 64 * 1024, 2048)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "is25wp01g", INFO(0x9d701b, 0, 64 * 1024, 2048)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "is25lp02g", INFO(0x9d6022, 0, 64 * 1024, 4096)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
				SPI_NOR_TB_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES) },
	{ "is25wx256",  INFO(0x9d5b19, 0, 128 * 1024, 256)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
		      SPI_NOR_BP3_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_OCTAL_READ |
			   SPI_NOR_OCTAL_DTR_READ | SPI_NOR_OCTAL_DTR_PP)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES | SPI_NOR_IO_MODE_EN_VOLATILE)
		MFR_FLAGS(USE_FSR)
		.fixups = &is25wx256_fixups },
	{ "is25lx512m",  INFO(0x9d5a1a, 0, 128 * 1024, 512)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP |
		      SPI_NOR_BP3_SR_BIT6)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_OCTAL_READ |
			   SPI_NOR_OCTAL_DTR_READ | SPI_NOR_OCTAL_DTR_PP)
		FIXUP_FLAGS(SPI_NOR_4B_OPCODES | SPI_NOR_IO_MODE_EN_VOLATILE)
		MFR_FLAGS(USE_FSR)
		.fixups = &is25wx256_fixups },

	/* PMC */
	{ "pm25lv512",   INFO(0,        0, 32 * 1024,    2)
		NO_SFDP_FLAGS(SECT_4K)
		.fixups = &pm25lv_nor_fixups
	},
	{ "pm25lv010",   INFO(0,        0, 32 * 1024,    4)
		NO_SFDP_FLAGS(SECT_4K)
		.fixups = &pm25lv_nor_fixups
	},
	{ "pm25lq032",   INFO(0x7f9d46, 0, 64 * 1024,   64)
		NO_SFDP_FLAGS(SECT_4K) },
};

static void issi_nor_default_init(struct spi_nor *nor)
{
	struct spi_nor_flash_parameter *params = spi_nor_get_params(nor, 0);

	nor->flags &= ~SNOR_F_HAS_16BIT_SR;
	params->quad_enable = spi_nor_sr1_bit6_quad_enable;
}

static const struct spi_nor_fixups issi_fixups = {
	.default_init = issi_nor_default_init,
};

const struct spi_nor_manufacturer spi_nor_issi = {
	.name = "issi",
	.parts = issi_nor_parts,
	.nparts = ARRAY_SIZE(issi_nor_parts),
	.fixups = &issi_fixups,
};
