// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * OpenOCD flash driver for Westberry WB32F10x and WB32FQ95xx microcontrollers
 *
 * Copyright (C) 2026 Eric Molitor github.com/emolitor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Supported devices:
 * WB32F10x series: ARM Cortex-M3 with up to 256KB flash
 * - WB32F101xx: 32-64 KB Flash, 8-20 KB SRAM
 * - WB32F103xx: 64-128 KB Flash, 12-28 KB SRAM
 * - WB32F104xx: 96-256 KB Flash, 20-36 KB SRAM
 * - WB32F105xx: 128-256 KB Flash, 28-36 KB SRAM
 *
 * WB32FQ95xx series: ARM Cortex-M3 with up to 256KB flash
 * - WB32FQ95xx: 32-256 KB Flash, 12-36 KB SRAM
 *
 * Implementation approach:
 * Uses direct FMC register writes from the OpenOCD host instead of a
 * target-resident flash algorithm. This is more reliable because:
 * 1. PRE_OP calibration is run on-target before flash sessions
 * 2. No algorithm timeout/BKPT detection issues
 * 3. Matches the proven manual register-write workaround exactly
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/arm_adi_v5.h>
#include <target/armv7m.h>

/* WB32F10x Memory Map */
#define WB32_FLASH_BASE         0x08000000
#define WB32_SRAM_BASE          0x20000000

/* WB32F10x Peripheral Base Addresses */
#define WB32_PWR_BASE           0x40010000
#define WB32_ANCTL_BASE         0x40010400
#define WB32_RCC_BASE           0x40010C00
#define WB32_SYS_BASE           0x40016400
#define WB32_FMC_BASE           0x40017800

/* PWR Registers (for ANCTL unlock) */
#define WB32_PWR_ANAKEY1        (WB32_PWR_BASE + 0x028)
#define WB32_PWR_ANAKEY2        (WB32_PWR_BASE + 0x02C)

/* ANCTL Registers (for FHSI oscillator) */
#define WB32_ANCTL_FHSIENR      (WB32_ANCTL_BASE + 0x038)
#define WB32_ANCTL_FHSISR       (WB32_ANCTL_BASE + 0x03C)

/* RCC Registers */
#define WB32_RCC_PCLKENR        (WB32_RCC_BASE + 0x060)

/* SYS Registers (Device ID and Memory Size) */
#define WB32_SYS_ID             (WB32_SYS_BASE + 0x000)
#define WB32_SYS_MEMSZ          (WB32_SYS_BASE + 0x004)

/* FMC Registers */
#define WB32_FMC_CON            (WB32_FMC_BASE + 0x000)
#define WB32_FMC_CRCON          (WB32_FMC_BASE + 0x004)
#define WB32_FMC_STAT           (WB32_FMC_BASE + 0x008)
#define WB32_FMC_KEY            (WB32_FMC_BASE + 0x00C)
#define WB32_FMC_ADDR           (WB32_FMC_BASE + 0x010)
#define WB32_FMC_DATA1          (WB32_FMC_BASE + 0x01C)
#define WB32_FMC_BUF            (WB32_FMC_BASE + 0x100)

/* FMC_CON Register Bits */
#define FMC_CON_OP_MASK         0x1F
#define FMC_CON_WREN            (1 << 6)
#define FMC_CON_WR              (1 << 7)

/* FMC_STAT Register Bits */
#define FMC_STAT_ERR            (1 << 2)

/* FMC Operation Codes */
#define FMC_OP_CLEAR_LATCH      0x04
#define FMC_OP_PAGE_ERASE       0x08
#define FMC_OP_SECTOR_ERASE     0x09
#define FMC_OP_PAGE_PROGRAM     0x0C
#define FMC_OP_CHIP_ERASE       0x1B

/* FMC_CON values: magic prefix (0x7F5F0D) | WREN bit | operation code */
#define FMC_CON_BASE            0x7F5F0D40
#define FMC_CON_TRIGGER         0x00800080

/* FMC Unlock Keys */
#define FMC_KEY1                0x5188DA08
#define FMC_KEY2                0x12586590

/* ANCTL Unlock Keys */
#define ANCTL_KEY1              0x03
#define ANCTL_KEY2              0x0C

/* Flash Geometry */
#define WB32_PAGE_SIZE          256
#define WB32_SECTOR_SIZE        4096
#define WB32_PAGES_PER_SECTOR   16

/* Timeout values (in ms) */
#define WB32_FLASH_TIMEOUT      1000
#define WB32_FHSI_TIMEOUT       100
#define WB32_MASS_ERASE_TIMEOUT 30000

/* Cortex-M Debug registers (for lockup recovery) */
#define DCB_DHCSR           0xE000EDF0
#define DCB_DHCSR_DBGKEY    (0xA05Ful << 16)
#define DCB_DHCSR_C_DEBUGEN (1ul << 0)
#define DCB_DHCSR_C_HALT    (1ul << 1)
#define DCB_DHCSR_S_LOCKUP  (1ul << 19)
#define NVIC_AIRCR          0xE000ED0C
#define AIRCR_VECTKEY       (0x5FAul << 16)
#define AIRCR_VECTCLRACTIVE (1ul << 1)

/* CHIP_ID values (extracted from SYS_ID bits [23:18]) */
#define CHIP_ID_WB32F101        0x11
#define CHIP_ID_WB32F102        0x12
#define CHIP_ID_WB32F103        0x13
#define CHIP_ID_WB32F104        0x14
#define CHIP_ID_WB32F105        0x15
#define CHIP_ID_WB32FQ95        0x3A

/* Device-specific data */
struct wb32f10x_flash_bank {
	bool probed;
	uint32_t device_id;
	uint8_t chip_id;       /* CHIP_ID from SYS_ID[23:18] */
	uint32_t flash_size;
	uint32_t sram_size;
};

/* Flash size lookup table based on SYS_MEMSZ[3:0] */
static const struct {
	uint8_t code;
	uint32_t flash_size;
} wb32f10x_flash_sizes[] = {
	{ 0x03, 256 * 1024 },
	{ 0x04, 128 * 1024 },
	{ 0x06,  96 * 1024 },
	{ 0x07,  64 * 1024 },
	{ 0x0F, 192 * 1024 },
	{ 0x00,  32 * 1024 },
};

/*
 * PRE_OP calibration routine (ChibiOS variant, 228 bytes / 57 words)
 *
 * This pre-compiled Thumb-2 routine reads factory timing calibration
 * data from 0x1FFFF000 and configures the flash controller. It MUST
 * be called before any erase or program operation.
 *
 * Source: ChibiOS-Contrib hal_efl_lld.c (WB32FQ95xx port)
 * Calling convention: void PRE_OP(void) — no parameters, no return value
 */
static const uint32_t wb32_pre_op_chibios[] = {
	0x4FF0E92D, 0x21034832, 0x210C6281, 0xF8DF62C1,
	0x2100C0C4, 0x1000F8CC, 0xF44F4608, 0x1C40767A,
	0xDBFC42B0, 0xF8CC2201, 0x20002000, 0x42B01C40,
	0x4829DBFC, 0xF0436803, 0x60030380, 0x302C4826,
	0xF4436803, 0x60036320, 0x46104691, 0x323C4A22,
	0x468A6010, 0x49214608, 0x48216008, 0x0340F8D0,
	0x25004F1E, 0x5107F3C0, 0x3BFFF04F, 0x22001F3F,
	0x4610465C, 0xEA5F683B, 0xD10678C0, 0xD10142A3,
	0xE0002401, 0x44222400, 0x1C40461C, 0xDBF12814,
	0xD91B2A02, 0xD9012910, 0xE0003910, 0x480D2100,
	0x68021F00, 0x627FF022, 0x5201EA42, 0xF8CC6002,
	0x2000A000, 0x42B01C40, 0xF8CCDBFC, 0x20009000,
	0x42B01C40, 0x1C6DDBFC, 0xDBD02D05, 0x8FF0E8BD,
	0x40010000, 0x40010438, 0x40010C20, 0x4000B804,
	0x1FFFF000
};

/*
 * PRE_OP calibration routine (vendor library variant, 240 bytes / 60 words)
 *
 * Source: WB32F10x_StdPeriph_Lib wb32f10x_fmc.c
 * This variant includes an additional check on the chip revision.
 * Use for WB32F10x family if ChibiOS variant doesn't work.
 */
static const uint32_t __attribute__((unused)) wb32_pre_op_vendor[] = {
	0x4FF0E92D, 0xF8D14935, 0xF3C00200, 0x28023083,
	0x4833D861, 0x62822203, 0x62C2220C, 0xC0C4F8DF,
	0xF8CC2200, 0x46102000, 0x767AF44F, 0x42B01C40,
	0x2301DBFC, 0x3000F8CC, 0x1C402000, 0xDBFC42B0,
	0x68044829, 0x0480F044, 0x48276004, 0x6804302C,
	0x6420F444, 0x469A6004, 0x4B234618, 0x6018333C,
	0x46104691, 0x60104A21, 0x0340F8D1, 0xF3C02500,
	0xF04F5107, 0x1F173BFF, 0x465C2200, 0x683B4610,
	0x78C0EA5F, 0x42A3D106, 0x2401D101, 0x2400E000,
	0x461C4422, 0x28141C40, 0xDBF12A02, 0x2910D91B,
	0x3910D901, 0x2100E000, 0x1F00480E, 0xF0226802,
	0xEA42627F, 0x60025201, 0x9000F8CC, 0x1C402000,
	0xDBFC42B0, 0xA000F8CC, 0x1C402000, 0xDBFC42B0,
	0x2D051C6D, 0xE8BDDBD0, 0x00008FF0, 0x1FFFF000,
	0x40010000, 0x40010438, 0x40010C20, 0x4000B804
};

/*
 * PRE_OP wrapper (12 bytes Thumb-2 code):
 *   LDR   R0, [PC, #4]   ; Load PRE_OP address (Thumb bit set)
 *   BLX   R0             ; Call PRE_OP
 *   BKPT  #0             ; Signal completion to OpenOCD
 *   .word pre_op_addr|1  ; PRE_OP function pointer (Thumb mode)
 *
 * Machine code:
 *   0x4801    LDR R0, [PC, #4]
 *   0x4780    BLX R0
 *   0xBE00    BKPT #0
 *   0x0000    NOP (padding for alignment)
 *   <4 bytes: address of PRE_OP code | 1>
 */
static const uint8_t pre_op_wrapper[] = {
	0x01, 0x48,  /* LDR R0, [PC, #4] */
	0x80, 0x47,  /* BLX R0 */
	0x00, 0xBE,  /* BKPT #0 */
	0x00, 0x00,  /* NOP (alignment padding) */
	/* 4 bytes for PRE_OP address follow dynamically */
};
#define PRE_OP_WRAPPER_SIZE     8
#define PRE_OP_WRAPPER_TOTAL    12  /* 8 bytes code + 4 bytes address */

/* ========================================================================== */
/* Internal Helper Functions                                                  */
/* ========================================================================== */

/**
 * Enable the FHSI (48 MHz Fast High-Speed Internal) oscillator.
 * Required before any flash operation.
 */
static int wb32f10x_enable_fhsi(struct target *target)
{
	int retval;
	uint32_t fhsisr;
	int timeout = WB32_FHSI_TIMEOUT;

	retval = target_read_u32(target, WB32_ANCTL_FHSISR, &fhsisr);
	if (retval != ERROR_OK)
		return retval;

	if (fhsisr & 0x01) {
		LOG_DEBUG("FHSI already enabled and ready");
		return ERROR_OK;
	}

	/* Unlock ANCTL registers */
	retval = target_write_u32(target, WB32_PWR_ANAKEY1, ANCTL_KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, WB32_PWR_ANAKEY2, ANCTL_KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* Enable FHSI */
	retval = target_write_u32(target, WB32_ANCTL_FHSIENR, 0x01);
	if (retval != ERROR_OK)
		return retval;

	while (timeout > 0) {
		retval = target_read_u32(target, WB32_ANCTL_FHSISR, &fhsisr);
		if (retval != ERROR_OK)
			return retval;

		if (fhsisr & 0x01) {
			LOG_DEBUG("FHSI enabled and ready");
			return ERROR_OK;
		}

		alive_sleep(1);
		timeout--;
	}

	LOG_ERROR("Timeout waiting for FHSI oscillator");
	return ERROR_TIMEOUT_REACHED;
}

/**
 * Detect and recover from CPU lockup state.
 *
 * When flash contains a corrupted vector table, the CPU enters lockup on
 * reset (HardFault with invalid SP). This prevents target_run_algorithm()
 * from working, making the chip unprogrammable.
 *
 * Recovery uses VECTCLRACTIVE to clear exception state without triggering
 * a vector table fetch from potentially-corrupted flash.
 */
static int wb32f10x_recover_from_lockup(struct target *target)
{
	uint32_t dhcsr;
	int retval;

	retval = target_read_u32(target, DCB_DHCSR, &dhcsr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Could not read DHCSR — skipping lockup check");
		return ERROR_OK;
	}

	if (!(dhcsr & DCB_DHCSR_S_LOCKUP))
		return ERROR_OK;

	LOG_WARNING("CPU is in lockup state (DHCSR=0x%08" PRIx32
		") — attempting recovery", dhcsr);

	/* Halt the core to clear lockup state */
	retval = target_write_u32(target, DCB_DHCSR,
		DCB_DHCSR_DBGKEY | DCB_DHCSR_C_DEBUGEN | DCB_DHCSR_C_HALT);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to halt CPU during lockup recovery");
		return retval;
	}

	/* Clear all active exception state via VECTCLRACTIVE */
	retval = target_write_u32(target, NVIC_AIRCR,
		AIRCR_VECTKEY | AIRCR_VECTCLRACTIVE);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write AIRCR VECTCLRACTIVE");
		return retval;
	}

	/* Verify recovery */
	retval = target_read_u32(target, DCB_DHCSR, &dhcsr);
	if (retval != ERROR_OK)
		return retval;

	if (dhcsr & DCB_DHCSR_S_LOCKUP) {
		LOG_ERROR("CPU still in lockup after recovery attempt "
			"(DHCSR=0x%08" PRIx32 ")", dhcsr);
		return ERROR_TARGET_FAILURE;
	}

	LOG_WARNING("CPU lockup recovery successful — flash contained "
		"corrupted vector table");
	return ERROR_OK;
}

/**
 * Run the PRE_OP calibration routine on the target.
 *
 * PRE_OP reads factory timing calibration data from 0x1FFFF000 and
 * configures the flash controller. Both the vendor standard peripheral
 * library and ChibiOS call this before every erase/program operation.
 *
 * We call it once at the start of each flash session (erase or write).
 * The calibration remains valid as long as the target stays halted.
 */
static int wb32f10x_run_pre_op(struct target *target)
{
	struct working_area *wa = NULL;
	struct armv7m_algorithm armv7m_info;
	int retval;

	/* Recover from lockup before attempting to run algorithm */
	retval = wb32f10x_recover_from_lockup(target);
	if (retval != ERROR_OK)
		return retval;

	const uint32_t *pre_op_code = wb32_pre_op_chibios;
	uint32_t pre_op_size = sizeof(wb32_pre_op_chibios);

	/* Allocate extra 256 bytes for stack — PRE_OP only needs ~36 bytes
	 * (9 pushed registers) but we're generous to be safe. */
	uint32_t stack_size = 256;
	uint32_t total_size = PRE_OP_WRAPPER_TOTAL + pre_op_size + stack_size;

	retval = target_alloc_working_area(target, total_size, &wa);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to allocate working area for PRE_OP (%u bytes)",
			total_size);
		return retval;
	}

	/* Write the wrapper code */
	retval = target_write_buffer(target, wa->address,
		PRE_OP_WRAPPER_SIZE, pre_op_wrapper);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Write the PRE_OP function address (wrapper + 12 = start of PRE_OP code, with Thumb bit) */
	uint32_t pre_op_addr = (wa->address + PRE_OP_WRAPPER_TOTAL) | 1;
	uint8_t addr_bytes[4];
	target_buffer_set_u32(target, addr_bytes, pre_op_addr);
	retval = target_write_buffer(target, wa->address + PRE_OP_WRAPPER_SIZE,
		4, addr_bytes);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Write the PRE_OP code itself */
	retval = target_write_buffer(target, wa->address + PRE_OP_WRAPPER_TOTAL,
		pre_op_size, (const uint8_t *)pre_op_code);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Run it */
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* Pass a valid SP pointing to the top of the stack area.
	 * When flash contains garbage, SP is loaded from the corrupted
	 * vector table (often 0x00000000), causing PRE_OP to fault. */
	struct reg_param reg_params[1];
	init_reg_param(&reg_params[0], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, wa->address + total_size);

	LOG_DEBUG("Running PRE_OP calibration at 0x%08" TARGET_PRIxADDR
		" (SP=0x%08" TARGET_PRIxADDR ")", wa->address, wa->address + total_size);

	retval = target_run_algorithm(target,
		0, NULL,           /* no memory arguments */
		1, reg_params,     /* SP register parameter */
		wa->address, 0,    /* entry point, exit point (0 = detect BKPT) */
		WB32_FLASH_TIMEOUT, &armv7m_info);

	destroy_reg_param(&reg_params[0]);

	if (retval != ERROR_OK)
		LOG_ERROR("PRE_OP calibration failed");
	else
		LOG_DEBUG("PRE_OP calibration complete");

cleanup:
	target_free_working_area(target, wa);
	return retval;
}

/**
 * Log FMC diagnostic state for debugging flash operation failures.
 */
static void wb32f10x_log_fmc_state(struct target *target, const char *context)
{
	uint32_t fmc_con, fmc_stat, fhsisr, pclkenr;

	if (target_read_u32(target, WB32_FMC_CON, &fmc_con) == ERROR_OK &&
	    target_read_u32(target, WB32_FMC_STAT, &fmc_stat) == ERROR_OK &&
	    target_read_u32(target, WB32_ANCTL_FHSISR, &fhsisr) == ERROR_OK &&
	    target_read_u32(target, WB32_RCC_PCLKENR, &pclkenr) == ERROR_OK) {
		LOG_ERROR("%s: FMC_CON=0x%08" PRIx32 " FMC_STAT=0x%08" PRIx32
			" FHSISR=0x%08" PRIx32 " PCLKENR=0x%08" PRIx32,
			context, fmc_con, fmc_stat, fhsisr, pclkenr);
		if (fmc_con & FMC_CON_WR)
			LOG_ERROR("  WR bit still set — flash operation did not complete");
		if (fmc_stat & FMC_STAT_ERR)
			LOG_ERROR("  ERR bit set — flash operation error");
		if (!(fhsisr & 0x01))
			LOG_ERROR("  FHSI not ready — flash clock source missing");
		if (!(pclkenr & 0x01))
			LOG_ERROR("  Panel clock disabled — flash controller not clocked");
	}
}

/**
 * Wait for a flash operation to complete by polling FMC_CON.WR bit.
 */
static int wb32f10x_wait_flash_op(struct target *target, int timeout_ms)
{
	uint32_t fmc_con;
	int retval;

	while (timeout_ms > 0) {
		retval = target_read_u32(target, WB32_FMC_CON, &fmc_con);
		if (retval != ERROR_OK)
			return retval;

		if (!(fmc_con & FMC_CON_WR))
			return ERROR_OK;

		alive_sleep(1);
		timeout_ms--;
	}

	wb32f10x_log_fmc_state(target, "Flash operation timeout");
	return ERROR_TIMEOUT_REACHED;
}

/**
 * Execute a single FMC operation via direct register writes.
 *
 * Sequence for erase/program operations:
 *   0. Run PRE_OP calibration (required before every erase/program)
 *   1. Enable panel clock
 *   2. Set FMC_ADDR
 *   3. Set FMC_CON with operation code
 *   4. Unlock with KEY1, KEY2
 *   5. Trigger with 0x00800080
 *   6. Poll WR bit until clear
 *   7. Check FMC_STAT for errors
 *   8. Disable panel clock
 *   9. Clear FMC_CON
 *
 * PRE_OP is required before every erase and program operation per
 * both the vendor library (wb32f10x_fmc.c) and ChibiOS (hal_efl_lld.c).
 * Clear latch (op 0x04) does NOT require PRE_OP.
 */
static int wb32f10x_flash_op_direct(struct target *target,
	uint32_t addr, uint32_t op_code, int timeout_ms)
{
	uint32_t fmc_stat;
	int retval;

	/* PRE_OP calibration required before every erase/program operation.
	 * Clear latch (0x04) is the only operation that skips PRE_OP. */
	if (op_code != FMC_OP_CLEAR_LATCH) {
		retval = wb32f10x_run_pre_op(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("PRE_OP failed before flash op 0x%02x", op_code);
			return retval;
		}
	}

	/* Enable panel clock */
	retval = target_write_u32(target, WB32_RCC_PCLKENR, 0x01);
	if (retval != ERROR_OK)
		return retval;

	/* Set target address */
	retval = target_write_u32(target, WB32_FMC_ADDR, addr);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Set operation: magic prefix | WREN | op_code */
	retval = target_write_u32(target, WB32_FMC_CON,
		FMC_CON_BASE | op_code);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Unlock */
	retval = target_write_u32(target, WB32_FMC_KEY, FMC_KEY1);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, WB32_FMC_KEY, FMC_KEY2);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Trigger */
	retval = target_write_u32(target, WB32_FMC_CON, FMC_CON_TRIGGER);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Wait for completion */
	retval = wb32f10x_wait_flash_op(target, timeout_ms);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Check for errors */
	retval = target_read_u32(target, WB32_FMC_STAT, &fmc_stat);
	if (retval != ERROR_OK)
		goto cleanup;

	if (fmc_stat & FMC_STAT_ERR) {
		LOG_ERROR("Flash operation error (op=0x%02x addr=0x%08" PRIx32
			" FMC_STAT=0x%08" PRIx32 ")", op_code, addr, fmc_stat);
		retval = ERROR_FLASH_OPERATION_FAILED;
		goto cleanup;
	}

	/* Disable panel clock */
	target_write_u32(target, WB32_RCC_PCLKENR, 0x00);

	/* Clear FMC_CON */
	target_write_u32(target, WB32_FMC_CON, 0x005F0000);

	return ERROR_OK;

cleanup:
	/* Best-effort cleanup on error */
	target_write_u32(target, WB32_RCC_PCLKENR, 0x00);
	target_write_u32(target, WB32_FMC_CON, 0x005F0000);
	return retval;
}

static uint32_t wb32f10x_get_flash_size(uint8_t code)
{
	for (size_t i = 0; i < ARRAY_SIZE(wb32f10x_flash_sizes) - 1; i++) {
		if (wb32f10x_flash_sizes[i].code == code)
			return wb32f10x_flash_sizes[i].flash_size;
	}
	return 32 * 1024;
}

static uint32_t wb32f10x_get_sram_size(uint32_t sys_memsz)
{
	/*
	 * Both WB32F10x and WB32FQ95xx use SYS_MEMSZ bits [5:4] for SRAM size
	 * with the same encoding:
	 *   00: 36 KB
	 *   01: 28 KB
	 *   10: 20 KB
	 *   11: 12 KB
	 */
	static const uint32_t sram_sizes[] = {
		36 * 1024,  /* 0x0 */
		28 * 1024,  /* 0x1 */
		20 * 1024,  /* 0x2 */
		12 * 1024,  /* 0x3 */
	};

	uint8_t code = (sys_memsz >> 4) & 0x03;
	return sram_sizes[code];
}

/* ========================================================================== */
/* OpenOCD Flash Driver Interface Functions                                   */
/* ========================================================================== */

FLASH_BANK_COMMAND_HANDLER(wb32f10x_flash_bank_command)
{
	struct wb32f10x_flash_bank *wb32_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	wb32_info = calloc(1, sizeof(struct wb32f10x_flash_bank));
	if (!wb32_info)
		return ERROR_FAIL;

	bank->driver_priv = wb32_info;
	wb32_info->probed = false;

	return ERROR_OK;
}

static const char *wb32_get_device_name(uint8_t chip_id)
{
	switch (chip_id) {
	case CHIP_ID_WB32F101: return "WB32F101";
	case CHIP_ID_WB32F102: return "WB32F102";
	case CHIP_ID_WB32F103: return "WB32F103";
	case CHIP_ID_WB32F104: return "WB32F104";
	case CHIP_ID_WB32F105: return "WB32F105";
	case CHIP_ID_WB32FQ95: return "WB32FQ95";
	default: return "Unknown";
	}
}

static int wb32f10x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct wb32f10x_flash_bank *wb32_info = bank->driver_priv;
	uint32_t sys_id, sys_memsz;
	uint8_t flash_code;
	uint32_t num_sectors;
	int retval;

	retval = target_read_u32(target, WB32_SYS_ID, &sys_id);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read WB32 device ID");
		return retval;
	}

	retval = target_read_u32(target, WB32_SYS_MEMSZ, &sys_memsz);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read WB32 memory size");
		return retval;
	}

	wb32_info->device_id = sys_id;

	/*
	 * Chip family detection:
	 * CHIP_ID is in SYS_ID bits [23:18] for all WB32 devices.
	 * - WB32F10x:   CHIP_ID = 0x11-0x15 (WB32F101-WB32F105)
	 * - WB32FQ95xx: CHIP_ID is NOT in 0x11-0x15 range
	 *
	 * Note: Both families have SYS_ID[31:24] = 0x3A, so we cannot use
	 * the upper byte to distinguish them.
	 */
	uint8_t chip_id_raw = (sys_id >> 18) & 0x3F;
	if (chip_id_raw >= CHIP_ID_WB32F101 && chip_id_raw <= CHIP_ID_WB32F105) {
		wb32_info->chip_id = chip_id_raw;  /* WB32F10x family */
	} else {
		wb32_info->chip_id = CHIP_ID_WB32FQ95;  /* Assume WB32FQ95 for other values */
	}
	flash_code = sys_memsz & 0x0F;

	wb32_info->flash_size = wb32f10x_get_flash_size(flash_code);
	wb32_info->sram_size = wb32f10x_get_sram_size(sys_memsz);

	LOG_INFO("%s: Device ID=0x%08" PRIx32 ", Flash=%u KB, SRAM=%u KB",
		wb32_get_device_name(wb32_info->chip_id),
		wb32_info->device_id,
		wb32_info->flash_size / 1024,
		wb32_info->sram_size / 1024);

	bank->base = WB32_FLASH_BASE;
	bank->size = wb32_info->flash_size;

	free(bank->sectors);

	num_sectors = wb32_info->flash_size / WB32_SECTOR_SIZE;
	bank->num_sectors = num_sectors;

	bank->sectors = calloc(num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * WB32_SECTOR_SIZE;
		bank->sectors[i].size = WB32_SECTOR_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	wb32_info->probed = true;

	return ERROR_OK;
}

static int wb32f10x_auto_probe(struct flash_bank *bank)
{
	struct wb32f10x_flash_bank *wb32_info = bank->driver_priv;

	if (wb32_info->probed)
		return ERROR_OK;

	return wb32f10x_probe(bank);
}

static int wb32f10x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (first > last || last >= bank->num_sectors)
		return ERROR_FLASH_SECTOR_INVALID;

	retval = wb32f10x_enable_fhsi(target);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("Erasing sectors %u to %u", first, last);

	for (unsigned int sector = first; sector <= last; sector++) {
		uint32_t sector_addr = bank->base + (sector * WB32_SECTOR_SIZE);

		LOG_DEBUG("Erasing sector %u at 0x%08" PRIx32, sector, sector_addr);

		retval = wb32f10x_flash_op_direct(target, sector_addr,
			FMC_OP_SECTOR_ERASE, WB32_FLASH_TIMEOUT);
		if (retval != ERROR_OK) {
			LOG_ERROR("Sector erase failed at sector %u (0x%08" PRIx32 ")",
				sector, sector_addr);
			wb32f10x_log_fmc_state(target, "Sector erase failure");
			return retval;
		}

		bank->sectors[sector].is_erased = 1;
	}

	return ERROR_OK;
}

static int wb32f10x_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	retval = wb32f10x_enable_fhsi(target);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("Writing %" PRIu32 " bytes at 0x%08" TARGET_PRIxADDR, count, bank->base + offset);

	uint32_t bytes_written = 0;
	uint32_t address = bank->base + offset;

	while (bytes_written < count) {
		uint32_t page_addr = address & ~(WB32_PAGE_SIZE - 1);

		/* Step 1: Clear page latch (no PRE_OP needed per vendor pattern) */
		retval = wb32f10x_flash_op_direct(target, page_addr,
			FMC_OP_CLEAR_LATCH, WB32_FLASH_TIMEOUT);
		if (retval != ERROR_OK) {
			LOG_ERROR("Clear latch failed at 0x%08" PRIx32, page_addr);
			wb32f10x_log_fmc_state(target, "Clear latch failure");
			return retval;
		}

		/* Step 2: Write up to 256 bytes to FMC_BUF */
		uint32_t page_remaining = WB32_PAGE_SIZE;
		uint32_t data_remaining = count - bytes_written;
		uint32_t chunk = page_remaining < data_remaining ? page_remaining : data_remaining;

		if (chunk == WB32_PAGE_SIZE) {
			/* Full page — write directly */
			retval = target_write_memory(target, WB32_FMC_BUF, 4,
				WB32_PAGE_SIZE / 4, buffer + bytes_written);
		} else {
			/* Partial page — pad with 0x00 (erased state).
			 * WB32 flash erases to 0x00. Programming sets bits (0→1),
			 * so padding with 0x00 preserves the erased state of
			 * unwritten bytes within the page. */
			uint8_t page_buf[WB32_PAGE_SIZE];
			memset(page_buf, 0x00, WB32_PAGE_SIZE);
			memcpy(page_buf, buffer + bytes_written, chunk);
			retval = target_write_memory(target, WB32_FMC_BUF, 4,
				WB32_PAGE_SIZE / 4, page_buf);
		}
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write page data to FMC_BUF");
			return retval;
		}

		/* Step 3: Program page */
		retval = wb32f10x_flash_op_direct(target, page_addr,
			FMC_OP_PAGE_PROGRAM, WB32_FLASH_TIMEOUT);
		if (retval != ERROR_OK) {
			LOG_ERROR("Page program failed at 0x%08" PRIx32, page_addr);
			wb32f10x_log_fmc_state(target, "Page program failure");
			return retval;
		}

		bytes_written += chunk;
		address += chunk;
	}

	return ERROR_OK;
}

static int wb32f10x_erase_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t buffer[WB32_SECTOR_SIZE / 4];
	int retval;

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		uint32_t sector_addr = bank->base + bank->sectors[sector].offset;
		bool erased = true;

		retval = target_read_memory(target, sector_addr, 4,
			WB32_SECTOR_SIZE / 4, (uint8_t *)buffer);
		if (retval != ERROR_OK)
			return retval;

		/*
		 * WB32 erased flash reads as 0x00000000 (not 0xFF like
		 * conventional NOR flash). Confirmed by hardware test:
		 * after sector erase, all words read as 0x00000000.
		 * This matches ChibiOS efl_lld_verify_erase() which
		 * checks for 0x00.
		 */
		for (unsigned int i = 0; i < WB32_SECTOR_SIZE / 4; i++) {
			if (buffer[i] != 0x00000000) {
				erased = false;
				break;
			}
		}

		bank->sectors[sector].is_erased = erased ? 1 : 0;
	}

	return ERROR_OK;
}

static int wb32f10x_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	LOG_WARNING("WB32F10x flash protection not implemented");
	return ERROR_OK;
}

static int wb32f10x_protect_check(struct flash_bank *bank)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = 0;

	return ERROR_OK;
}

static int wb32f10x_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct wb32f10x_flash_bank *wb32_info = bank->driver_priv;

	if (!wb32_info->probed) {
		command_print_sameline(cmd, "WB32 flash bank not probed yet");
		return ERROR_OK;
	}

	command_print_sameline(cmd,
		"%s: Device ID=0x%08" PRIx32 ", Flash=%u KB, SRAM=%u KB, "
		"%u sectors of %u bytes",
		wb32_get_device_name(wb32_info->chip_id),
		wb32_info->device_id,
		wb32_info->flash_size / 1024,
		wb32_info->sram_size / 1024,
		bank->num_sectors,
		WB32_SECTOR_SIZE);

	return ERROR_OK;
}

static int wb32f10x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = wb32f10x_enable_fhsi(target);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("Performing mass erase...");

	retval = wb32f10x_flash_op_direct(target, WB32_FLASH_BASE,
		FMC_OP_CHIP_ERASE, WB32_MASS_ERASE_TIMEOUT);
	if (retval != ERROR_OK) {
		LOG_ERROR("Mass erase failed");
		wb32f10x_log_fmc_state(target, "Mass erase failure");
		return retval;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;

	LOG_INFO("Mass erase complete");

	return ERROR_OK;
}

COMMAND_HANDLER(wb32f10x_handle_mass_erase_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	return wb32f10x_mass_erase(bank);
}

static const struct command_registration wb32f10x_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = wb32f10x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash memory",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration wb32f10x_command_handlers[] = {
	{
		.name = "wb32f10x",
		.mode = COMMAND_ANY,
		.help = "WB32F10x flash driver commands",
		.usage = "",
		.chain = wb32f10x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver wb32f10x_flash = {
	.name = "wb32f10x",
	.commands = wb32f10x_command_handlers,
	.flash_bank_command = wb32f10x_flash_bank_command,
	.erase = wb32f10x_erase,
	.protect = wb32f10x_protect,
	.write = wb32f10x_write,
	.read = default_flash_read,
	.probe = wb32f10x_probe,
	.auto_probe = wb32f10x_auto_probe,
	.erase_check = wb32f10x_erase_check,
	.protect_check = wb32f10x_protect_check,
	.info = wb32f10x_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
