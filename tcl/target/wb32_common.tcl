# SPDX-License-Identifier: GPL-2.0-or-later
#
# Common helper procedures for Westberry WB32 microcontrollers
# Shared by wb32.cfg — the unified target config for all WB32 families.
#
# Note: Hardware testing confirms the WB32FQ95 reports chip_id=0x14
# (same as WB32F104) in SYS_ID[23:18]. The F10x flash-size table
# returns correct results for all flash_codes observed on real FQ95
# hardware. The FQ95-specific constants and lookup paths below are
# retained for any future variant that reports a different chip_id.
#
# Note: TCL switch uses string comparison, so all cases must use
# decimal values to match the output of [expr].
#

# Chip family constants (decimal, matching C driver CHIP_ID defines)
set _WB32_CHIP_ID_F101 17
set _WB32_CHIP_ID_F102 18
set _WB32_CHIP_ID_F103 19
set _WB32_CHIP_ID_F104 20
set _WB32_CHIP_ID_F105 21
set _WB32_CHIP_ID_FQ95 58

# Extract chip_id from SYS_ID register value.
# Returns decimal chip_id (17-21 for WB32F10x, 58 for WB32FQ95).
proc wb32_chip_id {sys_id} {
	return [expr {($sys_id >> 18) & 0x3F}]
}

# Return true if chip_id is in the WB32F10x family (F101-F105).
proc wb32_is_f10x {chip_id} {
	global _WB32_CHIP_ID_F101 _WB32_CHIP_ID_F105
	return [expr {$chip_id >= $_WB32_CHIP_ID_F101 && $chip_id <= $_WB32_CHIP_ID_F105}]
}

# Flash size lookup from SYS_MEMSZ[3:0] with chip_id disambiguation.
# Must match the C driver (wb32f10x.c) flash size tables.
proc wb32_flash_size_kb {flash_code chip_id} {
	if {[wb32_is_f10x $chip_id]} {
		# WB32F10x flash size encoding
		switch $flash_code {
			3 { return 256 }
			4 { return 128 }
			6 { return 96 }
			7 { return 64 }
			15 { return 192 }
			0 { return 32 }
			default { return "unknown (code=$flash_code)" }
		}
	} else {
		# WB32FQ95xx flash size encoding
		switch $flash_code {
			0 { return 256 }
			1 { return 128 }
			3 { return 256 }
			default { return "unknown (code=$flash_code)" }
		}
	}
}

# SRAM size lookup from SYS_MEMSZ[5:4]
proc wb32_sram_size_kb {sys_memsz} {
	set sram_code [expr {($sys_memsz >> 4) & 0x03}]
	switch $sram_code {
		0 { return 36 }
		1 { return 28 }
		2 { return 20 }
		3 { return 12 }
	}
}

# Display device information by reading SYS_ID and SYS_MEMSZ registers.
# Works for both WB32F10x and WB32FQ95xx families.
proc wb32_info {} {
	set sys_id [mrw 0x40016400]
	set sys_memsz [mrw 0x40016404]

	set chip_id [wb32_chip_id $sys_id]
	set flash_code [expr {$sys_memsz & 0x0F}]
	set sram_kb [wb32_sram_size_kb $sys_memsz]

	# chip_id is decimal: 17=WB32F101, 18=WB32F102, ..., 58=WB32FQ95
	switch $chip_id {
		17 { set name "WB32F101" }
		18 { set name "WB32F102" }
		19 { set name "WB32F103" }
		20 { set name "WB32F104" }
		21 { set name "WB32F105" }
		58 { set name "WB32FQ95" }
		default { set name "Unknown (chip_id=0x[format %02x $chip_id])" }
	}

	set flash_kb [wb32_flash_size_kb $flash_code $chip_id]

	echo "WB32 Device Information:"
	echo "  Device:      $name"
	echo "  SYS_ID:      0x[format %08x $sys_id]"
	echo "  SYS_MEMSZ:   0x[format %08x $sys_memsz]"
	echo "  Flash:       $flash_kb KB at 0x08000000"
	echo "  SRAM:        $sram_kb KB at 0x20000000"
	echo "  Core:        ARM Cortex-M3"
}

proc wb32_dump_flash {filename {length 0x1000}} {
	echo "Dumping $length bytes of flash to $filename..."
	dump_image $filename 0x08000000 $length
	echo "Done."
}

proc wb32_dump_all_flash {filename} {
	# Read registers and use chip_id to disambiguate flash size encoding
	set sys_id [mrw 0x40016400]
	set sys_memsz [mrw 0x40016404]
	set chip_id [wb32_chip_id $sys_id]
	set flash_code [expr {$sys_memsz & 0x0F}]
	set flash_kb [wb32_flash_size_kb $flash_code $chip_id]
	set size [expr {$flash_kb * 1024}]
	echo "Dumping entire flash ($flash_kb KB) to $filename..."
	dump_image $filename 0x08000000 $size
	echo "Done."
}

proc wb32_verify_flash {filename {offset 0x08000000}} {
	echo "Verifying flash against $filename at [format 0x%08x $offset]..."
	verify_image $filename $offset
	echo "Done."
}

proc wb32_load_ram {filename {address 0x20000000}} {
	echo "Loading $filename to RAM at $address..."
	load_image $filename $address
	echo "Done."
}
