/*
 * Copyright (c) 2004-2008 Texas Instruments
 *
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
MEMORY
{
#define FLASH_BASE 0x08000000
	FLASH (r) :	ORIGIN = FLASH_BASE, LENGTH = 2 << 20
	SRAM (rw) :	ORIGIN = 0x20000000, LENGTH = 20 << 10
}
SECTIONS
{
	/* Must be first so that it can be mapped to 0x00000000 */
	.vectors FLASH_BASE :
	{
		*(.vectors)
	} > FLASH

	.text :
	{
		*(.__image_copy_start)
		CPUDIR/start.o (.text*)
		*(.text*)
	} > FLASH

	.rodata : {
		. = ALIGN(4);
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
	} > FLASH

	.data : {
		. = ALIGN(4);
		*(.data*)
	} > SRAM AT> FLASH

	.u_boot_list : {
		. = ALIGN(4);
		KEEP(*(SORT(.u_boot_list*)));
	} > FLASH

	.image_copy_end :
	{
		. = ALIGN(4);
		*(.__image_copy_end)
	} > FLASH

/*
	.rel_dyn_start :
	{
		*(.__rel_dyn_start)
	} > FLASH

	.rel.dyn : {
		*(.rel*)
	} > FLASH

	.rel_dyn_end :
	{
		*(.__rel_dyn_end)
	} > FLASH

	.end :
	{
		*(.__end)
		_image_binary_end = .;
	} > FLASH
*/
/*
 * Compiler-generated __bss_start and __bss_end, see arch/arm/lib/sections.c
 * __bss_base and __bss_limit are for linker only (overlay ordering)
 */
/*
	.bss_start __rel_dyn_start (OVERLAY) : {
		KEEP(*(.__bss_start));
		__bss_base = .;
	} > SRAM

	.bss __bss_base (OVERLAY) : {
		*(.bss*)
		 . = ALIGN(4);
		 __bss_limit = .;
	} > SRAM

	.bss_end __bss_limit (OVERLAY) : {
		KEEP(*(.__bss_end));
	} > SRAM

	.dynsym _image_binary_end : { *(.dynsym) }
	.dynbss : { *(.dynbss) }
	.dynstr : { *(.dynstr*) }
	.dynamic : { *(.dynamic*) }
	.plt : { *(.plt*) }
	.interp : { *(.interp*) }
	.gnu.hash : { *(.gnu.hash) }
	.gnu : { *(.gnu*) }
	.ARM.exidx : { *(.ARM.exidx*) }
	.gnu.linkonce.armexidx : { *(.gnu.linkonce.armexidx.*) }
*/

	.bss :
	{
		_bss_start = .;
		*(.bss*)
		*(COMMON)
		_bss_end = .;
	} > SRAM

	/DISCARD/ :
	{
		*(*)
	}
}
