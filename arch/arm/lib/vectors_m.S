/*
 * Copyright (c) 2014
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>
#include <linux/linkage.h>

_invalid_entry:
1:	b	1b
ENDPROC(_invalid_entry)

	.data
	.align	8
ENTRY(vector_table)
	.long	0		@  0: stack pointer
	.long	_invalid_entry	@  1
	.long	_invalid_entry	@  2
	.long	_invalid_entry	@  3
	.long	_invalid_entry	@  4
	.long	_invalid_entry	@  5
	.long	_invalid_entry	@  6
	.long	_invalid_entry	@  7: reserved
	.long	_invalid_entry	@  8: reserved
	.long	_invalid_entry	@  9: reserved
	.long	_invalid_entry	@ 10: reserved
	.long	_invalid_entry	@ 11
	.long	_invalid_entry	@ 12
	.long	_invalid_entry	@ 13: reserved
	.long	_invalid_entry	@ 14
	.long	_invalid_entry	@ 15
	.rept	91
	.long	_invalid_entry
	.endr
