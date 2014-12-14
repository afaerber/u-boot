/*
 * Copyright (C) 2014
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <serial.h>

DECLARE_GLOBAL_DATA_PTR;

__weak struct serial_device *default_serial_console(void)
{
	return NULL;
}
