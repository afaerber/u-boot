/*
 * Ingenic JZ47xx GPIO
 *
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <asm/io.h>
#include <mach/jz4780.h>

int gpio_get_value(unsigned gpio)
{
	void __iomem *gpio_regs = (void __iomem *)GPIO_BASE;
	int port = gpio / 32;
	int pin = gpio % 32;

	return readl(gpio_regs + GPIO_PXPIN(port)) & BIT(pin);
}

int gpio_set_value(unsigned gpio, int value)
{
	void __iomem *gpio_regs = (void __iomem *)GPIO_BASE;
	int port = gpio / 32;
	int pin = gpio % 32;

	if (value)
		writel(BIT(pin), gpio_regs + GPIO_PXPAT0S(port));
	else
		writel(BIT(pin), gpio_regs + GPIO_PXPAT0C(port));

	return 0;
}

int gpio_direction_input(unsigned gpio)
{
	void __iomem *gpio_regs = (void __iomem *)GPIO_BASE;
	int port = gpio / 32;
	int pin = gpio % 32;

	writel(BIT(pin), gpio_regs + GPIO_PXINTC(port));
	writel(BIT(pin), gpio_regs + GPIO_PXMASKS(port));
	writel(BIT(pin), gpio_regs + GPIO_PXPAT1S(port));

	return 0;
}

int gpio_direction_output(unsigned gpio, int value)
{
	void __iomem *gpio_regs = (void __iomem *)GPIO_BASE;
	int port = gpio / 32;
	int pin = gpio % 32;

	writel(BIT(pin), gpio_regs + GPIO_PXINTC(port));
	writel(BIT(pin), gpio_regs + GPIO_PXMASKS(port));
	writel(BIT(pin), gpio_regs + GPIO_PXPAT1C(port));

	gpio_set_value(gpio, value);

	return 0;
}

int gpio_request(unsigned gpio, const char *label)
{
	int port = gpio / 32;

	if (port >= 6)
		return -EINVAL;

	return 0;
}

int gpio_free(unsigned gpio)
{
	return 0;
}
