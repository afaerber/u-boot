/*
 * u-boot/arch/arm/cpu/armv7/mb86s7x/timer.c
 *
 * Copyright (C) 2011-2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <common.h>
#include <div64.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>

/* Global Timer Register */
#define TIMER_REG_LOAD (CONFIG_SYS_TIMERBASE + 0x00)
#define TIMER_REG_VALUE (CONFIG_SYS_TIMERBASE + 0x04)
#define TIMER_REG_CONTROL (CONFIG_SYS_TIMERBASE + 0x08)


#define TIMER_LOAD_VAL 0xffffffff
#define TIMER_FREQ (CONFIG_TIMER_CLK / 16) /* Peripheral clock is divided by 16 */


DECLARE_GLOBAL_DATA_PTR;

#define timestamp (gd->tbl)
#define lastdec (gd->lastinc)


static inline unsigned long long tick_to_time(unsigned long long tick)
{
	tick *= CONFIG_SYS_HZ;
	do_div(tick, TIMER_FREQ);

	return tick;
}

static inline unsigned long long usec_to_tick(unsigned long long usec)
{
	usec *= TIMER_FREQ;
	do_div(usec, 1000000);

	return usec;
}

/* nothing really to do with interrupts, just starts up a counter. */
int timer_init(void)
{
	writel(TIMER_LOAD_VAL, TIMER_REG_LOAD);  /* decrement counter */

	writel(0x86, TIMER_REG_CONTROL);
	/* bit0: OnShot -> 0 = wrapping mode (default) */
	/* bit1: TimerSize -> 1 = 32-bit counter. */
	/* bit2-3: TimerPre -> 01 = 4 stages of prescale, clock is divided by 16 */
	/* bit4: Reserved */
	/* bit5: IntEnable -> 0 = Timer module Interrupt disabled */
	/* bit6: TimerMode -> 0 = Timer module is in free-running mode (default) */
	/* bit7: TimerEn -> 1 = Timer module enabled. */

	reset_timer_masked();

	return 0;
}

/*
 * timer without interrupts
 */
unsigned long long get_ticks(void)
{
	ulong now = readl(TIMER_REG_VALUE);

	if (now <= lastdec) { /* normal mode (non roll) */
	/* move stamp forward with absolut diff ticks */
		timestamp += (lastdec - now);
	}
	else {/* we have rollover of incrementer */
		timestamp += lastdec + TIMER_LOAD_VAL - now;
	}
	lastdec = now;
	return timestamp;
}

void reset_timer_masked(void)
{
	/* reset time */
	lastdec = readl(TIMER_REG_VALUE);
	timestamp = 0;
}

ulong get_timer_masked(void)
{
	return tick_to_time(get_ticks());
}

void __udelay(unsigned long usec)
{
	unsigned long long tmp;
	ulong tmo;

	tmo = usec_to_tick(usec);
	tmp = get_ticks() + tmo; /* get current timestamp */

	while (get_ticks() < tmp) /* loop till event */
 /*NOP*/;
}

void reset_timer(void)
{
	reset_timer_masked();
}

ulong get_timer(ulong base)
{
	return get_timer_masked() - base;
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return TIMER_FREQ;
}
