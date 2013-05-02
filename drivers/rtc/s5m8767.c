/*
 * Copyright (c) 2013 The Chromium OS Authors.
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <i2c.h>
#include <rtc.h>
#include <s5m8767.h>

DECLARE_GLOBAL_DATA_PTR;

enum {
	S5M8767_RTC_SEC_TK = 0x0,
	S5M8767_RTC_MIN_TK = 0x1,
	S5M8767_RTC_HR_TK = 0x2,
	S5M8767_RTC_WD_TK = 0x3,
	S5M8767_RTC_DATE_TK = 0x4,
	S5M8767_RTC_MON_TK = 0x5,
	S5M8767_RTC_YEAR_TK = 0x6,
	S5M8767_RTC_CENT_TK = 0x7,

	S5M8767_RTC_SEC_ALARM0 = 0x8,
	S5M8767_RTC_MIN_ALARM0 = 0x9,
	S5M8767_RTC_HR_ALARM0 = 0xa,
	S5M8767_RTC_WD_ALARM0 = 0xb,
	S5M8767_RTC_DATE_ALARM0 = 0xc,
	S5M8767_RTC_MON_ALARM0 = 0xd,
	S5M8767_RTC_YEAR_ALARM0 = 0xe,
	S5M8767_RTC_CENT_ALARM0 = 0xf,

	S5M8767_RTC_SEC_ALARM1 = 0x10,
	S5M8767_RTC_MIN_ALARM1 = 0x11,
	S5M8767_RTC_HR_ALARM1 = 0x12,
	S5M8767_RTC_WD_ALARM1 = 0x13,
	S5M8767_RTC_DATE_ALARM1 = 0x14,
	S5M8767_RTC_MON_ALARM1 = 0x15,
	S5M8767_RTC_YEAR_ALARM1 = 0x16,
	S5M8767_RTC_CENT_ALARM1 = 0x17,

	S5M8767_RTC_CTRLM = 0x18,
	S5M8767_RTC_CTRL = 0x19,
	S5M8767_RTC_TEST_STATUS = 0x1a,
	S5M8767_RTC_SMPL_WTSR = 0x1b,
	S5M8767_RTC_TEST_CON = 0x1c
};

enum {
	S5M8767_RTC_HR_TK_AM_PM = (0x1 << 6)
};

enum {
	S5M8767_RTC_CTRLM_BCDM = (0x1 << 0),
	S5M8767_RTC_CTRLM_MODE24_12NM = (0x1 << 1),
	S5M8767_RTC_CTRLM_ALMCENTM = (0x1 << 6),
	S5M8767_RTC_CTRLM_1SECM = (0x1 << 7)
};

enum {
	S5M8767_RTC_CTRL_BCD = (0x1 << 0),
	S5M8767_RTC_CTRL_MODE24_12N = (0x1 << 1),
	S5M8767_RTC_CTRL_ALMCENT = (0x1 << 6),
	S5M8767_RTC_CTRL_1SEC = (0x1 << 7)
};

enum {
	S5M8767_RTC_TEST_CON_UDR = (0x1 << 0),
	S5M8767_RTC_TEST_CON_SEC_FREEZE = (0x1 << 2),
	S5M8767_RTC_TEST_CON_TIME_EN = (0x1 << 3),
	S5M8767_RTC_TEST_CON_TEST_OSC = (0x1 << 4),
	S5M8767_RTC_TEST_CON_SEL_OSC = (0x1 << 5),
	S5M8767_RTC_TEST_CON_UDR_T_MASK = (0x3 << 6)
};

enum {
	DATA_SECOND,
	DATA_MINUTE,
	DATA_HOUR,
	DATA_WEEKDAY,
	DATA_DATE,
	DATA_MONTH,
	DATA_YEAR,
	DATA_CENTURY,

	DATA_SIZE
};


static int s5m8767_rtc_write(unsigned int reg, uint8_t val)
{
	return i2c_write(S5M8767_RTC_I2C_ADDR, reg, 1, &val, 1);
}

static int s5m8767_rtc_read(unsigned int reg, uint8_t *val)
{
	return i2c_read(S5M8767_RTC_I2C_ADDR, reg, 1, val, 1);
}

static int init_done;

static int rtc_init(void)
{
	int res;

	if (init_done)
		return 0;

	res = fdt_node_offset_by_compatible(gd->fdt_blob, 0,
					    "samsung,s5m8767-pmic");
	if (res < 0) {
		printf("%s: Failed to find s5m8767 in the fdt.\n", __func__);
		return -1;
	}

	if (s5m8767_rtc_write(S5M8767_RTC_CTRLM,
			      S5M8767_RTC_CTRLM_BCDM |
			      S5M8767_RTC_CTRLM_MODE24_12NM)) {
		printf("%s: Failed to set ctrlm.\n", __func__);
		return -1;
	}

	if (s5m8767_rtc_write(S5M8767_RTC_CTRL,
			      S5M8767_RTC_CTRL_MODE24_12N)) {
		printf("%s: Failed to set ctrl.\n", __func__);
		return -1;
	}

	init_done = 1;

	return 0;
}

int rtc_get(struct rtc_time *tm)
{
	int old_bus = i2c_get_bus_num();
	uint8_t data[DATA_SIZE];

	i2c_set_bus_num(0);
	if (rtc_init()) {
		printf("Failed to initialize the RTC.\n");
		i2c_set_bus_num(old_bus);
		return -1;
	}

	if (i2c_read(S5M8767_RTC_I2C_ADDR, S5M8767_RTC_SEC_TK, 1,
		     data, DATA_SIZE)) {
		printf("Failed to read from the RTC.\n");
		i2c_set_bus_num(old_bus);
		return -1;
	}

	/*
	 * If the seconds are zero, there may have been an update while
	 * reading the registers which would give us a bad result. Reading
	 * them again ensures we get a good result.
	 */
	if (!data[DATA_SECOND]) {
		if (i2c_read(S5M8767_RTC_I2C_ADDR, S5M8767_RTC_SEC_TK, 1,
			     data, DATA_SIZE)) {
			printf("Failed to read from the RTC.\n");
			i2c_set_bus_num(old_bus);
			return -1;
		}
	}

	tm->tm_sec = data[DATA_SECOND];
	tm->tm_min = data[DATA_MINUTE];
	tm->tm_hour = data[DATA_HOUR] & 0x3f;
	tm->tm_wday = __builtin_ctz(data[DATA_WEEKDAY]);
	tm->tm_mday = data[DATA_DATE];
	tm->tm_mon = data[DATA_MONTH];
	tm->tm_year = data[DATA_YEAR];

	if (tm->tm_year < 70)
		tm->tm_year += 2000;
	else
		tm->tm_year += 1900;

	i2c_set_bus_num(old_bus);
	return 0;
}

int rtc_set(struct rtc_time *tm)
{
	int old_bus = i2c_get_bus_num();
	uint8_t data[DATA_SIZE];
	uint8_t reg;

	i2c_set_bus_num(0);
	if (rtc_init()) {
		printf("Failed to initialize the RTC.\n");
		i2c_set_bus_num(old_bus);
		return -1;
	}

	memset(data, 0, sizeof(data));
	data[DATA_SECOND] = tm->tm_sec;
	data[DATA_MINUTE] = tm->tm_min;
	data[DATA_HOUR] = tm->tm_hour;
	data[DATA_WEEKDAY] = 0x1 << tm->tm_wday;
	data[DATA_DATE] = tm->tm_mday;
	data[DATA_MONTH] = tm->tm_mon;
	data[DATA_YEAR] = tm->tm_year % 100;

	if (tm->tm_hour > 12)
		data[DATA_HOUR] |= S5M8767_RTC_HR_TK_AM_PM;

	if (i2c_write(S5M8767_RTC_I2C_ADDR, S5M8767_RTC_SEC_TK, 1,
		      data, DATA_SIZE)) {
		printf("Failed to set data registers.\n");
		i2c_set_bus_num(old_bus);
		return -1;
	}

	if (s5m8767_rtc_write(S5M8767_RTC_TEST_CON,
			      S5M8767_RTC_TEST_CON_TIME_EN |
			      S5M8767_RTC_TEST_CON_UDR)) {
		printf("%s: Failed to set test_con.\n", __func__);
		i2c_set_bus_num(old_bus);
		return -1;
	}

	do {
		if (s5m8767_rtc_read(S5M8767_RTC_TEST_CON, &reg)) {
			printf("%s: Failed to read test_con.\n", __func__);
			i2c_set_bus_num(old_bus);
			return -1;
		}
	} while (reg & S5M8767_RTC_TEST_CON_UDR);

	if (s5m8767_rtc_write(S5M8767_RTC_TEST_CON, 0)) {
		printf("%s: Failed to set test_con.\n", __func__);
		i2c_set_bus_num(old_bus);
		return -1;
	}

	i2c_set_bus_num(old_bus);
	return 0;
}

void rtc_reset(void)
{
	struct rtc_time tm;

	init_done = 0;

	memset(&tm, 0, sizeof(tm));
	rtc_set(&tm);
}
