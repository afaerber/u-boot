/*
 * u-boot/arch/arm/include/asm/arch-mb8ac0300/ddr3c.h
 *
 * Copyright (C) 2010-2011 FUJITSU SEMICONDUCTOR LIMITED
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
#ifndef _DDR3C_H
#define _DDR3C_H

typedef volatile struct{
	volatile	 u32  CTL000;
	volatile	 u32  CTL001;
	volatile	 u32  CTL002;
	volatile	 u32  CTL003;
	volatile	 u32  CTL004;
	volatile	 u32  CTL005;
	volatile	 u32  CTL006;
	volatile	 u32  CTL007;
	volatile	 u32  CTL008;
	volatile	 u32  CTL009;
	volatile	 u32  CTL010;
	volatile	 u32  CTL011;
	volatile	 u32  CTL012;
	volatile	 u32  CTL013;
	volatile	 u32  CTL014;
	volatile	 u32  CTL015;
	volatile	 u32  CTL016;
	volatile	 u32  CTL017;
	volatile	 u32  CTL018;
	volatile	 u32  CTL019;
	volatile	 u32  CTL020;
	volatile	 u32  CTL021;
	volatile	 u32  CTL022;
	volatile	 u32  CTL023;
	volatile	 u32  CTL024;
	volatile	 u32  CTL025;
	volatile	 u32  CTL026;
	volatile	 u32  CTL027;
	volatile	 u32  CTL028;
	volatile	 u32  CTL029;
	volatile	 u32  CTL030;
	volatile	 u32  CTL031;
	volatile	 u32  CTL032;
	volatile	 u32  CTL033;
	volatile	 u32  CTL034;
	volatile	 u32  CTL035;
	volatile	 u32  CTL036;
	volatile	 u32  CTL037;
	volatile	 u32  CTL038;
	volatile	 u32  CTL039;
	volatile	 u32  CTL040;
	volatile	 u32  CTL041;
	volatile	 u32  CTL042;
	volatile	 u32  CTL043;
	volatile	 u32  CTL044;
	volatile	 u32  CTL045;
	volatile	 u32  CTL046;
	volatile	 u32  CTL047;
	volatile	 u32  CTL048;
	volatile	 u32  CTL049;
	volatile	 u32  CTL050;
	volatile	 u32  CTL051;
	volatile	 u32  CTL052;
	volatile	 u32  CTL053;
	volatile	 u32  CTL054;
	volatile	 u32  CTL055;
	volatile	 u32  CTL056;
	volatile	 u32  CTL057;
	volatile	 u32  CTL058;
	volatile	 u32  CTL059;
	volatile	 u32  CTL060;
	volatile	 u32  CTL061;
	volatile	 u32  CTL062;
	volatile	 u32  CTL063;
	volatile	 u32  CTL064;
	volatile	 u32  CTL065;
	volatile	 u32  CTL066;
	volatile	 u32  CTL067;
	volatile	 u32  CTL068;
	volatile	 u32  CTL069;
	volatile	 u32  CTL070;
	volatile	 u32  CTL071;
	volatile	 u32  CTL072;
	volatile	 u32  CTL073;
	volatile	 u32  CTL074;
	volatile	 u32  CTL075;
	volatile	 u32  CTL076;
	volatile	 u32  CTL077;
	volatile	 u32  CTL078;
	volatile	 u32  CTL079;
	volatile	 u32  CTL080;
	volatile	 u32  CTL081;
	volatile	 u32  CTL082;
	volatile	 u32  CTL083;
	volatile	 u32  CTL084;
	volatile	 u32  CTL085;
	volatile	 u32  CTL086;
	volatile	 u32  CTL087;
	volatile	 u32  CTL088;
	volatile	 u32  CTL089;
	volatile	 u32  CTL090;
	volatile	 u32  CTL091;
	volatile	 u32  CTL092;
	volatile	 u32  CTL093;
	volatile	 u32  CTL094;
	volatile	 u32  CTL095;
	volatile	 u32  CTL096;
	volatile	 u32  CTL097;
	volatile	 u32  CTL098;
	volatile	 u32  CTL099;
	volatile	 u32  CTL100;
	volatile	 u32  CTL101;
	volatile	 u32  CTL102;
	volatile	 u32  CTL103;
	volatile	 u32  CTL104;
	volatile	 u32  CTL105;
	volatile	 u32  CTL106;
	volatile	 u32  CTL107;
	volatile	 u32  CTL108;
	volatile	 u32  CTL109;
	volatile	 u32  CTL110;
	volatile	 u32  CTL111;
	volatile	 u32  CTL112;
	volatile	 u32  CTL113;
	volatile	 u32  CTL114;
} DATABAHN_type;

#define DATABAHN		((DATABAHN_type *) DDR3C_BASE)

/* CTL000 */
#define DRAM_CLASS		((u32)((u32)0x6 << 8))
#define START0			((u32)((u32)0x0 << 0))
#define START1			((u32)((u32)0x1 << 0))

/* CTL001 */
#define MAX_CS_REG		((u32)((u32)0x0 << 16))
#define MAX_COL_REG		((u32)((u32)0x0 << 8))
#define MAX_ROW_REG		((u32)((u32)0x0 << 0))

/* CTL002 */
#define TINIT			((u32)((u32)0xF << 0))

/* CTL003 */
#define TRST_PWRON		((u32)((u32)0x50 << 0))

/* CTL004 */
#define CKE_INACTIVE		((u32)((u32)0xC8 << 0))

/* CTL005 */
#define TBST_INT_INTERVAL	((u32)((u32)0x4 << 24))
#define WRLAT			((u32)((u32)0x5 << 16))
#define CASLAT_LIN		((u32)((u32)0xA << 8))
#define INITAREF		((u32)((u32)0x0 << 0))

/* CTL006 */
#define TRAS_MIN		((u32)((u32)0xF << 24))
#define TRC			((u32)((u32)0x15 << 16))
#define TRRD			((u32)((u32)0x4 << 8))
#define TCCD			((u32)((u32)0x4 << 0))

/* CTL007 */
#define TMRD			((u32)((u32)0x6 << 24))
#define TRTP			((u32)((u32)0x4 << 16))
#define TRP			((u32)((u32)0x6 << 8))
#define TWTR			((u32)((u32)0x4 << 0))

/* CTL008 */
#define TRAS_MAX		((u32)((u32)0x6DB0 << 8))
#define TMOD			((u32)((u32)0xC << 0))

/* CTL009 */
#define AP			((u32)((u32)0x0 << 24))
#define WRITEINTERP		((u32)((u32)0x0 << 16))
#define TCKESR			((u32)((u32)0x4 << 8))
#define TCKE			((u32)((u32)0x3 << 0))

/* CTL010 */
#define TWR_INT			((u32)((u32)0x6 << 24))
#define TRCD_INT		((u32)((u32)0x6 << 16))
#define TRAS_LOCKOUT		((u32)((u32)0x1 << 8))
#define CONCURRENTAP		((u32)((u32)0x1 << 0))

/* CTL011 */
#define NO_CMD_INIT		((u32)((u32)0x0 << 24))
#define TDLL			((u32)((u32)0x200 << 8))
#define TDAL			((u32)((u32)0xC << 0))

/* CTL012 */
#define TCPD			((u32)((u32)0x0 << 16))
#define TFAW			((u32)((u32)0x10 << 8))
#define BSTLEN			((u32)((u32)0x3 << 0))

/* CTL013 */
#define ADDRESS_MIRRORING	((u32)((u32)0x0 << 16))
#define REG_DIMM_ENABLE		((u32)((u32)0x0 << 8))
#define TRP_AB			((u32)((u32)0x7 << 0))

/* CTL014 */
#define RDIMM_CTL_0_31_0	((u32)((u32)0x24101065 << 0))

/* CTL015 */
#define RDIMM_CTL_0_63_32	((u32)((u32)0x00001070 << 0))

/* CTL016 */
#define RDIMM_CW_HOLD_CKE_EN	((u32)((u32)0x0 << 24))
#define RDIMM_CWW_REQ		((u32)((u32)0x0 << 16))
#define RDIMM_CWW_MAP		((u32)((u32)0xFFF << 0))

/* CTL017 */
#define RDIMM_TSTAB		((u32)((u32)0x960 << 8))
#define RDIMM_TMRD		((u32)((u32)0x8 << 0))

/* CTL018 */
#define TREF_ENABLE		((u32)((u32)0x1 << 24))
#define AUTO_REFRESH_MODE	((u32)((u32)0x0 << 16))
#define AREFRESH		((u32)((u32)0x0 << 8))
#define REG_DIMM_PARITY_ERROR	((u32)((u32)0x0 << 0))

/* CTL019 */
#define TREF			((u32)((u32)0xC2D << 16))
#define TRFC			((u32)((u32)0x78 << 0))

/* CTL020 */
#define POWER_DOWN		((u32)((u32)0x0 << 24))
#define TREF_INTERVAL		((u32)((u32)0x78 << 0))

/* CTL021 */
#define TXPDLL			((u32)((u32)0xA << 16))
#define TPDEX			((u32)((u32)0x3 << 0))

/* CTL022 */
#define TXSNR			((u32)((u32)0x7C << 16))
#define TXSR			((u32)((u32)0x200 << 0))

/* CTL023 */
#define ENABLE_QUICK_SREFRESH	((u32)((u32)0x1 << 24))
#define SREFRESH_EXIT_NO_REFRESH	((u32)((u32)0x0 << 16))
#define PWRUP_SREFRESH_EXIT	((u32)((u32)0x0 << 8))
#define SREFRESH		((u32)((u32)0x0 << 0))

/* CTL024 */
#define PWRDN_SHIFT_DELAY	((u32)((u32)0x0 << 8 ))
#define CKE_DELAY		((u32)((u32)0x0 << 0))

/* CTL025 */
#define WRITE_MODEREG		((u32)((u32)0x0 << 0))

/* CTL026 */
#define MR0_DATA_0		((u32)((u32)0x410 << 8))
#define MRW_STATUS		((u32)((u32)0x0 << 0))

/* CTL027 */
#define MR2_DATA_0		((u32)((u32)0x0 << 16))
#define MR1_DATA_0		((u32)((u32)0x9042 << 0))

/* CTL028 */
#define MR3_DATA_0		((u32)((u32)0x0 << 16))
#define MRSINGLE_DATA_0		((u32)((u32)0x0 << 0))

/* CTL029 */
#define MR1_DATA_1		((u32)((u32)0x9042 << 16 ))
#define MR0_DATA_1		((u32)((u32)0x410 << 0 ))

/* CTL030 */
#define MRSINGLE_DATA_1		((u32)((u32)0x0 << 16))
#define MR2_DATA_1		((u32)((u32)0x0 << 0))

/* CTL031 */
#define ZQINIT			((u32)((u32)0x200 << 16))
#define MR3_DATA_1		((u32)((u32)0x0 << 0))

/* CTL032 */
#define ZQCS			((u32)((u32)0x40 << 16 ))
#define ZQCL			((u32)((u32)0x100 << 0))

/* CTL033 */
#define ZQ_IN_PROGRESS		((u32)((u32)0x0 << 24 ))
#define REFRESH_PER_ZQ		((u32)((u32)0x40 << 16))
#define ZQ_ON_SREF_EXIT		((u32)((u32)0x2 << 8))
#define ZQ_REQ			((u32)((u32)0x0 << 0))

/* CTL034 */
#define COLUMN_SIZE		((u32)((u32)0x2 << 24 ))
#define ADDR_PINS		((u32)((u32)0x1 << 16 ))
#define EIGHT_BANK_MODE		((u32)((u32)0x1 << 8 ))
#define ZQCS_ROTATE		((u32)((u32)0x1 << 0 ))

/* CTL035 */
#define ADDR_CMP_EN		((u32)((u32)0x1 << 24 ))
#define COMMAND_AGE_COUNT	((u32)((u32)0xFF << 16 ))
#define AGE_COUNT		((u32)((u32)0xFF << 8 ))
#define APREBIT			((u32)((u32)0xA << 0 ))

/* CTL036 */
#define RW_SAME_EN		((u32)((u32)0x1 << 24 ))
#define PRIORITY_EN		((u32)((u32)0x1 << 16 ))
#define PLACEMENT_EN		((u32)((u32)0x1 << 8 ))
#define BANK_SPLIT_EN		((u32)((u32)0x1 << 0 ))

/* CTL037 */
#define CS_MAP			((u32)((u32)0x3 << 24 ))
#define INHIBIT_DRAM_CMD	((u32)((u32)0x0 << 16 ))
#define SWAP_PORT_RW_SAME_EN	((u32)((u32)0x1 << 8 ))
#define SWAP_EN			((u32)((u32)0x1 << 0 ))

/* CTL038 */
#define Q_FULLENESS		((u32)((u32)0x0 << 24 ))
#define FAST_WRITE		((u32)((u32)0x0 << 16 ))
#define REDUCT			((u32)((u32)0x0 << 8 ))
#define BURST_ON_FLY_BIT	((u32)((u32)0xC << 0 ))

/* CTL039 */
#define RESYNC_DLL_PER_AREF_EN	((u32)((u32)0x1 << 16 ))
#define RESYNC_DLL		((u32)((u32)0x0 << 8 ))
#define CONTROLLER_BUSY		((u32)((u32)0x0 << 0 ))

/* CTL040 */
#define INT_STATUS		((u32)((u32)0x0 << 0 ))

/* CTL041 */
#define INT_ACK			((u32)((u32)0x0 << 0 ))

/* CTL042 */
#define INT_MASK		((u32)((u32)0x20 << 0 ))

/* CTL043 */
#define OUT_OF_RANGE_ADDR	((u32)((u32)0x0 << 0 ))

/* CTL044 */
#define OUT_OF_RANGE_TYPE	((u32)((u32)0x0 << 16 ))
#define OUT_OF_RANGE_LENGTH	((u32)((u32)0x0 << 8 ))
#define OUT_OF_RANGE_ADDR_33_32	((u32)((u32)0x0 << 0 ))

/* CTL045 */
#define OUT_OF_RANGE_SOURCE_ID	((u32)((u32)0x0 << 0 ))

/* CTL046 */
#define PORT_CMD_ERROR_ADDR	((u32)((u32)0x0 << 0 ))

/* CTL047 */
#define PORT_CMD_ERROR_TYPE	((u32)((u32)0x0 << 24 ))
#define PORT_CMD_ERROR_ID	((u32)((u32)0x0 << 8 ))
#define PORT_CMD_ERROR_ADDR_33_32	((u32)((u32)0x0 << 0 ))

/* CTL048 */
#define ODT_WR_MAP_CS0		((u32)((u32)0x1 << 24 ))
#define ODT_RD_MAP_CS0		((u32)((u32)0x2 << 16 ))
#define PORT_DATA_ERROR_ID	((u32)((u32)0x0 << 0 ))

/* CTL049 */
#define ADD_ODT_CLK_W2R_SAMECS	((u32)((u32)0x0 << 24 ))
#define ADD_ODT_CLK_R2W_SAMECS	((u32)((u32)0x1 << 16 ))
#define OUT_WR_MAP_CS1		((u32)((u32)0x2 << 8 ))
#define OUT_RD_MAP_CS1		((u32)((u32)0x1 << 0 ))

/* CTL050 */
#define R2W_DIFFCS_DLY		((u32)((u32)0x6 << 24 ))
#define R2R_DIFFCS_DLY		((u32)((u32)0x1 << 16 ))
#define ADD_ODT_CLK_SAMETYPE_DIFFCS	((u32)((u32)0x2 << 8 ))
#define ADD_ODT_CLK_DIFFTYPE_DIFFCS	((u32)((u32)0x2 << 0 ))

/* CTL051 */
#define R2W_SAMECS_DLY		((u32)((u32)0x6 << 24 ))
#define R2R_SAMECS_DLY		((u32)((u32)0x0 << 16 ))
#define W2W_DIFFCS_DLY		((u32)((u32)0x1 << 8 ))
#define W2R_DIFFCS_DLY		((u32)((u32)0x6 << 0 ))

/* CTL052 */
#define OCD_ADJUST_PUP_CS_0	((u32)((u32)0x0 << 24 ))
#define OCD_ADJUST_PDN_CS_0	((u32)((u32)0x0 << 16 ))
#define W2W_SAMECS_DLY		((u32)((u32)0x0 << 8 ))
#define W2R_SAMECS_DLY		((u32)((u32)0x0 << 0 ))

/* CTL053 */
#define SWLVL_EXIT		((u32)((u32)0x0 << 24 ))
#define SWLVL_START		((u32)((u32)0x0 << 16 ))
#define SWLVL_LOAD		((u32)((u32)0x0 << 8 ))
#define SWLVL_LEVELING_MODE	((u32)((u32)0x0 << 0 ))

/* CTL054 */
#define SWLVL_RESP_1		((u32)((u32)0x0 << 24 ))
#define SWLVL_RESP_0		((u32)((u32)0x0 << 16 ))
#define LVL_STATUS		((u32)((u32)0x0 << 8 ))
#define SWLVL_OP_DONE		((u32)((u32)0x0 << 0 ))

/* CTL055 */
#define WRLVL_CS		((u32)((u32)0x0 << 24 ))
#define WRLVL_REQ		((u32)((u32)0x0 << 16 ))
#define SWLVL_RESP_3		((u32)((u32)0x0 << 8 ))
#define SWLVL_RESP_2		((u32)((u32)0x0 << 0 ))

/* CTL056 */
#define WRLVL_EN		((u32)((u32)0x0 << 16 ))
#define WLMRD			((u32)((u32)0x28 << 8 ))
#define WLDQSEN			((u32)((u32)0x19 << 0 ))

/* CTL057 */
#define WRLVL_ERROR_STATUS	((u32)((u32)0x0 << 16 ))
#define WRLVL_REFRESH_INTERVAL	((u32)((u32)0x0 << 0 ))

/* CTL058 */
#define WRLVL_DELAY_0		((u32)((u32)0x1E << 8 ))
#define WRLVL_REG_EN		((u32)((u32)0x0 << 0 ))

/* CTL059 */
#define WRLVL_DELAY_2		((u32)((u32)0x1A << 16 ))
#define WRLVL_DELAY_1		((u32)((u32)0x1C << 0 ))

/* CTL060 */
#define RDLVL_GATE_REQ		((u32)((u32)0x0 << 24 ))
#define RDLVL_REQ		((u32)((u32)0x0 << 16 ))
#define WRLVL_DELAY_3		((u32)((u32)0x17 << 0 ))

/* CTL061 */
#define RDLVL_REG_EN		((u32)((u32)0x0 << 24 ))
#define RDLVL_BEGIN_DELAY_EN	((u32)((u32)0x0 << 16 ))
#define RDLVL_EDGE		((u32)((u32)0x0 << 8 ))
#define RDLVL_CS		((u32)((u32)0x0 << 0 ))

/* CTL062 */
#define RDLVL_BEGIN_DELAY_0	((u32)((u32)0x0 << 8 ))
#define RDLVL_GATE_REG_EN	((u32)((u32)0x0 << 0 ))

/* CTL063 */
#define RDLVL_MIDPOINT_DELAY_0	((u32)((u32)0x0 << 16 ))
#define RDLVL_END_DELAY_0	((u32)((u32)0x0 << 0 ))

/* CTL064 */
#define RDLVL_OFFSET_DIR_0	((u32)((u32)0x0 << 16 ))
#define RDLVL_OFFSET_DELAY_0	((u32)((u32)0x0 << 0 ))

/* CTL065 */
#define RDLVL_GATE_DELAY_0	((u32)((u32)0x32 << 16 ))
#define RDLVL_DELAY_0		((u32)((u32)0x2323 << 0 ))

/* CTL066 */
#define RDLVL_END_DELAY_1	((u32)((u32)0x0 << 16 ))
#define RDLVL_BEGIN_DLEAY_1	((u32)((u32)0x0 << 0 ))

/* CTL067 */
#define RDLVL_OFFSET_DELAY_1	((u32)((u32)0x0 << 16 ))
#define RDLVL_MIDPOINT_DELAY_1	((u32)((u32)0x0 << 0 ))

/* CTL068 */
#define RDLVL_DELAY_1		((u32)((u32)0x2323 << 8 ))
#define RDLVL_OFFSET_DIR_1	((u32)((u32)0x0 << 0 ))

/* CTL069 */
#define RDLVL_BEGIN_DELAY_2	((u32)((u32)0x0 << 16 ))
#define RDLVL_GATE_DELAY_1	((u32)((u32)0x30 << 0 ))

/* CTL070 */
#define RDLVL_MIDPOINT_DELAY_2	((u32)((u32)0x0 << 16 ))
#define RDLVL_END_DELAY_2	((u32)((u32)0x0 << 0 ))

/* CTL071 */
#define RDLVL_OFFSET_DIR_2	((u32)((u32)0x0 << 16 ))
#define RDLVL_OFFSET_DELAY_2	((u32)((u32)0x0 << 0 ))

/* CTL072 */
#define RDLVL_GATE_DELAY_2	((u32)((u32)0x2D << 16 ))
#define RDLVL_DELAY_2		((u32)((u32)0x2323 << 0 ))

/* CTL073 */
#define RDLVL_END_DELAY_3	((u32)((u32)0x0 << 16 ))
#define RDLVL_BEGIN_DELAY_3	((u32)((u32)0x0 << 0 ))

/* CTL074 */
#define RDLVL_OFFSET_DELAY_3	((u32)((u32)0x0 << 16 ))
#define RDLVL_MIDPOINT_DELAY_3	((u32)((u32)0x0 << 0 ))

/* CTL075 */
#define RDLVL_DELAY_3		((u32)((u32)0x2323 << 8 ))
#define RDLVL_OFFSET_DIR_3	((u32)((u32)0x0 << 0 ))

/* CTL076 */
#define AXI0_EN_SIZE_LT_WIDTH_INSTR	((u32)((u32)0xFFFF << 16 ))
#define RDLVL_GATE_DELAY_3	((u32)((u32)0x2B << 0 ))

/* CTL077 */
#define AXI0_FIFO_TYPE_REG	((u32)((u32)0x0 << 16 ))

/* CTL078 */
#define AXI1_EN_SIZE_LT_WIDTH_INSTR	((u32)((u32)0xFFFF << 0 ))

/* CTL079 */
#define AXI2_EN_SIZE_LT_WIDTH_INSTR	((u32)((u32)0xFFFF << 8 ))
#define AXI1_FIFO_TYPE_REG	((u32)((u32)0x0 << 0 ))

/* CTL080 */
#define AXI3_EN_SIZE_LT_WIDTH_INSTR	((u32)((u32)0xFFFF << 16 ))
#define AXI2_FIFO_TYPE_REG	((u32)((u32)0x0 << 8 ))

/* CTL081 */
#define AXI3_FIFO_TYPE_REG	((u32)((u32)0x0 << 16 ))

/* CTL082 */
#define AXI0_PRIORITY0_RELATIVE_PRIORITY	((u32)((u32)0x4 << 24 ))
#define WRR_PARAM_VALUE_ERR	((u32)((u32)0x0 << 16 ))
#define WEIGHTED_ROUND_ROBIN_WEIGHT_SHARING	((u32)((u32)0x0 << 8 ))
#define WEIGHTED_ROUND_ROBIN_LATENCY_CONTROL	((u32)((u32)0x0 << 0 ))

/* CTL083 */
#define AXI0_PORT_ORDERING	((u32)((u32)0x0 << 24 ))
#define AXI0_PRIORITY3_RELATIVE_PRIORITY	((u32)((u32)0x4 << 16  ))
#define AXI0_PRIORITY2_RELATIVE_PRIORITY	((u32)((u32)0x4 << 8 ))
#define AXI0_PRIORITY1_RELATIVE_PRIORITY	((u32)((u32)0x4 << 0 ))

/* CTL084 */
#define AXI1_PRIORITY1_RELATIVE_PRIORITY	((u32)((u32)0x3 << 24 ))
#define AXI1_PRIORITY0_RELATIVE_PRIORITY	((u32)((u32)0x3 << 16 ))
#define AXI0_PRIORITY_RELAX	((u32)((u32)0x64 << 0 ))

/* CTL085 */
#define AXI1_PORT_ORDERING	((u32)((u32)0x1 << 16 ))
#define AXI1_RPIORITY3_RELATIVE_PRIORITY	((u32)((u32)0x3 << 8 ))
#define AXI1_RPIORITY2_RELATIVE_PRIORITY	((u32)((u32)0x3 << 0 ))

/* CTL086 */
#define AXI2_PRIORITY1_RELATIVE_PRIORITY	((u32)((u32)0x2 << 24 ))
#define AXI2_PRIORITY0_RELATIVE_PRIORITY	((u32)((u32)0x2 << 16 ))
#define AXI1_PRIORITY_RELAX	((u32)((u32)0x64 << 0 ))

/* CTL087 */
#define AXI2_PORT_ORDERING	((u32)((u32)0x2 << 16 ))
#define AXI2_PRIORITY3_RELATIVE_PRIORITY	((u32)((u32)0x2 << 8 ))
#define AXI2_PRIORITY2_RELATIVE_PRIORITY	((u32)((u32)0x2 << 0 ))

/* CTL088 */
#define AXI3_PRIORITY1_RELATIVE_PRIORITY	((u32)((u32)0x1 << 24 ))
#define AXI3_PRIORITY0_RELATIVE_PRIORITY	((u32)((u32)0x1 << 16 ))
#define AXI2_PRIORITY_RELAX	((u32)((u32)0x64 << 0 ))

/* CTL089 */
#define AXI3_PORT_ORDERING	((u32)((u32)0x3 << 16 ))
#define AXI3_PRIORITY3_RELATIVE_PRIORITY	((u32)((u32)0x1 << 8 ))
#define AXI3_PRIORITY2_RELATIVE_PRIORITY	((u32)((u32)0x1 << 0 ))

/* CTL090 */
#define MEM_RST_VALID		((u32)((u32)0x0 << 24 ))
#define CKE_STATUS		((u32)((u32)0x0 << 16 ))
#define AXI3_PRIORITY_RELAX	((u32)((u32)0x64 << 0 ))

/* CTL091 */
#define TDFI_PHY_WRLAT		((u32)((u32)0x0 << 24 ))
#define DLL_RST_ADJ_DLY		((u32)((u32)0x0 << 16 ))
#define DLL_RST_DELAY		((u32)((u32)0x0 << 0 ))

/* CTL092 */
#define DRAM_CLK_DISABLE	((u32)((u32)0x0 << 24 ))
#define TDFI_RDDATA_EN		((u32)((u32)0x0 << 16 ))
#define TDFI_PHY_RDLAT		((u32)((u32)0x7 << 8 ))
#define UPDATE_ERROR_STATUS	((u32)((u32)0x0 << 0 ))

/* CTL093 */
#define TDFI_CTRLUPD_MAX	((u32)((u32)0x1E << 8 ))
#define TDFI_CTRLUPD_MIN	((u32)((u32)0x1 << 0 ))

/* CTL094 */
#define TDFI_PHYUPD_TYPE1	((u32)((u32)0x200 << 16 ))
#define TDFI_PHYUPD_TYPE0	((u32)((u32)0x200 << 0 ))

/* CTL095 */
#define TDFI_PHYUPD_TYPE3	((u32)((u32)0x200 << 16 ))
#define TDFI_PHYUPD_TYPE2	((u32)((u32)0x200 << 0 ))

/* CTL096 */
#define TDFI_PHYUPD_RESP	((u32)((u32)0xC2D << 0 ))

/* CTL097 */
#define TDFI_CTRLUPD_INTERVAL	((u32)((u32)0x0 << 0 ))

/* CTL098 */
#define DFI_WRLVL_MAX_DELAY	((u32)((u32)0x8000 << 16 ))
#define WRLAT_ADJ		((u32)((u32)0x5 << 8 ))
#define RDLAT_ADJ		((u32)((u32)0x3 << 0 ))

/* CTL099 */
#define TDFI_WRLVL_RESPLAT	((u32)((u32)0x0 << 24 ))
#define TDFI_WRLVL_LOAD		((u32)((u32)0x0 << 16 ))
#define TDFI_WRLVL_DLL		((u32)((u32)0x0 << 8 ))
#define TDFI_WRLVL_EN		((u32)((u32)0x0 << 0 ))

/* CTL100 */
#define TDFI_WRLVL_WW		((u32)((u32)0x0 << 0 ))

/* CTL101 */
#define TDFI_WRLVL_RESP		((u32)((u32)0x0 << 0 ))

/* CTL102 */
#define TDFI_WRLVL_MAX		((u32)((u32)0x0 << 0 ))

/* CTL103 */
#define RDLVL_GATE_MAX_DELAY	((u32)((u32)0x10 << 16 ))
#define RDLVL_MAX_DELAY		((u32)((u32)0xFFFF << 0 ))

/* CTL104 */
#define TDFI_RDLVL_RESPLAT	((u32)((u32)0x0 << 24 ))
#define TDFI_RDLVL_LOAD		((u32)((u32)0x0 << 16 ))
#define TDFI_RDLVL_DLL		((u32)((u32)0x0 << 8 ))
#define TDFI_RDLVL_EN		((u32)((u32)0x0 << 0 ))

/* CTL105 */
#define TDFI_RDLVL_RR		((u32)((u32)0x0 << 0 ))

/* CTL106 */
#define TDFI_RDLVL_RESP		((u32)((u32)0x0 << 0 ))

/* CTL107 */
#define RDLVL_RESP_MASK		((u32)((u32)0x0 << 0 ))

/* CTL108 */
#define RDLVL_GATE_RESP_MASK	((u32)((u32)0x0 << 0 ))

/* CTL109 */
#define RDLVL_GATE_PREAMBLE_CHECK_EN	((u32)((u32)0x0 << 16 ))
#define RDLVL_GATE_EN		((u32)((u32)0x0 << 8 ))
#define RDLVL_EN		((u32)((u32)0x0 << 0 ))

/* CTL110 */
#define TDFI_RDLVL_MAX		((u32)((u32)0x0 << 0 ))

/* CTL111 */
#define RDLVL_GATE_DQ_ZERO_COUNT	((u32)((u32)0x2 << 8 ))
#define RDLVL_DQ_ZERO_COUNT	((u32)((u32)0x4 << 0 ))

/* CTL112 */
#define RDLVL_ERROR_STATUS	((u32)((u32)0x0 << 0 ))

/* CTL113 */
#define RDLVL_GATE_REFRESH_INTERVAL	((u32)((u32)0x0 << 16 ))
#define RDLVL_REFRESH_INTERVAL	((u32)((u32)0x0 << 0 ))

/* CTL114 */
#define ODT_ALT_EN		((u32)((u32)0x1 << 0 ))


#define OBSOLETE		((u32)((u32)0x0 << 0 ))

#endif	/* _DDR3C_H */
