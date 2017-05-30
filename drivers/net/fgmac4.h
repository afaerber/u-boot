/*
 * u-boot/drivers/net/fgmac4.h
 *
 * Copyright (C) 2010-2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This	program	is free	software: you can redistribute it and/or modify
 * it under the	terms of the GNU General Public	License	as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This	program	is distributed in the hope that	it will	be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If	not, see <http://www.gnu.org/licenses/>.
 */
/*
 * FGMAC4 Ethernet
 */

#ifndef	__FGMAC4_H
#define	__FGMAC4_H
#ifdef CONFIG_DRIVER_FGMAC4

/*
 * F_GMAC4 Register Address Map
 */
#define	FGMAC4_REG_MCR		0x0000	/* MAC Configuration Register        */
#define	FGMAC4_REG_MFFR		0x0004	/* MAC Frame Filter Register         */
#define	FGMAC4_REG_MHTRH	0x0008	/* MAC Hash Table Register(High)     */
#define	FGMAC4_REG_MHTRL	0x000C	/* MAC Hash Table Register(Low)      */
#define	FGMAC4_REG_GAR		0x0010	/* GMII	Address	Register             */
#define	FGMAC4_REG_GDR		0x0014	/* GMII	Data Register                */
#define	FGMAC4_REG_FCR		0x0018	/* Flow	Control	Register             */
#define	FGMAC4_REG_VTR		0x001C	/* VLAN	Tag Register                 */

#define	FGMAC4_REG_RWFFR	0x0028	/*Remote Wake-up Frame FilterRegister*/
#define	FGMAC4_REG_PMTR		0x002C	/* PMT Register                      */
#define FGMAC4_REG_LPICSR	0x0030	/* LPI Control and Status Register   */
#define FGMAC4_REG_LPITCR	0x0034	/* LPI Timers Control Register       */
#define FGMAC4_REG_ISR		0x0038	/* Interrupt Status Register         */
#define	FGMAC4_REG_IMR		0x003C	/* Interrupt Mask Register           */

#define	FGMAC4_REG_MAR0H	0x0040	/* MAC Address0 Register(High)       */
#define	FGMAC4_REG_MAR0L	0x0044	/* MAC Address0 Register(Low)        */
#define	FGMAC4_REG_MAR1H	0x0048	/* MAC Address1 Register(High)       */
#define	FGMAC4_REG_MAR1L	0x004C	/* MAC Address1 Register(Low)        */
#define	FGMAC4_REG_MAR2H	0x0050	/* MAC Address2 Register(High)       */
#define	FGMAC4_REG_MAR2L	0x0054	/* MAC Address2 Register(Low)        */
#define	FGMAC4_REG_MAR3H	0x0058	/* MAC Address3 Register(High)       */
#define	FGMAC4_REG_MAR3L	0x005C	/* MAC Address3 Register(Low)        */
#define	FGMAC4_REG_MAR4H	0x0060	/* MAC Address4 Register(High)       */
#define	FGMAC4_REG_MAR4L	0x0064	/* MAC Address4 Register(Low)        */
#define	FGMAC4_REG_MAR5H	0x0068	/* MAC Address5 Register(High)       */
#define	FGMAC4_REG_MAR5L	0x006C	/* MAC Address5 Register(Low)        */
#define	FGMAC4_REG_MAR6H	0x0070	/* MAC Address6 Register(High)       */
#define	FGMAC4_REG_MAR6L	0x0074	/* MAC Address6 Register(Low)        */
#define	FGMAC4_REG_MAR7H	0x0078	/* MAC Address7 Register(High)       */
#define	FGMAC4_REG_MAR7L	0x007C	/* MAC Address7 Register(Low)        */
#define	FGMAC4_REG_MAR8H	0x0080	/* MAC Address8 Register(High)       */
#define	FGMAC4_REG_MAR8L	0x0084	/* MAC Address8 Register(Low)        */
#define	FGMAC4_REG_MAR9H	0x0088	/* MAC Address9 Register(High)       */
#define	FGMAC4_REG_MAR9L	0x008C	/* MAC Address9 Register(Low)        */
#define	FGMAC4_REG_MAR10H	0x0090	/* MAC Address10 Register(High)      */
#define	FGMAC4_REG_MAR10L	0x0094	/* MAC Address10 Register(Low)       */
#define	FGMAC4_REG_MAR11H	0x0098	/* MAC Address11 Register(High)      */
#define	FGMAC4_REG_MAR11L	0x009C	/* MAC Address11 Register(Low)       */
#define	FGMAC4_REG_MAR12H	0x00A0	/* MAC Address12 Register(High)      */
#define	FGMAC4_REG_MAR12L	0x00A4	/* MAC Address12 Register(Low)       */
#define	FGMAC4_REG_MAR13H	0x00A8	/* MAC Address13 Register(High)      */
#define	FGMAC4_REG_MAR13L	0x00AC	/* MAC Address13 Register(Low)       */
#define	FGMAC4_REG_MAR14H	0x00B0	/* MAC Address14 Register(High)      */
#define	FGMAC4_REG_MAR14L	0x00B4	/* MAC Address14 Register(Low)       */
#define	FGMAC4_REG_MAR15H	0x00B8	/* MAC Address15 Register(High)      */
#define	FGMAC4_REG_MAR15L	0x00BC	/* MAC Address15 Register(Low)       */

#define FGMAC4_REG_RSR		0x00D8	/* RGMII status Reigster             */

#define	FGMAC4_REG_TSCR		0x0700	/* Time	Stamp Control Register       */
#define	FGMAC4_REG_SSIR		0x0704	/* Sub-Second Increment Register     */
#define	FGMAC4_REG_STSR		0x0708	/* System Time - Seconds Register    */
#define	FGMAC4_REG_STNR		0x070C	/* System Time - Nanoseconds Register */
#define	FGMAC4_REG_STSUR	0x0710	/* System Time-Seconds Update Reg     */
#define	FGMAC4_REG_STNUR	0x0714	/* System Time-Nanoseconds Update Reg */
#define	FGMAC4_REG_TSAR		0x0718	/* Time Stamp Addend Register        */
#define	FGMAC4_REG_TTSR		0x071C	/* Target Time Seconds Register      */
#define	FGMAC4_REG_TTNR		0x0720	/* Target Time Nanoseconds Register  */
#define	FGMAC4_REG_STHWSR	0x0724	/* System Time-High Word Seconds Reg */
#define	FGMAC4_REG_TSR		0x0728	/* Time Stamp Status Register        */
#define	FGMAC4_REG_PPSCR	0x072C	/* PPC Control Register              */
#define	FGMAC4_REG_ATNR		0x0730	/* Auxiliary Time Stamp-Nanosecond Reg*/
#define	FGMAC4_REG_ATSR		0x0734	/* Auxiliary Time Stamp-Seconds Reg  */

#define	FGMAC4_REG_MAR16H	0x0800	/* MAC Address16 Register(High)      */
#define	FGMAC4_REG_MAR16L	0x0804	/* MAC Address16 Register(Low)       */
#define	FGMAC4_REG_MAR17H	0x0808	/* MAC Address17 Register(High)      */
#define	FGMAC4_REG_MAR17L	0x080C	/* MAC Address17 Register(Low)       */
#define	FGMAC4_REG_MAR18H	0x0810	/* MAC Address18 Register(High)      */
#define	FGMAC4_REG_MAR18L	0x0814	/* MAC Address18 Register(Low)       */
#define	FGMAC4_REG_MAR19H	0x0818	/* MAC Address19 Register(High)      */
#define	FGMAC4_REG_MAR19L	0x081C	/* MAC Address19 Register(Low)       */
#define	FGMAC4_REG_MAR20H	0x0820	/* MAC Address20 Register(High)      */
#define	FGMAC4_REG_MAR20L	0x0824	/* MAC Address20 Register(Low)       */
#define	FGMAC4_REG_MAR21H	0x0828	/* MAC Address21 Register(High)      */
#define	FGMAC4_REG_MAR21L	0x082C	/* MAC Address21 Register(Low)       */
#define	FGMAC4_REG_MAR22H	0x0830	/* MAC Address22 Register(High)      */
#define	FGMAC4_REG_MAR22L	0x0834	/* MAC Address22 Register(Low)       */
#define	FGMAC4_REG_MAR23H	0x0838	/* MAC Address23 Register(High)      */
#define	FGMAC4_REG_MAR23L	0x083C	/* MAC Address23 Register(Low)       */
#define	FGMAC4_REG_MAR24H	0x0840	/* MAC Address24 Register(High)      */
#define	FGMAC4_REG_MAR24L	0x0844	/* MAC Address24 Register(Low)       */
#define	FGMAC4_REG_MAR25H	0x0848	/* MAC Address25 Register(High)      */
#define	FGMAC4_REG_MAR25L	0x084C	/* MAC Address25 Register(Low)       */
#define	FGMAC4_REG_MAR26H	0x0850	/* MAC Address26 Register(High)      */
#define	FGMAC4_REG_MAR26L	0x0854	/* MAC Address26 Register(Low)       */
#define	FGMAC4_REG_MAR27H	0x0858	/* MAC Address27 Register(High)      */
#define	FGMAC4_REG_MAR27L	0x085C	/* MAC Address27 Register(Low)       */
#define	FGMAC4_REG_MAR28H	0x0860	/* MAC Address28 Register(High)      */
#define	FGMAC4_REG_MAR28L	0x0864	/* MAC Address28 Register(Low)       */
#define	FGMAC4_REG_MAR29H	0x0868	/* MAC Address29 Register(High)      */
#define	FGMAC4_REG_MAR29L	0x086C	/* MAC Address29 Register(Low)       */
#define	FGMAC4_REG_MAR30H	0x0870	/* MAC Address30 Register(High)      */
#define	FGMAC4_REG_MAR30L	0x0874	/* MAC Address30 Register(Low)       */
#define	FGMAC4_REG_MAR31H	0x0878	/* MAC Address31 Register(High)      */
#define	FGMAC4_REG_MAR31L	0x087C	/* MAC Address31 Register(Low)       */

#define	FGMAC4_REG_BMR		0x1000	/* DMA BUS Mode Register             */
#define	FGMAC4_REG_TPDR		0x1004	/* DMA Transmit Poll Demand Register */
#define	FGMAC4_REG_RPDR		0x1008	/* DMA Receive Poll Demand Register  */
#define	FGMAC4_REG_RDLAR	0x100C	/* DMA Rx Desc List Address Register */
#define	FGMAC4_REG_TDLAR	0x1010	/* DMA Tx Desc List Address Register */
#define	FGMAC4_REG_SR		0x1014	/* DMA Status Register               */
#define	FGMAC4_REG_OMR		0x1018	/* DMA Operation Mode Register       */
#define	FGMAC4_REG_IER		0x101C	/* DMA Interrupt Enable Register     */
#define	FGMAC4_REG_MFOCR	0x1020	/* DMA Missed Frame Register         */
#define FGMAC4_REG_RIWTR	0x1024	/* DMA Rx Intr Watchdog Timer Reg    */

#define FGMAC4_REG_AHBSR	0x102C	/* DMA AHB Status Register           */

#define	FGMAC4_REG_CHTDR	0x1048	/* DMA Cur Host Tx Desc Register     */
#define	FGMAC4_REG_CHRDR	0x104C	/* DMA Cur Host Rx Desc Register     */
#define	FGMAC4_REG_CHTBAR	0x1050	/* DMA Cur Host TX Buffer Addr Reg   */
#define	FGMAC4_REG_CHRBAR	0x1054	/* DMA Cur Host RX Buffer Addr Reg   */

/* MMC(MAC Management Counters) Register Address Map */
#define FGMAC4_REG_MMC_CNTL			0x0100
#define FGMAC4_REG_MMC_INTR_RX			0x0104
#define FGMAC4_REG_MMC_INTR_TX			0x0108
#define FGMAC4_REG_MMC_INTR_MASK_RX		0x010C
#define FGMAC4_REG_MMC_INTR_MASK_TX		0x0110

#define FGMAC4_REG_MMC_TXOTCETCOUNT_GB		0x0114
#define FGMAC4_REG_MMC_TXFRAMECOUNT_GB		0x0118
#define FGMAC4_REG_MMC_TXBROADCASTFRAMES_G	0x011C
#define FGMAC4_REG_MMC_TXMULTICASTFRAMES_G	0x0120
#define FGMAC4_REG_MMC_TX64OCTECS_GB		0x0124
#define FGMAC4_REG_MMC_TX65TO127OCTETS_GB	0x0128
#define FGMAC4_REG_MMC_TX128TO255OCTETS_GB	0x012C
#define FGMAC4_REG_MMC_TX256TO511OCTETS_GB	0x0130
#define FGMAC4_REG_MMC_TX512TO1023OCTETS_GB	0x0134
#define FGMAC4_REG_MMC_TX1024TOMAXOCTETS_GB	0x0138
#define FGMAC4_REG_MMC_TXUNICASTFRAMES_GB	0x013C
#define FGMAC4_REG_MMC_TXMULTICASTFRAMES_GB	0x0140
#define FGMAC4_REG_MMC_TXBROADCASTFRAMES_GB	0x0144
#define FGMAC4_REG_MMC_TXUNDERFLOWERROR		0x0148
#define FGMAC4_REG_MMC_TXSINGLECOL_G		0x014C
#define FGMAC4_REG_MMC_TXMULTICOL_G		0x0150
#define FGMAC4_REG_MMC_TXDEFERRED		0x0154
#define FGMAC4_REG_MMC_TXLATECOL		0x0158
#define FGMAC4_REG_MMC_TXEXESSCOL		0x015C
#define FGMAC4_REG_MMC_TXCARRIERERROR		0x0160
#define FGMAC4_REG_MMC_TXOCTETCOUNT_G		0x0164
#define FGMAC4_REG_MMC_TXFRAMECOUNT_G		0x0168
#define FGMAC4_REG_MMC_TXEXECESSDEF		0x016C
#define FGMAC4_REG_MMC_TXPAUSEFRAMES		0x0170
#define FGMAC4_REG_MMC_TXVLANFRAMES_G		0x0174

#define FGMAC4_REG_MMC_RXFRAMECOUNT_GB		0x0180
#define FGMAC4_REG_MMC_RXOCTETCOUNT_GB		0x0184
#define FGMAC4_REG_MMC_RXOCTETCOUNT_G		0x0188
#define FGMAC4_REG_MMC_RXBROADCASTFRAMES_G	0x018C
#define FGMAC4_REG_MMC_RXMULTICASTFRAMES_G	0x0190
#define FGMAC4_REG_MMC_RXCRCERROR		0x0194
#define FGMAC4_REG_MMC_RXALIGNMENTERROR		0x0198
#define FGMAC4_REG_MMC_RXRUNTERROR		0x019C
#define FGMAC4_REG_MMC_RXJABBERERROR		0x01A0
#define FGMAC4_REG_MMC_RXUNDERSIZE_G		0x01A4
#define FGMAC4_REG_MMC_RXOVERSIZE_G		0x01A8
#define FGMAC4_REG_MMC_RX64OCTETS_GB		0x01AC
#define FGMAC4_REG_MMC_RX65TO127OCTETS_GB	0x01B0
#define FGMAC4_REG_MMC_RX128TO255OCTETS_GB	0x01B4
#define FGMAC4_REG_MMC_RX256TO511OCTETS_GB	0x01B8
#define FGMAC4_REG_MMC_RX512TO1023OCTETS_GB	0x01BC
#define FGMAC4_REG_MMC_RX1024TOMAXOCTETS_GB	0x01C0
#define FGMAC4_REG_MMC_RXUNICASTFRAMES_G	0x01C4
#define FGMAC4_REG_MMC_RXLENGTHERROR		0x01C8
#define FGMAC4_REG_MMC_RXOUTOFRANGETYPE		0x01CC
#define FGMAC4_REG_MMC_RXPAUSEFRAMES		0x01D0
#define FGMAC4_REG_MMC_RXFIFOOVERFLOW		0x01D4
#define FGMAC4_REG_MMC_RXVLANFRAMES_GB		0x01D8
#define FGMAC4_REG_MMC_RXWATCHDOGERROR		0x01DC

#define FGMAC4_REG_MMC_IPC_INTR_MASK_RX		0x0200

#define FGMAC4_REG_MMC_IPC_INTR_RX		0x0208

#define FGMAC4_REG_MMC_RXIPV4_GB_FRMS		0x0210
#define FGMAC4_REG_MMC_RXIPV4_HDRERR_FRMS	0x0214
#define FGMAC4_REG_MMC_RXIPV4_NOPAY_FRMS	0x0218
#define FGMAC4_REG_MMC_RXIPV4_FRAG_FRMS		0x021C
#define FGMAC4_REG_MMC_RXIPV4_UDSBL_FRMS	0x0220
#define FGMAC4_REG_MMC_RXIPV6_GB_FRMS		0x0224
#define FGMAC4_REG_MMC_RXIPV6_HDRERR_FRMS	0x0228
#define FGMAC4_REG_MMC_RXIPV6_NOPAY_FRMS	0x022C
#define FGMAC4_REG_MMC_RXUDP_GB_FRMS		0x0230
#define FGMAC4_REG_MMC_RXUDP_ERR_FRMS		0x0234
#define FGMAC4_REG_MMC_RXTCP_GB_FRMS		0x0238
#define FGMAC4_REG_MMC_RXTCP_ERR_FRMS		0x023C
#define FGMAC4_REG_MMC_RXICMP_GB_FRMS		0x0240
#define FGMAC4_REG_MMC_RXICMP_ERR_FRMS		0x0244

#define FGMAC4_REG_MMC_RXIPV4_GB_OCTETS		0x0250
#define FGMAC4_REG_MMC_RXIPV4_HDRERR_OCTETS	0x0254
#define FGMAC4_REG_MMC_RXIPV4_NOPAY_OCTETS	0x0258
#define FGMAC4_REG_MMC_RXIPV4_FRAG_OCTETS	0x025C
#define FGMAC4_REG_MMC_RXIPV4_UDSBL_OCTETS	0x0260
#define FGMAC4_REG_MMC_RXIPV6_GB_OCTETS		0x0264
#define FGMAC4_REG_MMC_RXIPV6_HDRERR_OCTETS	0x0268
#define FGMAC4_REG_MMC_RXIPV6_NOPAY_OCTETS	0x026C
#define FGMAC4_REG_MMC_RXUDP_GB_OCTETS		0x0270
#define FGMAC4_REG_MMC_RXUDP_ERR_OCTETS		0x0274
#define FGMAC4_REG_MMC_RXTCP_GB_OCTETS		0x0278
#define FGMAC4_REG_MMC_RXTCP_ERR_OCTETS		0x027C
#define FGMAC4_REG_MMC_RXICMP_GB_OCTETS		0x0280
#define FGMAC4_REG_MMC_RXICMP_ERR_OCTETS	0x0284

/*
 * Values and Masks
 */
#define	SET_0			0x00000000
#define	SET_1			0xFFFFFFFF

/* IMR : Interrupt Mask	Register */
#define	FGMAC4_IMR_LPIIM	0x00000400	/* LPI Interrupt Mask         */
#define	FGMAC4_IMR_TSIM		0x00000200	/* Time Stamp Interrupt Mask  */
#define	FGMAC4_IMR_PIM		0x00000008	/* PMT Interrupt Mask         */
#define	FGMAC4_IMR_RGIM		0x00000001	/* RGMII Interrupt Mask       */

/* MCR:MAC Configuration Register */
#define	FGMAC4_MCR_CST		0x02000000	/* CRC strip for Type frames  */
#define	FGMAC4_MCR_TC		0x01000000	/* Tx Configuration in RGMII  */
#define	FGMAC4_MCR_WD		0x00800000	/*Disable RX Watchdog timeout */
#define	FGMAC4_MCR_JD		0x00400000	/* Disable TX Jabber timer    */
#define	FGMAC4_MCR_BE		0x00200000	/* Frame Burst Enable         */
#define	FGMAC4_MCR_JE		0x00100000	/* Jumbo Frame Enable         */
#define	FGMAC4_MCR_DCRS		0x00010000	/*Disable Carrier During Trans*/
#define	FGMAC4_MCR_PS		0x00008000	/* Port Select 0:GMII,1:MII   */
#define	FGMAC4_MCR_FES		0x00004000	/* Speed                      */
#define	FGMAC4_MCR_DO		0x00002000	/* Disable Receive Own        */
#define	FGMAC4_MCR_LM		0x00001000	/* Loop-back Mode             */
#define	FGMAC4_MCR_DM		0x00000800	/* Duplex mode                */
#define	FGMAC4_MCR_IPC		0x00000400	/* Cehcksum Offload           */
#define	FGMAC4_MCR_DR		0x00000200	/* Disable Retry              */
#define	FGMAC4_MCR_LUD		0x00000100	/* Link Up/Down               */
#define	FGMAC4_MCR_BL_00	0x00000000	/* Back-off Limit is setted 0 */
#define	FGMAC4_MCR_DC		0x00000010	/* Deferral Check             */
#define	FGMAC4_MCR_TX_ENABLE	0x00000008	/* Enable Transmitter         */
#define	FGMAC4_MCR_RX_ENABLE	0x00000004	/* Enable Receiver            */
#define	FGMAC4_MCR_IFG_MASK	0x000E0000
#define	FGMAC4_MCR_IFG_96	0x00000000
#define	FGMAC4_MCR_IFG_88	0x00020000
#define	FGMAC4_MCR_IFG_80	0x00040000
#define	FGMAC4_MCR_IFG_72	0x00060000
#define	FGMAC4_MCR_IFG_64	0x00080000
#define	FGMAC4_MCR_IFG_56	0x000A0000
#define	FGMAC4_MCR_IFG_48	0x000C0000
#define	FGMAC4_MCR_IFG_40	0x000E0000
	/* bit15:PS(Port Select) */
#define	FGMAC4_MCR_GMII_PORT	0x00000000
#define	FGMAC4_MCR_MII_PORT	0x00008000
	/* bit11:DM(Duplex mode) */
#define	FGMAC4_MCR_HALF_DUPLEX	0x00000000
#define	FGMAC4_MCR_FULL_DUPLEX	0x00000800

/* GAR:GMII Address Register */
#define	FGMAC4_GAR_GW_R		0x00000000	/* GMII/MII Read              */
#define	FGMAC4_GAR_GW_W		0x00000002	/* GMII/MII Write             */
#define	FGMAC4_GAR_GB		0x00000001	/* GMII/MII Busy              */
#define FGMAC4_GAR_PA_SHIFT	11		/* PHY address bit field shift*/
#define FGMAC4_GAR_GR_MASK	0x0000001F	/* GMII register field mask   */
#define FGMAC4_GAR_GR_SHIFT	6		/* GMII register field shift  */
#define FGMAC4_GAR_CR_MASK	0x0000000F	/* Clock range field mask     */
#define FGMAC4_GAR_CR_SHIFT	2		/* Clock range field shift    */

/* BMR:MDC Bus Mode Register */
#define	FGMAC4_BMR_TXPR		0x08000000	/* Tx Pririty Higher Than Rx  */
#define	FGMAC4_BMR_MB		0x04000000	/* Mixed Burst                */
#define	FGMAC4_BMR_AAL		0x02000000	/* Address-Aligned Beats      */
#define	FGMAC4_BMR_8XPBL	0x01000000	/* MAX burst is 8times of PBL */
#define	FGMAC4_BMR_USP		0x00800000	/* Rx/TxDMA Use Separate PBL  */
#define	FGMAC4_BMR_RPBL_32	0x00400000	/* RX Burst Length is 32 Bytes*/
#define	FGMAC4_BMR_RPBL_16	0x00200000	/* RX Burst Length is 16 Bytes*/
#define	FGMAC4_BMR_RPBL_8	0x00100000	/* RX Burst Length is 8 Bytes */
#define	FGMAC4_BMR_RPBL_4	0x00080000	/* RX Burst Length is 4 Bytes */
#define	FGMAC4_BMR_RPBL_2	0x00040000	/* RX Burst Length is 2 Bytes */
#define	FGMAC4_BMR_RPBL_1	0x00020000	/* RX Burst Length is 1 Bytes */
#define	FGMAC4_BMR_FB		0x00010000	/* AHB Fixed Burst Mode       */
#define	FGMAC4_BMR_PR_00	0x00000000	/* RX TX Priority ratio is 1:1*/
#define	FGMAC4_BMR_PR_01	0x00004000	/* RX TX Priority ratio is 2:1*/
#define	FGMAC4_BMR_PR_10	0x00008000	/* RX TX Priority ratio is 3:1*/
#define	FGMAC4_BMR_PR_11	0x0000C000	/* RX TX Priority ratio is 4:1*/
#define	FGMAC4_BMR_PBL_32	0x00002000	/* Burst Length is 32 Bytes   */
#define	FGMAC4_BMR_PBL_16	0x00001000	/* Burst Length is 16 Bytes   */
#define	FGMAC4_BMR_PBL_8	0x00000800	/* Burst Length is 8 bytes    */
#define	FGMAC4_BMR_PBL_4	0x00000400	/* Burst Length is 4 Bytes    */
#define	FGMAC4_BMR_PBL_2	0x00000200	/* Burst Length is 2 Bytes    */
#define	FGMAC4_BMR_PBL_1	0x00000100	/* Burst Length is 1 Bytes    */
#define	FGMAC4_BMR_ATDS		0x00000080	/* Alternate Descriptor Size  */
#define	FGMAC4_BMR_DSL		0x00000000	/* Descripter Skip Length     */
#define	FGMAC4_BMR_DA		0x00000002	/* RX have priority           */
#define	FGMAC4_BMR_SOFTWARE_RESET    0x00000001 /* software reset             */

/* OMR:MDC Operation Mode Register */
#define FGMAC4_OMR_RSF		0x02000000  /* RX after whole frame in FIFO   */
#define FGMAC4_OMR_TSF		0x00200000  /* TX after whole frame in FIFO   */
#define	FGMAC4_OMR_START_TX	0x00002000  /* Start Transmissin	      */
#define	FGMAC4_OMR_START_RX	0x00000002  /* Start Receive		      */
#define	FGMAC4_OMR_TTC_64B	0x00000000  /* TX after	64byte written in FIFO*/
#define	FGMAC4_OMR_TTC_128B	0x00004000  /*TX after 128byte written in FIFO*/
#define	FGMAC4_OMR_TTC_192B	0x00008000  /*TX after 192byte written in FIFO*/
#define	FGMAC4_OMR_TTC_256B	0x0000C000  /*TX after 256byte written in FIFO*/
#define	FGMAC4_OMR_TTC_40B	0x00010000  /* TX after	40byte written in FIFO*/
#define	FGMAC4_OMR_TTC_32B	0x00014000  /* TX after	32byte written in FIFO*/
#define	FGMAC4_OMR_TTC_24B	0x00018000  /* TX after	24byte written in FIFO*/
#define	FGMAC4_OMR_TTC_16B	0x0001C000  /* TX after	16byte written in FIFO*/

/* IER:MDC Interrupt Enable Register */
#define	FGMAC4_IER_NIE	0x00010000	/* Normal Interrupt Summary Enable    */
#define	FGMAC4_IER_AIE	0x00008000	/* Abnormal Interrupt Summary Enable  */
#define	FGMAC4_IER_ERE	0x00004000	/* Early Receive Interrupt Enable     */
#define	FGMAC4_IER_FBE	0x00002000	/* Fatal Bus Error Enable             */
#define	FGMAC4_IER_ETE	0x00000400	/* Early Transmit Interrupt Enable    */
#define	FGMAC4_IER_RWE	0x00000200	/*Receive Watchdog Timeout Enable     */
#define	FGMAC4_IER_RSE	0x00000100	/* Receive Process Stopped Enable     */
#define	FGMAC4_IER_RUE	0x00000080	/* Receive Buffer Unavailable Enable  */
#define	FGMAC4_IER_RIE	0x00000040	/* Receive Interrupt Enable           */
#define	FGMAC4_IER_UNE	0x00000020	/* Transmit underflow Enable          */
#define	FGMAC4_IER_OVE	0x00000010	/* Receive Overflow Enable            */
#define	FGMAC4_IER_TJE	0x00000008	/* Transmit Jabber Timeout            */
#define	FGMAC4_IER_TUE	0x00000004	/* Transmit Buffer Unavailable        */
#define	FGMAC4_IER_TSE	0x00000002	/* Transmit Process Stopped           */
#define	FGMAC4_IER_TIE	0x00000001	/* Transmit Interrupt                 */

/* PMTR:PMT Register */
#define FGMAC4_PMTR_WFE	0x00000004	/* Wake-Up Frame Enable               */
#define FGMAC4_PMTR_MPE	0x00000002	/* Magic Packet Enable                */


/* Descriptor Status */
#define	OWN_BIT			0x80000000 /* Own bit                         */

#define	FGMAC4_RDES1_DIC	0x80000000 /* Disable Interrupt on Completion */
#define	FGMAC4_RDES1_RER	0x00008000 /* Receive End of Ring             */

#define	FGMAC4_TDES0_LS		0x20000000 /* Last Segment                    */
#define	FGMAC4_TDES0_FS		0x10000000 /* First Segment                   */
#define	FGMAC4_TDES0_TER	0x00200000 /* Transmit End of Ring            */

/* RX descriptor error status
 * ES[bit15]:Error Summary
 * DE[bit14]:Descriptor Error
 * SAF[bit13]:Source Address Filter Fail
 * LE[bit12]:Length Error
 * OE[bit11]:Overflow Error
 * RWT[bit4]:Receive watchdog timeout
 * RE[bit3]:Receive Error
 * DE[bit2]:Dribble Bit Error
 * CE[bit1]:CRC	Error
 */
#define	FGMAC4_RX_DESC_ERR	0x0000F81E


/* Tx/Rx descriptor definition */
struct fgmac4_desc {
	volatile u32 opts1;
	volatile u32 opts2;
	u32 addr1;
	u32 addr2;

	/* for rx descriptor, this is extended status descriptor;
	 * for tx descriptor, this is a	reserved descriptor.
	 */
	volatile u32 ex_status_rsv;
	u32 reserved;			/* reserved */
	volatile u32 time_stmp_low;	/* time	stamp low */
	volatile u32 time_stmp_high;	/* time	stamp high */
};

/* PHY information */
struct fgmac4_phy_info {
	u32 adv;
	u32 speed;
	u32 duplex;
	u32 autoneg;
	u16 phy_addr;
};

/* Private information	*/
struct eth_info	{
	int mdc_clk;			/* MDC clock			     */
	struct fgmac4_phy_info phy_info;/* PHY info			     */

	dma_addr_t ring_dma;		/* base	address	of descriptor memory */
	struct fgmac4_desc *rx_ring;	/* RX Desc ring's pointer	     */
	struct fgmac4_desc *tx_ring;	/* TX Desc ring's pointer	     */
	unsigned char* rx_buf_array;	/* RX buffer pointer */
	unsigned char* tx_buf_array;	/* TX buffer pointer */
	u32 rx_ring_num;		/* Current RX desc number	     */
	u32 tx_ring_num;		/* Current TX desc number	     */
	u32 rx_buf_sz;			/* MAX size of socket buffer	     */

	struct eth_device netdev;
};

#endif	/* CONFIG_DRIVER_FGMAC4	*/

#endif	/* __FGMAC4_H */
