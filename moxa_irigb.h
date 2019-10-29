#ifndef __IRIGB__
#define __IRIGB__

#define MX_IRIGB_MINOR	111
#define MX_IRIGB_NAME	"moxa_irigb"

#define IOCTL_GET_REGISTER		_IOR(MX_IRIGB_MINOR,0x800,unsigned long)
#define IOCTL_SET_REGISTER		_IOW(MX_IRIGB_MINOR,0x801,unsigned long)
#define IOCTL_SETCLR_REGISTER_BIT	_IOW(MX_IRIGB_MINOR,0x802,unsigned long)
#define IOCTL_GET_TIMESRC_STATUS	_IOW(MX_IRIGB_MINOR,0x803,unsigned long)

#define MAX_PAIRS 4

struct reg_val_pair_struct {
	u64 count;
	u64 addr[MAX_PAIRS];
	u64 val[MAX_PAIRS];
};

struct reg_bit_pair_struct {
	u64 addr;
	u64 set_bit;
	u64 clear_bit;
};

//-----------------------------------------------------------------------------
// PCI Device/Vendor Ids.
//-----------------------------------------------------------------------------

#ifndef PCI_VENDOR_ID_MOXA
#define	PCI_VENDOR_ID_MOXA	0x1393	/* Moxa vendor ID */
#endif

#ifndef MX_IRIGB_DEVICE_ID
#define MX_IRIGB_DEVICE_ID      0xA004
#endif

//-----------------------------------------------------------------------------
// Define the Registers Offset
//-----------------------------------------------------------------------------
enum _FPGA_REGS {
	DEVICEID,		// Device ID Register
	DATECODE,		// FPGA firmware date code
	SYSCON,			// System configuration
	LPBTCNT,		// Loop back test counter
	INPORTCON,		// Input port configuration
	OUTPORTCON,		// Output port configuration
	PORTDAT,		// Input / Output port data
	IRIGBDE0DAT0,		// IRIG-B Decoder 0 Data 0
	IRIGBDE0DAT1,		// IRIG-B Decoder 0 Data 1
	IRIGBDE0DAT2,		// IRIG-B Decoder 0 Data 2
	IRIGBDE0DAT3,		// IRIG-B Decoder 0 Data 3
	IRIGBDE0CNT,		// IRIG-B Decoder 0 Counter, count start from 2 'P' bit
	IRIGBDE1DAT0,		// IRIG-B Decoder 1 Data 0
	IRIGBDE1DAT1,		// IRIG-B Decoder 1 Data 1
	IRIGBDE1DAT2,		// IRIG-B Decoder 1 Data 2
	IRIGBDE1DAT3,		// IRIG-B Decoder 1 Data 3
	IRIGBDE1CNT,		// IRIG-B Decoder 1 Counter, count start from 2 'P' bit
	PPSCON,			// Pulse per second configuration
	TMCON,			// Time module configuration
	INTSTS,			// Interrupt status
	INTMSK,			// Interrupt mask
	PPSDETIMEOUT,		// Pulse per second timeout
	RTCCON,			// RTC Configuration
	RTCDAT0,		// RTC Data register 0
	RTCDAT1,		// RTC Data register 1
	RTCDAT2,		// RTC Data register 2
	RTCDAT3,		// RTC Data register 3
	RTCLS,			// RTC Leap second
	RTCDST,			// RTC DST
	SFIFOCON,		// Serial FIFO configuration
	SFIFODAT,		// Serial FIFO data
	NLEDCON,		// Notify LED configuration

	MAX_ITEMS
};

//-----------------------------------------------------------------------------
// Define the System Configuration Registers
//-----------------------------------------------------------------------------
#define SYSCON_BIT_RESET			(1<<0)
#define SYSCON_BIT_LBT				(1<<31)

//-----------------------------------------------------------------------------
// Define the Loop back test counter Registers
//-----------------------------------------------------------------------------
#define LPBTCNT_UNIT    			40 /* 40 nano second per unit */

//-----------------------------------------------------------------------------
// Define the input port configuration Registers
//-----------------------------------------------------------------------------
#define INPSEL_INP0     			0
#define INPSEL_INP1     			1
#define INPSEL_INP2     			2
#define INPSEL_INP3     			3
#define INPSEL_INP4     			4
#define INPSEL_INP5     			5
#define INPSEL_INP6     			6
#define INPSEL_INP7     			7

#define INPORTCON_MASK				0x7
#define INPORTCON_IRIGDE0_BIT_S			0
#define INPORTCON_IRIGDE1_BIT_S			4
#define INPORTCON_PPSDE_BIT_S			8
#define INPORTCON_LBT_BIT_S			12
#define INPORTCON_BIT_IRIGDE0_DIS		(1<<3)
#define INPORTCON_BIT_IRIGDE1_DIS		(1<<7)
#define INPORTCON_BIT_PPSDE_DIS			(1<<11)
#define INPORTCON_BIT_LBT_DIS			(1<<15)
#define INPORTCON_BIT_INV0			(1<16)
#define INPORTCON_BIT_INV1			(1<17)
#define INPORTCON_BIT_INV2			(1<18)
#define INPORTCON_BIT_INV3			(1<19)
#define INPORTCON_BIT_INV4			(1<20)
#define INPORTCON_BIT_INV5			(1<21)
#define INPORTCON_BIT_INV6			(1<22)
#define INPORTCON_BIT_INV7			(1<23)

//-----------------------------------------------------------------------------
// Define the output port configuration Registers
//-----------------------------------------------------------------------------
#define OUTPSEL_GPO         			0x0
#define OUTPSEL_IRIGBEN     			0x1
#define OUTPSEL_PPSEN       			0x3
#define OUTPSEL_HIZ         			0x5
#define OUTPSEL_SOUT        			0x6
#define OUTPSEL_LPTS        			0x7
#define OUTPSEL_INP0        			0x8
#define OUTPSEL_INP1        			0x9
#define OUTPSEL_INP2        			0xa
#define OUTPSEL_INP3        			0xb
#define OUTPSEL_INP4        			0xc
#define OUTPSEL_INP5        			0xd
#define OUTPSEL_INP6        			0xe
#define OUTPSEL_INP7        			0xf

#define OUTPORTCON_MASK				0xf
#define OUTPORTCON_P0_BIT_S			0
#define OUTPORTCON_P1_BIT_S			5
#define OUTPORTCON_P2_BIT_S			10
#define OUTPORTCON_P3_BIT_S			15
#define OUTPORTCON_P4_BIT_S			20
#define OUTPORTCON_P5_BIT_S			25

#define OUTPORTCON_BIT_INV0			(1<<4)
#define OUTPORTCON_BIT_INV1			(1<<9)
#define OUTPORTCON_BIT_INV2			(1<<14)
#define OUTPORTCON_BIT_INV3			(1<<19)
#define OUTPORTCON_BIT_INV4			(1<<24)
#define OUTPORTCON_BIT_INV5			(1<<29)
#define OUTPORTCON_BIT_INV6			(1<<30)
#define OUTPORTCON_BIT_INV7			(1<<31)

//-----------------------------------------------------------------------------
// Define the port data Registers
//-----------------------------------------------------------------------------
#define PORTDATA_MASK				(0xFFFF)
#define PORTDATA_INPUT_BIT_S			(0)
#define PORTDATA_OUTPUT_BIT_S			(16)

//-----------------------------------------------------------------------------
// Define the IRIG-B decode data Registers
//-----------------------------------------------------------------------------
#define IRIGBDEDAT0_SEC_MASK			(0xf)
#define IRIGBDEDAT0_SEC_BIT_S			(0)
#define IRIGBDEDAT0_TSEC_MASK			(0x7)
#define IRIGBDEDAT0_TSEC_BIT_S			(5)
#define IRIGBDEDAT0_MIN_MASK			(0xf)
#define IRIGBDEDAT0_MIN_BIT_S			(8)
#define IRIGBDEDAT0_TMIN_MASK			(0x7)
#define IRIGBDEDAT0_TMIN_BIT_S			(13)
#define IRIGBDEDAT0_HOUR_MASK			(0xf)
#define IRIGBDEDAT0_HOUR_BIT_S			(17)
#define IRIGBDEDAT0_THOUR_MASK			(0x3)
#define IRIGBDEDAT0_THOUR_BIT_S			(22)

#define IRIGBDEDAT1_DAY_MASK			(0xf)
#define IRIGBDEDAT1_DAY_BIT_S			(0)
#define IRIGBDEDAT1_TDAY_MASK			(0x3f)
#define IRIGBDEDAT1_TDAY_BIT_S			(5)
#define IRIGBDEDAT1_HDAY_MASK			(0x3)
#define IRIGBDEDAT1_HDAY_BIT_S			(9)
#define IRIGBDEDAT1_YEAR_MASK			(0xf)
#define IRIGBDEDAT1_YEAR_BIT_S			(18)
#define IRIGBDEDAT1_TYEAR_MASK			(0xf)
#define IRIGBDEDAT1_TYEAR_BIT_S			(23)

#define IRIGBDEDAT2_BIT_LSP			(1<<0)
#define IRIGBDEDAT2_BIT_LS			(1<<1)
#define IRIGBDEDAT2_BIT_DSP			(1<<2)
#define IRIGBDEDAT2_BIT_DST			(1<<3)
#define IRIGBDEDAT2_BIT_TZS			(1<<4)
#define IRIGBDEDAT2_BIT_TZH			(1<<9)
#define IRIGBDEDAT2_TZ_MASK			(0xf)
#define IRIGBDEDAT2_TZ_BIT_S			(5)
#define IRIGBDEDAT2_TQ_MASK			(0xf)
#define IRIGBDEDAT2_TQ_BIT_S			(10)
#define IRIGBDEDAT2_BIT_PAR			(1<<14)

//-----------------------------------------------------------------------------
// Define the Pulse per second configuration Registers
//-----------------------------------------------------------------------------
#define PPSCON_DE_HOLDTIME_MASK			(0xffff)
#define PPSCON_DE_HOLDTIME_BIT_S		(0)
#define PPSCON_EN_PULSEWIDTH_MASK		(0xffff)
#define PPSCON_EN_PULSEWIDTH_BIT_S		(16)

//-----------------------------------------------------------------------------
// Define the Time module configuration Registers
//-----------------------------------------------------------------------------
#define TMCON_BIT_IRIGDE0PARCHK			(1<<0)
#define TMCON_BIT_IRIGDE1PARCHK			(1<<1)
#define TMCON_BIT_IRIGENPARCHK			(1<<2)

//-----------------------------------------------------------------------------
// Define the Interrupt status/mask Registers
//-----------------------------------------------------------------------------
#define INTSTS_BIT_IRIG0DE_OFF			(1<<0)
#define INTSTS_BIT_IRIG0DE_FRMERR		(1<<1)
#define INTSTS_BIT_IRIG0DE_PARERR		(1<<2)
#define INTSTS_BIT_IRIG0DE_DONE			(1<<3)
#define INTSTS_BIT_IRIG1DE_OFF			(1<<4)
#define INTSTS_BIT_IRIG1DE_FRMERR		(1<<5)
#define INTSTS_BIT_IRIG1DE_PARERR		(1<<6)
#define INTSTS_BIT_IRIG1DE_DONE			(1<<7)
#define INTSTS_BIT_PPSDE_TIMEOUT		(1<<8)
#define INTSTS_BIT_PPSDE_DONE			(1<<9)
#define INTSTS_BIT_IRIGEN_DONE			(1<<10)
#define INTSTS_BIT_PPSEN_DONE			(1<<11)

//-----------------------------------------------------------------------------
// Define the RTC configuration Registers
//-----------------------------------------------------------------------------
#define RTCCON_SYNCSRC_MASK			(0x7)
#define RTCCON_SYNCSRC_FREERUN			(0x0)
#define RTCCON_SYNCSRC_IRIG0			(0x1)
#define RTCCON_SYNCSRC_IRIG1			(0x2)
#define RTCCON_SYNCSRC_PPS			(0x3)
#define RTCCON_BIT_LS_TYPE			(1<<5)

//-----------------------------------------------------------------------------
// Define the RTC data Registers
//-----------------------------------------------------------------------------
#define RTCDAT0_SEC_MASK			(0xff)
#define RTCDAT0_SEC_BIT_S			(0)
#define RTCDAT0_MIN_MASK			(0xff)
#define RTCDAT0_MIN_BIT_S			(8)
#define RTCDAT0_HOUR_MASK			(0xff)
#define RTCDAT0_HOUR_BIT_S			(16)
#define RTCDAT0_DAY_MASK			(0xff)
#define RTCDAT0_DAY_BIT_S			(24)
#define RTCDAT1_MONTH_MASK			(0xff)
#define RTCDAT1_MONTH_BIT_S			(0)
#define RTCDAT1_YEAR_MASK			(0xffff)
#define RTCDAT1_YEAR_BIT_S			(8)
#define RTCDAT1_BIT_COMMIT_TIME			(1<<31)
#define RTCDAT2_NANO_SEC
#define RTCDAT3_BIT_LSP				(1<<0)
#define RTCDAT3_BIT_LS				(1<<1)
#define RTCDAT3_BIT_DSP				(1<<2)
#define RTCDAT3_BIT_DST				(1<<3)
#define RTCDAT3_BIT_TZS				(1<<4)
#define RTCDAT3_BIT_TZH				(1<<5)
#define RTCDAT3_TZ_MASK				(0xf)
#define RTCDAT3_TZ_BIT_S			(6)
#define RTCDAT3_TQ_MASK				(0xf)
#define RTCDAT3_TQ_BIT_S			(10)

//-----------------------------------------------------------------------------
// Define the RTC leap second Registers
//-----------------------------------------------------------------------------
#define RTCLS_MIN_MASK				(0x7f)
#define RTCLS_MIN_BIT_S				(0)
#define RTCLS_HOUR_MASK				(0x3f)
#define RTCLS_HOUR_BIT_S			(7)
#define RTCLS_DAY_MASK				(0x3ff)
#define RTCLS_DAY_BIT_S				(13)
#define RTCLS_YEAR_MASK				(0xff)
#define RTCLS_YEAR_BIT_S			(23)
#define RTCLS_BIT_ENABLE			(1<<31)

//-----------------------------------------------------------------------------
// Define the RTC daylight saving time Registers
//-----------------------------------------------------------------------------
#define RTCDST_ST_HOUR_MASK			(0x3f)
#define RTCDST_ST_HOUR_BIT_S			(0)
#define RTCDST_ST_DAY_MASK			(0x3ff)
#define RTCDST_ST_DAY_BIT_S			(6)
#define RTCDST_ED_HOUR_MASK			(0x3f)
#define RTCDST_ED_HOUR_BIT_S			(16)
#define RTCDST_ED_DAY_MASK			(0x3ff)
#define RTCDST_ED_DAY_BIT_S			(22)

//-----------------------------------------------------------------------------
// Define the Serial FIFO configuration Registers
//-----------------------------------------------------------------------------
#define SFIFO_BIT_STS_FULL			(1<<16)
#define SFIFO_BIT_STS_EMPTY			(1<<17)
#define SFIFO_BIT_STS_CLEAR			(1<<18)

//-----------------------------------------------------------------------------
// Define the Serial FIFO data Registers
//-----------------------------------------------------------------------------
#define SFIFODAT_WIDTH_MASK			(0x7fffffff)
#define SFIFODAT_WIDTH_BIT_S			(0)
#define SFIFO_BIT_LEVEL				(1<<31)

//-----------------------------------------------------------------------------
// Define the Notify LED configuration Registers
//-----------------------------------------------------------------------------
#define NLED_MODE_OUTP0				(0)
#define NLED_MODE_OUTP1				(1)
#define NLED_MODE_OUTP2				(2)
#define NLED_MODE_OUTP3				(3)
#define NLED_MODE_OUTP4				(4)
#define NLED_MODE_OUTP5				(5)
#define NLED_MODE_IRIG0_OK			(6)
#define NLED_MODE_IRIG1_OK			(7)
#define NLED_MODE_INP0				(8)
#define NLED_MODE_INP1				(9)
#define NLED_MODE_INP2				(10)
#define NLED_MODE_INP3				(11)
#define NLED_MODE_INP4				(12)
#define NLED_MODE_INP5				(13)
#define NLED_MODE_INP6				(14)
#define NLED_MODE_INP7				(15)
#define NLED_MODE_MASK				(0xf)
#define NLED_P0_BIT_S				(0)
#define NLED_P1_BIT_S				(4)
#define NLED_P2_BIT_S				(8)
#define NLED_P3_BIT_S				(12)
#define NLED_P4_BIT_S				(16)
#define NLED_P5_BIT_S				(20)
#define NLED_P6_BIT_S				(24)
#define NLED_P7_BIT_S				(28)


#endif /* endof __IRIGB__ */
