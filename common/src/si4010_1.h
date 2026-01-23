#ifndef _SI4010_H
#define _SI4010_H
/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       si4010.h
 *    TARGET:     Si4010
 *    TOOLCHAIN:  Keil, Raisonance
 *    DATE:       May 05, 2010, Wednesday
 *    RELEASE:    2.1 (MPech), ROM version 02.00
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      This header file defines all SFRs, X registers, and bit
 *      addressable registers for the si4010.
 *
 *      X registers are locations in xdata which are defined to have specific
 *      hardware uses.
 *
 *      Include of SiLabs si4010_types.h is required to define the types.
 *
 *------------------------------------------------------------------------------
 *
 *    INCLUDES:
 */
#include "si4010_types.h"

/*
 *------------------------------------------------------------------------------
 *
 *    MACROS:
 */

/*--------------------------------------------------------------
 *  SFR registers masks
 *--------------------------------------------------------------
 */

/* -- P0 .. 0x80 */
#define M_GPIO0               0x01
#define M_GPIO1               0x02
#define M_GPIO2               0x04
#define M_GPIO3               0x08
#define M_GPIO4               0x10
#define M_GPIO5               0x20
#define M_GPIO6               0x40
#define M_GPIO7               0x80

#define M_PORT_LED_ON         0x20
#define M_GPIO_LED            0x20

#define B_GPIO0               0
#define B_GPIO1               1
#define B_GPIO2               2
#define B_GPIO3               3
#define B_GPIO4               4
#define B_GPIO5               5
#define B_GPIO6               6
#define B_GPIO7               7

#define B_PORT_LED_ON         5
#define B_GPIO_LED            5

/* -- P1 .. 0x90 */
#define M_GPIO8               0x01
#define M_GPIO9               0x02

#define B_GPIO8               0
#define B_GPIO9               1

/* -- PCON .. 0x87 */
#define M_IDLE                0x01
#define M_STOP                0x02
#define M_GF5_0               0xfc

#define B_IDLE                0
#define B_STOP                1
#define B_GF5_0               2

/* -- CLKOUT_SET .. 0x8f */
#define M_CLKOUT_DIV          0x1f
#define M_CLKOUT_SYM          0x20
#define M_CLKOUT_INV          0x40
#define M_CLKOUT_CLR          0x80

#define B_CLKOUT_DIV          0
#define B_CLKOUT_SYM          5
#define B_CLKOUT_INV          6
#define B_CLKOUT_CLR          7

/* -- RBIT_DATA .. 0x99 */
#define M_ODS_NODATA          0x04
#define M_ODS_EMPTY           0x08
#define M_XO_CKGOOD           0x10
#define M_GPIO_LED_DRIVE      0x20

#define B_ODS_NODATA          2
#define B_ODS_EMPTY           3
#define B_XO_CKGOOD           4
#define B_GPIO_LED_DRIVE      5

/* -- FC_CTRL .. 0x9b */
#define M_FC_MODE             0x07
#define M_FC_DIV_SEL          0x20
#define M_FC_BUSY             0x40
#define M_FC_DONE             0x80

#define B_FC_MODE             0
#define B_FC_DIV_SEL          5
#define B_FC_BUSY             6
#define B_FC_DONE             7

/* -- RTC_CTRL .. 0x9c */
#define M_RTC_DIV             0x07
#define M_RTC_CLR             0x20
#define M_RTC_ENA             0x40
#define M_RTC_INT             0x80

#define B_RTC_DIV             0
#define B_RTC_CLR             5
#define B_RTC_ENA             6
#define B_RTC_INT             7

/* -- IE .. 0xa8 */
#define M_EINT0               0x01
#define M_ETMR2               0x02
#define M_EDMD                0x04
#define M_ERTC                0x08
#define M_EODS                0x10
#define M_ETMR3               0x20
#define M_EINT1               0x40
#define M_EA                  0x80

#define B_EINT0               0
#define B_ETMR2               1
#define B_EDMD                2
#define B_ERTC                3
#define B_EODS                4
#define B_ETMR3               5
#define B_EINT1               6
#define B_EA                  7

/* -- ODS_CTRL .. 0xa9 */
#define M_ODS_EN              0x01
#define M_FORCE_PA            0x02
#define M_FORCE_DIV           0x04
#define M_FORCE_LC            0x08
#define M_FSK_MODE            0x10
#define M_FSK_FORCE_DEV       0x20
#define M_ODS_SHIFT_CTRL      0xc0

#define B_ODS_EN              0
#define B_FORCE_PA            1
#define B_FORCE_DIV           2
#define B_FORCE_LC            3
#define B_FSK_MODE            4
#define B_FSK_FORCE_DEV       5
#define B_ODS_SHIFT_CTRL      6

/* -- ODS_TIMING .. 0xaa */
#define M_ODS_CK_DIV          0x07
#define M_ODS_EDGE_TIME       0x18
#define M_ODS_GROUP_WIDTH     0xe0

#define B_ODS_CK_DIV          0
#define B_ODS_EDGE_TIME       3
#define B_ODS_GROUP_WIDTH     5

/* -- ODS_WARM1 .. 0xae */
#define M_ODS_WARM_PA         0x0f
#define M_ODS_WARM_DIV        0xf0

#define B_ODS_WARM_PA         0
#define B_ODS_WARM_DIV        4

/* -- ODS_WARM2 .. 0xaf */
#define M_ODS_WARM_LC         0x0f

#define B_ODS_WARM_LC         0

/* -- PORT_CTRL .. 0xb5 */
#define M_PORT_LED            0x03
#define M_PORT_MIDRANGE       0x04
#define M_PORT_5_MIDRANGE     0x08
#define M_PORT_DRV2X          0x10
#define M_PORT_MATRIX         0x20
#define M_PORT_ROFF           0x40
#define M_PORT_STROBE         0x80

#define B_PORT_LED            0
#define B_PORT_MIDRANGE       2
#define B_PORT_5_MIDRANGE     3
#define B_PORT_DRV2X          4
#define B_PORT_MATRIX         5
#define B_PORT_ROFF           6
#define B_PORT_STROBE         7

/* -- PORT_SET .. 0xb6 */
#define M_PORT_SEREN          0x01
#define M_PORT_SERMODE        0x02
#define M_PORT_REFEN          0x04
#define M_PORT_CLKEN          0x08
#define M_PORT_CLKOUT         0x30
#define M_EDGE_INT0           0x40
#define M_EDGE_INT1           0x80

#define B_PORT_SEREN          0
#define B_PORT_SERMODE        1
#define B_PORT_REFEN          2
#define B_PORT_CLKEN          3
#define B_PORT_CLKOUT         4
#define B_EDGE_INT0           6
#define B_EDGE_INT1           7

/* -- PORT_INTCFG .. 0xb7 */
#define M_SEL_INT0            0x07
#define M_NEG_INT0            0x08
#define M_SEL_INT1            0x70
#define M_NEG_INT1            0x80

#define B_SEL_INT0            0
#define B_NEG_INT0            3
#define B_SEL_INT1            4
#define B_NEG_INT1            7

/* -- IP .. 0xb8 */
#define M_PINT0               0x01
#define M_PTMR2               0x02
#define M_PDMD                0x04
#define M_PRTC                0x08
#define M_PODS                0x10
#define M_PTMR3               0x20
#define M_PINT1               0x40

#define B_PINT0               0
#define B_PTMR2               1
#define B_PDMD                2
#define B_PRTC                3
#define B_PODS                4
#define B_PTMR3               5
#define B_PINT1               6

/* -- TMR3CTRL .. 0xb9 */
#define M_TMR3L_RUN           0x01
#define M_TMR3H_RUN           0x02
#define M_TMR3L_CAP           0x04
#define M_TMR3H_CAP           0x08
#define M_TMR3SPLIT           0x10
#define M_TMR3INTL_EN         0x20
#define M_TMR3INTL            0x40
#define M_TMR3INTH            0x80

#define B_TMR3L_RUN           0
#define B_TMR3H_RUN           1
#define B_TMR3L_CAP           2
#define B_TMR3H_CAP           3
#define B_TMR3SPLIT           4
#define B_TMR3INTL_EN         5
#define B_TMR3INTL            6
#define B_TMR3INTH            7

/* -- SYSGEN .. 0xbe */
#define M_SYSGEN_DIV          0x07
#define M_PORT_HOLD           0x08
#define M_RTC_TICKCLR         0x10
#define M_POWER_1ST_TIME      0x20
#define M_SYSGEN_SHUTDOWN     0x80

#define B_SYSGEN_DIV          0
#define B_PORT_HOLD           3
#define B_RTC_TICKCLR         4
#define B_POWER_1ST_TIME      5
#define B_SYSGEN_SHUTDOWN     7

/* -- INT_FLAGS .. 0xbf */
#define M_INT0_FLAG           0x01
#define M_INT1_FLAG           0x02
#define M_ODS_FLAG            0x04
#define M_VOID0_FLAG          0x08
#define M_VOID1_FLAG          0x10

#define B_INT0_FLAG           0
#define B_INT1_FLAG           1
#define B_ODS_FLAG            2
#define B_VOID0_FLAG          3
#define B_VOID1_FLAG          4

/* -- TMR2CTRL .. 0xc8 */
#define M_TMR2L_RUN           0x01
#define M_TMR2H_RUN           0x02
#define M_TMR2L_CAP           0x04
#define M_TMR2H_CAP           0x08
#define M_TMR2SPLIT           0x10
#define M_TMR2INTL_EN         0x20
#define M_TMR2INTL            0x40
#define M_TMR2INTH            0x80

#define B_TMR2L_RUN           0
#define B_TMR2H_RUN           1
#define B_TMR2L_CAP           2
#define B_TMR2H_CAP           3
#define B_TMR2SPLIT           4
#define B_TMR2INTL_EN         5
#define B_TMR2INTL            6
#define B_TMR2INTH            7

/* -- TMR_CLKSEL .. 0xc9 */
#define M_TMR2L_MODE          0x03
#define M_TMR2H_MODE          0x0c
#define M_TMR3L_MODE          0x30
#define M_TMR3H_MODE          0xc0

#define B_TMR2L_MODE          0
#define B_TMR2H_MODE          2
#define B_TMR3L_MODE          4
#define B_TMR3H_MODE          6

/* -- PA_LVL .. 0xce */
#define M_PA_LVL_BIAS         0x07
#define M_PA_LVL_NSLICE       0xf8

#define B_PA_LVL_BIAS         0
#define B_PA_LVL_NSLICE       3

/* -- PSW .. 0xd0 */
#define M_P                   0x01
#define M_F1                  0x02
#define M_OV                  0x04
#define M_RS0                 0x08
#define M_RS1                 0x10
#define M_F0                  0x20
#define M_AC                  0x40
#define M_CY                  0x80

#define B_P                   0
#define B_F1                  1
#define B_OV                  2
#define B_RS0                 3
#define B_RS1                 4
#define B_F0                  5
#define B_AC                  6
#define B_CY                  7

/* -- PROT0_CTRL .. 0xda */
#define M_NVM_BLOWN           0x07
#define M_NVM_WR_PROT         0x08
#define M_MTP_PROT            0x10
#define M_CLOCK_PROT          0x20
#define M_C2_OFF              0x40
#define M_NVM_PROT            0x80

#define B_NVM_BLOWN           0
#define B_NVM_WR_PROT         3
#define B_MTP_PROT            4
#define B_CLOCK_PROT          5
#define B_C2_OFF              6
#define B_NVM_PROT            7

/* -- BOOT_FLAGS .. 0xdd */
#define M_BFLAG_SYS           0x0f
#define M_BFLAG_POR           0xf0

#define B_BFLAG_SYS           0
#define B_BFLAG_POR           4

#define M_BOOT_CONT_SYS       0x01
#define M_BOOT_DONE_SYS       0x02
#define M_BOOT_FAIL_SYS       0x04
#define M_BOOT_CONT_POR       0x10
#define M_BOOT_TRIM_POR       0x20

#define B_BOOT_CONT_SYS       0
#define B_BOOT_DONE_SYS       1
#define B_BOOT_FAIL_SYS       2
#define B_BOOT_CONT_POR       4
#define B_BOOT_TRIM_POR       5

/* -- LC_FSK .. 0xe4 */
#define M_FSK_DEVIATION       0x7f
#define M_LC_DIVM             0x80

#define B_FSK_DEVIATION       0
#define B_LC_DIVM             7

/* -- EIE1 .. 0xe6 */
#define M_ESPI                0x01
#define M_EI2C                0x02
#define M_EFC                 0x04
#define M_EVOID0              0x08
#define M_EVOID1              0x10

#define B_ESPI                0
#define B_EI2C                1
#define B_EFC                 2
#define B_EVOID0              3
#define B_EVOID1              4

/* -- SYS_SET .. 0xee */
#define M_AES_DECRYPT         0x08

#define B_AES_DECRYPT         3

/* -- EIP1 .. 0xf6 */
#define M_PSPI                0x01
#define M_PI2C                0x02
#define M_PFC                 0x04
#define M_PVOID0              0x08
#define M_PVOID1              0x10

#define B_PSPI                0
#define B_PI2C                1
#define B_PFC                 2
#define B_PVOID0              3
#define B_PVOID1              4

/*--------------------------------------------------------------
 *  XREG registers masks
 *--------------------------------------------------------------
 */

/* -- bPA_TRIM .. 0x4012 */
#define M_PA_BIAS_TRIM      0x0f
#define M_PA_MAX_DRV        0x10

#define B_PA_BIAS_TRIM      0
#define B_PA_MAX_DRV        4

/* -- bXO_CTRL .. 0x4016 */
#define M_XO_ENA            0x01
#define M_XO_LOWCAP         0x02
#define M_XO_TST            0x0c

#define B_XO_ENA            0
#define B_XO_LOWCAP         1
#define B_XO_TST            2

/* -- bPORT_TST .. 0x4017 */
#define M_PORT_TST_IND      0x0f
#define M_PORT_TST_FS       0x70

#define B_PORT_TST_IND      0
#define B_PORT_TST_FS       4

/*
 *------------------------------------------------------------------------------
 *
 *    SFR DEFINITIONS:
 */

/* ---- SFR registers */
sfr P0             = 0x80;
sfr SP             = 0x81;
sfr DPL            = 0x82;
sfr DPH            = 0x83;
sfr GFM_DATA       = 0x84;
sfr GFM_CONST      = 0x85;
sfr SBOX_DATA      = 0x86;
sfr PCON           = 0x87;
sfr CLKOUT_SET     = 0x8f;
sfr P1             = 0x90;
sfr RBIT_DATA      = 0x99;
sfr FC_CTRL        = 0x9b;
sfr RTC_CTRL       = 0x9c;
sfr FC_INTERVAL    = 0x9d;
sfr P2             = 0xa0;
sfr P0CON          = 0xa4;
sfr P1CON          = 0xa5;
sfr IE             = 0xa8;
sfr ODS_CTRL       = 0xa9;
sfr ODS_TIMING     = 0xaa;
sfr ODS_DATA       = 0xab;
sfr ODS_RATEL      = 0xac;
sfr ODS_RATEH      = 0xad;
sfr ODS_WARM1      = 0xae;
sfr ODS_WARM2      = 0xaf;
sfr GPR_CTRL       = 0xb1;
sfr GPR_DATA       = 0xb2;
sfr PORT_CTRL      = 0xb5;
sfr PORT_SET       = 0xb6;
sfr PORT_INTCFG    = 0xb7;
sfr IP             = 0xb8;
sfr TMR3CTRL       = 0xb9;
sfr TMR3RL         = 0xba;
sfr TMR3RH         = 0xbb;
sfr TMR3L          = 0xbc;
sfr TMR3H          = 0xbd;
sfr SYSGEN         = 0xbe;
sfr INT_FLAGS      = 0xbf;
sfr TMR2CTRL       = 0xc8;
sfr TMR_CLKSEL     = 0xc9;
sfr TMR2RL         = 0xca;
sfr TMR2RH         = 0xcb;
sfr TMR2L          = 0xcc;
sfr TMR2H          = 0xcd;
sfr PA_LVL         = 0xce;
sfr PSW            = 0xd0;
sfr PROT0_CTRL     = 0xda;
sfr BOOT_FLAGS     = 0xdd;
sfr ACC            = 0xe0;
sfr LC_FSK         = 0xe4;
sfr EIE1           = 0xe6;
sfr SYS_SET        = 0xee;
sfr B              = 0xf0;
sfr EIP1           = 0xf6;

/* ---- Bit addressable ------------------------------------------------- */

/* P0 0x80 */
sbit GPIO0 = P0 ^ 0;
sbit GPIO1 = P0 ^ 1;
sbit GPIO2 = P0 ^ 2;
sbit GPIO3 = P0 ^ 3;
sbit GPIO4 = P0 ^ 4;
sbit GPIO5 = P0 ^ 5;
sbit GPIO6 = P0 ^ 6;
sbit GPIO7 = P0 ^ 7;

sbit PORT_LED_ON = P0 ^ 5;
sbit GPIO_LED    = P0 ^ 5;

/* P1 0x90 */
sbit GPIO8 = P1 ^ 0;
sbit GPIO9 = P1 ^ 1;

/* IE 0xa8 */
sbit EINT0       = IE ^ B_EINT0;
sbit ETMR2       = IE ^ B_ETMR2;
sbit EDMD        = IE ^ B_EDMD;
sbit ERTC        = IE ^ B_ERTC;
sbit EODS        = IE ^ B_EODS;
sbit ETMR3       = IE ^ B_ETMR3;
sbit EINT1       = IE ^ B_EINT1;
sbit EA          = IE ^ B_EA;

/* IP 0xb8 */
sbit PINT0       = IP ^ B_PINT0;
sbit PTMR2       = IP ^ B_PTMR2;
sbit PDMD        = IP ^ B_PDMD;
sbit PRTC        = IP ^ B_PRTC;
sbit PODS        = IP ^ B_PODS;
sbit PTMR3       = IP ^ B_PTMR3;
sbit PINT1       = IP ^ B_PINT1;
sbit IP_UNUSED   = IP ^ 7;

/* TMR2CTRL 0xc8 */
sbit TMR2L_RUN   = TMR2CTRL ^ B_TMR2L_RUN;
sbit TMR2H_RUN   = TMR2CTRL ^ B_TMR2H_RUN;
sbit TMR2L_CAP   = TMR2CTRL ^ B_TMR2L_CAP;
sbit TMR2H_CAP   = TMR2CTRL ^ B_TMR2H_CAP;
sbit TMR2SPLIT   = TMR2CTRL ^ B_TMR2SPLIT;
sbit TMR2INTL_EN = TMR2CTRL ^ B_TMR2INTL_EN;
sbit TMR2INTL    = TMR2CTRL ^ B_TMR2INTL;
sbit TMR2INTH    = TMR2CTRL ^ B_TMR2INTH;

/* PSW 0xd0 */
sbit P           = PSW ^ 0;
sbit F1          = PSW ^ 1;
sbit OV          = PSW ^ 2;
sbit RS0         = PSW ^ 3;
sbit RS1         = PSW ^ 4;
sbit F0          = PSW ^ 5;
sbit AC          = PSW ^ 6;
sbit CY          = PSW ^ 7;

/*
 *------------------------------------------------------------------------------
 *
 *    INTERRUPT PRIORITIES
 */

#define INTERRUPT_INT0             0   /* External interrupt 0 .. edge only */
#define INTERRUPT_TMR2             1   /* Timer 2 overflow */
#define INTERRUPT_DMD              2   /* Temp sensor and TS demod */
#define INTERRUPT_RTC              3   /* Real Time Clock pulse */
#define INTERRUPT_ODS              4   /* Output Data Serializer interrupt */
#define INTERRUPT_TMR3             5   /* Timer 3 overflow */
#define INTERRUPT_INT1             6   /* External interrupt 1 .. edge only */
                                       /*   7 .. reserved */
                                       /*   8 .. reserved */
#define INTERRUPT_FC               9   /* Frequency Counter interrupt */
#define INTERRUPT_VOID0           10   /* Software interrupt .. future use */
#define INTERRUPT_VOID1           11   /* Software interrupt .. future use */

/*
 *------------------------------------------------------------------------------
 *
 *    XREG EXTERNAL XDATA REGISTERS .. EXTERNAL VARIABLES:
 */

#ifdef _SI4010_DATA_C

/* -- XREG registers */
volatile BYTE  xdata bLPOSC_TRIM      _at_ 0x4002;
volatile LWORD xdata lFC_COUNT        _at_ 0x4008;
volatile WORD  xdata wPA_CAP          _at_ 0x400c;
volatile BYTE  xdata bPA_TRIM         _at_ 0x4012;
volatile BYTE  xdata bXO_CTRL         _at_ 0x4016;
volatile BYTE  xdata bPORT_TST        _at_ 0x4017;

/* Read only array of 16 bytes of MTP (EEPROM) */
volatile BYTE  xdata abMTP_RDATA[16]  _at_ 0x4040;

#else

/* ---- XREG external declarations -------------------------------------- */

extern volatile BYTE  xdata bLPOSC_TRIM;
extern volatile LWORD xdata lFC_COUNT;
extern volatile WORD  xdata wPA_CAP;
extern volatile BYTE  xdata bPA_TRIM;
extern volatile BYTE  xdata bXO_CTRL;
extern volatile BYTE  xdata bPORT_TST;

extern volatile BYTE  xdata abMTP_RDATA[16]; /* See above for details */

#endif /* _SI4010_DATA_C */

/*
 *------------------------------------------------------------------------------
 *
 *    XDATA BOOT STATUS .. EXTERNAL VARIABLES:
 */

#ifdef _SI4010_DATA_C

/* -- Boot status and boot reserved space */
volatile BYTE  xdata abBoot_Reserve0[19] _at_ 0x11e0;
volatile BYTE  xdata abBoot_Reserve1[6]  _at_ 0x11f7;

volatile BYTE  xdata wBoot_DpramTrimBeg  _at_ 0x11f3;
volatile BYTE  xdata wBoot_NvmUserBeg    _at_ 0x11f5;
volatile BYTE  xdata wBoot_NvmCopyAddr   _at_ 0x11fd;
volatile BYTE  xdata bBoot_BootStat      _at_ 0x11ff;

#else

/* ---- XDATA boot status declarations ---------------------------------- */

extern volatile BYTE  xdata abBoot_Reserve0[19];
extern volatile BYTE  xdata abBoot_Reserve1[6];

extern volatile BYTE  xdata wBoot_DpramTrimBeg;
extern volatile BYTE  xdata wBoot_NvmUserBeg;
extern volatile BYTE  xdata wBoot_NvmCopyAddr;
extern volatile BYTE  xdata bBoot_BootStat;

#endif /* _SI4010_DATA_C */

/*
 *------------------------------------------------------------------------------
 */
#endif /* _SI4010_H */

