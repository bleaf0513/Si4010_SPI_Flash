#ifndef _SI4010_API_ROM
#define _SI4010_API_ROM
/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       si4010_api_rom.h
 *    TARGET:     Si4010
 *    TOOLCHAIN:  Generic
 *    DATE:       May 06, 2010, Thursday
 *    RELEASE:    2.2  (JimG, MPech), ROM version 02.00, trim 3
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      This header file is to be used with the si4010 API in ROM
 *      and should be included in all files which use the API functions
 *      from ROM.
 *
 *------------------------------------------------------------------------------
 *
 *    INCLUDES:
 */
#include "si4010.h"

/*
 *------------------------------------------------------------------------------
 *
 *    MACROS:
 */

/* SYS system module .. constant Trim Id system variable. User may need
 * to know the variable value before linking using IDE to see External Memory
 * location 0x11D6. */
#define bSys_TrimId_c   ((const BYTE)(*(BYTE const xdata *) 0x11D6))

/* ENC encoding module .. for vStl_EncodeSetup function */
#define bEnc_NoneNrz_c          0   /* No encoding */
#define bEnc_Manchester_c       1   /* Manchester encoding */
#define bEnc_4b5b_c             2   /* 4b-5b encoding */
#define bEnc_Custom_c           3   /* Custom encoding */

/* FC frequency counter module .. for vFc_Setup() function.
 * For example:
 *   vFc_Setup( bFc_ModeClkOsc_c | bFc_CountLc_c, bInterval );
 *
 * FC mode .. interval clock source, FC_CTRL[2:0] = FC_MODE */
#define bFc_ModeFcDisable_c     0
#define bFc_ModeClkRef_c        1
#define bFc_ModeClkOsc_c        2
#define bFc_ModeClkSys_c        3
#define bFc_ModeClkXo_c         4
#define bFc_ModeSleepTim_c      6

/* FC counter source selector .. ORed with the FC_MODE,
 * FC_CTRL[5] = FC_DIV_SEL */
#define bFc_CountLc_c           (0 << 5)
#define bFc_CountDiv_c          (1 << 5)

/*
 *------------------------------------------------------------------------------
 *
 *    TYPE DEFINITIONS:
 */

/* ---- Button Service Routine */

/* Structure for BSR setup. Space for Push Tracking Structure (PTS)
 * must be reserved by a designer in XDATA prior to initializing BSR. */
typedef struct {
  BYTE          bButtonMask;      /* Button watch mask */
  BYTE   xdata *pbPtsReserveHead; /* Allocated PTS array pointer */
  BYTE          bPtsSize;         /* Number of entries to be in PTS */
  BYTE          bPushQualThresh;  /* # consec button states to qualify push */
} tBsr_Setup;

/* An element of the internal Push Tracking Structure (PTS) */
typedef struct {
  BYTE  bButtonVector;  /* Vector of bits representing each button */
  BYTE  bTimestamp;     /* Timestamp associated with button vector */
} tBsr_PtsElement;

/* ---- Output data serializer (ODS) */

/* Structure for ODS setup.
 * For bGroupWidth only 3 LSb used. Value is (bitwidth-1).
 * For example if Tx bit width is 8 bits the value is set to 7,
 * that is for no or Mancheste encoding, if the width is 5 then
 * set to 4 for 4/5 encoding, etc. */
typedef struct {
  BYTE  bModulationType; /* Only LSb used. 0: OOK, 1: FSK */
  BYTE  bClkDiv;
  BYTE  bEdgeRate;       /* Only 2 LSb used */
  BYTE  bGroupWidth;     /* Only 3 LSb used, set to (bitwidth-1) */
  WORD  wBitRate;        /* Only bits 14 through 0 used */
  BYTE  bLcWarmInt;
  BYTE  bDivWarmInt;
  BYTE  bPaWarmInt;
} tOds_Setup;

/* ---- Power amplifier (PA) */

/* PA setup structure */
typedef struct {
  float fAlpha;
  float fBeta;
  BYTE  bLevel;
  BYTE  bMaxDrv;
  WORD  wNominalCap;
} tPa_Setup;

/* ---- Frequency casting (FCAST) */

/* Structure passed to vFCast_XoSetup function */
typedef struct {
  float fXoFreq;   /* Frequency of the external oscillator [Hz] */
  BYTE  bLowCap;   /* XO_LOWCAP bit control */
} tFCast_XoSetup;

/*
 *------------------------------------------------------------------------------
 *
 *    FUNCTION PROTOTYPES:
 */

/* ---- AES */

void    vAes_Cipher             /* AES 128 bit key encryption .. works
                                   on global IDATA State and RoundKey */
        (
        BYTE idata      *pbioState,
        BYTE idata      *pbioRoundKey
        );

void    vAes_InvCipher          /* AES 128 bit key decryption .. works
                                   on global IDATA State and RoundKey */
        (
        BYTE idata      *pbioState,
        BYTE idata      *pbioRoundKey
        );

void    vAes_InvGenKey          /* Generate inverse starting key */
        (
        BYTE idata      *pbioRoundKey
        );

/* ---- Button Service Routine (BSR) */

WORD    wBsr_Pop        /* Returns WORD: MSB timestamp, LSB button vector */
        (
        void
        );

void    vBsr_Setup
        (
        tBsr_Setup xdata *priSetup  /* Setup structure */
        );

void    vBsr_InitPts    /* Initialize PTS */
        (
        BYTE    biPtsSize
        );

BYTE    bBsr_GetPtsItemCnt
        (
        void
        );

void    vBsr_Service
        (
        void
        );

/* Returns WORD: MSB timestamp, LSB button vector of the latest button push */
WORD    wBsr_GetCurrentButton
        (
        void
        );

BYTE    bBsr_GetTimestamp
        (
        void
        );

/* ---- Temperature sensor (TS) and TS demodulator */

void    vDmdTs_Setup
        (
         BYTE   biTsCtrl,          /* Temp sensor TS control */
         BYTE   biDmdCtrl,         /* Demodulator DMD control */
         BYTE   biSampleSkipCount  /* # samples to skip before valid */
         );

int     iDmdTs_GetData   /* Get the current reading on temperature sensor */
        (
        void
        );

/* Get the latest sample read from the temperature sensor. This has the
 * side effect of clearing the samples taken count. */
int     iDmdTs_GetLatestDmdSample
        (
        void
        );

/* Get the latest temperature sensor sample with the SW trim value
 * iDmdTs_TempCal applied. This has the side effect of clearing
 * the samples taken count. */
int     iDmdTs_GetLatestTemp
        (
        void
        );

void    vDmdTs_ClearDmd   /* Clears the temperature sensor demod */
        (
        void
        );

void    vDmdTs_ClearDmdIntFlag   /* Clears the DMD interrupt flag */
        (
        void
        );

/* To be called from the TS ISR, which must be set in the main application.
 * It handles the skipping and collecting of TS data samples. */
void    vDmdTs_IsrCall
        (
        void
        );

/* Returns # of samples taken in TS ISR vDmdTs_IsrCall */
BYTE    bDmdTs_GetSamplesTaken
        (
        void
        );

/* The most significant two bits of biDmdTsSetEnable byte control the enable
 * of temperature sensor: bit 6 .. demod enable, bit 7 .. TS enable */
void    vDmdTs_Enable
        (
        BYTE    biDmdTsSetEnable   /* b7: TS enable, b6: demod enable */
        );

/* Wrapper for setting up the TS module for temperature sensing */
void    vDmdTs_RunForTemp
        (
        BYTE    biSampleSkipCount  /* # samples in vDmdTs_IsrCall()
                                    * to skip before collecting */
        );

/* Reset the sample counters in the DmdTs module  */
void    vDmdTs_ResetCounts
        (
        BYTE    biSampleSkipCount  /* # samples in vDmdTs_IsrCall()
                                    * to skip before collecting */
        );

/* ---- Encoding .. see also STL below */

BYTE    bEnc_4b5bEncode    /* Encodes two nibbles into two 5 bit blocks */
        (
        BYTE xdata *pboEncodedBytes,  /* Encoded array, each byte 5 bits */
        BYTE    biByteToEncode        /* Byte to be encoded */
        );

void    vEnc_Set4b5bLastBit
        (
        BYTE    bLastBitValue         /* Value to set into bEnc_4b5bLastBit */
        );

/* Manchester encoding .. each output byte contains a Manchester encoded
 * byte corresponding a nibble of the input byte */
BYTE    bEnc_ManchesterEncode
        (
        BYTE xdata *pboEncodedBytes,  /* Output encoded byte array */
        BYTE    biByteToEncode        /* Byte o be encoded */
        );

/* ---- Frequency casting */

/* Setup function which is required to be called prior to vFCast_Tune */
void    vFCast_Setup
        (
        void
        );

/* Setup function which is required to be called prior to
 * vFCast_Tune when using an external oscillator.
 * Input pointer to a structure in XDATA containing the XO frequency
 * and XO_LOWCAP bit setting */
void    vFCast_XoSetup
        (
        tFCast_XoSetup xdata *priXoSetup
        );

/* Tune the chip to the desired frequency. Takes about ~5ms. */
void    vFCast_Tune
        (
        float   fiDesiredFreq   /* Frequncy in [Hz] */
        );

/* Sets up proper FSK adjustment based on the desired frequency */
void    vFCast_FskAdj
        (
        BYTE    biFskDev  /* Desired FSK deviation number based on spreadsheed. */
        );

/* Calculate fine tune value and apply the result to hardware.
 * Input iiCurrentTemp is current temperature in internal temperature
 * representation .. output of iDmdTs_GetLatestTemp function. */
void    vFCast_FineTune
        (
        int     iiCurrentTemp
        );

/* ---- Frequency counter */

void    vFc_Setup
        (
        BYTE   biFcCtrl,
        BYTE   biFcInterval
        );

void    vFc_StartCount  /* Start the frequency counter */
        (
        void
        );

void    vFc_PollDone    /* Poll the FC_DONE flag and wait for done */
        (
        void
        );

LWORD   lFc_StartPollGetCount  /* Full FC run (start, poll, get count) */
        (
        void
        );

LWORD   lFc_GetCount    /* Return current FC_COUNT, done or not */
        (
        void
        );

/* ---- HVRAM */

BYTE    bHvram_Read     /* Read a byte from HVRAM */
        (
        BYTE    biReadAddr      /* HVRAM address */
        );

void    vHvram_Write /* Write a byte to HVRAM */
        (
        BYTE    biWriteAddr,
        BYTE    biWriteData
        );


/* ---- MTP and balanced counters (EEPROM) */

/* Depending on the width of a counter, the MTP module functions are to be
 * called using biMNibbleCnt argument values as specified in the following
 * table. The fewer bits used to store the counter the fewer transitions
 * the MTP will tolerate. Hence, the lower max values.
 *
 *   +---------------+-------+-----------+-------------+
 *   | Counter Width | Bytes | Max Value | biNibbleCnt |
 *   +---------------+-------+-----------+-------------+
 *   |     20 bits   |   3   |   550k    |   3         |
 *   |     24 bits   |   3   |   730k    |   4         |
 *   |     28 bits   |   4   |   915k    |   5         |
 *   |     32 bits   |   4   |  1040k    |   6         |
 *   +---------------+-------+-----------+-------------+
 *
 * Note that the biIndex value is a 0-based byte index into the MTP
 * byte array. Even though recommended values are 0, 4, 8, and 12
 * for counter starts, one can use any value if using counters of shorter
 * width then 4 bytes.
 *
 * For example if using 24 bit counter it only requires 3 bytes.
 * If user decided to have 4 24-bit counters, then it is possible
 * to put them on biIndex 0, 3, 6, 9 boundaries and have 4 byte starting
 * at index 12 for some other use. Any mix and match of the counters
 * is possible as long as their stored do not overlap. For 20 and 28 bit
 * counters there are 4 bits of counter byte unused and avaliable
 * for customer user, but they have to stay within a counter as a nibble.
 */

LWORD   lMtp_GetDecCount  /* Get decimal value of counter in abMtp_FullImage */
        (
        BYTE    biIndex,    /* Byte index of counter start in abMtp_FullImage
                             * Recommended values: 0, 4, 8, 12 */
        BYTE    biNibbleCnt /* Counter width. Valid values: 3, 4, 5, 6 */
        );

void    vMtp_SetDecCount  /* Set decimal value of counter in abMtp_FullImage */
        (
        LWORD   xdata *pliDecValue,  /* Pointer to decimal value */
        BYTE    biIndex,    /* Byte index of counter start in abMtp_FullImage
                             * Recommended values: 0, 4, 8, 12 */
        BYTE    biNibbleCnt /* Counter width. Valid values: 3, 4, 5, 6 */
        );

void    vMtp_IncCount     /* Increment specified counter in abMtp_FullImage */
        (
        BYTE    biIndex,    /* Same as above. Recommended values: 0, 4, 8, 12 */
        BYTE    biNibbleCnt /* Counter width. Valid values: 3, 4, 5, 6 */
        );

BYTE    bMtp_Write  /* Write 128 bits abMtp_FullImage to MTP. Disables ints.
                     * Returns: 0 .. success, 1 .. error */
        (
        BYTE    biProgLimit  /* Max # programming cycles */
        );

BYTE xdata *pbMtp_Read   /* Read 128 bits out of MTP into abMtp_FullImage.
                          * Returns pointer to abMtp_FullImage in XDATA. */
        (
        void
        );

void    vMtp_Strobe      /* Latch content of MTP into the XREG abMTP_RDATA */
        (
        void
        );

/* ---- MVdd battery Vdd measurement */

/* Input byte denotes a wait in 17us increments between the enabling
 * heavy load blocks on chip and battery measurement. If 0 then
 * no heavy loads are turned on. */
int     iMVdd_Measure
        (
        BYTE    biWait  /* 17us increments */
        );

/* ---- NVM copy */

/* Set the NVM address to read from. Must be called at least before
 * the first bNvm_CopyBlock call. */
void    vNvm_SetAddr
        (
        WORD    wiAddr
        );

WORD    wNvm_GetAddr   /* Get the first unread NVM address */
        (
        void
        );

/* Copy block starting at address set by vNvm_SetAddr. Return value of 0xFF
 * denotes an error during the copy process. */
BYTE    bNvm_CopyBlock
        (
        void
        );

/* Enable NVM for read. Must be called before the first bNvm_CopyBlock call */
void    vNvm_McEnableRead
        (
        void
        );

/* Disable NVM after the read is done. Must be called after the last
 * bNvm_CopyBlock call to save power. */
void    vNvm_McDisableRead
        (
        void
        );

/* ---- Output data serializer (ODS) */

void    vOds_Setup
        (
        tOds_Setup xdata *priSetup  /* ODS setup input structure */
        );

void    vOds_Enable
        (
        BYTE    biSetEnable   /* 1 .. ODS is enable, 0 .. ODS disable */
        );

void    vOds_WriteData
        (
        BYTE    biWriteData  /* Write data to ODS serializer */
        );

/* ---- Power amplifier (PA) */

void    vPa_Setup
        (
        tPa_Setup xdata *priSetup  /* PA setup structure */
        );

void    vPa_Tune
        (
        WORD    wiTemp    /* Temperature for Alpha term calculation */
        );

/* ---- System functions */

/* Controls which state the bandgap and LDO will be put into
 * biBandGapLdoCtrl values:
 *   0 : Bandgap Off, LDO low
 *   1 : Bandgap On,  LDO low
 *   2 : Bandgap On,  LDO 1.24v
 */
void    vSys_BandGapLdo
        (
        BYTE    biBandGapLdoCtrl
        );

WORD    wSys_GetRomId   /* Returns the ROM ID in BCD: 0x0100 = 01.00 */
        (
        void
        );

WORD    wSys_GetChipId  /* Get chip ID */
        (
        void
        );

BYTE    bSys_GetRevId   /* Get chip revision */
        (
        void
        );

/* Get SiLabs production ID number, random for each part */
LWORD   lSys_GetProdId
        (
        void
        );

/* Return boot status bBoot_BootStat. This function should be called
 * instead of directly accessing the global bBoot_BootStat variable. */
BYTE    bSys_GetBootStatus
        (
        void
        );

/* Set new SYSGEN clk_sys setting. The change is glitch free. */
void    vSys_SetClkSys
        (
        BYTE    biClkSetting  /* Only bottom 3 bits are used */
        );

/* Amount of time to be waited after boot on first battery insertion
 * in 40ms increments. */
void    vSys_Setup
        (
        BYTE    biFirstBatteryInsertWaitTime  /* 40ms increments */
        );

LWORD   lSys_GetMasterTime  /* Get current system master time in [ms] */
        (
        void
        );

void    vSys_IncMasterTime  /* Add time to master time in [ms] */
        (
        BYTE    biIncAmount /* [ms] to be added to lSys_Mastertime. */
        );

void    vSys_SetMasterTime  /* Set master time in [ms] */
        (
        LWORD   biSetAmount /* [ms] to set the mater time to */
        );

void    vSys_LedIntensity  /* Set LED intensity only */
        (
        BYTE    biLedIntensity  /* LED intensity value 0 .. 3 */
        );

void    vSys_Shutdown      /* Shutdown the chip */
        (
        void
        );

/* Software 16 bit delay function. Given the argument wiIntervalCount
 * the function will wait N system clock cycles. The function
 * switches to the fastest system clock internally and returns
 * the saved clock setting upon return.
 *
 * Overall clock cycle count spent inside of the function:
 *   N_clk_sys = 32 + 23 * wiIntervalCount
 *
 * Time spent:
 *   T = 14 * T_clk_sys_entry
 *       + (18 + 23 * wiIntervalCount) * T_clk_sys_fast
 *
 * The N includes the LJMP from the jump table and the final
 * RET instruction. It does not include the call instruction
 * from the caller.
 */
void    vSys_16BitDecLoop
        (
        WORD    wiIntervalCount
        );

/* Software 8 bit delay function. Given the argument biIntervalCount
 * the function will wait N system clock cycles. The function
 * switches to the fastest system clock internally and returns
 * the saved clock setting upon return.
 *
 * Overall clock cycle count spent inside of the function:
 *   N_clk_sys = 35 + 10 * biIntervalCount
 *
 * Time spent:
 *   T = 14 * T_clk_sys_entry
 *       + (21 + 10 * biIntervalCount) * T_clk_sys_fast
 *
 * The N includes the LJMP from the jump table and the final
 * RET instruction. It does not include the call instruction
 * from the caller.
 */
void    vSys_8BitDecLoop
        (
        BYTE    biIntervalCount
        );

void    vSys_FirstPowerUp
        (
        void
        );

WORD    wSys_GetKeilVer
        (
        void
        );

/* Force LC oscillator on if not already on and wait 125us */
void    vSys_ForceLc
        (
        void
        );

/* Adjust the 24MHz oscillator based on current temperature */
void    vSys_LpOscAdj
        (
        void
        );

/* ---- Sleep timer */

LWORD   lSleepTim_GetCount  /* Updates global variable value with the current
                             * timer value */
        (
        void
        );

void    vSleepTim_SetCount
        (
        LWORD   liCount  /* Value of 24 bit counter plus a power on bit
                          * in position 24 (25th bit) */
        );

/* Check duty cycle. The user will call this function prior to each transmit
 * if Tx duty cycle control is required.
 *
 * This function reads the UCHTIM counter and compares the value read there
 * with the value deemed to be 1 hour for this particular chip. This is done
 * using the soft register uchtim_time2bits and the following equation.
 *
 * Return:
 *   0 .. read value is greater than or equal to the
 *        1 hour value, 0 is returned meaning no transmit
 *        will be allowed.
 *   1 .. read value is less than the 1 hour value,
 *        1 is returned meaning a transmit will be allowed.
 */
BYTE    bSleepTim_CheckDutyCycle
        (
        void
        );

void    vSleepTim_AddTxTimeToCounter
        (
        WORD    wiIntegrand
        );

/* Return the value which represents one hour count. When this value
 * is set into the counter it will time out in 1 hour. Scale this
 * value to get the desired time to be set into the couner for chip wakeup. */
LWORD   lSleepTim_GetOneHourValue
        (
        void
        );

/* ---- Single Tx loop */

/* Encoding setup .. MUST be called before the byte encoding
 * is required. For customer encoding the user function passed
 * as pointer must have the prototype:
 *   BYTE    bEnc_CustomEncode  .. return # valid bytes in output array
 *           (
 *           BYTE xdata *pboEncodedBytes,  .. ptr to a result byte array
 *           BYTE    biByteToEncode        .. byte to be encoded
 *           );
 */
void    vStl_EncodeSetup
        (
        BYTE    biEncodeType,   /* See bEnc_..._c constants */
        BYTE    (code *pyiEncodeByteFcn)( BYTE xdata *, BYTE )
        );

/* Single frame Tx loop */
void    vStl_SingleTxLoop
        (
        BYTE    xdata *pbiFrameHead,  /* Pointer to frame in XDATA */
        BYTE    biFrameSize           /* Size of frame to be sent */
        );

/* Call before the first call to vStl_SingleTxLoop */
void    vStl_PreLoop
        (
        void
        );

/* Call after the last call to vStl_SingleTxLoop */
void    vStl_PostLoop
        (
        void
        );

/* Single byte encode function. Calls the encoding function previously
 * set by the vStl_EncodeSetup and returns the number of valid bytes
 * in the encoded array. Called from within the vStl_SingleTxLoop. */
BYTE    bStl_EncodeByte
        (
        BYTE xdata *pboEncodedBytes,
        BYTE    biByteToEncode
        );

/*
 *------------------------------------------------------------------------------
 */
#endif /* _SI4010_API_ROM */

