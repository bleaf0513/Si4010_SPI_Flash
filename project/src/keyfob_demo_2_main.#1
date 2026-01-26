/*------------------------------------------------------------------------------
 * Silicon Laboratories, Inc.
 * Si4010 RevC
 * Toolchain: Keil C51
 * Bit-banged SPI Flash Read (CMD 0x03)
 *------------------------------------------------------------------------------*/

#include <intrins.h>
#include "si4010.h"
#include "si4010_api_rom.h"
#include "keyfob_demo_2.h"

/*------------------------------------------------------------------------------
 * TYPE DEFINITIONS (Keil-safe)
 *------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 * GPIO DEFINITIONS
 *------------------------------------------------------------------------------*/
sbit MY_LED    = P0^4;

/* SPI FLASH (bit-banged) */
sbit SPI_CS    = P1^1;   /* GPIO9  */
sbit SPI_CLK   = P0^6;   /* GPIO6  */
sbit SPI_MOSI  = P1^0;   /* GPIO8  */
sbit SPI_MISO  = P0^7;   /* GPIO7  */

/*------------------------------------------------------------------------------
 * SPI LOW-LEVEL SUPPORT
 *------------------------------------------------------------------------------*/
void spi_delay(void)
{
    _nop_();
    _nop_();
}

/*------------------------------------------------------------------------------
 * SPI WRITE BYTE (MSB first)
 *------------------------------------------------------------------------------*/
void spi_write_byte(BYTE _data)
{
    BYTE i;

    for (i = 0; i < 8; i++)
    {
        SPI_MOSI = (_data & 0x80) ? 1 : 0;
        _data <<= 1;

        SPI_CLK = 1;
        spi_delay();
        SPI_CLK = 0;
        spi_delay();
    }
}

/*------------------------------------------------------------------------------
 * SPI READ BYTE (MSB first)
 *------------------------------------------------------------------------------*/
BYTE spi_read_byte(void)
{
    BYTE i;
    BYTE _data = 0;

    for (i = 0; i < 8; i++)
    {
        _data <<= 1;

        SPI_CLK = 1;
        spi_delay();
        if (SPI_MISO)
            _data |= 1;
        SPI_CLK = 0;
        spi_delay();
    }
    return _data;
}

/*------------------------------------------------------------------------------
 * SPI FLASH READ (CMD = 0x03)
 *------------------------------------------------------------------------------*/
void spi_flash_read(LWORD    addr, BYTE *buf, BYTE len)
{
    BYTE i;

    SPI_CS = 0;

    spi_write_byte(0x03);                 /* READ command */
    spi_write_byte((BYTE)(addr >> 16));   /* A23..A16 */
    spi_write_byte((BYTE)(addr >> 8));    /* A15..A8  */
    spi_write_byte((BYTE)(addr));         /* A7..A0   */

    for (i = 0; i < len; i++)
        buf[i] = spi_read_byte();

    SPI_CS = 1;
}

/*------------------------------------------------------------------------------
 * READ FREQUENCY FROM FLASH @ 0x000000
 * Stored as LITTLE-ENDIAN DWORD
 *------------------------------------------------------------------------------*/
LWORD spi_flash_read_freq(void)
{
    BYTE  b[4];
    LWORD f;

    spi_flash_read(0x000000UL, b, 4);

    f  = ((LWORD)b[3] << 24);
    f |= ((LWORD)b[2] << 16);
    f |= ((LWORD)b[1] << 8);
    f |=  (LWORD)b[0];

    return f;
}

/*------------------------------------------------------------------------------
 * MAIN
 *------------------------------------------------------------------------------*/
void main(void)
{
    LWORD fFlashFreqHz;

    /* Basic system init */
    PDMD = 1;
    PORT_CTRL &= ~(M_PORT_MATRIX | M_PORT_ROFF | M_PORT_STROBE);
    GPIO_LED = 0;

    vSys_Setup(10);
    vSys_SetMasterTime(0);
    vSys_BandGapLdo(1);

    if ((PROT0_CTRL & M_NVM_BLOWN) > 1)
    {
        if (SYSGEN & M_POWER_1ST_TIME)
            vSys_FirstPowerUp();
    }

    vSys_LedIntensity(3);
    lLEDOnTime = 20;
    lPartID = lSys_GetProdId();
    bIsr_DebounceCount = 0;

    /* Button setup */
    rBsrSetup.bButtonMask = bButtonMask_c;
    rBsrSetup.pbPtsReserveHead = abBsrPtsPlaceHolder;
    rBsrSetup.bPtsSize = 3;
    rBsrSetup.bPushQualThresh = 3;
    vBsr_Setup(&rBsrSetup);

    RTC_CTRL = (0x07 << B_RTC_DIV) | M_RTC_CLR;
    RTC_CTRL |= M_RTC_ENA;
    ERTC = 1;
    EA = 1;

    /* ---------- FLASH FREQUENCY READ ---------- */
    fFlashFreqHz = spi_flash_read_freq();

    if (fFlashFreqHz < 300000000UL || fFlashFreqHz > 350000000UL)
        fFlashFreqHz = 315000000UL;   /* Safe fallback */

    fDesiredFreqOOK = (float)fFlashFreqHz;
    fDesiredFreqFSK = (float)fFlashFreqHz;
    bFskDev = b_315_RkeFskDev_c;

    /* PA setup */
    rPaSetup.bLevel      = b_315_PaLevel_c;
    rPaSetup.wNominalCap = b_315_PaNominalCap_c;
    rPaSetup.bMaxDrv     = b_315_PaMaxDrv_c;
    rPaSetup.fAlpha      = 0;
    rPaSetup.fBeta       = 0;
    vPa_Setup(&rPaSetup);

#ifdef OOK
    rOdsSetup.bModulationType = bModOOK_c;
    vStl_EncodeSetup(bEncodeManchester_c, NULL);
    fDesiredFreq = fDesiredFreqOOK;
    bPreamble = bPreambleManch_c;
#else
    rOdsSetup.bModulationType = bModFSK_c;
    vStl_EncodeSetup(bEnc_NoneNrz_c, 0);
    fDesiredFreq = fDesiredFreqFSK;
    bPreamble = bPreambleNrz_c;
#endif

    rOdsSetup.bGroupWidth = 7;
    rOdsSetup.bClkDiv = 5;
    rOdsSetup.bEdgeRate = 0;
    rOdsSetup.bLcWarmInt = 0;
    rOdsSetup.bDivWarmInt = 5;
    rOdsSetup.bPaWarmInt = 4;
    rOdsSetup.wBitRate = 417;
    vOds_Setup(&rOdsSetup);

    vFCast_Setup();

    iBatteryMv = iMVdd_Measure(bBatteryWait_c);
    bBatStatus = (iBatteryMv >= iLowBatMv_c);

    vDmdTs_RunForTemp(3);
    while (!bDmdTs_GetSamplesTaken());

    /* ---------- MAIN LOOP ---------- */
    while (1)
    {
        vButtonCheck();
        MY_LED = 1;

        if (bButtonState)
        {
            MY_LED = 0;
            bRepeatCount = bRepeatCount_c;
            vRepeatTxLoop();
        }
        else if ((lSys_GetMasterTime() >> 5) > bMaxWaitForPush_c)
        {
            EA = 0;
            vSys_Shutdown();
        }
    }
}

/*------------------------------------------------------------------------------
 * RTC INTERRUPT
 *------------------------------------------------------------------------------*/
void isr_rtc(void) interrupt INTERRUPT_RTC using 1
{
    RTC_CTRL &= ~M_RTC_INT;
    vSys_IncMasterTime(5);
    bIsr_DebounceCount++;

    if ((bIsr_DebounceCount % bDebounceInterval_c) == 0)
        vBsr_Service();
}

/*------------------------------------------------------------------------------
 * BUTTON CHECK
 *------------------------------------------------------------------------------*/
void vButtonCheck(void)
{
    ERTC = 0;
    bButtonState = 0;

    if (bBsr_GetPtsItemCnt())
    {
        bButtonState = wBsr_Pop() & 0xFF;

        if (bPrevBtn)
        {
            bPrevBtn = bButtonState;
            bButtonState = 0;
        }
        else
        {
            bPrevBtn = bButtonState;
        }
    }

    ERTC = 1;
}
