#ifndef _SI4010_API_ADD_H
#define _SI4010_API_ADD_H
/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       si4010_api_add.h
 *    TARGET:     Si4010
 *    TOOLCHAIN:  Keil
 *    DATE:       June 04, 2010, Friday
 *    RELEASE:    2.3 (MPech), ROM version 02.00, trim 3
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      This header file defines the functions present in the additional
 *      API library
 *
 *        lib/si4010_api_add_<toolchain>.lib
 *
 *      The library is a addition library implementing additional
 *      functions which are not included in ROM. The functions
 *      will occupy user CODE, XDATA, DATA/IDATA space. However,
 *      the linker can optimize the usage of the variables
 *      during the linking process.
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

/* Number of bytes collected by the bFStep_Collect function */
#define bFStep_DataLength_c     15

/*
 *------------------------------------------------------------------------------
 *    TYPE DEFINITIONS:
 *------------------------------------------------------------------------------
 *
 *    FUNCTION PROTOTYPES:
 */

/* ---- Temperature sensor (TS) and TS demodulator */

/* Get the temperature calibration factor. Only needed when the user
 * wants to get the temperature raw straight from the TS by using
 * iDmdTs_GetData(). The temperature is then iDmdTs_GetData()
 * value plus the calibration factor. */
int     iDmdTs_GetTempCal       /* Get temperature calibration */
        (
        void
        );

/* ---- Frequency counter functions */

/* Get the ratio of the high frequency clock delivered as clock
 * to the frequency counter to the carrier frequency f_rf */
float   fFc_GetFrfRatio
        (
        void
        );

/* Run FC and return a ratio of interval counter frequency
 * over the carrier frequency: f_int/f_rf */
float   fFc_CountFintOverFrf
        (
        void
        );

/* ---- FStep */

/* Must be called immediately after vFCast_Tune() call to collect
 * the generated LC tune data */
BYTE    bFStep_Collect
        (
        BYTE    xdata *pboData
        );

/* Behaves like vFCast_Tune() from Tx chain control point of view.
 * Forced DMD TS interrupt to be on and forces LC to be on. */
void    vFStep_Apply
        (
        BYTE    xdata *pbiData
        );

/* Applies the new LC tune constants only. It does not touch the DMD TS
 * interrupt setting nor it changes the LC oscillator forced on setting. */
void    vFStep_ApplyFast
        (
        BYTE    xdata *pbiData
        );

/* ---- NVM */

/* Enable NVM, load one block from NVM, and disable NVM. Recommended
 * function to load the user overlays. */
BYTE    bNvm_LoadBlock
        (
        WORD    wiNvmAddr  /* Block start address in NVM */
        );

/* ---- System functions */

/* Return the api_add library ID version in BCD. For example
 * 0x0201 means 02.01 version. The main version number corresponds
 * with the ROM main number as returned by wSys_GetRomId.
 * The secondary number is the revision of the library release. */

WORD    wSys_GetApiAddId
        (
        void
        );

/*
 *------------------------------------------------------------------------------
 */
#endif /* _SI4010_API_ADD_H */

