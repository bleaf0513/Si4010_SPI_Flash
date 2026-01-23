/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       isr_dmd.c
 *    TARGET:     Si4010
 *    TOOLCHAIN:  Generic
 *    DATE:       March 24, 2010, Wednesday
 *    RELEASE:    2.0 (JimG), ROM version 02.00
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      This module contains the interrupt service routine for
 *      the temperature demodulator (DMD). It is triggered when
 *      a demodulator sample is ready.
 *
 *------------------------------------------------------------------------------
 *
 *    INCLUDES:
 */
#include "si4010.h"
#include "si4010_api_rom.h"

/*
 *------------------------------------------------------------------------------
 *    MACROS:
 *    TYPEDEFS:
 *    PROTOTYPES OF STATIC FUNCTIONS:
 *    STATIC CONSTANT VARIABLES:
 *    STATIC VARIABLES GLOBAL TO ALL LOCAL FUNCTIONS:
 *==============================================================================
 *
 *    VISIBLE FUNCTIONS:
 */
void    vIsr_Dmd
        (
        void
        )
        interrupt INTERRUPT_DMD using 2  /* Use RB2 for this ISR */
/*------------------------------------------------------------------------------
 *
 *    FUNCTION DESCRIPTION:
 *      This is the interrupt service routine for the DMD. It clears the DMD
 *      interrupt flag and calls the vDmdTs_IsrCall() which handles the
 *      interface to the demodulator and temperature sensor.
 *
 *------------------------------------------------------------------------------
 */
{
/*
 *------------------------------------------------------------------------------
 *
 *    VARIABLES:
 *
 *------------------------------------------------------------------------------
 */

/* Clear the demodulator interrupt flag */
  vDmdTs_ClearDmdIntFlag();

/* Call DMD TS function that handles skipping samples and getting the sample
 * from the temperature demodulator. */
  vDmdTs_IsrCall();
}

/*
 *------------------------------------------------------------------------------
 */



