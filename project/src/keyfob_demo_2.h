#ifndef _RKE_DEMO_H
#define _RKE_DEMO_H
/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       rke_demo.h
 *    TARGET:     Si4010 RevC
 *    TOOLCHAIN:  Keil
 *    DATE:       March 31, 2012,
 *    RELEASE:    3.0 (Tamas Nagy), ROM version 02.00
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      Header file for the rke_demo module.
 *
 *------------------------------------------------------------------------------
 *
 *    INCLUDES:
 */
#include "si4010_types.h"



//#define CRYSTAL			//define this if external crystal is used on GPIO0
//#define OOK                           //define this if OOK is used, otherwise FSK will be used

//defines GPIO pins used for button input (see button vector mapping in AN370)
#define bButtonMask_c   0x1F;	// GPIO0, GPIO1, GPIO2, GPIO3, GPIO4 

#define bEncodeNoneNrz_c       	0   /* No encoding */
#define bEncodeManchester_c    	1   /* Manchester encoding */
#define bModOOK_c				0
#define bModFSK_c				1

#define bPreambleSize_c		   	13	
#define bSyncSize_c                     2       
#define bPayloadSize_c		   	7
#define bCrcSize_c                      2	
#define bFrameSize_c		        (bPreambleSize_c + bSyncSize_c + bPayloadSize_c + bCrcSize_c)
#define bPayloadStartIndex_c            bPreambleSize_c + bSyncSize_c
#define bPreambleNrz_c			0xaa
#define bPreambleManch_c		0xff
#define bSync1_c				0x2d
#define bSync2_c				0xd4
//RF settings used in rke_demo_2 fw of demo keyfobs in Silabs kits:
  #define f_315_RkeFreqOOK_c		316660000.0	// for RKEdemo OOK
  #define f_315_RkeFreqFSK_c		316703093.0	// for RKEdemo FSK, upper frequency
  #define b_315_RkeFskDev_c			102	// deviation 43kHz
  #define b_315_PaLevel_c			60	// PA level
  #define b_315_PaMaxDrv_c			0	// PA level
  #define b_315_PaNominalCap_c		        270	// PA level

  #define f_433_RkeFreqOOK_c		433920000.0	// for RKEdemo OOK
  #define f_433_RkeFreqFSK_c		433979050.0	// for RKEdemo FSK, upper frequency
  #define b_433_RkeFskDev_c			102	// deviation 59kHz
  #define b_433_PaLevel_c			77	// PA level
  #define b_433_PaMaxDrv_c			1	// PA level
  #define b_433_PaNominalCap_c		        192	// PA level

  #define f_868_RkeFreqOOK_c		868300000.0	// for RKEdemo OOK
  #define f_868_RkeFreqFSK_c		868418922.0	// for RKEdemo FSK
  #define b_868_RkeFskDev_c			103     // deviation 119kHz
  #define b_868_PaLevel_c			77	// PA level
  #define b_868_PaMaxDrv_c			1	// PA level
  #define b_868_PaNominalCap_c		        81	// PA level

  #define f_915_RkeFreqOOK_c		917000000.0	// for RKEdemo OOK
  #define f_915_RkeFreqFSK_c		917120194.0	// for RKEdemo FSK
  #define b_915_RkeFskDev_c			93      // deviation 120kHz
  #define b_915_PaLevel_c			77	// PA level
  #define b_915_PaMaxDrv_c			1	// PA level
  #define b_915_PaNominalCap_c		        69	// PA level

#define iLowBatMv_c		2500

#define bButton1_c              0x01
#define bButton2_c              0x02
#define bButton3_c              0x04
#define bButton4_c              0x08
#define bButton5_c              0x10
//defines position of button bits in status byte of the transmitted packets
#define M_ButtonBits_c			0x1F    

#define wSys_16Bit_40ms_c		41737
// Amount of time, the application will wait after boot without
// getting a qualified button push before giving up and shutting the chip down
#define	bMaxWaitForPush_c		50
#define wRepeatInterval_c		100     //ms
#define bRepeatCount_c			3
#define bBatteryWait_c 			100
 // This specifies the number of times that the RTC ISR is called between 
 // each call to the button service routine.  The actual time between
 // calls is dependent on the setting of RTC_CTRL.RTC_DIV which dictates how
 // often the RTC ISR is called (5ms in this demo).It means that the interval
 // between calls to the vBsr_Service() function is 5*2=10ms.
#define	bDebounceInterval_c		2
// Size of FIFO of captured buttons .. max number of unserviced button changes
#define bPtsSize_c                      3  
//---------------------------------------------------------------------------
//    PROTOTYPES OF STATIC FUNCTIONS:
//-------------------------------------------------------------------------
void vPacketAssemble    /* Assemble packet for Rke demo receiver*/
      (
        void
      );
void vButtonCheck       /* Buttoons checking */
      (
        void
      );
void vRepeatTxLoop
      (
       void
      );
void vCalculateCrc
	  (
	  void
	  );
void vConvertPacket     /* change bit order and invert if necessary*/
      (
       BYTE bModType
      );
  //---------------------------------------------------------------------------
  //    VARIABLES:

  BYTE bIsr_DebounceCount;
  LWORD xdata lLEDOnTime;
  BYTE xdata bRepeatCount;
  LWORD lTimestamp;
  BYTE bdata bPrevBtn = 0;
  BYTE xdata *pbFrameHead;//Pointer to the head of the frame to be sent out.
  BYTE xdata bFrameSize;
  xdata BYTE abFrame[bFrameSize_c];
  xdata LWORD lPartID;
  BYTE bBatStatus;
  code BYTE abConvTable[16] = {0x00,0x88,0x44,0xcc,0x22,0xaa,0x66,0xee,0x11,0x99,
  	0x55,0xdd,0x33,0xbb,0x77,0xff};
  int iBatteryMv;
 
  BYTE bStatus;
  BYTE xdata bButtonState;

  float xdata fDesiredFreqOOK;
  float xdata fDesiredFreqFSK;
  float xdata fDesiredFreq;
  BYTE  xdata bFskDev;
  BYTE	bPreamble;
  /* Structure for setting up the ODS .. must be in XDATA */
  tOds_Setup xdata rOdsSetup;

  /* Structure for setting up the XO .. must be in XDATA */
  tFCast_XoSetup xdata rXoSetup;

  /* Structure for setting up the PA .. must be in XDATA */
  tPa_Setup xdata  rPaSetup;

  /* BSR control structure */
  tBsr_Setup xdata rBsrSetup;
  BYTE xdata abBsrPtsPlaceHolder [bPtsSize_c * 2] = {0};
  WORD wPacketCount;

#endif /* _RKE_DEMO_H */