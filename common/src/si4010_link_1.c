/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       si4010_link.c
 *    TARGET:     Si4010
 *    TOOLCHAIN:  Keil, Raisonance
 *    DATE:       June 14, 2010, Monday
 *    RELEASE:    2.2 (MPech), ROM version 02.00, trim 3
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      Definitions of reserved spaced in DATA and IDATA.
 *      The file must be included in the project and linked with
 *      an application to notify a linker about the absolute reserved
 *      spaces. By doing the space reserve this way makes things easier
 *      for linking.
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

/* API occupied DATA space */
#define bLink_DataApiBeg_c    0x30      /* DATA beginning address */
#define bLink_DataApiLen_c    0x10      /* Length in bytes */

/* API occupied IDATA space */
#define bLink_IdataApiBeg_c   0xc0      /* IDATA beginning address */
#define bLink_IdataApiLen_c   (0xff-bLink_IdataApiBeg_c+1)  /* Till the end */

/*
 *------------------------------------------------------------------------------
 *
 *    GLOBAL VARIABLES:
 */

/* -- Reserved DATA and IDATA API spaces */
BYTE data  abSi4010_DataApiReserve[bLink_DataApiLen_c]
                        _at_ bLink_DataApiBeg_c;

BYTE idata abSi4010_IdataApiReserve[bLink_IdataApiLen_c]
                        _at_ bLink_IdataApiBeg_c;

/* -- Reserved XDATA space */

/* The CODE/XDATA memory has address ranges 0x0000 .. 0x11FF. The si4010.h
 * through si4010_data.c specifies boot reserved area 0x11E0 .. 0x11FF.
 * Therefore, only the area 0x0000 .. 0x11DF is possibly available for user
 * CODE and XDATA.
 *
 * IMPORTANT: There is further limit on CODE/XDATA memory since
 * the Factory chip will use some the CODE/XDATA memory towards the
 * end of the CODE/XDATA space, ending at 0x11DF. The user must read
 * the wBoot_DpramTrimBeg variable to get the first occupied
 * address by Factory data. Anything below that (towards 0x0000)
 * is available for user CODE/XDATA to use.
 *
 * To automatically guide linker to avoid the reserved space
 * the user may modify the following section and modify
 * the beginning address of the reserved area in the CODE/XDATA RAM.
 * The user has to look at the content of wBoot_DpramTrimBeg
 * residing at the address 0x11F3. The variable content is directly accessible
 * from the SiLabs IDE:
 *   View -> Debug Windows -> Si4010 -> System Vars
 *
 * The user is still responsible for AVOIDING the CODE/XDATA
 * section overlap and for the CODE not reaching over
 * XDATA boundary specified in wBoot_DpramTrimBeg.
 */

/* >-- Modify and/or comment/uncomment the following section as needed: */

  #define wLink_XdataApiBeg_c  (0x1080)  /* <-- wBoot_DpramTrimBeg content */
  #define wLink_XdataApiLen_c  (0x11E0-wLink_XdataApiBeg_c)

  BYTE xdata abSi4010_XdataApiReserve[wLink_XdataApiLen_c]
                                _at_ wLink_XdataApiBeg_c;

/* <-- End of the linker reserved XDATA dummy section */

/*
 *------------------------------------------------------------------------------
 */

