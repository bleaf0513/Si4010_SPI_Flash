#ifndef _SI4010_TYPES_H
#define _SI4010_TYPES_H
/*------------------------------------------------------------------------------
 *                          Silicon Laboratories, Inc.
 *                           http://www.silabs.com
 *                               Copyright 2010
 *------------------------------------------------------------------------------
 *
 *    FILE:       si4010_types.h
 *    TARGET:     Si4010
 *    TOOLCHAIN:  Keil, Raisonance
 *    DATE:       May 05, 2010, Wednesday
 *    RELEASE:    2.1 (MPech), ROM version 02.00
 *
 *------------------------------------------------------------------------------
 *
 *    DESCRIPTION:
 *      This file contains the type definitions for the Si4010.
 *
 *------------------------------------------------------------------------------
 *
 *    TYPE DEFINITIONS:
 */

/* -- Si4010 types */

/* Byte .. unsigned 8 bits .. prefix: b */
typedef unsigned char BYTE;

/* Word .. unsigned 16 bits .. prefix: w */
typedef unsigned int WORD;

/* Long word .. unsigned 32 bits .. prefix: l (el) */
typedef unsigned long int LWORD;

/* -- SiLabs types .. not currently used */
typedef unsigned char      u8;   /* b */
typedef unsigned int       u16;  /* w */
typedef unsigned long int  u32;  /* l */

typedef signed char        i8;    /* c */
typedef signed int         i16;   /* i */
typedef signed long int    i32;   /* j */

/* -- Typedefs for signed typed */

/* Char .. signed 8 bits .. prefix: c. Forced as signed per Misra rules. */
typedef signed char CHAR;

/* Integer .. signed 16 bits .. prefix: i */
/* Long integer .. signed 32 bits .. prefix: j */

/* Float .. IEEE 32 bits .. prefix: f */

/*
 *------------------------------------------------------------------------------
 */
#endif /* _SI4010_TYPES_H */

