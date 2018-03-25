/* 
 * File: Datatypes 
 * Author: Miguel A. Duran
 * Comments: Datatypes according to MISRA C 2004 guideline
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MISRA_DATATYPES_H
#define	MISRA_DATATYPES_H

#include <xc.h> 

typedef unsigned int              uint16_t;
typedef signed int                int16_t;
typedef unsigned short long int   uint24_t;
typedef signed short long int     int24_t;
typedef unsigned long int         uint32_t;
typedef signed long int           int32_t;
typedef char                      char_t;
typedef unsigned char             uint8_t;
typedef signed char               int8_t;
/*
 * The following datatypes should be uncommented IF the microcontroller
 * is able to use floating datatype AND the compiler support it.
 */
/*
   typedef float        float32_t;
   typedef double       float64_t:
   typedef long double  float128_t
 
 */
#endif	/* DATATYPES_H */