/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: sr_portable.h            Version: 1.0                          */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date: 15.03.2019                        */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description:                                                              */
/* Includes the type definitions for the target platform                    */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
#ifndef _SR_PORTABLE_H_
#define _SR_PORTABLE_H_

#include <stdint.h>
#include <limits.h>

typedef unsigned char Boolean;
typedef uint8_t U8;    
typedef char S8;
#if CHAR_BIT != 8
	#error "CHAR_BIT != 8, please re-typedef S8"
#endif
typedef uint16_t U16;   
typedef int16_t S16;   
typedef uint32_t U32; 
typedef int32_t S32; 
typedef float F32;  
typedef double F64;  
typedef uint64_t U64;
typedef int64_t S64;

#ifdef _MSC_VER
    #define TLS __declspec(thread)
	#define SR_DEF_PACKED
    #define FOLLOW_PRINTF_FORMAT(start, next) 
#else
    #define TLS thread_local
    #define SR_DEF_PACKED __attribute__( ( packed, aligned(1) ) ) 
    #define FOLLOW_PRINTF_FORMAT(start, next) __attribute__((format(printf, start, next)))
#endif

#endif
