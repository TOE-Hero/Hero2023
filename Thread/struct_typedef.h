#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>

/*------ 宏定义__packed，用来结构体字节对齐 -----*/
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION < 6010050) /* ARM Compiler V6 behind */
  #ifndef __weak
    #define __weak  __attribute__((weak))
  #endif
  #ifndef __packed
    #define __packed  __attribute__((packed))
  #endif
#endif

/*------ 以下为stdint库已经定义的类型 -----*/
typedef signed char int8_t;
typedef signed short int int16_t;
//typedef signed int int32_t;//这个在stdint库中被定义为 typedef long int32_t 
typedef signed long long int64_t;
/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
//typedef unsigned int uint32_t;//这个在stdint库中被定义为 typedef unsigned long uint32_t
typedef unsigned long long uint64_t;
/*---------------------------------------*/
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


#endif



