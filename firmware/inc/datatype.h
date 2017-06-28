/*
Copyright (c) 2017, BigCat Wireless Pvt Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.



THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/**
 * @defgroup HeptaDataType User defined data types
 * @brief Header file for common data types
 */

#ifndef HEPTA_DATATYPES_H
#define HEPTA_DATATYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * OS and mode
 */

#if defined (__linux__) && defined (__KERNEL__)

/**
 * Linux kernel mode
 */

#include <linux/kernel.h>
#include <linux/types.h>

#elif defined (__freebsd) && defined (_KERNEL)

/**
 * FreeBSD kernel mode
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>

#elif defined (__linux__) || defined (__freebsd)

/**
 * Linux or FreeBSD, user mode
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

#else
#error Unsupported operating system
#endif 

/**
 * @ingroup HeptaDataType
 * @def NULL
 * @brief NULL definition.
 */

#ifndef NULL
#define NULL (0)
#endif

/**
 * @ingroup HeptaDataType
 * @def TRUE
 * @brief True value definition.
 */

#ifndef TRUE
#define TRUE (1)
#endif

/**
 * @ingroup HeptaDataType
 * @def FALSE
 * @brief False value definition.
 */

#ifndef FALSE
#define FALSE (0)
#endif

/**
 * @ingroup HeptaDataType
 * @def ENABLED
 * @brief Enable value definition.
 */

#ifndef ENABLED
#define ENABLED (1)
#endif

/**
 * @ingroup HeptaDataType
 * @def DISABLED
 * @brief Disable value definition.
 */

#ifndef DISABLED
#define DISABLED (0)
#endif

/**
 * @ingroup HeptaDataType
 * @def ERROR
 * @brief Error value definition.
 */

#ifndef ERROR
#define ERROR (-1)
#endif

/**
 * @ingroup HeptaDataType
 * @def SUCCESS
 * @brief SUCCESS value definition.
 */

#ifndef SUCCESS
#define SUCCESS (0)
#endif
/** 
 * @ingroup HeptaDataType
 * @{
 */

/**
 *  @brief Function is executed successfully
 */
#define HEPTA_SUCCESS 0
/**
 *  @brief Function is failed but unknown reason
 */
#define HEPTA_FAILURE -1
/**
 *  @brief Parameter to the function is not correct
 */
#define HEPTA_INVALID_PARAM -2
/**
 *  @brief Either hardware or software resource is not available
 */
#define HEPTA_RESOURCE_NOT_AVAIL -3
/**
 *  @brief Unsupported parameter is given to the function.
 */
#define HEPTA_UNSUPPORTED -4
/**
 *  @brief Memory allocation in the function is failed
 */
#define HEPTA_MEM_ALLOC_FAILURE -5
/**
 *  @brief Link list enqueue failure
 */
#define HEPTA_ENQUEUED -6
/**
 *  @brief Linklist dequeue failure
 */
#define HEPTA_DEQUEUED   -7

/** @} */

/**
 *   @ingroup HeptaDataType
 *   @brief user defined variable for byte access
 *   @typedef hepta_u8
 */

typedef unsigned char hepta_u8;



/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for single bit access.
 *   @enum HeptaBoolean
 *		provides a constant sets on boolean variable.
 */

typedef enum 
{
	HEPTA_FALSE = FALSE, /**< False value */
	HEPTA_TRUE = TRUE /**< True value */
}HeptaBoolean;




/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for signed byte memory or variable.
 *   @typedef   hepta_8
 */

typedef signed char hepta_8;

/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for unsigned word memory or variable
 *   @typedef   hepta_u16
 */

typedef unsigned short  hepta_u16;

/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for signed word memory or variable
 *   @typedef   hepta_16
 */

typedef signed short hepta_16;


/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for unsigned double word memory or variable
 *   @typedef   hepta_u32
 */

typedef unsigned int hepta_u32;

/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for signed double word memory or variable
 *   @typedef   hepta_32

 */
typedef signed int hepta_32;

/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for unsigned 64bit memory or variable
 *   @typedef   hepta_u64
 */


typedef unsigned long hepta_u64;

/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for signed 64bit memory or variable
 *   @typedef   hepta_64
 */

typedef signed long hepta_64;

/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of unsigned byte memory or variable
 *   @typedef   phepta_u8
 */

typedef unsigned char *phepta_u8;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of signed byte memory or variable
 *   @typedef   phepta_8
 */

typedef signed char *phepta_8;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of unsigned word memory or variable
 *   @typedef   phepta_u16
 */

typedef unsigned short  *phepta_u16;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of signed word memory or variable
 *   @typedef   phepta_16
 */

typedef signed short *phepta_16;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of unsigned double word memory or variable
 *   @typedef   phepta_u32
 */


typedef unsigned int *phepta_u32;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of signed double word memory or variable
 *   @typedef   phepta_32
 */

typedef signed int * phepta_32;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of unsigned 64bit memory or variable
 *   @typedef   phepta_u64
 */


typedef unsigned long * phepta_u64;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of signed 64bit memory or variable
 *   @typedef   phepta_64
 */

typedef signed long * phepta_64;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for holding bus address
 *   @typedef   phepta_bus_address
 */

typedef volatile unsigned int * phepta_bus_address;

/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for pointer of the handlers
 *   @typedef   hepta_handler
 */

typedef void*  hepta_handler;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for status of the operations
 *   @typedef   hepta_status
 */
typedef hepta_32 hepta_status;
/**
 *   @ingroup HeptaDataType
 *   @brief user defined constants for callbacks
 *   @typedef   pHeptaCallback
 */
typedef void(* pHeptaCallback)(void *pCallbackTag, hepta_status status);

/*#define	DEBUG_LEVEL_ALL   0x0
#define	DEBUG_LEVEL_INIT  0x1
#define DEBUG_LEVEL_EXIT  0x2
#define DEBUG_LEVEL_INFO  0x3
#define	DEBUG_LEVEL_WARN  0x4
#define	DEBUG_LEVEL_ERROR 0X5 */


extern int DebugLevel;
//#define _DEBUG_PRINT_

#ifdef _DEBUG_PRINT_
#define DEBUG_PRINT(X,Y) if(X>=DebugLevel) {do{ printf("%s : %d ",__FUNCTION__,__LINE__); printf Y; } while(0); } else{ do { }while(0);}
#else
#define DEBUG_PRINT(X,Y)
#endif


#define MSGDMA_IP 1
//#define MSGDMA_STANDALONE 1

#define COMMAND_MODE_ARGUMENT 1
#define HPS_DMA_BLOCKING 1
#define ALIGNED_PACKET   1
#define HPS_DMA_SUPPORT  1


void hepta_start_time_measurement();
float hepta_stop_time_measurement();



#ifdef __cplusplus
} /** close the extern "C" { */
#endif

#endif /** HEPTA_TYPES_H */

