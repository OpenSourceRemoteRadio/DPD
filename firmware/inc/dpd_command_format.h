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
 * @file dpd_command_format.h
 * @brief Ethernet Header
 * */

#ifndef HEPTA_A10_2X4_COMMAND_FORMAT_H_
#define HEPTA_A10_2X4_COMMAND_FORMAT_H_
#ifdef __cplusplus
extern "C"
{
#endif
/******************************************************************************
 * Include public/global Header files
******************************************************************************/

#include <stdio.h>	
#define _GNU_SOURCE 
#define __USE_GNU	 
#include <stdlib.h>      
#include <string.h>      
#include <unistd.h>      
#include <sys/types.h>   
#include <sys/socket.h>  
#include <sys/select.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>   
#include <netdb.h>       
#include <signal.h>      
#include <unistd.h>      
#include <stdio.h>       
#include <stdlib.h>      
#include <string.h>      
#include <unistd.h>      
#include <sys/types.h>   
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>   
#include <sys/mman.h>    
#include <netdb.h>       
#include <sys/stat.h>    
#include <fcntl.h>       
#include <pthread.h>     
#include <sys/ioctl.h>   
#include <stdbool.h>

/******************************************************************************
 * Include private Header files
******************************************************************************/


#define TCP_PACKET_HEADER_STRING 		"alt"
#define TCP_PACKET_HEADER_STRING_SIZE   (sizeof(TCP_PACKET_HEADER_STRING)-1)
#define TCP_PACKET_HEADER_SIZE 			sizeof(Packet_Header)

#define COMMAND_DECODER_FUNCTION_NAME hepta_a10_2x4_command_decoder

/**
 * @ingroup CommandManager
 * @brief Tells the decoder in which module the operation to be performed
 * */


typedef enum
{
    MODULE_INDEX_SYSID=0,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_CPRI=1,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_SPI=2,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_CAPCTRL=3,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_JESDTX=4,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_JESDRX=5,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_DOWNLINK=6,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_REGMAP_DFD=7,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_UPLINK=8,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_DPD_REG_MAP_DFD=9,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_DPD_REG_MAP=10,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_ILC=11,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_TIMER=12,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_HEPTA_DPD_XCORR=13,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_PTX=14,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_POBSRX=15,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_CTX_WRITE=16,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_COBSRX_WRITE=17,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_TSE_TRANSMIT=18,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_TSE_CAPTURE=19,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_ETHERNET=20,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_MM2ST=21,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_AVST_TO_AVMM=22,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_MSGDMA=23,    /**< Operation to be performed in the sysid module*/
    MODULE_INDEX_GOERTZEL_CAPTURE=24,    /**< Operation to be performed in the buffer_mem module*/
    MODULE_INDEX_GOERTZEL_SPECTRUM=25,    /**< Operation to be performed in the buffer_mem module*/
    MODULE_INDEX_COMMAND_MANAGER=26,    /**< To configure the Command Manager module*/
}Module_Index_Enum_Def;

/**
 * @ingroup CommandManager
 * @brief Tells the decoder what to operation be performed
 * */


typedef enum
{
    OPERATION_TYPE_SET=0,    /**< set operation*/
    OPERATION_TYPE_GET=1,    /**< get operation*/
    OPERATION_TYPE_CONFIG=2,    /**< config operation*/
    OPERATION_TYPE_SERVICE=3,    /**< service operation*/
    OPERATION_TYPE_ERROR=4,    /**< Inform a error report to the server*/
    OPERATION_TYPE_ACK=5,    /**< Acknowledge to an action to the server*/
    OPERATION_TYPE_MANAGE=6,    /**< Command to manage the application*/
}Operation_Type_Enum_Def;

typedef struct
{
    char Header_String[3];    /**< This is the first field of Ethernet Packet. Incoming packets will be checked for this string*/
    unsigned int Id:8;    /**< Unique Id for each incomming command. Supports 256 commands simultaenously.*/
    unsigned int Length;    /**< Number of bytes succeeding header. This includes Command and data.  Maximum 65536 bytes of command and data transfer possible*/
    unsigned char server_index;
    char pData[0];    /**< Points to the memory that has command header*/
}__attribute__((__packed__))Packet_Header,*pPacket_Header;


typedef struct
{
    unsigned int Transfer_Type:1;    /**< This field specifies whether command is polling or service type. */
    unsigned int Module_Index:7;    /**< Index of module to configure.  Maximum 128 modules can be configured*/
    unsigned int Operation_Type:4;    /**< Operation to be performed e.g Read, Write and Config. Supports maximum 16 operations.*/
    unsigned int Instance:4;    /**< Instance of module to configure.  Maximum 16 instances of each modules can be configured*/
    unsigned int Resource:8;    /**< Index of the resource to be accessed*/
    unsigned short Address;    /**< Offset address or index of configuration to be done. Maximum 65536 address/offsets can be configured*/
    unsigned int Bit_Mask;    /**< To mask resgiter data during write and read*/
    //unsigned short Length;    /**< Number of data offsets succeeding command header.  Maximum 65536 words of data transfer supported*/
    unsigned int Length:22;    /**< Number of data offsets succeeding command header.  Maximum 65536 words of data transfer supported*/
    //char reserved[3];
    unsigned int is_CPRI_CM:1;		/**<Decides if it is C&M packet*/
    unsigned int server_index:1;
    //unsigned int reserved:6;
    char pData[0];    			/**< Points to the memory that has data*/
}__attribute__((__packed__))Command_Header,*pCommand_Header;


#ifdef __cplusplus
}/* close the extern "C" { */
#endif
#endif
