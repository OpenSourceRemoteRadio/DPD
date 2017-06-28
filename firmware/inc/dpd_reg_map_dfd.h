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
 * @file dpd_reg_map_dfd.h
 * @defgroup dpd_reg_map_dfd HEPTA A10 2X4 DPD REG MAP DFD
 * @brief Header file for dpd_reg_map_dfd module
 * */

#ifndef HEPTA_A10_2X4_DPD_REG_MAP_DFD_H_
#define HEPTA_A10_2X4_DPD_REG_MAP_DFD_H_
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

#include "dpd_protocol.h"
#include "dpd_firmware.h"
#include "dpd_command_format.h"

#define Hepta_A10_2x4_dpd_reg_map_dfd_DPD_REV_PATH_DPD_REG_MAP_DFD_INSTANCE_INDEX 0
#define Hepta_A10_2x4_dpd_reg_map_dfd_REGISTER_MAP_RESOURCE_INDEX 0

#define Hepta_A10_2x4_dpd_reg_map_dfd_CONFIG_COUNT 0
#define Hepta_A10_2x4_dpd_reg_map_dfd_SERVICE_COUNT 0
/******************************************************************************
 * Enumeration definitions
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map_dfd
 * @brief DPD Reverse path pattern Tx play control
 * */


typedef enum
{
    DPD_REG_MAP_DFD_Start_RevPath_Ptx_Start=1,    /**< RevPath Ptx Start*/
    DPD_REG_MAP_DFD_Start_RevPath_Ptx_Stop=0,    /**< RevPath Ptx Stop*/
}DPD_REG_MAP_DFD_Revpath_Pattern_TxBuf_CTRL_Start_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map_dfd
 * @brief DPD reverse path pattern obs play control
 * */


typedef enum
{
    DPD_REG_MAP_DFD_Start_Revpath_PObs_Start=1,    /**< Revpath PObs Start*/
    DPD_REG_MAP_DFD_Start_Revpath_PObs_Stop=0,    /**< Revpath PObs Stop*/
}DPD_REG_MAP_DFD_Revpath_Pattern_ObsRxBuf_CTRL_Start_Enum_Def;


/******************************************************************************
 * Register bit map definitions
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map_dfd
 * @brief Definition of Revpath Pattern TxBuf CTRL
 * */


typedef union
{
    struct
   {
        unsigned int Start : 1;	/**< DPD Reverse path pattern Tx play control*/
        unsigned int Resrvd1 : 31;	
   }reg;
    unsigned int value;
}dpd_reg_map_dfd_RevpathPatternTxBufCTRL_Def,*pdpd_reg_map_dfd_RevpathPatternTxBufCTRL_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map_dfd
 * @brief Definition of Revpath Pattern ObsRxBuf CTRL
 * */


typedef union
{
    struct
   {
        unsigned int Start : 1;	/**< DPD reverse path pattern obs play control*/
        unsigned int Resrvd1 : 31;	
   }reg;
    unsigned int value;
}dpd_reg_map_dfd_RevpathPatternObsRxBufCTRL_Def,*pdpd_reg_map_dfd_RevpathPatternObsRxBufCTRL_Def;


/******************************************************************************
 * Register_Map Register map definition
******************************************************************************/

typedef struct
{
    volatile unsigned int Resrvd1[7];
    volatile dpd_reg_map_dfd_RevpathPatternTxBufCTRL_Def RevpathPatternTxBufCTRL;
    volatile dpd_reg_map_dfd_RevpathPatternObsRxBufCTRL_Def RevpathPatternObsRxBufCTRL;
}hepta_a10_2x4_dpd_reg_map_dfd_Register_Map,*phepta_a10_2x4_dpd_reg_map_dfd_Register_Map;

/******************************************************************************
 * Function declaration
******************************************************************************/

pdevice_handler hepta_a10_2x4_dpd_reg_map_dfd_get_handler(unsigned int instance_num);
void * Hepta_A10_2x4_dpd_reg_map_dfd_isr_thread(void * dpd_reg_map_dfd_handler_ptr);
void * Hepta_A10_2x4_dpd_reg_map_dfd_register_isr(void * dpd_reg_map_dfd_handler_ptr,isr_callback callback_function,void * callback_argument);
void Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_txbuf_ctrl_start_config_RevPath_Ptx_Start(pdevice_handler dpd_reg_map_dfd_device_handler);
void Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_txbuf_ctrl_start_config_RevPath_Ptx_Stop(pdevice_handler dpd_reg_map_dfd_device_handler);
void Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_txbuf_ctrl_config_Start(pdevice_handler dpd_reg_map_dfd_device_handler,DPD_REG_MAP_DFD_Revpath_Pattern_TxBuf_CTRL_Start_Enum_Def Start_configuration);
DPD_REG_MAP_DFD_Revpath_Pattern_TxBuf_CTRL_Start_Enum_Def Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_txbuf_ctrl_read_Start(pdevice_handler dpd_reg_map_dfd_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_txbuf_ctrl_isStart_RevPath_Ptx_Start(pdevice_handler dpd_reg_map_dfd_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_txbuf_ctrl_isStart_RevPath_Ptx_Stop(pdevice_handler dpd_reg_map_dfd_device_handler);
void Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_obsrxbuf_ctrl_start_config_Revpath_PObs_Start(pdevice_handler dpd_reg_map_dfd_device_handler);
void Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_obsrxbuf_ctrl_start_config_Revpath_PObs_Stop(pdevice_handler dpd_reg_map_dfd_device_handler);
void Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_obsrxbuf_ctrl_config_Start(pdevice_handler dpd_reg_map_dfd_device_handler,DPD_REG_MAP_DFD_Revpath_Pattern_ObsRxBuf_CTRL_Start_Enum_Def Start_configuration);
DPD_REG_MAP_DFD_Revpath_Pattern_ObsRxBuf_CTRL_Start_Enum_Def Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_obsrxbuf_ctrl_read_Start(pdevice_handler dpd_reg_map_dfd_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_obsrxbuf_ctrl_isStart_Revpath_PObs_Start(pdevice_handler dpd_reg_map_dfd_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_dfd_revpath_pattern_obsrxbuf_ctrl_isStart_Revpath_PObs_Stop(pdevice_handler dpd_reg_map_dfd_device_handler);
void Hepta_A10_2x4_dpd_reg_map_dfd_reg_read(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_dpd_reg_map_dfd_reg_write(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_dpd_reg_map_dfd_config(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_dpd_reg_map_dfd_service(pCommand_Header pCmdHeader,unsigned int packet_id);
int send_service_ack_disable(psocket_manager_header socket_handler,unsigned int Id,Module_Index_Enum_Def module_index);

#ifdef __cplusplus
}/* close the extern "C" { */
#endif
#endif
