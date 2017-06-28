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
 * @file hepta_a10_2x4_hepta_dpd_xcorr.h
 * @defgroup hepta_a10_2x4_hepta_dpd_xcorr HEPTA A10 2X4 HEPTA DPD XCORR
 * @brief Header file for hepta_dpd_xcorr module
 * */

#ifndef HEPTA_A10_2X4_HEPTA_DPD_XCORR_H_
#define HEPTA_A10_2X4_HEPTA_DPD_XCORR_H_
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

#define Hepta_A10_2x4_hepta_dpd_xcorr_DPD_REV_PATH_HEPTA_DPD_XCORR_HEPTA_DPD_XCORR_INSTANCE_INDEX 0
#define Hepta_A10_2x4_hepta_dpd_xcorr_REGISTER_MAP_RESOURCE_INDEX 0

#define Hepta_A10_2x4_hepta_dpd_xcorr_CONFIG_COUNT 0
#define Hepta_A10_2x4_hepta_dpd_xcorr_SERVICE_COUNT 0
/******************************************************************************
 * Enumeration definitions
******************************************************************************/


/******************************************************************************
 * Register bit map definitions
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Definition of CORR_COEFFS
 * */


typedef union
{
    struct
   {
        unsigned int CORRELATION_COEFFS : 16;	
        unsigned int Resrvd1 : 16;	
   }reg;
    unsigned int value;
}hepta_dpd_xcorr_CORR_COEFFS_Def,*phepta_dpd_xcorr_CORR_COEFFS_Def;


/******************************************************************************
 * Register_Map Register map definition
******************************************************************************/

typedef struct
{
    volatile hepta_dpd_xcorr_CORR_COEFFS_Def CORR_COEFFS[512];
}hepta_a10_2x4_hepta_dpd_xcorr_Register_Map,*phepta_a10_2x4_hepta_dpd_xcorr_Register_Map;

/******************************************************************************
 * Function declaration
******************************************************************************/

pdevice_handler hepta_a10_2x4_hepta_dpd_xcorr_get_handler(unsigned int instance_num);
void * Hepta_A10_2x4_hepta_dpd_xcorr_isr_thread(void * hepta_dpd_xcorr_handler_ptr);
void * Hepta_A10_2x4_hepta_dpd_xcorr_register_isr(void * hepta_dpd_xcorr_handler_ptr,isr_callback callback_function,void * callback_argument);
void Hepta_A10_2x4_hepta_dpd_xcorr_reg_read(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_hepta_dpd_xcorr_reg_write(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_hepta_dpd_xcorr_config(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_hepta_dpd_xcorr_service(pCommand_Header pCmdHeader,unsigned int packet_id);
int send_service_ack_disable(psocket_manager_header socket_handler,unsigned int Id,Module_Index_Enum_Def module_index);

#ifdef __cplusplus
}/* close the extern "C" { */
#endif
#endif
