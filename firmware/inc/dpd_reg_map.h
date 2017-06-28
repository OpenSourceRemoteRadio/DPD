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
 * @file dpd_reg_map.h
 * @defgroup hepta_a10_2x4_dpd_reg_map HEPTA A10 2X4 DPD REG MAP
 * @brief Header file for dpd_reg_map module
 * */

#ifndef HEPTA_A10_2X4_DPD_REG_MAP_H_
#define HEPTA_A10_2X4_DPD_REG_MAP_H_
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

#define Hepta_A10_2x4_dpd_reg_map_DPD_REV_PATH_DPD_REG_MAP_INSTANCE_INDEX 0
#define Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX 0

#define Hepta_A10_2x4_dpd_reg_map_CONFIG_COUNT 0
#define Hepta_A10_2x4_dpd_reg_map_SERVICE_COUNT 0
/******************************************************************************
 * Enumeration definitions
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Correlation conrol
 * */


typedef enum
{
    DPD_REG_MAP_Corr_start_Start_Correlation=1,    /**< Start Correlation*/
}DPD_REG_MAP_CONTROL_Corr_start_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Reverse path capture status
 * */


typedef enum
{
    DPD_REG_MAP_RP_Capture_Performed_Reverse_path_capture_done=1,    /**< Reverse path capture done*/
}DPD_REG_MAP_FPGA_EVENTS_RP_Capture_Performed_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Correlation Lag status
 * */


typedef enum
{
    DPD_REG_MAP_Lag_Value_Computed_Lag_computation_done=1,    /**< Lag computation done*/
}DPD_REG_MAP_FPGA_EVENTS_Lag_Value_Computed_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Ctx configuration
 * */


typedef enum
{
    DPD_REG_MAP_Start_Ctx_Start_Capture=1,    /**< Ctx Start Capture*/
    DPD_REG_MAP_Start_Ctx_Stop_Capture=0,    /**< Ctx Stop Capture*/
}DPD_REG_MAP_CAPTX_BUF_CNTRL_Start_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Cobsrx configuration
 * */


typedef enum
{
    DPD_REG_MAP_Start_Cobsrx_Start_Capture=1,    /**< Cobsrx Start Capture*/
    DPD_REG_MAP_Start_Cobsrx_Stop_Capture=0,    /**< Cobsrx Stop Capture*/
}DPD_REG_MAP_CAPOBSRX_BUF_CNTRL_Start_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Cobs capture selection
 * */


typedef enum
{
    DPD_REG_MAP_Cobs_Capture_Select_Cobs_Pattern_data=0,    /**< Cobs Pattern data*/
    DPD_REG_MAP_Cobs_Capture_Select_Cobs_DPD_input_data=1,    /**< Cobs DPD input data*/
    DPD_REG_MAP_Cobs_Capture_Select_Cobs_DPD_output_data=2,    /**< Cobs DPD output data*/
    DPD_REG_MAP_Cobs_Capture_Select_Cobs_ADC_data=3,    /**< Cobs ADC data*/
}DPD_REG_MAP_Rev_Capture_Config_Cobs_Capture_Select_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Ctx capture selection
 * */


typedef enum
{
    DPD_REG_MAP_Ctx_Capture_Select_Ctx_Pattern_data=0,    /**< Ctx Pattern data*/
    DPD_REG_MAP_Ctx_Capture_Select_Ctx_DPD_input_data=1,    /**< Ctx DPD input data*/
    DPD_REG_MAP_Ctx_Capture_Select_Ctx_DPD_output_data=2,    /**< Ctx DPD output data*/
    DPD_REG_MAP_Ctx_Capture_Select_Ctx_ADC_data=3,    /**< Ctx ADC data*/
}DPD_REG_MAP_Rev_Capture_Config_Ctx_Capture_Select_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Reverse path capture configuration
 * */


typedef enum
{
    DPD_REG_MAP_Capture_Start_Reverse_path_capture_start=1,    /**< Reverse path capture start*/
    DPD_REG_MAP_Capture_Start_Reverse_path_capture_stop=0,    /**< Reverse path capture stop*/
}DPD_REG_MAP_Rev_Capture_Config_Capture_Start_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Reverse path sync capture configuration
 * */


typedef enum
{
    DPD_REG_MAP_Sync_Capture_bypass_Reverse_path_sync_capture_disable=1,    /**< Reverse path sync capture disable*/
    DPD_REG_MAP_Sync_Capture_bypass_Reverse_path_sync_capture_enable=0,    /**< Reverse path sync capture enable*/
}DPD_REG_MAP_Rev_Capture_Config_Sync_Capture_bypass_Enum_Def;


/******************************************************************************
 * Register bit map definitions
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of CONTROL
 * */


typedef union
{
    struct
   {
        unsigned int Corr_start : 1;	/**< Correlation conrol*/
        unsigned int Resrvd1 : 31;	
   }reg;
    unsigned int value;
}dpd_reg_map_CONTROL_Def,*pdpd_reg_map_CONTROL_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of CORR_LAG
 * */


typedef union
{
    struct
   {
        unsigned int CORRELATION_LAG : 16;	
        unsigned int Resrvd1 : 16;	
   }reg;
    unsigned int value;
}dpd_reg_map_CORR_LAG_Def,*pdpd_reg_map_CORR_LAG_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of CORR_MAX_VAL
 * */


typedef union
{
    struct
   {
        unsigned int CORRELATION_MAX_VALUE : 16;	
        unsigned int Resrvd1 : 16;	
   }reg;
    unsigned int value;
}dpd_reg_map_CORR_MAX_VAL_Def,*pdpd_reg_map_CORR_MAX_VAL_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of FPGA_EVENTS
 * */


typedef union
{
    struct
   {
        unsigned int RP_Capture_Performed : 1;	/**< Reverse path capture status*/
        unsigned int Lag_Value_Computed : 1;	/**< Correlation Lag status*/
        unsigned int Resrvd1 : 30;	
   }reg;
    unsigned int value;
}dpd_reg_map_FPGA_EVENTS_Def,*pdpd_reg_map_FPGA_EVENTS_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of TX_LAG
 * */


typedef union
{
    struct
   {
        unsigned int CAPTURE_TX_LAG : 16;	
        unsigned int Resrvd1 : 16;	
   }reg;
    unsigned int value;
}dpd_reg_map_TX_LAG_Def,*pdpd_reg_map_TX_LAG_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of OBSRX_LAG
 * */


typedef union
{
    struct
   {
        unsigned int CAPTURE_OBSRX_LAG : 16;	
        unsigned int Resrvd1 : 16;	
   }reg;
    unsigned int value;
}dpd_reg_map_OBSRX_LAG_Def,*pdpd_reg_map_OBSRX_LAG_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of CAPTX_BUF_CNTRL
 * */


typedef union
{
    struct
   {
        unsigned int Start : 1;	/**< Ctx configuration*/
        unsigned int Resrvd1 : 31;	
   }reg;
    unsigned int value;
}dpd_reg_map_CAPTX_BUF_CNTRL_Def,*pdpd_reg_map_CAPTX_BUF_CNTRL_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of CAPOBSRX_BUF_CNTRL
 * */


typedef union
{
    struct
   {
        unsigned int Start : 1;	/**< Cobsrx configuration*/
        unsigned int Resrvd1 : 31;	
   }reg;
    unsigned int value;
}dpd_reg_map_CAPOBSRX_BUF_CNTRL_Def,*pdpd_reg_map_CAPOBSRX_BUF_CNTRL_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of GAIN_VAL
 * */


typedef union
{
    struct
   {
        unsigned int GAIN_VALUE : 16;	
        unsigned int Resrvd1 : 16;	
   }reg;
    unsigned int value;
}dpd_reg_map_GAIN_VAL_Def,*pdpd_reg_map_GAIN_VAL_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of Capture_Delay_Pattern
 * */


typedef union
{
    struct
   {
        unsigned int Resrvd1 : 32;	
   }reg;
    unsigned int value;
}dpd_reg_map_Capture_Delay_Pattern_Def,*pdpd_reg_map_Capture_Delay_Pattern_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of Capture_Delay_DPD_in
 * */


typedef union
{
    struct
   {
        unsigned int Resrvd1 : 32;	
   }reg;
    unsigned int value;
}dpd_reg_map_Capture_Delay_DPD_in_Def,*pdpd_reg_map_Capture_Delay_DPD_in_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of Capture_Delay_DPD_out
 * */


typedef union
{
    struct
   {
        unsigned int Resrvd1 : 32;	
   }reg;
    unsigned int value;
}dpd_reg_map_Capture_Delay_DPD_out_Def,*pdpd_reg_map_Capture_Delay_DPD_out_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of Capture_Delay_ADC_out
 * */


typedef union
{
    struct
   {
        unsigned int Resrvd1 : 32;	
   }reg;
    unsigned int value;
}dpd_reg_map_Capture_Delay_ADC_out_Def,*pdpd_reg_map_Capture_Delay_ADC_out_Def;

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Definition of Rev_Capture_Config
 * */


typedef union
{
    struct
   {
        unsigned int Cobs_Capture_Select : 4;	/**< Cobs capture selection*/
        unsigned int Resrvd1 : 4;	
        unsigned int Ctx_Capture_Select : 4;	/**< Ctx capture selection*/
        unsigned int Resrvd2 : 4;	
        unsigned int Capture_Start : 1;	/**< Reverse path capture configuration*/
        unsigned int Sync_Capture_bypass : 1;	/**< Reverse path sync capture configuration*/
        unsigned int Resrvd3 : 14;	
   }reg;
    unsigned int value;
}dpd_reg_map_Rev_Capture_Config_Def,*pdpd_reg_map_Rev_Capture_Config_Def;


/******************************************************************************
 * Register_Map Register map definition
******************************************************************************/

typedef struct
{
    volatile unsigned int Resrvd1[1];
    volatile dpd_reg_map_CONTROL_Def CONTROL;
    volatile unsigned int Resrvd2[1];
    volatile dpd_reg_map_CORR_LAG_Def CORR_LAG;
    volatile dpd_reg_map_CORR_MAX_VAL_Def CORR_MAX_VAL;
    volatile unsigned int Resrvd3[2];
    volatile dpd_reg_map_FPGA_EVENTS_Def FPGA_EVENTS;
    volatile dpd_reg_map_TX_LAG_Def TX_LAG;
    volatile dpd_reg_map_OBSRX_LAG_Def OBSRX_LAG;
    volatile dpd_reg_map_CAPTX_BUF_CNTRL_Def CAPTX_BUF_CNTRL;
    volatile dpd_reg_map_CAPOBSRX_BUF_CNTRL_Def CAPOBSRX_BUF_CNTRL;
    volatile dpd_reg_map_GAIN_VAL_Def GAIN_VAL;
    volatile dpd_reg_map_Capture_Delay_Pattern_Def Capture_Delay_Pattern;
    volatile dpd_reg_map_Capture_Delay_DPD_in_Def Capture_Delay_DPD_in;
    volatile dpd_reg_map_Capture_Delay_DPD_out_Def Capture_Delay_DPD_out;
    volatile dpd_reg_map_Capture_Delay_ADC_out_Def Capture_Delay_ADC_out;
    volatile dpd_reg_map_Rev_Capture_Config_Def Rev_Capture_Config;
}hepta_a10_2x4_dpd_reg_map_Register_Map,*phepta_a10_2x4_dpd_reg_map_Register_Map;

/******************************************************************************
 * Function declaration
******************************************************************************/

pdevice_handler hepta_a10_2x4_dpd_reg_map_get_handler(unsigned int instance_num);
void * Hepta_A10_2x4_dpd_reg_map_isr_thread(void * dpd_reg_map_handler_ptr);
void * Hepta_A10_2x4_dpd_reg_map_register_isr(void * dpd_reg_map_handler_ptr,isr_callback callback_function,void * callback_argument);
void Hepta_A10_2x4_dpd_reg_map_control_corr_start_config_Start_Correlation(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_control_config_Corr_start(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_CONTROL_Corr_start_Enum_Def Corr_start_configuration);
DPD_REG_MAP_CONTROL_Corr_start_Enum_Def Hepta_A10_2x4_dpd_reg_map_control_read_Corr_start(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_control_isCorr_start_Start_Correlation(pdevice_handler dpd_reg_map_device_handler);
DPD_REG_MAP_FPGA_EVENTS_RP_Capture_Performed_Enum_Def Hepta_A10_2x4_dpd_reg_map_fpga_events_read_RP_Capture_Performed(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_fpga_events_isRP_Capture_Performed_Reverse_path_capture_done(pdevice_handler dpd_reg_map_device_handler);
DPD_REG_MAP_FPGA_EVENTS_Lag_Value_Computed_Enum_Def Hepta_A10_2x4_dpd_reg_map_fpga_events_read_Lag_Value_Computed(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_fpga_events_isLag_Value_Computed_Lag_computation_done(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_start_config_Ctx_Start_Capture(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_start_config_Ctx_Stop_Capture(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_config_Start(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_CAPTX_BUF_CNTRL_Start_Enum_Def Start_configuration);
DPD_REG_MAP_CAPTX_BUF_CNTRL_Start_Enum_Def Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_read_Start(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_isStart_Ctx_Start_Capture(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_isStart_Ctx_Stop_Capture(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_start_config_Cobsrx_Start_Capture(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_start_config_Cobsrx_Stop_Capture(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_config_Start(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_CAPOBSRX_BUF_CNTRL_Start_Enum_Def Start_configuration);
DPD_REG_MAP_CAPOBSRX_BUF_CNTRL_Start_Enum_Def Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_read_Start(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_isStart_Cobsrx_Start_Capture(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_isStart_Cobsrx_Stop_Capture(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_cobs_capture_select_config_Cobs_Pattern_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_cobs_capture_select_config_Cobs_DPD_input_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_cobs_capture_select_config_Cobs_DPD_output_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_cobs_capture_select_config_Cobs_ADC_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_config_Cobs_Capture_Select(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_Rev_Capture_Config_Cobs_Capture_Select_Enum_Def Cobs_Capture_Select_configuration);
DPD_REG_MAP_Rev_Capture_Config_Cobs_Capture_Select_Enum_Def Hepta_A10_2x4_dpd_reg_map_rev_capture_config_read_Cobs_Capture_Select(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCobs_Capture_Select_Cobs_Pattern_data(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCobs_Capture_Select_Cobs_DPD_input_data(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCobs_Capture_Select_Cobs_DPD_output_data(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCobs_Capture_Select_Cobs_ADC_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_ctx_capture_select_config_Ctx_Pattern_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_ctx_capture_select_config_Ctx_DPD_input_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_ctx_capture_select_config_Ctx_DPD_output_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_ctx_capture_select_config_Ctx_ADC_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_config_Ctx_Capture_Select(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_Rev_Capture_Config_Ctx_Capture_Select_Enum_Def Ctx_Capture_Select_configuration);
DPD_REG_MAP_Rev_Capture_Config_Ctx_Capture_Select_Enum_Def Hepta_A10_2x4_dpd_reg_map_rev_capture_config_read_Ctx_Capture_Select(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCtx_Capture_Select_Ctx_Pattern_data(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCtx_Capture_Select_Ctx_DPD_input_data(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCtx_Capture_Select_Ctx_DPD_output_data(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCtx_Capture_Select_Ctx_ADC_data(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_capture_start_config_Reverse_path_capture_start(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_capture_start_config_Reverse_path_capture_stop(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_config_Capture_Start(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_Rev_Capture_Config_Capture_Start_Enum_Def Capture_Start_configuration);
DPD_REG_MAP_Rev_Capture_Config_Capture_Start_Enum_Def Hepta_A10_2x4_dpd_reg_map_rev_capture_config_read_Capture_Start(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCapture_Start_Reverse_path_capture_start(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCapture_Start_Reverse_path_capture_stop(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_sync_capture_bypass_config_Reverse_path_sync_capture_disable(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_sync_capture_bypass_config_Reverse_path_sync_capture_enable(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_config_Sync_Capture_bypass(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_Rev_Capture_Config_Sync_Capture_bypass_Enum_Def Sync_Capture_bypass_configuration);
DPD_REG_MAP_Rev_Capture_Config_Sync_Capture_bypass_Enum_Def Hepta_A10_2x4_dpd_reg_map_rev_capture_config_read_Sync_Capture_bypass(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isSync_Capture_bypass_Reverse_path_sync_capture_disable(pdevice_handler dpd_reg_map_device_handler);
bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isSync_Capture_bypass_Reverse_path_sync_capture_enable(pdevice_handler dpd_reg_map_device_handler);
void Hepta_A10_2x4_dpd_reg_map_reg_read(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_dpd_reg_map_reg_write(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_dpd_reg_map_config(pCommand_Header pCmdHeader,unsigned int packet_id);
void Hepta_A10_2x4_dpd_reg_map_service(pCommand_Header pCmdHeader,unsigned int packet_id);
int send_service_ack_disable(psocket_manager_header socket_handler,unsigned int Id,Module_Index_Enum_Def module_index);

#ifdef __cplusplus
}/* close the extern "C" { */
#endif
#endif
