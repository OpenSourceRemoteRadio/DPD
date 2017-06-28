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
*//


#ifndef HEPTA_A10_2X4_DPD_ALGORITHM_H_
#define HEPTA_A10_2X4_DPD_ALGORITHM_H_

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

#include "hepta_a10_2x4_downlink.h"
#include "dpd_protocol.h"
#include "dpd_firmware.h"
#include "hepta_a10_2x4_ethernet.h"
#include "dpd_input_parameters.h"

typedef enum
{
	ALG_CNTRL_HPS=0x1,
	ALG_CNTRL_MATLAB,
}DPD_algorithm_CNTRL;

typedef enum
{
	CONFIG_ALG_CNTRL=0x1,
	CONFIG_TIME_PER_ITERATION,
	CONFIG_NO_OF_ITERATIONS,
	CONFIG_CONDITION_NUMBER_THRESHOLD,
	CONFIG_MEAN_POWER_THRESHOLD,
	CONFIG_PEAKY_CAPTURE_THRESHOLD,
	CONFIG_PEAK_COUNT_THRESHOLD,
	CONFIG_CORRELATION_THRESHOLD,
	CONFIG_ADAPT,
}DPD_Config;

typedef enum
{
	State_Sanity_Check=0x1,
	State_Main_Operation,
	State_Recovery,
	State_DPD_Total=State_Recovery,
}HeptaDPDStates;


typedef struct
{
	pthread_t        	pdpdThread;
	DPD_algorithm_CNTRL dpd_algorithm_control;
	unsigned int 		timer_interval_in_microsecs;
	unsigned int		number_of_iterations;
	pthread_mutex_t    	lock;
	pthread_cond_t	   	condition_signal; /**< Condition to signal the read/write threads*/
	unsigned int		triggers;
	cpu_set_t 			dpd_cpu_set;
	unsigned int		service_mask;
	Hepta_comp *		dpd_coefficients_ptr;
	HeptaDPDStates		dpd_current_state;
	bool				dpd_gain_increment;
	unsigned int * 		lut_coeff_ptr;
	unsigned int * 		backup_lut_coeff_ptr;
	double				condition_number_threshold;
	float				mean_power_threshold;
	unsigned int		capture_max_peak_count;
	float				capture_max_peak_abs;
	unsigned int		correlation_threshold;
	unsigned int		algorithm_retry_count;
	unsigned int		algorithm_max_retry_count;
	double	 			current_condition_number;
	float 				current_corx_mean_power;
	bool 				current_corx_peaky_capture;
	int					current_correlation_lag;
	double	 			max_abs_tx;
	double	 			max_abs_obs;
}hepta_dpd_Algorithm_Cntrl,*phepta_dpd_Algorithm_Cntrl;


#endif /* HEPTA_A10_2X4_DPD_ALGORITHM_H_ */
