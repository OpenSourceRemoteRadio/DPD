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
 * @file dpd_input_parameters.h
 * @brief Header file for defining input parameters.
 */

//HAVE TO BE DEFINED

#include "dpd_datatype.h"
#include <stdbool.h>


#define samplesize 16384 	/**< Buffer size of samples collecetd from input and output of power amplifier*/
#define MD 8			/**< Memory depth*///MD=M+1 if MD=8 then M=7

#define corxlength (samplesize+7)
#define CTxlength samplesize
#define Glength (corxlength*4)
#define N (MD*4)
#define GxGlength (N*N)
#define GxCTxlength (N*1)

/**< when MD=8, we have finally 32 coefficients and when MD=9, we have 36 coefficients*/


bool hepta_dpd_peaky_capture(Hepta_comp *buffer,unsigned int length, Hepta32F peak_threshold, Hepta32I max_peak_count);
Hepta64B hepta_dpd_condition_num(Hepta_comp *buffer,Hepta64B inv_buffer[(2*N)][(4*N)]);
Hepta32F hepta_dpd_mean_power(Hepta_comp * buffer,unsigned int length);
Hepta64B split_32UBuf (pHepta32U ptr_iq,Hepta_comp *sym,unsigned int size);
