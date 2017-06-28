 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %  
 % Copyright (c) 2017, BigCat Wireless Pvt Ltd
 % All rights reserved.
 % 
 % Redistribution and use in source and binary forms, with or without
 % modification, are permitted provided that the following conditions are met:
 % 
 %     * Redistributions of source code must retain the above copyright notice,
 %       this list of conditions and the following disclaimer.
 %
 %     * Redistributions in binary form must reproduce the above copyright
 %       notice, this list of conditions and the following disclaimer in the
 %       documentation and/or other materials provided with the distribution.
 %
 %     * Neither the name of the copyright holder nor the names of its contributors
 %       may be used to endorse or promote products derived from this software
 %       without specific prior written permission.
 % 
 % 
 % 
 % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 % AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 % IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 % DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 % FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 % DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 % SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 % CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 % OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 % OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 % 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% demo_dpd_param - DSPBA Design Parameters Start

clear demo_dpd_param;
DSPBA_Features.EnableFanoutBlocks=true;             % To use FANOUT block in the design

%% System Parameters
demo_dpd_param.ChanCount     = 1;                    % How many real data channels
demo_dpd_param.ClockRate     = 245.760000;           % The system clock rate in MHz
demo_dpd_param.SampleRate    = 245.760000;            % The data rate per channel in MSps (mega-samples per second)
demo_dpd_param.ClockMargin   = 100.0;                  % Adjust the pipelining effort

%% Data Type Specification
demo_dpd_param.input_word_length      = 16;
demo_dpd_param.input_fraction_length  = 15; 
demo_dpd_param.output_word_length     = 16;         % Output data: bit width
demo_dpd_param.output_fraction_length = 15;         % Output data: fraction width



%% DPD address map
DPD_Start_Addr=0;

Coeff1_Addr=DPD_Start_Addr+1;
Coeff2_Addr=DPD_Start_Addr+2; 
Coeff3_Addr=DPD_Start_Addr+3;
Coeff4_Addr=DPD_Start_Addr+4;
Coeff5_Addr=DPD_Start_Addr+5;
Coeff6_Addr=DPD_Start_Addr+6;
Coeff7_Addr=DPD_Start_Addr+7;
Coeff8_Addr=DPD_Start_Addr+8;
LUT_Update_Addr=DPD_Start_Addr+9;
LUT_Read_Bank_Addr=DPD_Start_Addr+10;
Predistort_En_Addr=DPD_Start_Addr+11;

%% DPD IP Parameters

% Unity Coefficients
load('dpd_luts.mat');
% LUTS.ENTRIES(:,1)=0;

cordic_gain = 1.8;
WordLength = 16;
FractionLength = 15;
Coeff_width = 16;
Coeff_fractionwidth = 11;
OutputScalingFactor = 2^-FractionLength;
LUT_OutputScalingFactor = 2^-(FractionLength - 5);
stages = 8; %ceil(log2(size(LutContent,1))+2);
xy_angle_bits = 16;
xy_angle_fract_bits = 15;
xy_bits = 18;
xy_fract_bits = 14;
predistort0_id = 6;
predistort_en_value=1;       % '0' - dpd bypass
LUT_Read_Bank_Init_Val = 0;  % '0' - active bank , '1' - update banks
Latency = 0;                  
pipeline_Stages = 0;

%% Simulation Parameters
demo_dpd_param.SampleTime  = 1;                    % One unit in Simulink simulation is one clock cycle 
demo_dpd_param.endTime     = 5000;                 % How many simulation clock cycles
demo_dpd_param.ContiguousValidFrames   = 1;        % Create a sequence of valid and invalid frames of stimulus data in the Channelizer block
demo_dpd_param.ContiguousInvalidFrames = 0;        % This will produce all valid frames

%% Test Bench
carr_file_data_vec=load('tc06_lte20_30p72_0.mat');
carr_file_data=carr_file_data_vec.Hepta_Test_Vector_Data';
real_data = carr_file_data(1:2:end);
imag_data = carr_file_data(2:2:end);
%zero_data = zeros(1,length(carr_file_data)/2);
%demo_dpd_param.inputdata = zero_data;

data_quant=quantizer('fixed','floor','saturate',[16 15]);

real_data_quant=quantize(data_quant,real_data)*2^15;
imag_data_quant=quantize(data_quant,imag_data)*2^15;
demo_dpd_param.inputdata=imag_data_quant*2^16+real_data_quant;  

%% Derived Parameters 
demo_dpd_param.Period          = demo_dpd_param.ClockRate / demo_dpd_param.SampleRate;           % Clock cycles between consecutive data samples for a particular channel
demo_dpd_param.ChanWireCount   = ceil(demo_dpd_param.ChanCount/demo_dpd_param.Period);           % How many wires are needed to support the specified number of channels?
demo_dpd_param.ChanCycleCount  = ceil(demo_dpd_param.ChanCount/demo_dpd_param.ChanWireCount);    % Range of the channel signal

%% DSPBA Design Parameters End