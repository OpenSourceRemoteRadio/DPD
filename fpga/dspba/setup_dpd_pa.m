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
%% hepta_dpd_pa_param - DSPBA Design Parameters Start
clear hepta_dpd_pa_param;

%% System Parameters
hepta_dpd_pa_param.ChanCount             = 1;                    % How many real data channels
hepta_dpd_pa_param.ClockRate             = 245.760000;           % The system clock rate in MHz
hepta_dpd_pa_param.SampleRate            = 245.760000;           % The data rate per channel in MSps (mega-samples per second)
hepta_dpd_pa_param.ClockMargin           = 0.0;                  % Adjust the pipelining effort


%% Data Type Specification
hepta_dpd_pa_param.input_word_length     = 16;
hepta_dpd_pa_param.input_fraction_length = 15; 

%% Complex Mixer
hepta_dpd_pa_param.mixer_sample_rate     = hepta_dpd_pa_param.SampleRate; %Msps
hepta_dpd_pa_param.mixer_real_channel    = 1;
hepta_dpd_pa_param.mixer_freq_numbers    = 1;

%% Simulation Parameters
hepta_dpd_pa_param.SampleTime              = 1;                      % One unit in Simulink simulation is one clock cycle 
hepta_dpd_pa_param.endTime                 = 307199;                 % How many simulation clock cycles
hepta_dpd_pa_param.ContiguousValidFrames   = 1;        % Create a sequence of valid and invalid frames of stimulus data in the Channelizer block
hepta_dpd_pa_param.ContiguousInvalidFrames = 0;        % This will produce all valid frames

%% Stimulus data setup
hepta_dpd_pa_param.inputdata1(1,:)         = zeros(1,hepta_dpd_pa_param.endTime+1);

test_carr1                                 = 162.84;
test_data_3                                = 0.4*sin((0:hepta_dpd_pa_param.endTime)*2*pi*(test_carr1/hepta_dpd_pa_param.SampleRate));
test_data_4                                = 0.4*cos((0:hepta_dpd_pa_param.endTime)*2*pi*(test_carr1/hepta_dpd_pa_param.SampleRate));
test_data_5                                = test_data_4+i*test_data_3;
test_data_6                                = 0.4*(rand(1,hepta_dpd_pa_param.endTime+1));


hepta_dpd_pa_param.inputdata1(1,:)         = test_data_5;


%% Derived Parameters 
hepta_dpd_pa_param.Period                   = hepta_dpd_pa_param.ClockRate / hepta_dpd_pa_param.SampleRate;           % Clock cycles between consecutive data samples for a particular channel
hepta_dpd_pa_param.ChanWireCount            = ceil(hepta_dpd_pa_param.ChanCount/hepta_dpd_pa_param.Period);           % How many wires are needed to support the specified number of channels?
hepta_dpd_pa_param.ChanCycleCount           = ceil(hepta_dpd_pa_param.ChanCount/hepta_dpd_pa_param.ChanWireCount);    % Range of the channel signal
%% DSPBA Design Parameters End
