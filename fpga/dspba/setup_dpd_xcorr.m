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
%% hepta_dpd_xcorr_param - DSPBA Design Parameters Start
clear hepta_dpd_xcorr_param; 

%% System Parameters
hepta_dpd_xcorr_param.ChanCount             = 1;                    % How many data channels
hepta_dpd_xcorr_param.ClockRate             = 245.76000;            % The system clock rate in MHz
hepta_dpd_xcorr_param.SampleRate            = 30.7200;              % The data rate per channel in MSps (mega-samples per second)
hepta_dpd_xcorr_param.ClockMargin           = 0.0;                  % Adjust the pipelining effort

%% Simulation Parameters
hepta_dpd_xcorr_param.SampleTime            = 1;                    % One unit in Simulink simulation is one clock cycle 
%hepta_dpd_xcorr_param.endTime              = 4096*4;               % How many simulation clock cycles
hepta_dpd_xcorr_param.endTime               = 16384*8;              % How many simulation clock cycles

%% Data Type Specification
hepta_dpd_xcorr_param.input_word_length     = 16;                   % Input data: bit width
hepta_dpd_xcorr_param.input_fraction_length = 15;                   % Input data: fraction width

hepta_dpd_xcorr_param.scale_word_length     = 23;                   % Output data: bit width
hepta_dpd_xcorr_param.scale_fraction_length = 15;                   % Output data: fraction width


 
load_data_obsrx=load('dpd_xcorr_obsrx.txt');
load_data_tx=load('dpd_xcorr_tx.txt');
length(load_data_obsrx)
test_data_1 =[ zeros(1,4012)  load_data_obsrx(1:end)'];
test_data_1 = test_data_1(1:end);

figure(10);plot(load_data_tx(1:end),'r');hold on;plot(load_data_obsrx(1:end),'g');hold off;

%% ModelIP setup
%% Filter 1
hepta_dpd_xcorr_param.xcorr_filt.SampleRate            = hepta_dpd_xcorr_param.SampleRate;         % Input rate at xcorr_filt
%hepta_dpd_xcorr_param.xcorr_filt.FilterLength         = 128;                                      % Number of Taps
hepta_dpd_xcorr_param.xcorr_filt.ChanCount             = 1;
hepta_dpd_xcorr_param.xcorr_filt.coeff_word_length     = 16;                                       % xcorr_filt coefficient: bit width
hepta_dpd_xcorr_param.xcorr_filt.coeff_fraction_length = 15;                                       % xcorr_filt coefficient: fraction width 
hepta_dpd_xcorr_param.xcorr_filt.base_addr             = 0;                                        % xcorr_filt coefficient address map (start)

hepta_dpd_xcorr_param.xcorr_filt.coeffs = fi((load_data_tx(512:-1:1)),1,hepta_dpd_xcorr_param.xcorr_filt.coeff_word_length,hepta_dpd_xcorr_param.xcorr_filt.coeff_fraction_length);
%% Simulation Parameters
hepta_dpd_xcorr_param.inputdata1=zeros(hepta_dpd_xcorr_param.ChanCount,length(test_data_1));
test_data   = [test_data_1];
hepta_dpd_xcorr_param.inputdata1=test_data;



%% Derived Parameters 
hepta_dpd_xcorr_param.Period          = hepta_dpd_xcorr_param.ClockRate / hepta_dpd_xcorr_param.SampleRate;           % Clock cycles between consecutive data samples for a particular channel
hepta_dpd_xcorr_param.ChanWireCount   = ceil(hepta_dpd_xcorr_param.ChanCount/hepta_dpd_xcorr_param.Period);           % How many wires are needed to support the specified number of channels?
hepta_dpd_xcorr_param.ChanCycleCount  = ceil(hepta_dpd_xcorr_param.ChanCount/hepta_dpd_xcorr_param.ChanWireCount);    % Range of the channel signal
%% DSPBA Design Parameters End
