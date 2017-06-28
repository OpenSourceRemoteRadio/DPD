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
%% demo_dpd_create_lut_entries - Computes lut entries 
%
%% Description
%   Computes 2048(256*8) lut entries from 32x1 inverse coefficients
%
%% Format
%   [coeff_data] = dpd_create_lut_entries(inv_coeff)
%
%% Inputs
%  inv_coeff : Input matrix of size 32x1 and the data format is complex
%
%% Outputs
%   coeff_data : Output Matrix of size 2048x1 and the data format is 32 bit
%   i and q concatenated.
%
%% Usage
%   coeff_data = dpd_create_lut_entries(inv_coeff) 

function [coeff_data] = dpd_create_lut_entries(inv_coeff)

LUTS = [];
LUTS.LUT_SIZE = 256;
LUTS.LUT_MAX_PWR_dBM = 1;

M = 7;
KSize = 4;
MSize = M+1;
order=[0 2 4 6];

for C = 1:LUTS.LUT_SIZE 
    LUT_ADDRESS = C;
    delta =C/LUTS.LUT_SIZE;             
    LUT_ENTRY = transpose(sum(reshape(inv_coeff,KSize,MSize).*transpose(repmat((delta*LUTS.LUT_MAX_PWR_dBM).^order(1:KSize),MSize,1))));        
    LUTS.ENTRIES(LUT_ADDRESS,:) = LUT_ENTRY(:);
end

dpd_coeff = reshape(transpose(LUTS.ENTRIES),MSize*LUTS.LUT_SIZE,1);
coeff_data = zeros(MSize*LUTS.LUT_SIZE,1);

for coeff_index = 1:length(dpd_coeff)
       
    coeff_data_lsb = int16(imag(dpd_coeff(coeff_index)).*2^11);
    coeff_data_msb = int16(real(dpd_coeff(coeff_index)).*2^11);
   
    coeff_data_lsb = uint32(typecast(coeff_data_lsb,'uint16'));
    coeff_data_msb = uint32(typecast(coeff_data_msb,'uint16'));
    coeff_data_msb = bitshift(coeff_data_msb,16);
    
    coeff_data(coeff_index) = bitor(coeff_data_msb,coeff_data_lsb);

end
end
