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
function setup_cordic_block(stages, angle_bits, angle_frac_bits, xy_bits, xy_frac_bits)

try
    
    % find system and set as modifiable
    sys = gcb;
    set_param(sys, 'MaskSelfModifiable', 'on');
    
    % location parameters
    % location parameters
    x_start =300;
    x_end =1800;
    y1_r = 137;
    y2_r = 243;
    y1_v = 287;
    y2_v = 393;
    
    w = floor((x_end-x_start)/(2*stages));
    
    % delete lines from drawing
    delete_line(find_system(sys, 'SearchDepth', 1,...
                'FollowLinks', 'on', 'LookUnderMasks', 'all',...
                'FindAll', 'on', 'Type', 'line'));
                
    % delete rotate blocks
    blocks = find_system(sys, 'SearchDepth', 1,...
                'regexp','on',...
                'FollowLinks', 'on', 'LookUnderMasks', 'all',...
                'Name', 'Rotate*');
    for i=1:length(blocks)
        delete_block(blocks{i});
    end
    
    % delete vector blocks
    blocks = find_system(sys, 'SearchDepth', 1,...
                'regexp','on',...
                'FollowLinks', 'on', 'LookUnderMasks', 'all',...
                'Name', 'Vector*');
    for i=1:length(blocks)
        delete_block(blocks{i});
    end
    
    
    % Input / Output ports should NOT be deleted
    
    % Draw system 
    
    for i=0:stages
        
        pos = [x_start+2*i*w y1_r x_start+(2*i+1)*w y2_r];
        
        add_block('dpd_cordic_lib/Rotate', [sys '/Rotate' int2str(i)], 'angle_bits', num2str(angle_bits),...
            'angle_frac_bits', int2str(angle_frac_bits), 'xy_bits', int2str(xy_bits),...
            'xy_frac_bits', int2str(xy_frac_bits), 'stage', int2str(i),...
            'Position', pos);
        
        pos = [x_start+2*i*w y1_v x_start+(2*i+1)*w y2_v];
        
        add_block('dpd_cordic_lib/Vector', [sys '/Vector' int2str(i)], 'angle_bits', num2str(angle_bits),...
            'angle_frac_bits', int2str(angle_frac_bits), 'xy_bits', int2str(xy_bits),...
            'xy_frac_bits', int2str(xy_frac_bits), 'stage', int2str(i),...
            'Position', pos);
        
    end
    
    % connect lines
    
    add_line(sys, 'av/1', 'qv/1');
    add_line(sys, 'ac/1', 'qc/1');
    
    add_line(sys, 'xin/1', 'Rotate0/1');
    add_line(sys, 'yin/1', 'Rotate0/2');
    add_line(sys, 'pin/1', 'Rotate0/3');
    
    add_line(sys, 'xin/1', 'Vector0/1');
    add_line(sys, 'yin/1', 'Vector0/2');
    add_line(sys, 'pin/1', 'Vector0/3');
    
    for i=0:stages-1
        
        for j=1:3
            
            add_line(sys, ['Rotate' int2str(i) '/' int2str(j)], ['Rotate' int2str(i+1) '/' int2str(j)]);
            
            add_line(sys, ['Vector' int2str(i) '/' int2str(j)], ['Vector' int2str(i+1) '/' int2str(j)]);
        
        end
    
    end
    
    for i=1:3
        
        add_line(sys, 'v/1', ['Mux' int2str(i) '/1']);
        
        add_line(sys, ['Rotate' int2str(stages) '/' int2str(i)], ['Mux' int2str(i) '/2']);
        add_line(sys, ['Vector' int2str(stages) '/' int2str(i)], ['Mux' int2str(i) '/3']);
        
    end
    
    add_line(sys, 'Mux1/1','xout/1');
    add_line(sys, 'Mux2/1','yout/1');
    add_line(sys, 'Mux3/1','pout/1');

catch e
    % report errors
    disp(e.getReport)
end