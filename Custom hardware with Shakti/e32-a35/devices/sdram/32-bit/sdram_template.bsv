/* 
Copyright (c) 2018, IIT Madras All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions
  and the following disclaimer.  
* Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and/or other materials provided 
 with the distribution.  
* Neither the name of IIT Madras  nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------------------------

Author: Vishvesh Sundararaman
Email id: vishu.vivek@gmail.com
Details:
*/

package sdram_template;
import sdram_axi4_cfg::*;
import sdram_axi4_lite_cfg ::*;
`include "sdram.defines"
import device_common::*;

(*synthesize*)
module mkdummy#(Clock sdram_clock, Reset sdram_reset)(Empty);
	let core_clock<-exposeCurrentClock;
	let core_reset<-exposeCurrentReset;
	Ifc_sdram_wrap_axi4#(32,32,32,32,0,32,12,3) sdram1 <- mksdram_wrap_axi4 `ifdef sdram_ext_clk ( sdram_clock,sdram_reset) `endif ;
	Ifc_sdram_wrap_axi4lite#(32,32,32,32,0,32,12,3) sdram2 <- mksdram_wrap_axi4lite `ifdef sdram_ext_clk ( sdram_clock,sdram_reset) `endif ;

    rule rl_ipad;
     sdram1.io.ipad_sdr_din(?);
     sdram2.io.ipad_sdr_din(?);
    endrule
endmodule

endpackage
    
