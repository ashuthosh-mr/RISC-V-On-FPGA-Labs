/* 
Copyright (c) 2018, IIT Madras All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions
  and the following disclaimer.  
* Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and / or other materials provided 
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

Author : Arjun Menon
Email id : c.arjunmenon@gmail.com
Details: Input Qualification Control module

--------------------------------------------------------------------------------------------------
*/
package iqc;
  import Vector::*;
  interface Ifc_iqc#(numeric type n);
    method ActionValue#(Bit#(n)) qualify(Bit#(n) inputs);
  endinterface

  module mkiqc(Bit#(a) cycles, Ifc_iqc#(n) ifc);
    let val_n= valueOf(n);

    Vector#(n, Reg#(Bit#(a))) rg_qual_counter <- replicateM(mkReg(0));
    Vector#(n, Reg#(Bit#(1))) rg_qualified_signal <- replicateM(mkReg(0));
    Wire#(Bit#(n)) wr_input <-mkWire();

    for(Integer i=0; i<val_n; i=i+1) begin
      rule rl_count;
        if(rg_qualified_signal[i]!=wr_input[i]) begin
          if(rg_qual_counter[i]==cycles) begin
            rg_qualified_signal[i]<= wr_input[i];
            rg_qual_counter[i]<= 0;
          end
          else
            rg_qual_counter[i]<= rg_qual_counter[i] + 1;
        end
        else
          rg_qual_counter[i]<= 0;
      endrule
    end

    method ActionValue#(Bit#(n)) qualify(Bit#(n) inputs);
      wr_input<= inputs;
      if(cycles==0)
        return inputs;
      else
        return unpack(pack(readVReg(rg_qualified_signal)));
    endmethod

  endmodule

  (*synthesize*)
	module mkinst(Ifc_iqc#(6));
    let ifc();
    mkiqc#(4'd3) _temp(ifc);
    return (ifc);
  endmodule

endpackage
