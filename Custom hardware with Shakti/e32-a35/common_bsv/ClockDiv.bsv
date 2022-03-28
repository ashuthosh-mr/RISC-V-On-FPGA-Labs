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
*/
package ClockDiv;
  /*=== Project imports ==*/
  import Clocks::*;
  /*======================*/
  // =========================== Clock divider module ================ //
  interface Ifc_ClockDiv#(numeric type width);
    interface Clock slowclock;
    method Action divisor(Bit#(width) in);
  endinterface

  module mkClockDiv(Ifc_ClockDiv#(width));
    let defclock <- exposeCurrentClock;
    Reg#(Bit#(1)) clk <- mkReg(0);
    Reg#(Bit#(width)) rg_divisor <- mkReg(0);
    Reg#(Bit#(width)) rg_counter <- mkReg(0);
    MakeClockIfc#(Bit#(1)) new_clock <- mkUngatedClock(0);
    MuxClkIfc clock_selector <- mkUngatedClockMux(new_clock.new_clk,defclock);
    Bool clockmux_sel = rg_divisor!=0;
    rule increment_counter;
      if(rg_divisor!=0 && rg_counter >= rg_divisor)begin
        rg_counter <= 0;
        clk <= ~ clk;
      end
      else
        rg_counter <= rg_counter + 1;
    endrule

    rule generate_clock;
      new_clock.setClockValue(clk);
    endrule

    rule select_clock;
      clock_selector.select(clockmux_sel);
    endrule

    method Action divisor(Bit#(width) in);
      rg_divisor <= in != 0 ? in - 1 : 0;
    endmethod

    interface slowclock=clock_selector.clock_out;
  endmodule
  // ============================================================== //
  
endpackage
