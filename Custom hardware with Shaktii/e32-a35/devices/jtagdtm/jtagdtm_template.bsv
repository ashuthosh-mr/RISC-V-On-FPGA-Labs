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

Author: Neel Gala
Email id: neelgala@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/
package jtagdtm_template;
  import Vector::*;
  import FIFOF::*;
  import DReg::*;
  import SpecialFIFOs::*;
  import BRAMCore::*;
  import FIFO::*;
  import Clocks::*;

  import jtagdtm::*;
  import rbb_jtag::*;
  (*synthesize*)
  module mkdummy(Empty);

    Clock defaultclk <- exposeCurrentClock;

    MakeClockIfc#(Bit#(1)) tck_clk <-mkUngatedClock(1);
    MakeResetIfc trst <- mkReset(0,False,tck_clk.new_clk);

    CrossingReg#(Bit#(1)) tdi<-mkNullCrossingReg(tck_clk.new_clk,0);
		CrossingReg#(Bit#(1)) tms<-mkNullCrossingReg(tck_clk.new_clk,0);
		CrossingReg#(Bit#(1)) tdo<-mkNullCrossingReg(defaultclk,0,clocked_by tck_clk.new_clk, reset_by trst.new_rst);
		
    Ifc_jtag_driver_sim openocd <- mkRbbJtag();
    Ifc_jtagdtm jtag_tap <- mkjtagdtm(clocked_by tck_clk.new_clk, reset_by trst.new_rst);

    // Connecting rules
    rule rl_join_tdi;
      tdi <= openocd.wire_tdi();
    endrule

    rule rl_join_tms;
      tms <= openocd.wire_tms();
    endrule

    rule rl_join_tdo;
      tdo <= jtag_tap.tdo();
    endrule

    rule rl_join_tck;
      tck_clk.setClockValue(openocd.wire_tck());
    endrule
    
    rule rl_join_trst;
      if(openocd.wire_trst() == 1)
        trst.assertReset();
    endrule

    rule assignment;
      jtag_tap.tms_i(tms.crossed);
      jtag_tap.tdi_i(tdi.crossed);
    endrule

    rule assignmentr;
      openocd.wire_tdo(tdo.crossed);
    endrule

  endmodule
endpackage

