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
Author: P.George
Email id: command.paul@gmail.com
--------------------------------------------------------------------------------------------------
*/

// Template of an Ideal HART for debugging

package hart_template;

  import Connectable::*;
  import Clocks::*;
  import DReg :: * ;

  import riscvDebug013::*;
  import debug_types::*;

  (*synthesize*)
  module mkHartTemplate(Hart_Debug_Ifc);

    Clock curr_clk <- exposeCurrentClock;                                  // current default clock
    Reset curr_reset<-exposeCurrentReset;                                  // current default reset
    MakeResetIfc hart_reset <-mkReset(0,False,curr_clk);          // create a new reset for curr_clk
    Reset derived_reset <- mkResetEither(hart_reset.new_rst,curr_reset);     // OR default and new_rst

    Reg#(Bit#(1)) rg_reset_hart <- mkReg(0);              // Triggers the rule that resets your hart

    Reg#(Bit#(1)) rg_halted <- mkReg(0);                  // 0 : Hart "halted" , 1 hart Running
    Reg#(Bit#(1)) rg_available <- mkReg(1);               // 0 : Hart not Available for debugging

    Reg#(Bit#(1)) rg_halt_request <- mkDReg(0);  // Equvalent Struicture to absorb incoming requests
    Reg#(Bit#(1)) rg_resume_request <- mkDReg(0);// Equvalent Struicture to absorb incoming requests
    
    Reg#(Maybe#(Bit#(DXLEN))) rg_abst_response <- mkReg(tagged Invalid); // registered container for responses

    // No implict conditions hart state at the end of every cycle
    // rule hart_state; 
      // $display($time,"halted %h,available %h,halt_request %b,resume_request %b,reset_request %b",
                // rg_halted,rg_available,rg_halt_request,
                // rg_resume_request,rg_reset_hart);
    // endrule 

    rule run_control;
      if (rg_halt_request == 1)
        rg_halted <= 1;
      else if(rg_resume_request == 1)
        rg_halted <= 0;
      else if (rg_reset_hart == 1)
        rg_halted <= 0;
    endrule

    rule reset_control(rg_reset_hart == 1);
      hart_reset.assertReset();
    endrule

    //   Interface Population   
    method Action   abstractOperation(AbstractRegOp abstract_command)if (!(isValid(rg_abst_response)));
      // Condition that a new request will come in after the previous one has been serviced
      $display($time,"ABC\tAbstract Operation Recieved"); 
      rg_abst_response <= tagged Valid zeroExtend(32'hbebecafe) ;
    endmethod

    method ActionValue#(Bit#(DXLEN)) abstractReadResponse if (isValid(rg_abst_response));
      rg_abst_response <= tagged Invalid;
      $display($time,"ABR\tAbstract Response Enqueued"); 
      return validValue(rg_abst_response);
    endmethod

    method Action   haltRequest(Bit#(1) halt_request);
      rg_halt_request <= halt_request;
    endmethod

    method Action   resumeRequest(Bit#(1) resume_request);
      rg_resume_request <= resume_request;
    endmethod

    method Action   hartReset(Bit#(1) hart_reset_v); // Change to reset type // Signal TO Reset HART -Active HIGH
      rg_reset_hart <= hart_reset_v;
    endmethod

    method Bit#(1)  is_halted;
      return rg_halted;
    endmethod

    method Bit#(1)  is_unavailable;
      return (~rg_available);
    endmethod

    method Bit#(1) has_reset;
      return 1;
    endmethod
  endmodule

endpackage
