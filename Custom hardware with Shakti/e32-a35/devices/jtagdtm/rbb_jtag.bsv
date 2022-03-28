/*
Copyright (c) 2013, IIT Madras
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

*  Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
*  Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
*  Neither the name of IIT Madras  nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
// RBB_Shakti.bsv

package rbb_jtag;
  // This Module and eqv should be reset by the global reset and not by NDM reset !
  
  // // frame = {NA,NA,NA,reset,request_tdo,tck,tms,tdi}

  // import "BDPI" function ActionValue #(Bit #(1)) init_rbb_jtag(Bit#(1) dummy); // Calling this will make the simulator wait for openOCD					
  // import "BDPI" function ActionValue #(Bit #(8))get_frame(Bit#(1) dummy);
  // import "BDPI" function Action send_tdo(Bit #(1) tdo);
  // import "BDPI" function ActionValue #(Bit #(1)) accept_cxn(Bit#(1) dummy);
  
  import "BDPI" function ActionValue #(int) init_rbb_jtag(Bit#(1) dummy);
  import "BDPI" function ActionValue #(Bit #(8))get_frame(int client_fd);
  import "BDPI" function Action send_tdo(Bit #(1) tdo , int client_fd);


  interface Ifc_jtag_driver_sim;
    (*always_enabled,always_ready*)
    method Bit#(1) wire_tck;
    (*always_enabled,always_ready*)
    method Bit#(1) wire_trst;
    (*always_enabled,always_ready*)
    method Bit#(1) wire_tms;
    (*always_enabled,always_ready*)
    method Bit#(1) wire_tdi;
    // Input
    (*always_enabled,always_ready*)
    method Action wire_tdo(Bit#(1)tdo_in);
  endinterface

// Do not add synthesize attribute
module mkRbbJtag(Ifc_jtag_driver_sim);
  
    Reg#(Bit#(1)) tck <- mkReg(0);
    Reg#(Bit#(1)) trst<- mkReg(0);
    Reg#(Bit#(1)) tdi <- mkReg(0);
    Reg#(Bit#(1)) tms <- mkReg(0);
    Reg#(Bit#(1)) tdo <- mkReg(0);

    Reg#(Bit#(1)) rg_initial <- mkReg(0);
    Reg#(Bit#(1)) rg_req_tdo <- mkReg(0);
    Reg#(Bit#(1)) rg_end_sim <- mkReg(0);

    Reg#(int) rg_client_fd <- mkReg(32'hffffffff); // -1

    rule rl_initial(rg_initial == 0);
      let x <- init_rbb_jtag(0);
      if(x != 32'hffffffff)begin
        $display("xval = %h" , x);
        rg_initial <= 1'b1;
        rg_client_fd <= x;
      end
    endrule

    // Design two variants one that blocks and one that does not.
    rule rl_get_frame(rg_initial == 1'b1 && rg_req_tdo == 0);
      let x <- get_frame(rg_client_fd);
      tdi <= x[0];
      tms <= x[1];
      tck <= x[2];
      rg_req_tdo <= x[3];
      trst<= x[4];
    endrule

    rule rl_send_tdo(rg_initial == 1'b1 && rg_req_tdo == 1);
      send_tdo(tdo,rg_client_fd);
      rg_req_tdo <= 0;
    endrule

    rule rl_end_sim(rg_end_sim == 1);
      $finish();
    endrule

    //Interface def.
    method Bit#(1) wire_tck;
      return tck;
    endmethod
    method Bit#(1) wire_trst;
      return trst;
    endmethod
    method Bit#(1) wire_tms;
      return tms;
    endmethod
    method Bit#(1) wire_tdi;
      return tdi;
    endmethod
    method Action wire_tdo(Bit#(1)tdo_in);
      tdo <= tdo_in;
    endmethod
  endmodule
endpackage