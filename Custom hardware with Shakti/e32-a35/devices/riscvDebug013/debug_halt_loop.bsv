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
Details: This module holds the self loop for halt state
--------------------------------------------------------------------------------------------------
*/
package debug_halt_loop;
  import Vector::*;
  import FIFOF::*;
  import DReg::*;
  import SpecialFIFOs::*;
  import BRAMCore::*;
  import FIFO::*;

  import AXI4_Types::*;
  import AXI4_Fabric::*;
  import AXI4_Lite_Types::*;
  import AXI4_Lite_Fabric::*;
  import Semi_FIFOF::*;
  import BUtils::*;

  interface Ifc_debug_halt_loop_axi4#(numeric type awidth, 
                                numeric type dwidth, 
                                numeric type uwidth);
    interface AXI4_Slave_IFC#(awidth, dwidth, uwidth) slave;
  endinterface

  module mkdebug_halt_loop_axi4(Ifc_debug_halt_loop_axi4#(awidth, dwidth, uwidth))
    provisos(Add#(a__, dwidth, 128),
             Mul#(32, b__, dwidth),
             Mul#(16, c__, dwidth),
             Mul#(8, d__, dwidth));
    AXI4_Slave_Xactor_IFC#(awidth, dwidth, uwidth) s_xactor <- mkAXI4_Slave_Xactor;
    Reg#(Bit#(32)) instr_array [4];
    instr_array[0] <- mkReg('h0000100f); // fence.i
    instr_array[1] <- mkReg('h00000013); // nop
    instr_array[2] <- mkReg('hffdff06f); // j pc -4
    instr_array[3] <- mkReg('h0000006f); // self-loop

    rule recieve_read;
      let req <- pop_o(s_xactor.o_rd_addr);

      AXI4_Rd_Data#(dwidth, uwidth) resp = AXI4_Rd_Data {rresp : AXI4_OKAY, rdata: ? , 
        rlast : True, ruser : req.aruser, rid : req.arid};

      Bit#(128) line = {instr_array[3], instr_array[2], instr_array[1], instr_array[0]};
      line = line >> {req.araddr[3:0],3'b0};

      if(req.arlen != 0)
        resp.rresp = AXI4_SLVERR;

      Bit#(dwidth) data = truncate(line);
      case (req.arsize) 
        'd0: data = duplicate(data[7:0]); // byte access
        'd1: data = duplicate(data[15:0]); // half-word access
        'd2: data = duplicate(data[31:0]); // word access
        default: data = data;
      endcase

      resp.rdata = data;

      s_xactor.i_rd_data.enq(resp);
    endrule

    rule receive_write;
      let aw <- pop_o(s_xactor.o_wr_addr);
      let w <- pop_o(s_xactor.o_wr_data);
	    let b = AXI4_Wr_Resp {bresp : AXI4_SLVERR, buser : aw.awuser, bid : w.wid};
  	  s_xactor.i_wr_resp.enq (b);
    endrule

    interface slave = s_xactor.axi_side;
  endmodule
  
  interface Ifc_debug_halt_loop_axi4lite#(numeric type awidth, 
                                numeric type dwidth, 
                                numeric type uwidth);
    interface AXI4_Lite_Slave_IFC#(awidth, dwidth, uwidth) slave;
  endinterface

  module mkdebug_halt_loop_axi4lite(Ifc_debug_halt_loop_axi4lite#(awidth, dwidth, uwidth))
    provisos(Add#(a__, dwidth, 128),
             Mul#(32, b__, dwidth),
             Mul#(16, c__, dwidth),
             Mul#(8, d__, dwidth));
    AXI4_Lite_Slave_Xactor_IFC#(awidth, dwidth, uwidth) s_xactor <- mkAXI4_Lite_Slave_Xactor;
    Reg#(Bit#(32)) instr_array [4];
    instr_array[0] <- mkReg('h0000100f); // fence.i
    instr_array[1] <- mkReg('h00000013); // nop
    instr_array[2] <- mkReg('hffdff06f); // j pc -4
    instr_array[3] <- mkReg('h0000006f); // self-loop

    rule recieve_read;
      let req <- pop_o(s_xactor.o_rd_addr);

      AXI4_Lite_Rd_Data#(dwidth, uwidth) resp = AXI4_Lite_Rd_Data {rresp : AXI4_LITE_OKAY, rdata: ? , 
        ruser : req.aruser};

      Bit#(128) line = {instr_array[3], instr_array[2], instr_array[1], instr_array[0]};
      line = line >> {req.araddr[3:0],3'b0};

      Bit#(dwidth) data = truncate(line);
      case (req.arsize) 
        'd0: data = duplicate(data[7:0]); // byte access
        'd1: data = duplicate(data[15:0]); // half-word access
        'd2: data = duplicate(data[31:0]); // word access
        default: data = data;
      endcase

      resp.rdata = data;

      s_xactor.i_rd_data.enq(resp);
    endrule

    rule receive_write;
      let aw <- pop_o(s_xactor.o_wr_addr);
      let w <- pop_o(s_xactor.o_wr_data);
	    let b = AXI4_Lite_Wr_Resp {bresp : AXI4_LITE_SLVERR, buser : aw.awuser};
  	  s_xactor.i_wr_resp.enq (b);
    endrule

    interface slave = s_xactor.axi_side;
  endmodule
endpackage

