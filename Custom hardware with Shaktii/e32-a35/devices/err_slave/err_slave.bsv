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

Author : Neel Gala
Email id : neelgala@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/
package err_slave;
  import Vector::*;
  import FIFOF::*;
  import DReg::*;
  import SpecialFIFOs::*;
  import BRAMCore::*;
  import FIFO::*;

	import AXI4_Types:: *;
	import AXI4_Fabric:: *;
	import AXI4_Lite_Types:: *;
	import AXI4_Lite_Fabric:: *;
  import Semi_FIFOF::*;
  
  typedef enum {Idle, Burst} Mem_State deriving(Eq, Bits, FShow);

  interface Ifc_err_slave_axi4#(numeric type awidth, numeric type dwidth, numeric type uwidth);
    interface AXI4_Slave_IFC#(awidth, dwidth, uwidth) slave;
  endinterface

  module mkerr_slave_axi4(Ifc_err_slave_axi4#(awidth, dwidth, uwidth));
	  AXI4_Slave_Xactor_IFC #(awidth, dwidth, uwidth)  s_xactor <- mkAXI4_Slave_Xactor;
    Reg#(Mem_State) read_state <- mkReg(Idle);
    Reg#(Mem_State) write_state <- mkReg(Idle);
	  Reg#(Bit#(8)) rg_readburst_counter <- mkReg(0);
	  Reg#(Bit#(8)) rg_read_length <- mkReg(0);
    Reg#(Bit#(4)) rg_rd_id <- mkReg(0);
	  Reg#(AXI4_Wr_Resp	#(uwidth)) rg_write_response <- mkReg(?);
    rule receive_read_request(read_state == Idle);
      let ar <- pop_o(s_xactor.o_rd_addr);
      read_state <= Burst;
      rg_rd_id <= ar.arid;
      rg_readburst_counter <= 0;
		  rg_read_length <= ar.arlen;
    endrule

    rule send_error_response(read_state == Burst);
      AXI4_Rd_Data#(data_width, user_width) r = AXI4_Rd_Data {rresp: AXI4_DECERR, rdata: 0 , 
        rlast : rg_readburst_counter == rg_read_length, ruser : 0, rid : rg_rd_id};
      if(rg_readburst_counter == rg_read_length)
        read_state <= Idle;
      else
        rg_readburst_counter <= rg_readburst_counter + 1;

      s_xactor.i_rd_data.enq(r);
    endrule

    rule receive_write_request(write_state == Idle);
      let aw <- pop_o (s_xactor.o_wr_addr);
      let w  <- pop_o (s_xactor.o_wr_data);
	    let b = AXI4_Wr_Resp {bresp : AXI4_DECERR, buser : aw.awuser, bid : w.wid};
      if(!w.wlast)
        write_state <= Burst;
      else
  	  	s_xactor.i_wr_resp.enq (b);
      rg_write_response <= b;
    endrule
    // if the request is a write burst then keeping popping all the data on the data_channel and
    // send a error response on receiving the last data.
    rule write_request_data_channel(write_state == Burst);
      let w  <- pop_o (s_xactor.o_wr_data);
      if(w.wlast)begin
	  	  s_xactor.i_wr_resp.enq (rg_write_response);
        write_state <= Idle;
      end
    endrule
    interface slave = s_xactor.axi_side;
  endmodule
  
  interface Ifc_err_slave_axi4lite#(numeric type awidth, numeric type dwidth, numeric type uwidth);
    interface AXI4_Lite_Slave_IFC#(awidth, dwidth, uwidth) slave;
  endinterface
  
  module mkerr_slave_axi4lite(Ifc_err_slave_axi4lite#(awidth, dwidth, uwidth));
	  AXI4_Lite_Slave_Xactor_IFC #(awidth, dwidth, uwidth)  s_xactor <- mkAXI4_Lite_Slave_Xactor;
    rule receive_read_request;
      let ar<-pop_o(s_xactor.o_rd_addr);
      AXI4_Lite_Rd_Data#(data_width, user_width) r = AXI4_Lite_Rd_Data {rresp: AXI4_LITE_DECERR, rdata:0 , 
        ruser: 0};
      s_xactor.i_rd_data.enq(r);
    endrule

    rule receive_write_request;
      let aw <- pop_o (s_xactor.o_wr_addr);
      let w  <- pop_o (s_xactor.o_wr_data);
	    let b = AXI4_Lite_Wr_Resp {bresp: AXI4_LITE_DECERR, buser: aw.awuser};
  	 	s_xactor.i_wr_resp.enq (b);
    endrule
    interface slave = s_xactor.axi_side;
  endmodule
endpackage

