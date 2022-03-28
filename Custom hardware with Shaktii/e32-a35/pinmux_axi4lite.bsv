/*
Copyright (c) 2019, IIT Madras All rights reserved.

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
package pinmux_axi4lite;

	import AXI4_Lite_Types::*;
	import AXI4_Lite_Fabric::*;
	import Semi_FIFOF::*;
	import GetPut::*;
	import FIFO::*;
	import Clocks::*;
	import BUtils::*;
	import pinmux::*;
  import device_common::*;
  `include "Soc.defines"


  export Ifc_pinmux_axi4lite (..);
  export UserInterface     (..);
  export mkpinmux_axi4lite;
  export mkpinmux_user;

	interface UserInterface#(numeric type addr_width ,numeric type data_width);
		method ActionValue#(Tuple2#(Bit#(data_width),Bool)) read_req (Bit#(addr_width) addr,
		                                                              Bit#(2) size);
		method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data, Bit#(2) size);
    		interface IOCellSide pinmuxuser_iocell_side;
    		interface PeripheralSide pinmuxuser_peripheral_side;
	endinterface

	module mkpinmux_user (UserInterface#(addr_width,data_width))
      provisos(Mul#(16, a__, data_width),
              Add#(d__, 8, data_width),
              Mul#(8, b__, data_width),
              Mul#(4, f__, data_width),
              Add#(c__, 16, data_width),
	      Mul#(32, e__, data_width)
              );

		Reg#(Bit#(32)) pinmux_muxsel_config <-mkRegA(0);
		Ifc_pinmux pinmux <-mkpinmux();

		rule rl_program_pinmux_mux_lines;
			pinmux.mux_lines.cell7_mux(pinmux_muxsel_config[1:0]);
			pinmux.mux_lines.cell8_mux(pinmux_muxsel_config[3:2]);
			pinmux.mux_lines.cell9_mux(pinmux_muxsel_config[5:4]);
			pinmux.mux_lines.cell10_mux(pinmux_muxsel_config[7:6]);
			pinmux.mux_lines.cell12_mux(pinmux_muxsel_config[9:8]);
			pinmux.mux_lines.cell13_mux(pinmux_muxsel_config[11:10]);
			pinmux.mux_lines.cell16_mux(pinmux_muxsel_config[13:12]);
			pinmux.mux_lines.cell17_mux(pinmux_muxsel_config[15:14]);
			pinmux.mux_lines.cell18_mux(pinmux_muxsel_config[17:16]);
			pinmux.mux_lines.cell19_mux(pinmux_muxsel_config[19:18]);
			pinmux.mux_lines.cell20_mux(pinmux_muxsel_config[21:20]);
		endrule

		method ActionValue#(Tuple2#(Bit#(data_width),Bool)) read_req (Bit#(addr_width) addr, 
		                                                              Bit#(2) size);
      			if((addr ==`PinmuxConfigReg) && (size[1:0]==2'b00)) begin
				Bit#(8) data = pinmux_muxsel_config[7:0];
        			return tuple2(duplicate(data),True);
      			end
			else if( addr ==`PinmuxConfigReg && size[1:0]==2'b01) begin
				Bit#(16) data = pinmux_muxsel_config[15:0];
				return tuple2(duplicate(data),True);
			end
			else if(addr ==`PinmuxConfigReg && size[1:0]==2'b10 ) begin
				Bit#(32) data = pinmux_muxsel_config[31:0];
				return tuple2(duplicate(data),True);
			end
			else
				return tuple2(?,False);
		endmethod

		method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data, Bit#(2) size);
      			if( addr ==`PinmuxConfigReg && size[1:0]==2'b00)begin
				pinmux_muxsel_config[7:0] <= data[7:0];
				return True;
			end
      			else if( addr ==`PinmuxConfigReg && size[1:0]==2'b01)begin
				pinmux_muxsel_config[15:0] <= data[15:0];
				return True;
			end
      			else if( addr ==`PinmuxConfigReg && size[1:0]==2'b10)begin
				pinmux_muxsel_config[31:0] <= data[31:0];
				return True;
			end
			else
				return False;
		endmethod

	  interface pinmuxuser_iocell_side=pinmux.iocell_side;
	  interface pinmuxuser_peripheral_side=pinmux.peripheral_side;
	endmodule:mkpinmux_user

	interface Ifc_pinmux_axi4lite#(numeric type addr_width,
                               numeric type data_width,
                               numeric type user_width
                               );
		interface AXI4_Lite_Slave_IFC#(addr_width, data_width, user_width) slave;
    		interface IOCellSide pinmuxaxi4lite_iocell_side;
    		interface PeripheralSide pinmuxaxi4lite_peripheral_side;
  	endinterface

	module mkpinmux_axi4lite (Ifc_pinmux_axi4lite#(addr_width,data_width,user_width))
	// same provisos for the pinmux
    provisos(Mul#(16, a__, data_width),
              Add#(d__, 8, data_width),
              Mul#(8, b__, data_width),
              Mul#(4, f__, data_width),
              Add#(c__, 16, data_width),
	      Mul#(32, e__, data_width)
              );

		  AXI4_Lite_Slave_Xactor_IFC#(addr_width,data_width,user_width)  s_xactor <- 
		                                                                    mkAXI4_Lite_Slave_Xactor();
		  UserInterface#(addr_width,data_width) user_ifc<- mkpinmux_user();

			//capturing the read requests
			rule capture_read_request;
				let rd_req <- pop_o (s_xactor.o_rd_addr);
				let {rdata,succ} <- user_ifc.read_req(rd_req.araddr,unpack(rd_req.arsize));
				let lv_resp= AXI4_Lite_Rd_Data {rresp:succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR,
      	                                                      rdata: rdata, ruser: ?}; //TODO user?
				s_xactor.i_rd_data.enq(lv_resp);//sending back the response
			endrule

			// capturing write requests
			rule capture_write_request;
				let wr_req  <- pop_o(s_xactor.o_wr_addr);
				let wr_data <- pop_o(s_xactor.o_wr_data);
				let succ <- user_ifc.write_req(wr_req.awaddr,wr_data.wdata,unpack(wr_req.awsize));
      	let lv_resp = AXI4_Lite_Wr_Resp {bresp: succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, buser: ?};
      	s_xactor.i_wr_resp.enq(lv_resp);//enqueuing the write response
			endrule

			interface slave = s_xactor.axi_side;
			interface pinmuxaxi4lite_iocell_side= user_ifc.pinmuxuser_iocell_side;
			interface pinmuxaxi4lite_peripheral_side= user_ifc.pinmuxuser_peripheral_side;
	endmodule:mkpinmux_axi4lite

endpackage:pinmux_axi4lite
