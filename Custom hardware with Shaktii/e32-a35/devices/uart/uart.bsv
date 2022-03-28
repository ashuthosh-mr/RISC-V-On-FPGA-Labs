/* 
Copyright (c) 2018, IIT Madras All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions
  and the following disclaimer.  
* Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and/or other materials provided 
 with the distribution.  
* Neither the name of IIT Madras nor the names of its contributors may be used to endorse or 
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
package uart;
	  `include "Logger.bsv"       // for logging display statements.
	`include "uart.defines"

	import AXI4_Lite_Types::*;
	import AXI4_Lite_Fabric::*;
	import AXI4_Types::*;
	import AXI4_Fabric::*;
	import Semi_FIFOF::*;
	import RS232_modified::*;
	import GetPut::*;
	import FIFO::*;
	import Clocks::*;
	import BUtils::*;
  import device_common::*;

  export RS232             (..);
  export Ifc_uart_axi4lite (..);
  export Ifc_uart_axi4     (..);
  export UserInterface     (..);
  export mkuart_axi4lite;
  export mkuart_axi4;
  export mkuart_user;

	interface UserInterface#(numeric type addr_width ,numeric type data_width,  numeric type depth);
		method ActionValue#(Tuple2#(Bit#(data_width),Bool)) read_req (Bit#(addr_width) addr, 
																									AccessSize size);
		method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data, 
																									AccessSize size);
    	interface RS232 io;
	endinterface

	module mkuart_user#(parameter Bit#(16) baudrate)
      (UserInterface#(addr_width,data_width, depth))
      provisos(
        Add#(a__, 8, data_width),
        Add#(b__, 16, data_width),
        Mul#(16, c__, data_width),
        Mul#(8, d__, data_width),
        Add#(2, e__, depth)
      );

		Reg#(Bit#(16)) baud_value <-mkReg(baudrate);
		UART#(depth) uart <-mkUART(8,NONE,STOP_1,baud_value); // charasize,Parity,Stop Bits,BaudDIV
    Wire#(Bit#(4)) wr_status <- mkWire();
    rule capture_status;
      let lv_status= {pack(uart.receiver_not_empty), pack(uart.receiver_not_full), 
                                  pack(uart.transmittor_not_full), pack(uart.transmission_done)};
      wr_status<=lv_status;
    endrule

		method ActionValue#(Tuple2#(Bit#(data_width),Bool)) read_req (Bit#(addr_width) addr, 
																									AccessSize size);
      if( addr[3:0]==`StatusReg)begin
        return tuple2(duplicate({4'd0,wr_status}),True);
      end
			else if( addr[3:0]==`RxReg)begin
				Bit#(8) data =0;
				if(uart.receiver_not_empty)
				  data<-uart.tx.get; 
				return tuple2(duplicate(data),True);
			end
			else if(addr[3:0]==`BaudReg) begin
				return tuple2(duplicate(baud_value),True);
			end
			else
				return tuple2(?,False);
		endmethod

		method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data, 
																									AccessSize size);
			if(addr[3:0]==`TxReg)begin
				uart.rx.put(truncate(data));//putting write data in the UART
				return True;
			end
			else if(addr[3:0]==`BaudReg) begin
				baud_value<=truncate(data);
				return True;
			end
			else
				return False;
		endmethod

	  interface io=uart.rs232;	
	endmodule:mkuart_user

	interface Ifc_uart_axi4lite#(numeric type addr_width, 
                               numeric type data_width, 
                               numeric type user_width, 
                               numeric type depth);
		interface AXI4_Lite_Slave_IFC#(addr_width, data_width, user_width) slave; 
	   interface RS232 io;
  	endinterface

	module mkuart_axi4lite#(Clock uart_clock, Reset uart_reset, parameter Bit#(16) baudrate)
																			(Ifc_uart_axi4lite#(addr_width,data_width,user_width, depth))
	// same provisos for the uart
      provisos(
        Add#(a__, 8, data_width),
        Add#(b__, 16, data_width),
        Mul#(16, c__, data_width),
        Mul#(8, d__, data_width),
        Add#(2, e__, depth)
        );

		
		Clock core_clock<-exposeCurrentClock;
		Reset core_reset<-exposeCurrentReset;
		Bool sync_required=(core_clock!=uart_clock);
		AXI4_Lite_Slave_Xactor_IFC#(addr_width,data_width,user_width)  s_xactor <- mkAXI4_Lite_Slave_Xactor();

		if(!sync_required)begin // If uart is clocked by core-clock.
			UserInterface#(addr_width,data_width, depth) user_ifc<- mkuart_user(clocked_by uart_clock, 
                                                                    reset_by uart_reset, baudrate);
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
			interface io= user_ifc.io;
		end
		else begin // if core clock and uart_clock is different.
			UserInterface#(addr_width,data_width, depth) user_ifc<- mkuart_user(clocked_by uart_clock, 
                                                                    reset_by uart_reset, baudrate);
			SyncFIFOIfc#(AXI4_Lite_Rd_Addr#(addr_width,user_width)) ff_rd_request <- 
																							mkSyncFIFOFromCC(3,uart_clock);
			SyncFIFOIfc#(AXI4_Lite_Wr_Addr#(addr_width,user_width)) ff_wr_request <- 
																							mkSyncFIFOFromCC(3,uart_clock);
			SyncFIFOIfc#(AXI4_Lite_Wr_Data#(data_width)) ff_wdata_request <- mkSyncFIFOFromCC(3,uart_clock);
			SyncFIFOIfc#(AXI4_Lite_Rd_Data#(data_width,user_width)) ff_rd_response <- 
																				mkSyncFIFOToCC(3,uart_clock,uart_reset);
			SyncFIFOIfc#(AXI4_Lite_Wr_Resp#(user_width)) ff_wr_response <- 
																				mkSyncFIFOToCC(3,uart_clock,uart_reset);
			//capturing the read requests
			rule capture_read_request;
				let rd_req <- pop_o (s_xactor.o_rd_addr);
				ff_rd_request.enq(rd_req);
			endrule

			rule perform_read;
				let rd_req = ff_rd_request.first;
				ff_rd_request.deq;
				let {rdata,succ} <- user_ifc.read_req(rd_req.araddr,unpack(rd_req.arsize));
				let lv_resp= AXI4_Lite_Rd_Data {rresp:succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, 
      	                                                      rdata: rdata, ruser: ?}; //TODO user?
				ff_rd_response.enq(lv_resp);
			endrule

			rule send_read_response;
				ff_rd_response.deq;
				s_xactor.i_rd_data.enq(ff_rd_response.first);//sending back the response
			endrule              
	
			// capturing write requests
			rule capture_write_request;
				let wr_req  <- pop_o(s_xactor.o_wr_addr);
				let wr_data <- pop_o(s_xactor.o_wr_data);
				ff_wr_request.enq(wr_req);
				ff_wdata_request.enq(wr_data);
			endrule

			rule perform_write;
				let wr_req  = ff_wr_request.first;
				let wr_data = ff_wdata_request.first;
				let succ <- user_ifc.write_req(wr_req.awaddr,wr_data.wdata,unpack(wr_req.awsize));
      		let lv_resp = AXI4_Lite_Wr_Resp {bresp: succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, buser: ?};
				ff_wr_response.enq(lv_resp);
			endrule

			rule send_write_response;
				ff_wr_response.deq;
      		s_xactor.i_wr_resp.enq(ff_wr_response.first);//enqueuing the write response
			endrule
			interface slave = s_xactor.axi_side;
			interface io= user_ifc.io;
		end
	endmodule:mkuart_axi4lite



 	interface Ifc_uart_axi4#(numeric type addr_width, 
                           numeric type data_width, 
                           numeric type user_width, 
                           numeric type depth);
		(*prefix=""*) interface AXI4_Slave_IFC#(addr_width, data_width, user_width) slave;
		(*prefix=""*) interface RS232 io;
 	endinterface

	module mkuart_axi4#(Clock uart_clock, Reset uart_reset,  parameter Bit#(16) baudrate)
                                          (Ifc_uart_axi4#(addr_width,data_width,user_width, depth))
	// same provisos for the uart
      provisos(
        Add#(a__, 8, data_width),
        Add#(b__, 16, data_width),
        Mul#(16, c__, data_width),
        Mul#(8, d__, data_width),
        Add#(2, e__, depth)
        );
		Clock core_clock<-exposeCurrentClock;
		Reset core_reset<-exposeCurrentReset;
		Bool sync_required=(core_clock!=uart_clock);
		AXI4_Slave_Xactor_IFC #(addr_width,data_width,user_width)  s_xactor <- mkAXI4_Slave_Xactor();
		Reg#(Bit#(8)) rg_rdburst_count <- mkReg(0, clocked_by uart_clock, reset_by uart_reset);
		Reg#(Bit#(8)) rg_wrburst_count <- mkReg(0, clocked_by uart_clock, reset_by uart_reset);

		if(!sync_required)begin // If uart is clocked by core-clock.
			UserInterface#(addr_width,data_width, depth) user_ifc<- mkuart_user(clocked_by uart_clock, 
                                                                    reset_by uart_reset, baudrate);
		  Reg#(AXI4_Rd_Addr#(addr_width,user_width)) rg_rdpacket <- mkReg(?);
  		Reg#(AXI4_Wr_Addr#(addr_width,user_width)) rg_wrpacket <- mkReg(?);
			//capturing the read requests
			rule capture_read_request(rg_rdburst_count==0);
				let rd_req <- pop_o (s_xactor.o_rd_addr);
				let {rdata,succ} <- user_ifc.read_req(rd_req.araddr,unpack(truncate(rd_req.arsize)));
				rg_rdpacket<=rd_req;	
				if(rd_req.arlen!=0)
					rg_rdburst_count<=1;
				let lv_resp= AXI4_Rd_Data {rresp:succ?AXI4_OKAY:AXI4_SLVERR, rid:rd_req.arid, 
												rlast:(rd_req.arlen==0), rdata: rdata, ruser: ?}; //TODO user?
				s_xactor.i_rd_data.enq(lv_resp);//sending back the response
			endrule             

			rule burst_reads(rg_rdburst_count!=0);
				let rd_req=rg_rdpacket;
				let {rdata,succ} <- user_ifc.read_req(rd_req.araddr,unpack(truncate(rd_req.arsize)));
				if(rd_req.araddr[3:0]!=`RxReg || truncate(rd_req.arsize)!=pack(Byte) 
															|| rd_req.arburst!=00 /*FIXED*/)begin
					succ=False;
				end
				if(rg_rdburst_count==rd_req.arlen)
					rg_rdburst_count<=0;
				else
					rg_rdburst_count<=rg_rdburst_count+1;
				let lv_resp= AXI4_Rd_Data {rresp:succ?AXI4_OKAY:AXI4_SLVERR, rid:rd_req.arid, 
							rlast:(rd_req.arlen==rg_rdburst_count), rdata: rdata, ruser: ?}; //TODO user?
				s_xactor.i_rd_data.enq(lv_resp);//sending back the response
			endrule
	
			// capturing write requests
			rule capture_write_request(rg_wrburst_count==0);
				let wr_req  <- pop_o(s_xactor.o_wr_addr);
				let wr_data <- pop_o(s_xactor.o_wr_data);
				let succ <- user_ifc.write_req(wr_req.awaddr,wr_data.wdata,
																						unpack(truncate(wr_req.awsize)));
				rg_wrpacket<=wr_req;	
				if(wr_req.awlen!=0)
					rg_wrburst_count<=1;
      		let lv_resp = AXI4_Wr_Resp {bresp: succ?AXI4_OKAY:AXI4_SLVERR, buser: ?, bid:wr_data.wid};
				if(wr_data.wlast)
	      		s_xactor.i_wr_resp.enq(lv_resp);//enqueuing the write response
			endrule
			rule burst_writes(rg_wrburst_count!=0);
				let wr_req=rg_wrpacket;
				let wr_data <- pop_o(s_xactor.o_wr_data);
				let succ <- user_ifc.write_req(wr_req.awaddr,wr_data.wdata,
																						unpack(truncate(wr_req.awsize)));
				if(wr_req.awaddr[3:0]!=`TxReg || truncate(wr_req.awsize)!=pack(Byte) 
															|| wr_req.awburst!=00 /*FIXED*/)begin
					succ=False;
				end
				if(rg_wrburst_count==wr_req.awlen)
					rg_wrburst_count<=0;
				else
					rg_wrburst_count<=rg_wrburst_count+1;
      		let lv_resp = AXI4_Wr_Resp {bresp: succ?AXI4_OKAY:AXI4_SLVERR, buser: ?, bid:wr_data.wid};
				if(wr_data.wlast)
	      		s_xactor.i_wr_resp.enq(lv_resp);//enqueuing the write response
			endrule
			interface slave = s_xactor.axi_side;
			interface io= user_ifc.io;
		end
		else begin // if core clock and uart_clock is different.
			UserInterface#(addr_width,data_width, depth) user_ifc<- mkuart_user(clocked_by uart_clock, 
                                                                    reset_by uart_reset, baudrate);
			SyncFIFOIfc#(AXI4_Rd_Addr#(addr_width,user_width)) ff_rd_request <- 
														                      									mkSyncFIFOFromCC(3,uart_clock);
			SyncFIFOIfc#(AXI4_Wr_Addr#(addr_width,user_width)) ff_wr_request <- 
																							                      mkSyncFIFOFromCC(3,uart_clock);
			SyncFIFOIfc#(AXI4_Wr_Data#(data_width)) ff_wdata_request <- mkSyncFIFOFromCC(3,uart_clock);
			SyncFIFOIfc#(AXI4_Rd_Data#(data_width,user_width)) ff_rd_response <- 
																				                  mkSyncFIFOToCC(3,uart_clock,uart_reset);
			SyncFIFOIfc#(AXI4_Wr_Resp#(user_width)) ff_wr_response <- 
																				                  mkSyncFIFOToCC(3,uart_clock,uart_reset);

			//capturing the read requests
			rule capture_read_request;
				let rd_req <- pop_o (s_xactor.o_rd_addr);
				ff_rd_request.enq(rd_req);
			endrule

			rule perform_read(rg_rdburst_count==0);
				let rd_req = ff_rd_request.first;
        if(rd_req.arlen!=0)
          rg_rdburst_count<=1;
        else
				  ff_rd_request.deq;
				let {rdata,succ} <- user_ifc.read_req(rd_req.araddr,unpack(truncate(rd_req.arsize)));
				let lv_resp= AXI4_Rd_Data {rresp:succ?AXI4_OKAY:AXI4_SLVERR, rid:rd_req.arid, 
                      rlast:(rg_rdburst_count==rd_req.arlen), rdata: rdata, ruser: ?}; //TODO user?
				ff_rd_response.enq(lv_resp);
			endrule

      rule perform_read_burst(rg_rdburst_count!=0);
				let rd_req = ff_rd_request.first;
				let {rdata,succ} <- user_ifc.read_req(rd_req.araddr,unpack(truncate(rd_req.arsize)));
				if(rd_req.araddr[3:0]!=`RxReg || truncate(rd_req.arsize)!=pack(HWord) 
															|| rd_req.arburst!=00 /*FIXED*/)begin
					succ=False;
				end
				if(rg_rdburst_count==rd_req.arlen)begin
					rg_rdburst_count<=0;
          ff_rd_request.deq;
        end
				else
					rg_rdburst_count<=rg_rdburst_count+1;
				let lv_resp= AXI4_Rd_Data {rresp:succ?AXI4_OKAY:AXI4_SLVERR, rid:rd_req.arid, 
							rlast:(rd_req.arlen==rg_rdburst_count), rdata: rdata, ruser: ?}; //TODO user?
				ff_rd_response.enq(lv_resp);//sending back the response
      endrule

			rule send_read_response;
				ff_rd_response.deq;
				s_xactor.i_rd_data.enq(ff_rd_response.first);//sending back the response
			endrule              
	
			// capturing write requests
			rule capture_writeaddr_request;
				let wr_req  <- pop_o(s_xactor.o_wr_addr);
				ff_wr_request.enq(wr_req);
			endrule
			
      rule capture_writedata_request;
				let wr_data <- pop_o(s_xactor.o_wr_data);
				ff_wdata_request.enq(wr_data);
			endrule

			rule perform_write(rg_wrburst_count==0);
				let wr_req  = ff_wr_request.first;
				let wr_data = ff_wdata_request.first;
        if( wr_req.awlen!=0)
          rg_wrburst_count<=1;
        else 
          ff_wr_request.deq;
  
        ff_wdata_request.deq;
				let succ <- user_ifc.write_req(wr_req.awaddr,wr_data.wdata,
																						unpack(truncate(wr_req.awsize)));
     		let lv_resp = AXI4_Wr_Resp {bresp: succ?AXI4_OKAY:AXI4_SLVERR, buser: ?,bid:wr_data.wid};
        if(wr_data.wlast)
  				ff_wr_response.enq(lv_resp);
			endrule
			
      rule perform_burst_writes(rg_wrburst_count!=0);
				let wr_req=ff_wr_request.first;
				let wr_data =ff_wdata_request.first;
				let succ <- user_ifc.write_req(wr_req.awaddr,wr_data.wdata,
																						unpack(truncate(wr_req.awsize)));
				if(wr_req.awaddr[3:0]!=`TxReg || truncate(wr_req.awsize)!=pack(HWord) 
															|| wr_req.awburst!=00 /*FIXED*/)begin
					succ=False;
				end
				if(rg_wrburst_count==wr_req.awlen)begin
					rg_wrburst_count<=0;
          ff_wr_request.deq;
        end
				else
					rg_wrburst_count<=rg_wrburst_count+1;
      	let lv_resp = AXI4_Wr_Resp {bresp: succ?AXI4_OKAY:AXI4_SLVERR, buser: ?, bid:wr_data.wid};
				if(wr_data.wlast)
				  ff_wr_response.enq(lv_resp);
			endrule

			rule send_write_response;
				ff_wr_response.deq;
      		s_xactor.i_wr_resp.enq(ff_wr_response.first);//enqueuing the write response
			endrule
			interface slave = s_xactor.axi_side;
			interface io= user_ifc.io;
		end
	endmodule:mkuart_axi4
	
endpackage:uart
