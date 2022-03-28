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
package AXI4Lite_AXI4_Bridge;
	/*=== Project imports ====*/
	import AXI4_Lite_Fabric::*;
	import AXI4_Lite_Types::*;
	import AXI4_Fabric::*;
	import AXI4_Types ::*;
	import Semi_FIFOF	::*;
	import FIFO::*;
	import common_types::*;
	import device_common::*;
	`include "common_params.bsv"
  `include "Logger.bsv"
	/*======================*/
	/*=== Package imports ===*/
	import Clocks::*;
	/*=======================*/

	interface Ifc_AXI4Lite_AXI4_Bridge#(numeric type addr_width, numeric type data_width, numeric type user_width);
		interface AXI4_Slave_IFC#(addr_width, data_width, user_width) axi_slave;
		interface AXI4_Lite_Master_IFC#(addr_width, data_width, user_width) axi4_lite_master;
	endinterface

	typedef enum {RegularReq,BurstReq} BridgeState deriving (Bits,Eq,FShow);

	module mkAXI4Lite_AXI4_Bridge#(Clock fast_clock, Reset fast_reset)(Ifc_AXI4Lite_AXI4_Bridge#(addr_width, data_width, user_width));
		AXI4_Slave_Xactor_IFC #(addr_width, data_width, user_width)  s_xactor <- mkAXI4_Slave_Xactor(clocked_by fast_clock, reset_by fast_reset);
		AXI4_Lite_Master_Xactor_IFC #(addr_width,data_width,user_width) m_xactor <- mkAXI4_Lite_Master_Xactor;
		Reg#(BridgeState) rd_state <-mkReg(RegularReq,clocked_by fast_clock, reset_by fast_reset);
		Reg#(BridgeState) wr_state <-mkReg(RegularReq,clocked_by fast_clock, reset_by fast_reset);
		Reg#(Bit#(4)) rd_id<-mkReg(0);
		Reg#(Bit#(4)) wr_id<-mkReg(0);
		Reg#(Bit#(8)) request_counter<-mkReg(0,clocked_by fast_clock, reset_by fast_reset);
		Reg#(Bit#(8)) response_counter<-mkReg(0);
		Reg#(AXI4_Rd_Addr	#(addr_width,user_width)) rg_read_packet <-mkReg(?,clocked_by fast_clock , reset_by fast_reset);
		Reg#(AXI4_Wr_Addr	#(addr_width,user_width)) rg_write_packet<-mkReg(?,clocked_by fast_clock , reset_by fast_reset);

    let curr_clk<-exposeCurrentClock;
		/*=== FIFOs to synchronize data between the two clock domains ====*/
		//if(fast_clock==curr_clk) begin
			FIFO#(AXI4_Rd_Addr#(addr_width,user_width)) 				ff_rd_addr<- mkSizedFIFO(1);
			FIFO#(AXI4_Wr_Addr#(addr_width, user_width)) 				ff_wr_addr<- mkSizedFIFO(1);
			FIFO#(AXI4_Wr_Data#(data_width)) 										ff_wr_data<- mkSizedFIFO(1);
			FIFO#(AXI4_Rd_Data#(data_width,user_width)) 				ff_rd_resp<- mkSizedFIFO(1);
			FIFO#(AXI4_Wr_Resp#(user_width)) 										ff_wr_resp<- mkSizedFIFO(1);
			Reg#(Bit#(8)) rdburst_value <-mkReg(0);
		//end
		//else begin
		//	SyncFIFOIfc#(AXI4_Rd_Addr	#(addr_width,user_width))		ff_rd_addr <-	mkSyncFIFOToCC(1,fast_clock,fast_reset);
		//	SyncFIFOIfc#(AXI4_Wr_Addr	#(addr_width, user_width))	ff_wr_addr <-	mkSyncFIFOToCC(1,fast_clock,fast_reset);
		//	SyncFIFOIfc#(AXI4_Wr_Data	#(data_width))							ff_wr_data <-	mkSyncFIFOToCC(1,fast_clock,fast_reset);

		//	SyncFIFOIfc#(AXI4_Rd_Data	#(data_width,user_width))		ff_rd_resp <-	mkSyncFIFOFromCC(1,fast_clock);
		//	SyncFIFOIfc#(AXI4_Wr_Resp	#(user_width))							ff_wr_resp <-	mkSyncFIFOFromCC(1,fast_clock);
		//	Reg#(Bit#(8)) rdburst_value <-mkSyncRegToCC(0,fast_clock,fast_reset);
		//end
		/*=================================================================*/


		// These rule will receive the read request from the AXI4 fabric and pass it on to the AXI4Lite fabric.
		// If the request is a burst then they are broken down to individual axi4lite read requests. These
		// are carried out in the next rule. 
		rule capture_read_requests_from_Axi4(rd_state==RegularReq);
			let request<-pop_o(s_xactor.o_rd_addr);
			ff_rd_addr.enq(request);
			rg_read_packet<=request;
			rdburst_value<=request.arlen;
			if(request.arlen!=0) begin
				rd_state<=BurstReq;
			end
		endrule
		// In case a read-burst request is received on the fast bus, then the bursts have to broken down into
		// individual slow-bus read requests. 
		// This is rule is fired after the first read-burst request is sent to the slow_bus. This rule will continue to 
		// fire as long as the slow bus has capacity to receive a new request and the burst is not complete.
		// the difference between the each individual requests on the slow bus is only the address. All other 
		// parameters remain the same.
		rule generate_bust_read_requests(rd_state==BurstReq);
			let request=rg_read_packet;
			request.araddr= axi4burst_addrgen(request.arlen, request.arsize, request.arburst,request.araddr);
			rg_read_packet<=request;
			ff_rd_addr.enq(request);
			if(request.arlen==request_counter)begin
				rd_state<=RegularReq;
				request_counter<=0;
			end
			else
				request_counter<=request_counter+1;
		endrule
		rule send_read_request_on_slow_bus;
			let request=ff_rd_addr.first;
			ff_rd_addr.deq;
		 	let lite_request = AXI4_Lite_Rd_Addr {araddr: request.araddr, arsize:truncate(request.arsize),aruser: request.aruser, arprot: request.arprot}; // arburst: 00-FIXED 01-INCR 10-WRAP
   	   m_xactor.i_rd_addr.enq(lite_request);	
			rd_id<=request.arid;
		endrule
		// This rule will capture the write request from the AXI4 fabric and pass it on to the AXI4Lite fabric.
		// In case of burst requests, they are broken down to individual requests of axi4lite writes. Care
		// needs to be taken when writes are of different sizes in settin the write-strobe correctly.
		rule capture_write_requests_from_Axi4(wr_state==RegularReq);
			let wr_addr_req  <- pop_o (s_xactor.o_wr_addr);
	      let wr_data_req  <- pop_o (s_xactor.o_wr_data);
			ff_wr_addr.enq(wr_addr_req);
			ff_wr_data.enq(wr_data_req);
			rg_write_packet<=wr_addr_req;
			if(wr_addr_req.awlen!=0) begin
				wr_state<=BurstReq;
			end
			`logLevel( fabric, 0, $format("AXIBRIDGE: Write Request"))
			`logLevel( fabric, 0, $format("Address Channel:",fshow(wr_addr_req)))
			`logLevel( fabric, 0, $format("Data Channel:",fshow(wr_data_req)))
		endrule
		// In case a write-burst request is received on the fast bus, then the bursts have to broken down into
		// individual slow-bus write requests. 
		// This is rule is fired after the first write-burst request is sent to the slow_bus. This rule will continue to 
		// fire as long as the slow bus has capacity to receive a new request and the burst is not complete i.e.
		// fast bust xactor does not send wlast asserted.
		// The difference between the each individual requests on the slow bus is only the address. All other 
		// parameters remain the same.
		rule generate_bust_write_requests(wr_state==BurstReq);
			let request=rg_write_packet;
			request.awaddr= axi4burst_addrgen(request.awlen, request.awsize, request.awburst,request.awaddr);
	      let wr_data_req  <- pop_o (s_xactor.o_wr_data);
			ff_wr_addr.enq(request);
			ff_wr_data.enq(wr_data_req);
			rg_write_packet<=request;
			if(wr_data_req.wlast)begin
				wr_state<=RegularReq;
			end
			`logLevel( fabric, 0, $format("AXIBRIDGE: Burst Write Request"))
			`logLevel( fabric, 0, $format("Address Channel:",fshow(rg_write_packet)))
			`logLevel( fabric, 0, $format("Data Channel",fshow(wr_data_req)))
		endrule
		rule send_write_request_on_slow_bus;
			let wr_addr_req  = ff_wr_addr.first;
	      let wr_data_req  = ff_wr_data.first;
			ff_wr_data.deq;
			ff_wr_addr.deq;
			let aw = AXI4_Lite_Wr_Addr {awaddr: wr_addr_req.awaddr, awsize: truncate(wr_addr_req.awsize), awuser: wr_addr_req.awuser, awprot:wr_addr_req.awprot }; // arburst: 00-FIXED 01-INCR 10-WRAP
			let w  = AXI4_Lite_Wr_Data {wdata:  wr_data_req.wdata, wstrb: wr_data_req.wstrb};
			m_xactor.i_wr_addr.enq(aw);
			m_xactor.i_wr_data.enq(w);
			wr_id<=wr_addr_req.awid;
		endrule

		// This rule forwards the read response from the AXI4Lite to the AXI4 fabric.
		rule capture_read_responses;
			let response <- pop_o (m_xactor.o_rd_data);
			AXI4_Resp rresp= case(response.rresp)
				AXI4_LITE_OKAY  : AXI4_OKAY;
				AXI4_LITE_EXOKAY: AXI4_EXOKAY;
				AXI4_LITE_SLVERR: AXI4_SLVERR;
				AXI4_LITE_DECERR: AXI4_DECERR;
				default: AXI4_SLVERR; endcase;
			AXI4_Rd_Data#(data_width,user_width) r = AXI4_Rd_Data {rresp: rresp, rdata: response.rdata ,rlast:response_counter==rdburst_value, ruser: 0, rid:rd_id};
			if(response_counter==rdburst_value)
				response_counter<=0;
			else
				response_counter<=response_counter+1;
			ff_rd_resp.enq(r);
		endrule
		rule send_read_response_on_fast_bus;
			ff_rd_resp.deq;
			s_xactor.i_rd_data.enq(ff_rd_resp.first);
		endrule
		rule capture_write_responses;
			let response<-pop_o(m_xactor.o_wr_resp);
			AXI4_Resp bresp= case(response.bresp)
				AXI4_LITE_OKAY  : AXI4_OKAY;
				AXI4_LITE_EXOKAY: AXI4_EXOKAY;
				AXI4_LITE_SLVERR: AXI4_SLVERR;
				AXI4_LITE_DECERR: AXI4_DECERR;
				default: AXI4_SLVERR; endcase;
			let b = AXI4_Wr_Resp {bresp: bresp, buser:0, bid:wr_id};
			ff_wr_resp.enq(b);
		endrule
		rule send_write_response_on_fast_bus;
			ff_wr_resp.deq;
			s_xactor.i_wr_resp.enq(ff_wr_resp.first);
		endrule
		interface axi_slave=s_xactor.axi_side;
		interface axi4_lite_master=m_xactor.axi_side;
	endmodule
endpackage
