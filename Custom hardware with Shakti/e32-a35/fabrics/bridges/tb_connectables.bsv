package tb_connectables;

import Randomizable ::*; 
import BRAMCore ::*;
import Clocks ::*;
import BUtils ::*;
import Semi_FIFOF ::*;
import Vector ::*;
import FIFO :: *;
import SpecialFIFOs ::*;
import FIFOLevel ::*;

import AXI4_Types ::*;
import AXI4_Lite_Types ::*;
import bridge_connectables ::*;
import bridge_types ::*;
import axi_addr_generator ::*;
import AXI4_Fabric ::*;
import AXI4_Lite_Fabric ::*;
//`define wd_data_in 64 
//`define byte_offset 2
typedef TDiv#(`wd_data_in,8) WD_strb_in;
typedef TLog#(`wd_data_in) WD_data_bits_in;
typedef TLog#(WD_strb_in) WD_strb_bits_in;
`define USERSPACE 1
//`define wd_data_out 64 
//`define byte_offset 2
typedef TDiv#(`wd_data_out,8) WD_strb_out;
typedef TLog#(`wd_data_out)  WD_data_bits_out;
typedef TLog#(WD_strb_out)  WD_strb_bits_out;
typedef 15 Mem_size;

(*synthesize*)
module mktb_connectables(Empty);

	let v_WD_strb_in = valueOf(WD_strb_in);
	let v_WD_strb_bits_in = valueOf(WD_strb_bits_in);
	let v_WD_strb_bits_out = valueOf(WD_strb_bits_out);

	Clock clock1 <- exposeCurrentClock;
	Clock clock2 <- mkAbsoluteClock(0, 10);
	//Reset defaultreset <- exposeCurrentReset;
	Reset reset1 <- exposeCurrentReset;
	Reset reset2 <- mkAsyncResetFromCR(0, clock2);

	BRAM_DUAL_PORT_BE#(Bit#(TSub#(Mem_size,3)),Bit#(`wd_data_out),WD_strb_out) 
																			 data <- mkBRAMCore2BELoad(valueOf(TExp#(TSub#(Mem_size,3))),
																																				 False,
																																				 "memory_file.txt",
																																				 False, 
																																clocked_by clock2, reset_by reset2);

	BRAM_DUAL_PORT_BE#(Bit#(TSub#(Mem_size,3)),Bit#(`wd_data_out),WD_strb_out) 
																data_golden <- mkBRAMCore2BELoad(valueOf(TExp#(TSub#(Mem_size,3))),
																																				 False,
																																				 "memory_file.txt",
																																				 False);

	Randomize#(Bit#(`wd_addr_in)) rnd_awaddr	<- mkGenericRandomizer;
	Randomize#(Bit#(`wd_data_in)) rnd_awdata	<- mkGenericRandomizer;
	Randomize#(Bit#(8)) rnd_awlen	<- mkGenericRandomizer;
	Randomize#(Bit#(3)) rnd_awsize	<- mkGenericRandomizer;
	Randomize#(Bit#(2)) rnd_awburst <- mkConstrainedRandomizer(2'b00, 2'b01);



	Reg#(Bit#(TMul#(2,WD_strb_in))) rg_strb_shift <- mkReg(0);

	Reg#(Bit#(4)) rg_wr_tranx_id[2] <- mkCReg(2,0);
	Reg#(Int#(32)) rg_ctr_bram_index <- mkReg(0); 
	Reg#(Bool) rg_write_burst <- mkReg(False);
	Reg#(Bool) rg_init <- mkReg(True);
	Reg#(AXI4_Wr_Addr#(`wd_addr_in, `USERSPACE)) rg_wr_metadata <- mkReg(AXI4_Wr_Addr { awaddr : 0,
																																											 awuser : 0,
																																											 awlen  : 0, 
																																											 awsize  : 0, 
																																											 awburst  : 0, 
																																											 awid  : 0}); 
	Reg#(AXI4_Rd_Addr#(`wd_addr_in, `USERSPACE)) rg_read_packet <- mkReg(AXI4_Rd_Addr { araddr : 0,
																																											aruser : 0,
																																											arlen  : 0, 
																																											arsize  : 0, 
																																											arburst  : 0, 
																																											arid  : 0}, 
																															 clocked_by clock2, reset_by reset2); 
	Reg#(AXI4_Wr_Addr#(`wd_addr_in, `USERSPACE)) wr_addr_req <- mkReg(AXI4_Wr_Addr { awaddr : 0,
																																											 awuser : 0,
																																											 awlen  : 0, 
																																											 awsize  : 0, 
																																											 awburst  : 0, 
																																											 awid  : 0}); 
	Wire#(AXI4_Wr_Data#(`wd_data_in)) wr_data_req <- mkWire(); 
	Vector#(16, Reg#(AXI4_Rd_Addr#(`wd_addr_in, `USERSPACE))) 
																				rd_data_req <- replicateM(mkReg(AXI4_Rd_Addr { araddr : 0,
																																											aruser : 0,
																																											arlen  : 0, 
																																											arsize  : 0, 
																																											arburst  : 0, 
																																											arid  : 0}));

	FIFO#(AXI4_Rd_Addr#(`wd_addr_in, `USERSPACE)) ff_read_golden <- mkSizedFIFO(2);
	Reg#(Bit#(8)) rg_arlen <- mkReg(0, clocked_by clock2, reset_by reset2);
	Reg#(Bit#(WD_strb_in)) rg_strb_burst <- mkReg(0);
	Wire#(Bit#(`wd_data_in)) wr_data_response <- mkWire();
	Reg#(Bool) rg_rd_resp_burst <- mkReg(False, clocked_by clock2, reset_by reset2);
	Reg#(Bit#(8)) rg_rd_resp_counter <- mkReg(0, clocked_by clock2, reset_by reset2);

	AXI4_Master_Xactor_IFC #(`wd_addr_in, `wd_data_in, `USERSPACE) 
												m_xactor <- mkAXI4_Master_Xactor(clocked_by clock1, reset_by reset1);	
	AXI4_Slave_Xactor_IFC #(`wd_addr_out, `wd_data_out, `USERSPACE)  
												s_xactor <- mkAXI4_Slave_Xactor(clocked_by clock2, reset_by reset2);

	mkConnectionClocks(m_xactor.axi_side, clock1, reset1,
															s_xactor.axi_side, clock2, reset2);

//dumping the BRAM values
	Reg#(Int#(20)) rg_wr_counter <- mkReg(0);
	Reg#(Int#(20)) rg_wr_counter_resp <- mkReg(0, clocked_by clock2, reset_by reset2);
	Reg#(Bit#(12)) rg_address <- mkReg(0);
	Reg#(Bit#(12)) rg_address_resp <- mkReg(0, clocked_by clock2, reset_by reset2);
	SyncFIFOLevelIfc#(Bit#(64),2) ff_sync_fifo <- mkSyncFIFOLevel(clock1, reset1, clock2); 

	Reg#(Bool) rg_wr_resp_burst <- mkReg(False, clocked_by clock2, reset_by reset2);	
	Reg#(AXI4_Wr_Addr#(`wd_addr_out, `USERSPACE)) rg_wr_resp_meta <- mkReg(?, clocked_by clock2, reset_by reset2);	
	
rule rl_init_randomizers(rg_init);
	rg_init <= False;
	rnd_awaddr.cntrl.init();	
	rnd_awlen.cntrl.init();	
	rnd_awsize.cntrl.init();	
	rnd_awburst.cntrl.init();	
	rnd_awdata.cntrl.init();
endrule

rule rl_wr_send_req_master(!rg_write_burst && rg_wr_counter <= 2048);
	let lv_awaddr <- rnd_awaddr.next;	
	let lv_awlen <- rnd_awlen.next;	
	let lv_awburst <- rnd_awburst.next;	
	let lv_awsize <- rnd_awsize.next;	
	let lv_wdata <- rnd_awdata.next;	
	//Creating address mask according to awsize
	Bit#(`wd_addr_in) addr_mask = '1; 
	addr_mask = addr_mask << lv_awsize;
	lv_awaddr = lv_awaddr & addr_mask;
	//Creating strobe mask according to awsize
	Bit#(WD_strb_in) strb_mask = 0;
	Bit#(WD_strb_in) cnst_1 = '1;
	Bit#(TMul#(WD_strb_in,2)) strb_mask2 = {strb_mask, cnst_1};
	//Bit#(WD_strb_in) strb_mask = '1;
	Bit#(TAdd#(WD_strb_bits_in,1)) size_mask = 1; 
	size_mask = size_mask << lv_awsize;
	//size_mask = size_mask-1;
	strb_mask2 = strb_mask2 << size_mask;
	strb_mask = strb_mask2[2*v_WD_strb_in-1:v_WD_strb_in];
	Bit#(WD_strb_bits_in) word_offset = lv_awaddr[v_WD_strb_bits_in-1:0];
	//Now shift the strobe bits based on the byte address
	strb_mask = strb_mask << word_offset; 
	if(lv_awlen!=0) begin
		rg_write_burst <= True;
	end
	if(lv_awburst==0) begin
		let aligned_brst = countZerosMSB(lv_awlen);
		addr_mask = signExtend(8'h80);
		addr_mask = addr_mask >> aligned_brst;	
		if(`VERBOSITY>=2) $display(" addr_mask", lv_awaddr);
		lv_awaddr = addr_mask & lv_awaddr;
		lv_awlen = 8'h80 >> aligned_brst;
	end
	
	AXI4_Wr_Addr#(`wd_addr_in, `USERSPACE) aw = AXI4_Wr_Addr{ awaddr : lv_awaddr,
												 awuser : 0,
												 awlen 	: lv_awlen,
												 awsize : lv_awsize,
												 awburst: lv_awburst,
												 awid : 0};

	AXI4_Wr_Data#(`wd_data_in) w = AXI4_Wr_Data{  wdata : lv_wdata,
												 wstrb : strb_mask,
												 wid : 0,
												 wlast: lv_awlen==0};
	if(`VERBOSITY>=2) begin	
		if(`VERBOSITY>=2) $display(" AXI4 Write Address Request ", fshow(aw)); 
		if(`VERBOSITY>=2) $display(" AXI4 write data", fshow(w));
	end
	rg_wr_metadata <= aw;
	rg_strb_burst <= strb_mask;
	m_xactor.i_wr_addr.enq(aw);
	m_xactor.i_wr_data.enq(w);
	wr_addr_req <= aw;
	wr_data_req <= w;
	rg_wr_counter <= rg_wr_counter + 1;
endrule

rule rl_send_burst_req_master(rg_write_burst);
	let md = rg_wr_metadata;
	let lv_wdata <- rnd_awdata.next;	
	let address = burst_address_generator(md.awlen,
																				md.awsize,	
																				md.awburst,	
																				md.awaddr);	
	md.awaddr = address;
	md.awlen = md.awlen-1;
	rg_wr_metadata <= md;
	Bit#(TMul#(2,WD_strb_bits_in)) size_mask = 1;
	size_mask = size_mask << md.awsize;
	Bit#(TMul#(2,WD_strb_in)) lv_strb = zeroExtend(rg_strb_burst);
	lv_strb = lv_strb << size_mask; 
	rg_strb_shift <= truncateLSB(lv_strb);

	let w = AXI4_Wr_Data{  wdata : lv_wdata,
												 wstrb : truncateLSB(lv_strb),
												 wid : 0,
												 wlast: md.awlen==0};
	m_xactor.i_wr_data.enq(w);

	if(md.awlen==0) begin
		rg_write_burst <= False;
		$display(" AXI4 write burst last data ", fshow(w));
	end
	else begin
		if(`VERBOSITY>=2) begin
				$display(" AXI4 write burst data ", fshow(w));
		end
	end
endrule

rule rl_rd_send_req_master(rg_wr_tranx_id[1]<15);
	let lv_araddr <- rnd_awaddr.next;	
	let lv_arlen <- rnd_awlen.next;	
	let lv_arburst <- rnd_awburst.next;	
	let lv_arsize <- rnd_awsize.next;	
	//Creating address mask according to awsize
	Bit#(`wd_addr_in) addr_mask = '1; 
	addr_mask = addr_mask << lv_arsize;
	lv_araddr = lv_araddr & addr_mask;
	//Creating strobe mask according to awsize
	Bit#(WD_strb_in) strb_mask = 0;
	Bit#(WD_strb_in) cnst_1 = '1;
	Bit#(TMul#(WD_strb_in,2)) strb_mask2 = {strb_mask, cnst_1};
	//Bit#(WD_strb_in) strb_mask = '1;
	Bit#(TAdd#(WD_strb_bits_in,1)) size_mask = 1; 
	size_mask = size_mask << lv_arsize;
	//size_mask = size_mask-1;
	strb_mask2 = strb_mask2 << size_mask;
	strb_mask = strb_mask2[2*v_WD_strb_in-1:v_WD_strb_in];
	Bit#(WD_strb_bits_in) word_offset = lv_araddr[v_WD_strb_bits_in-1:0];
	//Now shift the strobe bits based on the byte address
	strb_mask = strb_mask << word_offset; 
	if(lv_arburst==0) begin
		let aligned_brst = countZerosMSB(lv_arlen);
		addr_mask = signExtend(8'h80);
		addr_mask = addr_mask >> aligned_brst;	
		if(`VERBOSITY>=1) $display(" addr_mask %h", lv_araddr);
		lv_araddr = addr_mask & lv_araddr;
		lv_arlen = 8'h80 >> aligned_brst;
	end
	
	AXI4_Rd_Addr#(`wd_addr_in, `USERSPACE) ar = AXI4_Rd_Addr{ araddr : lv_araddr,
												 aruser : 0,
												 arlen 	: lv_arlen,
												 arsize : lv_arsize,
												 arburst: lv_arburst,
												 arid : rg_wr_tranx_id[1]};

	if(`VERBOSITY>=1) $display("Read Channel ", fshow(ar));
	m_xactor.i_rd_addr.enq(ar);
	rg_wr_tranx_id[1] <= rg_wr_tranx_id[1] + 1;
	rd_data_req[rg_wr_tranx_id[1]] <= ar;	
endrule

rule rl_golden_output_wr;
		let aw = wr_addr_req;
		let w  = wr_data_req;
    Bit#(TSub#(Mem_size,3)) index_address=(aw.awaddr)[valueOf(Mem_size)-1:v_WD_strb_bits_out];
		data_golden.b.put(w.wstrb, index_address, truncate(w.wdata)); 
endrule

(*mutually_exclusive="rl_send_write_response, rl_send_read_bram_input"*)
rule rl_send_write_response(!rg_wr_resp_burst);
    let aw <- pop_o (s_xactor.o_wr_addr);                                                       
    let w  <- pop_o (s_xactor.o_wr_data);                                                       
    Bit#(TSub#(Mem_size,3)) index_address=(aw.awaddr)[valueOf(Mem_size)-1:v_WD_strb_bits_out];
		data.b.put(w.wstrb, index_address, truncate(w.wdata)); 
    //dmemLSB.b.put(w.wstrb[3:0],index_address,truncate(w.wdata));                                  
    //dmemMSB.b.put(w.wstrb[7:4],index_address,truncateLSB(w.wdata));                               
		aw.awaddr = burst_address_generator(aw.awlen, aw.awsize, aw.awburst, aw.awaddr);
		if(`VERBOSITY>=2) begin	
			if(`VERBOSITY>=1) $display($time, "\tMEMORY: Data being written into memory %h", w.wdata);
		end
		if(aw.awlen!=0) begin
			rg_wr_resp_burst <= True;
			rg_wr_resp_meta <= aw;
		end
		else begin
    	let b = AXI4_Wr_Resp {bresp: AXI4_OKAY, buser: aw.awuser, bid: 0};                      
    	s_xactor.i_wr_resp.enq (b);                                                               
			if(`VERBOSITY>=1) $display($time, "\tMEMORY: Data last of the response", fshow(w.wdata));
			if(`VERBOSITY>=1) $display($time, "\t--------------------------- \
																						---------------------------", fshow(w.wdata));
		end
endrule

rule rl_send_write_response_burst(rg_wr_resp_burst);
    let w  <- pop_o (s_xactor.o_wr_data);                                                       
		let aw = rg_wr_resp_meta;
    Bit#(TSub#(Mem_size,3)) index_address=(aw.awaddr)[valueOf(Mem_size)-1:v_WD_strb_bits_out];
		data.b.put(w.wstrb, index_address, truncate(w.wdata)); 
    //dmemLSB.b.put(w.wstrb[3:0],index_address,truncate(w.wdata));                                  
    //dmemMSB.b.put(w.wstrb[7:4],index_address,truncateLSB(w.wdata));                               
		aw.awaddr = burst_address_generator(aw.awlen, aw.awsize, aw.awburst, aw.awaddr);
		if(`VERBOSITY>=1) begin	
			$display($time, "\tMEMORY: Data %d being written into memory", w.wdata, aw.awlen);
		end
		if(w.wlast) begin
    	let b = AXI4_Wr_Resp {bresp: AXI4_OKAY, buser: aw.awuser, bid: 0};                      
    	s_xactor.i_wr_resp.enq (b);                                                               
			rg_wr_resp_burst <= False;
			if(`VERBOSITY>=1) begin	
				$display($time, "\tMEMORY: Memory write response");
			end
		end
		else begin
			aw.awlen = aw.awlen - 1;
			rg_wr_resp_meta <= aw;
		end
endrule

rule rl_send_read_bram_input(!rg_rd_resp_burst);
	let ar<- pop_o(s_xactor.o_rd_addr);                                                           
  rg_read_packet <= ar;
	rg_arlen <= ar.arlen-1;
  Bit#(TSub#(Mem_size,3)) index_address=(ar.araddr)[valueOf(Mem_size)-1:v_WD_strb_bits_out];
	data.a.put(0, index_address, ?);
	rg_rd_resp_burst <= True;
	rg_rd_resp_counter <= ar.arlen;
endrule

rule rl_send_read_response(rg_rd_resp_burst);
	Bit#(`wd_data_out) data_out = data.a.read();
	let addr_req = rg_read_packet;
	Bit#(WD_strb_bits_out) byte_address = addr_req.araddr[v_WD_strb_bits_out-1:0];
	data_out = data_out >> byte_address; 
	if(addr_req.arsize==0) begin
		data_out = duplicate(data_out[7:0]);
	end
	if(valueOf(`wd_data_out)>=16) begin
		if(addr_req.arsize==1) begin
			data_out = duplicate(data_out[15:0]);
		end
	end
	if(valueOf(`wd_data_out)>=32) begin
		if(addr_req.arsize==2) begin
			data_out = duplicate(data_out[31:0]);
		end
	end
	if(valueOf(`wd_data_out)>=64) begin
		if(addr_req.arsize==3) begin
			data_out = duplicate(data_out[63:0]);
		end
	end
	let address = burst_address_generator(addr_req.arlen,
																				addr_req.arsize,	
																				addr_req.arburst,	
																				addr_req.araddr);	
	AXI4_Rd_Data#(`wd_data_out,`USERSPACE) r = AXI4_Rd_Data {rresp: AXI4_OKAY, 	
																													 rdata: data_out, 
																													 ruser: 0,
																													 rid  : 0,
																													 rlast : rg_arlen==0};
	s_xactor.i_rd_data.enq(r);
  Bit#(TSub#(Mem_size,3)) index_address=address[valueOf(Mem_size)-1:v_WD_strb_bits_out];
	if(`VERBOSITY>=1) $display($time,"\tMemory Rd Response index address %h", index_address, fshow(r));
	if(rg_rd_resp_counter==0) begin
		rg_rd_resp_burst <= False;	
	end
	else begin
		data.a.put(0, index_address, ?);
		rg_rd_resp_counter <= rg_rd_resp_counter - 1;
		addr_req.araddr = address;
	end
	rg_read_packet <= addr_req;
endrule

rule rl_get_wr_response;
	let response <- pop_o(m_xactor.o_wr_resp);
	//if(response.bresp!=AXI4_OKAY)
	//	$finish(0);
endrule

rule rl_get_rd_response;
	let response <- pop_o(m_xactor.o_rd_data);
	wr_data_response <= response.rdata;
endrule

rule rl_golden_output_rd;
		let ar = rd_data_req[rg_wr_tranx_id[1]];
		ff_read_golden.enq(ar);
    Bit#(TSub#(Mem_size,3)) index_address=(ar.araddr)[valueOf(Mem_size)-1:v_WD_strb_bits_out];
		data_golden.a.put(0, index_address, ?); 
endrule

rule rl_golden_output_rd_resp;
		Bit#(`wd_data_in) data = data_golden.a.read;
		let metadata = ff_read_golden.first;	
		ff_read_golden.deq;
		rg_wr_tranx_id[0] <= rg_wr_tranx_id[0] - 1;
		Bit#(WD_strb_bits_out) byte_address = metadata.araddr[v_WD_strb_bits_out-1:0];
		data = data >> byte_address; 
	if(metadata.arsize==0) begin
		data = duplicate(data[7:0]);
	end
	if(valueOf(`wd_data_out)>=16) begin
		if(metadata.arsize==1) begin
			data = duplicate(data[15:0]);
		end
	end
	if(valueOf(`wd_data_out)>=32) begin
		if(metadata.arsize==2) begin
			data = duplicate(data[31:0]);
		end
	end
	if(valueOf(`wd_data_out)>=64) begin
		if(metadata.arsize==3) begin
			data = duplicate(data[63:0]);
		end
	end
	//if(wr_data_response != data) begin
	//	$finish(0);
	//end
endrule

rule rl_golden_iterative_check_req(rg_wr_counter > 2048 && rg_wr_counter < 4096);
	data_golden.a.put(0, rg_address, ?);
endrule

rule rl_golden_iterative_check_resp;
	let result_data = data_golden.a.read;
	rg_address <= rg_address + 1;	
	ff_sync_fifo.enq(result_data);
endrule

rule rl_data_iterative_check_req(rg_wr_counter_resp > 2048 && rg_wr_counter_resp < 4096);
	data.b.put(0, rg_address_resp, ?);
endrule

rule rl_data_iterative_check_resp;
	let actual_data = data.b.read;
	rg_address_resp <= rg_address_resp + 1;	
	//if(actual_data != ff_sync_fifo.first)
	//$finish(0);
	ff_sync_fifo.deq;
endrule

rule rl_finish(rg_wr_counter==4095);
		$finish(0);
endrule

endmodule
endpackage
