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

Author: Neel Gala, Deepa N Sarma
Email id: neelgala@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/
package bootrom;
  import BRAMCore :: *;
	import DReg::*;
	import Semi_FIFOF        :: *;
	import AXI4_Types   :: *;
	import AXI4_Fabric  :: *;
	import AXI4_Lite_Types   :: *;
	import AXI4_Lite_Fabric  :: *;
//  import Tilelink_lite_Types::*;
	import BUtils::*;
  import GetPut::*;
  import device_common::*;

  `include "Logger.bsv"
  export mkbootrom_axi4;
  export mkbootrom_axi4lite;
//  export mkbootrom_TLU;
  export Ifc_bootrom_axi4   (..);
  export Ifc_bootrom_axi4lite (..);
//  export Ifc_bootrom_TLU  (..);

  interface UserInterface#(numeric type addr_width,  numeric type data_width, 
                            numeric type index_size);
    method Action read_request (Bit#(addr_width) addr, AccessSize size);
    method Action write_request (Tuple3#(Bit#(addr_width), Bit#(data_width),  
                                                            Bit#(TDiv#(data_width, 8))) req);
    method ActionValue#(Tuple2#(Bool, Bit#(data_width))) read_response;
    method ActionValue#(Bool) write_response;
  endinterface
 
  // to make is synthesizable replace addr_width with Physical Address width
  // data_width with data lane width
  module mkbootrom#(parameter Integer slave_base)(UserInterface#(addr_width, data_width, index_size))
    provisos(Add#(data_width, a, 64), // provisos ensures we support only up-to 64-bit data width.
             Mul#(8, a__, data_width), // data_width should always be multiple of 8. 16 and 32.
             Mul#(16, b__, data_width),
             Mul#(32, c__, data_width));
  
    Integer byte_offset = valueOf(TDiv#(data_width, 32));
    Bit#(addr_width) base_address=fromInteger(slave_base);
  	// we create 2 32-bit BRAMs since the xilinx tool is easily able to map them to BRAM32BE cells
  	// which makes it easy to use data2mem for updating the bit file.

    // in case data_width is 32-bits, dmemMSB width becomes 0 and only dmemLSB is functional
    BRAM_PORT#(Bit#(index_size ), Bit#(TSub#(data_width, 32))) dmemMSB <- 
                                     mkBRAMCore1Load(valueOf(TExp#(index_size)), False, "boot.MSB", False);

    BRAM_PORT#(Bit#(index_size ), Bit#(32)) dmemLSB <- 
                                     mkBRAMCore1Load(valueOf(TExp#(index_size)), False, "boot.LSB", False);
  
    Reg#(Bool) read_request_sent <-mkDReg(False);
    Reg#(Tuple2#(Bit#(TAdd#(1,TDiv#(data_width,32))),AccessSize)) rg_req<- mkReg(tuple2(0,Byte));
    
    // A write request to bootrom has no significance.
    method Action write_request (Tuple3#(Bit#(addr_width), Bit#(data_width),  Bit#(TDiv#(data_width, 8))) req);
    	`logLevel( bootrom, 0, $format("BootROM: Illegal Write operation on BootROM"))
  	endmethod
  
    // The write response will always be an error.
    method ActionValue#(Bool) write_response;
      return True;
    endmethod
  
    // capture a read_request and latch the address on a BRAM.
    method Action read_request (Bit#(addr_width) addr, AccessSize size);
  	  Bit#(index_size) index_address=(addr-(base_address))[byte_offset+ valueOf(index_size) :
                                                                                  byte_offset+1];
      rg_req<= tuple2(addr[byte_offset:0],size);
  		dmemLSB.put(False, index_address, ?);
      dmemMSB.put(False, index_address, ?);
      read_request_sent<= True;
      `logLevel( bootrom, 0, $format("BootROM: Recieved Read Request for Address: %h \
Index Address: %h b: %d", addr, index_address, byte_offset))
  	endmethod
  
    // respond with data from the BRAM.
    method ActionValue#(Tuple2#(Bool, Bit#(data_width))) read_response if(read_request_sent);
      let {offset, size}=rg_req;
      Bit#(data_width) data={dmemMSB.read(), dmemLSB.read()};
      return tuple2(False, data);
    endmethod
  endmodule

  interface Ifc_bootrom_axi4#(numeric type addr_width, numeric type data_width, 
                              numeric type user_width, numeric type index_size);
    interface AXI4_Slave_IFC#(addr_width, data_width, user_width) slave; 
  endinterface

  typedef enum {Idle, Burst} Mem_State deriving(Eq, Bits, FShow);

  module mkbootrom_axi4#(parameter Integer slave_base)(Ifc_bootrom_axi4#(addr_width, data_width,
                                                                          user_width, index_width))
    provisos(Add#(data_width, a, 64), 
             Mul#(8, a__, data_width), 
             Mul#(16, b__, data_width), 
             Mul#(32, c__, data_width), 
             Add#(3, d__, TLog#(data_width)));
    UserInterface#(addr_width, data_width, index_width) dut <- mkbootrom(slave_base);
	  AXI4_Slave_Xactor_IFC #(addr_width, data_width, user_width)  s_xactor <- mkAXI4_Slave_Xactor;
    Reg#(Bit#(4)) rg_rd_id <-mkReg(0);
    Reg#(Mem_State) read_state <-mkReg(Idle);
    Reg#(Mem_State) write_state <-mkReg(Idle);
	  Reg#(Bit#(8)) rg_readburst_counter<-mkReg(0);
	  Reg#(AXI4_Rd_Addr	#(addr_width, user_width)) rg_read_packet <-mkReg(?);
	  Reg#(AXI4_Wr_Resp	#(user_width)) rg_write_response <-mkReg(?);
    Integer byte_offset = valueOf(TDiv#(data_width, 32));
    // If the request is single then simple send ERR. If it is a burst write request then change
    // state to Burst and do not send response.
    rule write_request_address_channel(write_state==Idle);
      let aw <- pop_o (s_xactor.o_wr_addr);
      let w  <- pop_o (s_xactor.o_wr_data);
	    let b = AXI4_Wr_Resp {bresp: AXI4_SLVERR, buser: aw.awuser, bid:aw.awid};
      if(!w.wlast)
        write_state<= Burst;
      else
  	  	s_xactor.i_wr_resp.enq (b);
      rg_write_response<= b;
    endrule
    // if the request is a write burst then keeping popping all the data on the data_channel and
    // send a error response on receiving the last data.
    rule write_request_data_channel(write_state==Burst);
      let w  <- pop_o (s_xactor.o_wr_data);
      if(w.wlast)begin
	  	  s_xactor.i_wr_resp.enq (rg_write_response);
        write_state<= Idle;
      end
    endrule
    // read first request and send it to the dut. If it is a burst request then change state to
    // Burst. capture the request type and keep track of counter.
    rule read_request_first(read_state==Idle);
		  let ar<- pop_o(s_xactor.o_rd_addr);
      dut.read_request(ar.araddr, unpack(truncate(ar.arsize)));
      rg_rd_id<= ar.arid;
      if(ar.arlen!=0)
        read_state<=Burst;
      rg_readburst_counter<= 0;
		  rg_read_packet<=ar;
    endrule
    // incase of burst read,  generate the new address and send it to the dut untill the burst
    // count has been reached.
    rule read_request_burst(read_state==Burst);
      if(rg_readburst_counter==rg_read_packet.arlen)
        read_state<=Idle;
      else begin
  		  let address=axi4burst_addrgen(rg_read_packet.arlen, rg_read_packet.arsize, 
            rg_read_packet.arburst, rg_read_packet.araddr);
        rg_read_packet.araddr<=address;
        rg_readburst_counter<= rg_readburst_counter+1;
        dut.read_request(address,unpack(truncate(rg_read_packet.arsize)));
      end
    endrule
    // get data from the bootrom. shift,  truncate, duplicate based on the size and offset.
    rule read_response;
      let {err, data0}<-dut.read_response;
  		let transfer_size=rg_read_packet.arsize;
      AXI4_Rd_Data#(data_width, user_width) r = AXI4_Rd_Data {rresp: AXI4_OKAY, rdata: data0 , 
        rlast:rg_readburst_counter==rg_read_packet.arlen, ruser: 0, rid:rg_read_packet.arid};
      `logLevel( bootrom, 1, $format("BootROM : Responding Read Request with Data: %h ", data0))
      s_xactor.i_rd_data.enq(r);
    endrule
    interface slave = s_xactor.axi_side;
  endmodule
  
  interface Ifc_bootrom_axi4lite#(numeric type addr_width, numeric type data_width, 
                              numeric type user_width, numeric type index_size);
    interface AXI4_Lite_Slave_IFC#(addr_width, data_width, user_width) slave; 
  endinterface


  module mkbootrom_axi4lite#(parameter Integer slave_base)(Ifc_bootrom_axi4lite#(addr_width,
                                                              data_width, user_width, index_width))
    provisos(Add#(data_width, a, 64), 
             Mul#(8, a__, data_width), 
             Mul#(16, b__, data_width), 
             Mul#(32, c__, data_width));
    UserInterface#(addr_width, data_width, index_width) dut <- mkbootrom(slave_base);
	  AXI4_Lite_Slave_Xactor_IFC #(addr_width, data_width, user_width)  s_xactor <- mkAXI4_Lite_Slave_Xactor;
    Integer byte_offset = valueOf(TDiv#(data_width, 32));
    Reg#(Bit#(2)) rg_size <-mkReg(3);
    Reg#(Bit#(TAdd#(1, TDiv#(data_width, 32)))) rg_offset <-mkReg(0);
    // If the request is single then simple send ERR. If it is a burst write request then change
    // state to Burst and do not send response.
    rule write_request_address_channel;
      let aw <- pop_o (s_xactor.o_wr_addr);
      let w  <- pop_o (s_xactor.o_wr_data);
	    let b = AXI4_Lite_Wr_Resp {bresp: AXI4_LITE_SLVERR, buser: aw.awuser};
	  	s_xactor.i_wr_resp.enq (b);
    endrule
    // read first request and send it to the dut. If it is a burst request then change state to
    // Burst. capture the request type and keep track of counter.
    rule read_request_first;
		  let ar<- pop_o(s_xactor.o_rd_addr);
      dut.read_request(ar.araddr,unpack(truncate(ar.arsize)));
      rg_size<= ar.arsize;
      rg_offset<= ar.araddr[byte_offset:0];
    endrule
    // get data from the bootrom. shift,  truncate, duplicate based on the size and offset.
    rule read_response;
      let {err, data0}<-dut.read_response;
      AXI4_Lite_Rd_Data#(data_width, user_width) r = AXI4_Lite_Rd_Data {rresp: AXI4_LITE_OKAY, rdata: data0 , 
        ruser: 0};
      `logLevel( bootrom, 1, $format("BootROM : Responding Read Request with Data: %h ", data0))
      s_xactor.i_rd_data.enq(r);
    endrule
    interface slave = s_xactor.axi_side;
  endmodule

  
//  interface Ifc_bootrom_TLU#(numeric type a, numeric type w, numeric type z);
//    interface Ifc_fabric_side_slave_link_lite#(a, w, z) slave; 
//  endinterface
//
//  (*mutually_exclusive="write_request_address_channel, read_response"*)
//  module mkbootrom_TLU#(parameter Integer slave_base)(Ifc_bootrom_TLU#(a, w, z))
//    provisos(Mul#(w, 8, data_width), 
//             Add#(data_width, e, 64), 
//             Mul#(8, a__,  data_width), 
//             Mul#(16, b__, data_width), 
//             Mul#(32, c__, data_width), 
//             Add#(d__, 2, z)); // to ensure that we are only operating upto 64 bits
//    UserInterface#(a, data_width) dut <- mkbootrom(slave_base);
//    Ifc_Slave_link_lite#(a, w, z)  s_xactor <- mkSlaveXactorLite(True, True);
//    Integer verbosity = `VERBOSITY;
//    Integer byte_offset = valueOf(TDiv#(data_width, 32));
//    Reg#(Bit#(z)) rg_size <-mkReg(3);
//    Reg#(Bit#(2)) rg_source <- mkReg(0);
//    Reg#(Bit#(TAdd#(1, TDiv#(data_width, 32)))) rg_offset <-mkReg(0);
//    Wire#(A_channel_lite#(a, w, z)) wr_request <- mkWire();
//
//    rule capture_request;
//      let req <- s_xactor.core_side.xactor_request.get;
//      wr_request<= req;
//    endrule
//    // If the request is single then simple send ERR. If it is a burst write request then change
//    // state to Burst and do not send response.
//    // TODO this will be obsolete if we iplemente master_route correctly. then the fabric itself
//    // will respond with an error without reaching the slave at all.
//    rule write_request_address_channel(wr_request.a_opcode==PutPartialData ||
//                                                                  wr_request.a_opcode==PutFullData);
//      let lv_resp = D_channel_lite{d_opcode: AccessAck, d_size: ?, d_source: wr_request.a_source,
//                                                      d_sink: ?, d_data: ?, d_error: True};
//	  	s_xactor.core_side.xactor_response.put(lv_resp);
//    endrule
//    // read first request and send it to the dut. If it is a burst request then change state to
//    // Burst. capture the request type and keep track of counter.
//    rule read_request_first(wr_request.a_opcode==Get_data);
//      dut.read_request(wr_request.a_address,unpack(truncate(wr_request.a_size)));
//      rg_size<= wr_request.a_size;
//      rg_offset<= wr_request.a_address[byte_offset:0];
//      rg_source<= wr_request.a_source;
//    endrule
//    // get data from the bootrom. shift,  truncate, duplicate based on the size and offset.
//    rule read_response;
//      let {err, data0}<-dut.read_response;
//      D_channel_lite#(w, z) lv_resp=D_channel_lite { d_opcode : AccessAckData, d_size : rg_size, 
//            d_source : rg_source, d_sink : ?, d_data : data0, d_error : False};
//  		if(verbosity!=0) 
//        `logLevel( bootrom, 0, $format("BootROM : Responding Read Request with Data: %h ", data0);
//	  	s_xactor.core_side.xactor_response.put(lv_resp);
//    endrule
//    interface slave = s_xactor.fabric_side;
//  endmodule
endpackage
