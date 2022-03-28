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

Author: Aditya Mathur	
Email id: aditya.mathur96@outlook.com
Details:

--------------------------------------------------------------------------------------------------
*/
package gpio;

`include "gpio.defines"

	/*==== Package imports ==== */
	import TriState          	::*;
	import Vector				::*;
	import BUtils				::*;	
	import ConfigReg			::*;
  import GetPut     ::*;
	/*===== Project Imports ===== */
	import Semi_FIFOF        :: *;
	import AXI4_Lite_Types   :: *;
	import AXI4_Lite_Fabric  :: *;
	import AXI4_Types 		 ::*;
	import AXI4_Fabric 		 ::*;
	import device_common	 ::*;


	export Ifc_gpio_axi4lite   (..);
  export Ifc_gpio_axi4       (..);
  export User_ifc (..);
  export GPIO (..);
  export mkgpio_axi4;
  export mkgpio_axi4lite;

  interface GPIO#(numeric type ionum);
		(*always_ready,always_enabled*)
		method Action gpio_in (Vector#(ionum,Bit#(1)) inp);
		method Vector#(ionum,Bit#(1))   gpio_out;
		method Vector#(ionum,Bit#(1))   gpio_out_en;
  endinterface
	interface User_ifc#(numeric type addr_width, numeric type data_width,numeric type ionum);
    (*always_ready,always_enabled*)
		interface Get#(Vector#(ionum ,Bit#(1))) sb_gpio_to_plic;
    interface GPIO#(ionum) io;
		method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data, AccessSize size);
		method ActionValue#(Tuple2#(Bool,Bit#(data_width))) read_req(Bit#(addr_width)addr,AccessSize size);
	endinterface

	module mkgpio(User_ifc#(addr_width,data_width,ionum))
		provisos(
				Add#(b__, data_width, 64),
        Add#(c__, ionum, 64)
			);
			
		Vector#(ionum ,ConfigReg#(Bool)) 	direction_reg		<-replicateM(mkConfigReg(False));
		Vector#(ionum ,ConfigReg#(Bit#(1))) dataout_register	<-replicateM(mkConfigReg(0));	
		Vector#(ionum ,ConfigReg#(Bit#(1))) datain_register		<-replicateM(mkConfigReg(0));	
		Vector#(ionum ,ConfigReg#(Bit#(1))) toplic				<-replicateM(mkConfigReg(0));

    let vionum = valueOf(ionum);

		rule capture_interrupt;
			for(Integer i=0;i<vionum ;i=i+1)
				toplic[i]<=(!direction_reg[i])?datain_register[i]:0;
		endrule

    interface io = interface GPIO#(ionum)
		  method Action gpio_in (Vector#(ionum,Bit#(1)) inp);
		  	for(Integer i=0;i<vionum ;i=i+1)
		  		datain_register[i]<=inp[i];
		  endmethod
		  method Vector#(ionum,Bit#(1))   gpio_out;
		  	Vector#(ionum,Bit#(1)) temp;
		  	for(Integer i=0;i<vionum ;i=i+1)
		  		temp[i]=dataout_register[i];
		  	return temp;
		  endmethod
		  method Vector#(ionum,Bit#(1))   gpio_out_en;
		  	Vector#(ionum,Bit#(1)) temp;
		  	for(Integer i=0;i<vionum ;i=i+1)
		  		temp[i]=pack(direction_reg[i]);
		  	return temp;
		  endmethod
    endinterface;

		method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data, AccessSize size);

			Bool success=True;
			
			//this method is incorrect because it assumes that the data is always in the lower 8,16,32 bit of the data that is transfered,
			// thus we need a mechanism to shift the data in accordance and then pass the data
			//Bit#(32) temp_data=(size==Word)?truncate(data):(size==HWord)?zeroExtend(data[15:0]):(size==Byte)?zeroExtend(data[7:0]):0;

	    Bit#(64) mask=size==Byte?'hff:size==HWord?'hFFFF:size==Word?'hFFFFFFFF:'1;
	    Bit#(6) shift_amt=zeroExtend(addr[2:0])<<3;
	    mask=mask<<shift_amt;
	    Bit#(64) datamask=zeroExtend(data)&mask;


			if( addr[6:0]>=`dir_reg && addr[6:0]<`dataout_reg )
				for(Integer i=0;i<vionum ;i=i+1)
					direction_reg[i]<=unpack(datamask[i]);
			else if(addr[6:0]>=`dataout_reg  && addr[6:0]< (`dataout_reg+8) )
				for(Integer i=0;i<vionum ;i=i+1)
					dataout_register[i]<=datamask[i];
			else
				success=False;
			return success;	
		endmethod

		method ActionValue#(Tuple2#(Bool, Bit#(data_width))) read_req (Bit#(addr_width) addr, AccessSize size);

			let dvalue=valueOf(data_width);
			Bool success= True;
			Bit#(data_width) data=0;
			Bit#(6) shift_amt=zeroExtend(addr[2:0])<<3;//generating the shift amount
			Bit#(64) temp =0;//parameterised
			
			if( addr[6:0]>=`dir_reg && addr[6:0]<`dataout_reg ) begin
				for(Integer i=0;i<vionum ;i=i+1)
					temp[i]=pack(direction_reg[i]);
			end
			else if(addr[6:0]>=`dataout_reg  && addr[6:0]< (`dataout_reg + 8)) begin
				for(Integer i=0;i<vionum ;i=i+1)
					temp[i]=datain_register[i];
			end
			else
				success=False;

			// shifting temp to get the required byte to the lowest position				
			temp = temp>>shift_amt;	
			if(size==Byte && dvalue%8==0)
       		temp=duplicate(temp[7:0]);
       	if(size==HWord && dvalue%16==0)
       		temp=duplicate(temp[15:0]);
       	else if(size==Word && dvalue%32==0)
       		temp=duplicate(temp[31:0]);
			else if(size == DWord && dvalue%64==0)	
			  	temp=duplicate(temp);

      		data=truncate(temp);
			return tuple2(success,data);
		endmethod

    interface sb_gpio_to_plic = interface Get
      method ActionValue#(Vector#(ionum ,Bit#(1))) get;
        Vector#(ionum,Bit#(1)) temp=readVReg(toplic);

        return temp;
      endmethod
    endinterface;

	endmodule:mkgpio

	interface Ifc_gpio_axi4lite#(numeric type addr_width, numeric type data_width, numeric type user_width,numeric type ionum);
		interface AXI4_Lite_Slave_IFC#(addr_width, data_width,user_width) slave;
    (*always_ready,always_enabled*)
		interface Get#(Vector#(ionum ,Bit#(1))) sb_gpio_to_plic;
    interface GPIO#(ionum) io;
	endinterface

	module mkgpio_axi4lite(Ifc_gpio_axi4lite#(addr_width,data_width,user_width,ionum))
		provisos(
				Add#(b__, data_width, 64),
        Add#(c__, ionum, 64)
			);

		User_ifc#(addr_width,data_width,ionum) gpio <-mkgpio;
		AXI4_Lite_Slave_Xactor_IFC#(addr_width,data_width,user_width)  s_xactor <- mkAXI4_Lite_Slave_Xactor();

		rule write_request;
			let addreq <- pop_o (s_xactor.o_wr_addr);
			let datareq  <- pop_o (s_xactor.o_wr_data);
			let succ <- gpio.write_req(addreq.awaddr, datareq.wdata,unpack(truncate(addreq.awsize)));
		  let ls = AXI4_Lite_Wr_Resp {bresp:succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, buser: addreq.awuser};
		  s_xactor.i_wr_resp.enq (ls);			
		endrule

		rule read_request;
			let req <- pop_o(s_xactor.o_rd_addr);
			let {succ,data}<- gpio.read_req(req.araddr,unpack(truncate(req.arsize)));
			let resp= AXI4_Lite_Rd_Data {rresp:succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, 
                                    rdata:data, ruser: ?};
	  		s_xactor.i_rd_data.enq(resp);
		endrule
	 	interface slave = s_xactor.axi_side;
    interface sb_gpio_to_plic=gpio.sb_gpio_to_plic;
    interface io = gpio.io;
	endmodule:mkgpio_axi4lite


	interface Ifc_gpio_axi4#(numeric type addr_width, numeric type data_width,numeric type user_width, numeric type ionum);
		interface AXI4_Slave_IFC#(addr_width,data_width,user_width) slave;
		interface Get#(Vector#(ionum ,Bit#(1))) sb_gpio_to_plic;
    interface GPIO#(ionum) io;
	endinterface

	module mkgpio_axi4(Ifc_gpio_axi4#(addr_width, data_width,user_width,ionum))
		provisos(
				Add#(b__, data_width, 64),
        Add#(c__, ionum, 64)
			);

		User_ifc#(addr_width,data_width,ionum) gpio <- mkgpio;
		AXI4_Slave_Xactor_IFC#(addr_width,data_width,user_width) s_xactor <- mkAXI4_Slave_Xactor();
		
		Reg#(Bit#(8)) rg_rdburst_count <- mkReg(0);
		Reg#(Bit#(8)) rg_wrburst_count <- mkReg(0);
		Reg#(AXI4_Rd_Addr#(addr_width,user_width)) rg_rdpacket <- mkReg(?);
 		Reg#(AXI4_Wr_Addr#(addr_width,user_width)) rg_wrpacket <- mkReg(?);	

		rule write_request(rg_wrburst_count==0);
			let addreq <- pop_o (s_xactor.o_wr_addr);
			let datareq  <- pop_o (s_xactor.o_wr_data);

		 	rg_wrpacket<=addreq;	

			let succ <- gpio.write_req(addreq.awaddr, datareq.wdata,unpack(truncate(addreq.awsize)));
			if(addreq.awlen!=0)
				rg_wrburst_count <=1;
		   	let ls = AXI4_Wr_Resp {bresp:succ?AXI4_OKAY:AXI4_SLVERR, buser: addreq.awuser, bid: addreq.awid};
		   	s_xactor.i_wr_resp.enq (ls);			
		endrule

		rule write_burst_request(rg_wrburst_count!=0);
			let addreq=rg_wrpacket;
			let datareq <-pop_o(s_xactor.o_wr_data);
	 		Bool succ=False;
	 		let resp = AXI4_Wr_Resp {bresp: succ?AXI4_SLVERR:AXI4_OKAY, buser: addreq.awuser, bid:addreq.awid};
			if(datareq.wlast)begin
	      		s_xactor.i_wr_resp.enq(resp);//enqueuing the write response
	      		rg_wrburst_count<=0;
	      	end
		endrule

		rule read_request(rg_rdburst_count==0);
			let req <- pop_o(s_xactor.o_rd_addr);

			rg_rdpacket<=req;

			let {succ,data}<- gpio.read_req(req.araddr,unpack(truncate(req.arsize)));

			if(req.arlen!=0)
				rg_rdburst_count<=1;

			let resp= AXI4_Rd_Data {rresp:succ?AXI4_OKAY:AXI4_SLVERR, rid:req.arid,rlast:(req.arlen==0),rdata:data, ruser: ?};
	  		s_xactor.i_rd_data.enq(resp);
		endrule

		rule read_burst_request(rg_rdburst_count!=0);
			let rd_req=rg_rdpacket;
			let {succ,data}<-gpio.read_req(rd_req.araddr,unpack(truncate(rd_req.arsize)));
			succ=False;
			if(rg_rdburst_count==rd_req.arlen)
				rg_rdburst_count<=0;
			else
				rg_rdburst_count<=rg_rdburst_count+1;
			let resp= AXI4_Rd_Data{rresp:succ?AXI4_OKAY:AXI4_SLVERR, rid:rd_req.arid,rlast:(rd_req.arlen==0),rdata:data, ruser: ?};
			s_xactor.i_rd_data.enq(resp);

		endrule
	 	interface slave = s_xactor.axi_side;
    interface sb_gpio_to_plic=gpio.sb_gpio_to_plic;
    interface io = gpio.io;
	endmodule:mkgpio_axi4
endpackage:gpio
