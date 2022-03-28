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
/*
Author: Deepa N Sarma
Email id: deepans.88@gmail.com

This mo:dule
1.Transalates  AXI master request to mcpu master request
2.Provides support for burst transfers
2.Manages dynamic bus sizing which is a feature of mcpu
3.Transalates mcpu response to AXI response
4.Includes a transactor converting 64 bit transactions to multiple 32 bit transactions
5.Modifies the data alignment as per endianness of the slave addressed
*/

package mcpu;

	/* ======== Package imports ======= */
	import Vector				::*;
	import FIFO					::*;
	import ConfigReg		::*;
	import  AXI4_Types  ::*;
	import  AXI4_Fabric ::*;
	import  Semi_FIFOF ::*;
 	import BUtils::*;
  import device_common ::*;
	/*================================== */

	/*========= Project imports ======== */
	`include "mcpu_parameters.bsv"
  `include "Logger.bsv"
	import FIFOF					::*;
	import mcpu_master :: *;
  import mcpu_defines :: *;
	/*================================== */


	interface Ifc_mcpu_top;
  interface Mcpu_out proc_ifc;
	interface  Data_bus_inf proc_dbus;
  interface AXI4_Slave_IFC#(`PADDR,`Reg_width,`USERSPACE) slave_axi_mcpu;
	method Action rd_ipl(Bit#(3) ip);
		/*-============================================================================= */
	endinterface

	typedef enum {DATA,INST} Priority_cache deriving (Bits, Eq, FShow);
	typedef enum
  {DATA_MODE_8_READ,DATA_MODE_16_READ,DATA_MODE_32_READ,DATA_MODE_8_WRITE,
  DATA_MODE_16_WRITE,DATA_MODE_32_WRITE,DATA_MODE_64_READ1,DATA_MODE_64_WRITE1,DATA_MODE_64_READ2,DATA_MODE_64_WRITE2,INST_MODE}
  Data_mode deriving (Bits, Eq, FShow);

  function Bit#(2) modeconv_mcpu(Bit#(3) transfer_size );
  
  if(transfer_size==0)//8 bit transfer
       return 2'b01;
  else if (transfer_size==1)//16 bit transfer
       return 2'b10;
  else
       return 2'b00;//32 bit transfer
  endfunction
          
	(*synthesize*)
	module mkmcpu_top(Ifc_mcpu_top);

    String mcpu =" ";
		
		AXI4_Slave_Xactor_IFC #(`PADDR,`Reg_width,`USERSPACE) s_xactor <- mkAXI4_Slave_Xactor;
	  Mcpu_master proc_master <-mkmcpumaster;
		Reg#(Bit#(`Reg_width_mcpu_slave)) response_buff  <-mkReg(0);//To buffer multiple cycle transfers
    FIFOF#(Bit#(4)) ff_id <-mkSizedFIFOF(2);//To store request address of instruction
    FIFOF#(Bit#(`Reg_width_mcpu_slave)) ff_address <-mkSizedFIFOF(2);//To store request address of instruction
		FIFOF#(Data_mode) ff_req<-mkSizedFIFOF(2);//To keep track of last pending request
		FIFOF#(Bool) ff_last<-mkSizedFIFOF(2);//To keep track of last pending request
    Reg#(Bit#(2)) rg_port_count <- mkReg(0);//To keep track of multi cycle requests
		Reg#(Bit#(32)) rg_inst_rcvd <- mkReg(0);
    Reg#(Bit#(3)) rg_endian  <- mkReg(0);//Dynamic endianness indicator
    Reg#(Bool) dw_write <- mkReg(False);//Double word write
    Reg#(Bool) dw_read  <-mkReg(False);//Double word read
    Reg#(Bool) err_buff  <-mkReg(False);//Buffer the bus_error across double word writes
    Reg#(Bit#(1)) response_berr  <-mkReg(0);//Bus error
    Reg#(Bit#(`PADDR)) dw_addr <-mkReg(0);//Double word address
    Reg#(Bit#(`PADDR)) rg_burst_addr <-mkReg(0);//Buffer addresses for bursts
    Reg#(Bit#(`PADDR)) dw_read_addr <-mkReg(0);//Buffer addresses for double word transactions
    Reg#(Bit#(2)) rg_burst <-mkReg(0);//Buffer arburst for burst transfer
    Reg#(Bit#(8)) rg_arlen <-mkReg(0);//Buffer burst length
    Reg#(Bit#(4)) rg_awid <-mkReg(0);//Buffer id
    Reg#(Bit#(8)) rg_counter <-mkReg(0);//Counter to initiate read bursts
    Reg#(Bit#(3)) rg_size <-mkReg(0);//Buffer burst size
    Reg#(Bit#(32))dw_data<-mkReg(0);//Buffer double word data
    Reg#(Bit#(32))data_buff<-mkReg(0);//Buffer response if takes multi cycle
    Reg#(Bool)read_burst_mode <-mkReg(False);//read burst active
    Reg#(Bool)write_burst_mode <-mkReg(False);//write burst active

//.......................SEND_REQUEST_TO_MEMORY..................................................//
//...............................................................................................//

// Rule fires if no double word writes or burst writes are pending 
		rule check_wr_request_to_memory(!dw_write && !dw_read && !read_burst_mode && !write_burst_mode);
      
      let info<-pop_o(s_xactor.o_wr_addr);
      let data<-pop_o(s_xactor.o_wr_data);

      Bool sram_big =False;
      if (rg_endian<2)
        rg_endian <= rg_endian+1;
      else
        sram_big  = True;

       
      
      
      let request=Req_mcpu{addr:truncate(info.awaddr),wr_data:truncate(data.wdata),
      mode:modeconv_mcpu(info.awsize),fun_code:3'b010,rd_req:0,endian_big:sram_big};


      if(info.awsize==3'b011 && endian(info.awaddr,True)==Big)
      request.wr_data = data.wdata[63:32];
     
      //Burst_transfers
      if (info.awlen!=0)
      begin
        write_burst_mode <= True;
        rg_counter   <= info.awlen-8'd1;
        rg_arlen   <= info.awlen;
        rg_burst_addr <= info.awaddr;
        rg_burst <= info.awburst;
        rg_size <=info.awsize;
        rg_awid <=info.awid;
        ff_last.enq(False);
        `logLevel( mcpu, 1, $format("MCPU:Sending first word burst mode from address"))
      end
      else
        ff_last.enq(True);
      
      //Accessing interrupt registers 
      if(info.awaddr[31:4]==28'hFFFF_FFF)
			request.fun_code=3'b111;	
      proc_master.get_req(request);
      ff_id.enq(info.awid);
      `logLevel( mcpu, 1, $format("MCPU:Enqueing write request for address %h",info.awaddr))
      ff_address.enq(info.awaddr);
      //Request scheduling
      case(request.mode)
	  		2'b00 :if(info.awsize==3'b011)
               ff_req.enq(DATA_MODE_64_WRITE1);
               else
               ff_req.enq(DATA_MODE_32_WRITE);
      	2'b01 :ff_req.enq(DATA_MODE_8_WRITE);
      	2'b10 :ff_req.enq(DATA_MODE_16_WRITE);
      endcase
      
      //Managing double word transfers
      if(info.awsize==3'b011)begin
        dw_write<=True;
        dw_addr<=info.awaddr;
        if(endian(info.awaddr,True)==Big)
        begin
          dw_data<=data.wdata[31:0];
        end
        else
        begin
          dw_data<=data.wdata[63:32];
        end
      end
     
    endrule

    //Rule fires on burst writes
		rule check_wr_request_to_memory_burst(!dw_write && !dw_read && write_burst_mode);
      
      let addr = rg_burst_addr;
      let data<-pop_o(s_xactor.o_wr_data);
      
      //Controlling dynamic endianess of sram 
      Bool sram_big =False;
      if (rg_endian<2)
        rg_endian <= rg_endian+1;
      else
        sram_big  = True;

       
      //Generating last word for endianness
      if (data.wlast)
      begin
         ff_last.enq(True);
        write_burst_mode <=False;
      end
      else
      begin
        ff_last.enq(False);
      end
       
     //Generating address for burst_transfers
      if(rg_size!=0)
      addr= axi4burst_addrgen(rg_arlen,rg_size,rg_burst,rg_burst_addr);
      
      rg_burst_addr<=addr;


      let request=Req_mcpu{addr:truncate(addr),wr_data:truncate(data.wdata),
      mode:modeconv_mcpu(rg_size),fun_code:3'b010,rd_req:0,endian_big:sram_big};
      
      if(rg_size==3'b011 && endian(addr,True)==Big)
      request.wr_data = data.wdata[63:32];
     
      //accessing interrupt registers 
      if(addr[31:4]==28'hFFFF_FFF)
			request.fun_code=3'b111;	

      proc_master.get_req(request);
      ff_id.enq(rg_awid);
      `logLevel( mcpu, 1, $format("MCPU:Enqueing write request for address %h",addr))
      ff_address.enq(addr);
      
      //Request scheduling
      case(request.mode)
	  		2'b00 :if(rg_size==3'b011)
               ff_req.enq(DATA_MODE_64_WRITE1);
               else
               ff_req.enq(DATA_MODE_32_WRITE);
      	2'b01 :ff_req.enq(DATA_MODE_8_WRITE);
      	2'b10 :ff_req.enq(DATA_MODE_16_WRITE);
      endcase
      
      
      //Managing double word transfers
      if(rg_size==3'b011)begin
        dw_write<=True;
        dw_addr<=addr;
        if(endian(addr,True)==Big)
        begin
          dw_data<=data.wdata[31:0];
        end
        else
        begin
          dw_data<=data.wdata[63:32];
        end
      end

  	endrule



    //Rule fires in 2nd cycle of double wor d writes
		rule check_wr_request_to_memory_dw_write(dw_write);
      
      Bool sram_big =False;
      if (rg_endian<2)
        rg_endian <= rg_endian+1;
      else
        sram_big  = True;

      let request=Req_mcpu{addr:dw_addr+4,wr_data:dw_data,mode:2'b00,fun_code:3'b010,rd_req:0,endian_big:sram_big};

      //accessing  interrupt_registers
      if(dw_addr[31:4]==28'hFFFF_FFF)
 			request.fun_code=3'b111;	
      proc_master.get_req(request);
      `logLevel( mcpu, 1, $format("MCPU:Enqueing write request for address %h",dw_addr+4))
      ff_address.enq(dw_addr);
      dw_write<=False;
      ff_req.enq(DATA_MODE_64_WRITE2);

  	endrule


    //Rule fires if no double word or burst request is pending

	  rule check_read_request_to_memory(!dw_write && !dw_read && !read_burst_mode && !write_burst_mode);
      
      let info<- pop_o(s_xactor.o_rd_addr);
      ff_id.enq(info.arid);
      ff_address.enq(info.araddr);
     //sram is little-endian for first four cycles and big-endian after that 
      Bool sram_big=False;
      if (rg_endian <2)
        rg_endian <= rg_endian+1;
      else
        sram_big = True;
      //For burst transfers:-
      if (info.arlen!=0)
      begin
        read_burst_mode <= True;
        rg_counter <= info.arlen-8'd1;
        rg_arlen   <= info.arlen;
        rg_burst_addr <= info.araddr;
        rg_size <=info.arsize;
        rg_burst<=info.arburst;
        ff_last.enq(False);
        `logLevel( mcpu, 1, $format("MCPU:Enqueing read request for address %h",info.araddr))
      end
      else
        ff_last.enq(True);
        let request=Req_mcpu{addr:truncate(info.araddr),wr_data:?,mode:modeconv_mcpu(info.arsize),
        fun_code:3'b010,rd_req:1,endian_big:sram_big};
        if (info.araddr[31:4]==28'hFFFF_FFF)
        request.fun_code=3'b111;
            
       //Request scheduling
        case(request.mode)
			  2'b00 :if(info.arsize ==3'b011)
                ff_req.enq(DATA_MODE_64_READ1);
               else
                ff_req.enq(DATA_MODE_32_READ);
      	2'b01 :ff_req.enq(DATA_MODE_8_READ);
      	2'b10 :ff_req.enq(DATA_MODE_16_READ);
        endcase
      proc_master.get_req(request);

      //Managing double word transaction
      if(info.arsize==3'b011)begin
        dw_read<=True;
        dw_read_addr<=info.araddr;
      end

    endrule

   //Rule fires if burst mode is pending

	 rule check_read_request_to_memory_burst(!dw_write && !dw_read && read_burst_mode);
     
      let addr = rg_burst_addr;
      //sram is little-endian for first four cycles and big-endian after that 
      Bool sram_big=False;
      if (rg_endian <2)
        rg_endian <= rg_endian+1;
      else
        sram_big = True;
      //tracking the count for burst transfers:-
      if (rg_counter!=0)
      begin
        rg_counter <= rg_counter-8'd1;
        ff_last.enq(False);
      end
      else
      begin
        read_burst_mode <=False;
        ff_last.enq(True);//Enqueuing last word info
      end
       
       let shift =  32'b1;
     //Generating address for burst_transfers
      if(rg_size!=0)
      addr= axi4burst_addrgen(rg_arlen,rg_size,rg_burst,rg_burst_addr);
      rg_burst_addr<=addr;
      
      ff_address.enq(addr);
      let request=Req_mcpu{addr:truncate(addr),wr_data:?,mode:modeconv_mcpu(rg_size),
      fun_code:3'b010,rd_req:1,endian_big:sram_big};
      //function code if accessing interrupt registers(reading interrupt_vector)
      if (rg_burst_addr[31:4]==28'hFFFF_FFF)
       request.fun_code=3'b111;
      
       
       //Request scheduling
      case(request.mode)
			  2'b00 :if(rg_size ==3'b011)
                ff_req.enq(DATA_MODE_64_READ1);
               else
                ff_req.enq(DATA_MODE_32_READ);
      	2'b01 :ff_req.enq(DATA_MODE_8_READ);
      	2'b10 :ff_req.enq(DATA_MODE_16_READ);
      endcase

      `logLevel( mcpu, 1, $format("MCPU:Enqueing read request for address %h",addr))
      proc_master.get_req(request);

           
      //Managing double word transaction
      if(rg_size==3'b011)begin
        dw_read<=True;
        dw_read_addr<=addr;
      end

   endrule



	 rule check_read_request_to_memory_dw_read(dw_read);
      
      ff_address.enq(dw_read_addr);
      
      //sram is little-endian for first four cycles and big-endian after that 
      Bool sram_big=False;
      if (rg_endian <2)
        rg_endian <= rg_endian+1;
      else
        sram_big = True;
      let request=Req_mcpu{addr:dw_read_addr+4,wr_data:?,mode:2'b00,
      fun_code:3'b010,rd_req:1,endian_big:sram_big};
      
      //Managing double word transaction
      if (dw_read_addr[31:4]==28'hFFFF_FFF)
       request.fun_code=3'b111;
      
      `logLevel( mcpu, 1, $format("MCPU:Enqueing read request for address %h",dw_read_addr+4))
      proc_master.get_req(request);		
      dw_read<=False;
      ff_req.enq(DATA_MODE_64_READ2);

   endrule


  (* preempts = "check_read_request_to_memory,check_wr_request_to_memory"*)
  (* mutually_exclusive = "check_read_request_to_memory_burst,check_wr_request_to_memory"*)
  (* mutually_exclusive = "check_read_request_to_memory_burst,check_wr_request_to_memory_burst"*)
  (* mutually_exclusive = "check_read_request_to_memory_burst,check_wr_request_to_memory_dw_write"*)
  (* preempts = "check_read_request_to_memory_dw_read,check_wr_request_to_memory_dw_write"*)
  (* preempts = "check_wr_request_to_memory_burst,check_read_request_to_memory"*)

//...................................SEND RESPONSE TO MEMORY...............................//
//While sending response to memory slave_width is checked. If slave width is lesser than 
//requested transfer size,we receive data in multiple cycles and send the response back to core
//after we completely receive the data.In case of burst transfers,whenever the last request is send,
// is stored in ff_last.On recieving the response for last transaction,this is indicated in 
//response back to the core

	(* mutually_exclusive="send_read_response_from_memory_to_mem_stage_8,\
  send_read_response_from_memory_to_mem_stage_16,send_read_response_from_memory_to_mem_stage_32,\
  send_write_response_from_memory_to_mem_stage_8,send_write_response_from_memory_to_mem_stage_16,\
  send_write_response_from_memory_to_mem_stage_32"*)

  rule send_read_response_from_memory_to_mem_stage_8(ff_req.first==DATA_MODE_8_READ); 
    let response =proc_master.put_resp(); 
      ff_req.deq();
      let r = AXI4_Rd_Data {rresp: AXI4_OKAY, rdata:duplicate(response.data[7:0]),
      rlast:False, ruser: 0, rid: ff_id.first };
      //checking last word for burst_transfers
      if(ff_last.first)
      begin
        r.rlast=True;
        ff_id.deq; //id remains same through out burst
      end
      r.rid=ff_id.first;
      ff_last.deq;

      if(response.berr==1'b1)
      r.rresp = AXI4_SLVERR;
      ff_address.deq();
      s_xactor.i_rd_data.enq(r);
      `logLevel( mcpu, 1, $format("MCPU:Data received %h with id ",response.data,fshow(ff_id.first)))
	endrule
                
      
	rule send_read_response_from_memory_to_mem_stage_16(ff_req.first==DATA_MODE_16_READ);
			let response =proc_master.put_resp();
			if(response.port_type==2'b10 )begin
					ff_req.deq();
          let r = AXI4_Rd_Data {rresp: AXI4_OKAY,rdata:duplicate(response.data[15:0]),rlast:False,
          ruser: 0, rid: ff_id.first };
          
          
          
          //checking last word for burst_transfers
          if(ff_last.first)
          begin
            r.rlast=True;
            ff_id.deq;
          end
          r.rid=ff_id.first;
          ff_last.deq;

    			
          if(response.berr==1'b1)
					r.rresp = AXI4_SLVERR;
					ff_address.deq();
					s_xactor.i_rd_data.enq(r);
          `logLevel( mcpu, 1, $format("MCPU:Data received %h ",response.data))
			end						
			else if(response.port_type==2'b01)//If slave_port is 8
          if(rg_port_count==0)begin
            response_buff<=response.data;
            if(response.berr==1'b1)
            response_berr<=1'b1;
            rg_port_count<=rg_port_count+1;
          end
          else begin	
            ff_req.deq();

             //checking last word for burst_transfers
            let r = AXI4_Rd_Data {rresp: AXI4_OKAY,rdata:duplicate({response.data[7:0],
            response_buff[7:0]}),rlast:False,ruser: 0,rid: ff_id.first };
            if(endian(ff_address.first,response.endian_big)==Big)
            r = AXI4_Rd_Data {rresp: AXI4_OKAY, rdata:duplicate({response_buff[7:0],
            response.data[7:0]}),rlast:False,ruser: 0,rid: ff_id.first };
            if(response.berr==1'b1||response_berr==1'b1)
            r.rresp = AXI4_SLVERR;

          //checking last word for burst_transfers
            if(ff_last.first)
            begin
              r.rlast=True;
              ff_id.deq;
            end
            r.rid=ff_id.first;
            ff_last.deq;
            rg_port_count<=0;
            ff_address.deq();
            s_xactor.i_rd_data.enq(r);
            response_buff<=0;
            response_berr<=0;

          end

			else begin
					ff_req.deq();
					rg_port_count<=0;
					response_buff<=0;
          response_berr<=0;
          let r = AXI4_Rd_Data{rresp: AXI4_OKAY,rdata:duplicate(response.data[15:0]),
          rlast:False,ruser: 0,rid: ff_id.first };

          //checking last word for burst_transfers
          if(ff_last.first)
          begin
            r.rlast=True;
            ff_id.deq;
          end
          r.rid=ff_id.first;
          ff_last.deq;

    			if(response.berr==1'b1)
					r.rresp = AXI4_SLVERR;
					ff_address.deq();
          s_xactor.i_rd_data.enq(r);
          `logLevel( mcpu, 1, $format("MCPU:Data received %h with id %h to mem_stage",response.data,fshow(ff_id.first)))

				end
	endrule

  rule send_read_response_from_memory_to_mem_stage_32(ff_req.first==DATA_MODE_32_READ); 
			let response =proc_master.put_resp();
      if(response.port_type==2'b00) begin		
			  ff_req.deq();
        let r = AXI4_Rd_Data{rresp: AXI4_OKAY, rdata:duplicate({response.data}),rlast:False,ruser: 0,
        rid: ff_id.first };
    		if(response.berr == 1'b1)
			  r.rresp = AXI4_SLVERR;
        
        //checking last word for burst_transfers
        if(ff_last.first)
        begin
          r.rlast=True;
          ff_id.deq;
        end
        r.rid=ff_id.first;
        ff_last.deq;
				
        ff_address.deq;
        s_xactor.i_rd_data.enq(r);
       `logLevel( mcpu, 1, $format("MCPU:Data received %h with id %h to mem_stage",response.data,fshow(ff_id.first)))
      end

			// SLAVE_PORT is 16 bit
      else if(response.port_type==2'b10)
			if(rg_port_count==0)
			begin
				response_buff<=response.data;
        if(response.berr==1'b1)
        response_berr<=1'b1;
				rg_port_count<=rg_port_count+1;
			end
			else
			begin	
				ff_req.deq();
				rg_port_count<=0;
				response_buff<=0;
        let r = AXI4_Rd_Data{rresp: AXI4_OKAY, rdata:duplicate({response.data[15:0],
        response_buff[15:0]}),rlast:False,ruser: 0,rid: ff_id.first };
	      if(endian(ff_address.first,response.endian_big)==Big)
	      r = AXI4_Rd_Data{rresp: AXI4_OKAY,rdata:duplicate({response_buff[15:0],
        response.data[15:0]}),rlast:False,ruser:0,rid: ff_id.first };
        
        //checking last word for burst_transfers
        if(ff_last.first)
        begin
          r.rlast=True;
          ff_id.deq;
        end
        r.rid=ff_id.first;
        ff_last.deq;
      	
        if(response.berr==1'b1||response_berr==1'b1)
				r.rresp = AXI4_SLVERR;
				ff_address.deq;
        response_berr<=1'b0;
        s_xactor.i_rd_data.enq(r);
       `logLevel( mcpu, 1, $format("MCPU:Data received %h with id %h to mem_stage",r.rdata,fshow(ff_id.first)))

			end

			else if(response.port_type==2'b01)
			begin
			if(rg_port_count<3)
				begin
					if(endian(ff_address.first,response.endian_big) == Big )begin
          if(response.berr==1'b1)
            response_berr<=1'b1;
					  response_buff<={response_buff[23:0],response.data[7:0]};
          end
          else begin
          if(response.berr==1'b1)
            response_berr<=1'b1;
          response_buff<={response.data[7:0],response_buff[31:8]};
          end
					rg_port_count<=rg_port_count+1;
				end
		  else
				begin
					ff_req.deq();
					rg_port_count<=0;
					response_buff<=0;
          response_berr<=0;
          let r = AXI4_Rd_Data{rresp: AXI4_OKAY, rdata:duplicate({
          response.data[7:0],response_buff[31:8]}),rlast:False,ruser:0,rid: ff_id.first };
	        if(endian(ff_address.first,response.endian_big)==Big)
          r = AXI4_Rd_Data{rresp: AXI4_OKAY,rdata:duplicate({response_buff[23:0],response.data[7:0]}),rlast:False,ruser: 0,rid: ff_id.first };
    			if(response.berr==1'b1||response_berr==1'b1)    		
					r.rresp = AXI4_SLVERR;
      		ff_address.deq;

          //checking last word for burst_transfers
          if(ff_last.first)
          begin
            r.rlast=True;
            ff_id.deq;
          end
          r.rid=ff_id.first;
          ff_last.deq;
         `logLevel( mcpu, 1, $format("MCPU:Data received %h with id %h to mem_stage",r.rdata,fshow(ff_id.first)))
					s_xactor.i_rd_data.enq(r);
				end
      end
		endrule

  rule send_read_response_from_memory_to_mem_stage_64_READ1(ff_req.first==DATA_MODE_64_READ1); 
			let response = proc_master.put_resp();
      if(response.port_type==2'b00) begin		
			  ff_req.deq();
        data_buff<=response.data;
    		if(response.berr == 1'b1)
			  err_buff <= True;
				ff_address.deq;

			end
			// SLAVE_PORT is 16 bit
      else if(response.port_type==2'b10)
			if(rg_port_count==0)
			begin
				response_buff<=response.data;
				rg_port_count<=rg_port_count+1;
        if(response.berr==1'b1)
        response_berr<=1'b1;
			end
			else
			begin	
				ff_req.deq();
				rg_port_count<=0;
				response_buff<=0;
	      if(endian(ff_address.first,response.endian_big)==Big)
	      data_buff <= {response_buff[15:0],response.data[15:0]};
        else
        data_buff<={response.data[15:0],response_buff[15:0]};
      	if(response.berr==1'b1||response_berr==1'b1)
		    err_buff<=True;
				ff_address.deq;
			end

			else if(response.port_type==2'b01)
			begin
			if(rg_port_count<3)
				begin
					if(endian(ff_address.first,response.endian_big) == Big )
          begin
					 response_buff<={response_buff[23:0],response.data[7:0]};
      	   if(response.berr==1'b1)
		       response_berr<=1'b1;
          end
          else begin
        	 response_buff<={response.data[7:0],response_buff[31:8]};
      	   if(response.berr==1'b1)
		       response_berr<=1'b1;
          end
					 rg_port_count<=rg_port_count+1;
				end
		  else
				begin
					ff_req.deq();
					rg_port_count<=0;
					response_buff<=0;
          response_berr<=0;
	        if(endian(ff_address.first,response.endian_big)==Big)
          data_buff<={response_buff[23:0],response.data[7:0]};
          else
          data_buff<={response.data[7:0],response_buff[31:8]};
    			if(response.berr==1'b1||response_berr==1'b1)    		
				  err_buff<=True; 
					ff_address.deq;
				end
      end
		endrule

  rule send_read_response_from_memory_to_mem_stage_64_READ2(ff_req.first==DATA_MODE_64_READ2); 
			let response =proc_master.put_resp();
      if(response.port_type==2'b00) begin		
			  ff_req.deq();
        let r = AXI4_Rd_Data{rresp: AXI4_OKAY, rdata:{response.data,data_buff},rlast:False,ruser: 0,
        rid: ff_id.first };
        if(endian((ff_address.first),True)==Big)
        r = AXI4_Rd_Data{rresp: AXI4_OKAY, rdata:duplicate({data_buff,response.data}),rlast:False,ruser: 0,
        rid: ff_id.first };
    		if(response.berr == 1'b1||err_buff)
			  r.rresp = AXI4_SLVERR;
				ff_address.deq;

        //checking last word for burst_transfers
        if(ff_last.first)
        begin
          r.rlast=True;
          ff_id.deq;
        end
        r.rid=ff_id.first;
        ff_last.deq;

        s_xactor.i_rd_data.enq(r);

        `logLevel( mcpu, 1, $format("MCPU:Data received %h with id %h to mem_stage",r.rdata,fshow(ff_id.first)))

			end

			// SLAVE_PORT is 16 bit
      else if(response.port_type==2'b10)
			if(rg_port_count==0)
			begin
				response_buff<=response.data;
				rg_port_count<=rg_port_count+1;
        if(response.berr==1'b1)
        response_berr<=1;
			end
			else
			begin	
				ff_req.deq();
				rg_port_count<=0;
				response_buff<=0;
        response_berr<=1'b0;
        let r = AXI4_Rd_Data{rresp: AXI4_OKAY, rdata:duplicate({response.data[15:0],
        response_buff[15:0],data_buff}),rlast:False,ruser: 0,rid: ff_id.first };
	      if(endian(ff_address.first,response.endian_big)==Big)
	      r = AXI4_Rd_Data{rresp: AXI4_OKAY,rdata:duplicate({data_buff,response_buff[15:0],
        response.data[15:0]}),rlast:False,ruser:0,rid: ff_id.first };
      	if(response.berr==1'b1||err_buff||response_berr==1'b1)
				r.rresp = AXI4_SLVERR;

        //checking last word for burst_transfers
        if(ff_last.first)
        begin
          r.rlast=True;
          ff_id.deq;
        end
        r.rid=ff_id.first;
        ff_last.deq;
				ff_address.deq;
        s_xactor.i_rd_data.enq(r);
        `logLevel( mcpu, 1, $format("MCPU:Data received %h with id %h to mem_stage",r.rdata,fshow(ff_id.first)))

			end

			else if(response.port_type==2'b01)
			begin
			if(rg_port_count<3)
				begin
					if(endian(ff_address.first,response.endian_big) == Big )begin
            if(response.berr==1'b1)
              response_berr<=1;
					  response_buff<={response_buff[23:0],response.data[7:0]};
          end
          else begin
					response_buff<={response.data[7:0],response_buff[31:8]};
          if(response.berr==1'b1)
              response_berr<=1;
          end
					rg_port_count<=rg_port_count+1;
				end
		  else
				begin
					ff_req.deq();
					rg_port_count<=0;
					response_buff<=0;
          response_berr<=0;
          let r = AXI4_Rd_Data{rresp: AXI4_OKAY, rdata:duplicate({
          response.data[7:0],response_buff[31:8],data_buff}),rlast:False,ruser:0,rid: ff_id.first };
	        if(endian(ff_address.first,response.endian_big)==Big)
          r = AXI4_Rd_Data{rresp: AXI4_OKAY,rdata:duplicate({data_buff,response_buff[23:0],response.data[7:0]}),rlast:False,ruser: 0,rid: ff_id.first };
    			if(response.berr==1'b1||err_buff||response_berr==1'b1)    		
					r.rresp = AXI4_SLVERR; 
					ff_address.deq;


          //checking last word for burst_transfers
          if(ff_last.first)
          begin 
            r.rlast=True;
            ff_id.deq;
          end
          r.rid=ff_id.first;
          ff_last.deq;
					s_xactor.i_rd_data.enq(r);
          `logLevel( mcpu, 1, $format("MCPU:Data received %h with id %h to mem_stage",r.rdata,fshow(ff_id.first)))
        
				end
      end
		endrule


		rule send_write_response_from_memory_to_mem_stage_8(ff_req.first==DATA_MODE_8_WRITE);//MEMORY WRITE RESP TO DCACHE
			let response =proc_master.put_resp();
			ff_req.deq();
		  let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY,  bid: ff_id.first};
			if (response.berr==1'b1)
			resp.bresp=AXI4_SLVERR;
			ff_id.deq();
			ff_address.deq();
      ff_last.deq();
      if(ff_last.first)
	   	s_xactor.i_wr_resp.enq(resp);
      `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
		endrule
                
    rule send_write_response_from_memory_to_mem_stage_16(ff_req.first==DATA_MODE_16_WRITE);
			let response =proc_master.put_resp();
			if(response.port_type==2'b10 )begin
			  ff_req.deq();
		   	ff_id.deq();

		    let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
			  ff_address.deq();
			  if (response.berr==1'b1)
			  resp.bresp=AXI4_SLVERR;
         
        ff_last.deq();
        if(ff_last.first)
	    	s_xactor.i_wr_resp.enq(resp);
        `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
			end	

			else if(response.port_type==2'b01)
				if(rg_port_count==0)begin
					rg_port_count<=rg_port_count+1;
          if(response.berr==1'b1)
          response_berr<=1'b1;  
				end
				else begin	
				  ff_address.deq();
					ff_req.deq();
					rg_port_count<=0;
          `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
		      let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
				  if (response.berr==1'b1||response_berr==1'b1)
				  resp.bresp=AXI4_SLVERR;
          response_berr<=1'b0;
  				ff_id.deq();
          ff_last.deq();
          if(ff_last.first)
	    		s_xactor.i_wr_resp.enq(resp);
				end
			else
				begin
					ff_address.deq();
					ff_req.deq();
					rg_port_count<=0;
					response_buff<=0;
          response_berr<=1'b0;
		      let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
					if (response.berr==1'b1)
					resp.bresp=AXI4_SLVERR;
					ff_id.deq();
          ff_last.deq();
          if(ff_last.first)
	    		s_xactor.i_wr_resp.enq(resp);
					`logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
				end
		endrule

  	rule send_write_response_from_memory_to_mem_stage_32(ff_req.first==DATA_MODE_32_WRITE); 
      let response =proc_master.put_resp();
      if(response.port_type==2'b00)
      begin

        ff_req.deq();
        let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
        if (response.berr==1'b1)
        resp.bresp=AXI4_SLVERR;
        ff_id.deq();
        ff_address.deq();
        ff_last.deq();
        if(ff_last.first)
        s_xactor.i_wr_resp.enq(resp);
       `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
		end

		else if(response.port_type==2'b10)
		if(rg_port_count==0)
		begin
      if(response.berr==1'b1)
      response_berr<=1'b1;  
			rg_port_count<=rg_port_count+1;
		end
		else
		begin	
			ff_req.deq();
			ff_address.deq();
			rg_port_count<=0;
		  ff_id.deq();
			let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
			if (response.berr==1'b1||response_berr==1'b1)
			resp.bresp=AXI4_SLVERR;
      response_berr<=1'b0;
      ff_last.deq();
      if(ff_last.first)
	  	s_xactor.i_wr_resp.enq(resp);
      `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))

		end
		else if(response.port_type==2'b01)
		begin
			if(rg_port_count<3)
				begin
        if(response.berr==1'b1)
          response_berr<=1'b1;  
					rg_port_count<=rg_port_count+1;
				end
			else
				begin
				ff_req.deq();
				ff_address.deq();
				rg_port_count<=0;
	     	let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
				if (response.berr==1'b1||response_berr==1'b1)
				resp.bresp=AXI4_SLVERR;
				ff_id.deq();
        response_berr<=1'b0;
        ff_last.deq();
        if(ff_last.first)
	  		s_xactor.i_wr_resp.enq(resp);
       `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
				end
			end
		endrule

  	rule send_write_response_from_memory_to_mem_stage_64_WRITE1(ff_req.first==DATA_MODE_64_WRITE1); 
      let response =proc_master.put_resp();
      if(response.port_type==2'b00)
      begin		
        ff_req.deq();
        let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
        if(response.berr==1'b1)
        err_buff<=True;
        ff_address.deq();
		end

		else if(response.port_type==2'b10)
		if(rg_port_count==0)
		begin
      if(response.berr==1'b1)
      response_berr<=1;
			rg_port_count<=rg_port_count+1;
       
		end
		else
		begin	
			ff_req.deq();
			ff_address.deq();
			rg_port_count<=0;
      if(response.berr==1'b1||response_berr==1'b1)
      err_buff<=True;
      response_berr<=1'b0;

		end
		else if(response.port_type==2'b01)
		begin
			if(rg_port_count<3)
				begin
          if(response.berr==1'b1)
            response_berr<=1;
          rg_port_count<=rg_port_count+1;
				end
			else
				begin
				ff_req.deq();
				ff_address.deq();
        if(response.berr==1'b1||response_berr==1'b1)
         err_buff<=True;
         response_berr<=1'b0;
				rg_port_count<=0;
				end
			end
		endrule

  	rule send_write_response_from_memory_to_mem_stage_64_WRITE2(ff_req.first==DATA_MODE_64_WRITE2); 
      let response =proc_master.put_resp();
      if(response.port_type==2'b00)
      begin		
        ff_req.deq();
        let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
        if (response.berr==1'b1||err_buff)
        resp.bresp=AXI4_SLVERR;
        ff_id.deq();
        ff_address.deq();
        response_berr<=1'b0;
        if(ff_last.first)
        s_xactor.i_wr_resp.enq(resp);
       `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
        ff_last.deq;
      end

      else if(response.port_type==2'b10)
      if(rg_port_count==0)
      begin
        if(response.berr==1'b1)
        response_berr<=1'b1;
        rg_port_count<=rg_port_count+1;
      end
      else
      begin	
        ff_req.deq();
        ff_address.deq();
        rg_port_count<=0;
        ff_id.deq();
        let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
        if (response.berr==1'b1||err_buff||response_berr==1'b1)
        resp.bresp=AXI4_SLVERR;
        response_berr<=1'b0;
        if(ff_last.first)
        begin
        s_xactor.i_wr_resp.enq(resp);
       `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
        end

        ff_last.deq;
      end

      else if(response.port_type==2'b01)
      begin
        if(rg_port_count<3)
          begin
            if(response.berr==1'b1)
            response_berr<=1'b1;
            rg_port_count<=rg_port_count+1;
          end
        else
          begin
          ff_req.deq();
          ff_address.deq();
          rg_port_count<=0;
          response_berr<=1'b0;
          let resp =  AXI4_Wr_Resp {bresp: AXI4_OKAY, bid: ff_id.first};
          if (response.berr==1'b1||response_berr==1'b1||err_buff)
          resp.bresp=AXI4_SLVERR;
          ff_id.deq();
          if(ff_last.first)
          s_xactor.i_wr_resp.enq(resp);
          ff_last.deq;
          `logLevel( mcpu, 1, $format("MCPU:Received Write Response: from address %h to",fshow(ff_address.first)))
          end
        end
      err_buff<=False;
      endrule	

		interface proc_ifc  = proc_master.mcpu_interface;
		interface proc_dbus = proc_master.data_bus;
	  interface slave_axi_mcpu = s_xactor.axi_side;


endmodule
endpackage
