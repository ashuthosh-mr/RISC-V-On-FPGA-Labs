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
This module
1.Generates mcpu master bus signals
2.Controls data alignment according to endianness of the slave
3.Manages dynamic bus_sizing
 */

package mcpu_master;
//import TriState ::*;
import FIFOF ::*;
import mcpu_defines ::*;
`include "Logger.bsv"
//import Clocks ::*;
/*
   READ CYCLE
   1.STATE 0  : receives req (function codes,r/w,siz1,siz0,addrbus) latched

   2.STATE 1  : assert address strobe and data strobe

   3. STATE 2 : Wait

   4. STATE 3 : Check for acknowledgement

   5. STATE 4 : latch data

   6. STATE 5:  Release address strobe and data strobe(acknowledge lines will be released one cycle later)

   WRITE CYCLE


   1.STATE 0 : receives req (function codes,r/w,siz1,siz0,addrbus) latched

   2.STATE 1 : assert address strobe

   3. STATE 2 : place data on the bus and wait for acknowledgement

   4. STATE 3 : assert data and strobe and enable required data lines

   5. STATE 4 : No operation

   6. STATE 5:Release address strobe and data strobe(acknowledge lines will be released one cycle later)READ CYCLE

Data_mode :01 byte
10 Word
00 Long word
 */

  typedef enum
  {RCV_REQ,PRC_REQ_1,PRC_REQ_2,PRC_REQ_3,LATCH_DATA,END_REQ,ADDR_INVALID,HALT
  }State_master deriving (Bits,Eq);

  typedef enum
  {LW,BYTE,WORD,TRI_BYTE}Data_mode deriving (Bits,Eq);               

  interface Mcpu_out;
  (*always_enabled,always_ready*)
    method Bit#(1) wr_as_l();
  (*always_enabled,always_ready*)
    method Bit#(1) wr_ds_l();
  (*always_enabled,always_ready*)
    method Bit#(1) wr_wr_l();
  (*always_ready*)
    method Action rd_dsack_0_l(Bit#(1) x);
  (*always_ready*)
    method Action rd_dsack_1_l(Bit#(1) y);
  (*always_ready*)
    method Action rd_berr_l(Bit#(1) z);
  (*always_ready*)
    method Action rd_halt_l(Bit#(1) u);
  endinterface:Mcpu_out

  interface Data_bus_inf;

  method Bit#(8) wr_byte_31_24();
  (*always_enabled,always_ready*)
    method Bit#(1) wr_siz1();
  (*always_enabled,always_ready*)
    method Bit#(1) wr_siz0();
    method Bool wr_mode_en();
  (*always_enabled,always_ready*)
    method Bit#(32) wr_addr();
    method Bool wr_addr_en();
    method Bit#(4)  wr_en();

    method Bit#(3) wr_fun_code();
    method Bool wr_fun_code_en();
    method Bit#(8) wr_byte_23_16();
    method Bit#(8) wr_byte_15_8();
    method Bit#(8) wr_byte_7_0();

    method Action rd_byte_31_24(Bit #(8) d3);
    method Action rd_byte_23_16(Bit #(8) d2);
    method Action rd_byte_15_8 (Bit #(8) d1);
    method Action rd_byte_7_0  (Bit #(8) d0);

    endinterface

    interface Mcpu_master ;
      method Action get_req (Req_mcpu req);
      interface Mcpu_out mcpu_interface;
      interface Data_bus_inf data_bus;
      method  Resp_mcpu put_resp();
    endinterface:Mcpu_master

    typedef enum{Little,Big} End deriving (Eq, Bits);



  (*synthesize*)

    module mkmcpumaster(Mcpu_master);



    String mcpu_master = " ";
    //Fifos to get request and send response
    FIFOF #(Req_mcpu) ff_cpu_req <- mkFIFOF;
    FIFOF #(Resp_mcpu) ff_cpu_resp <- mkFIFOF;
    //............................................................//
    Reg #(State_master) rg_master_state <- mkRegA (RCV_REQ);
    Reg#(Bit#(2)) rg_mode <- mkReg(0);
    Reg#(Bool) rg_mode_en <- mkReg(False);//Enable SIZ1,SIZ0
    Reg#(Bit#(3)) rg_fun_code <- mkReg(0);
    Reg#(Bool) rg_fun_code_en <- mkReg(False);
    Reg#(Bit#(1))  rg_as_l <- mkReg(1);//address_stop
    Reg#(Bit#(1))  rg_ds_l <- mkReg(1);//data_strobe
    Reg#(Bit#(1))  rg_stop <- mkReg(0);//Indicates processor stopped
    Reg#(Bit#(1))  rg_retry<- mkReg(0);//Indicates retry mode
    //............................................................//
    Wire#(Bit#(1)) dsack_0_l<-mkDWire(1);//ack from VIC controller,indicates port size 0-32 bit port,1-8 bit port,2-16 bit port
    Wire#(Bit#(1)) dsack_1_l<-mkDWire(1);
    Wire#(Bit#(1)) berr_l <-mkDWire(1);//Bus error from slave
    Wire#(Bit#(1)) halt_l<-mkDWire(1);//halt signal from the slave
    //Wire#(Bit#(3)) ipl_l<-mkDWire(1);//interrupt prioritylevel

    Reg #(Bit#(32))  rg_addr       <- mkReg(0);//addr_bus
    Reg #(Bool)      rg_addr_en    <- mkReg(False);
    Reg #(Bit#(8))   rg_data_out_4 <- mkReg(0);//wr_data_4
    Reg #(Bit#(8))   rg_data_out_3 <- mkReg(0);//wr_data_3
    Reg #(Bit#(8))   rg_data_out_2 <- mkReg(0);//wr_data_2
    Reg #(Bit#(8))   rg_data_out_1 <- mkReg(0);//wr_data_1

    Wire #(Bit#(8)) rg_data_in_4 <- mkDWire(0);//rd_data_4
    Wire #(Bit#(8)) rg_data_in_3 <- mkDWire(0);//rd_data_3
    Wire #(Bit#(8)) rg_data_in_2 <- mkDWire(0);//rd_data_2
    Wire #(Bit#(8)) rg_data_in_1 <- mkDWire(0);//rd_data_1
    Wire #(Req_mcpu) mcpu_req    <- mkWire();
    Wire #(Resp_mcpu) mcpu_resp  <- mkWire();

    Reg #(Bool)     rg_sram_big <- mkReg(False);
    Reg #(Bit#(1))  rg_wr_l     <- mkReg(1'd1);//read : 1,write:0
    Reg #(Bit#(4))  rg_data_control <-mkReg(0);//Enable bits to data and control registers
    Reg #(Bit#(2))  rg_cntl_wd<-mkReg(0); //To synchronize if more than one cycle is required to r/w data

    //............Tristate signals...............//
    /*
     */
    (* mutually_exclusive = "rcv_req_new,prc_req,prc_req_1,prc_req_2,end_req" *)

    /* REQ_RCV rg_master_state....
       1. Master receives request
       2. address,FC2-FC0,Data,Write lines are driven
     */
    rule rcv_req_new(rg_master_state == RCV_REQ && rg_cntl_wd == 2'b00 && (halt_l == 1'b1) &&
    (berr_l==1'b1)) ;//To recieve new request
      let req =mcpu_req;

      `logLevel( mcpu_master, 1, $format("MCPU_MASTER:Request received to address %h req_type\
      %h",req.addr,req.rd_req))
      `logLevel( mcpu_master, 1, $format("MCPU_MASTER:funcode: %h mode of operation:\
      %h",req.fun_code,req.mode))

     // ff_cpu_req.deq();
      rg_addr <= req.addr;
      rg_mode <= req.mode;
      rg_addr_en <= True;
      rg_fun_code_en <= True;
      rg_mode_en <= True;
      rg_fun_code <= req.fun_code;
      rg_wr_l <= req.rd_req;
      rg_master_state <= PRC_REQ_1;
      rg_sram_big <= req.endian_big;

//............Multiplexing logic to route data to data bus......................................................................................................

     if (req.mode==2'b01)//For 8 bit data operations
       case({req.addr[1],req.addr[0]})
        2'b00:rg_data_out_4 <= req.wr_data[7:0];
        2'b01:begin
        rg_data_out_4 <= req.wr_data[7:0];
        rg_data_out_3 <= req.wr_data[7:0];
        end	
        2'b10:
        begin
        rg_data_out_4 <= req.wr_data[7:0];
        rg_data_out_2 <= req.wr_data[7:0];
        end	
        2'b11:
        begin 
        rg_data_out_4 <= req.wr_data[7:0];
        rg_data_out_3 <= req.wr_data[7:0];
        rg_data_out_1 <= req.wr_data[7:0];
        end
       endcase


     else  if (req.mode==2'b10)//For 16 data operations
      case({req.addr[1],req.addr[0]})
        2'b00:
          if (endian(req.addr,req.endian_big)==Little)begin
            rg_data_out_4 <= req.wr_data[7:0];
            rg_data_out_3 <= req.wr_data[15:8];
          end
          else begin
            rg_data_out_4 <= req.wr_data[15:8];
            rg_data_out_3 <= req.wr_data[7:0];
          end
        2'b10: 
          if(endian(req.addr,req.endian_big)==Little)begin
              rg_data_out_4 <= req.wr_data[7:0];
              rg_data_out_3 <= req.wr_data[15:8];
              rg_data_out_2 <= req.wr_data[7:0];
              rg_data_out_1 <= req.wr_data[15:8];
          end
          else begin
              rg_data_out_4 <= req.wr_data[15:8];
              rg_data_out_3 <= req.wr_data[7:0];
              rg_data_out_2 <= req.wr_data[15:8];
              rg_data_out_1 <= req.wr_data[7:0];
            end
      endcase

      else  if (req.mode==2'b00)//for 32 bit operation
        case({req.addr[1],req.addr[0]})
          2'b00:
          begin
            if(endian(req.addr,req.endian_big)==Little)
              begin
              rg_data_out_1 <= req.wr_data[31:24];
              rg_data_out_2 <= req.wr_data[23:16];
              rg_data_out_3 <= req.wr_data[15:8];
              rg_data_out_4 <= req.wr_data[7:0];
              end
            else
              begin		
              rg_data_out_4 <= req.wr_data[31:24];
              rg_data_out_3 <= req.wr_data[23:16];
              rg_data_out_2 <= req.wr_data[15:8];
              rg_data_out_1 <= req.wr_data[7:0];
              end
         end
        endcase
  //..............................................................................................................................................................
    endrule

    rule prc_req(rg_master_state==RCV_REQ && rg_cntl_wd!=2'b00);//If a request takes multiple cycles
      if(rg_cntl_wd==2'b01)//32 bit operation from 8 bit or 16 bit slave
      begin	
      rg_data_out_4 <= rg_data_out_3;
      rg_data_out_3 <= rg_data_out_2;
      rg_data_out_2 <= rg_data_out_1;
      rg_data_out_1 <= 8'b0;
      rg_addr     <= rg_addr+1;
      rg_mode     <= rg_mode-1;
      end
      else
      begin//32 bit operation on a 16 bit slave
      rg_data_out_4 <= rg_data_out_2;
      rg_data_out_3 <= rg_data_out_1;
      rg_data_out_2 <= 8'b0;
      rg_data_out_1 <= 8'b0;
      rg_addr     <= rg_addr+2;
      rg_mode     <= 2'b10;
      end
      rg_master_state <= PRC_REQ_1;
    endrule
    /*
       InPRC_REQ_1....

       1.Address strobe is asserted

       2.Data Strobe is asserted if it is a read cycle

       3. Data lines are enabled for write cycle

     */

    rule prc_req_1(rg_master_state==PRC_REQ_1);
      `logLevel( mcpu_master, 1, $format("MASTER_STATE 2: Activating address strobe"))
      rg_as_l <= 0;
      if(rg_wr_l==1'b1)
      begin
        rg_ds_l <= 0;
        if(({dsack_0_l,dsack_1_l} != 2'b11)||(berr_l == 1'b0)||(halt_l == 1'b0))
        rg_master_state <= LATCH_DATA;
      end
      else 
        rg_master_state <= PRC_REQ_2;
        if(rg_wr_l==1'b0)
        rg_data_control <= 4'b1111;
    endrule
    /*
       In PRC_REQ_2
       2.Data_strobe is asserted for write cycle
       3.On acknowledgement goes to latch data state
     */
    
    rule prc_req_2(rg_master_state==PRC_REQ_2);		
    begin
      `logLevel(mcpu_master,1,$format("MASTER_STATE_3 :Activating data strobe write"))
       rg_ds_l<=1'b0;
       if(({dsack_0_l,dsack_1_l} != 2'b11)||(berr_l == 1'b0)||(halt_l == 1'b0))
        rg_master_state<=LATCH_DATA;
    end
    endrule


//.......................Latch_data when not using tristatebuffers.............................//
//........................In LATCH DATA state................................................../
//........................Latches on-to data and response.....................................//

    rule latch_data(rg_master_state==LATCH_DATA);
    
      //setting control bits for multicycle transfers
      if({dsack_0_l,dsack_1_l} == rg_mode) begin
        rg_cntl_wd <= 2'b00;//Done can take the next request
      end
      //if a 32 bit word r/w  fro/to  a 16 bit port      
      else if({dsack_0_l,dsack_1_l} == 2'b10 && rg_mode == 2'b00) begin	
        rg_cntl_wd <= 2'b10;
      end
      // If a 32 bit word or 1 16 bit is  r/w  fro/to  a 8 bit port   	 	
      else if({dsack_0_l,dsack_1_l} == 2'b01 && rg_mode != 2'b01)begin
        rg_cntl_wd <= 2'b01;
      end

      if(berr_l==1'b0 && halt_l==1'b0) begin
          rg_retry<=1;
      end
      else if(berr_l!=1'b0 && halt_l==1'b0) begin
          rg_stop<=1;
      end
      else if(berr_l==1'b0) begin			
          let resp_data = Resp_mcpu{endian_big:rg_sram_big,data:{rg_data_in_4,rg_data_in_3,rg_data_in_2,8'b0},
          berr:1};
          mcpu_resp<=resp_data;
      end
      else
      if (rg_mode==2'b01 )//Receiving 8 bit data
      case({dsack_0_l,dsack_1_l})
//......................................SLAVE PORT..............................//:-32 byte
//.......................................................................................
       2'b00 :
       case({rg_addr[1],rg_addr[0]})//Position where byte read from depends on A1,A0          
           2'b00 :
            begin
              let resp_data = Resp_mcpu{endian_big:rg_sram_big,  data:{24'b0,
              rg_data_in_4},berr:0,port_type:{dsack_0_l,dsack_1_l}};
              mcpu_resp<=resp_data; 
             end
           2'b01 :
            begin
              let resp_data = Resp_mcpu{endian_big:rg_sram_big,  data:{24'b0,
              rg_data_in_3},berr:0,port_type:{dsack_0_l,dsack_1_l}};
              mcpu_resp<=resp_data;
            end
          2'b10 :
          begin
             let resp_data = Resp_mcpu{endian_big:rg_sram_big,  data:{24'b0,
             rg_data_in_2},berr:0,port_type:{dsack_0_l,dsack_1_l}};
             mcpu_resp<=resp_data;
          end
          2'b11 :
          begin
             let resp_data = Resp_mcpu{endian_big:rg_sram_big,  data:{24'b0,
             rg_data_in_1},berr:0,port_type:{dsack_0_l,dsack_1_l}};
             mcpu_resp<=resp_data;
          end
      endcase

      //If slave is an 8 bit port		
        2'b01  : 
        begin
          let resp_data = Resp_mcpu{endian_big:rg_sram_big, data:{24'b0,rg_data_in_4
          },berr:0,port_type:{dsack_0_l,dsack_1_l}};
          mcpu_resp<=resp_data;
        end
        //if slave is a 16 bit port
        2'b10 :
        if (rg_addr[0]==1'b0)begin
            let resp_data =Resp_mcpu{endian_big:rg_sram_big,  data:{24'b0,rg_data_in_4
            },berr:0,port_type:{dsack_0_l,dsack_1_l}};      
            mcpu_resp<=resp_data;
        end
        else
        begin
          let resp_data =Resp_mcpu{endian_big:rg_sram_big,  data:{24'b0,rg_data_in_3
          },berr:0,port_type:{dsack_0_l,dsack_1_l}};      
          mcpu_resp<=resp_data;
        end

        endcase

        else
        if(rg_mode==2'b10)//Receiving 16 bit data
        
        case({dsack_0_l,dsack_1_l})
        //..........................SLAVE PORT......................................32 byte
          2'b00 :
              case({rg_addr[1],rg_addr[0]})//Position where byte read from depends on A1,A0          
                  
                  2'b00 :
                  begin
                   let resp_data = Resp_mcpu{endian_big:rg_sram_big,  data:{16'b0,rg_data_in_3,
                   rg_data_in_4},berr:0,port_type:{dsack_0_l,dsack_1_l}};
                   if(endian(rg_addr,rg_sram_big)==Big)
                   resp_data = Resp_mcpu{endian_big:rg_sram_big,data:{16'b0,rg_data_in_4,
                   rg_data_in_3},berr:0,port_type:{dsack_0_l,dsack_1_l}};
                   mcpu_resp<=resp_data;
                  end

                  2'b10 :
                  begin
                   let resp_data = Resp_mcpu{endian_big:rg_sram_big,data:{16'b0,rg_data_in_1,
                   rg_data_in_2},berr:0,port_type:{dsack_0_l,dsack_1_l}};
                   if (endian(rg_addr,rg_sram_big)==Big)
                   resp_data = Resp_mcpu{endian_big:rg_sram_big,data:{16'b0,rg_data_in_2,
                   rg_data_in_1},berr:0,port_type:{dsack_0_l,dsack_1_l}};
                   mcpu_resp<=resp_data;
                  end

              endcase

        //If slave is an 8 bit port		
          2'b01  :
          begin
            let resp_data = Resp_mcpu{endian_big:rg_sram_big, data:{24'b0,rg_data_in_4},berr:0,port_type:
            {dsack_0_l,dsack_1_l}};
            mcpu_resp<=resp_data;
          end

          //if slave is a 16 bit port

          2'b10 :

          begin
            let resp_data =Resp_mcpu{endian_big:rg_sram_big,  data:{16'b0,rg_data_in_3,rg_data_in_4},
            berr:0,port_type:{dsack_0_l,dsack_1_l}};      
            if (endian(rg_addr,rg_sram_big)==Big)
            resp_data = Resp_mcpu{endian_big:rg_sram_big,  data:{16'b0,rg_data_in_4,rg_data_in_3},
            berr:0,port_type:{dsack_0_l,dsack_1_l}};
            mcpu_resp<=resp_data;
          end

          endcase

        else
          if(rg_mode==2'b00)//Receiving 32 bit data
          case({dsack_0_l,dsack_1_l})
          //...........SLAVE PORT....................:-32 bit port..............................//
          2'b00 :
          begin
            let resp_data = Resp_mcpu{endian_big:rg_sram_big,data:{rg_data_in_1,rg_data_in_2,rg_data_in_3,rg_data_in_4},
            berr:0,port_type:{dsack_0_l,dsack_1_l}};
            if (endian(rg_addr,rg_sram_big)==Big)
            resp_data = Resp_mcpu{endian_big:rg_sram_big,
            data:{rg_data_in_4,rg_data_in_3,rg_data_in_2,rg_data_in_1},berr:0,port_type:{dsack_0_l,dsack_1_l}};
            mcpu_resp<=resp_data; 
          end
        //.......................................16 bit port....................................//	
        2'b10 :
        begin
            let resp_data = Resp_mcpu{endian_big:rg_sram_big,data:{16'b0,rg_data_in_3,rg_data_in_4},
            berr:0,port_type:{dsack_0_l,dsack_1_l}};
            if(endian(rg_addr,rg_sram_big)==Big)
            resp_data = Resp_mcpu{endian_big:rg_sram_big,data:{16'b0,rg_data_in_4,rg_data_in_3},
            berr:0,port_type:{dsack_0_l,dsack_1_l}};
            mcpu_resp<=resp_data;
        end

        //........................If slave is an 8 bit port......................................//		
        2'b01  : 
        begin
            let resp_data = Resp_mcpu{endian_big:rg_sram_big, data:{24'b0,rg_data_in_4},
            berr:0,port_type:{dsack_0_l,dsack_1_l}};
            mcpu_resp<=resp_data;
        end

        endcase

        else
        begin
            let resp_data = Resp_mcpu{endian_big:rg_sram_big,
            data:{24'b0,rg_data_in_4},berr:0,port_type:{dsack_0_l,dsack_1_l}};
            mcpu_resp<=resp_data;
        end
        rg_master_state<=END_REQ;
    endrule
  /*
     1.Releases address and data strobe and data bus

     2. DSACK lines are deasserted one cycle after

     One more stage should be inserted here
   */

    rule end_req(rg_master_state==END_REQ);
      `logLevel( mcpu_master, 1, $format("MASTER_STATE 6:Releasing data strobes"))
      rg_ds_l<=1;
      rg_as_l<=1;
      rg_data_control<=4'b0000;
      rg_master_state<=ADDR_INVALID; 
    endrule	

    rule rel_addr(rg_master_state==ADDR_INVALID);//Release the address and data bus when transfer is finished
      if(rg_stop==0 && rg_retry==0)begin
        if(((dsack_0_l!=0)&&(dsack_1_l!=0))||(berr_l!=1'b0))
        rg_master_state<=RCV_REQ;
        if(rg_cntl_wd==2'b00) begin
          `logLevel( mcpu_master, 1, $format("MASTER_STATE 7:Releasing data strobes"))
          rg_addr_en<=False;
          rg_mode_en<=False;
          rg_fun_code_en<=False;
        end
        $display("Ready to receive a new request",$time);
      end
      else
      rg_master_state<=HALT;
    endrule


    rule stop_wait(rg_master_state==HALT);
    //If HALT.....resume when halt signal is negated
      if(rg_stop==1)
        if (halt_l!=0)begin
        rg_stop<=0;
        rg_master_state<=RCV_REQ;
       // $display("\tMASTER_STATE_7:Releasing address strobes",$time);
        rg_addr_en<=False;
        rg_mode_en<=False;
        rg_fun_code_en<=False;
      end
      else 
        rg_master_state<=HALT;
      //If Retry.........start retry cycle when bus error and halt are negated
      else
      if(halt_l!=0 && berr_l!=0)
      begin
        rg_retry<=0;
        rg_master_state<=PRC_REQ_1;
      end
      else
        rg_master_state<=HALT;
    endrule

    //Methods for driving signals in and out of master


    interface Mcpu_out mcpu_interface;
    
    method Bit#(1) wr_as_l();
    return rg_as_l;
    endmethod

    method Bit#(1) wr_ds_l();
    return rg_ds_l;
    endmethod


    method Bit#(1) wr_wr_l();
    return rg_wr_l;
    endmethod


    method Action rd_dsack_0_l(Bit#(1) x);
    dsack_0_l<=x;
    endmethod

    method Action rd_dsack_1_l(Bit#(1) y);
    dsack_1_l<=y;
    endmethod

    method Action rd_berr_l(Bit#(1) z);
    berr_l<=z;
    endmethod

    method Action rd_halt_l(Bit#(1) u);
    halt_l<=u;
    endmethod
    endinterface

    /*Methods to emulate tristate functionality*/

    interface Data_bus_inf data_bus;


    method Bit#(1) wr_siz0();
    return rg_mode[0];
    endmethod

    method Bit#(1) wr_siz1();
    return rg_mode[1];
    endmethod

    method Bit#(32) wr_addr();
    return rg_addr;
    endmethod

    method Bit#(8) wr_byte_31_24();
    return rg_data_out_4;
    endmethod

    method Bit#(8) wr_byte_23_16();
    return rg_data_out_3;
    endmethod

    method Bit#(8) wr_byte_15_8();
    return rg_data_out_2;
    endmethod

    method Bit#(3) wr_fun_code();
    return rg_fun_code;
    endmethod

    method Bit#(8) wr_byte_7_0();
    return rg_data_out_1;
    endmethod

    method Action rd_byte_31_24(Bit #(8) d4);
    rg_data_in_4<=d4;
    endmethod

    method Action rd_byte_23_16(Bit #(8) d3);
    rg_data_in_3<=d3;
    endmethod

    method Action rd_byte_15_8 (Bit #(8) d2);
    rg_data_in_2<=d2;
    endmethod

    method Action rd_byte_7_0  (Bit #(8) d1);
    rg_data_in_1<=d1;
    endmethod


    method Bit#(4) wr_en();
    return rg_data_control;
    endmethod

    method Bool wr_mode_en();
    return rg_mode_en;
    endmethod


    method Bool wr_addr_en();
    return rg_addr_en;
    endmethod


    method Bool wr_fun_code_en();
    return rg_fun_code_en;
    endmethod


    endinterface

    method Action get_req(Req_mcpu req)if(rg_master_state==RCV_REQ && rg_cntl_wd == 2'b00); 
    mcpu_req<=req;
    endmethod

    method   Resp_mcpu put_resp();
    return mcpu_resp;
    endmethod



    endmodule

    endpackage






















