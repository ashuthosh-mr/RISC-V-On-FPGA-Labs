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


package sdram;
`include "sdram.defines"
import Semi_FIFOF        :: *;
import AXI4_Types   :: *;
import AXI4_Fabric  :: *;
import bsvmksdrc_top :: *;
import BUtils            ::*;
import Connectable ::*;
import ConfigReg ::*;
import DReg::*;
import FIFOF::*;
import Clocks::*;
import device_common::*;

export Ifc_sdram_out      (..);
export Ifc_sdram_axi4     (..); // interface export
export mksdram_axi4;        // module export    

interface Ifc_sdram_out#(numeric type io_width);
	(*always_enabled,always_ready*)
    method Action ipad_sdr_din(Bit#(io_width) pad_sdr_din);
    method Bit#(9) sdram_sdio_ctrl();
    method Bit#(io_width) osdr_dout();
    method Bit#(8) osdr_den_n();
    method Bool osdr_cke();
    method Bool osdr_cs_n();
    method Bool osdr_ras_n ();
    method Bool osdr_cas_n ();
    method Bool osdr_we_n ();
    method Bit#(8) osdr_dqm ();
    method Bit#(2) osdr_ba ();
    method Bit#(13) osdr_addr ();
    interface Clock sdram_clk;    
endinterface

interface Ifc_sdram_axi4#(numeric type addr_width, 
                           numeric type data_width, 
                           numeric type user_width,
                           numeric type io_width,
                           numeric type rfrsh_timer_width,
                           numeric type rfrsh_row_width);                       
      interface AXI4_Slave_IFC#(addr_width, data_width, user_width) slave_mem;
      interface AXI4_Slave_IFC#(addr_width, data_width, user_width) slave_cfg;
      interface Ifc_sdram_out#(io_width) io;
endinterface

typedef enum{
    IDLE,
    WRITE_START,
    WAIT_DELAY,
    WRITE_FIRST,
    WRITE_DATA0,
    WRITE_DATA1
} Write_state deriving(Bits, Eq, FShow);

typedef enum{
    IDLE,
    START_READ,
    READ_DATA,
    READ_FLUSH
} Read_state deriving(Bits, Eq,FShow);

typedef enum {
    IDLE,
    START_SPLIT,
    SEND_VALUE
} Write_split_states deriving(Bits, Eq, FShow);



function (Bit#(9)) fn_wr_len(Bit#(8) length, Bit#(3) awsize, Bit#(3) lwr_addr);
     Bit#(3) w_packet = 0;
     Bit#(9) s_length = 0;
 case(awsize)
        
     'd0 : begin
         w_packet = lwr_addr >> awsize;
         s_length = ((zeroExtend(length) + zeroExtend(w_packet)) >> 3) + 1;
     end
     'd1: begin
         w_packet = lwr_addr >> awsize;
         s_length = ((zeroExtend(length) + zeroExtend(w_packet)) >> 2) + 1;
 //`ifdef verbose $display($time(),"\t SSSS w_packet %b s_lenght %h length %h", w_packet, s_length, length); `endif
     end
     'd2 : begin
        //w_packet = lwr_addr >> awsize;
        // s_length = ((s_length + w_packets) >> 1) + 1;
        s_length = zeroExtend(length >> 1) + 1;
     end
     'd3 : begin
         s_length = zeroExtend(length) + 1;
     end
 endcase

 return s_length;
endfunction


function Bit#(26) fn_wr_address(Bit#(addr_width) address);
    Bit#(29) sdr_addr = address[31:3];
    return sdr_addr[25:0];
endfunction




//(*synthesize*)
    
module mksdram_axi4#(Clock clk0, Reset rst0) (Ifc_sdram_axi4#(addr_width, 
                                                      data_width, 
                                                      user_width,
                                                      io_width,
                                                      rfrsh_timer_width,
                                                      rfrsh_row_width))
                                                      provisos(
                                                         Add#(e__, 1, data_width),
                                                         Add#(g__, 2, data_width),
                                                         Add#(h__, 3, data_width),
                                                         Add#(i__, 4, data_width),
                                                         Add#(f__, 13, data_width),
                                                         Add#(a__, 8, data_width),
                                                         Add#(b__, 9, data_width),
                                                         Add#(c__, 12, data_width),
                                                         Add#(d__, 8, addr_width),
                                                         Add#(j__, data_width, 64),
                                                        Mul#(strb, 8, data_width),
                                                        Div#(data_width, 8, strb),
                                                Add#(k__, 8, TDiv#(data_width, 8)),
                                                Add#(l__, rfrsh_row_width, data_width),
                                                Add#(m__, rfrsh_timer_width, data_width),
                                                Add#(n__, 3, rfrsh_row_width),
                                                Add#(o__, 12, rfrsh_timer_width),
//                                                SizedLiteral#(Bit#(rfrsh_timer_width), 12),
//                                                SizedLiteral#(Bit#(rfrsh_row_width), 3),
                                                Add#(p__, data_width, io_width),
                                                Add#(q__, io_width, data_width)
                                                      );
    
function Bit#(data_width) fn_wr_split_data(Bit#(data_width) data, Bit#(strb) wstrb);
    Bit#(data_width) data0 = 0;
    Bit#(8) temp_data0 = 0;
    for(Integer i=0; i<valueOf(strb); i=i+1) begin
        if(wstrb[i]==1)
            temp_data0 = data[8*(i)+7:8*(i)];
            data0[8*(i)+7:8*(i)] = temp_data0; 
    end
//    if(wstrb[0] == 1)
//        data0[7:0] = data[7:0];
//    else
//        data0[7:0] = 0;
//
//    if(wstrb[1] == 1)
//        data0[15:8] = data[15:8];
//    else
//        data0[15:8] = 0;
//
//    if(wstrb[2] == 1)
//        data0[23:16] = data[23:16];
//    else
//        data0[23:16] = 0;
//
//    if(wstrb[3] == 1)
//        data0[31:24] = data[31:24];
//    else
//        data0[31:24] = 0;
//
//    if(wstrb[4] == 1)
//        data0[39:32] = data[39:32];
//    else
//        data0[39:32] = 0;
//
//    if(wstrb[5] == 1)
//        data0[47:40] = data[47:40];
//    else
//        data0[47:40] = 0;
//
//    if(wstrb[6] == 1)
//        data0[55:48] = data[55:48];
//    else
//        data0[55:48] = 0;
//
//    if(wstrb[7] == 1)
//        data0[63:56] = data[63:56];
//    else
//        data0[63:56] = 0;
//
    return data0;
endfunction

function Bit#(data_width) fn_rd_data(Bit#(3) bsize,Bit#(6) lwr_addr,Bit#(data_width) data);
    //Integer b_strb = valueOf(TLog#(strb));
    Bit#(64) max_width=0;
    Bit#(64) in_data = zeroExtend(data);
    if(bsize=='b000) begin
        Bit#(8) out_data = in_data[((8) * (lwr_addr+1)) - 1 : (8 * lwr_addr)];
        max_width = duplicate(out_data);
    end
    if(bsize=='b001) begin
        Bit#(16) out_data = in_data[((16) * (lwr_addr+1)) - 1 : (16 * lwr_addr)];
        max_width = duplicate(out_data);
    end
    if(bsize=='b010) begin
        Bit#(32) out_data = in_data[((32) * (lwr_addr+1)) - 1 : (32 * lwr_addr)];
        max_width = duplicate(out_data);
    end
    if(valueOf(data_width)==64) begin
        if(bsize=='b011) begin
            Bit#(64) out_data = in_data; 
            max_width = out_data;
        end
    end
    return truncate(max_width);
//    case(bsize)
//        'b000: begin
//            case(lwr_addr)
//                'b000: begin
//                    max_width = duplicate(data[7:0]);
//                end
//                'b001: begin
//                    max_width = duplicate(data[15:8]);
//                end
//                'b010: begin
//                    max_width = duplicate(data[23:16]);
//                end
//                'b011: begin
//                    max_width = duplicate(data[31:24]);
//                end
//                'b100: begin
//                    max_width = duplicate(data[39:32]);
//                end
//                'b101: begin
//                    max_width = duplicate(data[47:40]);
//                end
//                'b110: begin
//                    max_width = duplicate(data[55:48]);
//                end
//                'b111: begin
//                    max_width = duplicate(data[63:56]);
//                end
//            endcase
//        end
//
//        'b001: begin
//    return duplicate(data[lwr_addr[b_strb*16] : lwr_addr[(bstrb-1)*16]])
//            case(lwr_addr) 
//                'b000: begin
//                    max_width = duplicate(data[15:0]);
//                end
//                'b010: begin
//                    max_width = duplicate(data[31:16]);
//                end
//                'b100: begin
//                    max_width = duplicate(data[47:32]);
//                end
//                'b110: begin
//                    max_width = duplicate(data[63:48]);
//                end
//            endcase
//        end
//
//        'b010: begin
//            case(lwr_addr)
//                'b000: begin
//                    max_width = duplicate(data[31:0]);
//                end
//                'b100: begin
//                    max_width = duplicate(data[63:32]);
//                end
//            endcase
//        end
//        'b011: begin
//            if(valueOf(data_width)==64)
//                max_width = data;
//        end
//    endcase
//    return truncate(max_width);
endfunction

//Reset rst0 <- mkAsyncResetFromCR (0, clk0);

Reg#(Bit#(9))        rg_delay_count <- mkReg(0,clocked_by clk0, reset_by rst0);
Reg#(Bit#(9))        rg_rd_actual_len <- mkReg(0,clocked_by clk0, reset_by rst0);
Reg#(bit)            rg_app_req <- mkDReg(0,clocked_by clk0, reset_by rst0);
Reg#(bit)            rg_app_req_wrap <- mkConfigReg(0,clocked_by clk0, reset_by rst0);
Reg#(Bit#(26))       rg_app_req_addr <- mkConfigReg(0,clocked_by clk0, reset_by rst0);
Reg#(Bit#(4))        rg_cfg_sdr_tras_d <- mkConfigReg(4'h4,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(4))        rg_cfg_sdr_trp_d <- mkConfigReg(4'h2,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(4))        rg_cfg_sdr_trcd_d <- mkConfigReg(4'h2,clocked_by clk0, reset_by rst0); 
Reg#(bit)            rg_cfg_sdr_en <- mkConfigReg(1'h0,clocked_by clk0, reset_by rst0);
Reg#(Bit#(2))        rg_cfg_req_depth <- mkConfigReg(2'h3,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(13))       rg_cfg_sdr_mode_reg <- mkConfigReg(13'h032,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(3))        rg_cfg_sdr_cas <- mkConfigReg(3'h3,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(4))        rg_cfg_sdr_trcar_d <- mkConfigReg(4'h7,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(4))        rg_cfg_sdr_twr_d <- mkConfigReg(4'h1,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(2))        rg_cfg_sdr_width <- mkConfigReg(2'b0,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(2))        rg_cfg_colbits <- mkConfigReg(2'b01,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(9))        rg_cfg_sdio_ctrl <- mkConfigReg(9'b000100011,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(8))        rg_cfg_sdr_clk_delay <- mkConfigReg(8'b00001000,clocked_by clk0, reset_by rst0);

Reg#(Bit#(rfrsh_timer_width ))  rg_cfg_sdr_rfsh <- mkConfigReg('h100,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(rfrsh_row_width)) rg_cfg_sdr_rfmax <- mkConfigReg('h6,clocked_by clk0, reset_by rst0); 
Reg#(Bit#(9))                   rg_app_req_len <- mkConfigReg(0,clocked_by clk0, reset_by rst0);
Reg#(Bit#(4))                   rg_lwraddr <- mkConfigReg(0,clocked_by clk0, reset_by rst0);
Reg#(Bit#(3))                   rg_arsize <- mkConfigReg(0,clocked_by clk0, reset_by rst0);
Reg#(bit)                       rg_app_req_wr_n <- mkConfigReg(0,clocked_by clk0, reset_by rst0);
Reg#(Bit#(TDiv#(data_width,8)))                   rg_app_wr_en_n <- mkDWire('hFF,clocked_by clk0, reset_by rst0);
Reg#(Bit#(data_width))                  rg_app_wr_data <- mkDWire(0,clocked_by clk0, reset_by rst0);
Wire#(Bool)                     wr_sdr_init_done <- mkDWire(False,clocked_by clk0, reset_by rst0);
Wire#(Bool)                     wr_app_req_ack <- mkDWire(False,clocked_by clk0, reset_by rst0);
Wire#(Bool)                     wr_app_wr_next_req <- mkDWire(False,clocked_by clk0, reset_by rst0);
Wire#(Bool)                     wr_app_rd_valid <- mkDWire(False,clocked_by clk0, reset_by rst0);
Wire#(Bool)                     wr_app_last_rd <- mkDWire(False,clocked_by clk0, reset_by rst0);
Wire#(Bool)                     wr_app_last_wr <- mkDWire(False,clocked_by clk0, reset_by rst0);
Wire#(Bit#(data_width))                 wr_app_rd_data <- mkWire(clocked_by clk0, reset_by rst0);


Reg#(Bit#(4))     rg_rid          <- mkReg(0, clocked_by clk0, reset_by rst0);
Reg#(bit)         rg_rd_not_active_flag <- mkSyncRegToCC(0,clk0, rst0);
Reg#(Bit#(4))     rg_ctrl_rid     <- mkReg(0);
Reg#(Bit#(4))     rg_wid          <- mkReg(0);

Reg#(Write_split_states) rg_wr_split_states <- mkReg(IDLE); 
Reg#(Bit#(3))            rg_awsize          <- mkReg(0);
Reg#(Bit#(9))            rg_ac_count        <- mkReg(0);
Reg#(Bit#(6))            rg_single_burst    <- mkReg(0);
Reg#(Bit#(data_width))           rg_wr_ac_data      <- mkReg(0);
Reg#(Bit#(TDiv#(data_width,8)))            rg_wr_ac_wstrb     <- mkReg(0);
Reg#(Bit#(4))            rg_wr_lwr_addr     <- mkReg(0); 
Reg#(Bit#(9))            rg_local_actual_wr_length <- mkReg(0);
Reg#(Bit#(9))            rg_actual_wr_length <- mkSyncRegFromCC(0,clk0);
Reg#(Bit#(addr_width))           rg_wr_address       <- mkSyncRegFromCC(0,clk0);

Reg#(Bit#(3))            rg_awsize_sclk       <- mkSyncRegFromCC(0,clk0);

Reg#(Bit#(9))            rg_burst_counter        <- mkReg(0);

Bool burst_eq = (rg_burst_counter == rg_local_actual_wr_length);

Reg#(Bit#(3))          rg_packets          <- mkReg(0);
Reg#(Bit#(3))          rg_packet_counter   <- mkReg(0);


Reg#(Write_state) rg_write_states <- mkReg(IDLE,clocked_by clk0, reset_by rst0);
Reg#(Read_state) rg_read_states <- mkReg(IDLE,clocked_by clk0, reset_by rst0);

FIFOF#(AXI4_Wr_Addr#(addr_width, user_width)) ff_wr_addr        <- mkSizedFIFOF(13);
FIFOF#(AXI4_Wr_Data#(data_width))        ff_wr_data        <- mkSizedFIFOF(13);
SyncFIFOIfc#(Bit#(data_width))           ff_ac_wr_data     <- mkSyncFIFOFromCC(13,clk0);
SyncFIFOIfc#(Bit#(TDiv#(data_width,8)))                    ff_ac_wr_wstrb    <- mkSyncFIFOFromCC(13,clk0);

SyncFIFOIfc#(Bool) ff_sync_write_response<-mkSyncFIFOToCC(1,clk0,rst0);

   //FIFOF#(AXI4_Rd_Addr#(addr_width,user_width)) ff_rd_addr <- mkSizedFIFOF(3);
FIFOF#(Bit#(data_width)) ff_rd_data <- mkSizedFIFOF(86,clocked_by clk0, reset_by rst0);
SyncFIFOIfc#(AXI4_Rd_Addr#(addr_width, user_width)) ff_rd_addr <- mkSyncFIFOFromCC(30,clk0);
SyncFIFOIfc#(AXI4_Rd_Data#(data_width, user_width)) ff_sync_read_response <-mkSyncFIFOToCC(13,clk0,rst0);

SyncFIFOIfc#(Tuple2#(Bit#(addr_width),Bit#(data_width))) ff_sync_ctrl_write<- mkSyncFIFOFromCC(1,clk0);
SyncFIFOIfc#(Bit#(addr_width)) ff_sync_ctrl_read<- mkSyncFIFOFromCC(1,clk0);
SyncFIFOIfc#(Bit#(data_width)) ff_sync_ctrl_read_response<- mkSyncFIFOToCC(1,clk0,rst0);

// Polling Registers
Reg#(Bit#(2)) rg_poll_cnt <- mkReg(0,clocked_by clk0, reset_by rst0);
Reg#(Bool)    rg_polling_status <- mkSyncRegToCC(False,clk0,rst0);
Reg#(Bool)    rg_polling_status_clk0 <- mkReg(False,clocked_by clk0, reset_by rst0);
Reg#(Bool)    rg_rd_trnc_flg <- mkReg(False);
Reg#(Bool)    rg_wr_trnc_flg <- mkReg(False);
Reg#(bit)     rg_odd_len     <- mkReg(0);   
// hardcoding the parameter value to resolve provisos
AXI4_Slave_Xactor_IFC #(addr_width, data_width, user_width)  s_xactor_sdram     <- mkAXI4_Slave_Xactor;
AXI4_Slave_Xactor_IFC #(addr_width, data_width, user_width)  s_xactor_cntrl_reg <- mkAXI4_Slave_Xactor;
Ifc_sdram#(io_width, rfrsh_timer_width, rfrsh_row_width) sdr_cntrl <- mksdrc_top(clocked_by clk0, reset_by rst0);

function Action fn_wr_cntrl_reg(Bit#(data_width) data, Bit#(addr_width) address);
   
   action
   Bit#(8) addr = truncate(address);
   case(truncate(addr)) 

       `ACTPRE_DELAY    : rg_cfg_sdr_tras_d <= data[3:0];

       `PREACT_DELAY    : rg_cfg_sdr_trp_d <= data[3:0];
       
       `ACT_RW_DELAY    : rg_cfg_sdr_trcd_d <= data[3:0];
      
       `EN_SDRAM        : rg_cfg_sdr_en <= data[0];
      
       `MAX_REQ         : rg_cfg_req_depth <= data[1:0];
      
       `MODE_REG        : rg_cfg_sdr_mode_reg <= data[12:0];
      
       `CAS_LATNCY      : rg_cfg_sdr_cas <= data[2:0];
      
       `AUTO_REFRESH    : rg_cfg_sdr_trcar_d <= data[3:0];
      
       `RECRY_DELAY     : rg_cfg_sdr_twr_d <= data[3:0];
     // 
       `RFRSH_TIMER     : rg_cfg_sdr_rfsh <= extend(data[11:0]);

       `RFRSH_ROW_CNT   : rg_cfg_sdr_rfmax <= extend(data[2:0]);

       `SDR_WIDTH       : rg_cfg_sdr_width <= data [1:0];

       `SDR_COLBITS     : rg_cfg_colbits <= data [1:0];
       
       `SDR_SDIO_CTRL   : rg_cfg_sdio_ctrl <= data [8:0];

       `SDR_CLK_DELAY   : rg_cfg_sdr_clk_delay <= data [7:0];

       default          : noAction;
  endcase
  endaction 
endfunction

function Bit#(data_width) fn_rd_cntrl_reg(Bit#(addr_width) address);
//    provisos(
//             Add#(a__, 1, data_width),
//             Add#(b__, 2, data_width),
//             Add#(c__, 8, data_width),
//             Add#(d__, 9, data_width),
//             Add#(e__, 12, data_width),
//             Add#(f__, 13, data_width),
//             Add#(g__, 8, addr_width)
//            );
  Bit#(8) addr = truncate(address);
  case(addr)

       `ACTPRE_DELAY    :  return extend(rg_cfg_sdr_tras_d);

       `PREACT_DELAY    :  return extend(rg_cfg_sdr_trp_d);
       
       `ACT_RW_DELAY    :  return extend(rg_cfg_sdr_trcd_d);
      
       `EN_SDRAM        :  return extend(rg_cfg_sdr_en);
      
       `MAX_REQ         :  return extend(rg_cfg_req_depth);
      
       `MODE_REG        :  return extend(rg_cfg_sdr_mode_reg);
      
       `CAS_LATNCY      :  return extend(rg_cfg_sdr_cas);
      
       `AUTO_REFRESH    :  return extend(rg_cfg_sdr_trcar_d);
      
       `RECRY_DELAY     : return extend(rg_cfg_sdr_twr_d);
      
       `RFRSH_TIMER     : return extend(rg_cfg_sdr_rfsh);

       `RFRSH_ROW_CNT   : return extend(rg_cfg_sdr_rfmax);

       `SDR_INIT_DONE   : return extend(pack(wr_sdr_init_done));

       `SDR_WIDTH       : return extend(rg_cfg_sdr_width);

       `SDR_COLBITS     : return extend(rg_cfg_colbits);

       `SDR_SDIO_CTRL   : return extend(rg_cfg_sdio_ctrl);

       `SDR_CLK_DELAY   : return extend(rg_cfg_sdr_clk_delay);

    endcase 
endfunction
          
//(*preempts="(rl_pop_read_request, rl_send_rd_data, rl_send_read_data, rl_flush_redundant_data), rl_write_transaction_write_start"*)

rule rl_for_writing_ctrl_reg(ff_sync_ctrl_write.notFull);
    let aw <- pop_o(s_xactor_cntrl_reg.o_wr_addr);
    let w  <- pop_o(s_xactor_cntrl_reg.o_wr_data);
    `ifdef verbose $display($time,"\tSDRAM: control_reg written addr %x data %x", aw.awaddr, w.wdata); `endif
	  ff_sync_ctrl_write.enq(tuple2(aw.awaddr,w.wdata));
    let w_resp = AXI4_Wr_Resp {bresp: AXI4_OKAY, buser: 0, bid: aw.awid}; 
    s_xactor_cntrl_reg.i_wr_resp.enq(w_resp);
endrule

rule rl_perform_write_to_ctrl(ff_sync_ctrl_write.notEmpty);
	let {awaddr,wdata}=ff_sync_ctrl_write.first;
  `ifdef verbose $display("\tSDRAM: "); `endif
	ff_sync_ctrl_write.deq;
`ifdef verbose $display($time,"\tSDRAM: Actually writing data: %h to addr: %h", wdata, awaddr); `endif
    fn_wr_cntrl_reg(wdata , truncate(awaddr));    
endrule

rule rl_for_read_cntrl_reg;
    let ar <- pop_o(s_xactor_cntrl_reg.o_rd_addr);
	  ff_sync_ctrl_read.enq(ar.araddr);
	  rg_ctrl_rid<=ar.arid;
 endrule

 rule rl_send_ctrl_read_response(ff_sync_ctrl_read.notEmpty);
	 ff_sync_ctrl_read_response.enq(fn_rd_cntrl_reg(truncate(ff_sync_ctrl_read.first)));
	 ff_sync_ctrl_read.deq;
endrule

rule sync_ctr_response(ff_sync_ctrl_read_response.notEmpty);
	ff_sync_ctrl_read_response.deq;
    let r = AXI4_Rd_Data {rresp: AXI4_OKAY, rdata:ff_sync_ctrl_read_response.first ,
    rlast: True, ruser: 0, rid: rg_ctrl_rid};
    s_xactor_cntrl_reg.i_rd_data.enq(r);
endrule

rule rl_direct_connection_insdram;
    sdr_cntrl.iapp_req(rg_app_req);
    sdr_cntrl.iapp_req_wrap(rg_app_req_wrap);
    sdr_cntrl.iapp_req_addr(rg_app_req_addr);
    sdr_cntrl.iapp_req_len(rg_app_req_len);
    sdr_cntrl.iapp_req_wr_n(rg_app_req_wr_n);
    sdr_cntrl.iapp_wr_data(extend(rg_app_wr_data));
    sdr_cntrl.iapp_wr_en_n(truncate(rg_app_wr_en_n));
endrule

rule rl_direct_connection_outsdram;
    wr_sdr_init_done <= sdr_cntrl.osdr_init_done ;
    wr_app_req_ack <= sdr_cntrl.oapp_req_ack ();
    wr_app_wr_next_req <= sdr_cntrl.oapp_wr_next_req ();
    wr_app_rd_valid <= sdr_cntrl.oapp_rd_valid ();
    wr_app_last_rd <= sdr_cntrl.oapp_last_rd ();
    wr_app_last_wr <= sdr_cntrl.oapp_last_wr ();
    wr_app_rd_data <= extend(sdr_cntrl.oapp_rd_data);
endrule

rule rl_direct_connection_config_reg;
    sdr_cntrl.icfg_sdr_tras_d(rg_cfg_sdr_tras_d);
    sdr_cntrl.icfg_sdr_trp_d(rg_cfg_sdr_trp_d);
    sdr_cntrl.icfg_sdr_trcd_d(rg_cfg_sdr_trcd_d);
    sdr_cntrl.icfg_sdr_en(rg_cfg_sdr_en);
    sdr_cntrl.icfg_req_depth(rg_cfg_req_depth);
    sdr_cntrl.icfg_sdr_mode_reg(rg_cfg_sdr_mode_reg);
    sdr_cntrl.icfg_sdr_cas(rg_cfg_sdr_cas);
    sdr_cntrl.icfg_sdr_trcar_d(rg_cfg_sdr_trcar_d);
    sdr_cntrl.icfg_sdr_twr_d(rg_cfg_sdr_twr_d);
    sdr_cntrl.icfg_sdr_rfsh(rg_cfg_sdr_rfsh);
    sdr_cntrl.icfg_sdr_rfmax(rg_cfg_sdr_rfmax);
    sdr_cntrl.icfg_sdr_width(rg_cfg_sdr_width);
    sdr_cntrl.icfg_colbits(rg_cfg_colbits);
endrule

rule rl_intial_polling(rg_polling_status_clk0 == False && wr_sdr_init_done == True);
    `ifdef verbose	$display($time,"\tSDRAM: POLLING MODE: %d",rg_poll_cnt); `endif 
    case (rg_poll_cnt)
        0: begin
            rg_app_req <= 1;
            rg_app_req_addr <= 0;
            rg_app_req_len <= 1;
            rg_app_req_wr_n <= 0;
            rg_app_wr_en_n <= 'hFF;
            rg_app_wr_data <= 0;
            rg_poll_cnt <= rg_poll_cnt + 1;
        end
        1: begin
            rg_app_req <= 1;
            rg_app_req_addr <= 0;
            rg_app_req_len <= 1;
            rg_app_req_wr_n <= 0;
            rg_app_wr_en_n <= 'hFF;
            rg_app_wr_data <= 0;
            rg_poll_cnt <= rg_poll_cnt + 1;
        end
        2: begin
            rg_app_req <= 0;
            rg_app_req_addr <= 0;
            rg_app_wr_en_n <= 'hFF;
            rg_app_wr_data <= 0;
            if(wr_app_wr_next_req == True)
                rg_poll_cnt <= rg_poll_cnt + 1;
        end
        3: begin
            rg_polling_status <= True;
				 rg_polling_status_clk0<=True;
        end
    endcase
endrule

/******************* WRITE TRANSACTION ****************/

rule rl_parallel_data_enq(rg_polling_status == True && rg_rd_trnc_flg == False);
    let aw <- pop_o(s_xactor_sdram.o_wr_addr);
    let w  <- pop_o(s_xactor_sdram.o_wr_data);
    ff_wr_addr.enq(aw);
    ff_wr_data.enq(w);      
    rg_wr_trnc_flg <= True;
    `ifdef verbose $display($time,"\tSDRAM: WRITE_FIRST Parallel enq %h addr: %h",w.wdata,aw.awaddr); `endif
endrule

rule rl_write_split_state(rg_wr_split_states == IDLE);
    let aw = ff_wr_addr.first();
    if(aw.awsize != 3) begin
        rg_actual_wr_length <= fn_wr_len(aw.awlen, aw.awsize, aw.awaddr[2:0]);

        rg_local_actual_wr_length <= extend(aw.awlen);
    end
    else begin
        rg_actual_wr_length <= extend(aw.awlen) + 1;
        rg_local_actual_wr_length <= extend(aw.awlen);
    end
    rg_wid <= aw.awid;
    rg_awsize <= aw.awsize;
    rg_awsize_sclk <= aw.awsize;
    rg_wr_address <= aw.awaddr;
    rg_wr_lwr_addr <= extend(aw.awaddr[2:0]);
    rg_wr_split_states <= START_SPLIT;
    rg_packets <=  (aw.awsize==0)?7:(aw.awsize==1)?3:(aw.awsize == 2)? 1 : (aw.awsize == 3) ? 0 : 0; //(64 >> (aw.awsize+4));  //1 << ~aw.awsize[1:0];
    rg_packet_counter <= (aw.awaddr[2:0]) >> aw.awsize;
    `ifdef verbose $display($time,"Initial Values -- Starting IDLE to START_SPLIT"); `endif
    //$display("Initial Values: rg_packets: %h rg_packet_counter: %h ",(64>>(aw.awsize+4)),aw.awaddr[2:0]>>aw.awsize);
    rg_burst_counter <= 0;
endrule


rule rl_write_data_splitting0(rg_wr_split_states == START_SPLIT && rg_awsize != 3);
   rg_wr_ac_data <= rg_wr_ac_data | fn_wr_split_data(ff_wr_data.first.wdata,ff_wr_data.first.wstrb);
   rg_wr_ac_wstrb <= rg_wr_ac_wstrb | ff_wr_data.first.wstrb;
   rg_burst_counter <= rg_burst_counter + 1;
   //`ifdef verbose $display($time,"burst_eq: %h rg_packets: %h rg_packet_counter: %d rg_burst_counter: %d rg_local_actual_wr_length %d rg_wr_ac_data %h rg_wr_ac_wstrb %b",burst_eq,rg_packets,rg_packet_counter
   //,rg_burst_counter,rg_local_actual_wr_length, fn_wr_split_data(ff_wr_data.first.wdata,ff_wr_data.first.wstrb),
   //ff_wr_data.first.wstrb); `endif
       if(burst_eq || rg_packets == rg_packet_counter) begin
           rg_wr_split_states <= SEND_VALUE;
       end
       else begin
          rg_packet_counter <= rg_packet_counter + 1;
       end
        ff_wr_data.deq();
        ff_wr_addr.deq();
endrule

rule rl_write_data_splitting1(rg_wr_split_states == SEND_VALUE && rg_awsize != 3);
    `ifdef verbose $display($stime,"Splitting1 Enqueued data rg_wr_ac_data %h rg_wr_ac_wstrb %b", rg_wr_ac_data, rg_wr_ac_wstrb); `endif
    ff_ac_wr_data.enq(rg_wr_ac_data);
    ff_ac_wr_wstrb.enq(rg_wr_ac_wstrb);
    rg_wr_ac_data <= 0;            
    rg_wr_ac_wstrb <= 0;
    rg_wr_lwr_addr <= 0;
    rg_packet_counter <= 0;
    `ifdef verbose $display($time,"Sending Value to the SDRAM"); `endif
    rg_wr_split_states <= START_SPLIT;
endrule

rule rl_write_data_spliting3(rg_wr_split_states == START_SPLIT && rg_awsize == 3);
    ff_ac_wr_data.enq(ff_wr_data.first.wdata);
    ff_ac_wr_wstrb.enq(ff_wr_data.first.wstrb);
    ff_wr_data.deq();
    ff_wr_addr.deq();
endrule

rule rl_start_write_transaction(rg_write_states == IDLE && wr_sdr_init_done == True);        
    if(ff_ac_wr_data.notEmpty()) begin
        if(rg_awsize_sclk == 0)
            rg_write_states <= WAIT_DELAY;
        else
        rg_write_states <= WRITE_START;
        `ifdef verbose $display($time,"\tSDRAM: Going to write start state"); `endif
    end
endrule

rule rl_wait_delay(rg_write_states == WAIT_DELAY);
    if(rg_delay_count == 14) begin
        rg_write_states <= WRITE_START;
        rg_delay_count <= 0;
    end
    else
    rg_delay_count <= rg_delay_count + 1;
endrule

rule rl_write_transaction_write_start(rg_write_states == WRITE_START && wr_sdr_init_done == True && rg_read_states == IDLE);
    `ifdef verbose $display($time,"\tSDRAM: WRITE_START state Controller Length %d",rg_actual_wr_length); `endif
    rg_app_req <= 1;
    rg_app_req_addr <= fn_wr_address(rg_wr_address);
    rg_app_req_len <= extend(rg_actual_wr_length);
    rg_app_req_wr_n <= 0;
    rg_app_wr_data <= 0;
    rg_app_wr_en_n <= 'hFF;
    rg_write_states <= WRITE_FIRST;
    rg_delay_count <= extend(rg_actual_wr_length) - 1;
endrule

rule rl_write_transaction_write_first(rg_write_states == WRITE_FIRST && wr_app_wr_next_req == False);
    `ifdef verbose $display($time,"\tSDRAM: WRITE_FIRST state next is false data %x",ff_ac_wr_data.first); `endif
    rg_app_req <= 0;
    rg_app_wr_en_n <= ~(ff_ac_wr_wstrb.first());
    rg_app_wr_data <= ff_ac_wr_data.first();               
endrule

rule rl_write_transaction_write_data(rg_write_states == WRITE_FIRST && wr_app_wr_next_req == True);
`ifdef verbose $display($time,"\tSDRAM: WRITE_DATA state next is true sending data %x %b",ff_ac_wr_data.first,
wr_app_wr_next_req); `endif
    rg_app_req <= 0;
    rg_app_wr_data <= ff_ac_wr_data.first();
    rg_app_wr_en_n <= ~(ff_ac_wr_wstrb.first());
    ff_ac_wr_data.deq;
    ff_ac_wr_wstrb.deq;
    rg_delay_count <= rg_delay_count - 1;
    if(rg_delay_count == 0) begin
       rg_write_states <= IDLE;
			ff_sync_write_response.enq(True);
    end
endrule

rule synchronize_write_response(ff_sync_write_response.notEmpty);
	ff_sync_write_response.deq;
	let w_resp = AXI4_Wr_Resp {bresp: AXI4_OKAY, buser: 0, bid: rg_wid}; //TODO user value is null
  s_xactor_sdram.i_wr_resp.enq(w_resp);
   rg_wr_split_states <= IDLE;
   rg_ac_count <= 0;
   rg_wr_trnc_flg <= False;
  `ifdef verbose   $display($time,"\tSDRAM: WRITE complete state true"); `endif
endrule
    
   /****************** Read Transaction ********************/
	//	`ifdef verbose
	//	  rule display_read_states;
	//		$display($time,"\tSDRAM: Read State: ",fshow(rg_read_states));
	//	  endrule
	//	`endif
rule rl_paralel_read_req_enq(rg_polling_status == True && rg_wr_trnc_flg == False);
    let ar <- pop_o(s_xactor_sdram.o_rd_addr);
     ff_rd_addr.enq(ar);
     rg_rd_trnc_flg <= True;
    `ifdef verbose	$display($time,"\tSDRAM: Got Read request from AXI for AddresS: %h",ar.araddr); `endif
endrule

rule rl_read_idle_state(rg_read_states == IDLE);
    if(ff_rd_addr.notEmpty() == True) begin
        rg_read_states <= START_READ;
        rg_rd_not_active_flag <= 0;
        `ifdef verbose  $display($time,"\tSDRAM: READ IDLE state"); `endif
    end

endrule

rule rl_pop_read_request(wr_sdr_init_done == True && rg_read_states == START_READ && (rg_write_states == IDLE || 
rg_write_states == WRITE_START));
let ar = ff_rd_addr.first;
`ifdef verbose $display($time,"\tSDRAM: STAR_READ state ar.arlen %d ar.arsize %d ar.araddr %h ar.arburst %b", ar.
arlen,ar.arsize, ar.araddr, ar.arburst); `endif

rg_app_req <= 1;
if(ar.arburst == 2)
    rg_app_req_wrap <= 1;
else
    rg_app_req_wrap <= 0;
rg_lwraddr <= extend(ar.araddr[2:0]);
rg_arsize <= ar.arsize;
rg_app_req_addr <= fn_wr_address(ar.araddr);
rg_app_req_len <= fn_wr_len(ar.arlen, ar.arsize, ar.araddr[2:0]);
rg_rd_actual_len <= extend(ar.arlen);
rg_app_req_wr_n <= 1;
rg_delay_count <= 0;
rg_read_states <= READ_DATA;
rg_rid <= ar.arid;
`ifdef verbose $display($time,"\t SSSSS SDRAM START_READ length "); `endif
endrule

rule rl_send_rd_data(wr_app_rd_valid == True);
    `ifdef verbose $display($time,"\tSDRAM: READ DATA1 state %x",wr_app_rd_data); `endif
    ff_rd_data.enq(wr_app_rd_data);
endrule

rule rl_send_read_data(rg_read_states==READ_DATA);
`ifdef verbose $display($time,"\tSDRAM: Response from BFM. RequestLenght: %d CurrentCount: %d",rg_rd_actual_len,
rg_delay_count); `endif
rg_app_req_wrap <= 0;
if(rg_lwraddr < 8) begin
      let r = AXI4_Rd_Data {rresp: AXI4_OKAY, rdata: fn_rd_data(rg_arsize, extend(rg_lwraddr), ff_rd_data.first), rlast: 
      (rg_rd_actual_len == rg_delay_count), ruser: 0, rid: rg_rid};
  		  ff_sync_read_response.enq(r);
  		  ff_rd_addr.deq;
  		  if(rg_arsize!=3)
          rg_lwraddr <= rg_lwraddr + (1 << rg_arsize);
  			else
  				ff_rd_data.deq;
  		//`ifdef verbose $display($time,"\tSDRAM: SENDING READ DATA : %h, rg_lwraddr %b rg_arsize %d", 
         // fn_rd_data(rg_arsize, rg_lwraddr, ff_rd_data.first),rg_lwraddr, rg_arsize); `endif
  			`ifdef verbose $display($time,"\tSDRAM: Removing Request for Addr : %h",ff_rd_addr.first.araddr); `endif
   if(rg_delay_count == rg_rd_actual_len) begin
    `ifdef verbose  $display($time,"\tSDRAM: SENT ALL READ DATA state rg_delay_count %d rg_rd_actual_len %d", 
    rg_delay_count, rg_rd_actual_len); `endif
      rg_read_states <= READ_FLUSH;
      rg_rd_not_active_flag <= 1;
      rg_delay_count <= 0;
   end
   else 
     rg_delay_count <= rg_delay_count + 1;
 end
 else if(rg_lwraddr > 7) begin
        `ifdef verbose $display($time,"\tSDRAM: Dequeuing ff READ"); `endif
        rg_lwraddr <= 0;
        ff_rd_data.deq;
 end
endrule

rule rl_flush_redundant_data(rg_read_states == READ_FLUSH);
    ff_rd_data.clear();
    rg_read_states <= IDLE;
endrule

rule send_synchronized_read_response(ff_sync_read_response.notEmpty);
  let r=ff_sync_read_response.first;
  `ifdef verbose	$display($time,"\tSDRAM: Sending Read response: %h rlast: %b",r.rdata,r.rlast); `endif
  ff_sync_read_response.deq;
  if(rg_rd_not_active_flag == 1)
  rg_rd_trnc_flg <= False;
  s_xactor_sdram.i_rd_data.enq(r);
endrule


interface Ifc_sdram_out io;

    method Action ipad_sdr_din(Bit#(io_width) pad_sdr_din);
        sdr_cntrl.ipad_sdr_din(pad_sdr_din);
    endmethod
    method Bit#(9) sdram_sdio_ctrl();
        return rg_cfg_sdio_ctrl;
    endmethod
    method Bit#(io_width) osdr_dout();
        return sdr_cntrl.osdr_dout();
    endmethod
    method Bit#(8) osdr_den_n();
        return sdr_cntrl.osdr_den_n();
    endmethod
    method Bool osdr_cke();
        return sdr_cntrl.osdr_cke();
    endmethod

    method Bool osdr_cs_n();
        return sdr_cntrl.osdr_cs_n();
    endmethod

    method Bool osdr_ras_n ();
        return sdr_cntrl.osdr_ras_n;
    endmethod

    method Bool osdr_cas_n ();
        return sdr_cntrl.osdr_cas_n;
    endmethod

    method Bool osdr_we_n ();
        return sdr_cntrl.osdr_we_n;
    endmethod

    method Bit#(8) osdr_dqm ();
        return sdr_cntrl.osdr_dqm;
    endmethod

    method Bit#(2) osdr_ba ();
        return sdr_cntrl.osdr_ba;
    endmethod

    method Bit#(13) osdr_addr ();
        return sdr_cntrl.osdr_addr;
    endmethod
    
    interface sdram_clk = clk0;
endinterface


interface slave_mem= s_xactor_sdram.axi_side;
interface slave_cfg = s_xactor_cntrl_reg.axi_side;

endmodule
endpackage
