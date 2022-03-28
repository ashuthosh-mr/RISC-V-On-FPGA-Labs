/* 
Copyright (c) 2013, IIT Madras All rights reserved.

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

Author: Vishvesh Sundaraman
Email id: vishu.vivek@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/

package InterfaceNandFlashController ;


`include "nand.defines"
import Vector::*;
import DReg::*;
import ClockDiv::*;
import ConfigReg ::*;
import AXI4_Types   :: *;
import AXI4_Fabric  :: *;
import Semi_FIFOF ::*;
import NandFlashController ::*;
import Connectable ::*;
import FIFOF::*;
import InterfaceECCencoder ::*;
import bsvmkdecoder_wrapper ::*;
import Clocks::*;

`include "global_parameters_Flash.bsv"

// Controller register address parameter

///////////////////////////////////////////////////////////////////////////////////////////////////
//		NVM - NAND FLASH CONTROLLER INTERFACE
///////////////////////////////////////////////////////////////////////////////////////////////////

interface Ifc_NandFlashController;
    interface Onfi_Interface_top onfi_interface;
    interface AXI4_Slave_IFC#(`PADDR, `Reg_width,`USERSPACE) axi4_slave_onfi;
    interface AXI4_Slave_IFC#(`PADDR, `Reg_width,`USERSPACE) axi4_slave_onfi_cfg_reg;
    interface Clock clk_new;
endinterface

function ActionValue #(t) pop_i (FIFOF #(t) f);
    actionvalue 
        f.deq;
        return f.first;
    endactionvalue 
endfunction

(*synthesize*)
//(*clock_family="default_clock,clk_inv, clock_divider"*)
//(*clock_ancestors="clock_divider AOF core_clk "*)
//(*clock_ancestors="clk_inv AOF core_clk "*)
module mkInterfaceNandflashController `ifdef Out_clock #(Clock clk0,Clock clk1,Clock clk2,Clock clk3,Clock clk4) `endif (Ifc_NandFlashController);

Clock core_clk <- exposeCurrentClock();
Reset core_rst <- mkAsyncResetFromCR(0, core_clk);

`ifndef Out_clock
Ifc_ClockDiv#(`DIVWIDTH) clock_divider <- mkClockDiv(clocked_by core_clk, reset_by core_rst);
`endif
`ifdef Out_clock
Clock clk_inv <- invertCurrentClock(clocked_by clk0);
Reset rst_inv <- mkAsyncResetFromCR(0,clk_inv);
Reset rst0    <- mkAsyncResetFromCR(0,clk0);
`else
Clock clk_inv <- invertCurrentClock(clocked_by clock_divider.slowclock);
Reset rst_inv <- mkAsyncResetFromCR(0,clk_inv);
Reset rst0 <- mkAsyncResetFromCR(0,clock_divider.slowclock);
`endif

`ifdef Out_clock
MuxClkIfc clk_mux <- mkUngatedClockMux(clk0, clk_inv);
Reset     rst_mux <- mkAsyncResetFromCR(0,clk_mux.clock_out);
`else
MuxClkIfc clk_mux <- mkUngatedClockMux(clock_divider.slowclock, clk_inv);
Reset     rst_mux <- mkAsyncResetFromCR(0,clk_mux.clock_out);
`endif

`ifdef Out_clock
Ifc_NFC_Interface onfi_controller <- mkNandFlashController(clk_inv, rst_inv, clk_mux.clock_out,rst_mux, clocked_by  clk0, reset_by rst0);
`else
Ifc_NFC_Interface onfi_controller <- mkNandFlashController(clk_inv, rst_inv, clk_mux.clock_out,rst_mux, clocked_by  clock_divider.slowclock, reset_by rst0);
`endif

AXI4_Slave_Xactor_IFC#(`PADDR,`Reg_width,`USERSPACE) s_xactor_onfi         <- mkAXI4_Slave_Xactor;
AXI4_Slave_Xactor_IFC#(`PADDR,`Reg_width,`USERSPACE) s_xactor_onfi_cfg_reg <- mkAXI4_Slave_Xactor;
FIFOF#(AXI4_Rd_Addr#(`PADDR,`USERSPACE)) ff_rd_addr <- mkSizedFIFOF(1);

`ifdef Out_clock
SyncFIFOIfc#(Bit#(`Reg_width)) ff_sync_write_io    <- mkSyncFIFOFromCC(5, clk0);
SyncFIFOIfc#(Bit#(1)) ff_write_io_start <- mkSyncFIFOFromCC(1, clk0);
SyncFIFOIfc#(Bit#(8)) ff_sync_read_io   <- mkSyncFIFOFromCC(1, clk0);
SyncFIFOIfc#(AXI4_Rd_Data#(`Reg_width, `USERSPACE)) ff_sync_read_erase_cc_io   <- mkSyncFIFOToCC(9,
clk0, rst0);
SyncFIFOIfc#(Tuple2#(Bit#(`PADDR),Bit#(`Reg_width))) ff_sync_ctrl_write <- mkSyncFIFOFromCC(1,clk0);
`else
SyncFIFOIfc#(Bit#(`Reg_width)) ff_sync_write_io <- mkSyncFIFOFromCC(5, clock_divider.slowclock);
SyncFIFOIfc#(Bit#(1))          ff_write_io_start <- mkSyncFIFOFromCC(1, clock_divider.slowclock);
SyncFIFOIfc#(Bit#(8)) ff_sync_read_io   <- mkSyncFIFOFromCC(1, clock_divider.slowclock);
SyncFIFOIfc#(AXI4_Rd_Data#(`Reg_width, `USERSPACE)) ff_sync_read_erase_cc_io   <- mkSyncFIFOToCC(9,
clock_divider.slowclock, rst0);
SyncFIFOIfc#(Tuple2#(Bit#(`PADDR),Bit#(`Reg_width))) ff_sync_ctrl_write <- mkSyncFIFOFromCC(1,clock_divider.slowclock);
`endif

  Reg#(Bit#(`DIVWIDTH)) rg_clock_divisor <- mkReg(2, clocked_by core_clk,reset_by core_rst);
  Reg#(bit)             rg_clk_select    <- mkReg(1);

// local registers
    Reg#(Bit#(10)) rg_rsp_cnt 		  <- mkConfigReg(0);
    `ifdef Out_clock
    Reg#(Bit#(9))  rg_rd_data_cnt  <- mkReg(0, clocked_by clk0, reset_by rst0);
    Reg#(Bit#(32)) rg_read_addr    <- mkReg(0, clocked_by clk0, reset_by rst0);
    Reg#(Bit#(32)) rg_write_addr   <- mkReg(0, clocked_by clk0, reset_by rst0);
    Reg#(Bit#(11)) rg_cfg_page_len   <- mkConfigReg(0,clocked_by clk0, reset_by rst0);
    Reg#(Bit#(1))  rg_control_rd_er  <- mkConfigReg(0,clocked_by clk0, reset_by rst0);
    Reg#(bit)      rg_read_start     <- mkReg(0,clocked_by clk0, reset_by rst0);
    Reg#(Bit#(8))  rg_timing_mode    <- mkReg(0,clocked_by clk0, reset_by rst0);
    `else
    Reg#(Bit#(9))  rg_rd_data_cnt  <- mkReg(0, clocked_by clock_divider.slowclock, reset_by rst0);
    Reg#(Bit#(32)) rg_read_addr    <- mkReg(0, clocked_by clock_divider.slowclock, reset_by rst0);
    Reg#(Bit#(32)) rg_write_addr   <- mkReg(0, clocked_by clock_divider.slowclock, reset_by rst0);
    Reg#(Bit#(11)) rg_cfg_page_len   <- mkConfigReg(0,clocked_by clock_divider.slowclock , reset_by rst0);
    Reg#(Bit#(1))  rg_control_rd_er  <- mkConfigReg(0,clocked_by clock_divider.slowclock , reset_by rst0);
    Reg#(bit)      rg_read_start     <- mkReg(0,clocked_by clock_divider.slowclock, reset_by rst0);
    Reg#(Bit#(8))  rg_timing_mode    <- mkReg(0,clocked_by clock_divider.slowclock, reset_by rst0);
    `endif
    
//Control Registers for NandflashController

    Reg#(Bit#(4))  rg_axi_id         <- mkReg(0);
`ifdef Out_clock
	SyncFIFOIfc#(Bit#(`Reg_width)) ff_read_cntrl_read_response <- mkSyncFIFOToCC(1, clk0, rst0);
	SyncFIFOIfc#(Bit#(`PADDR))     ff_sync_read_response       <- mkSyncFIFOFromCC(1,clk0);
`else
	SyncFIFOIfc#(Bit#(`Reg_width)) ff_read_cntrl_read_response <- mkSyncFIFOToCC(1, clock_divider.slowclock, rst0);
	SyncFIFOIfc#(Bit#(`PADDR)) ff_sync_read_response           <- mkSyncFIFOFromCC(1,clock_divider.slowclock);
`endif

    let nfc_ifc = onfi_controller.nvm_nfc_interface;

//    (* conflict_free = "rl_read_request, rl_read_response" *) 

function Action fn_wr_control_reg(Bit#(32)addr, Bit#(64)data);
    action
    case(addr[7:0])
        `LENGTH_RD_ER : begin
            rg_cfg_page_len     <= data[10:0];
            rg_control_rd_er    <= data[11];
                $display($stime,"LENGTH_RD_ER register cfg_len %x",data[10:0]);
        end
        `ADDR_WE : begin
                rg_write_addr   <= data[31:0];
                $display($stime,"ADDR_WE write register");
        end
        `ADDR_RE : begin
                rg_read_addr    <= data[31:0];
                $display($stime,"ADDR_WE read register");
                rg_read_start   <= 1;
        end   
        `TIMING_MODE : begin
//            onfi_controller.cfg_reg_interface._timing_mode(data[7:0]);
              rg_timing_mode <= data[7:0];
        end
    endcase
    endaction
endfunction

function Bit#(`Reg_width) fn_rd_control_reg(Bit#(32)addr);
    case(addr[7:0])
        `LENGTH_RD_ER :  return extend({ rg_control_rd_er,rg_cfg_page_len});

        `ADDR_WE :
                return extend(rg_write_addr);

        `ADDR_RE :
                return extend(rg_read_addr);

        `TIMING_MODE :
                return extend(rg_timing_mode);
    endcase
endfunction


rule select_clk_sel_inv;
  clk_mux.select(rg_clk_select == 0);
endrule

//Clock mux selection
`ifndef Out_clock
rule rl_clock_divider;
      clock_divider.divisor(rg_clock_divisor); //TODO add config register
endrule
`endif
/////////////////////////// WRITE AND READ rules for control register /////////////////////////////

rule rl_write_control_reg_cc;
    let aw <- pop_o(s_xactor_onfi_cfg_reg.o_wr_addr);
    let w  <- pop_o(s_xactor_onfi_cfg_reg.o_wr_data);
    ff_sync_ctrl_write.enq(tuple2(aw.awaddr,w.wdata));
    let w_resp = AXI4_Wr_Resp {bresp: AXI4_OKAY, buser: 0, bid: aw.awid};
    s_xactor_onfi_cfg_reg.i_wr_resp.enq(w_resp);
    $display($stime,"InterfaceNFC: sending write control registeraddr %x response to the host length %b",aw.awaddr, w.wdata);
endrule

rule rl_write_perform_ctrl_reg;
    let {awaddr,wdata}=ff_sync_ctrl_write.first;
    fn_wr_control_reg(awaddr, wdata);
    ff_sync_ctrl_write.deq;
    $display($stime,"InterfaceNFC: writing control register on the onfi domain");
endrule

    
rule rl_read_control_reg_cc;
    let ar <- pop_o(s_xactor_onfi_cfg_reg.o_rd_addr);
    ff_sync_read_response.enq(ar.araddr);
    rg_axi_id <= ar.arid;
endrule

rule rl_read_ctrl_reg ;
    let addr = ff_sync_read_response.first;
    let data = fn_rd_control_reg(addr);
    ff_read_cntrl_read_response.enq(data);
    ff_sync_read_response.deq;
endrule

rule rl_sending_read_ctrl_reg_response;
    let resp = AXI4_Rd_Data{rresp:AXI4_OKAY,rdata:ff_read_cntrl_read_response.first,rlast:True,ruser:0, rid: rg_axi_id};
    ff_read_cntrl_read_response.deq();
    s_xactor_onfi_cfg_reg.i_rd_data.enq(resp);
    $display($stime,"InterfaceNFC: sending READ control register response to the host");
endrule

/////////////////////////Write and Read IO//////////////////////////////////////////////////////////

rule rl_write_request_addr;
    let aw <- pop_o(s_xactor_onfi.o_wr_addr);
    $display($stime,"InterfaceNFC: Receiving write request from host in the interface wr_addr %x id %x"
    ,aw.awaddr,aw.awid);
    ff_write_io_start.enq(1);
    rg_rsp_cnt <= extend(aw.awlen) + 1;
    rg_axi_id  <= aw.awid;
endrule 
    
rule rl_write_request_data(rg_rsp_cnt != 0);
    let w  <- pop_o(s_xactor_onfi.o_wr_data);
    ff_sync_write_io.enq(w.wdata);
        if(rg_rsp_cnt == 1) begin
        let w_resp = AXI4_Wr_Resp {bresp: AXI4_OKAY, buser: 0, bid: rg_axi_id}; //TODO add id
        s_xactor_onfi.i_wr_resp.enq(w_resp);
        rg_rsp_cnt <= 0;
        $display($stime,"InterfaceNFC: sending write response to the host id %x \n",rg_axi_id);
    end
    else
        rg_rsp_cnt <= rg_rsp_cnt - 1;
endrule

rule rl_write_addr_to_controller;
    nfc_ifc._write_addr(zeroExtend(rg_write_addr),rg_cfg_page_len);
    ff_write_io_start.deq();
endrule

rule rl_write_data_to_controller(nfc_ifc.busy_ == 1'b0);
    nfc_ifc._write_data(ff_sync_write_io.first);
    $display($stime,"InterfaceNFC: sending write data to nfc \n");
    ff_sync_write_io.deq();
endrule


//This rule is to initiate read transaction on onfi triggered by config reg
rule rl_config_addr_read (nfc_ifc.busy_ == 1'b0 && rg_read_start == 1);
//    let ar <- pop_i(ff_rd_addr);
    if(rg_control_rd_er == 0) begin // for read request
        nfc_ifc._request_data(zeroExtend(rg_read_addr), rg_cfg_page_len);
        $display($stime(),"InterfaceNFC: : received read request from host aruser_fifo rg_cfg_page_len %x",rg_cfg_page_len);
    end
    else if(rg_control_rd_er == 1) begin//for erase request
        nfc_ifc._request_erase(zeroExtend(rg_read_addr));
        $display($stime(),"InterfaceNFC: : received erase request from host");
    end
    rg_read_start <= 0;
endrule

// This rule is polling from core or dma to access the controllers local buffer
// The rg_data_to_nvm register in NandFlashController doesnt have any control it will fire when the 
// fifo is not empty TODO if this creates any problem need to add control for that register in 
// NandflashController
rule rl_read_request;
    let ar <- pop_o(s_xactor_onfi.o_rd_addr);
    $display($stime(),"InterfaceNFC: received read request for data from fifo arlength %x",ar.arlen);
    ff_sync_read_io.enq(ar.arlen);
//    rg_rd_data_cnt <= extend(ar.arlen) + 1;
    rg_axi_id <= ar.arid;
endrule

rule rl_read_length;
    let len = ff_sync_read_io.first;
    rg_rd_data_cnt <= extend(len) + 1;
    ff_sync_read_io.deq();
endrule

rule rl_sending_read_erase_response(rg_rd_data_cnt != 0);

    if(nfc_ifc.erase_success_() == 1 && rg_control_rd_er == 1) begin
	    let r = AXI4_Rd_Data {rresp: AXI4_OKAY, rdata: 0, rlast: True, ruser: 0, rid: 0};
        ff_sync_read_erase_cc_io.enq(r);
	end
    else begin       
            let lv_data <- nfc_ifc._get_data_();
            let r = AXI4_Rd_Data {rresp: AXI4_OKAY, rdata: lv_data,
            rlast:(rg_rd_data_cnt ==  1),ruser: 0, rid: 0};
            $display($stime(),"InterfaceNFC: sending read response to the host");
            rg_rd_data_cnt <= rg_rd_data_cnt - 1;
//            nfc_ifc.read_resp_ready_(s_xactor_onfi.i_rd_data.notFull());
            ff_sync_read_erase_cc_io.enq(r);
    end
endrule

rule rl_sending_cc_read_erase_response;
    let r = ff_sync_read_erase_cc_io.first;
    let d = AXI4_Rd_Data {rresp: r.rresp, rdata: r.rdata,
    rlast:r.rlast,ruser: 0, rid: rg_axi_id};
    s_xactor_onfi.i_rd_data.enq(d);
    ff_sync_read_erase_cc_io.deq();
endrule

//ECC Encoder
/*rule rl_data_from_ecc ;
	onfi_controller.nfc_ecc_enc_interface._data_from_ecc_encoder(ecc_enc_nfc_ifc.data_to_nfc) ;
endrule*/

let onfi_nfc = onfi_controller.nfc_onfi_interface;

//rule rl_data_in_connection;
//      onfi_nfc.data0_data_in(wr_data0);
//      onfi_nfc.data1_data_in(wr_data1);
//endrule

Reg#(bit)      rg_ready_busy_n_0   <- mkReg(0); 
Reg#(bit)      rg_ready_busy_n_1   <- mkReg(0); 
Reg#(bit)      rg_dqs_in           <- mkReg(0);
Reg#(bit)      rg_dqs_c_in         <- mkReg(0);
Reg#(bit)      rg_dqs2_in          <- mkReg(0);
Reg#(bit)      rg_dqs2_c_in        <- mkReg(0);
Reg#(Bit#(8))  rg_data0_in         <- mkReg(0);
Reg#(Bit#(8))  rg_data1_in         <- mkReg(0);
`ifdef Out_clock
Reg#(Bool)     rg_dqs_out          <- mkReg(False,clocked_by clk0, reset_by rst0);
Reg#(Bool)     rg_dqs_c_out        <- mkReg(True,clocked_by  clk0, reset_by rst0);
Reg#(Bool)     rg_dqs2_out         <- mkReg(False,clocked_by clk0, reset_by rst0);
Reg#(Bool)     rg_dqs2_c_out       <- mkReg(True,clocked_by  clk0, reset_by rst0);
`else
Reg#(Bool)     rg_dqs_out          <- mkReg(False,clocked_by clock_divider.slowclock, reset_by rst0);
Reg#(Bool)     rg_dqs_c_out        <- mkReg(True,clocked_by clock_divider.slowclock, reset_by rst0);
Reg#(Bool)     rg_dqs2_out         <- mkReg(False,clocked_by clock_divider.slowclock, reset_by rst0);
Reg#(Bool)     rg_dqs2_c_out       <- mkReg(True,clocked_by clock_divider.slowclock, reset_by rst0);
`endif
Wire#(bit)     wr_we_n             <- mkWire();

ReadOnly#(bit) wr_null_ce0_n <- mkNullCrossingWire(core_clk, onfi_nfc.onfi_ce0_n_);
ReadOnly#(bit) wr_null_ce1_n <- mkNullCrossingWire(core_clk, onfi_nfc.onfi_ce1_n_);
ReadOnly#(bit) wr_null_sync_we_n   <- mkNullCrossingWire(core_clk,onfi_nfc.onfi_sync_we_n_);
ReadOnly#(bit) wr_null_async_we_n  <- mkNullCrossingWire(core_clk,onfi_nfc.onfi_async_we_n_);
ReadOnly#(bit) wr_null_timing_set  <- mkNullCrossingWire(core_clk,onfi_nfc.timing_set_);
ReadOnly#(bit) wr_null_re_n  <- mkNullCrossingWire(core_clk, onfi_nfc.onfi_re_n_);
ReadOnly#(bit) wr_null_wp_n  <- mkNullCrossingWire(core_clk, onfi_nfc.onfi_wp_n_);
ReadOnly#(bit) wr_null_cle   <- mkNullCrossingWire(core_clk, onfi_nfc.onfi_cle_);
ReadOnly#(bit) wr_null_ale   <- mkNullCrossingWire(core_clk, onfi_nfc.onfi_ale_);
`ifdef Out_clock
ReadOnly#(bit) wr_null_ready_busy_0 <- mkNullCrossingWire(clk0, rg_ready_busy_n_0);
ReadOnly#(bit) wr_null_ready_busy_1 <- mkNullCrossingWire(clk0, rg_ready_busy_n_1);
`else
ReadOnly#(bit) wr_null_ready_busy_0 <- mkNullCrossingWire(clock_divider.slowclock, rg_ready_busy_n_0);
ReadOnly#(bit) wr_null_ready_busy_1 <- mkNullCrossingWire(clock_divider.slowclock, rg_ready_busy_n_1);
`endif
ReadOnly#(Bool) wr_null_dqs_out      <- mkNullCrossingWire(core_clk, onfi_nfc.dqs_out);
ReadOnly#(bit) wr_null_dqs_enable    <- mkNullCrossingWire(core_clk, onfi_nfc.dqs_enable);
`ifdef Out_clock
ReadOnly#(bit) wr_null_dqs_in        <- mkNullCrossingWire(clk0, rg_dqs_in);
`else
ReadOnly#(bit) wr_null_dqs_in        <- mkNullCrossingWire(clock_divider.slowclock,
rg_dqs_in);
`endif
ReadOnly#(Bool) wr_null_dqs_c_out    <- mkNullCrossingWire(core_clk, onfi_nfc.dqs_c_out);
ReadOnly#(bit) wr_null_dqs_c_enable    <- mkNullCrossingWire(core_clk, onfi_nfc.dqs_c_enable);
`ifdef Out_clock
ReadOnly#(bit) wr_null_dqs_c_in        <- mkNullCrossingWire(clk0, rg_dqs_c_in);
`else
ReadOnly#(bit) wr_null_dqs_c_in        <- mkNullCrossingWire(clock_divider.slowclock,
rg_dqs_c_in);
`endif
ReadOnly#(Bool) wr_null_dqs2_out      <- mkNullCrossingWire(core_clk, onfi_nfc.dqs2_out);
ReadOnly#(bit) wr_null_dqs2_enable    <- mkNullCrossingWire(core_clk, onfi_nfc.dqs2_enable);
`ifdef Out_clock
ReadOnly#(bit) wr_null_dqs2_in        <- mkNullCrossingWire(clk0, rg_dqs2_in);
`else
ReadOnly#(bit) wr_null_dqs2_in        <- mkNullCrossingWire(clock_divider.slowclock,
rg_dqs2_in);
`endif
ReadOnly#(Bool) wr_null_dqs2_c_out    <- mkNullCrossingWire(core_clk, onfi_nfc.dqs2_c_out);
ReadOnly#(bit) wr_null_dqs2_c_enable    <- mkNullCrossingWire(core_clk, onfi_nfc.dqs2_c_enable);
`ifdef Out_clock
ReadOnly#(bit) wr_null_dqs2_c_in        <- mkNullCrossingWire(clk0, rg_dqs2_c_in);
`else
ReadOnly#(bit) wr_null_dqs2_c_in        <- mkNullCrossingWire(clock_divider.slowclock,
rg_dqs2_c_in);
`endif
ReadOnly#(Bit#(8)) wr_null_data0_out    <- mkNullCrossingWire(core_clk, onfi_nfc.data0_out);
ReadOnly#(bit) wr_null_data0_enable     <- mkNullCrossingWire(core_clk, onfi_nfc.data0_enable);
ReadOnly#(Bit#(8)) wr_null_data0_in     <- mkNullCrossingWire(clk_mux.clock_out,
rg_data0_in);
ReadOnly#(Bit#(8)) wr_null_data1_out    <- mkNullCrossingWire(core_clk, onfi_nfc.data1_out);
ReadOnly#(bit) wr_null_data1_enable     <- mkNullCrossingWire(core_clk, onfi_nfc.data1_enable);
ReadOnly#(Bit#(8)) wr_null_data1_in     <- mkNullCrossingWire(clk_mux.clock_out,
rg_data1_in);

rule rl_connection_ready_busy_crossing;
  onfi_nfc._ready_busy0_n_m(wr_null_ready_busy_0);
  onfi_nfc._ready_busy1_n_m(wr_null_ready_busy_1);
endrule

rule rl_onfi_we_n;
    if(wr_null_timing_set == 1)
        wr_we_n <= wr_null_sync_we_n;
    else
        wr_we_n <= wr_null_async_we_n;
endrule

rule rl_connection_dqs_data_crossing;
  onfi_nfc.dqs_in(wr_null_dqs_in);
  onfi_nfc.dqs_c_in(wr_null_dqs_c_in);
  onfi_nfc.dqs2_in(wr_null_dqs2_in);
  onfi_nfc.dqs2_c_in(wr_null_dqs2_c_in);
  endrule

rule rl_data_to_controller;
  onfi_nfc.data0_data_in(wr_null_data0_in);
  onfi_nfc.data1_data_in(wr_null_data1_in);
endrule


interface onfi_interface = interface Onfi_Interface_top ;
      
      method bit onfi_ce0_n_ () ;
          return wr_null_ce0_n ;
      endmethod: onfi_ce0_n_
     
      method bit onfi_ce1_n_ () ;
          return wr_null_ce1_n ;
      endmethod: onfi_ce1_n_
      	
      method bit onfi_we_n_ () ;
          return wr_we_n ;
      endmethod: onfi_we_n_

      method bit onfi_re_n_ () ;
          return wr_null_re_n ;
      endmethod: onfi_re_n_

      method bit onfi_wp_n_ () ;
          return wr_null_wp_n ;
      endmethod: onfi_wp_n_

      method bit onfi_cle_ () ;
          return wr_null_cle ;
      endmethod: onfi_cle_

      method bit onfi_ale_ () ;
          return wr_null_ale ;
      endmethod: onfi_ale_

      method Action _ready_busy0_n_m ( _ready_busy_n ) ;
          rg_ready_busy_n_0 <= _ready_busy_n;
      endmethod: _ready_busy0_n_m
  
      method Action _ready_busy1_n_m ( _ready_busy_n ) ;
          rg_ready_busy_n_1 <= _ready_busy_n;
      endmethod: _ready_busy1_n_m

      method Bool dqs_out;
          return wr_null_dqs_out;
      endmethod

      method Bit#(1) dqs_enable;
          return wr_null_dqs_enable;
      endmethod

      method Action dqs_in(Bit#(1) dqin);
          rg_dqs_in <= dqin;
      endmethod                   

      method Bool dqs_c_out;
          return wr_null_dqs_c_out;
      endmethod

      method Bit#(1) dqs_c_enable;
          return wr_null_dqs_c_enable;
      endmethod

      method Action dqs_c_in(Bit#(1) dqcin);
          rg_dqs_c_in <= dqcin;
      endmethod

      method Bool dqs2_out;
          return wr_null_dqs2_out;
      endmethod

      method Bit#(1) dqs2_enable;
          return wr_null_dqs2_enable;
      endmethod

      method Action dqs2_in(Bit#(1) dqin);
          rg_dqs2_in <= dqin;
      endmethod                   

      method Bool dqs2_c_out;
          return wr_null_dqs2_c_out;
      endmethod

      method Bit#(1) dqs2_c_enable;
          return wr_null_dqs2_enable;
      endmethod

      method Action dqs2_c_in(Bit#(1) dqcin);
          rg_dqs2_c_in <= dqcin;
      endmethod

      method Bit#(8) data0_out;
          return wr_null_data0_out;
      endmethod

      method Bit#(1) data0_enable;
          return wr_null_data0_enable;
      endmethod

      method Action data0_data_in(Bit#(8) data0_in);
          rg_data0_in <= data0_in;
      endmethod

      method Bit#(8) data1_out;
          return wr_null_data1_out;
      endmethod

      method Bit#(1) data1_enable;
          return wr_null_data1_enable;
      endmethod

      method Action data1_data_in(Bit#(8) data1_in);
          rg_data1_in <= data1_in;
      endmethod
		
endinterface;

   interface axi4_slave_onfi = s_xactor_onfi.axi_side;
   interface axi4_slave_onfi_cfg_reg = s_xactor_onfi_cfg_reg.axi_side;
   interface clk_new = clk_inv;            

endmodule
endpackage
