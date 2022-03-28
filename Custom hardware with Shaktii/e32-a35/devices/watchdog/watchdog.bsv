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

Author : Arjun Menon
Email id : c.arjunmenon@gmail.com
Details:
Config Registers
      1. Counter Compare Reg
      2. Control Reg:
           Bit 0-   Watchdog enable
           Bit 1- 0: Interrupt mode
                  1: Reset mode
           Bit 2  1: Soft Reset
      3. Reset_duration: Number of clock cycles that reset should be held low
--------------------------------------------------------------------------------------------------
*/
package watchdog;
  import AXI4_Lite_Types::*;
  import AXI4_Lite_Fabric::*;
  import AXI4_Fabric::*;
	import AXI4_Types::*;
	import AXI4_Fabric::*;
  import Semi_FIFOF::*;
  import BUtils::*;
  import ConfigReg::*;
  `include "Logger.bsv"           // for logging

  interface Ifc_watchdog#(numeric type addr_width, numeric type data_width);
    method ActionValue#(Bool) set_register(Bit#(addr_width) addr, Bit#(data_width) data);
    method Tuple2#(Bit#(data_width), Bool) read_register(Bit#(addr_width) addr);
    (*always_ready, always_enabled*) method Bit#(1) reset_out;
    method Bit#(1) interrupt;
  endinterface

  module mkwatchdog(Integer wd_control, Integer reset_cycles, Ifc_watchdog#(addr_width, data_width) ifc)
    provisos(Add#(a__, 16, data_width));

    //Memory-mapped Config Registers
    Reg#(Bit#(data_width)) rg_watchdog_cycles <- mkConfigReg('d-1);
    Reg#(Bit#(16)) rg_reset_cycles <- mkConfigReg(fromInteger(reset_cycles));
    Reg#(Bit#(16)) rg_control <- mkConfigReg(fromInteger(wd_control));

    Reg#(Bit#(data_width)) rg_watchdog_counter <- mkConfigReg('d-1);
    Reg#(Bit#(16)) rg_reset_counter <- mkConfigReg(fromInteger(reset_cycles));
    Reg#(Bool) rg_reset_start <- mkReg(False);

    Wire#(Bool) wr_active <- mkDWire(False);

    rule rl_decrement_watchdog_counter(!rg_reset_start && rg_control[2]==0);
      if(wr_active)
        rg_watchdog_counter<= rg_watchdog_cycles;
      else if(rg_watchdog_counter!=0)
        rg_watchdog_counter<= rg_watchdog_counter-1;
    endrule

    rule rl_gen_reset_pulse(!rg_reset_start);
      //if watchdog enabled, counter reaches 0 and in Reset mode
      if(rg_control[0]==1 && rg_watchdog_counter=='d0 && rg_control[1]==1) begin
        rg_reset_start<= True;
      end
      //If Soft Reset mode
      
      else (*split*) if(rg_control[2]=='d1) begin
        rg_control[2]<= 'b0; 
        rg_watchdog_counter<= rg_watchdog_cycles; //Once soft reset, then WDT also restarts
        rg_reset_start<= True;
        rg_reset_counter<= rg_reset_cycles;
        `logLevel( wdt, 0, $format("WDT: Beginning soft reset..."))
      end
      //(*nosplit*)
      //else the reset is not set
    endrule

    rule rl_gen_reset_signal(rg_reset_start);
      rg_reset_counter<= rg_reset_counter-1;
      if(rg_reset_counter==1) begin
        `logLevel( wdt, 0, $format("WDT: Done with soft reset..."))
        rg_reset_start<= False;
      end
    endrule

    method ActionValue#(Bool) set_register(Bit#(addr_width) addr, Bit#(data_width) data) if(!rg_reset_start && rg_control[2]==0);
      if(addr[7:0]=='h0) begin
        rg_watchdog_cycles<= data;
        return True;
      end
      else if(addr[7:0]=='h8) begin
        rg_control<= truncate(data);
        return True;
      end
      else if(addr[7:0]=='h10) begin
        rg_reset_cycles<= truncate(data);
        return True;
      end
      else if(addr[7:0]=='h18) begin
        wr_active<= True;
        return True;
      end
      else
        return False;
    endmethod

    method Tuple2#(Bit#(data_width), Bool) read_register(Bit#(addr_width) addr);
      if(addr=='h0)
        return tuple2(zeroExtend(rg_watchdog_cycles), True);
      else if(addr=='h8)
        return tuple2(zeroExtend(rg_control), True);
      else if(addr=='h10)
        return tuple2(zeroExtend(rg_reset_cycles), True);
      else
        return tuple2(?, False);
    endmethod

    //Active low reset out which would reset the SoC
    method Bit#(1) reset_out;
      return pack(!rg_reset_start);
    endmethod

    method Bit#(1) interrupt;
      //if watchdog is enabled, counter reaches 0 and in Interrupt mode
      if(rg_control[0]==1 && rg_watchdog_counter=='d0 && rg_control[1]==0)
        return 1;
      else
        return 0;
    endmethod
  endmodule

  interface Ifc_watchdog_axi4lite#(numeric type addr_width, numeric type data_width, numeric type user_width);
    interface AXI4_Lite_Slave_IFC#(addr_width, data_width, user_width) slave; 
    (*always_ready, always_enabled*) method Bit#(1) reset_out;
    method Bit#(1) interrupt;
  endinterface

  module mkwatchdog_axi4lite(Reset ext_rst, Integer wd_control, Integer reset_cycles, Ifc_watchdog_axi4lite#(addr_width, data_width, user_width) ifc)
                             provisos(Add#(16, a__, data_width));
    AXI4_Lite_Slave_Xactor_IFC#(addr_width,data_width,user_width)  s_xactor <- mkAXI4_Lite_Slave_Xactor();
    Ifc_watchdog#(addr_width, data_width) wdt <- mkwatchdog(reset_by ext_rst, wd_control, reset_cycles);

    rule rl_capture_read_req;
      let rd_req <- pop_o (s_xactor.o_rd_addr);
      let {rdata,succ} = wdt.read_register(rd_req.araddr);
      let lv_resp= AXI4_Lite_Rd_Data { rresp: succ ? AXI4_LITE_OKAY : AXI4_LITE_SLVERR, 
                                       rdata: rdata,
                                       ruser: ? };
      s_xactor.i_rd_data.enq(lv_resp);//sending back the response
    endrule

    rule rl_capture_write_req;
      let wr_req  <- pop_o(s_xactor.o_wr_addr);
      let wr_data <- pop_o(s_xactor.o_wr_data);
      let succ <- wdt.set_register(wr_req.awaddr, wr_data.wdata);

      let lv_resp = AXI4_Lite_Wr_Resp { bresp: succ ? AXI4_LITE_OKAY : AXI4_LITE_SLVERR,
                                        buser: ?};
      s_xactor.i_wr_resp.enq(lv_resp);//enqueuing the write response
    endrule

    interface slave= s_xactor.axi_side;
    method reset_out= wdt.reset_out;
    method interrupt= wdt.interrupt;

  endmodule


  interface Ifc_watchdog_axi4#(numeric type addr_width, numeric type data_width, numeric type user_width);
    interface AXI4_Slave_IFC#(addr_width, data_width, user_width) slave; 
    (*always_ready, always_enabled*) method Bit#(1) reset_out;
    method Bit#(1) interrupt;
  endinterface

  module mkwatchdog_axi4(Reset ext_rst, Integer wd_control, Integer reset_cycles, Ifc_watchdog_axi4#(addr_width, data_width, user_width) ifc)
                             provisos(Add#(16, a__, data_width));
    AXI4_Slave_Xactor_IFC#(addr_width,data_width,user_width)  s_xactor <- mkAXI4_Slave_Xactor();
    Ifc_watchdog#(addr_width, data_width) wdt <- mkwatchdog(reset_by ext_rst, wd_control, reset_cycles);

    rule rl_capture_read_req;
      let rd_req <- pop_o (s_xactor.o_rd_addr);
      let {rdata,succ} = wdt.read_register(rd_req.araddr);
      let lv_resp= AXI4_Rd_Data { rresp: succ ? AXI4_OKAY : AXI4_SLVERR,
                                  rid: rd_req.arid,
                                  rlast: True,
                                  rdata: rdata,
                                  ruser: ? };
      s_xactor.i_rd_data.enq(lv_resp);//sending back the response
    endrule

    rule rl_capture_write_req;
      let wr_req  <- pop_o(s_xactor.o_wr_addr);
      let wr_data <- pop_o(s_xactor.o_wr_data);
      let succ <- wdt.set_register(wr_req.awaddr, wr_data.wdata);

      let lv_resp = AXI4_Wr_Resp { bresp: succ ? AXI4_OKAY : AXI4_SLVERR,
                                   bid: wr_req.awid,
                                   buser: ?};
      s_xactor.i_wr_resp.enq(lv_resp);//enqueuing the write response
    endrule

    interface slave= s_xactor.axi_side;
    method reset_out= wdt.reset_out;
    method interrupt= wdt.interrupt;

  endmodule
endpackage
