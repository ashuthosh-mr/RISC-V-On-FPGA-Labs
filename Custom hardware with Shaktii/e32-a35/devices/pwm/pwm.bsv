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
*/
package pwm;
	

	`include "pwm.defines"


	import Clocks::*;
	import ClockDiv::*;
	import ConcatReg::*;
	import Semi_FIFOF::*;
	import BUtils ::*;
 	import AXI4_Lite_Types::*;
  import AXI4_Types::*;

  export Ifc_pwm_axi4lite   (..);
  export Ifc_pwm_axi4       (..);
  export mkpwm_axi4;
  export mkpwm_axi4lite;
  export PWMIO              (..);

  typedef enum {Byte=0, HWord=1, Word=2, DWord=3} AccessSize deriving(Bits,Eq,FShow);

  (*always_ready, always_enabled*)
	interface PWMIO;
		method Bit#(1) pwm_o;
	endinterface
	interface User_ifc#(numeric type addr_width, numeric type data_width, numeric type pwmwidth);
		method  ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data, AccessSize size);
		method ActionValue#(Tuple2#(Bool, Bit#(data_width))) read_req(Bit#(addr_width) addr, AccessSize size);
		interface PWMIO io;
    method Bit#(1) sb_interrupt;
	endinterface
	//generic module
	module mkpwm#(Clock ext_clock, Reset ext_reset)(User_ifc#(addr_width,data_width,pwmwidth))
			provisos(
				Add#(a__, pwmwidth, data_width),
				Add#(b__, 8, data_width),
				Mul#(8, c__, data_width),
				Mul#(pwmwidth, d__, data_width)
				);
			let bus_clock <- exposeCurrentClock;
		    let bus_reset <- exposeCurrentReset;

		    Reg#(Bit#(pwmwidth)) period <- mkReg(0);
		    Reg#(Bit#(pwmwidth)) duty_cycle <- mkReg(0);
		    Reg#(Bit#(pwmwidth)) clock_divisor <- mkReg(0);
		    // =========== Control registers ================== //
		    Reg#(Bit#(1)) clock_selector <- mkReg(0);     // bit-0
		    Reg#(Bit#(1)) pwm_enable <- mkReg(0);         // bit-1
		    Reg#(Bit#(1)) pwm_start  <- mkReg(0);         // bit-2
		    Reg#(Bit#(1)) continous_once <- mkReg(0);     // bit-3
		    Reg#(Bit#(1)) pwm_output_enable <- mkReg(0);  // bit-4
		    Reg#(Bit#(1)) interrupt <- mkReg(0);          // bit-5
		    Reg#(Bit#(1)) reset_counter <- mkReg(0);      // bit-7
		    Reg#(Bit#(8)) control = concatReg8(reset_counter, readOnlyReg(0), readOnlyReg(interrupt), 
		                                       pwm_output_enable, continous_once, pwm_start, pwm_enable, 
		                                       clock_selector);


		    // ================================================ //

	    // Generate a reset signal is there is a reset from the bus interface of if the reset_counter
	    // bit in the control register is set. The new reset is called overall_reset. Only the counter
	    // and the output signals need to be reset by this.
	    MakeResetIfc control_reset <- mkReset(1,False, bus_clock);
	    rule generate_reset;
	      if(reset_counter==1)
	        control_reset.assertReset;
	    endrule
	    Reset overall_reset <- mkResetEither(bus_reset,control_reset.new_rst);

	    // Select between bus clock or external clock 
	    MuxClkIfc clock_selection <- mkUngatedClockMux(ext_clock,bus_clock);
	    Reset async_reset <- mkAsyncResetFromCR(0,clock_selection.clock_out);
	    rule select_busclk_extclk;
	      clock_selection.select(clock_selector==1);
	    endrule

	    // The following register is required to transfer the divisor value from bus_clock to 
	    // external clock domain. This is necessary if the clock divider needs to operate on the
	    // external clock. In this case, the divisor value should also come from the same clock domain.
	    Reg#(Bit#(pwmwidth)) clock_divisor_sync <- mkSyncRegFromCC(0, clock_selection.clock_out); 
	    rule transfer_data_from_clock_domains;
	      clock_divisor_sync <= clock_divisor;
	    endrule
	    
	    // The PWM can operate on a slowed-down clock. The following module generates a slowed-down
	    // clock based on the value given in register divisor. Since the clock_divider works on a muxed
	    // clock domain of the external clock or bus_clock, the divisor (which operates on the bus_clock
	    // will have to be synchronized and sent to the divider
	    Ifc_ClockDiv#(pwmwidth) clock_divider <- mkClockDiv(clocked_by clock_selection.clock_out, 
	                                         reset_by async_reset);
	    let downclock = clock_divider.slowclock; 
	    Reset downreset <- mkAsyncReset(0,overall_reset,downclock);
	    rule generate_slow_clock;
	      clock_divider.divisor(clock_divisor_sync);
	    endrule

	    // ======= Actual Counter and PWM signal generation ======== //
	    Reg#(Bit#(1)) pwm_output <- mkReg(0,clocked_by downclock,reset_by downreset);
	    Reg#(Bit#(pwmwidth)) rg_counter <-mkReg(0,clocked_by downclock,reset_by downreset); 
	    
	    // create synchronizers for clock domain crossing.
	    Reg#(Bit#(1)) sync_pwm_output <- mkSyncRegToCC(0,downclock,downreset);
	    ReadOnly#(Bit#(1)) pwm_signal <- mkNullCrossingWire(bus_clock, pwm_output);
	    Reg#(Bit#(1)) sync_continous_once <- mkSyncRegFromCC(0,downclock);
	    Reg#(Bit#(pwmwidth)) sync_duty_cycle <- mkSyncRegFromCC(0,downclock);
	    Reg#(Bit#(pwmwidth)) sync_period <- mkSyncRegFromCC(0,downclock);
	    Reg#(Bit#(1)) sync_pwm_enable <- mkSyncRegFromCC(0,downclock);
	    Reg#(Bit#(1)) sync_pwm_start <- mkSyncRegFromCC(0,downclock);
	    
	    rule sync_pwm_output_to_default_clock;
	      sync_pwm_output <= pwm_output;
	    endrule

	    // capture the synchronized values from the default clock domain to the downclock domain for
	    // actual timer and pwm functionality.
	    rule sync_from_default_to_downclock;
	      sync_continous_once <= continous_once;
	      sync_duty_cycle <= duty_cycle;
	      sync_period <= period;
	      sync_pwm_enable <= pwm_enable;
	      sync_pwm_start <= pwm_start;
	    endrule
	    let temp = sync_period==0?0:sync_period-1;

	    // This rule generates the interrupt in the timer mode and resets it if the user-write interface
	    // writes a value of 1 to the reset_counter bit.
	    rule generate_interrupt_in_timer_mode;
	      if(pwm_enable==0)
	        interrupt <= sync_pwm_output;
	      else if(reset_counter==1)
	        interrupt <= 0;
	      else
	        interrupt <= 0;
	    endrule

	    // This rule performs the actual pwm and the timer functionality. if pwm_enable is 1 then the
	    // PWM mode is selected. Every time the counter value equals/crosses the period value it is
	    // reset and the output pwm_output signal is toggled. 
	    // The timer mode is selected when pwm_enable is 0. Here again 2 more modes are possible. if the
	    // continous_once bit is 0 then the timer is in one time. In this case once the counter reaches
	    // the period value it raises an interrupt and stops the counter. In the continuous mode
	    // however, when the counter reaches the period value the interrupt is raise, the counter is
	    // reset to 0 and continues counting. During continuous counting the interrupt can be cleared by
	    // the user but will be set back when the counter reaches the period value.
	    rule compare_and_generate_pwm(sync_pwm_start==1);
	      let cntr = rg_counter+1;
	      if(sync_pwm_enable==1)begin // PWM mode enabled
	        if(rg_counter >= temp)
	          rg_counter <= 0;
	        else 
	          rg_counter <= cntr;
	        if(rg_counter < sync_duty_cycle)
	          pwm_output <= 1;
	        else
	          pwm_output <= 0;
	      end
	      else begin  // Timer mode enabled.
	        if(sync_continous_once==0) begin // One time mode.
	          if(rg_counter >= temp)begin
	              pwm_output <= 1;
	          end
	          else begin
							pwm_output <= 0;
	            rg_counter <= cntr;
			  end
	        end
	        else begin // Continous mode.
	          if(rg_counter >= temp)begin
	            pwm_output <= 1;
	            rg_counter <= 0;
	          end
	         else begin
							pwm_output <= 0;
	            rg_counter <= cntr;
	          end
	        end
	      end
	    endrule

			method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data, AccessSize size);
				Bool success =True;
				let pw=valueOf(pwmwidth);
				AccessSize allowed_access=(pw==8)?Byte:(pw==16)?HWord:Word;
				if(addr [3:0] ==`ControlReg && size==Byte)
					control<=truncate(data);
				else if( addr [3:0] ==`PeriodReg && allowed_access==size)
					period<=truncate(data);
				else if( addr [3:0] ==`DutyReg && allowed_access==size)
					duty_cycle<=truncate(data);
				else if( addr [3:0] ==`ClockReg && allowed_access==size)
					clock_divisor<=truncate(data);
				else
					 success=False;
				 return success;
			endmethod:write_req

			method ActionValue#(Tuple2#(Bool, Bit#(data_width))) read_req(Bit#(addr_width) addr, AccessSize size);
				Bool success= True;
				let pw=valueOf(pwmwidth);
				Bit#(data_width) data=0;
				AccessSize allowed_access=(pw==8)?Byte:(pw==16)?HWord:Word;
				if(addr[3:0]==`ControlReg && size==Byte)
					data=duplicate(control);
				else if( addr [3:0] ==`PeriodReg && allowed_access==size)
					data=duplicate(period);
				else if( addr [3:0] ==`DutyReg && allowed_access==size)
					data=duplicate(duty_cycle);
				else if( addr [3:0] ==`ClockReg && allowed_access==size)
					data=duplicate(clock_divisor);
				else
					 success=False;
				 return tuple2(success,data);
				
			endmethod
			interface io=interface PWMIO
				method pwm_o=pwm_output_enable==1?pwm_signal:0;
			endinterface;
      method sb_interrupt=interrupt;

	endmodule:mkpwm

	//axi4lite
	interface Ifc_pwm_axi4lite#(numeric type addr_width, numeric type data_width, numeric type user_width, numeric type pwmwidth);
		interface AXI4_Lite_Slave_IFC#(addr_width, data_width, user_width) slave;
		interface PWMIO io;
    method Bit#(1) sb_interrupt;
	endinterface

	module mkpwm_axi4lite#(Clock ext_clock, Reset ext_reset)(Ifc_pwm_axi4lite#(addr_width,data_width,user_width, pwmwidth))
		provisos(
				Add#(a__, pwmwidth, data_width),
				Add#(b__, 8, data_width),
				Mul#(8, c__, data_width),
				Mul#(pwmwidth, d__, data_width)
				);
		User_ifc#(addr_width,data_width,pwmwidth) pwm <-mkpwm(ext_clock, ext_reset);
		AXI4_Lite_Slave_Xactor_IFC#(addr_width,data_width,user_width)  s_xactor <- mkAXI4_Lite_Slave_Xactor();

		rule read_request;
	  		let req <- pop_o (s_xactor.o_rd_addr);
      		let {succ,data} <- pwm.read_req(req.araddr,unpack(truncate(req.arsize)));
	  		let resp= AXI4_Lite_Rd_Data {rresp:succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, 
                                    rdata:data, ruser: ?};
	  		s_xactor.i_rd_data.enq(resp);
     endrule

     rule write_request;
       let addreq <- pop_o(s_xactor.o_wr_addr);
       let datareq <- pop_o(s_xactor.o_wr_data);
       let succ <- pwm.write_req(addreq.awaddr, datareq.wdata,unpack(truncate(addreq.awsize)));
       let resp = AXI4_Lite_Wr_Resp {bresp: succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, buser: ?};
       s_xactor.i_wr_resp.enq(resp);
     endrule

     interface io = pwm.io;
     interface slave = s_xactor.axi_side;
     method sb_interrupt=pwm.sb_interrupt;
	endmodule

	//axi4
	interface Ifc_pwm_axi4#(numeric type addr_width, numeric type data_width, numeric type user_width, numeric type pwmwidth);
		interface AXI4_Slave_IFC#(addr_width,data_width,user_width)	slave;
		interface PWMIO io;
    method Bit#(1) sb_interrupt;
	endinterface
	module mkpwm_axi4#(Clock ext_clock, Reset ext_reset)(Ifc_pwm_axi4#(addr_width,data_width,user_width, pwmwidth))
	provisos(
				Add#(a__, pwmwidth, data_width),
				Add#(b__, 8, data_width),
				Mul#(8, c__, data_width),
				Mul#(pwmwidth, d__, data_width)
				);
		User_ifc#(addr_width,data_width,pwmwidth) pwm <-mkpwm(ext_clock, ext_reset);
		AXI4_Slave_Xactor_IFC#(addr_width,data_width,user_width) s_xactor<-mkAXI4_Slave_Xactor();
		Reg#(Bit#(8)) rg_rdburst_count <- mkReg(0);
		Reg#(Bit#(8)) rg_wrburst_count <- mkReg(0);

		Reg#(AXI4_Rd_Addr#(addr_width,user_width)) rg_rdpacket <- mkReg(?);
 		Reg#(AXI4_Wr_Addr#(addr_width,user_width)) rg_wrpacket <- mkReg(?);


		rule read_request(rg_rdburst_count==0);
			let req<-pop_o(s_xactor.o_rd_addr);
			let {succ,data}<-pwm.read_req(req.araddr,unpack(truncate(req.arsize)));
			rg_rdpacket<=req;	

			if(req.arlen!=0)
				rg_rdburst_count<=1;
			let resp= AXI4_Rd_Data{rresp:succ?AXI4_OKAY:AXI4_SLVERR, rid:req.arid,rlast:(req.arlen==0),rdata:data, ruser: ?};
			s_xactor.i_rd_data.enq(resp);
		endrule
		rule read_burst(rg_rdburst_count!=0);
			let rd_req=rg_rdpacket;
			let {succ,data}<-pwm.read_req(rd_req.araddr,unpack(truncate(rd_req.arsize)));
			succ=False;
			if(rg_rdburst_count==rd_req.arlen)
				rg_rdburst_count<=0;
			else
				rg_rdburst_count<=rg_rdburst_count+1;
			let resp= AXI4_Rd_Data{rresp:succ?AXI4_OKAY:AXI4_SLVERR, rid:rd_req.arid,rlast:(rd_req.arlen==0),rdata:data, ruser: ?};
			s_xactor.i_rd_data.enq(resp);
		endrule

		rule write_request(rg_wrburst_count==0);
			let addreq <- pop_o(s_xactor.o_wr_addr);
	        let datareq <- pop_o(s_xactor.o_wr_data);
	        rg_wrpacket<=addreq;
	        let succ <- pwm.write_req(addreq.awaddr, datareq.wdata,unpack(truncate(addreq.awsize)));
	        if(addreq.awlen!=0)
	        	rg_wrburst_count<=1;
	        
	        let resp = AXI4_Wr_Resp {bresp: succ?AXI4_SLVERR:AXI4_OKAY, buser: ?, bid:addreq.awid};
	        if(datareq.wlast)
	        	s_xactor.i_wr_resp.enq(resp);
		endrule
		rule write_burst(rg_wrburst_count!=0);
			let addreq=rg_wrpacket;
	        let datareq <- pop_o(s_xactor.o_wr_data);
	 		Bool succ=False;
			let resp = AXI4_Wr_Resp {bresp: succ?AXI4_SLVERR:AXI4_OKAY, buser: ?, bid:addreq.awid};
			if(datareq.wlast)begin
	      		s_xactor.i_wr_resp.enq(resp);//enqueuing the write response
	      		rg_wrburst_count<=0;
	      	end

		endrule

    method sb_interrupt=pwm.sb_interrupt;
		interface io=pwm.io;
		interface slave = s_xactor.axi_side;
	endmodule

endpackage
