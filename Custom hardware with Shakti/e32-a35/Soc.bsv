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

Author: Neel Gala
Email id: neelgala@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/
package Soc;
  // project related imports
	import Semi_FIFOF:: *;
	import AXI4_Lite_Types:: *;
	import AXI4_Lite_Fabric:: *;
  import Clocks :: *;
  import common_types::*;
  `include "Soc.defines"

  // peripheral imports
  import clint::*;
  import err_slave::*;
  import eclass:: * ;
  import pwm_cluster :: * ;
  import uart_cluster :: * ;
  import spi_cluster :: * ;
  import mixed_cluster :: * ;
  import uart :: *;
  import spi :: *;
  import pwm :: *;
  import i2c :: *;
  import gpio :: *;
  import bram :: *;
  import debug_types::*;     
  import xilinxdtm::*;                                                                                                                                                                                                
  import riscvDebug013::*;                                                                        
  import debug_halt_loop::*;
  import debug_types::*;
  import pinmux::*;
  import pinmux_axi4lite :: * ;                

  // package imports
  import Connectable:: *;
  import GetPut:: *;
  import Vector::*;

 
  function Bit#(TLog#(`Num_Slaves)) fn_slave_map (Bit#(`paddr) addr);
    Bit#(TLog#(`Num_Slaves)) slave_num = 0;
    if(addr >= `MemoryBase && addr<= `MemoryEnd)
      slave_num = `Memory_slave_num;
    else if(addr >= `Memory2Base && addr<= `Memory2End)
      slave_num = `Memory2_slave_num;
    else if(addr >= `ClintBase && addr <= `ClintEnd)
      slave_num = `Clint_slave_num;
    else if(addr >= `DebugBase && addr <= `DebugEnd)
      slave_num = `Debug_slave_num;
    else if(addr >= `PWMClusterBase && addr <= `PWMClusterEnd)
      slave_num = `PWMCluster_slave_num;
    else if(addr >= `UARTClusterBase && addr <= `UARTClusterEnd)
      slave_num = `UARTCluster_slave_num;
    else if(addr >= `SPIClusterBase && addr <= `SPIClusterEnd)
      slave_num = `SPICluster_slave_num;
    else if(addr >= `MixedClusterBase && addr <= `MixedClusterEnd)
      slave_num = `MixedCluster_slave_num;
    else if(addr >= `PLICBase && addr <= `PLICEnd)
      slave_num = `MixedCluster_slave_num;
    else if(addr >= `BootBase && addr <= `BootEnd)
      slave_num = `Boot_slave_num;
    else
      slave_num = `Err_slave_num;
      
    return slave_num;
  endfunction:fn_slave_map

  interface Ifc_Soc;
    interface Ifc_spi_io spi0_io;
//    interface Ifc_spi_io spi2_io;
    interface RS232 uart0_io;
		method I2C_out i2c0_out;									//I2c IO interface
		method I2C_out i2c1_out;									//I2c IO interface
    (*always_ready, always_enabled*)
    interface AXI4_Lite_Master_IFC#(`paddr, 32, 0) xadc_master;
    interface IOCellSide iocell_io;
    (*always_enabled,always_ready*)
    method Action  gpio_4(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_7(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_8(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_14(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_15(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_16(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_17(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_18(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_19(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_20(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_21(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_22(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_23(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_24(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_25(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_26(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_27(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_28(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_29(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_30(Bit#(1) in);
    (*always_enabled,always_ready*)
    method Action  gpio_31(Bit#(1) in);

    (*always_enabled,always_ready*)
    method Bit#(1)gpio_4_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_7_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_8_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_14_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_15_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_16_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_17_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_18_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_19_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_20_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_21_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_22_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_23_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_24_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_25_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_26_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_27_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_28_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_29_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_30_out;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_31_out;

    (*always_enabled,always_ready*)
    method Bit#(1)gpio_4_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_7_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_8_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_14_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_15_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_16_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_17_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_18_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_19_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_20_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_21_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_22_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_23_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_24_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_25_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_26_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_27_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_28_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_29_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_30_outen;
    (*always_enabled,always_ready*)
    method Bit#(1)gpio_31_outen;
// ------------- JTAG IOs ----------------------//
    (*always_enabled,always_ready*)
    method Action wire_tms(Bit#(1) tms_in);
    (*always_enabled,always_ready*)
    method Action wire_tdi(Bit#(1) tdi_in);
  `ifdef bscan2e //---  Shift Register Control ---//
    (*always_enabled,always_ready*)
    method Action wire_capture(Bit#(1) capture_in);
    (*always_enabled,always_ready*)
    method Action wire_run_test(Bit#(1) run_test_in);
    (* always_enabled,always_ready*)
    method Action wire_sel (Bit#(1) sel_in);
    (* always_enabled,always_ready*)
    method Action wire_shift (Bit#(1) shift_in);
    (* always_enabled,always_ready*)
    method Action wire_update (Bit#(1) update_in);
  `endif
    (*always_enabled,always_ready*)
    method Bit#(1) wire_tdo;                                                            
    // ---------------------------------------------//
    (*always_ready, always_enabled*)
    method Action ext_interrupts(Bit#(2) i);
  `ifdef rtldump
    interface Get#(DumpType) io_dump;
  `endif
  endinterface
    
  (*synthesize*)
  module mkeclass(Ifc_eclass_axi4lite);
    let ifc();
    mkeclass_axi4lite#(`resetpc) _temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
  module mkSoc#(Clock tck_clk, Reset trst)(Ifc_Soc);
    let curr_clk<-exposeCurrentClock;
    let curr_reset<-exposeCurrentReset;

    let eclass <- mkeclass();

    AXI4_Lite_Fabric_IFC #(`Num_Masters, `Num_Slaves, `paddr, XLEN, USERSPACE) 
                                                        fabric <- mkAXI4_Lite_Fabric(fn_slave_map);
    Ifc_clint_axi4lite#(`paddr, XLEN, 0, 1, 16) clint <- mkclint_axi4lite();
    Ifc_debug_halt_loop_axi4lite#(`paddr, XLEN, USERSPACE) debug_memory <-
                                                                        mkdebug_halt_loop_axi4lite;
    Ifc_pwm_cluster pwm_cluster <- mkpwm_cluster;
    Ifc_uart_cluster uart_cluster <- mkuart_cluster;
    Ifc_spi_cluster spi_cluster <- mkspi_cluster;
    Ifc_mixed_cluster mixed_cluster <- mkmixed_cluster;
    Ifc_err_slave_axi4lite#(`paddr,XLEN,0) err_slave <- mkerr_slave_axi4lite;
    Ifc_bram_axi4lite#(`paddr, XLEN, 0,  17) mem <- mkbram_axi4lite('h80000000, "code.mem","Memory");
    Ifc_bram_axi4lite#(`paddr, XLEN, 0,  16) mem2 <- mkbram_axi4lite('h80020000, "code1.mem","Memory");
    Ifc_bram_axi4lite#(`paddr, XLEN, 0,  13) boot <- mkbram_axi4lite('h1000, "boot.mem","Boot");
    Wire#(Bit#(2)) wr_ext_interrutps <- mkWire();

    Wire#(Bit#(1)) wr_gpio4_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio7_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio8_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio14_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio15_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio16_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio17_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio18_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio19_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio20_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio21_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio22_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio23_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio24_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio25_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio26_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio27_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio28_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio29_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio30_in <- mkDWire(0);
    Wire#(Bit#(1)) wr_gpio31_in <- mkDWire(0);
    // -------------------------------- JTAG + Debugger Setup ---------------------------------- //
    // null crossing registers to transfer input signals from current_domain to tck domain
    CrossingReg#(Bit#(1)) tdi<-mkNullCrossingReg(tck_clk,0);                                        
    CrossingReg#(Bit#(1)) tms<-mkNullCrossingReg(tck_clk,0);
`ifdef bscan2e
    CrossingReg#(Bit#(1)) capture <- mkNullCrossingReg(tck_clk,0);
    CrossingReg#(Bit#(1)) run_test <- mkNullCrossingReg(tck_clk,0);
    CrossingReg#(Bit#(1)) sel <- mkNullCrossingReg(tck_clk,0);
    CrossingReg#(Bit#(1)) shift <- mkNullCrossingReg(tck_clk,0);
    CrossingReg#(Bit#(1)) update <- mkNullCrossingReg(tck_clk,0);   
`endif                                        
    // null crossing registers to transfer signals from tck to curr_clock domain.
    CrossingReg#(Bit#(1)) tdo<-mkNullCrossingReg(curr_clk,0,clocked_by tck_clk, reset_by trst);     
    // Tap Controller jtag_tap
    `ifdef bscan2e
      Ifc_xilinxdtm jtag_tap <- mkxilinxdtm(clocked_by tck_clk, reset_by trst);                                         
    `else
      Ifc_jtagdtm jtag_tap <- mkjtagdtm(clocked_by tck_clk, reset_by trst);            
    `endif
    Ifc_riscvDebug013 debug_module <- mkriscvDebug013();                                           

    // synFIFOs to transact data between JTAG and debug module                                                                                                    
    SyncFIFOIfc#(Bit#(41)) sync_request_to_dm <-mkSyncFIFOToCC(1,tck_clk,trst);                     
    SyncFIFOIfc#(Bit#(34)) sync_response_from_dm <-mkSyncFIFOFromCC(1,tck_clk);                     
                           
		Wire#(Bit#(8)) wr_external_interrupts <- mkDWire('d0);
             // ----------- Connect JTAG IOs through null-crossing registers ------ //
    rule assign_jtag_inputs;                                                                                
      jtag_tap.tms_i(tms.crossed);                                                                  
      jtag_tap.tdi_i(tdi.crossed);                                                                  
      `ifdef bscan2e
        jtag_tap.capture_i(capture.crossed);
      	jtag_tap.run_test_i(run_test.crossed);
      	jtag_tap.sel_i(sel.crossed);
      	jtag_tap.shift_i(shift.crossed);
      	jtag_tap.update_i(update.crossed);                                                                   
      `endif
    endrule                                                                                         
                                                                                                    
    rule assign_jtag_output;                                                                                 
      tdo <= jtag_tap.tdo(); //  Launched by a register clocked by inverted tck                     
    endrule                                                                                        
            // ------------------------------------------------------------------- //

    // capture jtag tap request into a syncfifo first.                                                                                                                  
    rule connect_tap_request_to_syncfifo;                                                           
      let x<-jtag_tap.request_to_dm;                                                                
      sync_request_to_dm.enq(zeroExtend(x));          
    endrule                                                                                         

    // send captured synced jtag tap request to the debug module
    rule read_synced_request_to_dm;                                                                 
      sync_request_to_dm.deq;                                                                       
      debug_module.dtm.putCommand.put(sync_request_to_dm.first);                                    
    endrule                                                                                         

    // collect debug response into a syncfifo
    rule connect_debug_response_to_syncfifo;                                                        
      let x <- debug_module.dtm.getResponse.get;                                                    
      sync_response_from_dm.enq(x);          
    endrule                                  

    // send synced debug response back to the JTAG
    rule read_synced_response_from_dm;                                                              
      sync_response_from_dm.deq;                                                                    
      jtag_tap.response_from_dm(sync_response_from_dm.first);                                       
    endrule                                                                                         

    rule connect_interrupt_lines;
      mixed_cluster.interrupts({uart_cluster.uart_interrupts, pwm_cluster.pwm0_sb_interrupt, 
                                                  pwm_cluster.pwm1_sb_interrupt, 
                                                  pwm_cluster.pwm2_sb_interrupt, 
                                                  pwm_cluster.pwm3_sb_interrupt, 
                                                  pwm_cluster.pwm4_sb_interrupt, 
                                                  pwm_cluster.pwm5_sb_interrupt});
    endrule
    
    mkConnection (eclass.debug_server ,debug_module.hart);
      
    // ------------------------------------------------------------------------------------------//
    mkConnection(debug_module.debug_master,fabric.v_from_masters[`Debug_master_num]);
   	mkConnection(eclass.master_d,	fabric.v_from_masters[`Mem_master_num]);
   	mkConnection(eclass.master_i, fabric.v_from_masters[`Fetch_master_num]);

  	mkConnection (fabric.v_to_slaves [`Clint_slave_num ],clint.slave);
    mkConnection (fabric.v_to_slaves [`Debug_slave_num ] , debug_memory.slave);
    mkConnection (fabric.v_to_slaves [`PWMCluster_slave_num], pwm_cluster.slave);
    mkConnection (fabric.v_to_slaves [`UARTCluster_slave_num], uart_cluster.slave);
    mkConnection (fabric.v_to_slaves [`SPICluster_slave_num], spi_cluster.slave);
    mkConnection (fabric.v_to_slaves [`MixedCluster_slave_num], mixed_cluster.slave);
    mkConnection (fabric.v_to_slaves [`Err_slave_num ] , err_slave.slave);
	  mkConnection (fabric.v_to_slaves [`Memory_slave_num], mem.slave);
	  mkConnection (fabric.v_to_slaves [`Memory2_slave_num], mem2.slave);
	  mkConnection (fabric.v_to_slaves [`Boot_slave_num], boot.slave);

    // sideband connection
    mkConnection(eclass.sb_clint_msip,clint.sb_clint_msip);
    mkConnection(eclass.sb_clint_mtip,clint.sb_clint_mtip);
    mkConnection(eclass.sb_clint_mtime,clint.sb_clint_mtime);

    rule connect_ext_plic_interrupt;
      eclass.sb_ext_interrupt.put(mixed_cluster.sb_ext_interrupt);
    endrule

    rule connect_pinmux_peripheral_output_lines;
      mixed_cluster.pinmuxtop_peripheral_side.pwm0.out.put(pwm_cluster.pwm0_io.pwm_o);
      mixed_cluster.pinmuxtop_peripheral_side.pwm1.out.put(pwm_cluster.pwm1_io.pwm_o);
      mixed_cluster.pinmuxtop_peripheral_side.pwm2.out.put(pwm_cluster.pwm2_io.pwm_o);
      mixed_cluster.pinmuxtop_peripheral_side.pwm3.out.put(pwm_cluster.pwm3_io.pwm_o);
      mixed_cluster.pinmuxtop_peripheral_side.pwm4.out.put(pwm_cluster.pwm4_io.pwm_o);
      mixed_cluster.pinmuxtop_peripheral_side.pwm5.out.put(pwm_cluster.pwm5_io.pwm_o);

		  mixed_cluster.pinmuxtop_peripheral_side.gpioa.out.put(
						{ mixed_cluster.gpio_io.gpio_out[13],
						  mixed_cluster.gpio_io.gpio_out[12],
						  mixed_cluster.gpio_io.gpio_out[11],
						  mixed_cluster.gpio_io.gpio_out[10],
						  mixed_cluster.gpio_io.gpio_out[9],
						  mixed_cluster.gpio_io.gpio_out[6],
						  mixed_cluster.gpio_io.gpio_out[5],
						  mixed_cluster.gpio_io.gpio_out[3],
						  mixed_cluster.gpio_io.gpio_out[2],
						  mixed_cluster.gpio_io.gpio_out[1],
    					mixed_cluster.gpio_io.gpio_out[0]} );
		  mixed_cluster.pinmuxtop_peripheral_side.gpioa.out_en.put(
						{ mixed_cluster.gpio_io.gpio_out_en[13],
						  mixed_cluster.gpio_io.gpio_out_en[12],
						  mixed_cluster.gpio_io.gpio_out_en[11],
						  mixed_cluster.gpio_io.gpio_out_en[10],
						  mixed_cluster.gpio_io.gpio_out_en[9],
						  mixed_cluster.gpio_io.gpio_out_en[6],
						  mixed_cluster.gpio_io.gpio_out_en[5],
						  mixed_cluster.gpio_io.gpio_out_en[3],
						  mixed_cluster.gpio_io.gpio_out_en[2],
						  mixed_cluster.gpio_io.gpio_out_en[1],
						  mixed_cluster.gpio_io.gpio_out_en[0]} );

		   mixed_cluster.pinmuxtop_peripheral_side.mspi.clk.put(spi_cluster.spi1_io.sclk);
		   mixed_cluster.pinmuxtop_peripheral_side.mspi.nss.put(spi_cluster.spi1_io.nss);
		   mixed_cluster.pinmuxtop_peripheral_side.mspi.mosi.put(spi_cluster.spi1_io.mosi);

		   mixed_cluster.pinmuxtop_peripheral_side.uart1.tx.put(uart_cluster.uart1_io.sout);
		   mixed_cluster.pinmuxtop_peripheral_side.uart2.tx.put(uart_cluster.uart2_io.sout);
    endrule

    rule connect_pinmux_peripheral_input_lines;
		  let pinmux_gpio_in <- (mixed_cluster.pinmuxtop_peripheral_side.gpioa.in.get);
      let gpio_in_combined = unpack({
			  wr_gpio31_in,
			  wr_gpio30_in,
			  wr_gpio29_in,
			  wr_gpio28_in,
			  wr_gpio27_in,
			  wr_gpio26_in,
			  wr_gpio25_in,
			  wr_gpio24_in,
			  wr_gpio23_in,
			  wr_gpio22_in,
			  wr_gpio21_in,
			  wr_gpio20_in,
			  wr_gpio19_in,
			  wr_gpio18_in,
			  wr_gpio17_in,
			  wr_gpio16_in,
			  wr_gpio15_in,
			  wr_gpio14_in,
			  pinmux_gpio_in[10],
			  pinmux_gpio_in[9],
			  pinmux_gpio_in[8],
			  pinmux_gpio_in[7],
			  pinmux_gpio_in[6],
			  wr_gpio8_in,
			  wr_gpio7_in,
			  pinmux_gpio_in[5],
			  pinmux_gpio_in[4],
			  wr_gpio4_in,
			  pinmux_gpio_in[3],
			  pinmux_gpio_in[2],
			  pinmux_gpio_in[1],
			  pinmux_gpio_in[0]	});
		   mixed_cluster.gpio_io.gpio_in(gpio_in_combined);

		   let pinmux_spi1_miso <- mixed_cluster.pinmuxtop_peripheral_side.mspi.miso.get;
		   spi_cluster.spi1_io.miso(pinmux_spi1_miso);

		   let pinmux_uart1_rx <- (mixed_cluster.pinmuxtop_peripheral_side.uart1.rx.get);
		   uart_cluster.uart1_io.sin(pinmux_uart1_rx);
		   let pinmux_uart2_rx <- (mixed_cluster.pinmuxtop_peripheral_side.uart2.rx.get);
		   uart_cluster.uart2_io.sin(pinmux_uart2_rx);
    endrule

    // ------------- JTAG IOs ----------------------//
    method Action wire_tms(Bit#(1)tms_in);                                                        
      tms <= tms_in;                                                                              
    endmethod                    
    method Action wire_tdi(Bit#(1)tdi_in);                                                        
      tdi <= tdi_in;                                                                              
    endmethod                                                                                     
    `ifdef bscan2e
      method Action wire_capture(Bit#(1) capture_in);
        capture <= capture_in;
      endmethod
      method Action wire_run_test(Bit#(1) run_test_in);
        run_test <= run_test_in;
      endmethod
      method Action wire_sel (Bit#(1) sel_in);
        sel <= sel_in;
      endmethod
      method Action wire_shift (Bit#(1) shift_in);
        shift <= shift_in;
      endmethod
      method Action wire_update (Bit#(1) update_in);
        update <= update_in;
      endmethod
    `endif
    method Bit#(1)wire_tdo;                                                                       
      return tdo.crossed();                                                                       
    endmethod
    method Action  gpio_4(Bit#(1) in);
         wr_gpio4_in <= in;
    endmethod
    method Action  gpio_7(Bit#(1) in);
         wr_gpio7_in <= in;
    endmethod
    method Action  gpio_8(Bit#(1) in);
         wr_gpio8_in <= in;
    endmethod
    method Action  gpio_14(Bit#(1) in);
         wr_gpio14_in <= in;
    endmethod
    method Action  gpio_15(Bit#(1) in);
         wr_gpio15_in <= in;
    endmethod
    method Action  gpio_16(Bit#(1) in);
         wr_gpio16_in <= in;
    endmethod
    method Action  gpio_17(Bit#(1) in);
         wr_gpio17_in <= in;
    endmethod
    method Action  gpio_18(Bit#(1) in);
         wr_gpio18_in <= in;
    endmethod
    method Action  gpio_19(Bit#(1) in);
         wr_gpio19_in <= in;
    endmethod
    method Action  gpio_20(Bit#(1) in);
         wr_gpio20_in <= in;
    endmethod
    method Action  gpio_21(Bit#(1) in);
         wr_gpio21_in <= in;
    endmethod
    method Action  gpio_22(Bit#(1) in);
         wr_gpio22_in <= in;
    endmethod
    method Action  gpio_23(Bit#(1) in);
         wr_gpio23_in <= in;
    endmethod
    method Action  gpio_24(Bit#(1) in);
         wr_gpio24_in <= in;
    endmethod
    method Action  gpio_25(Bit#(1) in);
         wr_gpio25_in <= in;
    endmethod
    method Action  gpio_26(Bit#(1) in);
         wr_gpio26_in <= in;
    endmethod
    method Action  gpio_27(Bit#(1) in);
         wr_gpio27_in <= in;
    endmethod
    method Action  gpio_28(Bit#(1) in);
         wr_gpio28_in <= in;
    endmethod
    method Action  gpio_29(Bit#(1) in);
         wr_gpio29_in <= in;
    endmethod
    method Action  gpio_30(Bit#(1) in);
         wr_gpio30_in <= in;
    endmethod
    method Action  gpio_31(Bit#(1) in);
         wr_gpio31_in <= in;
    endmethod

    method Bit#(1)gpio_4_out = mixed_cluster.gpio_io.gpio_out[4];
    method Bit#(1)gpio_7_out = mixed_cluster.gpio_io.gpio_out[7];
    method Bit#(1)gpio_8_out = mixed_cluster.gpio_io.gpio_out[8];
    method Bit#(1)gpio_14_out = mixed_cluster.gpio_io.gpio_out[14];
    method Bit#(1)gpio_15_out = mixed_cluster.gpio_io.gpio_out[15];
    method Bit#(1)gpio_16_out = mixed_cluster.gpio_io.gpio_out[16];
    method Bit#(1)gpio_17_out = mixed_cluster.gpio_io.gpio_out[17];
    method Bit#(1)gpio_18_out = mixed_cluster.gpio_io.gpio_out[18];
    method Bit#(1)gpio_19_out = mixed_cluster.gpio_io.gpio_out[19];
    method Bit#(1)gpio_20_out = mixed_cluster.gpio_io.gpio_out[20];
    method Bit#(1)gpio_21_out = mixed_cluster.gpio_io.gpio_out[21];
    method Bit#(1)gpio_22_out = mixed_cluster.gpio_io.gpio_out[22];
    method Bit#(1)gpio_23_out = mixed_cluster.gpio_io.gpio_out[23];
    method Bit#(1)gpio_24_out = mixed_cluster.gpio_io.gpio_out[24];
    method Bit#(1)gpio_25_out = mixed_cluster.gpio_io.gpio_out[25];
    method Bit#(1)gpio_26_out = mixed_cluster.gpio_io.gpio_out[26];
    method Bit#(1)gpio_27_out = mixed_cluster.gpio_io.gpio_out[27];
    method Bit#(1)gpio_28_out = mixed_cluster.gpio_io.gpio_out[28];
    method Bit#(1)gpio_29_out = mixed_cluster.gpio_io.gpio_out[29];
    method Bit#(1)gpio_30_out = mixed_cluster.gpio_io.gpio_out[30];
    method Bit#(1)gpio_31_out = mixed_cluster.gpio_io.gpio_out[31];

    method gpio_4_outen = mixed_cluster.gpio_io.gpio_out_en[4];
    method gpio_7_outen = mixed_cluster.gpio_io.gpio_out_en[7];
    method gpio_8_outen = mixed_cluster.gpio_io.gpio_out_en[8];
    method gpio_14_outen = mixed_cluster.gpio_io.gpio_out_en[14];
    method gpio_15_outen = mixed_cluster.gpio_io.gpio_out_en[15];
    method gpio_16_outen = mixed_cluster.gpio_io.gpio_out_en[16];
    method gpio_17_outen = mixed_cluster.gpio_io.gpio_out_en[17];
    method gpio_18_outen = mixed_cluster.gpio_io.gpio_out_en[18];
    method gpio_19_outen = mixed_cluster.gpio_io.gpio_out_en[19];
    method gpio_20_outen = mixed_cluster.gpio_io.gpio_out_en[20];
    method gpio_21_outen = mixed_cluster.gpio_io.gpio_out_en[21];
    method gpio_22_outen = mixed_cluster.gpio_io.gpio_out_en[22];
    method gpio_23_outen = mixed_cluster.gpio_io.gpio_out_en[23];
    method gpio_24_outen = mixed_cluster.gpio_io.gpio_out_en[24];
    method gpio_25_outen = mixed_cluster.gpio_io.gpio_out_en[25];
    method gpio_26_outen = mixed_cluster.gpio_io.gpio_out_en[26];
    method gpio_27_outen = mixed_cluster.gpio_io.gpio_out_en[27];
    method gpio_28_outen = mixed_cluster.gpio_io.gpio_out_en[28];
    method gpio_29_outen = mixed_cluster.gpio_io.gpio_out_en[29];
    method gpio_30_outen = mixed_cluster.gpio_io.gpio_out_en[30];
    method gpio_31_outen = mixed_cluster.gpio_io.gpio_out_en[31];
    interface spi0_io = spi_cluster.spi0_io;
//    interface spi1_io = spi_cluster.spi1_io;
    interface uart0_io = uart_cluster.uart0_io;
		method  i2c0_out = mixed_cluster.i2c0_out;									//I2c IO interface
		method  i2c1_out = mixed_cluster.i2c1_out;									//I2c IO interface
    interface iocell_io = mixed_cluster.pinmuxtop_iocell_side;						//GPIO IO interface
    interface xadc_master = mixed_cluster.xadc_master;
    method Action ext_interrupts(Bit#(2) i);
      wr_ext_interrutps <= i;
    endmethod
  `ifdef rtldump
    interface io_dump= eclass.io_dump;
  `endif

  endmodule: mkSoc
endpackage: Soc
