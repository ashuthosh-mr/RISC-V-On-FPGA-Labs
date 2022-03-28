/* 
Copyright (c) 2019, IIT Madras All rights reserved.

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
package mixed_cluster;
	import AXI4_Lite_Types:: *;
	import AXI4_Lite_Fabric:: *;
  import err_slave::*;
  import Connectable:: *;
  import GetPut:: *;
  
  import i2c :: * ;
  import gpio :: * ;
  import clint :: * ;
  import plic :: * ;
  import pinmux :: * ;
  import pinmux_axi4lite :: * ;
  `include "Soc.defines"

  interface Ifc_mixed_cluster;
		method I2C_out i2c0_out;									//I2c IO interface
		method I2C_out i2c1_out;									//I2c IO interface
    method Bit#(1) sb_ext_interrupt;
    (*always_ready, always_enabled*)
    interface GPIO#(32) gpio_io;						//GPIO IO interface
    interface IOCellSide pinmuxtop_iocell_side;
    interface PeripheralSide pinmuxtop_peripheral_side;
    (*always_ready, always_enabled*)
		method Action interrupts(Bit#(9) inp);
    interface AXI4_Lite_Slave_IFC#(`paddr, 32, 0) slave;
    interface AXI4_Lite_Master_IFC#(`paddr, 32, 0) xadc_master;
  endinterface

  (*synthesize*)
  module mki2c (Ifc_i2c_axi4lite#(`paddr, 32, 0));
	  let core_clock<-exposeCurrentClock;
  	let core_reset<-exposeCurrentReset;
    let ifc();
    mki2c_axi4lite#(core_clock, core_reset) _temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
  module mkgpio(Ifc_gpio_axi4lite#(`paddr, 32, 0, 32));
    let ifc();
    mkgpio_axi4lite _temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
  module mkplic(Ifc_plic_axi4lite#(`paddr, 32, 0, 28, 2, 0));
    let ifc();
    mkplic_axi4lite#(`PLICBase)_temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
  module mkpinmuxtop(Ifc_pinmux_axi4lite#(`paddr, 32, 0));
    let ifc();
    mkpinmux_axi4lite _temp(ifc);
    return ifc;
  endmodule

  function Bit#(TLog#(`MixedCluster_Num_Slaves)) fn_slave_map (Bit#(`paddr) addr);
    Bit#(TLog#(`MixedCluster_Num_Slaves)) slave_num = 0;
    if(addr>= `GPIOBase && addr<= `GPIOEnd)
      slave_num = `GPIO_slave_num;
    else if(addr>= `PLICBase && addr<= `PLICEnd)
      slave_num = `PLIC_slave_num;
    else if(addr>= `I2C1Base && addr<= `I2C1End)
      slave_num = `I2C1_slave_num;
    else if(addr>= `I2C0Base && addr<= `I2C0End)
      slave_num = `I2C0_slave_num;
    else if(addr>= `XADCBase && addr<= `XADCEnd)
      slave_num = `XADC_slave_num;
    else if(addr >= `PinmuxBase && addr <= `PinmuxEnd)
      slave_num = `Pinmux_slave_num;
    else
      slave_num = `MixedCluster_err_slave_num;
      
    return slave_num;
  endfunction:fn_slave_map

  (*synthesize*)
  module mkmixed_cluster(Ifc_mixed_cluster);
	
		AXI4_Lite_Master_Xactor_IFC #(`paddr, 32, 0) c2m_xactor <- mkAXI4_Lite_Master_Xactor;
		AXI4_Lite_Slave_Xactor_IFC #(`paddr, 32, 0) c2s_xactor <- mkAXI4_Lite_Slave_Xactor;
    AXI4_Lite_Fabric_IFC #(`MixedCluster_Num_Masters, `MixedCluster_Num_Slaves, `paddr, 32,0) 
                                                    fabric <- mkAXI4_Lite_Fabric(fn_slave_map);
    let i2c0 <- mki2c;
    let i2c1 <- mki2c;
    let gpio <- mkgpio();
    let plic <- mkplic();
    let pinmuxtop <- mkpinmuxtop();
    Ifc_err_slave_axi4lite#(`paddr, 32, 0 ) err_slave <- mkerr_slave_axi4lite;
		Wire#(Bit#(9)) wr_external_interrupts <- mkDWire('d0);
    Wire#(Bit#(1)) wr_sb_ext_interrupt <- mkDWire(0);

		//Rule to connect PLIC interrupt to the core's sideband
		rule rl_core_plic_connection;
			let {lv_plic_intr, x}<- plic.intrpt_note_sb.get;
			wr_sb_ext_interrupt <= pack(lv_plic_intr);
		endrule

    rule rl_connect_plic_connections;
			let tmp <- gpio.sb_gpio_to_plic.get;
			Bit#(16) lv_gpio_intr= truncate(pack(tmp));
			Bit#(28) plic_inputs= {wr_external_interrupts[8:6], i2c1.isint, i2c0.isint, lv_gpio_intr, wr_external_interrupts[5:0],0};
			plic.ifc_external_irq_io(plic_inputs);
		endrule

   	mkConnection(c2m_xactor.axi_side, fabric.v_from_masters[0]);

    mkConnection(c2s_xactor.o_wr_addr,c2m_xactor.i_wr_addr);
    mkConnection(c2s_xactor.o_wr_data,c2m_xactor.i_wr_data);
    mkConnection(c2m_xactor.o_wr_resp,c2s_xactor.i_wr_resp);
    mkConnection(c2s_xactor.o_rd_addr,c2m_xactor.i_rd_addr);
    mkConnection(c2m_xactor.o_rd_data,c2s_xactor.i_rd_data);
		
   	mkConnection (fabric.v_to_slaves [`I2C0_slave_num ],		i2c0.slave);
   	mkConnection (fabric.v_to_slaves [`I2C1_slave_num ],		i2c1.slave);
		mkConnection (fabric.v_to_slaves [`PLIC_slave_num ], plic.slave);
		mkConnection (fabric.v_to_slaves [`GPIO_slave_num ], gpio.slave);
		mkConnection (fabric.v_to_slaves [`Pinmux_slave_num ], pinmuxtop.slave);
    mkConnection (fabric.v_to_slaves [`MixedCluster_err_slave_num ] , err_slave.slave);
		
    method I2C_out i2c0_out= i2c0.io;
    method I2C_out i2c1_out= i2c1.io;
    method sb_ext_interrupt = wr_sb_ext_interrupt;
    interface gpio_io= gpio.io;
		method Action interrupts(Bit#(9) inp);
			wr_external_interrupts<= inp;
		endmethod
    interface pinmuxtop_iocell_side = pinmuxtop.pinmuxaxi4lite_iocell_side;
    interface pinmuxtop_peripheral_side = pinmuxtop.pinmuxaxi4lite_peripheral_side;
    interface slave= c2s_xactor.axi_side;
    interface xadc_master = fabric.v_to_slaves[`XADC_slave_num];
  endmodule
endpackage

