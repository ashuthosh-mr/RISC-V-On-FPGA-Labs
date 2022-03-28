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
package spi_cluster;
	import AXI4_Lite_Types:: *;
	import AXI4_Lite_Fabric:: *;
  import spi::*;
  import err_slave::*;
  import Connectable:: *;
  import GetPut:: *;
  
  `include "Soc.defines"

  function Bit#(TLog#(`SPICluster_Num_Slaves)) fn_slave_map (Bit#(`paddr) addr);
    Bool slave_exist = True;
    Bit#(TLog#(`SPICluster_Num_Slaves)) slave_num = 0;
    if(     addr>= `SPI0Base && addr<= `SPI0End )
      slave_num =  `SPI0_slave_num;
    else if(addr>= `SPI1Base && addr<= `SPI1End )
      slave_num =  `SPI1_slave_num;
    /*else if(addr>= `SPI2Base && addr<= `SPI2End )
      slave_num =  `SPI2_slave_num;*/
    else
      slave_num = `SPICluster_err_slave_num;
      
    return slave_num;
  endfunction:fn_slave_map

  interface Ifc_spi_cluster;
    interface Ifc_spi_io spi0_io;
    interface Ifc_spi_io spi1_io;
//    interface Ifc_spi_io spi2_io;
    interface AXI4_Lite_Slave_IFC#(`paddr, 32, 0) slave;
  endinterface

  (*synthesize*)
  module mkspi(Ifc_spi_controller#(32, 32, 0));
	  let core_clock<-exposeCurrentClock;
  	let core_reset<-exposeCurrentReset;
    let ifc();
    mkspi_controller#(core_clock, core_reset) _temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
  module mkspi_cluster(Ifc_spi_cluster);
    let curr_clk<- exposeCurrentClock;
    let curr_reset <- exposeCurrentReset;
		AXI4_Lite_Master_Xactor_IFC #(`paddr, 32, 0) c2m_xactor <- mkAXI4_Lite_Master_Xactor;
		AXI4_Lite_Slave_Xactor_IFC #(`paddr, 32, 0) c2s_xactor <- mkAXI4_Lite_Slave_Xactor;
    AXI4_Lite_Fabric_IFC #(`SPICluster_Num_Masters, `SPICluster_Num_Slaves, `paddr, 32,0) 
                                                    fabric <- mkAXI4_Lite_Fabric(fn_slave_map);
    let spi0 <- mkspi();
    let spi1 <- mkspi();
//    let spi2 <- mkspi();
    Ifc_err_slave_axi4lite#(`paddr, 32, 0 ) err_slave <- mkerr_slave_axi4lite;
   	
   	mkConnection(c2m_xactor.axi_side, fabric.v_from_masters[0]);

    mkConnection(c2s_xactor.o_wr_addr,c2m_xactor.i_wr_addr);
    mkConnection(c2s_xactor.o_wr_data,c2m_xactor.i_wr_data);
    mkConnection(c2m_xactor.o_wr_resp,c2s_xactor.i_wr_resp);
    mkConnection(c2s_xactor.o_rd_addr,c2m_xactor.i_rd_addr);
    mkConnection(c2m_xactor.o_rd_data,c2s_xactor.i_rd_data);
		
 	  mkConnection (fabric.v_to_slaves [`SPI0_slave_num ],spi0.slave);
 	  mkConnection (fabric.v_to_slaves [`SPI1_slave_num ],spi1.slave);
// 	  mkConnection (fabric.v_to_slaves [`SPI2_slave_num ],spi2.slave);
    mkConnection (fabric.v_to_slaves [`SPICluster_err_slave_num ] , err_slave.slave);

    interface spi0_io=spi0.io;
    interface spi1_io=spi1.io;
//    interface spi2_io=spi2.io;
    interface slave= c2s_xactor.axi_side;
  endmodule
endpackage

