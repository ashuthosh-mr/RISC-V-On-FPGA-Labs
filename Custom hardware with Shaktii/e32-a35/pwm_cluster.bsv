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
package pwm_cluster;
	import AXI4_Lite_Types:: *;
	import AXI4_Lite_Fabric:: *;
  import pwm::*;
  import err_slave::*;
  import Connectable:: *;
  import GetPut:: *;
  
  `include "Soc.defines"

  function Bit#(TLog#(`PWMCluster_Num_Slaves)) fn_slave_map (Bit#(`paddr) addr);
    Bool slave_exist = True;
    Bit#(TLog#(`PWMCluster_Num_Slaves)) slave_num = 0;
    if(     addr>= `PWM0Base && addr<= `PWM0End )
      slave_num =  `PWM0_slave_num;
    else if(addr>= `PWM1Base && addr<= `PWM1End )
      slave_num =  `PWM1_slave_num;
    else if(addr>= `PWM2Base && addr<= `PWM2End )
      slave_num =  `PWM2_slave_num;
    else if(addr>= `PWM3Base && addr<= `PWM3End )
      slave_num =  `PWM3_slave_num;
    else if(addr>= `PWM4Base && addr<= `PWM4End )
      slave_num =  `PWM4_slave_num;
    else if(addr>= `PWM5Base && addr<= `PWM5End )
      slave_num =  `PWM5_slave_num;
    else
      slave_num = `PWMCluster_err_slave_num;
      
    return slave_num;
  endfunction:fn_slave_map

  interface Ifc_pwm_cluster;
    interface PWMIO pwm0_io;
    interface PWMIO pwm1_io;
    interface PWMIO pwm2_io;
    interface PWMIO pwm3_io;
    interface PWMIO pwm4_io;
    interface PWMIO pwm5_io;
    (*always_ready, always_enabled*)
    method Bit#(1) pwm0_sb_interrupt;
    (*always_ready, always_enabled*)
    method Bit#(1) pwm1_sb_interrupt;
    (*always_ready, always_enabled*)
    method Bit#(1) pwm2_sb_interrupt;
    (*always_ready, always_enabled*)
    method Bit#(1) pwm3_sb_interrupt;
    (*always_ready, always_enabled*)
    method Bit#(1) pwm4_sb_interrupt;
    (*always_ready, always_enabled*)
    method Bit#(1) pwm5_sb_interrupt;
    interface AXI4_Lite_Slave_IFC#(`paddr, 32, 0) slave;
  endinterface

  (*synthesize*)
  module mkpwm(Ifc_pwm_axi4lite#(`paddr, 32, 0, 16));
	  let core_clock<-exposeCurrentClock;
  	let core_reset<-exposeCurrentReset;
    let ifc();
    mkpwm_axi4lite#(core_clock, core_reset) _temp(ifc);
    return ifc;
  endmodule

  (*synthesize*)
  module mkpwm_cluster(Ifc_pwm_cluster);
    let curr_clk<- exposeCurrentClock;
    let curr_reset <- exposeCurrentReset;
		AXI4_Lite_Master_Xactor_IFC #(`paddr, 32, 0) c2m_xactor <- mkAXI4_Lite_Master_Xactor;
		AXI4_Lite_Slave_Xactor_IFC #(`paddr, 32, 0) c2s_xactor <- mkAXI4_Lite_Slave_Xactor;
    AXI4_Lite_Fabric_IFC #(`PWMCluster_Num_Masters, `PWMCluster_Num_Slaves, `paddr, 32,0) 
                                                    fabric <- mkAXI4_Lite_Fabric(fn_slave_map);
    let pwm0 <- mkpwm();
    let pwm1 <- mkpwm();
    let pwm2 <- mkpwm();
    let pwm3 <- mkpwm();
    let pwm4 <- mkpwm();
    let pwm5 <- mkpwm();
    Ifc_err_slave_axi4lite#(`paddr, 32, 0 ) err_slave <- mkerr_slave_axi4lite;
   	
   	mkConnection(c2m_xactor.axi_side, fabric.v_from_masters[0]);

    mkConnection(c2s_xactor.o_wr_addr,c2m_xactor.i_wr_addr);
    mkConnection(c2s_xactor.o_wr_data,c2m_xactor.i_wr_data);
    mkConnection(c2m_xactor.o_wr_resp,c2s_xactor.i_wr_resp);
    mkConnection(c2s_xactor.o_rd_addr,c2m_xactor.i_rd_addr);
    mkConnection(c2m_xactor.o_rd_data,c2s_xactor.i_rd_data);
		
 	  mkConnection (fabric.v_to_slaves [`PWM0_slave_num ],pwm0.slave);
 	  mkConnection (fabric.v_to_slaves [`PWM1_slave_num ],pwm1.slave);
 	  mkConnection (fabric.v_to_slaves [`PWM2_slave_num ],pwm2.slave);
 	  mkConnection (fabric.v_to_slaves [`PWM3_slave_num ],pwm3.slave);
 	  mkConnection (fabric.v_to_slaves [`PWM4_slave_num ],pwm4.slave);
 	  mkConnection (fabric.v_to_slaves [`PWM5_slave_num ],pwm5.slave);
    mkConnection (fabric.v_to_slaves [`PWMCluster_err_slave_num ] , err_slave.slave);

    interface pwm0_io=pwm0.io;
    interface pwm1_io=pwm1.io;
    interface pwm2_io=pwm2.io;
    interface pwm3_io=pwm3.io;
    interface pwm4_io=pwm4.io;
    interface pwm5_io=pwm5.io;
    method pwm0_sb_interrupt=pwm0.sb_interrupt;
    method pwm1_sb_interrupt=pwm1.sb_interrupt;
    method pwm2_sb_interrupt=pwm2.sb_interrupt;
    method pwm3_sb_interrupt=pwm3.sb_interrupt;
    method pwm4_sb_interrupt=pwm4.sb_interrupt;
    method pwm5_sb_interrupt=pwm5.sb_interrupt;
    interface slave= c2s_xactor.axi_side;
  endmodule
endpackage

