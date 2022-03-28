/* 
Copyright (c) 2013, IIT Madras All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions
  and the following disclaimer.  
* Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and / or other materials provided 
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

Author : Anmol
Email id : anmol.sahoo25@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/

/*
    Transactor typeclasses
*/

package FabricTransactors;

/*
    Package imports
*/

import AXI4_Lite_Types::*;
import AXI4_Lite_Fabric::*;
import AXI4_Types:: *;
import AXI4_Fabric:: *;

/*
    Declaring a type-class for the core transactor
    The typeclass defines that any instance,
    must atleast provide a module called getTransactor.
*/

typeclass CoreTransactor#(
    type interface_type,
    type rd_addr_type,
    type rd_resp_type,
    type wr_addr_type,
    type wr_data_type,
    type wr_resp_type,
    type fabric_resp_type
)
dependencies(
    interface_type determines (
        rd_addr_type, 
        rd_resp_type, 
        wr_addr_type, 
        wr_data_type,
        wr_resp_type,
        fabric_resp_type
    )
);

    // Function to create transactor
    module getTransactor (interface_type);

    // Function to create packets
    function rd_addr_type get_read_req_packet();
    function wr_addr_type get_write_req_packet();
    function wr_data_type get_write_data_packet();
    
    // Function to send and receive packets
    function fabric_resp_type get_okay();

endtypeclass

/*
    AXI4 master transactor instance
    Here we define an instance, that lets us create the AXI4 Master Transactor
*/
instance CoreTransactor#(
    AXI4_Master_Xactor_IFC#(wd_addr, wd_data, wd_user),
    AXI4_Rd_Addr          #(wd_addr, wd_user),
    AXI4_Rd_Data          #(wd_data, wd_user),
    AXI4_Wr_Addr          #(wd_addr, wd_user),
    AXI4_Wr_Data          #(wd_data),
    AXI4_Wr_Resp          #(wd_user),
    AXI4_Resp
);
    // Master transactor module
    module getTransactor (AXI4_Master_Xactor_IFC#(wd_addr, wd_data, wd_user));
        let ifc();
        mkAXI4_Master_Xactor _temp(ifc);
        return ifc;
    endmodule

    // Function to create a read address packet
    function AXI4_Rd_Addr#(wd_addr, wd_user) get_read_req_packet();
        return AXI4_Rd_Addr {};
    endfunction

    // Function to create a write address packet
    function AXI4_Wr_Addr#(wd_addr, wd_user) get_write_req_packet();
        return AXI4_Wr_Addr {};
    endfunction

    // Function to create a write data packet
    function AXI4_Wr_Data#(wd_data) get_write_data_packet();
        return AXI4_Wr_Data {};
    endfunction

    function AXI4_Resp get_okay();
        return AXI4_OKAY;
    endfunction
endinstance

endpackage : FabricTransactors