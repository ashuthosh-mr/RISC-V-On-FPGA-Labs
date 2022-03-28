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
Author: P.George
Email id: command.paul@gmail.com
--------------------------------------------------------------------------------------------------
*/

package riscvDebug013_testbench;

  import riscvDebug013::*;
  import hart_template::*;

  import StmtFSM::*;
  import Connectable:: *;
  import GetPut::*;
  import debug_types::*;
  import AXI4_Fabric:: *;
  import bram::*;

  // These tests are bigger than they seem , disable then selectively to improve compile time 

  // Function MACROS For Scalable Test writing
  `define FIVO(x) fromInteger(valueOf(x))

  `define DMI_READ(x) device.dtm.putCommand.put({x,32'd0,2'b01});         \
    action                                                                \
      let resp <- device.dtm.getResponse.get();                           \
      $display($time,"\tDTM:: \tADDR: %h\tDATA: %h\tOP: %h\t-> %h,%h",    \
        x,32'd0,2'b01,resp[33:2],resp[1:0]);                              \
      dmi_resp_data <= resp[33:2];                                        \
    endaction

  `define DMI_WRITE(x,y) device.dtm.putCommand.put({x,y,2'b10});          \
    action                                                                \
      let resp <- device.dtm.getResponse.get();                           \
      $display($time,"\tDTM:: \tADDR: %h\tDATA: %h\tOP: %h\t-> %h,%h",    \
        x,y,2'b10,resp[33:2],resp[1:0]);                                  \
    endaction

  // AXI Fabric Slave Address Decoder
  function Tuple2 #(Bool, Bit#(1)) fn_slave_map (Bit#(DPADDR) addr);
    Bool slave_exist = True;
    Bit#(1) slave_num = 0;
    if(addr >= 0 && addr<= 32'h000fffff )
      slave_num = 0;
    else
      slave_exist = False;
    return tuple2(slave_exist, slave_num);
  endfunction:fn_slave_map

  (*synthesize*)
  module mkdummy(Empty);
    /*      Test Environment    */
    // Hardcoded for PADDR 32 AND XLEN 32
    Hart_Debug_Ifc hart <- mkHartTemplate();
    Ifc_riscvDebug013 device <- mkriscvDebug013();

    // AXI4_Fabric_IFC #(`Num_Masters, `Num_Slaves, PADDR, XLEN, USERSPACE)
    AXI4_Fabric_IFC #(1,2,32,32,0)  fabric <- mkAXI4_Fabric(fn_slave_map);
    Ifc_bram_axi4   #(32,32,0,18)   main_memory0 <- mkbram_axi4('h00000000,"test.mem","test.mem","mem");

    mkConnection (device.debug_master,fabric.v_from_masters[0]);
    mkConnection (fabric.v_to_slaves[0],main_memory0.slave);
    mkConnection (hart,device.hart);

    Reg#(Bit#(7))   dmi_address   <-  mkReg(0);
    Reg#(Bit#(32))  dmi_resp_data <-  mkReg(0);
    Reg#(Bit#(32))  i             <-  mkReg(0); // Iteration index
    Reg#(Bit#(32))  imax          <-  mkReg(0); // Iteration index bound
    /*      Test Sequences      */

    // resetDM with DM Active
    Stmt resetDM = seq
      $display($time,"RST\tReseting DM");
      `DMI_WRITE(`FIVO(DMCONTROL),({31'd0,1'b0})) 
      `DMI_WRITE(`FIVO(DMCONTROL),({31'd0,1'b1}))
    endseq;
    FSM fsm_resetDM <- mkFSM(resetDM);

    // test0 Put - Get DMI - Zero Out and Read Back , Zeroing DmActive Resets the DM
    Stmt test0 = seq
      for( dmi_address <= 0 ; dmi_address <= 7'h40; dmi_address <= dmi_address +1)seq
        `DMI_WRITE(dmi_address,32'h00000000)
      endseq
      for( dmi_address <= 0 ; dmi_address <= 7'h40; dmi_address <= dmi_address +1)seq
        `DMI_READ(dmi_address)
      endseq
    endseq;
    FSM fsm_test0 <- mkFSM(test0);

    Stmt test_reset_values = seq
      for( dmi_address <= 0 ; dmi_address <= 7'h40; dmi_address <= dmi_address +1)seq
        `DMI_WRITE(dmi_address,32'hFFFFFFFF)
      endseq
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      for( dmi_address <= 0 ; dmi_address <= 7'h40; dmi_address <= dmi_address +1)seq
        `DMI_READ(dmi_address)
      endseq
    endseq;
    FSM fsm_test_reset_values <- mkFSM(test_reset_values);
    //    System Bus Access Tests   

    // sbTest0 Busy bits get set on Write,and get cleared on W1C
    Stmt sbTest0 = seq
      `DMI_READ(`FIVO(SBCS))
      `DMI_READ(`FIVO(SBDATA0))
      `DMI_READ(`FIVO(SBCS))          // No Busy Bits
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:21],1'b1,dmi_resp_data[19:17],1'b1,1'b1,dmi_resp_data[14:0]}))
      `DMI_WRITE(`FIVO(SBDATA0),32'hAAAAAAAA)
      `DMI_READ(`FIVO(SBCS))          // sbBusy Should be asserted
      `DMI_WRITE(`FIVO(SBDATA0),32'hAAAAAAAA)
      `DMI_READ(`FIVO(SBCS))          // Sb Busy Error should be asserted
      if(dmi_resp_data[22:21] != 2'b11)
        $display("FAIL: Busy bitS not set !");
      while(dmi_resp_data[21] == 1'b1 )seq
        `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
      endseq
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:23],1'b1,dmi_resp_data[21:0]})) // Clear sbBusyError
      `DMI_READ(`FIVO(SBCS))          // Sb Busy Error should be de asserted
      `DMI_WRITE(`FIVO(SBDATA0),32'hBBBBBBBB)
      if(dmi_resp_data[22] == 1'b1)
        $display("FAIL: Busy bit is set !");
    endseq;
    FSM fsm_sbTest0 <- mkFSM(sbTest0);

    // sbTest1
    Stmt sbTest1 = seq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:21],1'b1,dmi_resp_data[19:17],1'b1,1'b1,dmi_resp_data[14:0]}))
      `DMI_WRITE(`FIVO(SBADDRESS0),32'h0000ffff)        // Mis Aligned Address sets SbError to 3
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:15],3'b111,dmi_resp_data[11:0]}))
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBADDRESS0),32'h0000fffc)
      `DMI_READ(`FIVO(SBDATA0))    // Dont Wait and get error set
      `DMI_READ(`FIVO(SBCS))
      while(dmi_resp_data[21] == 1'b1 )seq
        `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
      endseq
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:23],1'b1,dmi_resp_data[21:0]})) // Clear sbBusyError
      `DMI_READ(`FIVO(SBDATA0))
      `DMI_READ(`FIVO(SBCS))
      while(dmi_resp_data[21] == 1'b1 )seq
        `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
      endseq
      `DMI_WRITE(`FIVO(SBADDRESS0),32'hFFFFFFF0)
      `DMI_READ(`FIVO(SBDATA0))
      `DMI_READ(`FIVO(SBCS)) 
      while(dmi_resp_data[21] == 1'b1 )seq
        `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
      endseq
    endseq;
    FSM fsm_sbTest1 <- mkFSM(sbTest1);

    // sbTest2 - Increment
    // Success should read back 0,1,2,3,4,9,66660006,66660007,66660008,66660009
    Stmt sb_test2 = seq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:17],1'b1,dmi_resp_data[15:0]})) // Set autoIncrement
      `DMI_READ(`FIVO(SBCS)) // Read Back
      for(i<=0;i<5;i<=i+1)seq
        `DMI_WRITE(`FIVO(SBDATA0),i) // Write to Data 0 addresses should be incrementing
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
      endseq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:17],1'b0,dmi_resp_data[15:0]})) //reset autoIncrement
      for(i<=5;i<10;i<=i+1)seq
        `DMI_WRITE(`FIVO(SBDATA0),i) // Write to Data 0 addresses should be incrementing
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
      endseq
      // Read Final Mem Config at zero offset
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:21],1'b1,dmi_resp_data[19:0]})) // Setup Read on Address
      for(i<=0;i<10;i<=i+1)seq
        `DMI_WRITE(`FIVO(SBADDRESS0),(i*4)) // Write to Data 0 addresses should be incrementing
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
        `DMI_READ(`FIVO(SBDATA0))
      endseq
    endseq;
    FSM fsm_sb_test2 <- mkFSM(sb_test2);

    // sbTest3 - Increment - read Address
    // Success should read back 66660000,66660004,66660008,6666000c,66660010 
    //                        - 66660000,66660004,66660008,6666000c,66660010
    //                        - No More REad Requests Generated for this test.
    // In effect the presence of aut increment does not affect read on address.
    Stmt sb_test3 = seq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:21],1'b1,dmi_resp_data[19:17],1'b1,dmi_resp_data[15:0]})) // Set autoIncrement + read on Address
      `DMI_READ(`FIVO(SBCS)) // Read Back verify
      for(i<=0;i<5;i<=i+1)seq
        `DMI_WRITE(`FIVO(SBADDRESS0),(i*16)) // Read every 4th word , should ignore auto incremented value
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
      endseq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:17],1'b0,dmi_resp_data[15:0]})) //reset autoIncrement
      for(i<=0;i<5;i<=i+1)seq
        `DMI_WRITE(`FIVO(SBADDRESS0),(i*16)) // Read every 4th word , should ignore auto incremented value
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
      endseq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:21],1'b0,dmi_resp_data[19:0]})) //reset read on address
      `DMI_READ(`FIVO(SBCS))
      for(i<=0;i<5;i<=i+1)seq
        `DMI_WRITE(`FIVO(SBADDRESS0),(i*16)) // Read every 4th word , should ignore auto incremented value
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
      endseq
    endseq;
    FSM fsm_sb_test3 <- mkFSM(sb_test3);

    // sbTest4 - Increment - read Data
    // Success should read back - 66660000,66660001,66660002,66660003,66660004
    //                          - 66660005,66660005,66660005,66660005,66660005
    //                          - No Further Read Requests issued 

    Stmt sb_test4 = seq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:17],1'b1,1'b1,dmi_resp_data[14:0]}))// Set autoIncrement ,read on Data
      `DMI_READ(`FIVO(SBCS)) // Read Back verify
      for(i<=0;i<5;i<=i+1)seq
        `DMI_READ(`FIVO(SBDATA0))
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
      endseq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:17],1'b0,dmi_resp_data[15:0]})) //reset autoIncrement
      for(i<=0;i<5;i<=i+1)seq
        `DMI_READ(`FIVO(SBDATA0)) // Shoiuld just read the last value multiple times 
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
      endseq
      `DMI_READ(`FIVO(SBCS))
      `DMI_WRITE(`FIVO(SBCS),({dmi_resp_data[31:16],1'b0,dmi_resp_data[14:0]})) //reset read on data
      for(i<=0;i<5;i<=i+1)seq
        `DMI_READ(`FIVO(SBDATA0))   // Should not read anything
        `DMI_READ(`FIVO(SBCS))
        while(dmi_resp_data[21] == 1'b1 )seq
          `DMI_READ(`FIVO(SBCS))      // poll on sbBusy
        endseq
      endseq
    endseq;
    FSM fsm_sb_test4 <- mkFSM(sb_test4);

    // Tests For Abstract Commands
    // abst_test0 
    // This Test attempts to read the entirety of the Abstract command address range for hart 0
    Stmt abst_test0 = seq
      $display("abst_test0");
      imax <= 10;
      for(i<=0;i<imax;i <= i +1 )seq
        `DMI_WRITE(`FIVO(COMMAND),({8'd0,1'b0,3'd2,1'b1,2'b01,1'b0,i[15:0]}))
        `DMI_READ(`FIVO(ABSTRACTCTS))
        while(dmi_resp_data[12] == 1)seq   // poll on abst_busy 
          `DMI_READ(`FIVO(ABSTRACTCTS))
        endseq
      endseq
    endseq;
    FSM fsm_abst_test0 <- mkFSM(abst_test0);
        
    // abst_test1
    // This Test attempts to write 0 and read back the entirety of the Abstract command address range for hart 0
    Stmt abst_test1 = seq
      $display("abst_test1");
      imax <= 10;
      `DMI_WRITE(`FIVO(ABSTRACTDATASTART),32'hffffffff)
      `DMI_WRITE((`FIVO(ABSTRACTDATASTART)+1),32'hffffffff)
      for(i<=0;i<imax;i <= i +1 )seq
        `DMI_WRITE(`FIVO(COMMAND),({8'd0,1'b0,3'd2,1'b1,2'b01,1'b1,i[15:0]}))
        `DMI_READ(`FIVO(ABSTRACTCTS))
        while(dmi_resp_data[12] == 1)seq   // poll on abst_busy 
          `DMI_READ(`FIVO(ABSTRACTCTS))
        endseq
      endseq
      for(i<=0;i<imax;i <= i +1 )seq
        `DMI_WRITE(`FIVO(COMMAND),({8'd0,1'b0,3'd2,1'b1,2'b01,1'b0,i[15:0]}))
        `DMI_READ(`FIVO(ABSTRACTCTS))
        while(dmi_resp_data[12] == 1)seq   // poll on abst_busy 
          `DMI_READ(`FIVO(ABSTRACTCTS))
        endseq
      endseq
    endseq;
    FSM fsm_abst_test1 <- mkFSM(abst_test1);
    
    // abst_test2
    // Attempt Abstract command for a hart that does not exist ( Hartsel is not hart 0)
    
    // abst_test3
    // TEst Abstract Auto for all Data registers 

    // abst_test4
    // Trigger Abstract Errors for un supported functions 

    // abst_test4
    // Trigger Abstract Errors for un supported functions 
    

    // Hart Selection and Run control tests

    /*        Test Driver      */
    Stmt testBench = seq
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_test_reset_values.start;
      fsm_test_reset_values.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_test0.start;
      fsm_test0.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_sbTest0.start;
      fsm_sbTest0.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_sbTest1.start;
      fsm_sbTest1.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_sb_test2.start;
      fsm_sb_test2.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_sb_test3.start;
      fsm_sb_test3.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_sb_test4.start;
      fsm_sb_test4.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_abst_test0.start;
      fsm_abst_test0.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      fsm_abst_test1.start;
      fsm_abst_test1.waitTillDone;
      fsm_resetDM.start;
      fsm_resetDM.waitTillDone;
      delay(100);
      $display($time,"\tEnd of Test");
      $finish();
    endseq;
    FSM tests <- mkFSM(testBench);

    rule startTests;
      tests.start;
    endrule

    rule timeout;
      let x <- $time();
      if(x >= 1000000)begin
        $finish();
        $display("Timeout");
      end
    endrule
  endmodule
endpackage

