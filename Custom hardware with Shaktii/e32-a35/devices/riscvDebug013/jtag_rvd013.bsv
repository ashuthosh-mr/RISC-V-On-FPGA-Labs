package jtag_rvd013;
  import Vector::*;
  import FIFOF::*;
  import DReg::*;
  import SpecialFIFOs::*;
  import BRAMCore::*;
  import FIFO::*;
  import Clocks::*;
  import Connectable::*;
  import GetPut::*;
  
  import AXI4_Fabric:: *;
  import bram::*;

  import debug_types::*;
  import jtagdtm::*;
  import rbb_jtag::*;
  import riscvDebug013::*;
  import hart_template::*;

  ///////
  import "BDPI" function ActionValue #(int) init_rbb_jtag(Bit#(1) dummy);
  import "BDPI" function ActionValue #(Bit #(8))get_frame(int client_fd);
  import "BDPI" function Action send_tdo(Bit #(1) tdo , int client_fd);
  //////

  // AXI Fabric Slave Address Decoder
  function Tuple2 #(Bool, Bit#(1)) fn_slave_map (Bit#(PADDR) addr);
    Bool slave_exist = True;
    Bit#(1) slave_num = 0;
    if(addr >= 0 && addr<= 32'h000fffff )
      slave_num = 0;
    else
      slave_exist = False;
    return tuple2(slave_exist, slave_num);
  endfunction:fn_slave_map

  module mkdummy(Empty);
    Clock defaultclk <- exposeCurrentClock;

    MakeClockIfc#(Bit#(1)) tck_clk <-mkUngatedClock(1);
    MakeResetIfc trst <- mkReset(0,False,tck_clk.new_clk);

    CrossingReg#(Bit#(1)) tdi<-mkNullCrossingReg(tck_clk.new_clk,0);
    CrossingReg#(Bit#(1)) tms<-mkNullCrossingReg(tck_clk.new_clk,0);
    CrossingReg#(Bit#(1)) tdo<-mkNullCrossingReg(defaultclk,0,clocked_by tck_clk.new_clk, reset_by trst.new_rst); 

    Ifc_jtagdtm jtag_tap <- mkjtagdtm(clocked_by tck_clk.new_clk, reset_by trst.new_rst);
    // ReadOnly#(Bit#(1)) tdo_crossed <- mkNullCrossingWire(defaultclk,jtag_tap.tdo);
    //////
    // Ifc_jtag_driver_sim openocd <- mkRbbJtag(tck_clk.new_clk);
    Reg#(Bit#(1)) rg_initial <- mkReg(0);
    Reg#(Bit#(1)) rg_end_sim <- mkReg(0);

    //Reg#(int) rg_client_fd <- mkReg(32'hffffffff,clocked_by tck_clk.new_clk, reset_by trst.new_rst); // -1
    Reg#(int) rg_client_fd <- mkReg(32'hffffffff);

    rule rl_initial(rg_initial == 0);
      let x <- init_rbb_jtag(0);
      if(x != 32'hffffffff)begin
        //$display("xval = %h" , x);
        rg_initial <= 1'b1;
        rg_client_fd <= x;
      end
    endrule

    Wire#(Bit#(1)) wr_tdo <- mkWire();

    Reg#(Bit#(1)) tdi_delay <- mkReg(0);
    Reg#(Bit#(1)) tms_delay <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor2 <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor3 <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor4 <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor5 <- mkReg(0);
    // Needed to spread the Jtag signals to properly work with the neg edge sampling business withh vpis

    rule rl_get_frame((rg_initial == 1'b1));
      let x <- get_frame(rg_client_fd);
      delayed_actor <= truncate(x);
      delayed_actor2 <= delayed_actor;
      delayed_actor3 <= delayed_actor2;
      delayed_actor4 <= delayed_actor3;
      delayed_actor5 <= delayed_actor4;
      tck_clk.setClockValue(delayed_actor2[2]); // Should I delay acting to one cycle post the reception of this message  ?
      if( delayed_actor2[4] == 1 )
        trst.assertReset();
      if(delayed_actor5[3] == 1 )
        send_tdo(tdo.crossed(),rg_client_fd);
      tdi <= delayed_actor[0];
      tms <= delayed_actor[1];
      if( x[5] == 1)begin
        $display("OpenOcd Exit");
        $finish();
      end
    endrule

    rule assignment;
      jtag_tap.tms_i(tms.crossed);
      jtag_tap.tdi_i(tdi.crossed);
      jtag_tap.bs_chain_i(0);  
      jtag_tap.debug_tdi_i(0);
    endrule

    rule rl_wr_tdo;
      tdo <= jtag_tap.tdo(); //  Launched by a register clocked by inverted tck
    endrule

    ///////
    
    Hart_Debug_Ifc hart <- mkHartTemplate();
    Ifc_riscvDebug013 device <- mkriscvDebug013();

    // AXI4_Fabric_IFC #(`Num_Masters, `Num_Slaves, PADDR, XLEN, USERSPACE)
    AXI4_Fabric_IFC #(1,2,32,32,0)  fabric <- mkAXI4_Fabric(fn_slave_map);
    Ifc_bram_axi4   #(32,32,0,18)   main_memory0 <- mkbram_axi4('h00000000,"test.mem","test.mem","Test Memory");

    // Connect Jtag and Debugger

    SyncFIFOIfc#(Bit#(41)) sync_request_to_dm <-mkSyncFIFOToCC(1,tck_clk.new_clk,trst.new_rst);
    SyncFIFOIfc#(Bit#(34)) sync_response_from_dm <-mkSyncFIFOFromCC(1,tck_clk.new_clk);
    
    rule connect_tap_request_to_syncfifo;
      let x<-jtag_tap.request_to_dm;
      sync_request_to_dm.enq(zeroExtend(x));
    endrule
    rule read_synced_request_to_dm;
      sync_request_to_dm.deq;
      device.dtm.putCommand.put(sync_request_to_dm.first);
    endrule

    rule connect_debug_response_to_syncfifo;
      let x <- device.dtm.getResponse.get;
      sync_response_from_dm.enq(x);
    endrule
    rule read_synced_response_from_dm;
      sync_response_from_dm.deq;
      jtag_tap.response_from_dm(sync_response_from_dm.first);
    endrule

    mkConnection (device.debug_master,fabric.v_from_masters[0]);
    mkConnection (fabric.v_to_slaves[0],main_memory0.slave);
    mkConnection (hart,device.hart);
    
  endmodule

endpackage