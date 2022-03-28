package xilinxdtm;
/*====== Package imports ======= */
  import Clocks::*;
  import ConcatReg::*;
  import FIFO::*;
  import FIFOF::*;
  import SpecialFIFOs::*;
  import DReg::*;
/*======= Project imports ===== */
  `include "jtagdefines.bsv"
  `include "Logger.bsv"
/*============================== */

  interface Ifc_xilinxdtm;
    /*======== JTAG input pins ===== */
    (*always_enabled,always_ready*)
    method Action tms_i(Bit#(1) tms);
    (*always_enabled,always_ready*)
    method Action tdi_i(Bit#(1) tdi);
    /*    Shift Register Control     */
    (*always_enabled,always_ready*)
    method Action capture_i(Bit#(1) capture);
    (*always_enabled,always_ready*)
    method Action run_test_i(Bit#(1) run_test);
    (* always_enabled,always_ready*)
    method Action sel_i (Bit#(1) sel);
    (* always_enabled,always_ready*)
    method Action shift_i (Bit#(1) shift);
    (* always_enabled,always_ready*)
    method Action update_i (Bit#(1) update);
    /*======= JTAG Output Pins ====== */
    (*always_enabled,always_ready*)
    method Bit#(1) tdo;

    /*======== DMI Interface ============= */
    method Action response_from_dm(Bit#(34) responsedm);
    method ActionValue#(Bit#(40)) request_to_dm;
  endinterface
  
  function Reg#(t) readOnlyReg(t r);
    return (interface Reg;
      method t _read = r;
      method Action _write(t x) = noAction;
    endinterface);
  endfunction
	
  (*synthesize*)
  module mkxilinxdtm(Ifc_xilinxdtm);
    Clock def_clk<-exposeCurrentClock;
    Clock invert_clock<-invertCurrentClock;
    Reset invert_reset<-mkAsyncResetFromCR(0,invert_clock);
    /*========= FIFOs to communicate with the DM==== */
    FIFOF#(Bit#(40)) request_to_DM <-mkUGFIFOF1();
    FIFOF#(Bit#(34)) response_from_DM <-mkUGFIFOF1();
    /*=== Wires to capture the input pins === */
    Wire#(Bit#(1)) wr_tms<-mkDWire(0);
    Wire#(Bit#(1)) wr_tdi<-mkDWire(0);
    Wire#(Bit#(1)) wr_capture<-mkDWire(0);
    Wire#(Bit#(1)) wr_run_test<-mkDWire(0);
    Wire#(Bit#(1)) wr_sel<-mkDWire(0);
    Wire#(Bit#(1)) wr_shift<-mkDWire(0);
    Wire#(Bit#(1)) wr_update<-mkDWire(0);
  /*========= Main Data Register =========== */
    Reg#(Bit# (139)) srg_mdr <- mkReg(0);

    Wire#(Bool) wr_dmi_hardreset <- mkDWire(False);
    Wire#(Bool) wr_dmi_reset <- mkDWire(False);

    Reg#(Bit#(3))	idle=readOnlyReg(3'd7);
    Reg#(Bit#(2))	dmistat<-mkReg(0);
    Reg#(Bit#(6))	abits =readOnlyReg(6'd6);
    Reg#(Bit#(4))	version = readOnlyReg('d1);
    
    Reg#(Bit#(2))	response_status<-mkReg(0);
    Reg#(Bool)		capture_repsonse_from_dm<-mkReg(False);
    Reg#(Bit#(1)) rg_tdo<-mkReg(0, clocked_by invert_clock, reset_by invert_reset);


    ReadOnly#(Bit#(139))	crossed_srg_mdr <- mkNullCrossingWire(invert_clock,srg_mdr);
    ReadOnly#(Bit#(1)) crossed_output_tdo <- mkNullCrossingWire(def_clk,rg_tdo);
    
    Reg#(Bit#(5)) rg_pseudo_ir <- mkReg(0);

    /*======= perform dtmcontrol shifts ======== */
    rule generate_tdo_outputpin;
      rg_tdo <= crossed_srg_mdr[0];
    endrule
    //-------------------------
    rule shift_mdr((wr_sel == 1'b1) && (wr_shift == 1'b1));
      srg_mdr<={wr_tdi,srg_mdr[138:1]};
    endrule

    Reg#(Bit#(41)) rg_packet <- mkReg(0);

    rule tunneled_update ((wr_sel == 1'b1) && (wr_update ==1'b1) && (wr_capture ==1'b0) &&
                          (wr_shift == 1'b0) );
      Bit#(139) mdr_data_r = srg_mdr;
      Bit#(1) idr = mdr_data_r[138];
      Bit#(7) message_len = mdr_data_r[137:131];
      Bit#(128) scan_input = mdr_data_r[130:3];
      // Skipping message length and trying to only capture the IR :?
      let packet = scan_input[127:87];

      if(idr == 1'b0)begin
        rg_pseudo_ir <= scan_input[127:123];
      end
      else begin
        if (rg_pseudo_ir == 5'h10)begin
          // ONLY DMI HARD RESET AND RESET BITS ARE WRITABLE
          wr_dmi_hardreset <= (scan_input[113] == 1'b1); // W1 behavior
          wr_dmi_reset <= (scan_input[112] == 1'b1); // W1 behavior // W1 is asserted
        end
        else if  (rg_pseudo_ir == 5'h11)begin
          if(request_to_DM.notFull && capture_repsonse_from_dm==False)begin
            request_to_DM.enq(packet[39:0]);
            capture_repsonse_from_dm<=True;
          end
          rg_packet <= packet[40:0];
          dmistat <= 2'b11;
        end
      end
      srg_mdr <= 0;
    endrule

    rule tunneled_capture((wr_sel == 1'b1) && (wr_capture == 1'b1) && (wr_update ==1'b0) && 
                          (wr_shift == 1'b0) );
      Bit#(139) capture_frame = 139'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF;
      if (rg_pseudo_ir == 5'h10) begin
        capture_frame[138:40] = 0;
        capture_frame[39:0] = { 19'd0, pack(wr_dmi_hardreset),pack(wr_dmi_reset),
                                1'd0,idle,dmistat,abits,version,3'b000};
      end
      else if  (rg_pseudo_ir == 5'h11)begin
        //capture_frame[138:37] = 0;
        if(response_from_DM.notEmpty)begin 
          let x=response_from_DM.first[33:0];
          x[1:0]=x[1:0]|response_status;// keeping the lower 2 bits sticky
          capture_frame[36:0] = {x,3'b000};  
          response_from_DM.deq; 
          capture_repsonse_from_dm<=False;
          dmistat<=x[1:0];
        end
        else if(capture_repsonse_from_dm) begin
          response_status<=3;
          dmistat<=2'b11;
          capture_frame[43:0] = {rg_packet[40:2],2'b11,3'b000};
        end
        else begin
          capture_frame[43:0] = {rg_packet[40:2],2'b00,3'b000};
        end
      end 
      else begin
        capture_frame[43:0] = {rg_packet[40:2],2'b00,3'b000};
      end
      srg_mdr <= capture_frame;
    endrule
    //-------------------------
    rule dmihardreset_generated(wr_dmi_hardreset);
      request_to_DM.deq;
      response_from_DM.deq;
      capture_repsonse_from_dm<=False;
    endrule

    rule dmireset_generated(wr_dmi_reset);
      response_status<=0;
      capture_repsonse_from_dm<=False;
    endrule
    //-------------------------
    method Action tms_i(Bit#(1) tms);
      wr_tms <= tms;
    endmethod
    method Action tdi_i(Bit#(1) tdi);
      wr_tdi <= tdi;
    endmethod
    method Action capture_i(Bit#(1) capture);
      wr_capture <= capture;
    endmethod
    method Action run_test_i(Bit#(1) run_test);
      wr_run_test <= run_test;
    endmethod
    method Action sel_i (Bit#(1) sel);
    wr_sel <= sel;
    endmethod
    method Action shift_i (Bit#(1) shift);
      wr_shift <= shift;
    endmethod
    method Action update_i (Bit#(1) update);
      wr_update <= update;
    endmethod
    method Bit#(1) tdo;
      return crossed_output_tdo;
    endmethod
    //-------------------------
    method Action response_from_dm(Bit#(34) responsedm) if(response_from_DM.notFull);
      if(capture_repsonse_from_dm)
        response_from_DM.enq(responsedm);
    endmethod
    method ActionValue#(Bit#(40)) request_to_dm if(request_to_DM.notEmpty);
      request_to_DM.deq;
      return request_to_DM.first;
    endmethod
    //-------------------------
  endmodule
endpackage
