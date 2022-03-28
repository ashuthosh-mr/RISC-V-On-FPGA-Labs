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
Author Names : Vinod.G, Ayush Mittal
Email ID : g.vinod1993@gmail.com, 29ayush@gmail.com

I2C Controller top module which is compliant with UM10204 I2C Specification provided by NXP
Semiconductors. This is a single master, multiple slave controller.
*/
// ================================================
package i2c;
// Bluespec Libraries
import  TriState         ::*;
import  Counter          ::*;
// ================================================
//  Project Imports and Includes
import  AXI4_Lite_Types  ::*;
import  AXI4_Lite_Fabric ::*;
import  AXI4_Types ::*;
import  AXI4_Fabric ::*;
import device_common::*;

import  ConcatReg        ::*;
import  BUtils            ::*;
import  Semi_FIFOF       ::*;
import  Clocks::*;
import  FIFO::*;
`include "i2c.defs"
`include "Logger.bsv"


typedef union tagged {
  void Dead;
  void Idle;
  void STA;
  void NAck_1;
  void RTSTA;
  void SendAddr;
  void Intrpt;
  void SendData;
  void ReadData;
  void ResetI2C;
  void BError;
  void Ack;
  void NAck;
  void SwitchMode;
  void MultipleTrans;
  void End;
  void Idleready;
} MTrans_State deriving (Bits, Eq);

typedef union tagged {
  void Dead;
  void Idle;
  void STA;
  void SendAddr;
  void Intrpt;
  void ReadData;
  void Ack;
  void NAck;
  void End;
} MRecv_State  deriving (Bits, Eq);

typedef enum    {
    Write,
    Read
} Transaction deriving (Bits, Eq);

// ================================================
// I2C Registers Map -- Custom for now. Should Adhere to the Driver

typedef Bit#(8) I2C_RegWidth;

//  ==============================================
//  Function Definitions
function Reg#(t) readOnlyReg(t r);
    return (interface Reg;
        method t _read = r;
        method Action _write(t x) = noAction;
    endinterface);
endfunction

function Reg#(t) writeOnlyReg(Reg#(t) r)
    provisos(
        Literal#(t)
            );
    return (interface Reg;
        method t _read = 0;
        method Action _write(t x);
            r._write(x);
        endmethod
    endinterface);
endfunction
function Reg#(t) conditionalWrite(Reg#(t) r, Bool a);
    return (interface Reg;
        method t _read = r._read;
        method Action _write(t x);
                if(a)
                r._write(x);
        endmethod
    endinterface);
endfunction
// ================================================
// Interface Declarations
   (*always_enabled, always_ready*)
   interface I2C_out;
       method Bit#(1) scl_out;
       method Action scl_in(Bit#(1) in);
       method Bool scl_out_en;
       method Bit#(1) sda_out;
       method Action sda_in(Bit#(1) in);
       method Bool sda_out_en;
    endinterface


    interface Ifc_i2c_user#(numeric type addr_width, numeric type data_width, numeric type user_width);
		  method ActionValue#(Tuple2#(Bit#(data_width),Bool)) read_req (Bit#(addr_width) addr,
																									AccessSize size);
		  method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data,
																									AccessSize size);
      interface I2C_out io;
      (* always_enabled, always_ready *)
      method Bit#(1) isint();
      method Bit#(1) timerint();
      method Bit#(1) isber();
      method Action resetc (Bit#(1) rst);
    endinterface

    //  ==============================================
//  Module Definition
    (* doc = "This Module implements the I2C Master/Slave Controller based on NXP PCF8584" *)
    module mki2c_user(Ifc_i2c_user#(addr_width, data_width, user_width))        //For the sake of ease of compiling - having a non-polymorphic type TODO Change
        provisos(
                 Add#(a__, 8, addr_width),
                 Add#(b__, 32, data_width),
                 Mul#(16, c__, data_width),
                 Mul#(8, d__, data_width),
                 Mul#(32, e__, data_width)
                );
//////////////////////////////////////////////////////////////////////////////
///////////Register Declarations/////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
      String i2c = "";

        //Timing Registers
      Reg#(Bit#(1))               val_SCL        <-  mkReg(1);                    // SCL value that is sent through the inout pin using tristate
      Reg#(Bit#(1))               val_SCL_in     <-  mkReg(1);
      Reg#(Bit#(1))               val_SDA        <-  mkReg(1);                    // SDA value that is sent through the inout pin using tristate
      Reg#(Bit#(1))               val_SDA_in     <- mkReg(1);
      Reg#(Bool)                  dOutEn         <-  mkReg(False);                 // Data out Enable for the SDA Tristate Buffer
      Reg#(Bool)                  cOutEn         <-  mkReg(False);                 // Data out Enable for the SCL Tristate Buffer

      Reg#(Bit#(8))              cprescaler     <-  mkReg(0);                    // Prescaler Counter for the Chip clock
      Reg#(Bit#(8))              rprescaler     <-  mkReg(0);                    // Prescaler Counter Restorer for the chip clock
      Reg#(Bit#(32))              coSCL          <-  mkReg(0);                    // SCL Counter for the SCL clock
      Reg#(Bit#(32))              reSCL          <-  mkReg(0);                    // SCL Counter Restorer for the SCL clock
      Reg#(Bit#(10))              cycwaste       <-  mkReg(0);                    // Waste Cycle count
      Reg#(Bit#(32))              c_scl       <-  mkReg(0);                    // Waste Cycle count

      //Programmable Registers (Will be used by the Device Drivers to Initialize - Based On PCF8584)
      Reg#(I2C_RegWidth)          s01            <-  mkReg(0);                    //I2C Own Address Slave Register
      Reg#(I2C_RegWidth)          s2             <-  mkReg('b11000000);           //Clock Register - Use to set the module and SCL clock
      Reg#(I2C_RegWidth)         drv0_rg     <-  mkReg(1);
      Reg#(I2C_RegWidth)         drv1_rg     <-  mkReg(1);
      Reg#(I2C_RegWidth)         drv2_rg     <-  mkReg(0);
      Reg#(I2C_RegWidth)         pd_rg       <-  mkReg(0);
      Reg#(I2C_RegWidth)         ppen_rg     <-  mkReg(0);
      Reg#(I2C_RegWidth)         prg_slew_rg <-  mkReg(1);
      Reg#(I2C_RegWidth)         puq_rg      <-  mkReg(0);
      Reg#(I2C_RegWidth)         pwrupzhl_rg <-  mkReg(0);
      Reg#(I2C_RegWidth)         pwrup_pull_en_rg     <-  mkReg(0);
      // Clock Reg Syntax difference from PCF5854 (maybe internally it does so  but who knows)
      // MSB = 1 By default
      // Intialization Of clock register will be accepted only if MSB = 0 ;
      // Once the initial initialization has been done and I2C Controller has started we dont care about that bit
      // Maybe the ultimate reset can be synced using it /~/Discuss

      Reg#(I2C_RegWidth)          s3             <-  mkReg(9);                    //Interrupt Vector Register

      //~/Discuss : If there are only 4-5 interupts why use a 8 bit register And decide upon final values for interupts

      Reg#(I2C_RegWidth)          s0             <-  mkRegU();                    //Data Shift Register
      //~ Maybe better to have preset Values rather than random


      // Control Registers
      Reg#(Bit#(1))               pin            <-  mkReg(1);             // Used as a software reset. If pin is 1 all status bits are reset.Used as transmission complete status in polled applications
      Reg#(Bit#(1))               eso            <-  mkReg(0);             // Enable Serial Output. ESO = 0 - Registers can be initialized. ESO = 1 - I2C Serial Transmission
      Reg#(Bit#(1))               es1            <-  mkReg(0);             // Selection of registers. Not used currently
      Reg#(Bit#(1))               es2            <-  mkReg(0);             // Selection of registers. Not used currently.
      Reg#(Bit#(1))               eni            <-  mkReg(0);             // Enables the external interrupt output, which is generated when the PIN is active (active low - 0)
      Reg#(Bit#(1))               sta            <-  mkReg(0);             // STA and STO generates the START and STOP bit for the processor
      Reg#(Bit#(1))               sto            <-  mkReg(0);
      Reg#(Bit#(1))               ack            <-  mkReg(1);             // This is normally set to 1. I2C automatically sends an acknowledge after a read/write transaction

      //  Status Registers - Many of the status bits are unused for now since only single master support is present
      Reg#(Bit#(1))               configchange           <-  mkReg(0);             // Should be set to 0 by the driver. No purpose -- Maybe can be checked if driver is initializing properly

      Reg#(Bit#(1))               zero           <-  mkReg(1);             // Should be set to 0 by the driver. No purpose -- Maybe can be checked if driver is initializing properly
      Reg#(Bit#(1))               sts            <-  mkReg(0);             // Used only Slave receiver mode to detect STOP. Not used
      Reg#(Bit#(1))               ber            <-  mkReg(0);             // Bus Error - Set to 1 when there is a misplaced START, STOP bit
      Reg#(Bit#(1))               ad0_lrb        <-  mkReg(0);             // LRB - holds the last received bit through I2C bus. AD0 - Generall Call bit used for broadcast. Valid only while PIN=0
      Reg#(Bit#(1))               aas            <-  mkReg(0);             // Addressed as slave - Used in Slave Receiver mode
      Reg#(Bit#(1))               lab            <-  mkReg(0);             // Lost Arbitration bit - Used in Multiple Master systems only to denote that the master lost the arbitration
      Reg#(Bit#(1))               bb             <-  mkReg(1);             // ~Bus Busy bit - Indicates that the bus is busy(0 = busy). Also used in multi master systems only // 0 = busy
      Reg#(Bit#(16))              i2ctime        <-  mkReg(0);  //~/Discuss :- Should We have it configurable PCA9654 does

      ////########## COnfigurable Registers Over.....
	    /// Unused Pins : zero
	    ///             : s3 4 MSB s2 5,6 bit
	    ////########################################

      //Custom added MM registers be read or written by the processor master in buffered mode. Support not added yet
      Reg#(Bit#(14))              i2ctimeout     <-  mkReg(1);  //~/Discuss :- Should We have it configurable PCA9654 does

      // Concatenated CSRs TODO
      Reg#(I2C_RegWidth)          mcontrolReg    =   concatReg8(pin,writeOnlyReg(eso),
                                                     writeOnlyReg(es1),writeOnlyReg(es2),
                                                     writeOnlyReg(eni),writeOnlyReg(sta),
                                                     writeOnlyReg(sto),writeOnlyReg(ack));

      Reg#(I2C_RegWidth)          mstatusReg     =   concatReg8(pin,readOnlyReg(zero),
                                                      readOnlyReg(sts),readOnlyReg(ber),
                                                      readOnlyReg(ad0_lrb),readOnlyReg(aas),
                                                      readOnlyReg(lab),readOnlyReg(bb));
      //FSM Registers
      Reg#(MTrans_State)          mTransFSM      <-  mkReg(Idle);
      Reg#(Bool)                  mod_start      <-  mkReg(False);  //Redundant actually
      Reg#(Bool)                  scl_start      <-  mkReg(False);
      Reg#(Bool)                  st_toggle      <-  mkReg(False);
      PulseWire                   pwI2C          <-  mkPulseWire;
      PulseWire                   pwSCL          <-  mkPulseWire;

      //Maintenance variables
      Reg#(Bit#(4))               dataBit        <-  mkReg(9);
      Reg#(Bool)                  last_byte_read <-  mkReg(False);
      Reg#(I2C_RegWidth)          controlReg     =   concatReg8(pin,eso,es1,es2,eni,sta,sto,ack);
      Reg#(Bit#(7))               statusReg      =   concatReg7(zero,sts,ber,ad0_lrb,aas,lab,bb);
      Reg#(Transaction)           operation      <-  mkReg(Write);
      Bool                        pwesoCond      = (pwI2C && eso == 1'b1);
      //pwI2C not required if the clock itself is going to be stepped down
      Bool                        pwsclCond      = (pwSCL && eso == 1'b1);
      Bool                        sclSync        = (pwsclCond && val_SCL == 1);  // Sends a tick on falling edge
      Bool                        sclnSync       = (pwsclCond && val_SCL == 0); // Sends a tick on risisng edge
      Bool                        startBit       = mTransFSM == STA;
      Bool                        stopBit        = mTransFSM == End;
      Bool                        sendAddr       = mTransFSM == SendAddr;
      Bool                        ackCond        = mTransFSM == Ack;
      Bool                        intCond        = mTransFSM == Intrpt;
      Bit#(3)                     startSig       = 3'b110; //To prevent quick transitions which might result in a spike
      Bit#(3)                     stopSig        = 3'b001;
      Reg#(Bit#(2))               sendInd        <- mkReg(2);


      //Intrupt Register
      Reg#(Bit#(1)) rstsig <- mkReg(0); //~/ Use wire Creg is costly
      Reg#(Bit#(6)) resetcount <- mkReg(0); // To count no. of cycles reset pin has been high

      //Module Instantiations
      //AXI4_Lite_Slave_Xactor_IFC #(`PADDR, `Reg_width, `USERSPACE)  s_xactor <- mkAXI4_Lite_Slave_Xactor;


      //Function to access Registers
      function Tuple2#(Bool,Bit#(data_width)) get_i2c(Bit#(8) i2c_state);
        case (i2c_state)
          `Control         : return tuple2(False,duplicate(mcontrolReg));   //~ Are we creating new reg each time
          `Status          : return tuple2(False,duplicate(mstatusReg));
          `S01             : return tuple2(False,duplicate(s01));
          `S0              : return tuple2(False,duplicate(s0));
          `S2              : return tuple2(False,duplicate(s2));
          `S3              : return tuple2(False,duplicate(s3));
          `Time            : return tuple2(False,duplicate(i2ctime));
          `SCL             : return tuple2(False,duplicate(c_scl));
          default : return tuple2(True,0);
        endcase
      endfunction

      //Function to write into the registers
      function ActionValue#(Bool) set_i2c(Bit#(8) i2c_state,Bit#(32) value )= actionvalue
          Bool err=False;
          case(i2c_state)
            `Control : begin
              zero <= 0;  //Indicates to the Driver that the control register has been written in the beginning
              `logLevel( i2c, 2, $format("Control Written"))
              if(!intCond && mTransFSM != NAck)
              	configchange <=1;
              if(value[2:1] == 2 && (intCond || mTransFSM == NAck) && bb == 0 ) begin  //RT START  CONDITIONS
                dataBit <= 9;
		            st_toggle <= True;
            	  mTransFSM <= RTSTA;
            	  dOutEn<= True;
            	  cOutEn<=True;
            	  `logLevel( i2c, 2, $format("Repeated Start Instruction received"))
            	  controlReg <= 8'hc5 | truncate(value);       //TODO 45h Check this out
            	  val_SDA <= 1;
            	  sendInd <= 2;
              end
              else if(value[2:1] == 2 && bb == 0) begin
              	`logLevel( i2c, 2, $format("Invalid Rt Start"))
              	mTransFSM <= End;
              	dOutEn <= True;
              	cOutEn <= True;
              	sendInd <= 2;
              	ber <=1; //~ add bus error functionality
              end
              else
                mcontrolReg._write(truncate(value));
            end
            `S01: begin s01._write(truncate(value)); `logLevel( i2c, 2, $format("S01 written")) end
            `S0 : begin s0._write(truncate(value)); `logLevel( i2c, 2, $format("S0 written")) pin <=1; end
            `S2 : begin
              if(eso == 0) begin
                mod_start <= True;
                scl_start <= True;
                rprescaler <= 0;
                mTransFSM <= Idle;
                s2._write(truncate(value));
              end
              `logLevel( i2c, 2, $format("S2 written"))
            end
            `S3 :  s3._write(truncate(value));  //~ default
            `SCL: begin
				      if(eso == 0) begin
                mod_start <= True;
                scl_start <= True;
				        reSCL<=0;
				        c_scl._write(truncate(value));
                mTransFSM <= Idle;
				      end
				      `logLevel( i2c, 2, $format("Received scl but eso was %d",eso))
			      end
            `Time    : i2ctime._write(truncate(value));
            default : err=True;
          endcase
          return err;
        endactionvalue;


      (* doc = "This sets the module's operating frequency based on the values in s2 register",
         doc = "Assuming Processor operates at 50MHz for now",
         doc = "PreScaler is an approximation of the available values" *)

      (* descending_urgency = "wait_interrupt,toggle_scl" *)
      (* descending_urgency = "send_stop_condition, toggle_scl" *)
      (* descending_urgency = "reset_state,resetfilter" *)
      (* descending_urgency = "reset_state, check_Ack" *)
      (* mutually_exclusive = "send_data,check_Ack,send_addr,receive_data " *)
      (* mutually_exclusive = "set_scl_clock,count_scl,restore_scl" *)

      rule set_i2c_clock (mod_start && eso==1'b0); //Sync problems might be there - check  //~ eso is mkregU + shouldnt it be 0
        `logLevel( i2c, 2, $format("I2C is Setting"))
        mod_start <= False;
        cprescaler <= s2;
        rprescaler <= s2;
        // TODO Currently I2C clock can be set only once when starting -- should see if it should
        //    dynamically changed
        //~ It Can be changed whenever wished but changes reflect only if bus is restarted
        // i.e not busy; no man ref
      endrule

      (* doc = "This rule is used to select one of the SCL clock frequencies among the ones based on the s2 register encoding" *)
      rule set_scl_clock(scl_start && eso == 1'b0 ); //~ eso should be 0 + regU not
        `logLevel( i2c, 2, $format("SCL is Setting"))
        scl_start <= False;
        coSCL <= c_scl;
        reSCL <= c_scl;
        // TODO Currently SCL Clock can be set only once when starting -- should see if it should be
        //    dynamically changed
        // TODO Currently 50MHz processor running freq. and ~8MHz Module operating freq. is assumed.
        //    Should generalize
        // TODO -8
      endrule

      (* doc = "Prescaler register counter which will trigger the rules making it operate in the stated freq. approx." *)
      rule count_prescale(cprescaler > 0);
          cprescaler <= cprescaler - 1;
      endrule


      (* doc = "A tick is sent which will fire the rules making it emulate a module clock, when counter becomes zero" *)
      rule restore_prescale(cprescaler == 0 && rprescaler > 0);
          cprescaler <= rprescaler;
          pwI2C.send;
          //        $display("tick i2c sent");
        //  $display("cprescaler : %d rprescaler : %d",cprescaler,rprescaler,$time);
      endrule


      (* doc = "This rule is used to count down SCL clock which is a scaled down version of the Module clock derived from the system clock. SCL toggles only during transmission. For Multiple masters, different scenarios might arise" *)
      rule count_scl(coSCL > 0 && pwesoCond && st_toggle);      //~ st_toggle dec false
        coSCL <= coSCL - 1;
      endrule

      (* doc = "This rule is used to restore the SCL count value. Sending a tick, operating as scl clock" *)
      rule restore_scl(coSCL == 0  && pwI2C  && reSCL > 0);
          coSCL <= reSCL;
          pwSCL.send;
      endrule

      (* doc = "This rule is used to toggle the Serial Clock line based on coSCL input" *)
      rule toggle_scl(st_toggle && pwSCL); //~ st_toggle is false
        val_SCL <= ~val_SCL;
      endrule

      //////////////State Machine Implementation ////////////////////////////////
      /*
       Initially, Module Is Off Eso = 0.
       Only Check Config Register And write into register rules should fire.

      ESO = 0 => Go to reset state.

      Interrupt Generation And Documentation
          Interrupt Vector = S3 Default Value: 00000000
          01.) 00H = Clock register Yet Not Initialized
          02.) 01H = Address has been sent waiting for slave to acknowledge // Not generated
          03.) 02H = Slave has acknowledged waitng for Data to be written
          04.) 03H = Slave Sent A Nack in send address phase
          05.) 04H = Slave Sent A Nack in Write Data Phase (data was being written to slave)
          06.) 05H = Master Was unable to send a ack during receive data phase
          07.) 06H = Master Received A data from slave please read
          08.) 07H = I2C Reset By Reset Signal //Not generated
          09.) 08H = Master Received Last Byte From Slave
          10.) 09H = Idle i.e Not Busy // Have to modify in multi master .i2c might be idle yet busy
          11.) 0AH = RT Strart sent // If int then it means enter address
          12.) 0BH = Start Sent
          13.) 0CH = timeout
          14.)
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      */

	    rule idler(mTransFSM == Idle);
		    if(cycwaste == 0) begin
	        cycwaste <= cycwaste + 1;
	      end
		    else if(cycwaste == 'd600) begin
    			mTransFSM <= Idleready;
		    	s3 <= 'h09;
    			cycwaste <= 0;
		      sendInd <= 2;
     	    i2ctimeout <= 1;
     	    // Should it leave control of SCL And SDA Line
		 	    statusReg  <= 1;
     	    resetcount <= 0 ;
		    end
		    else
			    cycwaste <= cycwaste + 1;
	    endrule
      (* mutually_exclusive = "check_control_reg, send_start_trans" *)
      (* doc = "Checks the control Reg (after Driver updates it) and based on byte loaded goes to Write/Read" *)
      rule check_control_reg(configchange == 1 && pwesoCond && !intCond && mTransFSM != Idle
                                                    && mTransFSM !=NAck); //~ mod edge and serial on
        `logLevel( i2c, 2, $format("Configchanged fire"))
        configchange <= 0 ;  // Discuss For More than 1 enques
        if(controlReg[2:1] == 2  && bb==0) begin  // Start
          `logLevel( i2c, 2, $format("Invalid Start"))
        end
        else if(controlReg[2:1] == 2) begin
           mTransFSM <= STA;
           st_toggle <= True;
           val_SDA <= 1;
           `logLevel( i2c, 2, $format("Start Received"))
           dOutEn <= True;
           cOutEn <= True;
        end
        else if(controlReg[2:1] == 1 ) begin // Stop
          `logLevel( i2c, 2, $format("Invalid Stop",mTransFSM))
    			mTransFSM <= End;
    			dOutEn <= True;
    			cOutEn <= True;
    			sendInd <= 2;
    			ber <=1; //~ add bus error functionality
        end
      endrule

      (*  doc = "Reset the I2C State Machine and wait for another transaction request" *)
      rule reset_state(mTransFSM == ResetI2C && pwI2C && ber == 0);
        st_toggle <= False;
        dOutEn <= False;
        cOutEn <= False;
        i2ctimeout <= 1;
        // Should it leave control of SCL And SDA Line
        controlReg <= 128;
        statusReg  <= 1;
        resetcount <= 0 ;
        sendInd <= 2;
        mTransFSM  <= Idle;
          //~/ should we have a reset interupt
      endrule

      (* doc = "Send Start bit to the Slave" *)
      //~ pwi2c & i2c on & both transrec - sta
      rule send_start_trans(val_SCL_in == 1 && startBit && pwesoCond && bb == 1);
        `logLevel( i2c, 2, $format("Came Here",sendInd))
        if(val_SDA == 0) begin
          mTransFSM <= SendAddr;  //~Once Cycle Delay whats 0-1 ?
          `logLevel( i2c, 2, $format("start sent"))
          bb  <= 0;
          sta <=0 ;
          s3 <=  'h0B;
          dataBit <= 9;
        end
        else begin
          val_SDA <= startSig[sendInd];
          sendInd <= sendInd-1;
       	end         //TODO check what happens when multiple start has to be send!!!!
      endrule
      //~ pwi2c & i2c on & both transrec - sta
      rule send_rtstart_trans(val_SCL_in == 1 && mTransFSM == RTSTA  && pwesoCond );
        `logLevel( i2c, 2, $format("Here"))
        if(val_SDA == 0)begin //If I read val_SDA - Next transaction is x or it is normal--- Why?  //~ it was because of code bug send ind going to -1
          mTransFSM <= Intrpt;  //~Once Cycle Delay whats 0-1 ?
          `logLevel( i2c, 2, $format("RT start sent"))
          pin<=0;
          i2ctimeout <=1;
          bb  <= 0;
          sta <=0 ;
          s3 <= 'h0A;
        end
        else begin
          val_SDA <= startSig[sendInd];
          sendInd <= sendInd-1;
        end         //TODO check what happens when multiple start has to be send!!!!
      endrule

    (* doc = "Send Slave Address for Transaction" *)
    //      rule send_addr(sendAddr && sclSync); // Fires On Falling Edge
    rule send_addr(sendAddr); // Fires On Falling Edge
      sendInd <= 2;
      if (dataBit > 1 && sclSync)
        begin
          dataBit <= dataBit - 1;
          val_SDA <= s0[dataBit - 2];
          `logLevel( i2c, 2, $format("Sending Bit %d In neg cycle Bit %d ", dataBit - 1,s0[dataBit-1]))
        end
        else if(dataBit == 1 && sclSync)
        begin
        dOutEn <= False;
        mTransFSM <= Ack;
        s3 <= 8'b1;
        if(s0[0] == 0)
          begin
            operation <= Write;
            dataBit <= 8;
          end
        else
          begin
            dataBit <= 8;
            operation <= Read;
          end
        `logLevel( i2c, 2, $format("Address Sent"))
        end
    endrule

    rule check_Ack(ackCond && pwsclCond);  //Should Fire When SCL is high  //~shouldn't it be pwsclcond or sclsync
       //`logLevel( i2c, 2, $format("Value : %d ,Condition : ",line_SDA,line_SDA!=0 ))
       dataBit <= 9;
       i2ctimeout <= 1;
       if(val_SDA_in != 0 && val_SCL == 1 ) begin //Line SCL is actually high
         ad0_lrb <= 1;
         mTransFSM <= NAck;
         dOutEn <= True;
         ber <= 1;
         if(sendAddr)
             s3 <= 'h03;
         else if(mTransFSM == ReadData)
             s3 <= 'h04;
         else
             s3 <= 'h05;
         `logLevel( i2c, 2, $format("Acknowledgement Not Received Bus Error"))
       end
       else if(val_SDA_in == 0 && val_SCL == 1) begin // Acknowledgement is done // Here line scl is actually low
         pin <= 0;
        `logLevel( i2c, 2, $format("eni : %d",eni))
         if(mTransFSM == SendAddr || mTransFSM == SendData )
           s3 <= 'h02;
         else
           s3 <= 'h06;
         ad0_lrb <= 0;
         dOutEn<= True;
         mTransFSM <= Intrpt;
         `logLevel( i2c, 2, $format("Acknowledgement Received. Waiting For Interupt Serve "))
       end
     endrule

(* doc = "Wait for the generated interrupt to be serviced" *)
 rule wait_interrupt(intCond && pwesoCond && val_SCL_in ==0 );
   if(pin == 1'b0) begin
     val_SCL <= 0;
     if(i2ctime[14] == 1)
       i2ctimeout <= i2ctimeout + 1; //~ Have to add timer interupt also reset timer count

     if(i2ctimeout <= i2ctime[13:0] )        //14 as enable int 15 as int
     begin    //Reset state
       mTransFSM <= End; i2ctime[15] <=1;
       s3 <= 'h0C;
     end
     else
       st_toggle <= False;
   end
   else begin
     `logLevel( i2c, 2, $format("Interupt Is Served. Along with pin & control reg - %d & operation - %d ",
                                                                         controlReg,operation))
     i2ctimeout <= 1;
     st_toggle <= True;
     if(controlReg[2:1] == 2) begin
       `logLevel( i2c, 2, $format("Invalid Start"))
       ber <=1 ;
       mTransFSM <= ResetI2C;
     end
     else if(controlReg[2:1] == 1) begin       //Signalling the end of transaction
       mTransFSM <= End;
       val_SDA <= 0;
       `logLevel( i2c, 2, $format("Received Stop Condition ",val_SDA))
     end
     else if(s3 == 'h0A) begin
       mTransFSM <= SendAddr;
       dOutEn <= True;
       `logLevel( i2c, 2, $format("Sending RT Address Bit %d In negative cycle Bit %d",dataBit - 1 , s0[dataBit - 1]))
       dataBit <= dataBit - 1;
       val_SDA <= s0[dataBit - 2];
     end
     else if(operation == Write) begin
       mTransFSM <= SendData;
       dOutEn <= True;
       `logLevel( i2c, 2, $format("Sending Bit %d In negative cycle Bit %d",dataBit - 1 , s0[dataBit - 1]))
       dataBit <= dataBit - 1;
       val_SDA <= s0[dataBit - 2];
     end
     else begin
       if(ack == 0)   //~ Value SCL what hc0 doesnt = nack?
           last_byte_read <= True;
       if(val_SCL == 0) begin  //Hopefully this should ward off read sync error that might //~ discuss maybe
         mTransFSM <= ReadData;
         dataBit <= 8;
         dOutEn <= False;  //~ when we set true
       end
     end
   end
 endrule

  (* doc = "Shift the 8-bit data through the SDA line at each low pulse of SCL" *)
  rule send_data(mTransFSM == SendData && sclSync);
    `logLevel( i2c, 2, $format("WData: TriState SDA Value : %b", val_SDA._read))
    if (dataBit > 1 && sclSync)
      begin
        dataBit <= dataBit - 1;
        val_SDA <= s0[dataBit - 2];
        `logLevel( i2c, 2, $format("Sending Bit %d In neg cycle Bit %d ", dataBit - 1,s0[dataBit-1]))
      end
      else if(dataBit == 1 && sclSync)
      begin
      dOutEn <= False;
      mTransFSM <= Ack;
      `logLevel( i2c, 2, $format("Data Sent"))
      `logLevel( i2c, 2, $format("Leaving Bus For Acknowledge"))
      end
  endrule

      (* doc = "Receive the 8-bit data through the SDA line at each high pulse of SCL" *)
      rule receive_data(mTransFSM == ReadData && sclnSync);  //~ Rising Edge
        `logLevel( i2c, 2, $format("Receiving Bit %d In Positive Cycle And Bit %d",dataBit - 1,val_SDA_in))
        dataBit <= dataBit - 1;
        s0[dataBit - 1 ] <= val_SDA_in;
      endrule

      rule resetfilter;
        if(rstsig == 1) begin
          resetcount <= resetcount + 1;
          if(resetcount == 'd59) begin
            ber <= 0;
            mTransFSM <= ResetI2C;
            `logLevel( i2c, 2, $format("Resetting"))
            s3 <= 'h07;
          end
        end
        else
          resetcount <= 0 ;
      endrule

      rule receive_data1((mTransFSM == ReadData || mTransFSM == NAck_1) && sclSync &&
                                              ( dataBit == 0  || dataBit == 8) );  //~ Falling Edge
        if(dataBit == 0) begin
          if(!last_byte_read) begin
            `logLevel( i2c, 2, $format("Going to Ack, Taking controll of the bus btw"))
            mTransFSM <= Ack;
            val_SDA <= 0;
            dOutEn <= True;
          end
          else begin
            `logLevel( i2c, 2, $format("Going to NAck, Taking controll of the bus btw"))
            mTransFSM <= NAck_1;
            val_SDA <= 1;
            dataBit <= 8 ;
            dOutEn <= True;
            pin <=0 ;
            s3 <= 'h08;
            last_byte_read <= False;
          end
        end
        else
      	  mTransFSM <= NAck;
      endrule


      rule wait_interrupt_receive_end ( dataBit == 8 && mTransFSM == NAck && val_SCL_in == 0);
        if(controlReg[2:1] == 1) begin
          mTransFSM <= End;
          val_SDA <= 0;
          st_toggle <= True;
        end
        else
        	st_toggle<=False; //Stop Signal goes from 0 to 1
      endrule


      (* doc = "Send a STOP bit signifying no more transaction from this master" *)
      rule send_stop_condition(stopBit && pwesoCond && val_SCL_in == 1); //~ it might be fal edge
        `logLevel( i2c, 2, $format("Sending Stop SDA Value : %b SCL Value : %b", val_SDA._read,val_SCL._read))
        if(val_SDA == 1) begin
          mTransFSM <= Idle;
          //  statusReg  <= 'b0000001;
          sto <= 0;
          st_toggle <= False;
          //Interrupt needs to be sent and final course of action is determined
        end
        else begin
          val_SDA <= stopSig[sendInd];
          sendInd <= sendInd - 1;
        end
      endrule

        //Rules for Communicating with AXI-4
        //TODO - Code different conditions such as only certain registers can be accessed at certain times
		  method ActionValue#(Tuple2#(Bit#(data_width),Bool)) read_req (Bit#(addr_width) addr,
																									AccessSize size);

        //TODO - What if a read request is issued to the data register
        `logLevel( i2c, 2, $format("AXI Read Request pin %d",pin))
        if(truncate(addr) == `S0) begin
   	     pin <=1;
	       `logLevel( i2c, 2, $format("Setting pin to 1 in read phase"))
        end
   	    else if(truncate(addr) == `Status) begin
   	       `logLevel( i2c, 2, $format("Clearing Status Bits"))
   	       ber <= 0;
   	    end

        let {err,data} =  get_i2c(truncate(addr));
        `logLevel( i2c, 2, $format("Register Read  %h: Value: %h erR:%b",addr,data,err))
        return tuple2(data,err);
      endmethod

		  method ActionValue#(Bool) write_req(Bit#(addr_width) addr, Bit#(data_width) data,
																									AccessSize size);
        `logLevel( i2c, 2, $format("Wr_addr : %h Wr_data: %h", addr, data))
        let err <- set_i2c(truncate(addr),truncate(data));
        `logLevel( i2c, 2, $format("Received Value %h err:%b",data,err))
        if(ber==1)
          return True;
        else
  	      return err;
      endmethod

      interface I2C_out io;
        method Bit#(1) scl_out;
            return val_SCL;
        endmethod
        method Action scl_in(Bit#(1) in);
            val_SCL_in <= in;
        endmethod
        method Bool scl_out_en;
            return (cOutEn && eso == 1'b1);
        endmethod
        method Bit#(1) sda_out;
            return val_SDA;
        endmethod
        method Action sda_in(Bit#(1) in);
            val_SDA_in <= in;
        endmethod
        method Bool sda_out_en;
            return (dOutEn && eso == 1'b1);
        endmethod
      endinterface

      method Bit#(1) isint=~pin & eni;

      method Bit#(1) isber = ber;

      method Bit#(1) timerint();
          return i2ctime[15];
      endmethod

      method Action resetc( Bit#(1) rst );
              rstsig <= rst;
      endmethod
    endmodule

    interface Ifc_i2c_axi4lite#(numeric type addr_width,
                                numeric type data_width,
                                numeric type user_width
                               );
      interface AXI4_Lite_Slave_IFC#(addr_width, data_width, user_width) slave;
      interface I2C_out io;
      method Bit#(1) isint();
      method Bit#(1) timerint();
      method Bit#(1) isber();
    endinterface

    module mki2c_axi4lite#(Clock i2c_clock, Reset i2c_reset)
                                             (Ifc_i2c_axi4lite#(addr_width, data_width, user_width))
       provisos(
                Add#(a__, 8, addr_width),
                Add#(b__, 32, data_width),
                Mul#(16, c__, data_width),
                Mul#(8, d__, data_width),
                Mul#(32, e__, data_width)
               );
		  Clock core_clock<-exposeCurrentClock;
  		Reset core_reset<-exposeCurrentReset;
		  Bool sync_required=(core_clock!=i2c_clock);
      AXI4_Lite_Slave_Xactor_IFC#(addr_width, data_width, user_width) s_xactor <-
                                                                        mkAXI4_Lite_Slave_Xactor();

      Ifc_i2c_user#(addr_width, data_width, user_width) i2c_user <- mki2c_user(
                                                                              clocked_by i2c_clock,
                                                                               reset_by i2c_reset);
      if(!sync_required)begin
        rule read_request;
          let rd_req <- pop_o(s_xactor.o_rd_addr);
          let {rdata,err} <- i2c_user.read_req(rd_req.araddr,?);
          let lv_resp= AXI4_Lite_Rd_Data {rresp:err?AXI4_LITE_SLVERR:AXI4_LITE_OKAY,
        	                                                      rdata: rdata, ruser: ?}; //TODO user?
          s_xactor.i_rd_data.enq(lv_resp);
        endrule

        rule write_request;
          let wr_req <- pop_o(s_xactor.o_wr_addr);
          let wr_data <- pop_o(s_xactor.o_wr_data);
          let err <- i2c_user.write_req(wr_req.awaddr, wr_data.wdata,?);
          let lv_resp = AXI4_Lite_Wr_Resp {bresp: err?AXI4_LITE_SLVERR:AXI4_LITE_OKAY, buser: ?};
          s_xactor.i_wr_resp.enq(lv_resp);
         endrule
      end
      else begin
        SyncFIFOIfc#(AXI4_Lite_Rd_Addr#(addr_width, user_width)) ff_rd_request <-
                                                                      mkSyncFIFOFromCC(3, i2c_clock);
        SyncFIFOIfc#(Tuple2#(AXI4_Lite_Wr_Addr#(addr_width, user_width),
                    AXI4_Lite_Wr_Data#(data_width))) ff_wr_request <- mkSyncFIFOFromCC(3, i2c_clock);

        SyncFIFOIfc#(AXI4_Lite_Rd_Data#(data_width, user_width)) ff_rd_response <-
                                                              mkSyncFIFOToCC(3, i2c_clock, i2c_reset);
        SyncFIFOIfc#(AXI4_Lite_Wr_Resp#(user_width)) ff_wr_response <-
                                                              mkSyncFIFOToCC(3, i2c_clock, i2c_reset);

        rule read_request;
          let rd_req <- pop_o(s_xactor.o_rd_addr);
          ff_rd_request.enq(rd_req);
        endrule

        rule send_rd_to_i2c;
          let rd_req = ff_rd_request.first;
          ff_rd_request.deq;
          let {rdata,err} <- i2c_user.read_req(rd_req.araddr,?);
          let lv_resp= AXI4_Lite_Rd_Data {rresp:err?AXI4_LITE_SLVERR:AXI4_LITE_OKAY,
        	                                                      rdata: rdata, ruser: ?}; //TODO user?
          ff_rd_response.enq(lv_resp);
        endrule

        rule send_read_response;
          ff_rd_response.deq;
          s_xactor.i_rd_data.enq(ff_rd_response.first);
        endrule

        rule write_request;
          let wr_req <- pop_o(s_xactor.o_wr_addr);
          let wr_data <- pop_o(s_xactor.o_wr_data);
          ff_wr_request.enq(tuple2(wr_req, wr_data));
        endrule

        rule send_wr_to_i2c;
          let wr_req = tpl_1(ff_wr_request.first);
          let wr_data = tpl_2(ff_wr_request.first);
          let err <- i2c_user.write_req(wr_req.awaddr, wr_data.wdata,?);
          let lv_resp = AXI4_Lite_Wr_Resp {bresp: err?AXI4_LITE_SLVERR:AXI4_LITE_OKAY, buser: ?};
          ff_wr_response.enq(lv_resp);
        endrule

         rule send_wr_resp;
           ff_wr_response.deq;
           s_xactor.i_wr_resp.enq(ff_wr_response.first);
         endrule
       end
       interface slave = s_xactor.axi_side;
       interface io = i2c_user.io;
       method isint()=i2c_user.isint;
       method timerint()=i2c_user.timerint;
       method isber()=i2c_user.isber;
    endmodule

    interface Ifc_i2c_axi4#(numeric type addr_width,
                            numeric type data_width,
                            numeric type user_width);
      interface AXI4_Slave_IFC#(addr_width, data_width, user_width) slave;
      interface I2C_out io;
      method Bit#(1) isint();
      method Bit#(1) timerint();
      method Bit#(1) isber();
    endinterface

    module mki2c_axi4#(Clock i2c_clock, Reset i2c_reset)(Ifc_i2c_axi4#(addr_width, data_width, user_width))
      provisos(
               Add#(a__, 8, addr_width),
               Add#(b__, 32, data_width),
               Mul#(16, c__, data_width),
               Mul#(8, d__, data_width),
               Mul#(32, e__, data_width)
              );
		  Clock core_clock<-exposeCurrentClock;
  		Reset core_reset<-exposeCurrentReset;
		  Bool sync_required=(core_clock!=i2c_clock);
      AXI4_Slave_Xactor_IFC#(addr_width, data_width, user_width) s_xactor <- mkAXI4_Slave_Xactor();

      Ifc_i2c_user#(addr_width, data_width, user_width) i2c_user <- mki2c_user(
                                                                                 clocked_by i2c_clock,
                                                                                 reset_by i2c_reset);
		  if(!sync_required)begin // If uart is clocked by core-clock.
        rule read_request;
          let rd_req <- pop_o(s_xactor.o_rd_addr);
          let {rdata,err} <- i2c_user.read_req(rd_req.araddr,?);
		  		let lv_resp= AXI4_Rd_Data {rresp:err?AXI4_SLVERR:AXI4_OKAY, rid:rd_req.arid,
		  										rlast:True, rdata: rdata, ruser: ?}; //TODO user?
          s_xactor.i_rd_data.enq(lv_resp);
        endrule

        rule write_request;
          let wr_req <- pop_o(s_xactor.o_wr_addr);
          let wr_data <- pop_o(s_xactor.o_wr_data);
          let err <- i2c_user.write_req(wr_req.awaddr, wr_data.wdata,?);
        	let lv_resp = AXI4_Wr_Resp {bresp: err?AXI4_SLVERR:AXI4_OKAY, buser: ?, bid:wr_data.wid};
          s_xactor.i_wr_resp.enq(lv_resp);
        endrule
      end
      else begin
        SyncFIFOIfc#(AXI4_Rd_Addr#(addr_width, user_width)) ff_rd_request <- mkSyncFIFOFromCC(3,
                                                                                        i2c_clock);
        SyncFIFOIfc#(Tuple2#(AXI4_Wr_Addr#(addr_width, user_width),
                             AXI4_Wr_Data#(data_width))) ff_wr_request <- mkSyncFIFOFromCC(3,
                                                                                      i2c_clock);
        SyncFIFOIfc#(AXI4_Rd_Data#(data_width, user_width)) ff_rd_response <- mkSyncFIFOToCC(3,
                                                                             i2c_clock, i2c_reset);
        SyncFIFOIfc#(AXI4_Wr_Resp#(user_width)) ff_wr_response <- mkSyncFIFOToCC(3, i2c_clock,
                                                                            i2c_reset);

        rule read_request;
          let rd_req <- pop_o(s_xactor.o_rd_addr);
          ff_rd_request.enq(rd_req);
        endrule

        rule send_rd_to_i2c;
          let rd_req = ff_rd_request.first;
          ff_rd_request.deq;
          let {rdata,err} <- i2c_user.read_req(rd_req.araddr,?);
		  		let lv_resp= AXI4_Rd_Data {rresp:err?AXI4_SLVERR:AXI4_OKAY, rid:rd_req.arid,
		  										rlast:True, rdata: rdata, ruser: ?}; //TODO user?
          ff_rd_response.enq(lv_resp);
        endrule

        rule send_read_response;
          ff_rd_response.deq;
          s_xactor.i_rd_data.enq(ff_rd_response.first);
        endrule

        rule write_request;
          let wr_req <- pop_o(s_xactor.o_wr_addr);
          let wr_data <- pop_o(s_xactor.o_wr_data);
          ff_wr_request.enq(tuple2(wr_req, wr_data));
        endrule

        rule send_wr_to_i2c;
          let wr_req = tpl_1(ff_wr_request.first);
          let wr_data = tpl_2(ff_wr_request.first);
          let err <- i2c_user.write_req(wr_req.awaddr, wr_data.wdata,?);
        	let lv_resp = AXI4_Wr_Resp {bresp: err?AXI4_SLVERR:AXI4_OKAY, buser: ?, bid:wr_data.wid};
          ff_wr_response.enq(lv_resp);
        endrule

        rule send_wr_resp;
          ff_wr_response.deq;
          s_xactor.i_wr_resp.enq(ff_wr_response.first);
        endrule
      end
      interface slave = s_xactor.axi_side;
      interface io = i2c_user.io;
      method isint()=i2c_user.isint;
      method timerint()=i2c_user.timerint;
      method isber()=i2c_user.isber;
   endmodule

endpackage
