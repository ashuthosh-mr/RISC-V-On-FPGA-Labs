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
package spi;

import ConcatReg ::*;
import Semi_FIFOF        :: *;
import FIFOLevel::*;
import AXI4_Lite_Types   :: *;
import AXI4_Lite_Fabric  :: *;
import FIFOF::*;
import Clocks::*;
import SpecialFIFOs::*;
import ConfigReg ::*;
import FIFO::*;
import BUtils::*;
`include "spi.defs"
`include "Logger.bsv"

typedef struct{
	Bit#(addr_width) addr;
	Bit#(3) burst_size;
	Bit#(data_width) wdata;
} Write_req#(numeric type addr_width, numeric type data_width) deriving (Bits, Eq);

typedef struct{
	Bit#(addr_width) addr;
	Bit#(3)  burst_size;
} Read_req#(numeric type addr_width) deriving (Bits, Eq);

typedef struct{
	Bit#(8)  total_bit_rx;
	Bit#(8)  total_bit_tx;
	bit 	 bidimode;
	bit		 bidioe;
	bit		 crcen;
	bit		 crcnext;
	bit		 crcl;
	bit		 rxonly;
	bit		 ssm;
	bit		 ssi;
	bit		 lsbfirst;
	bit		 spe;
	Bit#(3)  br;
	bit		 mstr;
	bit		 cpol;
	bit		 cpha;
} Cr1_cfg deriving (Bits, Eq);

typedef struct{
	Bit#(15) rsvd;
	bit		 rx_imm_start;
	bit		 rx_start; //user defined
	bit		 ldma_tx;
	bit		 ldma_rx;
	bit		 frxth;
	Bit#(4)	 ds;
	bit		 txeie;
	bit		 rxneie;
	bit		 errie;
	bit 	 frf;
	bit		 nssp;
	bit		 ssoe;
	bit		 txdmaen;
	bit		 rxdmaen;
} Cr2_cfg deriving (Bits, Eq);

typedef struct{
	Bit#(19) rsvd1; 
	Bit#(2)  ftlvl;
	Bit#(2)  frlvl;
	bit 	 fre;
	bit 	 bsy;
	bit		 ovr;
	bit		 modf;
	bit 	 crcerr;
	Bit#(2)  rsvd2;
	bit		 txe;
	bit		 rxne;
} Sr_cfg deriving (Bits, Eq);

typedef enum{
			 IDLE,
			 START_TRANSMIT,
			 DATA_TRANSMIT
		} Transmit_state deriving(Bits, Eq, FShow);
			 		
typedef enum{
			 IDLE,
			 START_RECEIVE,
			 DATA_RECEIVE,
			 RECEIVE_DONE
		} Receive_state deriving(Bits, Eq, FShow);


(*always_ready, always_enabled*)
interface Ifc_spi_io;
	method bit mosi;
	method bit sclk;
  method bit nss;
	method Action miso(bit dat);
endinterface

interface Ifc_spi_app#( numeric type addr_width, 
                        numeric type data_width);
	method Bit#(data_width) data_to_app;
	method Action read_request  (Read_req#(addr_width) rd_req);
	method Action write_request (Write_req#(addr_width, data_width) wr_req);
endinterface

interface Ifc_spi#( numeric type addr_width, 
                    numeric type data_width);
	interface Ifc_spi_io io;
	interface Ifc_spi_app#(addr_width, data_width) app_interface;
endinterface

interface Ifc_spi_controller#(numeric type addr_width,
                              numeric type data_width,
                              numeric type user_width);
	interface Ifc_spi_io io;
  interface AXI4_Lite_Slave_IFC#(addr_width, data_width, user_width) slave;
endinterface
	
(*conflict_free="rl_transmit_data_to_fifo, rl_transmit_start"*)
(*conflict_free="rl_transmit_data_to_fifo, rl_data_transmit"*)
(*conflict_free="rl_transmit_data_to_fifo, rl_receive_start_receive"*)
(*conflict_free="rl_transmit_start, rl_receive_start_receive"*)
(*conflict_free="rl_data_transmit, rl_receive_start_receive"*)
(*conflict_free="rl_transmit_data_to_fifo, rl_data_receive"*)
(*conflict_free="rl_transmit_idle, rl_data_receive"*)
(*conflict_free="rl_transmit_start,rl_data_receive"*)
(*conflict_free="rl_data_transmit, rl_data_receive"*)
(*conflict_free="rl_transmit_data_to_fifo, rl_receive_fifo_to_read_datareg"*)
(*conflict_free="rl_transmit_start, rl_receive_fifo_to_read_datareg"*)
(*conflict_free="rl_data_transmit, rl_receive_fifo_to_read_datareg"*)
(*conflict_free="rl_transmit_idle, rl_receive_fifo_to_read_datareg"*)
(*conflict_free="rl_write_to_cfg, rl_data_receive"*)
(*conflict_free="rl_write_to_cfg, rl_transmit_idle"*)
(*preempts="rl_transmit_idle, rl_chip_select_control"*)
(*preempts="rl_receive_idle, rl_chip_select_control"*)
(*preempts="rl_data_receive, rl_chip_select_control"*)
(*preempts="rl_receive_fifo_to_read_datareg, rl_write_to_cfg"*)
(*preempts="rl_transmit_data_to_fifo, rl_write_to_cfg"*)
(*preempts="rl_abort_tx_rx, rl_transmit_idle"*)
(*preempts="rl_abort_tx_rx, rl_transmit_start"*)
(*preempts="rl_abort_tx_rx, rl_data_transmit"*)
(*preempts="rl_abort_tx_rx, rl_receive_idle"*)
(*preempts="rl_abort_tx_rx, rl_receive_start_receive"*)
(*preempts="rl_abort_tx_rx, rl_data_receive"*)
(*preempts="rl_abort_tx_rx, rl_receive_done"*)

module mkspi(Ifc_spi#(addr_width, data_width))
  provisos(
          Add#(a__, 8, addr_width),
          Add#(b__, 32, data_width),
          Mul#(32, c__, data_width));


  Reg#(Cr1_cfg) rg_spi_cfg_cr1    <- mkReg(unpack(0));
  Reg#(Cr2_cfg) rg_spi_cfg_cr2    <- mkReg(unpack(0));
  Reg#(Sr_cfg)	rg_spi_cfg_sr     <- mkConfigReg(unpack(0));
  Reg#(Bit#(32))		rg_spi_cfg_dr1     <- mkReg(0);
  Reg#(Bit#(32))		rg_spi_cfg_dr2     <- mkReg(0);
  Reg#(Bit#(32))		rg_spi_cfg_dr3     <- mkReg(0);
  Reg#(Bit#(32))		rg_spi_cfg_dr4     <- mkReg(0);
  Reg#(Bit#(32))		rg_spi_cfg_dr5     <- mkReg(0);
  Reg#(Bit#(32))		rg_spi_cfg_crcpr   <- mkReg(0);
  Reg#(Bit#(32))		rg_spi_cfg_rxcrcr  <- mkReg(0);
  Reg#(Bit#(32))		rg_spi_cfg_txcrcr  <- mkReg(0);
  Reg#(Bit#(3)) 		rg_clk_counter	   <- mkReg(0);
  Reg#(bit)			    tx_data_en		   <- mkReg(0);
  
  // MOSI and MISO signals of the spi
  Wire#(bit)			wr_spi_in_io1		<- mkWire();
  Wire#(bit)			wr_spi_in_io2		<- mkWire();
  Reg#(bit)			    wr_spi_out_io1		<- mkReg(0);//TODO making wr_spi_out_io1 as Reg
  Wire#(bit)			wr_spi_out_io2		<- mkWire();
  Wire#(bit)			wr_spi_en_io1		<- mkWire();
  Wire#(bit)			wr_spi_en_io2		<- mkWire();
  //Reg#(bit)			wr_clk				<- mkReg(0);
  
  Reg#(Transmit_state) rg_transmit_state <- mkReg(IDLE);
  Reg#(Receive_state)	 rg_receive_state  <- mkReg(IDLE);
  
  Reg#(Bit#(8))		rg_data_tx		   <- mkReg(0);
  Reg#(Bit#(8))		rg_data_rx		   <- mkReg(0);
  Reg#(Bit#(8))		rg_data_counter	   	   <- mkReg(0);
  Reg#(Bit#(8))		rg_bit_count	   	   <- mkReg(0);
  Reg#(bit)			rg_transfer_done   <- mkReg(0);
  Reg#(bit)			rg_tx_rx_start	   <- mkReg(0);
  
  Reg#(bit)			rg_nss			   <- mkReg(1);
  Reg#(bit)			rg_clk			   <- mkReg(0);
  
  Reg#(Bit#(160))  rg_concat_reg = concatReg5(rg_spi_cfg_dr1, rg_spi_cfg_dr2, rg_spi_cfg_dr3, rg_spi_cfg_dr4, rg_spi_cfg_dr5); 
  Wire#(Bit#(addr_width))     wr_write_addr  <- mkWire();
  Wire#(Bit#(data_width))     wr_write_data  <- mkWire();
  Wire#(Bit#(addr_width))     wr_rd_addr	 <- mkWire();
  Wire#(Bit#(data_width))     wr_rd_data     <- mkWire();
  
  Wire#(bit)	wr_clk_en <- mkWire;
  Wire#(Bool) wr_transfer_en <- mkDWire(False);
  
  //Embedded fifo for receive and transmit
  FIFOF#(Bit#(8))		tx_fifo				  <- mkUGSizedFIFOF(`TXFIFO_DEPTH);
  FIFOF#(Bit#(8))		rx_fifo				  <- mkUGSizedFIFOF(`TXFIFO_DEPTH);
  
  //This function writes to configuration registers
  function Action fn_wr_cfg_reg(Bit#(32) data, Bit#(addr_width) address);
     
     action
     Bit#(8) addr = truncate(address);
     case(addr) 
  
         `CR1   : rg_spi_cfg_cr1	<= unpack(data);
  
         `CR2   : rg_spi_cfg_cr2	<= unpack(data);
         
         `SR    : rg_spi_cfg_sr		<= unpack(data);        

         `DR1   : rg_spi_cfg_dr1	<= data;
		 
	 `DR2   : rg_spi_cfg_dr2	<= data;

	 `DR3	: rg_spi_cfg_dr3	<= data;

	 `DR4	: rg_spi_cfg_dr4	<= data;

	 `DR5	: begin
					rg_spi_cfg_dr5	<= data;
		  			tx_data_en	<= 1;
					rg_data_counter <= 0;
					rg_bit_count	<= 0;
					tx_fifo.clear();
       	  end 
         `CRCPR  : rg_spi_cfg_crcpr   <= data;
        
         `RXCRCR : rg_spi_cfg_rxcrcr  <= data;
        
         `TXCRCR : rg_spi_cfg_txcrcr  <= data;
        
         default          : noAction;
     endcase
    endaction 
  endfunction
  
  // This function returns the configuration register 
  function Bit#(32) fn_rd_cfg_reg(Bit#(addr_width) address);
    Bit#(8) addr = truncate(address);
    case(addr)
  
         `CR1     : return pack(rg_spi_cfg_cr1);
  
         `CR2     : return pack(rg_spi_cfg_cr2);
         
         `SR      : return pack(rg_spi_cfg_sr);
        
         `DR1     : return rg_spi_cfg_dr1;

	 `DR2     : return rg_spi_cfg_dr2;

	 `DR3     : return rg_spi_cfg_dr3;

	 `DR4	  : return rg_spi_cfg_dr4;

	 `DR5	  : return rg_spi_cfg_dr5;
        
         `CRCPR  : return rg_spi_cfg_crcpr;
        
         `RXCRCR : return rg_spi_cfg_rxcrcr;
        
         `TXCRCR : return rg_spi_cfg_txcrcr;
        
     endcase
  endfunction
  
  rule rl_clock_phase_enable;
  	wr_clk_en <= (rg_spi_cfg_cr1.cpha == 1 && rg_spi_cfg_cr1.cpol == 1 && rg_clk == 0) ? 1 :
  				 (rg_spi_cfg_cr1.cpha == 1 && rg_spi_cfg_cr1.cpol == 0 && rg_clk == 1) ? 1 :
  				 (rg_spi_cfg_cr1.cpha == 0 && rg_spi_cfg_cr1.cpol == 1 && rg_clk == 1) ? 1 :
  				 (rg_spi_cfg_cr1.cpha == 0 && rg_spi_cfg_cr1.cpol == 0 && rg_clk == 0) ? 1 : 0;
  endrule
  
  //This rule takes data from the configration register DR and puts it into the
  //tx_fifo in the 8 bit format.
  rule rl_transmit_data_to_fifo(tx_data_en == 1);
  	if(rg_data_counter < 20) begin //  why 4 ? the data is of 32 bit (four 8bit data)
  		Bit#(8) data = 0;
		 if(rg_spi_cfg_cr1.lsbfirst == 1) begin
			data = rg_concat_reg[7 : 0];
  		 	rg_concat_reg <= rg_concat_reg >> 8;
		end
		else begin
			data = rg_concat_reg [159: 152];
  		 	rg_concat_reg <= rg_concat_reg << 8;
	    end	
  		tx_fifo.enq(data);
  		if(rg_data_counter < 19) 
  			rg_data_counter <= rg_data_counter + 1;
  		else begin
  			rg_data_counter <= 0;
  			tx_data_en      <= 0;
  		end
  		`logLevel( spi, 0, $format(" SPI : DR to  tx_fifo data %x", data))
  		 $display($stime," SPI : DR to  tx_fifo data %x rg_data_counter %d", data, rg_data_counter);
  	end
  endrule
  
  // This rule takes care of the bidirectional mode of the controller, full-
  // duplex, simplex and duplex (software programmable)
  rule rl_bidimode_bidioe;
  	if(rg_spi_cfg_cr1.bidimode == 0) begin // full duplex mode
  		wr_spi_en_io1 <= 1;
  		wr_spi_en_io2 <= 0;
  	end
  	else if(rg_spi_cfg_cr1.bidioe == 1) begin // transmit only mode
  		wr_spi_en_io2 <= 1;					  // Master mode so mosi pin is used
  	end
  	else if(rg_spi_cfg_cr1.bidioe == 0) begin //receive only mode
  		wr_spi_en_io2 <= 0;					  // Master mode so mosi pin is used
  	end
  endrule //TODO do we need rxonly mode also.. if bidioe is disabled isn't that enough?
  
  //TODO MSTR reg need to be used for control of the circuit
  
  
  rule rl_write_to_cfg;
  	fn_wr_cfg_reg(truncate(wr_write_data), wr_write_addr);
  	`logLevel( spi, 0, $format(" SPI : Write request wr_addr %x wr_data %x ",wr_write_addr,
                                                                              wr_write_data))
  	$display($stime," SPI : Write request wr_addr %x wr_data %x ",wr_write_addr, wr_write_data);
  endrule
  
  rule rl_read_from_cfg;
  	wr_rd_data <= duplicate(fn_rd_cfg_reg(wr_rd_addr));
  endrule
  
  // This rule takes care of the chip select pin control
  rule rl_chip_select_control(rg_spi_cfg_cr1.ssm == 1);
  		rg_nss <= rg_spi_cfg_cr1.ssi;
  endrule
  
  // This rule generates the clock according to software specified baudrate
  rule rl_generate_clk_baud_rate;
  	if(rg_nss == 1 ) begin // chip select is active low
  		rg_clk <= rg_spi_cfg_cr1.cpol == 0 ? 0 : 1;
  	end
      else begin	
  		if(rg_spi_cfg_cr1.br == rg_clk_counter) begin
  			rg_clk <= ~rg_clk;
  			rg_clk_counter <= 0;
			wr_transfer_en <= True;
  		end
  		else
  			rg_clk_counter <= rg_clk_counter + 1;
  	end
  endrule
  
  //TODO need to add the error flags, rg_clk control in all the rules 
  
  /*************** TRANSMIT STATE *******************/
  // This rule is the deciding point to start the transmit state machine
  // this state machine starts when the tx_fifo is notempty and spi is enabled
  rule rl_transmit_idle(rg_transmit_state == IDLE && tx_data_en == 0 
                                                  && rg_spi_cfg_cr2.rx_start == 0);
  	if(rg_spi_cfg_cr1.spe == 1 && tx_fifo.notEmpty()) begin
  		rg_transmit_state <= START_TRANSMIT;
  		rg_data_tx <= tx_fifo.first();
  		rg_spi_cfg_sr.bsy <= 1;
  		rg_nss <= 0;
        rg_transfer_done <= 0;
		rg_data_counter <= 0;
		if(rg_spi_cfg_cr1.cpha == 0) begin
			if(rg_spi_cfg_cr1.lsbfirst == 1)
				wr_spi_out_io1 <= tx_fifo.first()[0];
			else
				wr_spi_out_io1 <= tx_fifo.first()[7];
		end
//		tx_fifo.deq;
		`logLevel( spi, 0, $format(" SPI : Transmit state has started rg_data_tx %x", 
                                                                                  tx_fifo.first()))
		$display($stime," SPI : Transmit state has started rg_data_tx %x",tx_fifo.first());
  	end
  	else if(wr_clk_en == 0 && wr_transfer_en == True && rg_spi_cfg_cr1.spe == 1 && rg_spi_cfg_cr2.rx_imm_start == 0)begin
  		rg_nss <= 1;
		rg_spi_cfg_cr1.spe <= 0;
  		`logLevel( spi, 0, $format(" SPI : Transmit state is in idle"))
  		$display($stime," SPI : Transmit state is in idle");
  	end
  endrule

// This rule transmits first bit with respect to clock phase configured
rule rl_transmit_start(rg_transmit_state == START_TRANSMIT && rg_nss == 0);
  	if(wr_clk_en == 1 || (wr_clk_en == 0 && rg_spi_cfg_cr1.cpha == 1) && wr_transfer_en == True) begin
  		let data = 0;
  //		wr_clk <= rg_clk;		
  		if(rg_spi_cfg_cr1.lsbfirst == 1) begin
  			data = rg_data_tx[0];
  			wr_spi_out_io1 	<= rg_data_tx[0];
  			rg_data_tx 	<= rg_data_tx >> 1;
  		end
  		else begin
  			data = rg_data_tx[7];
  			wr_spi_out_io1 	<= rg_data_tx[7];
  			rg_data_tx    	<= rg_data_tx << 1;
  		end
		rg_bit_count <= rg_bit_count + 1;
		rg_data_counter <= rg_data_counter + 1;
  	  `logLevel( spi, 0, $format(" SPI : START_TRANSMIT case2 counter %x data %x", rg_data_counter, 
                                                                                  data))
  	  $display($stime," SPI : START_TRANSMIT case2 counter %x data %x rg_bit_count %x", rg_data_counter,data, rg_bit_count);
  		rg_transmit_state <= DATA_TRANSMIT;
  	end	
 endrule
  
  //debug
  rule rl_debug;
  `logLevel( spi, 1, $format(" SPI : wr_spi_out_io1 %x", wr_spi_out_io1))
  endrule
  
  //This rule transmits the remaining data from the tx_fifo until it get empty
  rule rl_data_transmit(rg_transmit_state == DATA_TRANSMIT && rg_nss == 0);
	//$display($stime," Data transmit rule fired : %d ",rg_data_counter);
  	let data_tx = 0; // debug
  //		wr_clk <= rg_clk;
  		if(rg_data_counter < 6 && wr_clk_en == 0 && wr_transfer_en == True) begin
  			if(rg_spi_cfg_cr1.lsbfirst == 1) begin
  				data_tx = rg_data_tx[0];
  				wr_spi_out_io1 <= rg_data_tx[0];
  				rg_data_tx <= rg_data_tx >> 1;
  				rg_data_counter <= rg_data_counter + 1;
				rg_bit_count	<= rg_bit_count + 1;
  			end
  			else begin
  				data_tx = rg_data_tx[7];
  				wr_spi_out_io1 <= rg_data_tx[7];
  				rg_data_tx    <= rg_data_tx << 1;
  				rg_data_counter <= rg_data_counter + 1;
				rg_bit_count	<= rg_bit_count + 1;
  			end
  	    `logLevel( spi, 0, $format(" SPI : DATA_TRANSMIT case 1 data %x rg_data_counter %x",
                                                                         data_tx, rg_data_counter))
  	    $display($stime," SPI : DATA_TRANSMIT case 1 data %x rg_data_counter %x rg_bit_count %d ", 
		data_tx, rg_data_counter, rg_bit_count);
  		end
  		else if(rg_data_counter == 6 && wr_clk_en == 0 && wr_transfer_en == True) begin
  			if(rg_spi_cfg_cr1.lsbfirst == 1) begin
  				data_tx = rg_data_tx[0];
  				wr_spi_out_io1 <= rg_data_tx[0];
  				rg_data_tx <= rg_data_tx >> 1;
  				rg_data_counter <= rg_data_counter + 1;
				rg_bit_count	<= rg_bit_count + 1;
  				tx_fifo.deq();
  			end
  			else begin
  				data_tx = rg_data_tx[7];
  				wr_spi_out_io1 <= rg_data_tx[7];
  				rg_data_tx <= rg_data_tx << 1;
  				rg_data_counter <= rg_data_counter + 1;
				rg_bit_count	<= rg_bit_count + 1;
  				tx_fifo.deq();
  			end
  	    `logLevel( spi, 0, $format(" SPI : DATA_TRANSMIT case 2 data %x rg_data_counter %x ",
                                                                          data_tx, rg_data_counter))
  	    $display($stime," SPI : DATA_TRANSMIT case 2 data %x rg_data_counter %x rg_bit_count %d ",data_tx,
		 rg_data_counter, rg_bit_count);
  		end
  		else if(rg_data_counter == 7 && wr_clk_en == 0 && wr_transfer_en == True && rg_transfer_done == 0) begin
  				if(rg_spi_cfg_cr1.lsbfirst == 1) begin
  					data_tx = rg_data_tx[0];
  					wr_spi_out_io1 <= rg_data_tx[0];
  				end
  				else begin
  					data_tx = rg_data_tx[7];
  					wr_spi_out_io1 <= rg_data_tx[7];
  				end
  				if(tx_fifo.notEmpty()) begin
  					$display($stime(), " SPI : tx_fifo data %x", tx_fifo.first);
  					rg_data_tx <= tx_fifo.first;
					rg_data_counter <= 0;
  				end
  				else begin
					rg_transfer_done <= 1;
				end
				rg_bit_count <= rg_bit_count + 1;
  	      `logLevel( spi, 0, $format(" SPI : DATA_TRANSMIT case 3 data %x rg_data_counter %x ",
                                                                          data_tx, rg_data_counter))
  	      $display($stime," SPI : DATA_TRANSMIT case 3 data %x rg_data_counter %x rg_bit_count %d",
		  data_tx, rg_data_counter, rg_bit_count);
  		end
  	if(!tx_fifo.notEmpty() && wr_clk_en == 0 && wr_transfer_en == True && rg_transfer_done == 1) begin
  		rg_transmit_state <= IDLE;
		if(rg_spi_cfg_cr2.rx_imm_start == 1) begin
			rg_tx_rx_start <= 1;
		end
		else
			rg_nss <= 1;
		
		rg_spi_cfg_sr.bsy <= 0;
  		`logLevel( spi, 0, $format(" SPI : Transmit state going to idle"))
  		$display($stime," SPI : Transmit state going to idle");
  	end	
  endrule
  		
  /************* RECEIVE STATE ************/
  //This rule will decide the start of the receive state machine 
  // TODO define trigger event to start receive state to be defined
  rule rl_receive_idle(rg_receive_state == IDLE && (rg_spi_cfg_cr2.rx_start == 1 || (
rg_spi_cfg_cr2.rx_imm_start == 1 && rg_tx_rx_start == 1)) && rg_transmit_state == IDLE && wr_transfer_en == True); 
  		rg_receive_state <= START_RECEIVE;
		rg_data_counter <= 0;
		rg_bit_count    <= 0;
  		rg_nss <= 0;
  		`logLevel( spi, 0, $format(" SPI : Receive has started"))
  		$display($stime," SPI : Receive has started");
  endrule

  rule rl_receive_idle_stop(rg_receive_state == IDLE && rg_transmit_state == IDLE && 
	(rg_spi_cfg_cr2.rx_start == 0 || rg_spi_cfg_cr2.rx_imm_start == 0) 
	&& wr_clk_en == 0 && wr_transfer_en == True);
	rg_nss <= 1;
  endrule
  
  rule rl_receive_start_receive(rg_receive_state == START_RECEIVE && rg_nss == 0);
	Bit#(8) data_rx = 0;
	 if( wr_clk_en == 1 && wr_transfer_en == True) begin

  		if(rg_spi_cfg_cr1.lsbfirst == 1) begin
  			data_rx = {wr_spi_in_io2, rg_data_rx[6:0]};
  			rg_data_rx <= data_rx >> 1;
  			rg_data_counter <= rg_data_counter + 1;
			rg_bit_count	<= rg_bit_count + 1;
  		end
  		else begin
  			data_rx = {rg_data_rx[7:1], wr_spi_in_io2};
  			rg_data_rx 		<= data_rx << 1;
  			rg_data_counter <= rg_data_counter + 1;
			rg_bit_count	<= rg_bit_count + 1;
  		end
  	  `logLevel( spi, 0, $format(" SPI : START_RECEIVE case2 counter %x", rg_data_counter))
  	  $display($stime," SPI : START_RECEIVE case2 counter %x", rg_data_counter);
  		rg_receive_state <= DATA_RECEIVE;
  	end
  endrule
  
  rule rl_data_receive(rg_receive_state == DATA_RECEIVE && rg_nss == 0);
  	if(rg_data_counter < 8 && rx_fifo.notFull && wr_clk_en == 1 && wr_transfer_en == True) begin
  //		wr_clk <= rg_clk;
  		Bit#(8) data_rx = 0;
  //		if(wr_clk_en == 1) begin
  		if(rg_data_counter < 7) begin
  			if(rg_spi_cfg_cr1.lsbfirst == 1) begin
  				data_rx = {wr_spi_in_io2, rg_data_rx[6:0]};
  				rg_data_rx <= data_rx >> 1;
  				rg_data_counter <= rg_data_counter + 1;
				rg_bit_count	<= rg_bit_count + 1;
  			end
  			else begin
  				data_rx = {rg_data_rx[7:1], wr_spi_in_io2};
  				rg_data_rx 		<= data_rx << 1;
  				rg_data_counter <= rg_data_counter + 1;
				rg_bit_count	<= rg_bit_count + 1;
  			end
  	    `logLevel( spi, 0, $format(" SPI : DATA_RECEIVE case1 counter %x data_rx %x", 
                                                                          rg_data_counter, data_rx))
  	    $display($stime," SPI : DATA_RECEIVE case1 counter %x data_rx %x rg_bit_count %d",rg_data_counter, data_rx,
		 rg_bit_count);
  		end
  		else if(rg_data_counter == 7) begin
  			Bit#(8) data = 0;
  			if(rg_spi_cfg_cr1.lsbfirst == 1)
  				data = {wr_spi_in_io2, rg_data_rx[6 : 0]};				
  			else
  				data = {rg_data_rx[7 : 1], wr_spi_in_io2};
			rg_bit_count <= rg_bit_count + 1;
  			rx_fifo.enq(data);  
  			rg_data_counter <= 0;
  	    `logLevel( spi, 0, $format(" SPI : DATA_RECEIVE case2 counter %x data_rx %x", 
                                                                            rg_data_counter, data))
  	    $display($stime," SPI : DATA_RECEIVE case2 counter %x data_rx %x rg_bit_count %d",rg_data_counter, data,
		rg_bit_count);
  		end
  	end
  //	end
  	else if(!rx_fifo.notFull && wr_clk_en == 0 && wr_transfer_en == True) begin
  		rg_nss <= 1;
  		rg_data_counter <= 0;
  		rg_receive_state <= RECEIVE_DONE;
  		rg_spi_cfg_cr2 <= unpack(0); // rx_start and rx_imm_start are updated to zero
		rg_tx_rx_start <= 0;
  	  `logLevel( spi, 0, $format(" SPI : DATA_RECEIVE going to receive done"))
  	  $display($stime," SPI : DATA_RECEIVE going to receive done");
  	end
  endrule
  
  rule rl_receive_done(rg_receive_state == RECEIVE_DONE);
  	rg_receive_state <= IDLE;
  	`logLevel( spi, 0, $format(" SPI : RECEIVE_DONE going to idle"))
  	$display($stime," SPI : RECEIVE_DONE going to idle");
  endrule
  
rule rl_abort_tx_rx (wr_transfer_en == True &&  (rg_spi_cfg_cr1.spe == 1 && (
(rg_bit_count == rg_spi_cfg_cr1.total_bit_tx - 1) && rg_transmit_state != IDLE ) ||
((rg_spi_cfg_cr2.rx_start == 1 || rg_spi_cfg_cr2.rx_imm_start == 1) && 
rg_bit_count == (rg_spi_cfg_cr1.total_bit_rx - 1) && rg_receive_state != IDLE)));
	Bit#(8) data = 0;
	let data_tx = 0;
	if(rg_spi_cfg_cr2.rx_start == 0 && rg_transmit_state != IDLE 
	&& rg_bit_count == (rg_spi_cfg_cr1.total_bit_tx - 1) && wr_clk_en == 0) begin
  		if(rg_spi_cfg_cr1.lsbfirst == 1) begin
  			data_tx = rg_data_tx[0];
  			wr_spi_out_io1 <= rg_data_tx[0];
  		end
  		else begin
  			data_tx = rg_data_tx[7];
  			wr_spi_out_io1 <= rg_data_tx[7];
		end
		rg_transmit_state <= IDLE;
		rg_spi_cfg_sr.bsy <= 0;
		if(rg_spi_cfg_cr2.rx_imm_start == 1)
			rg_tx_rx_start <= 1;
		tx_fifo.clear();
		rg_bit_count <= 0;
	$display($stime()," SPI: ABORT tx %x", data_tx);
	end
	else if(rg_receive_state != IDLE && rg_bit_count == (rg_spi_cfg_cr1.total_bit_rx - 1)
	&& wr_clk_en == 1) begin
  			if(rg_spi_cfg_cr1.lsbfirst == 1)
  				data = {wr_spi_in_io2, rg_data_rx[6 : 0]};				
  			else
  				data = {rg_data_rx[7 : 1], wr_spi_in_io2};
		rg_receive_state  <= IDLE;
		rg_data_counter	  <= 0;
		rx_fifo.enq(data);
		rg_spi_cfg_cr2 <= unpack(0);
		rg_tx_rx_start <= 0;
//		rg_nss <= 1;
	$display($stime()," SPI: ABORT rg_data_rx %x", data);
	end	
endrule

  rule rl_receive_fifo_to_read_datareg(rx_fifo.notEmpty && rg_receive_state == IDLE);
  	Bit#(160) data = rg_concat_reg;
	if(rg_spi_cfg_cr1.lsbfirst == 1)	
	  	data[159 : 152] = rx_fifo.first();
	else
		data[7:0] = rx_fifo.first;
  	if(rg_data_counter < 19) begin
		if(rg_spi_cfg_cr1.lsbfirst == 1)
	  		rg_concat_reg <= data >> 8;
		else
	  		rg_concat_reg <= data << 8;
  		rg_data_counter <= rg_data_counter + 1;
		rx_fifo.deq();
  	end
  	else begin
		rg_concat_reg <= data;
  		rg_spi_cfg_sr.rxne <= 1;
		rx_fifo.deq();
  		rg_data_counter <= 0;
  	end
    `logLevel( spi, 0, $format(" SPI : rx_fifo to dr reg %x",data))
    $display($stime," SPI : rx_fifo to dr reg %x",data);
  endrule
  	
rule rl_abort_bitcount_finish(!rx_fifo.notEmpty && rg_receive_state ==IDLE &&
	rg_bit_count == rg_spi_cfg_cr1.total_bit_rx - 1);
		rg_spi_cfg_sr.rxne <= 1;
		if(rg_data_counter < 19 && rg_data_counter != 0) begin
			if(rg_spi_cfg_cr1.lsbfirst == 1)
	  			rg_concat_reg <= rg_concat_reg << 8;
			else
	  			rg_concat_reg <= rg_concat_reg >> 8;
		end
		rg_data_counter <= 0;
		rg_bit_count	<= 0;
endrule	
	
  	
  interface Ifc_spi_app app_interface;
  	method Bit#(data_width) data_to_app;
  		return wr_rd_data; // TODO hook created need to be changed later
  	endmethod
  	method Action read_request(Read_req#(addr_width) rd_req);
  		wr_rd_addr <= rd_req.addr; // TODO hook created and burst size if needed to be added or 
  								  // to removed from the method and struct
  	endmethod		
  	method Action write_request(Write_req#(addr_width, data_width)  wr_req);
  		wr_write_addr <= wr_req.addr; //TODO hook created and if needed in future use burst size
  		wr_write_data <= wr_req.wdata; // or remove from the struct itself
  	endmethod
  endinterface
  
  
  interface Ifc_spi_io io;
  	method bit mosi;
  		return wr_spi_out_io1;
  	endmethod
  	method bit sclk;
  		return  (rg_nss == 1) ? rg_spi_cfg_cr1.cpol : rg_clk;
  	endmethod
  	method Action miso(bit dat);
  		wr_spi_in_io2 <= dat ;
  	endmethod
      method bit nss;
  		return rg_nss;
  	endmethod
  endinterface
  
endmodule

module mkspi_controller#(Clock slow_clk, Reset slow_rst)(Ifc_spi_controller#(addr_width, 
                                                                            data_width,
                                                                            user_width))
  provisos(
          Add#(a__, 8, addr_width),
          Add#(b__, 32, data_width),
          Mul#(32, c__, data_width));

  AXI4_Lite_Slave_Xactor_IFC #(addr_width, data_width, user_width)  s_xactor_spi <- 
                                                                          mkAXI4_Lite_Slave_Xactor;
  
  Ifc_spi#(addr_width, data_width) spi <- mkspi(clocked_by slow_clk, reset_by slow_rst);
  
  
  SyncFIFOIfc#(Write_req#(addr_width, data_width)) 		ff_wr_req   <- mkSyncFIFOFromCC(1, slow_clk);

  // SyncFIFO is not used yet since the error in spi is not defined, after
  // defining the error this SyncFIFO maybe used
  //SyncFIFOIfc#(AXI4_Lite_Resp) 	ff_sync_wr_resp 	<- mkSyncFIFOToCC(1, slow_clk, slow_rst);
  SyncFIFOIfc#(Read_req#(addr_width))			ff_rd_req    		<- mkSyncFIFOFromCC(1, slow_clk);
  // This SyncFIFO only used for data may be in future the whole AXI response
  // will be used
  SyncFIFOIfc#(Bit#(data_width))		ff_sync_rd_resp	    <- mkSyncFIFOToCC(1, slow_clk, slow_rst);
  
  // considering spi controller wont get burst transaction address and data are
  // popped from same rule(moreover AXI_Lite is used which doesnt support burst mode), otherwise 
  // there should be a separate rule to pop address and data
  rule rl_write_request_from_core;
  	let aw <- pop_o(s_xactor_spi.o_wr_addr);
  	let w  <- pop_o(s_xactor_spi.o_wr_data);
  	`logLevel( spi, 0, $format(" SPI : Controller Write Channel"))
  	$display($stime," SPI : Controller Write Channel");
   	ff_wr_req.enq(Write_req {
  							  addr : truncate(aw.awaddr),
  							  burst_size : zeroExtend(aw.awsize),
  							  wdata : w.wdata });	
  
  	let w_resp = AXI4_Lite_Wr_Resp {bresp : AXI4_LITE_OKAY, buser : 0};
      s_xactor_spi.i_wr_resp.enq(w_resp);
  endrule
  
  rule rl_write_request_to_controller;
  	let w = ff_wr_req.first();	
  	spi.app_interface.write_request(w);
  	ff_wr_req.deq();
  endrule
  
  rule rl_read_request_from_core;      		
  	let ar <- pop_o(s_xactor_spi.o_rd_addr);
  	ff_rd_req.enq(Read_req {
  							addr : ar.araddr,
  							burst_size : zeroExtend(ar.arsize)});
  endrule
  
  rule rl_read_request_to_controller;
  	let r = ff_rd_req.first();
  	spi.app_interface.read_request(r);
  	ff_rd_req.deq();
  endrule
  
  rule rl_read_response_from_controller;
  	ff_sync_rd_resp.enq(spi.app_interface.data_to_app);
  endrule
  
  rule rl_read_response_to_core;
  	let rdata = ff_sync_rd_resp.first();
  	ff_sync_rd_resp.deq();
  	let r = AXI4_Lite_Rd_Data{rresp : AXI4_LITE_OKAY, rdata : rdata, ruser : 0};
  	s_xactor_spi.i_rd_data.enq(r);
  endrule
  
  interface io = spi.io;
  
  interface slave = s_xactor_spi.axi_side;

endmodule

endpackage

