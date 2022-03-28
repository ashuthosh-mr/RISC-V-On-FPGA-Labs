/* 
Copyright (c) 2013, IIT Madras All rights reserved.

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

Author: Vishvesh Sundaraman
Email id: vishu.vivek@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/
/*For the NVM the target will look like -> 4kB page. 256 pages per block and 2048 blocks (total 19 
addr bits)Assuming that NVM sends address in the following fashion(MSB to LSB):{Block_Addr(11bits)
, page_addr(8 bits)}Actually target has 2 chips. Each chip has 2 LUNs.Each lun has 2 plane.
Each plane has 1024 blocks. Each block has 64 pages(Total of 16Gb).*/
///////////////////////////////////////////////////////////////////////////////////////////////////
/*TODO: Need to gate re_n and we_n signals with the clk. Since ONFI expects these signals to be 
toggled.But doing it through rules means we_n & re_n runs at half the freq of clk, whch reduces the
BW. Hence need to find a way in bluespec where we can gate*/

/*ERRORS
1.Random programming of page in the block is prohibited in micron, the program command should be
sequential within the block.
2.Cannot write second time in a page without erasing.If wanted to write again in a page which is
written once, first erase and then write.//edited by Rathaiah
    AND THE ERROR IS: Partial page programming limit reached.  Block=0 Page=0 Limit=1
3.In this case,First WRITE(1page)-READ(1page) with addr as '0'.
Then WRITE(1page)-READ(1page) with addr as 'h100000000'.First WRITE and READ happened well.
There is an ERROR before Second WRITE with starting addr as 'h1000000000'.
AND THE ERROR is:
   3270975:2.state_count 1 and St2 //SECOND WRITE COMPLETED//

   3271005sending write response to the host 

main.top.target_ifc.uut_2.ERROR at time              3271030: tCS violation on We_n by                    5 rt                   10
main.top.target_ifc.uut_3.ERROR at time              3271030: MULTI-DIE STATUS READ (78h) IS PROHIBITED DURING AND AFTER POWER-UP RESET, OTP OPERATIONS, READ PARAMETERS, READ ID, READ UNIQUE ID, and GET/SET FEATURES.
main.top.target_ifc.uut_2.ERROR at time              3271030: MULTI-DIE STATUS READ (78h) IS PROHIBITED DURING AND AFTER POWER-UP RESET, OTP OPERATIONS, READ PARAMETERS, READ ID, READ UNIQUE ID, and GET/SET FEATURES.
   3271045:2.response received in the TB
   3271055:2.Sending read request to the controller
   3271095: received read request from host aruser 2730
main.top.target_ifc.uut_2.ERROR at time              3271160: DATA NOT Transfered on Re_n

CONCLUSIONS: Second write happened. But error came when checking status.
 */


`define verbose

package NandFlashController ;

`include "global_parameters_Flash.bsv"

import Vector::*;
import BRAM :: * ;
import FIFO::*;
import FIFOF::*;
import BRAMFIFO::*;
import DReg::*;
import TriState::*;
import ConfigReg ::*;
import SpecialFIFOs::*;
import DefaultValue ::*;
import AXI4_Types   :: *;
import AXI4_Fabric  :: *;
import Semi_FIFOF ::*;
import InterfaceECCencoder ::*;
import Clocks::*;

interface Ifc_NFC_Interface ;
	interface NandFlashInterface nvm_nfc_interface;
	interface ONFiInterface nfc_onfi_interface;
    interface ONFi_cfg_reg_interface cfg_reg_interface;
endinterface

//Creating enumeration for variois states in READ/PROGRAM/ERASE/RESET ... target

(* synthesize *)
//(*clock_family="clk,clk_inv"*)
module mkNandFlashController#(Clock clk_inv, Reset rst_inv,Clock clk_mux,Reset rst_mux)(Ifc_NFC_Interface) ;

Clock clk     <- exposeCurrentClock();


// Wires and Regs related to the Nand Flash Interface 
Reg#(Bit#(64))	  wr_address_from_nvm   <- mkReg(0);//address
Wire#(Bit#(`WDC)) wr_data_from_nvm 	    <- mkDWire(0);//data in
Reg#(Maybe#(Bit#(`WDC))) rg_data_to_nvm	<- mkDReg(tagged Invalid);// data out
Wire#(Bit#(1))	wr_nand_ce_n            <- mkDWire(1);// active low
Wire#(Bit#(1))	wr_nand_we_n            <- mkDWire(1);// active low
Wire#(Bit#(1))	wr_nand_re_n            <- mkDWire(1);// active low
Reg#(Bit#(1))	rg_interrupt            <- mkDReg(0);// active high
Reg#(Bit#(1))	rg_ready_busy_n	        <- mkReg(0);//active low.
Reg#(Bit#(11))	wr_w_length		        <- mkReg(0) ;	//Write length
Wire#(Bit#(11))	wr_r_length		        <- mkDWire(0) ;	//Read length
Wire#(Bit#(64))	wr_rd_addr_frm_nvm      <- mkDWire(0);  //Read address
Wire#(Bit#(1))	wr_nand_erase           <- mkDWire(0) ;	//Write length
Reg#(Bit#(1))	rg_write_success        <- mkDReg(0) ;	// active high
Reg#(Bit#(1))	rg_write_fail	        <- mkDReg(0) ;	// active high
Reg#(Bit#(1))	rg_erase_success        <- mkDReg(0) ;	// active high
Reg#(Bit#(1))	rg_erase_fail	        <- mkDReg(0) ;	// active high
Wire#(Bit#(1))	wr_nand_bbm_n           <- mkDWire(1) ;	// active low

// Regs and Wires for ONFI Interface
Vector#(`TOTAL_CHIPS,Reg#(Bit#(8)))  rg_data_to_flash           <- replicateM(mkReg(0));// data out 
Vector#(`TOTAL_CHIPS,Reg#(Bit#(8)))  rg_data_to_controller      <- replicateM(mkReg(0,clocked_by clk_mux,reset_by rst_mux)) ;// data out 
Vector#(`TOTAL_CHIPS, Reg#(Bool)) rg_dqs_to_flash    <- replicateM(mkReg(False, clocked_by clk_inv, reset_by rst_inv));
Vector#(`TOTAL_CHIPS, Reg#(Bool)) rg_dqs_c_to_flash  <- replicateM(mkReg(True, clocked_by clk_inv, reset_by rst_inv));
//Vector#(`TOTAL_CHIPS,Reg#(Bit#(8)))  rg_read_data_to_controller <- replicateM(mkSyncRegToCC(0, clk_inv, rst0));// data out 

ReadOnly#(Bit#(8)) wr_null_data0         <- mkNullCrossingWire(clk, rg_data_to_controller[0]);
ReadOnly#(Bit#(8)) wr_null_data1         <- mkNullCrossingWire(clk, rg_data_to_controller[1]);


ReadOnly#(Bool) wr_dqs_sync	       <-mkNullCrossingWire(clk, rg_dqs_to_flash[0]);
ReadOnly#(Bool) wr_dqs_c_sync	   <-mkNullCrossingWire(clk, rg_dqs_c_to_flash[0]);
ReadOnly#(Bool) wr_dqs2_sync	   <-mkNullCrossingWire(clk, rg_dqs_to_flash[1]);
ReadOnly#(Bool) wr_dqs2_c_sync	   <-mkNullCrossingWire(clk, rg_dqs_c_to_flash[1]);

Reg#(bit)  rg_en_rd_sync_data  <- mkSyncRegFromCC(0,clk_inv);


Reg#(bit)       rg_we_toggle   <- mkReg(1, clocked_by clk_inv, reset_by rst_inv);
Reg#(Bit#(1))   rg_onfi_we_n   <- mkReg(0);// active low
Reg#(Bit#(1))   rg_onfi_re_n   <- mkReg(1);// active low
Reg#(Bit#(1))   rg_onfi_cle    <- mkReg(0);// active high
Reg#(Bit#(1))   rg_onfi_ale	   <- mkReg(0);// active high
Reg#(Bit#(1))   rg_onfi_wp_n   <- mkReg(1);// active low
Vector#(`TOTAL_CHIPS,Wire#(Bit#(1))) wr_ready_busy_n  <- replicateM(mkWire);  // active low
Vector#(`TOTAL_CHIPS,Reg#(Bit#(1)))  rg_onfi_ce_n	  <- replicateM(mkReg(1));// active low

Reg#(bit)   we_flag     <- mkReg(0);
Reg#(bit)   set_we_ce   <- mkReg(0);
/* debug register and wire declaration*/

Wire#(Bool) wr_read_resp_ready <- mkWire();
// for testing to be deleted later
    


//Other Regs , Wires and FIFO's
//Will use  FIFO's for recieving data b/t NVMe and NFC..
FIFOF#(Bit#(`WDC))   ff_data_w_fifo   <- mkSizedBRAMFIFOF(`FIFO_ROWS+1);//FIFO size is 4 pages
//Will use  FIFO's for sending data b/t NVMe and NFC.
FIFOF#(Bit#(`WDC))   ff_data_r_fifo   <- mkSizedBRAMFIFOF(`FIFO_ROWS+1);	
Reg#(Bit#(1))        data_w_fifo_free <- mkReg(1); // Initially both write FIFO are free.
Reg#(Bit#(1))        data_r_fifo_free <- mkReg(1); // Initially both read FIFO are free.
Reg#(Bit#(64))       addr_register    <- mkReg(0); // Address register to store address.
Reg#(Bit#(64))       present_nvm_addr <- mkReg(0);
Reg#(Bit#(32))       rg_bbm_offset    <- mkReg(0); // To keep offset for bad block request
Reg#(Bit#(32))       rg_bbm_tempof    <- mkReg(0); // Temp offset for bad block request
Reg#(Bit#(`WDC))     rg_bit_map       <- mkReg(0); // To hold bbm data temparorily
Reg#(Bit#(TAdd#(TLog#(`WDC),1)))  bb_count   <- mkReg(0);// To keep count of bad blocks scanned
Reg#(Bit#(TAdd#(TLog#(`WDC),1)))  bb_count_t <- mkReg(0);
Reg#(Bit#(1))    send_bbm_data   <- mkReg(0);
Reg#(Bit#(64))   next_nvm_addr   <- mkReg(0); // Tracking Address from nvm with the length parameter.
Reg#(Bit#(TLog#(TAdd#(`PAGE_LENGTH,1))))   q_data_count_t  <- mkReg(0); // Track the data being put into the FIFO from NVM(_t means take) 
Reg#(Bit#(TLog#(TAdd#(`PAGE_LENGTH,1))))   q_data_count_g  <- mkReg(0); // Track the data being put into the FIFO from NVM(_g means give)
Reg#(Bit#(TLog#(TAdd#(`VALID_SPARE_AREA,1)))) spare_cnt  <- mkReg(0); // Track the spare area.
Reg#(Bit#(11))  local_length_w  <- mkReg(0);// This will hold the length parameter during the first request and then tracks(for write)
Reg#(Bit#(11))  local_length_r  <- mkReg(0);// This will hold the length parameter during the first request and then tracks(for read)
Reg#(Bit#(11))  pages2b_written <- mkReg(0);// This will hold the length parameter during the first request and then tracks(for write)
Reg#(Bit#(1))   chip_sel   <- mkConfigReg(0);// Points to which chip is going to be selected.
Reg#(Bit#(1))   plane_sel   <- mkConfigReg(0);// Points to which plane is going to be selected.
Reg#(Bit#(1))   cache_op_need   <- mkReg(0);// Flag to indicate whether cache operation is needed
Reg#(Bit#(8))   addr_cycl1   <- mkReg(0) ;// Can process 2 requests of write at a time, since we have 2 FIFO taking data from NVM
Reg#(Bit#(8))   addr_cycl2   <- mkReg(0) ;
Reg#(Bit#(8))   addr_cycl3   <- mkReg(0) ;
Reg#(Bit#(8))   addr_cycl4   <- mkReg(0) ;
Reg#(Bit#(8))   addr_cycl5   <- mkReg(0) ;
Reg#(Bit#(8))   a_cycl3_buff1   <- mkReg(0);// Temp location for addr
Reg#(Bit#(8))   a_cycl3_buff2   <- mkReg(0);// Temp location for addr
Reg#(Bit#(8))   a_cycl4_buff1   <- mkReg(0);// Temp location for  addr
Reg#(Bit#(8))   a_cycl4_buff2   <- mkReg(0);// Temp location for  addr
Reg#(Bit#(8))   a_cycl5_buff1   <- mkReg(0);// Temp location for  addr
Reg#(Bit#(8))   a_cycl5_buff2   <- mkReg(0);// Temp location for  addr
Reg#(Bit#(8))   mplane_cycl3_buff1   <- mkReg(0);// Temp location for multi plane addr
Reg#(Bit#(8))   mplane_cycl3_buff2   <- mkReg(0);
Reg#(Bit#(8))   mplane_cycl4_buff1   <- mkReg(0);
Reg#(Bit#(8))   mplane_cycl4_buff2   <- mkReg(0);
Reg#(Bit#(8))   mplane_cycl5_buff1   <- mkReg(0);
Reg#(Bit#(8))   mplane_cycl5_buff2   <- mkReg(0);
Reg#(Bit#(TLog#(TDiv#(`WDC,8)))) byte_count <- mkReg(0);// This is to keep track of how many bytes are sent to flash.
Reg#(Bit#(TLog#(TDiv#(`WDC,8)))) zero_index <- mkReg(0);// Index to compute filling of zeros in case of col offset.
Reg#(Bit#(1))                    new_r_req  <- mkReg(0);// New read request flag.
Reg#(Bit#(`WDC)) data_from_flash <- mkReg(0);// Buffer to store half words recieved from flash and convert them to `WDC to send to nvm.
Reg#(Bit#(1))    reset_flag      <- mkReg(0);// To reset the flash memory for power-on-reset.
Reg#(Bit#(1))    reset_wait_flag <- mkReg(1);// To wait before reset the flash memory for power-on-reset.
Reg#(Bit#(1))    reset_ongoing   <- mkReg(0);
Reg#(Bit#(1))    reset_applied   <- mkReg(0);
Reg#(Bit#(`COLUMN_WIDTH)) col_offset_p     <- mkReg(0);// Col offset during program
Reg#(Bit#(`COLUMN_WIDTH)) col_offset_r     <- mkReg(0);// Col offset during read
Reg#(Bit#(`COLUMN_WIDTH)) buf_col_offset_r <- mkReg(0);// Col offset during read
Reg#(Bit#(1)) get_next_addr         <- mkReg(0);
Reg#(Bit#(1)) block_erase_ongoing   <- mkReg(0);//Indicates erase is in progress
Reg#(Bit#(1)) read_pending          <- mkReg(0);//Indicates read in progress
Reg#(Bit#(1)) start_program         <- mkReg(0);//Indicates start of program cycle
Reg#(Bit#(1)) last_r_req            <- mkReg(0);//Indicates last read in progress
Reg#(Bit#(1)) stay_with_decision    <- mkReg(0);//Control for next flag "decide_read"
Reg#(Bit#(1)) decide_read           <- mkConfigReg(0);// Indicates decision of read
Reg#(Bit#(1)) initial_status_ck     <- mkReg(0);//Indicates if a status check is needed or not.
Reg#(Bit#(1)) q_fill_zeros          <- mkReg(0);//Indicates if Q needs zeros in caseof coloffset.
Reg#(Bit#(1)) multi_plane_r_pend_1  <- mkReg(0);//Multi plane op flag
Reg#(Bit#(1)) multi_plane_r_pend_2  <- mkReg(0);//Multi plane op flag
Reg#(Bit#(1)) page_r_pend_1         <- mkReg(0);//Multi plane op flag
Reg#(Bit#(1)) page_r_pend_2         <- mkReg(0);
Reg#(Bit#(1)) multi_plane_after_page<- mkReg(0);//To identify multiplane req aftr a single page req
Reg#(Bit#(2)) status_count          <- mkReg(0);
Reg#(Bit#(2)) status_done           <- mkReg(0);//flag to keep track of the status checks done
Reg#(Bit#(2)) alter_cnt             <- mkReg(0);  //during program.
Reg#(Bit#(1)) flag_end_read         <- mkReg(0);
Reg#(Bit#(1)) data_reg_loaded       <- mkReg(0);
Reg#(Bit#(1)) first_entry           <- mkReg(0);
Reg#(Bit#(1)) program_failed        <- mkReg(0);//To keep track if any program failed.
Reg#(Bit#(1)) last_status_r         <- mkReg(0);//To keep track of the last status read 
Reg#(Bit#(1)) first_byte            <- mkReg(0);
Reg#(Bit#(1)) allow_write_q         <- mkReg(0);
Reg#(Bit#(1)) cal_block_addr        <- mkReg(0);
Reg#(Bit#(1)) feature_data_done     <- mkReg(0);
Reg#(Bit#(1)) badblock_flag         <- mkReg(0);// To initiate BBM on power-on.TODO revert back to 1
Reg#(Bit#(1)) bb_search             <- mkReg(0);// To start searching for bad block
//ecc control reg
//Reg#(Bit#(2))         write_delay        <- mkReg(0) ;
//Reg#(Bit#(2))         end_write_delay    <- mkReg(0) ;
//Reg#(Bit#(1))	      send_ecc_enc_start <- mkReg(1) ;
Reg#(Bit#(10))        sector_byte_cnt <-mkReg(0);// Track each sector of 512B.Fixed.
Reg#(Erase_operation_state) erase_state <-mkReg(START_ERASE);//State variable for erase operation.
Reg#(Read_operation_state) read_state   <-mkReg(START_READ);//State variable for read operation. 
Reg#(BBM_operation_state) bbm_state     <-mkReg(GET_ADDR);//State variable for bbm operation.     
Reg#(Program_operation_state) program_state <-mkReg(START_PROGRAM);//State reg for write operation. 
Reg#(Feature_operation_state) feature_state <-mkReg(IDLE);//State reg for set feature. 
Reg#(Read_states)     present_r_state <-mkReg(IDLE);// Present read state.
Reg#(Read_states)     next_r_state    <-mkReg(IDLE);// Next read state.
Reg#(Program_states)  present_w_state <-mkReg(IDLE);// Present write state.
Reg#(Program_states)  next_w_state    <-mkReg(IDLE);// Next write state.
Reg#(Program_states)  prev_w_state    <-mkReg(IDLE);// Previous write state.
Reg#(Bit#(13))        delay_count     <-mkReg(0);// Counter to manage timing issues.
Reg#(Bit#(3))         d_sync_count    <-mkReg(0);
Reg#(Bit#(1))         timing_set      <- mkReg(0);// Bit indicates whether timing is set or not
//Reg#(Bit#(1))         timing_set_cc   <- mkSyncRegFromCC(0,clk_inv);// Bit indicates whether timing is set or not
Reg#(Bit#(1))         gate_we         <-mkReg(0);// To hold we_n for longer period than the clock.
Reg#(Bit#(8))         sreg            <-mkReg(0);// Status register
Reg#(Bit#(8))         dreg            <-mkReg(0);// Data reg

// Configuration register
Reg#(Bit#(8))       rg_cfg_timing_mode <- mkReg('h12);


Reg#(Bit#(TSub#(`MAX_ROW_BITS,`PAGE_WIDTH))) block_addr <- mkReg(0);//scan block address on power-on.
//Below registers are used for block erase operation and BBM purpose.
Vector#(`TOTAL_PLANE,Reg#(Bit#(8)))  alter_addr_cycl3     <- replicateM(mkReg(0)) ;
Vector#(`TOTAL_PLANE,Reg#(Bit#(8)))  alter_addr_cycl4     <- replicateM(mkReg(0)) ;
Vector#(`TOTAL_PLANE,Reg#(Bit#(8)))  alter_addr_cycl5     <- replicateM(mkReg(0)) ;

BRAM_Configure cfg_bbm_list = defaultValue ;
//Memory to store bad block list on power-on
BRAM1Port#(Bit#(TAdd#(`BLOCK_WIDTH,1)), Bit#(1))  bbm_list      <- mkBRAM1Server (cfg_bbm_list) ; 
Reg#(Bit#(TAdd#(`BLOCK_WIDTH,1))) bbm_list_addr <- mkReg(0);// Addres register for bbm list memory.

Wire#(Bit#(6)) wr_onfi_signal <- mkWire();
Wire#(bit)     wr_onfi_we_n   <- mkWire();  

Reg#(bit)		 enable_dataout  <- mkReg(0) ;	         // active high
Reg#(bit)		 enable_dqs      <- mkReg(0) ;	         // active high

Vector#(`TOTAL_CHIPS, Wire#(bit))       rg_dqs_to_controller    <- replicateM(mkWire());
Vector#(`TOTAL_CHIPS, Reg#(bit))        rg_dqs_c_to_controller  <- replicateM(mkReg(0));

ReadOnly#(Bit#(1))    wr_we_null_cc	    <- mkNullCrossingWire(clk, rg_we_toggle);

let lv_wr_state_machine = (start_program == 1'b1 && (present_w_state == PROGRAM_PAGE ||
present_w_state == PROGRAM_PAGE_CACHE || present_w_state == PROGRAM_PAGE_MULTI_PLANE) &&
wr_ready_busy_n[chip_sel] == 1);

rule rl_onfi_control_signal_connection;
    rg_onfi_ce_n[chip_sel]  <= wr_onfi_signal[5] ;
    rg_onfi_ce_n[~chip_sel] <= wr_onfi_signal[4] ;
    rg_onfi_re_n            <= wr_onfi_signal[3] ;
    rg_onfi_cle             <= wr_onfi_signal[2] ;
    rg_onfi_ale             <= wr_onfi_signal[1] ;
    enable_dataout          <= wr_onfi_signal[0];
endrule

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
rule rl_standby_onfi (wr_nand_ce_n == 1'b1) ;
// `ifdef verbose $display ("Nandflashcontroller:  CHIP ENABLE PIN IS DISABLED NFC \n"); `endif
endrule

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////Rules to take data from NVM to the FIFO while write                                   
/////////////////////////////////////////////////////////////////////////////////////////////////
rule rl_fshow_program_state;
//  `ifdef verbose      $display($stime(),"Nandflashcontroller: : PROGRAM_STATE",fshow(program_state));  `endif
endrule

/*Put data from NVMe into the write queue Our assumption is that for a given program command, the 
length does not cross over to the next block. i.e for a given program command,given A block address
,the length can go from 0 to PAGES-1 */
//If a prpgram fails at any instant need to discard everything, hence need this condition on rule
rule rl_q_data_from_nvme_fifo (wr_nand_ce_n == 1'b0 && wr_nand_we_n == 1'b0 && (data_r_fifo_free ==
    1'b1 || data_w_fifo_free == 1'b1 || allow_write_q == 1'b1) && program_failed == 1'b0);
`ifdef verbose $display($stime(),"Nandflashcontroller: : Control is in q_data_from_nvme local_length %d", local_length_w);`endif
ff_data_w_fifo.enq(wr_data_from_nvm) ;
if(q_data_count_t == 'h0) begin
 if(local_length_w == 'h0) begin //Corresponds to a new request from NVM
  local_length_w  <= wr_w_length;//Length to be stored for multiple writes(Initially)
  pages2b_written <= wr_w_length ;
  addr_register   <= fn_map_address(wr_address_from_nvm) ;
  next_nvm_addr   <= fn_get_next_nvm_addr(wr_address_from_nvm); 
/*Right now 2 chips are separated based on the MSB address bit. adjacent blocks can also be put in
two different chips.Say block0 in chip0 and block1 in chip1 as block0.For this address mapping has 
to be changed.We will deal this later*/
//MSB bit selects which chip is to be written into
  chip_sel <= wr_address_from_nvm[`COLUMN_WIDTH+`PAGE_WIDTH+`BLOCK_WIDTH+`PLANE_WIDTH+`LUN_WIDTH];
//All even pages lie in first plane of LUN and odd pages in second plane of LUN
//plane_sel <= wr_address_from_nvm[`COLUMN_WIDTH + `PAGE_WIDTH]; 
    plane_sel <= fn_map_address(wr_address_from_nvm)[`COLUMN_WIDTH + `PAGE_WIDTH];
  if(wr_w_length == 'h1)
      status_count <= 'h1;
  else if(wr_w_length == 'h2) begin
   if(wr_address_from_nvm[`COLUMN_WIDTH] == 'h0) //Even plane
    status_count <= 'h1; //Need to check status of only one LUN before start of read.
   else
    status_count <= 'h2; //Need to check status of two different LUNS before start of read.
  end
  else
      status_count <= 'h2;
  col_offset_p    <= wr_address_from_nvm[`COLUMN_WIDTH-1:0]; //Column address offset.
  byte_count      <= wr_address_from_nvm[`LBPR-1:0];//This is for byte offset within col offset.
/*For every sector of 512B we need to store ECC, hence taking the offset as initial value fo this.
Need to reset after every sector*/
  sector_byte_cnt <= {1'b0,wr_address_from_nvm[8:0]};
//FIFO is WDC bits wide, hence col address offset needs to be translated into numb of rows in fifo.
  q_data_count_t  <= q_data_count_t + wr_address_from_nvm[`COLUMN_WIDTH-1:`LBPR] +'h1; 
  end	
  else
  begin
   local_length_w  <= local_length_w - 'h1;
//Map the new address (which was stored in next_address reg in prev page write)
   addr_register   <= fn_map_address(next_nvm_addr); 
   //Next page address is current address + 1
   next_nvm_addr   <= fn_get_next_nvm_addr(next_nvm_addr); 
   plane_sel       <= next_nvm_addr[`COLUMN_WIDTH + `PAGE_WIDTH];
   //Col offset is applicable only for the first page in case of mutiple page request.
   col_offset_p      <= 'h0;
   q_data_count_t  <= q_data_count_t + 'h1;
  end
end
else if(q_data_count_t < (`PAGE_LENGTH-1))
    q_data_count_t <= q_data_count_t + 'h1;
else if(q_data_count_t < `PAGE_LENGTH)
begin
//Last but one data cycle, need to update data_w_fifo_free, since this is used to get ready signal
//and we dont want any delay in it
    q_data_count_t   <= q_data_count_t + 'h1;
    data_w_fifo_free <= 1'b0;
    allow_write_q    <= 1'b1;
end
else
begin
    q_data_count_t   <= 'h0;
    allow_write_q    <= 1'b0;
    start_program    <= 1'b1;
    if(col_offset_p == 'h0) //Need to reset col add since only first page must have column addr.
    begin
        addr_cycl1 <= 'h0;
        addr_cycl2 <= 'h0;
    end
    else
    begin
    `ifdef COLUMN_WIDTH_LT_8
        addr_cycl1	  <= {'h0,addr_register[`COLUMN_WIDTH-1:0]};
        addr_cycl2 	  <= 'h0;
    `elsif COLUMN_WIDTH_E_8 
        addr_cycl1	 <= addr_register[7:0];
        addr_cycl2	 <= 'h0;
    `else
        addr_cycl1       <= addr_register[7:0];
        Bit#(TSub#(16,`COLUMN_WIDTH)) fill_z = 'h0; //TODO ? 16
//        `COLUMN_WIDTH cannot be 16.Must be LTE 15
        addr_cycl2       <= {fill_z,addr_register[`COLUMN_WIDTH-1:8]};
    `endif
    end
    addr_cycl3	 <= addr_register[7+`COLUMN_WIDTH:`COLUMN_WIDTH];
    addr_cycl4	 <= addr_register[15+`COLUMN_WIDTH:`COLUMN_WIDTH+8];
    addr_cycl5	 <= addr_register[23+`COLUMN_WIDTH:`COLUMN_WIDTH+16];
end
endrule
	
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////  Rule to create state machine for write to target  ///////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
/*Implementing plane first and LUN next states*/
rule rl_write_state_decision (data_w_fifo_free == 1'b0 && present_w_state == IDLE);
	case(local_length_w)
		'h0     :	begin
            `ifdef verbose	$display("Nandflashcontroller: Write Length parameter is zero"); `endif
				end
		'h1 	: 	begin
/*If there is only one page to be written it is either the only page request from NVM or it can be 
any odd page request (like 5,9 etc). Hence it can mean PROGRAM PAGE (only one page) or 
PROGRAM_PAGE_CACHE (5,9.. pages)*/
					if(cache_op_need == 1'b0)
						present_w_state <= PROGRAM_PAGE;
					else
                        present_w_state <= PROGRAM_PAGE_CACHE;
					next_w_state    <= IDLE;
					cache_op_need <= 1'b0;
                    $display($stime(),"Nandflashcontroller: Control is in write state decision for single page");
				end
	'h2,'h3,'h4	 : 	begin
/*IF two pages to be written there are two possibilities. a) The pages fall on different LUNs
(plane1 of LUN0 , plane0 of LUN1 or plane1 of LUN1 and plane0 of LUN0) b) They fall on the same LUN
*/                  if(plane_sel == 1'b0)
					begin
						present_w_state <= PROGRAM_PAGE_MULTI_PLANE;
						if(cache_op_need == 1'b0)
							next_w_state <= PROGRAM_PAGE;
						else
							next_w_state <= PROGRAM_PAGE_CACHE;
						if(local_length_w == 'h2)
							cache_op_need <= 1'b0;
					end
					else
					begin
						if(cache_op_need == 1'b0)
							present_w_state <= PROGRAM_PAGE;
						else
							present_w_state <= PROGRAM_PAGE_CACHE;
						next_w_state <= IDLE;
					end
				end
	default 	: 	begin
				/*For any other case we go for program page cache operation*/
					if(plane_sel == 1'b0)
					begin
						present_w_state <= PROGRAM_PAGE_MULTI_PLANE;
						next_w_state    <= PROGRAM_PAGE_CACHE;
					end
					else
					begin
						present_w_state <= PROGRAM_PAGE_CACHE;
						next_w_state    <= IDLE;
					end
					cache_op_need <= 1'b1;
				end
	endcase
endrule

rule rl_we_toggle;
    rg_we_toggle <= ~rg_we_toggle;
endrule

//rule rl_prgm_sync_mode(timing_set == 1);
//    rg_onfi_we_n <= wr_we_null_cc;
//endrule

rule rl_prgm_sync_dqs;
    rg_dqs_to_flash[0]   <= unpack(~rg_we_toggle);
    rg_dqs_to_flash[1]   <= unpack(~rg_we_toggle);
    rg_dqs_c_to_flash[0] <= unpack(rg_we_toggle);
    rg_dqs_c_to_flash[1] <= unpack(rg_we_toggle);
endrule

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////// Rule to PROGRAM            ///////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

(*mutually_exclusive = "rl_pgrm_commands, rl_pgrm_commands_START_DATA, rl_pgrm_commands_DUMMY"*)
rule rl_pgrm_commands(lv_wr_state_machine && program_state != START_DATA &&
program_state != DUMMY && program_state != ECC_REQ);	
case(program_state)       
ASK_STATUS1 : begin 
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b1 ;
    lv_onfi[1]        = 1'b0 ;
    lv_onfi[0]        = 1'b1 ;
   if(prev_w_state == PROGRAM_PAGE_MULTI_PLANE && last_status_r == 1'b0)
    program_state <= START_PROGRAM ;
   else
   begin
   /*Check status of the LUN*/
    rg_data_to_flash[chip_sel]        <= 'h78 ;
    if(status_done <= 'h3)
        status_done   <= status_done + 'h1 ;
    program_state     <= WAIT_D1 ;
   end  
    wr_onfi_signal    <= lv_onfi;
    
end 
WAIT_D1: begin
    /*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count > 0 && d_sync_count < 5) begin
        d_sync_count <= d_sync_count + 1;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count    <= 0;
        lv_onfi[0]      = 0;
        lv_onfi[2]      = 0;
        program_state   <= P_ADDR_S ;
    end
    wr_onfi_signal    <= lv_onfi;
end
WAIT_NEW_3: begin
    /*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            d_sync_count    <= d_sync_count + 1;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            program_state   <= B_ADDR_S ;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            d_sync_count <= d_sync_count + 1;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            program_state <= L_ADDR_S ;
            delay_count   <='h0; 
        end
    wr_onfi_signal <= lv_onfi;
end
P_ADDR_S: begin
    wr_onfi_signal          <= 6'b011011;
    if(last_status_r == 1'b1)
        rg_data_to_flash[chip_sel]        <= a_cycl3_buff1;
    else
        rg_data_to_flash[chip_sel]        <= addr_cycl3 ;
    program_state           <= WAIT_NEW_3; 
    delay_count         <= 1;
end
B_ADDR_S: begin
    wr_onfi_signal          <= 6'b011011;
    if(last_status_r == 1'b1)
        rg_data_to_flash[chip_sel]        <= a_cycl4_buff1;
    else
        rg_data_to_flash[chip_sel]        <= addr_cycl4 ;
    program_state           <= WAIT_NEW_3;
    delay_count     <= delay_count + 1;
end
L_ADDR_S: begin
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    lv_onfi[1]        = 1'b1 ;
    lv_onfi[0]        = 1'b1 ;
    wr_onfi_signal          <= lv_onfi;
    if(last_status_r == 1'b1)
        rg_data_to_flash[chip_sel]        <= a_cycl5_buff1;
    else
        rg_data_to_flash[chip_sel]        <= addr_cycl5 ;
    program_state           <= WAIT_RE_WE1 ;                        
end
WAIT_RE_WE1: begin
    /*Wait for tWHR time period*/
    $display($stime,"WAIT_RE_WE1");
    Bit#(6)  lv_onfi;   
    lv_onfi[5]          = 1'b0 ;
    lv_onfi[4]          = 1'b1 ;
    enable_dqs          <= 0;
    if(delay_count == 0) begin
        lv_onfi[3]      = 1'b1 ;
        lv_onfi[2]      = 1'b0 ;
        lv_onfi[1]      = 1'b1; //rg_onfi_ale         <= 1'b1;
        lv_onfi[0]      = 1'b1;
        delay_count     <= delay_count + 1;
    end
    else if(delay_count == `TWHR_COUNT)
    begin
        lv_onfi[3]      = 1'b1 ;
        lv_onfi[2]      = 1'b0 ;
        lv_onfi[1]      = 1'b0; //rg_onfi_ale     <= 1'b0;
        lv_onfi[0]      = 1'b0; 
        delay_count     <= 'h0;
        program_state       <= ENABLE_S_READ1 ;
    end
    else begin
        lv_onfi[3]      = 1'b1 ;
        lv_onfi[2]      = 1'b0 ;
        lv_onfi[1]      = 1'b0; //rg_onfi_ale     <= 1'b0;
        lv_onfi[0]      = 1'b0;
        delay_count     <= delay_count + 'h1;
    end
    wr_onfi_signal  <= lv_onfi;
end
ENABLE_S_READ1: begin
    $display($stime,"ENABLE_S_READ1");
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[0]        = 1'b0 ;
    enable_dqs          <= 0;
    if(delay_count < 2) begin
        lv_onfi[3]        = 1'b0 ;
        lv_onfi[2]        = 1'b1 ;
        lv_onfi[1]        = 1'b1 ; 
        delay_count       <= delay_count + 1;
    end
    else begin 
        lv_onfi[3]        = 1'b0 ;
        lv_onfi[2]        = 1'b1 ;
        lv_onfi[1]        = 1'b1 ; 
        delay_count       <= 0;
        program_state     <= WAIT1;
    end
    wr_onfi_signal          <= lv_onfi;
end
WAIT1: begin //TODO WAIT1 and WAIT_FOR_tRHW can be merged
   $display($stime,"WAIT1");
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b0 ;
    lv_onfi[2]        = 1'b1 ;
    lv_onfi[1]        = 1'b1 ; 
    lv_onfi[0]        = 1'b0 ;
//  rg_en_rd_sync_data <= 0;
    if(delay_count == 2) begin 
        delay_count       <= 0;
        sreg              <= (chip_sel == 1) ? wr_null_data1 : wr_null_data0;
        program_state     <= WAIT_FOR_tRHW ;
    end
    else  
        delay_count       <= delay_count + 1;
    wr_onfi_signal        <= lv_onfi;
end
WAIT_FOR_tRHW: begin //vis: added for read to write high, timing issue.
    $display($stime,"WAIT_FOR_tRHW");
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    lv_onfi[1]        = 1'b0 ;
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal    <= lv_onfi;
    if(delay_count == `TRHW_COUNT) begin 
        program_state  <= READ_STATUS1;
        delay_count <= 0;
    end
    else delay_count <= delay_count + 1;
end
READ_STATUS1: begin
  $display($stime,"READ_STATUS1");
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    lv_onfi[1]        = 1'b0 ;
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal          <= lv_onfi;
    if(last_status_r == 1'b1) begin
        if(sreg[6] ==1'b1 && sreg[5] ==1'b1) begin
            program_state  <= DUMMY ;
            if(sreg[0] ==1'b1 || sreg[1] ==1'b1)
                program_failed <= 1'b1 ;
        end
        else
            program_state  <= ASK_STATUS1;
    end
    else begin
        if(present_w_state == PROGRAM_PAGE_CACHE) begin
            if(sreg[6] ==1'b1 && sreg[5] == 1'b1)
                if(status_done > status_count) //Need to check validity of prev write.
                    if(sreg[1] ==1'b1 || sreg[0] == 1'b1) begin
                        program_failed <= 1'b1 ;
                        program_state  <= DUMMY ;
                    end
                    else
                        program_state  <= START_PROGRAM ;
                else
                    program_state  <= ASK_STATUS1 ;
            else
                program_state <= ASK_STATUS1;
        end
/*If preceding the multi plane program there was a program page cache operation then we need to
check only the RDY flag.Hence cache_op_need == 1 says earlier we had cache op(refer to rule
state_decision)*/
       else if(present_w_state == PROGRAM_PAGE_MULTI_PLANE && (cache_op_need == 1'b1 ||
           (cache_op_need == 1'b0 && next_w_state == PROGRAM_PAGE_CACHE))) begin
           if(sreg[6] ==1'b1 && sreg[5] == 1'b1)
               if(status_done > status_count) //Need to check validity of prev write.
                   if(sreg[1]==1'b1 || sreg[0] == 1'b1) begin
                       program_failed <= 1'b1 ;
                       program_state  <= DUMMY ;
                   end
                   else 
                       program_state  <= START_PROGRAM ;
               else 
                   program_state  <= ASK_STATUS1 ;
           else
               program_state <= ASK_STATUS1;
       end
       else begin 
/*Here we did not have a cache operation , hence need to check both RDY and ARDY flags*/
            if(sreg[6] ==1'b1 && sreg[5] ==1'b1)
                if(status_done > status_count) //Need to check validity of prev write.
                    if(sreg[0] ==1'b1 || sreg[1] ==1'b1) begin
                        program_failed <= 1'b1 ;
                        program_state  <= DUMMY ;
                    end
                    else
                        program_state  <= START_PROGRAM ;
                else
                    program_state  <= ASK_STATUS1 ; 
           else
               program_state <= ASK_STATUS1;
        end
    end
end
ASK_STATUS2: begin
/*If LUN is busy dont waste 6 cycles by polling using 78h instead use 70h for polling*/
    /*Check status of the LUN*/
    wr_onfi_signal          <= 6'b011101;
    rg_data_to_flash[chip_sel]        <= 'h70 ;
    program_state           <= WAIT_RE_WE2 ;
end
WAIT_RE_WE2: begin
    /*Wait for tWHR time period*/
    wr_onfi_signal          <= 6'b011100;
    if(delay_count == `TWHR_COUNT)
    begin
        delay_count <= 'h0;
        program_state       <= ENABLE_S_READ2 ;
    end
    else
        delay_count <= delay_count + 'h1;
end
ENABLE_S_READ2: begin
    wr_onfi_signal          <= 6'b010000;
    program_state           <= WAIT2 ; 
end
WAIT2: begin
    wr_onfi_signal          <= 6'b011000;
    program_state           <= READ_STATUS2 ;
    sreg                    <= (chip_sel == 1) ? wr_null_data1 : wr_null_data0;
end
READ_STATUS2: begin
    wr_onfi_signal          <= 6'b010000;
    if(pages2b_written == 'h0 || last_status_r == 1'b1) begin//Final status checking  
        if(sreg[6] ==1'b1 && sreg[5] ==1'b1) begin           //after everything
            program_state  <= DUMMY ;
            if(sreg[0]==1'b1 || sreg[1] ==1'b1)
                program_failed <= 1'b1 ;
        end
        else
            program_state  <= ASK_STATUS1 ;
    end
    else begin
        if(present_w_state == PROGRAM_PAGE_CACHE) begin	
            if(sreg[6] == 1'b1 || sreg[5] == 1'b1)
                if(status_done > status_count) //Need to check validity of prev write.
                    if(sreg[1]==1'b1 || sreg[0] == 1'b1) begin
                        program_failed <= 1'b1 ;
                        program_state  <= DUMMY ;
                    end
                    else
                        program_state  <= START_PROGRAM ;
                else
                    program_state  <= ASK_STATUS1 ;
           else
               program_state <= ASK_STATUS1;
        end
/*If preceding the multi plane program there was a program page cache operation then we need to
check only the RDY flag.Hence cache_op_need == 1 says earlier we had cache op(refer to rule
state_decision)*/
       else if(present_w_state == PROGRAM_PAGE_MULTI_PLANE && (cache_op_need == 1'b1 ||
           (cache_op_need == 1'b0 && next_w_state == PROGRAM_PAGE_CACHE)))
           if(sreg[6] ==1'b1 && sreg[5] == 1'b1)
               if(status_done > status_count) //Need to check validity of prev write.
                   if(sreg[0]==1'b1 || sreg[1]==1'b1) begin
                       program_failed <= 1'b1 ;
                       program_state  <= DUMMY ;
                   end
                   else
                       program_state  <= START_PROGRAM ;
               else
                   program_state  <= ASK_STATUS1 ;
           else
               program_state <= ASK_STATUS1;
       else begin 
/*Here we did not have a cache operation , hence need to check both RDY and ARDY flags*/
            if(sreg[6]==1'b1 && sreg[5]==1'b1)
                if(status_done > status_count) //Need to check validity of prev write.
                    if(sreg[0]==1'b1 || sreg[1]==1'b1)begin
                        program_failed <= 1'b1 ;
                        program_state  <= DUMMY ;
                    end
                    else
                        program_state  <= START_PROGRAM ;
                else
                    program_state  <= ASK_STATUS1 ; 
            else
                program_state <= ASK_STATUS1;
        end
    end
end
START_PROGRAM: begin
/*Send 80h command series from here*/	
     if(delay_count == 0) begin
        delay_count             <= delay_count + 1;
        wr_onfi_signal          <= 6'b011000;
     end
     else begin
        wr_onfi_signal               <= 6'b011101;        
        rg_data_to_flash[chip_sel]   <= 'h80 ;
        program_state                <= WAIT_D2 ;
        delay_count                  <= 0; 
     end
end
WAIT_D2: begin
/*For tCLH we spend one cycle here*/
    if(delay_count == 0) begin
        delay_count         <= delay_count + 1;
        wr_onfi_signal          <= 6'b011101;
    end
    else if(delay_count > 0 && delay_count < 5) begin
        delay_count         <= delay_count + 1;
        wr_onfi_signal          <= 6'b011000;
    end
    else begin
        program_state           <= C_ADDR ;
        wr_onfi_signal          <= 6'b011000;
        delay_count <= 0;
    end
end
WAIT_NEW_1: begin
/*For tCLH spend one cycle here*/
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            wr_onfi_signal      <= 6'b011011;
            d_sync_count        <= d_sync_count + 1;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= d_sync_count + 1;
        end
        else begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= 0;
            program_state       <= C_ADDR1 ;
        end
    else if (delay_count =='h2) 
        if(d_sync_count == 0) begin
            wr_onfi_signal      <= 6'b011011;
            d_sync_count        <= d_sync_count + 1;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= d_sync_count + 1;
        end
        else begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= 0;
            program_state          <= P_ADDR ;
        end
    else if(delay_count == 'h3)
        if(d_sync_count == 0) begin
            wr_onfi_signal      <= 6'b011011;
            d_sync_count        <= d_sync_count + 1;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= d_sync_count + 1;
        end
        else begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= 0;
            program_state       <= B_ADDR;
        end
    else if(delay_count == 'h4) begin
        if(d_sync_count == 0) begin
            wr_onfi_signal      <= 6'b011011;
            d_sync_count        <= d_sync_count + 1;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= d_sync_count + 1;
        end
        else begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= 0;
            program_state       <= L_ADDR;
            delay_count<='h0;
        end
     end
end
C_ADDR: begin
    wr_onfi_signal               <= 6'b011011;
    rg_data_to_flash[chip_sel]   <= addr_cycl1 ;
    program_state                <= WAIT_NEW_1;
    delay_count                  <= 1;
end
C_ADDR1: begin
    wr_onfi_signal               <= 6'b011011;
    rg_data_to_flash[chip_sel]   <= addr_cycl2 ;
    program_state                <= WAIT_NEW_1;
    delay_count                  <= 2;
end
P_ADDR: begin
/*Send page address for program*/
    wr_onfi_signal               <= 6'b011011;
    rg_data_to_flash[chip_sel]   <= addr_cycl3 ;
    program_state                <= WAIT_NEW_1;
    delay_count                  <= 3; 
end
B_ADDR: begin
/*Send block address for program*/
    wr_onfi_signal               <= 6'b011011;
    rg_data_to_flash[chip_sel]   <= addr_cycl4 ;
    program_state                <= WAIT_NEW_1;
    delay_count                  <= 4; 
end
L_ADDR: begin
/*Send block address for program*/
    wr_onfi_signal               <= 6'b011011;
    rg_data_to_flash[chip_sel]   <= addr_cycl5 ;
    program_state                <= WAIT_D3 ; 
    q_data_count_g               <= truncate(col_offset_p>>`LBPR) ;
end
WAIT_D3: begin
/*Spend tADL time here.Also takes care of tALH.*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    if(delay_count == 0) begin
        lv_onfi[2]  = 1'b0 ;
        lv_onfi[1]  = 1'b1; //rg_onfi_ale             <= 1'b1 ;
        lv_onfi[0]  = 1'b1;
        delay_count <= delay_count + 1;
    end
    else if(delay_count <= `TADL_COUNT -2)begin
        lv_onfi[2]  = 1'b0 ;
        lv_onfi[1]  = 1'b0; //rg_onfi_ale             <= 1'b1 ;
        lv_onfi[0]  = 1'b0;
        delay_count <= delay_count + 1;
    end
    else if(delay_count == `TADL_COUNT - 1) begin
        lv_onfi[2]  = 1'b1 ;
        lv_onfi[1]  = 1'b1 ;//rg_onfi_ale        <= 1'b0;
        lv_onfi[0]  = 1'b0 ;
        enable_dqs  <= 1;
        delay_count <= delay_count + 1;
    end
    else if(delay_count == `TADL_COUNT) begin
        lv_onfi[2]  = 1'b1 ;
        lv_onfi[1]  = 1'b1 ;//rg_onfi_ale        <= 1'b0;
        lv_onfi[0]  = 1'b0 ;
        delay_count <= 'h0;
        program_state <= START_DATA;
    end
    else begin
        lv_onfi[2]  = 1'b1 ;
        lv_onfi[1]  = 1'b1; //rg_onfi_ale        <= 1'b0;
        lv_onfi[0]  = 1'b0 ;
        delay_count <= delay_count + 'h1;
    end
    wr_onfi_signal  <= lv_onfi;
end
WAIT_D4  :  begin
/*For tCLH we spend one cycle here*/
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == 5) begin
        program_state           <= END_COMMAND ;
        delay_count             <= 0;
    end
    else 
        delay_count             <= delay_count + 1;
end		
END_COMMAND: begin
    wr_onfi_signal          <= 6'b011101;
    if(present_w_state == PROGRAM_PAGE_MULTI_PLANE)
        rg_data_to_flash[chip_sel]        <= 'h11 ;
    else if(present_w_state == PROGRAM_PAGE_CACHE)
        rg_data_to_flash[chip_sel]        <= 'h15 ;
    else
        rg_data_to_flash[chip_sel]        <= 'h10 ;
    program_state           <= WAIT_TWB ;
    pages2b_written         <= pages2b_written - 'h1 ;//1 page is sent completely here.
end
WAIT_TWB: begin
/*Wait for tWB time period*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[1]        = 1'b0 ;
    if(delay_count < 1) begin // TODO parameterize hardcoded right nw
        lv_onfi[2]  = 1'b1;
        lv_onfi[0]  = 1'b1;
        delay_count <= delay_count + 1;
    end
    else if(delay_count == `TWB_COUNT) begin
        delay_count <= 0;
        lv_onfi[2] = 0;
        lv_onfi[0]  = 1'b0;
        program_state <= WAIT_D5;
    end
    else begin
        delay_count <= delay_count + 1;
        lv_onfi[2] = 0;
        lv_onfi[0]  = 1'b0;
    end
    wr_onfi_signal  <= lv_onfi;
end
WAIT_D5: begin
/*For tCLH we spend one cycle here.*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    lv_onfi[1]        = 1'b0 ;
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal    <= lv_onfi;
    /*In case of multi=plane n cache op need to take care of tCBSY and tDBSY timing*/
    if(present_w_state == PROGRAM_PAGE_MULTI_PLANE || present_w_state == PROGRAM_PAGE_CACHE)
    program_state           <= WAIT_STATUS_PLANE1 ;
    else 
    program_state           <= DUMMY ;
end
WAIT_STATUS_PLANE1: begin
/*Wait fot 2*tCCS time period.Because tDBSY and tCBSY values will be unkown from the basic timing
information.Since tCCS is the highest value, we can use 2 times as the value of tCCS.This had 
to be done because 70h and 78h operations does not work during the interval tDBSY and tCBSY 
in the model The ready busy pin cannot be checked for busy status because, in case of a genuine 
program , then we would end up waiting fore-ever till the program finishes */
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    lv_onfi[1]        = 1'b0 ;
    lv_onfi[0]        = 1'b0 ;
    if(delay_count == `TCCS_COUNT*2) begin
        delay_count <= 'h0;
        program_state  <= DUMMY ;
    end
    else
        delay_count <= delay_count + 'h1;
end
WAIT_D6: begin
    /*Spend time here ,since after final page cache reg status is not getting read from model.*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    lv_onfi[1]        = 1'b0 ;
    lv_onfi[0]        = 1'b0 ;
    if(delay_count == `TCCS_COUNT) begin
        delay_count <= 'h0;
        program_state       <= WAIT_D7 ;
    end
    else
        delay_count <= delay_count + 'h1;
    wr_onfi_signal          <= lv_onfi;
end
WAIT_D7: begin
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    lv_onfi[1]        = 1'b0 ;
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal    <= lv_onfi;
    if(wr_ready_busy_n[chip_sel] == 1'b0) 
        program_state       <= WAIT_D7 ;
    else 
        program_state  <= ASK_STATUS1;
end
default: begin
    /*Ideally control wont be here*/
    program_state      <= ASK_STATUS1 ;
end
endcase
endrule

rule rl_pgrm_commands_START_DATA(lv_wr_state_machine && program_state == START_DATA);
/*Need to send 8 bits of data to flash everytime from the actual WDC bit data*/
    Bit#(6)  lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[0]     = 1'b1 ;
      
    if(sector_byte_cnt == 'd512) begin//Here need to send the ECC data until spare bytes.
        if(spare_cnt == `SECTOR_SPARE_SIZE-1) begin
           if(q_data_count_g == `PAGE_LENGTH) begin
                   program_state   <= WAIT_D4;
                   enable_dqs      <=   0;
                   q_data_count_g  <= 'h0;
                   ff_data_w_fifo.clear ; 
                   spare_cnt       <= 'h0;
                   sector_byte_cnt <= 'h0;
            end
            else begin
                spare_cnt       <= 0;
                sector_byte_cnt <= 0;
            end
        end
        else begin
            spare_cnt  <=  spare_cnt + 'h1;
        end
        if(q_data_count_g == `PAGE_LENGTH - 2 || q_data_count_g == `PAGE_LENGTH - 1) begin
            lv_onfi[2]     = 1'b0;
            lv_onfi[1]     = 1'b0;
        end
        else begin
            lv_onfi[2]     = 1'b1 ;
            lv_onfi[1]     = 1'b1 ;
        end
    end
    else begin
        lv_onfi[2]     = 1'b1 ;
        lv_onfi[1]     = 1'b1 ;
        sector_byte_cnt        <=  sector_byte_cnt + 'h1 ;
        let data_to_flash       =  ff_data_w_fifo.first ;
        Bit#(TLog#(`WDC)) index = {'h0,byte_count} ;
        rg_data_to_flash[chip_sel]  <= data_to_flash[((index+'h1)*8 - 'h1):(index*8)] ;
//        $display($stime(),"Nandflashcontroller: data from flash to ecc");
        Bit#(TAdd#(TLog#(`WDC),1)) comp = {'h0,byte_count} ;
        if(q_data_count_g == `PAGE_LENGTH && comp == ((`WDC/8)-1)) begin
            byte_count <= 0;
        end
        else begin 
            if(comp == ((`WDC/8)-1)) begin
                q_data_count_g <= q_data_count_g + 'h1 ;
                ff_data_w_fifo.deq ;
                byte_count <= 0;
            end
            else begin
                byte_count <= byte_count + 'h1;
            end
        end
    end
    wr_onfi_signal  <= lv_onfi;
endrule

rule rl_pgrm_commands_DUMMY(lv_wr_state_machine && program_state == DUMMY);
    $display($stime(),"DUMMY");    
    Bit#(6)  lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[2]     = 1'b0 ;
    lv_onfi[1]     = 1'b0 ;
    lv_onfi[0]     = 1'b0 ;

    if(program_failed == 1'b1) begin
        status_count     <= 'h0 ;
        status_done      <= 'h0 ;
        local_length_w   <= 'h0 ;
        prev_w_state     <= IDLE ;
        present_w_state  <= IDLE ;
        next_w_state     <= IDLE ;
        data_w_fifo_free <= 1'b1 ;
        start_program    <= 1'b0 ;
        ff_data_w_fifo.clear ;
        rg_write_fail  <= 1'b1 ;
        pages2b_written <= 'h0 ;
    end
    else if((pages2b_written == 'h1 || pages2b_written == 'h2) &&
        present_w_state != PROGRAM_PAGE_MULTI_PLANE) begin
//Keep copy of present addresses.This is used to check the pgrm pass/fail for last but one LUN.
        a_cycl3_buff1   <= addr_cycl3;
        a_cycl4_buff1   <= addr_cycl4;
        a_cycl5_buff1   <= addr_cycl5;
        start_program    <= 1'b0 ;
        data_w_fifo_free <= 1'b1 ; //Free the Write FIFO
        prev_w_state     <= present_w_state ;
        program_state    <= ASK_STATUS1 ;
        last_status_r    <= 1'b0 ;
        status_done      <= 'h0 ;
        if(present_w_state == next_w_state)//This will happen once operations are finished on 1 LUN.
            present_w_state <= IDLE ;
        else
            present_w_state <= next_w_state ;
    end           
    /*Clear the length value if it is the last write of the request*/
    else if(local_length_w == 'h1 && pages2b_written == 'h0) begin//Indicates last page program    
        local_length_w <= 'h0 ;
        status_count   <= status_count - 'h1 ;
        program_state <= WAIT_D6 ;
        last_status_r  <= 1'b1 ;
        a_cycl3_buff2   <= addr_cycl3;
        a_cycl4_buff2   <= addr_cycl4;
        a_cycl5_buff2   <= addr_cycl5;
    end
    else if(status_count == 'h1 && local_length_w == 'h0) begin
        status_count  <= 'h0;
        program_state <= WAIT_D6 ;
        last_status_r <= 1'b1 ;
        a_cycl3_buff1   <= a_cycl3_buff2;
        a_cycl4_buff1   <= a_cycl4_buff2;
        a_cycl5_buff1   <= a_cycl5_buff2;
    end
    else begin
        start_program    <= 1'b0 ;
        data_w_fifo_free <= 1'b1 ; //Free the Write FIFO
        prev_w_state                    <= present_w_state ;
        program_state                   <= START_PROGRAM;
        last_status_r                   <= 1'b0 ;
        status_done                     <= 'h0 ;
        if(present_w_state == next_w_state) //This will happen once operations are finished on 1 LUN.
            present_w_state <= IDLE ;
        else
            present_w_state <= next_w_state ;
        if(local_length_w == 'h0)
            rg_write_success <= 1'h1 ;
    end
    wr_onfi_signal <= lv_onfi;
endrule



////////////////////////////////////////////////////////////////////////////////////////////////
/////////Rules to take addr and length from NVM while read to NVM                               
////////////////////////////////////////////////////////////////////////////////////////////////
rule rl_get_addr_from_nvm_for_read (wr_nand_ce_n == 1'b0 && wr_nand_we_n == 1'b1 && 
wr_nand_re_n == 1'b0 ); 
`ifdef verbose $display("Nandflashcontroller: %d: Rule to take addr and length from nvm",$stime()); `endif
   local_length_r    <= wr_r_length ;  
   next_nvm_addr     <= fn_get_next_nvm_addr(wr_rd_addr_frm_nvm); 
   present_nvm_addr  <= wr_rd_addr_frm_nvm; 
   addr_register     <= fn_map_address(wr_rd_addr_frm_nvm) ;
/*Right now 2 chips are separated based on the MSB address bit. adjacent blocks can also be put in 
two different chips.Depends on FTL.This case FTL gets number of chips as 2.Say block0 in chip0 and 
block1 in chip1 as block0.For this address mapping has to be changed.We will deal this later*/
   chip_sel        <= wr_rd_addr_frm_nvm[`COLUMN_WIDTH+`PAGE_WIDTH+`BLOCK_WIDTH+`PLANE_WIDTH+
   `LUN_WIDTH];//MSB bit selects which chip is to e written into
   col_offset_r    <= wr_rd_addr_frm_nvm[`COLUMN_WIDTH-1:0] ;
   byte_count      <= wr_rd_addr_frm_nvm[`LBPR-1:0];//This is for byte offset within col offset.
//This is to fill zeros in case of offset , so that no problem during enquing in FIFO while reading.
  Bit#(`LBPR) local_byte_count = wr_rd_addr_frm_nvm[`LBPR-1:0] ;
  if(local_byte_count != 'h0)
  	q_fill_zeros <= 1'b1 ;
  	
  if(wr_r_length == 'h1)
  	status_count <= 'h1 ;
  else if(wr_r_length == 'h2) begin
  	if(wr_rd_addr_frm_nvm[`COLUMN_WIDTH] == 'h0) //Even plane.
  		status_count <= 'h1 ;//Need to check status of only one LUN before start of read.
  	else
  		status_count <= 'h2 ;//Need to check status of two different LUNS before start of read.
  end
  else
  	status_count <= 'h2 ;
  new_r_req       <= 1'b1;
  get_next_addr   <= 1'b1;
  read_pending    <= 1'b1;
endrule
	
/////////////////////////////////////////////////////////////////////////////////////////////////
///////////Rule to create state machine to read from target                                      
/////////////////////////////////////////////////////////////////////////////////////////////////
/*Calculate address for next cycle first i.e Map the new address (which was stored in next_address
reg in prev page read)*/
rule rl_get_next_read_addr (data_r_fifo_free == 1'b1 && get_next_addr == 1'b1) ;
`ifdef verbose $display("Nandflashcontroller: %d: State machine to read from target %d",$stime(),status_count); `endif
	if(col_offset_r == 'h0) //Need to reset col add since only first page must have column addr.
	begin
		addr_cycl1 <= 'h0 ;
		addr_cycl2 <= 'h0 ;
	end
	else
	begin
	`ifdef COLUMN_WIDTH_LT_8
		addr_cycl1	 <= {'h0,addr_register[`COLUMN_WIDTH-1:0]};
		addr_cycl2	 <= 'h0 ;
	`elsif COLUMN_WIDTH_E_8 
		addr_cycl1	  <= addr_register[7:0];
		addr_cycl2	  <= 'h0 ;
	`else
		addr_cycl1       <= addr_register[7:0];
		Bit#(TSub#(16,`COLUMN_WIDTH)) fill_z = 'h0 ;
        //`COLUMN_WIDTH cannot be 16.Must be LTE 15
		addr_cycl2       <= {fill_z,addr_register[`COLUMN_WIDTH-1:8]} ;
	`endif
	end
	addr_cycl3			<= addr_register[7+`COLUMN_WIDTH:`COLUMN_WIDTH];
	addr_cycl4	 		<= addr_register[15+`COLUMN_WIDTH:`COLUMN_WIDTH+8];
	addr_cycl5	 		<= addr_register[23+`COLUMN_WIDTH:`COLUMN_WIDTH+16];
    //TODO since plane address is lower bit order of block address
	plane_sel       		<= fn_map_address(present_nvm_addr)[`COLUMN_WIDTH + `PAGE_WIDTH] ;  
	present_nvm_addr		<= next_nvm_addr ;
//Map the new address (which was stored in next_address reg in prev page read)
	addr_register                   <= fn_map_address(next_nvm_addr) ; 
	next_nvm_addr                   <= fn_get_next_nvm_addr(next_nvm_addr);
	get_next_addr                   <= 1'b0 ;
	if(stay_with_decision == 1'b1)
	begin
		stay_with_decision <= 1'b0 ;
		decide_read        <= 1'b0 ;
	end
	else
		decide_read 	   <= 1'b1 ;
endrule

/*Implementing plane first and LUN next states*/
rule rl_read_state_decision (data_r_fifo_free == 1'b1 && present_r_state == IDLE && 
decide_read == 1'b1) ;
`ifdef verbose       $display("Nandflashcontroller: %d: read state decision",$stime());  `endif
	decide_read        <= 1'b0 ;
	case(local_length_r)
		'h0 : 	begin
`ifdef verbose $display("Nandflashcontroller: Read Length parameter is zero"); `endif
			end
		'h1 : 	begin
/*We will not be bothered about cache operations in read, since we are spreading A block in 
4 planes, and reading these 4 pages(as a single block) can be done using concurrent plane read and 
the time needed will be same or better compared to cache op, hence we ignore cache op*/
				if(status_count == 'h1)
				begin
`ifdef verbose  $display("Nandflashcontroller: %d: READ state going to ASK_STATUS1",$stime()); `endif
                    read_state        <= START_READ;
					initial_status_ck <= 1'b1 ;
					status_count      <= 'h0 ;
				end
				else
				begin
					read_state        <= START_READ ;
				end
				present_r_state   <= READ_PAGE ;
				next_r_state      <= IDLE ;
				last_r_req        <= 1'b1 ;
			end
	   default  :	begin
/*IF two or more pages are to be read there are two possibilities. a) The pages fall on different 
LUNs(plane1 of LUN0 , plane0 of LUN1 or plane1 of LUN1 and plane0 of LUN0) b) They fall on the 
same LUN */
				if(plane_sel == 1'b0)
				begin
					present_r_state <= READ_PAGE_MULTI_PLANE ;
					next_r_state    <= READ_PAGE ;
					if(local_length_r == 'h2)
					begin
						last_r_req      <= 1'b1 ;
					end
					else
					begin
						last_r_req      <= 1'b0 ;
						local_length_r  <= local_length_r - 'h2 ;
					end
				end
				else
				begin
					present_r_state   <= READ_PAGE ;
					next_r_state      <= IDLE ;
					local_length_r    <= local_length_r - 'h1 ;
				end
				if(status_count != 'h0)
				begin
					read_state        <= START_READ;
					initial_status_ck <= 1'b1 ;
					status_count      <= status_count - 'h1 ;
				end
				else
					read_state        <= START_READ ;
			end
	endcase
endrule
	
//////////////////////////////////////////////////////////////////////////////////////////////////
///////////Rule to READ///////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
rule rl_read_commands ((present_r_state == READ_PAGE || present_r_state ==
    READ_PAGE_MULTI_PLANE) && get_next_addr == 1'b0 && data_r_fifo_free == 1'b1 &&
    wr_ready_busy_n[chip_sel] == 1'b1);
//        `ifdef verbose        $display("Nandflashcontroller: %d: Entering into read state machine",$stime()); `endif
case(read_state)
START_READ   : 	begin
/*Send 00h comand series from here*/
    wr_onfi_signal             <= 6'b011101;
    rg_data_to_flash[chip_sel] <= 'h00 ;
    read_state                 <= WAIT_D1 ;
end
WAIT_D1      :  begin
/*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count > 0 && d_sync_count < 5) begin
        d_sync_count <= d_sync_count + 1;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count   <= 0;
        lv_onfi[0]     =  0;
        lv_onfi[2]     =  0;
        read_state     <= C_ADDR ;
    end
    wr_onfi_signal          <= lv_onfi;
end
WAIT_NEW_3  :  begin
/*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            d_sync_count    <= d_sync_count + 1;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            read_state   <= C_ADDR1 ;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else if(delay_count == 'h2)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            d_sync_count <= d_sync_count + 1;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            read_state <= P_ADDR ;
        end
    else if(delay_count == 'h3) 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            d_sync_count <= d_sync_count + 1;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            read_state <= B_ADDR ;
        end
    else  
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            d_sync_count <= d_sync_count + 1;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            read_state <= L_ADDR ;
            delay_count   <='h0; 
        end
    wr_onfi_signal          <= lv_onfi;
    end
C_ADDR : begin
    wr_onfi_signal             <= 6'b011011;
    rg_data_to_flash[chip_sel] <= addr_cycl1 ;
    read_state                 <= WAIT_NEW_3 ;
    delay_count                <= 1;
end
C_ADDR1 : begin
    wr_onfi_signal             <= 6'b011011;    
    rg_data_to_flash[chip_sel] <= addr_cycl2;
    read_state                 <= WAIT_NEW_3 ;
    delay_count                <= 2;
end
P_ADDR : begin
    /*Send page address for read*/
    wr_onfi_signal          <= 6'b011011;
    rg_data_to_flash[chip_sel]        <= addr_cycl3 ;
    read_state              <= WAIT_NEW_3 ;
    delay_count             <= 3; 
end
B_ADDR	     : 	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state B_ADDR",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
	rg_data_to_flash[chip_sel]        <= addr_cycl4 ;
	read_state              <= WAIT_NEW_3 ;
    delay_count             <= 4;
	end 
L_ADDR	     : 	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state L_ADDR",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
	rg_data_to_flash[chip_sel]        <= addr_cycl5 ;
	read_state              <= WAIT_D2 ;
	end
WAIT_D2     :  begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_D2",$stime()); `endif
/*For tALH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[2]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[1]   = 1;
    end
    else if(d_sync_count > 0 && d_sync_count < 5) begin
        d_sync_count <= d_sync_count + 1;
        lv_onfi[0] = 0;
        lv_onfi[1] = 0;
    end
    else begin
        d_sync_count   <= 0;
        lv_onfi[0]     =  0;
        lv_onfi[1]     =  0;
        read_state     <= END_COMMAND;
    end
    wr_onfi_signal          <= lv_onfi;
    end 
END_COMMAND : 	begin
`ifdef verbose  $display("Nandflashcontroller: %d: READ state END_COMMAND",$stime()); `endif
    wr_onfi_signal          <= 6'b011101;
    col_offset_r	       <= 'h0 ; //Clear the col offset, since it is valid only for first page.
    buf_col_offset_r        <= col_offset_r ;//Keep copy of offset for the FIFO 
    if(present_r_state == READ_PAGE_MULTI_PLANE) begin
       rg_data_to_flash[chip_sel]	 <= 'h32 ;
       read_state	       <= WAIT_TWB1 ;
       if(multi_plane_r_pend_1 == 1'b0) begin
           multi_plane_r_pend_1 <= 1'b1 ;
           mplane_cycl3_buff1 <= addr_cycl3;
           mplane_cycl4_buff1 <= addr_cycl4;
           mplane_cycl5_buff1 <= addr_cycl5;
//If before multi-plane op there is an individual read op raise flag
           if(page_r_pend_1 == 1'b1) begin
               multi_plane_after_page <= 1'b1 ;
           end
           else begin
               multi_plane_after_page <= 1'b0 ;
           end
       end
       else begin
           multi_plane_r_pend_2 <= 1'b1 ;
           mplane_cycl3_buff2 <= addr_cycl3;
           mplane_cycl4_buff2 <= addr_cycl4;
           mplane_cycl5_buff2 <= addr_cycl5;
       end
    end
    else begin  //READ_PAGE command
        rg_data_to_flash[chip_sel]	 <= 'h30 ;
        if(last_r_req == 1'b1) begin
    //If it is the last page,then in next cycle we need to go to READ_MODE
           read_state      <= WAIT_D7 ;
           if(new_r_req == 1'b1) begin//There is only one page/multi-plane one page read request.
               page_r_pend_1   <= 1'b1 ;
               a_cycl3_buff1   <= addr_cycl3;
               a_cycl4_buff1   <= addr_cycl4;
               a_cycl5_buff1   <= addr_cycl5;
           end
           else begin
               page_r_pend_2   <= 1'b1 ;
               a_cycl3_buff2   <= addr_cycl3;
               a_cycl4_buff2   <= addr_cycl4;
               a_cycl5_buff2   <= addr_cycl5;
           end
        end
        else if(new_r_req == 1'b1) begin
//For the first READ_PAGE command don't go to READ_MODE during next cycle.
           read_state      <= WAIT_TWB1;
           page_r_pend_1   <= 1'b1 ;
           a_cycl3_buff1   <= addr_cycl3;
           a_cycl4_buff1   <= addr_cycl4;
           a_cycl5_buff1   <= addr_cycl5;
        end
        else begin
           read_state      <= WAIT_D7 ;
           page_r_pend_2   <= 1'b1 ;
           a_cycl3_buff2   <= addr_cycl3;
           a_cycl4_buff2   <= addr_cycl4;
           a_cycl5_buff2   <= addr_cycl5;
        end
    end
    end
WAIT_D7     :  begin
/*For tCLH we spend one cycle here*/
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_D7",$stime()); `endif
    Bit#(6) lv_onfi;
    lv_onfi[5] = 0;
    lv_onfi[4] = 1;
    lv_onfi[3] = 1;
    lv_onfi[1] = 0;

    if(delay_count == 0) begin
        lv_onfi[0] = 1;
        lv_onfi[2] = 1;
        delay_count <= 1;
    end
    else if(delay_count == `TWB_TR_COUNT) begin
        delay_count     <= 0;
        lv_onfi[0]      = 0;
        lv_onfi[2]      = 0;
        read_state      <= WAIT_TWB ;
    end
    else begin
        delay_count <= delay_count + 1;
        lv_onfi[0]  = 0;
        lv_onfi[2] = 0;
    end
    wr_onfi_signal  <= lv_onfi;
end
WAIT_TWB     :   begin
/*Wait for tWB time period*/
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_TWB",$stime()); `endif
    wr_onfi_signal          <=6'b011000;
    if(delay_count >= `TWB_COUNT_RD)
    begin
        delay_count <= 'h0;
        read_state       <= ASK_STATUS1 ;
    end
    else
        delay_count <= delay_count + 'h1;
    end
ASK_STATUS1 : 	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state ASK_STATUS1",$stime()); `endif
   	/*Check status of the LUN*/
    wr_onfi_signal          <= 6'b011101;
	rg_data_to_flash[chip_sel]        <= 'h78 ;
	read_state              <= WAIT_D4 ;
   	end 
WAIT_D4     :  begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_D4",$stime()); `endif                            
/*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count > 0 && d_sync_count < 5) begin
        d_sync_count <= d_sync_count + 1;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count    <= 0;
        lv_onfi[0]      = 0;
        lv_onfi[2]      = 0;
        read_state   <= P_ADDR_S ;
    end
    wr_onfi_signal    <= lv_onfi;
    end 
WAIT_NEW_1  :  begin
/*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            d_sync_count    <= d_sync_count + 1;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            read_state   <= B_ADDR_S ;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count > 0 && d_sync_count < 5) begin
            d_sync_count <= d_sync_count + 1;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            read_state <= L_ADDR_S ;
            delay_count   <='h0; 
        end
    wr_onfi_signal <= lv_onfi;
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_NEW_1",$stime()); `endif
end
 P_ADDR_S    :	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state P_ADDR_S",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
    if(initial_status_ck == 1'b1)
        rg_data_to_flash[chip_sel]        <= addr_cycl3 ;
//If multi plane is not preceeded by page op
    else if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b0) 
        rg_data_to_flash[chip_sel]        <= mplane_cycl3_buff1 ;
    else
        rg_data_to_flash[chip_sel]        <= a_cycl3_buff1 ;
    read_state              <= WAIT_NEW_1;
    delay_count             <= 1;
end
B_ADDR_S    : 	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state B_ADDR_S",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
    if(initial_status_ck == 1'b1)
        rg_data_to_flash[chip_sel]        <= addr_cycl4 ;
    else if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b0)
        rg_data_to_flash[chip_sel]        <= mplane_cycl4_buff1 ;
    else
        rg_data_to_flash[chip_sel]        <= a_cycl4_buff1 ;
    read_state              <= WAIT_NEW_1;
    delay_count             <= 2;
    end
L_ADDR_S    : 	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state L_ADDR_S",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
    if(initial_status_ck == 1'b1)
        rg_data_to_flash[chip_sel]        <= addr_cycl5 ;
    else if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b0)
        rg_data_to_flash[chip_sel]        <= mplane_cycl5_buff1 ;
    else
        rg_data_to_flash[chip_sel]        <= a_cycl5_buff1 ;
    read_state              <= WAIT_RE_WE1 ;
    end
WAIT_RE_WE1  :  begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_RE_WE1",$stime()); `endif
  	/*Wait for tWHR time period*/
    Bit#(6) lv_onfi;
  	lv_onfi[5]              = 1'b0 ;
  	lv_onfi[4]              = 1'b1 ;
    enable_dqs              <= 0;
    if(delay_count  == 0) begin
         lv_onfi[3]      = 1'b1 ;
         lv_onfi[2]      = 1'b0 ;
         lv_onfi[1]      = 1'b1; //rg_onfi_ale         <= 1'b1;
         lv_onfi[0]      = 1'b1;
         delay_count     <= delay_count + 1;
    end
  	else if(delay_count == `TWHR_COUNT)
  	begin
  		delay_count  <= 'h0;
        lv_onfi[3]   = 1'b1 ;
        lv_onfi[2]   = 1'b0 ;
        lv_onfi[1]   = 1'b0; //rg_onfi_ale      <= 1'b0;
        lv_onfi[0]   = 1'b0;
  		read_state   <= ENABLE_S_READ1 ;
  	end
    else begin
  	  delay_count <= delay_count + 'h1;
      lv_onfi[3]      = 1'b1 ;
      lv_onfi[2]      = 1'b0 ;
      lv_onfi[1]      = 1'b0;  //rg_onfi_ale <= 1'b0;
      lv_onfi[0]      = 1'b0;
    end
    wr_onfi_signal  <= lv_onfi;
  	end
ENABLE_S_READ1 :  begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state ENABLE_S_READ1",$stime()); `endif
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[0]        = 1'b0 ;
//    rg_en_rd_sync_data      <= 1;
    if(delay_count < 2) begin 
        lv_onfi[3]        = 1'b0 ;
        lv_onfi[2]        = 1'b1 ;
        lv_onfi[1]        = 1'b1 ; 
        delay_count       <= delay_count + 1;
    end
    else begin 
        lv_onfi[3]        = 1'b0 ;
        lv_onfi[2]        = 1'b0 ;
        lv_onfi[1]        = 1'b0 ; 
        read_state        <= WAIT1 ;
        delay_count       <= 0;
    end
    wr_onfi_signal          <= lv_onfi;
end
WAIT1:   begin
 `ifdef verbose         $display("Nandflashcontroller: %d: READ state WAIT1",$stime()); `endif
    wr_onfi_signal          <= 6'b010000;
//    rg_en_rd_sync_data      <= 0;
    if(delay_count == 2) begin 
        delay_count       <= 0;
        sreg              <=  unpack(chip_sel) ? wr_null_data1 : wr_null_data0 ;
        read_state        <= WAIT_FOR_tRHW ;
    end
    else 
        delay_count       <= delay_count + 1;
end
WAIT_FOR_tRHW: begin //vis: added for read to write high, timing issue.
//    $display($stime,"WAIT_FOR_tRHW");
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    lv_onfi[1]        = 1'b0 ;
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal    <= lv_onfi;
    if(delay_count == `TRHW_COUNT) begin 
        read_state  <= READ_STATUS1;
        delay_count <= 0;
    end
    else delay_count <= delay_count + 1;
end
READ_STATUS1 :	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state READ_STATUS1",$stime()); `endif
    wr_onfi_signal          <= 6'b011000;
    if(sreg[6] ==1'b1 && sreg[5] == 1'b1) begin
        if(initial_status_ck == 1'b1 && delay_count == `TRHW_COUNT) begin
			read_state        <= START_READ ;
			initial_status_ck <= 1'b0 ;
`ifdef verbose          $display("Nandflashcontroller: %d: case 1 READ state READ_STATUS1",$stime()); `endif
		end
        else if(delay_count == `TRHW_COUNT) begin
			read_state      <= WAIT_RE_WE3;
            delay_count     <= 0;
`ifdef verbose          $display("Nandflashcontroller: %d:case 2 READ state READ_STATUS1",$stime()); `endif
        end
        else begin
`ifdef verbose          $display("Nandflashcontroller: %d:case 3 READ state READ_STATUS1",$stime()); `endif
            delay_count     <= delay_count + 1;
        end
	end
	else
        if(delay_count  == `TRHW_COUNT) begin
			read_state      <= ASK_STATUS1 ;
            delay_count     <= 0;
        end
        else
            delay_count <= delay_count + 1;
	end
START_READ_C :	begin //Send 00 command to start reading
`ifdef verbose          $display("Nandflashcontroller: %d: READ state START_READ_C dreg %x",$stime(), wr_null_data0); `endif
    wr_onfi_signal          <= 6'b010110;
    if(rg_dqs_to_controller[chip_sel] == 1) begin
        read_state       <= CONT_READ;
        dreg             <= unpack(chip_sel)? wr_null_data1 : wr_null_data0;
        delay_count      <= 0;
    end
	end
WAIT_TCCS1   :   begin
/*Wait fot tCCS time period(300ns)(tWHR issue is also solved because this time is big)*/
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_TCCS1",$stime()); `endif
    Bit#(6) lv_onfi ;
	lv_onfi[5]     = 1'b0 ;
	lv_onfi[4]     = 1'b1 ;
//v_onfi[4]     = 1'b1 ;
	lv_onfi[3]     = 1'b1 ;
	lv_onfi[1]     = 1'b0 ;
    if(delay_count == 0) begin
        delay_count     <= delay_count + 1;
        lv_onfi[2]     = 1'b1 ;//rg_onfi_cle     <= 1;
        lv_onfi[0]     = 1'b1;
    end
    else if(delay_count == `TCCS_COUNT) begin
		delay_count <= 'h0;
		read_state  <= CONT_READ;
        lv_onfi[2]     = 1'b0 ; //rg_onfi_cle <= 0;
        lv_onfi[0]     = 1'b0;
	end
    else begin
		delay_count <= delay_count + 'h1;
        lv_onfi[2]     = 1'b0 ; //rg_onfi_cle <= 0;
        lv_onfi[0]     = 1'b0;
    end
    wr_onfi_signal <= lv_onfi;
	end
WAIT3        :  begin //Enable re_n 
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT3",$stime()); `endif
   wr_onfi_signal          <= 6'b010000;
   if(delay_count == 1) begin
       read_state              <= WAIT4 ;
       delay_count             <= 0;
   end
   else
       delay_count             <= delay_count + 1;
end
WAIT4        :  begin //Enable re_n 
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT4",$stime()); `endif
    wr_onfi_signal          <= 6'b011000;
	read_state              <= CONT_READ ;
	dreg                    <= unpack(chip_sel)?wr_null_data1:wr_null_data0 ;
    end
CONT_READ    :	begin
`ifdef verbose $display("Nandflashcontroller: %d: READ state CONT_READ case 0 dreg %x",$stime(),dreg);`endif
    Bit#(6) lv_onfi;
	lv_onfi[5]         = 1'b0 ;
	lv_onfi[4]         = 1'b1 ;
    lv_onfi[3]         = 1'b0 ;
	lv_onfi[2]         = 1'b1 ;
	lv_onfi[1]         = 1'b1 ;
	lv_onfi[0]         = 1'b0 ;
		dreg   <= unpack(chip_sel)?wr_null_data1:wr_null_data0;
        if(sector_byte_cnt == 'd512) begin
//Here need to send the recieve ECC data until spare bytes.Then need to decode it and check for errors.
        if(spare_cnt == `SECTOR_SPARE_SIZE-1) begin
				spare_cnt       <= 'h0 ;
				sector_byte_cnt <= 'h0 ;
                if(q_data_count_t == `PAGE_LENGTH) begin
			    	read_state       <= DUMMY ;
				    q_data_count_t   <= 'h0 ;
				end
			end
			else
	    		spare_cnt  <=  spare_cnt + 'h1 ;
    	end 
        else begin
            sector_byte_cnt      <= sector_byte_cnt + 'h1 ;
            Bit#(TAdd#(TLog#(`WDC),1)) comp = {'h0,byte_count} ;
            Bit#(TLog#(`WDC))         index = {'h0,byte_count} ;
            Bit#(TLog#(`WDC))       z_index = {'h0,zero_index} ;
            if(q_data_count_t == `PAGE_LENGTH && comp == ((`WDC/8)-1)) begin 
                buf_col_offset_r <= 'h0 ; //Reset this since first page read from cache is done.
                //Start putting out data in FIFO from this address.
                q_data_count_g   <= truncate(buf_col_offset_r>>`LBPR);
                if(q_fill_zeros == 'h0) begin
                    ff_data_r_fifo.enq({dreg,data_from_flash[index*8-'h1:0]}) ;
                end
                else begin
                    if(data_reg_loaded == 1'b1) begin
                       ff_data_r_fifo.enq({dreg,data_from_flash[index*8-'h1:(z_index*8)]}<<
                       (z_index*8));
                       data_reg_loaded <= 1'b0 ; //Reset flag
                    end
                    else begin
                        ff_data_r_fifo.enq({dreg,'h0});
                    end

                    q_fill_zeros <= 1'b0 ;
                end
                byte_count    <= 'h0 ;
                first_entry   <= 'h0 ;
                first_byte    <= 'h0;
            end
            else begin
                read_state   <= CONT_READ ; 
                if(comp == ((`WDC/8)-1)) begin
                    q_data_count_t <= q_data_count_t  + 'h1 ;
                    if(q_fill_zeros == 'h0) begin
                        ff_data_r_fifo.enq({dreg,data_from_flash[index*8-'h1:0]}) ;
                    end
                    else begin
                        if(data_reg_loaded == 1'b1) begin
                            ff_data_r_fifo.enq({dreg,data_from_flash[index*8-'h1:(z_index*8)]} 
                            <<(z_index*8));
                            data_reg_loaded <= 1'b0 ; //Reset flag
                        end
                        else
                            ff_data_r_fifo.enq({dreg,'h0});
                        q_fill_zeros <= 1'b0 ;
                    end
                    byte_count <= 'h0;
                    first_byte <= 'h0;
                end
                else begin
                    Bit#(`WDC) inter_data = {'h0,dreg} ;
                    if(first_byte == 1'b0) begin
                        data_from_flash <= inter_data ;
                        first_byte      <= 1'b1 ;
                    end
                    else	
                        data_from_flash <= data_from_flash | (inter_data<<(index*8)) ;
                    byte_count <= byte_count + 'h1 ;
                    if(first_entry == 1'b0) begin
                        if(q_fill_zeros == 1'b1) begin
                            data_reg_loaded <= 1'b1 ;
                            zero_index      <= byte_count ;
                        end
                        first_entry  <= 1'b1 ;
                    end
                end
            end
        end
    wr_onfi_signal  <= lv_onfi;
    end
WAIT_RE_WE3  :  begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_RE_WE3",$stime()); `endif
	/*Wait for tRHW time period*/
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == `TCCS_COUNT) begin
		delay_count <= 'h0;
		read_state       <= SELECT_C_R ;
	end
	else
		delay_count <= delay_count + 'h1;
    end
SELECT_C_R :  begin
/*Send 06h comand series from here*/
`ifdef verbose $display("Nandflashcontroller: %d: READ state SELECT_C_R",$stime()); `endif
    wr_onfi_signal             <= 6'b011101;
    rg_data_to_flash[chip_sel] <= 'h06 ;
    read_state                 <= WAIT_D5 ;
    end
WAIT_D5  :  begin
/*For tCLH we spend one cycle here*/
`ifdef verbose $display("Nandflashcontroller: %d: READ state WAIT_D5",$stime()); `endif
    if(d_sync_count == 0) begin
        d_sync_count   <= 1;
        wr_onfi_signal <= 6'b011101; 
    end
    else if(d_sync_count > 0 && d_sync_count < 4) begin
        d_sync_count   <= d_sync_count + 1;
        wr_onfi_signal <= 6'b011000;
    end
    else begin
        d_sync_count    <= 0;
        wr_onfi_signal <= 6'b011000;
        read_state      <= C_ADDR_1 ;
    end
    end 
WAIT_NEW_2  :  begin
/*For tCLH we spend one cycle here*/
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            wr_onfi_signal      <= 6'b011011;
            d_sync_count        <= d_sync_count + 1;
        end
        else if(d_sync_count > 0 && d_sync_count < 4) begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= d_sync_count + 1;
        end
        else begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= 0;
            read_state          <= C_ADDR_2 ;
        end
    else if (delay_count =='h2) 
        if(d_sync_count == 0) begin
            wr_onfi_signal      <= 6'b011011;
            d_sync_count        <= d_sync_count + 1;
        end
        else if(d_sync_count > 0 && d_sync_count < 4) begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= d_sync_count + 1;
        end
        else begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= 0;
            read_state          <= P_ADDR_1 ;
        end
    else if(delay_count == 'h3)
        if(d_sync_count == 0) begin
            wr_onfi_signal      <= 6'b011011;
            d_sync_count        <= d_sync_count + 1;
        end
        else if(d_sync_count > 0 && d_sync_count < 4) begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= d_sync_count + 1;
        end
        else begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= 0;
            read_state       <= B_ADDR_1;
        end
    else if(delay_count == 'h4)
        if(d_sync_count == 0) begin
            wr_onfi_signal      <= 6'b011011;
            d_sync_count        <= d_sync_count + 1;
        end
        else if(d_sync_count > 0 && d_sync_count < 4) begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= d_sync_count + 1;
        end
        else begin
            wr_onfi_signal      <= 6'b011000;
            d_sync_count        <= 0;
            read_state       <= L_ADDR_1;
            delay_count<='h0;
        end
    end
C_ADDR_1     : 	begin
/*Need to send column address here , in case of col offset. But read in our case happens for full 
page after first page in lengthy read, hence col addr is 0*/
`ifdef verbose          $display("Nandflashcontroller: %d: READ state C_ADDR_1",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
    rg_data_to_flash[chip_sel]        <= 'h0 ;
    read_state              <= WAIT_NEW_2 ;
    delay_count             <= 1;
    end
C_ADDR_2     : 	begin
/*Need to send column address here , in case of col offset. But read in our case happens for full 
page after first page in lengthy read, hence col addr is 0*/
`ifdef verbose          $display("Nandflashcontroller: %d: READ state C_ADDR_2",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
    rg_data_to_flash[chip_sel]        <= 'h0 ;
    read_state              <= WAIT_NEW_2 ;
    delay_count             <= 2;
    end
P_ADDR_1     : 	begin
/*Send page address for read*/
//`ifdef verbose          $display("Nandflashcontroller: %d: READ state P_ADDR_1",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
    if(initial_status_ck == 1'b1)
        rg_data_to_flash[chip_sel]        <= addr_cycl3 ;
    else if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b0)
        rg_data_to_flash[chip_sel]        <= mplane_cycl3_buff1 ;
    else
        rg_data_to_flash[chip_sel]        <= a_cycl3_buff1 ;
    read_state              <= WAIT_NEW_2 ; 
    delay_count             <= 3;
    $display($stime(),"Nandflashcontroller: : P_ADDR_1 a_cycl3_buff1 %x", a_cycl3_buff1);
    end
B_ADDR_1     : 	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state B_ADDR_1",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
    if(initial_status_ck == 1'b1)
        rg_data_to_flash[chip_sel]        <= addr_cycl4 ;
    else if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b0)
        rg_data_to_flash[chip_sel]        <= mplane_cycl4_buff1 ;
    else
        rg_data_to_flash[chip_sel]        <= a_cycl4_buff1 ;
    read_state              <= WAIT_NEW_2 ;
    delay_count             <= 4;
    $display($stime(),"Nandflashcontroller: : B_ADDR_1 a_cycl4_buff1 %x ", a_cycl4_buff1);
    end
L_ADDR_1     : 	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state L_ADDR_1",$stime()); `endif
    wr_onfi_signal          <= 6'b011011;
    if(initial_status_ck == 1'b1)
        rg_data_to_flash[chip_sel]        <= addr_cycl5 ;
    else if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b0)
        rg_data_to_flash[chip_sel]        <= mplane_cycl5_buff1 ;
    else
        rg_data_to_flash[chip_sel]        <= a_cycl5_buff1 ;
	read_state              <= WAIT_D6 ;
	$display($stime(),"Nandflashcontroller: : L_ADDR_1 a_cycl5_buff1 %x ", a_cycl5_buff1);
 	end
WAIT_D6     :  begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_D6",$stime()); `endif
/*For tALH we spend one cycle here*/
    if(d_sync_count == 0) begin
        d_sync_count   <= 1;
        wr_onfi_signal  <= 6'b011011;
    end
    else if(d_sync_count > 0 && d_sync_count < 5) begin
        d_sync_count   <= d_sync_count + 1;
        wr_onfi_signal     <= 6'b011000;
    end
    else begin
        d_sync_count    <= 0;
        wr_onfi_signal <= 6'b011000;
        read_state      <= END_COMMAND_1 ;
    end
    end 
END_COMMAND_1 : begin
`ifdef verbose $display("Nandflashcontroller: %d: READ state END_COMMAND_1",$stime()); `endif
    wr_onfi_signal          <= 6'b011101;
    rg_data_to_flash[chip_sel]        <= 'hE0 ; //End the command and start getting data.
    read_state              <= WAIT_TCCS2;
    end
WAIT_TCCS2   :   begin
`ifdef verbose $display("Nandflashcontroller: %d: READ state WAIT_TCCS2",$stime()); `endif
	/*Wait fot tCCS time period(300ns)*/
    Bit#(6) lv_onfi;
	lv_onfi[5]     = 1'b0 ;
	lv_onfi[4]     = 1'b1 ;
	lv_onfi[3]     = 1'b0 ;
	lv_onfi[1]     = 1'b0 ;
    if(delay_count == 0) begin
        delay_count <= delay_count + 1;
        lv_onfi[2]     = 1'b1 ; //rg_onfi_cle <= 1;
        lv_onfi[0]     = 1'b1 ;
    end
    else if(delay_count == `TCCS_COUNT) begin
		delay_count <= 'h0;
        lv_onfi[0]  = 1'b0;
        lv_onfi[2]  = 1'b0; //rg_onfi_cle <= 0;
		read_state  <= START_READ_C;
	end
    else begin
        lv_onfi[0]  = 1'b0 ;
        lv_onfi[2]  = 1'b0 ; //rg_onfi_cle <= 0;
		delay_count <= delay_count + 'h1;
    end
    wr_onfi_signal  <= lv_onfi;
	end
WAIT_TWB1     :   begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_TWB1",$stime()); `endif
	//Wait for tWB time period
    Bit#(6) lv_onfi;
	lv_onfi[5]     = 1'b0 ;
	lv_onfi[4]     = 1'b1 ;
	lv_onfi[3]     = 1'b1 ;
	lv_onfi[1]     = 1'b0 ;
	if(delay_count == 0) begin
		delay_count <= delay_count + 1;
		lv_onfi[2]  = 1;
        lv_onfi[0]  = 1'b1 ;
	end
    else if(delay_count == `TWB_COUNT) begin
		delay_count <= 'h0;
		lv_onfi[2]  = 0;
        lv_onfi[0]  = 1'b0 ;
		read_state       <= WAIT_D3 ;
	end
	else begin
		lv_onfi[2]  = 0; 
        lv_onfi[0]  = 1'b0 ;
		delay_count <= delay_count + 'h1;
	end
	wr_onfi_signal <= lv_onfi;
	end
WAIT_D3     :  begin
/*For tCLH we spend one cycle here and this also takes care of tDBSY*/
`ifdef verbose          $display("Nandflashcontroller: %d: READ state WAIT_D3",$stime()); `endif
    wr_onfi_signal          <= 6'b011000;
	if(present_r_state == READ_PAGE_MULTI_PLANE && wr_ready_busy_n[chip_sel] == 1'b0)
		read_state <= WAIT_STATUS_PLANE1 ;
	else
		read_state <= DUMMY ;
	end 
WAIT_STATUS_PLANE1   :   begin
`ifdef verbose $display("Nandflashcontroller: %d: READ state WAIT_STATUS_PLANE1",$stime()); `endif
/*Wait fot 2*tCCS time period.Because tDBSY and tCBSY values will be unkown from the basic timing 
information.Since tCCS is the highest value, we can use 2 times as the value of tCCS.This had to be
done because 70h and 78h operations does not work during the interval tDBSY and tCBSY in the model.
The ready busy pin cannot be checked for busy status because, in case of a genuine program , then 
we would end up waiting fore-ever till the program finishes */
    wr_onfi_signal          <= 6'b011100;
    if(delay_count == `TCCS_COUNT*2) begin
		delay_count <= 'h0;
		//read_state  <= WAIT_D3 ;
		read_state  <= DUMMY ;
	end
	else
		delay_count <= delay_count + 'h1;
	end
DUMMY	     :	begin
`ifdef verbose          $display("Nandflashcontroller: %d: READ state DUMMY",$stime()); `endif
    wr_onfi_signal          <= 6'b011000;
    col_offset_r            <= 'h0 ;
    buf_col_offset_r        <= col_offset_r ;
    /*Assign proper states for next cycle*/
    if(present_r_state == READ_PAGE_MULTI_PLANE) begin
    	get_next_addr     <= 1'b1 ;//Need to evaluate next address and keep it ready.
    	present_r_state   <= next_r_state ;
    	stay_with_decision <= 1'b1 ;
    	read_state        <= START_READ ;
    end
    else begin //READ PAGE case
    	if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b0 &&(last_r_req == 1'b1 || 
            multi_plane_r_pend_2 == 1'b1)) begin
    		multi_plane_r_pend_1 <= 1'b0 ;//Done with this read.
    		read_state           <= WAIT_RE_WE3 ; //SELECT CACHE REGISTER. since one cache in one 
            page_r_pend_1        <= 1'b0;        //  plane is already read out.
    		rg_interrupt         <= 1'b1;//Since it is Dreg will reset in next cycle.Hence a pulse.
    		data_r_fifo_free     <= 1'b0;//Make the read FIFO busy.
    	end
        else if(multi_plane_r_pend_1 == 1'b0 && multi_plane_after_page == 1'b0 && 
            multi_plane_r_pend_2 == 1'b1 && last_r_req == 1'b0) begin
    		multi_plane_r_pend_2 <= 1'b0 ;//Transfer all information to _1 temp variables.
    		multi_plane_r_pend_1 <= 1'b1 ;
    		mplane_cycl3_buff1 <= mplane_cycl3_buff2 ;
    		mplane_cycl4_buff1 <= mplane_cycl4_buff2 ;
    		mplane_cycl5_buff1 <= mplane_cycl5_buff2 ;
    		page_r_pend_2      <= 1'b0 ;
    		page_r_pend_1      <= 1'b1 ;
    		a_cycl3_buff1      <= a_cycl3_buff2 ;
    		a_cycl4_buff1      <= a_cycl4_buff2 ;
    		a_cycl5_buff1      <= a_cycl5_buff2 ;
    		present_r_state    <= IDLE ;
    		read_state         <= START_READ ;
    		get_next_addr      <= 1'b1 ;
    		rg_interrupt       <= 1'b1 ; //Since it is Dreg will reset in next cycle.Hence a pulse.
    		data_r_fifo_free   <= 1'b0 ; //Make the read FIFO busy.
    		$display($stime(),"Nandflashcontroller: : read page case 2 addr is stored m and addr_buf");							
    	end
    	else if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b1 
            && last_r_req == 1'b0) begin
    		multi_plane_after_page <= 1'b0 ;
    		page_r_pend_2          <= 1'b0 ;
    		page_r_pend_1          <= 1'b1 ;
    		a_cycl3_buff1          <= a_cycl3_buff2 ;
    		a_cycl4_buff1          <= a_cycl4_buff2 ;
    		a_cycl5_buff1          <= a_cycl5_buff2 ;
    		present_r_state        <= IDLE ;
    		read_state             <= START_READ ;
    		get_next_addr          <= 1'b1 ;
    		rg_interrupt           <= 1'b1 ; //Since it is Dreg will reset in next cycle.Hence a pulse.
    		data_r_fifo_free       <= 1'b0 ; //Make the read FIFO busy.
    		$display($stime(),"Nandflashcontroller: : read page case 3 addr is stored addr_buf");							
    	end
        else if(multi_plane_r_pend_1 == 1'b1 && multi_plane_after_page == 1'b1) begin
    		read_state             <= ASK_STATUS1 ; //Start polling actual multi-plane op.
    		multi_plane_after_page <= 1'b0 ; 
    		rg_interrupt           <= 1'b1 ; //Since it is Dreg will reset in next cycle.Hence a pulse.
    		data_r_fifo_free       <= 1'b0 ; //Make the read FIFO busy.
            if(page_r_pend_2 == 1'b1) begin//1st page is already read when here.
    			page_r_pend_2 <= 1'b0 ;
    			a_cycl3_buff1 <= a_cycl3_buff2 ;
    			a_cycl4_buff1 <= a_cycl4_buff2 ;
    			a_cycl5_buff1 <= a_cycl5_buff2 ;
    		$display($stime(),"Nandflashcontroller: : read page case 4 addr is stored addr_buf");							
    		end
    	end
        else if(last_r_req == 1'b1 && new_r_req == 1'b1) begin//Corresponds to one page request
    		last_r_req      <= 1'b0 ;
    		flag_end_read   <= 1'b1 ;
    		new_r_req       <= 1'b0 ;
    		page_r_pend_1   <= 1'b0 ;
    		present_r_state <= IDLE ;
    		read_state      <= START_READ ;
    		rg_interrupt    <= 1'b1 ; //Since it is Dreg will reset in next cycle.Hence a pulse.
    		data_r_fifo_free	  <= 1'b0 ; //Make the read FIFO busy.
    	end
        else if(new_r_req == 1'b1) begin
    		new_r_req       <= 1'b0 ;
    		get_next_addr   <= 1'b1 ;//Need to evaluate next address and keep it ready.
    		present_r_state <= IDLE ;
    		read_state      <= START_READ ;
    	end
        else begin // if(last_r_req == 1'b1 && new_r_req == 1'b0)
            if(page_r_pend_2 == 1'b1) begin //1st page is already read when here.
                if(multi_plane_r_pend_2 == 1'b1) begin
    				multi_plane_r_pend_1 <= 1'b1 ;
    				multi_plane_r_pend_2 <= 1'b0 ;
    				mplane_cycl3_buff1 <= mplane_cycl3_buff2 ;
    				mplane_cycl4_buff1 <= mplane_cycl4_buff2 ;
                    mplane_cycl5_buff1 <= mplane_cycl5_buff2 ;
                end
                page_r_pend_2 <= 1'b0 ;
                a_cycl3_buff1 <= a_cycl3_buff2 ;
                a_cycl4_buff1 <= a_cycl4_buff2 ;
                a_cycl5_buff1 <= a_cycl5_buff2 ;
                read_state    <= ASK_STATUS1;
            end
            else begin
                last_r_req      <= 1'b0 ;
                flag_end_read   <= 1'b1 ;
                present_r_state <= IDLE ;
            end
            rg_interrupt  <= 1'b1 ; //Since it is Dreg will reset in next cycle.Hence a pulse.
            data_r_fifo_free                 <= 1'b0 ; //Make the read FIFO busy.
        end
    end
    end
endcase
endrule	

//rule rl_send_data_to_nvm (data_r_fifo_free  == 1'b0 && ff_data_r_fifo.notEmpty == True);
//	rg_data_to_nvm	<= tagged Valid (ff_data_r_fifo.first) ;
//`ifdef verbose $display("Nandflashcontroller: %d: rg_data_to_nvm %h",$stime(),ff_data_r_fifo.first); `endif
//	ff_data_r_fifo.deq ;
//	if(q_data_count_g < `PAGE_LENGTH)
//		q_data_count_g <= q_data_count_g + 'h1 ;
//    else begin
//		q_data_count_g    <= 'h0 ;
//		data_r_fifo_free  <= 1'b1 ; //Make the write FIFO free.
//		//This is to help ready/busy decision making
//        if(flag_end_read   == 1'b1) begin
//			flag_end_read   <= 1'b0 ;
//			read_pending    <= 1'b0 ;
//		end
//	end
//endrule

/////////////////////////////////////////////////////////////////////////////////////////////////	
////////// Rule for Block erase                      
/////////////////////////////////////////////////////////////////////////////////////////////////
rule rl_block_erase_initiate_get_address (wr_nand_ce_n == 1'b0 && wr_nand_erase == 1'b1) ;
$display($stime(),"Nandflashcontroller: : Control in erase block start");
	chip_sel        <= wr_address_from_nvm[`COLUMN_WIDTH+`PAGE_WIDTH+`BLOCK_WIDTH+`PLANE_WIDTH
    +`LUN_WIDTH];//most MSB bit selects which chip is to e written into
	addr_register   <= fn_map_address(wr_address_from_nvm) ;
	cal_block_addr  <= 1'b1 ;
endrule
	
rule rl_calculate_block_address (cal_block_addr == 1'b1) ;
	Bit#(`PAGE_WIDTH) page_addr = 'h0 ;
	Bit#(2) block_map      = addr_register[`COLUMN_WIDTH+`PAGE_WIDTH+1:`COLUMN_WIDTH+`PAGE_WIDTH];
	Bit#(TSub#(`MAX_ROW_BITS,`PAGE_WIDTH)) block_addr_0 = addr_register[`MAX_ROW_BITS+
    `COLUMN_WIDTH-1:`COLUMN_WIDTH+`PAGE_WIDTH];
	Bit#(TSub#(`MAX_ROW_BITS,`PAGE_WIDTH)) block_addr_1 = {~addr_register[`MAX_ROW_BITS+1+
    `COLUMN_WIDTH],addr_register[`MAX_ROW_BITS+`COLUMN_WIDTH-2:`COLUMN_WIDTH+`PAGE_WIDTH]};
/*Each block is being split in 4 planes.Hence we can get an adress for block erase from any of 
these planes.We need to map this recieved address into 4 address in 4 diff planes for multi-plane 
block erase. Also we need to use the erase multi plane command, hence the addresses are taken in 
pairs*/
case(block_map)
	'h0 : 	begin
			Bit#(24) total_addr0 = {'h0,block_addr_0,page_addr};
			alter_addr_cycl3[0] <= total_addr0[7:0] ;
			alter_addr_cycl4[0] <= total_addr0[15:8] ;
			alter_addr_cycl5[0] <= total_addr0[23:16] ;
			Bit#(24) total_addr1 = {'h0,(block_addr_0+'h3),page_addr};
			alter_addr_cycl3[1] <= total_addr1[7:0] ;
			alter_addr_cycl4[1] <= total_addr1[15:8] ;
			alter_addr_cycl5[1] <= total_addr1[23:16] ;
		
			Bit#(24) total_addr2 = {'h0,block_addr_1,page_addr};
			alter_addr_cycl3[2] <= total_addr2[7:0] ;
			alter_addr_cycl4[2] <= total_addr2[15:8] ;
			alter_addr_cycl5[2] <= total_addr2[23:16] ;
			Bit#(24) total_addr3 = {'h0,block_addr_1+'h3,page_addr};
			alter_addr_cycl3[3] <= total_addr3[7:0] ;
			alter_addr_cycl4[3] <= total_addr3[15:8] ;
			alter_addr_cycl5[3] <= total_addr3[23:16] ;
		end
	'h1 :	begin
			Bit#(24) total_addr0 = {'h0,block_addr_0,page_addr};
			alter_addr_cycl3[1] <= total_addr0[7:0] ;
			alter_addr_cycl4[1] <= total_addr0[15:8] ;
			alter_addr_cycl5[1] <= total_addr0[23:16] ;
			Bit#(24) total_addr1 = {'h0,(block_addr_0+'h1),page_addr};
			alter_addr_cycl3[0] <= total_addr1[7:0] ;
			alter_addr_cycl4[0] <= total_addr1[15:8] ;
			alter_addr_cycl5[0] <= total_addr1[23:16] ;
		
			Bit#(24) total_addr2 = {'h0,block_addr_1,page_addr};
			alter_addr_cycl3[3] <= total_addr2[7:0] ;
			alter_addr_cycl4[3] <= total_addr2[15:8] ;
			alter_addr_cycl5[3] <= total_addr2[23:16] ;
			Bit#(24) total_addr3 = {'h0,block_addr_1+'h1,page_addr};
			alter_addr_cycl3[2] <= total_addr3[7:0] ;
			alter_addr_cycl4[2] <= total_addr3[15:8] ;
			alter_addr_cycl5[2] <= total_addr3[23:16] ;
		end
	'h2 :	begin
			Bit#(24) total_addr0 = {'h0,block_addr_0,page_addr};
			alter_addr_cycl3[0] <= total_addr0[7:0] ;
			alter_addr_cycl4[0] <= total_addr0[15:8] ;
			alter_addr_cycl5[0] <= total_addr0[23:16] ;
			Bit#(24) total_addr1 = {'h0,(block_addr_0-'h1),page_addr};
			alter_addr_cycl3[1] <= total_addr1[7:0] ;
			alter_addr_cycl4[1] <= total_addr1[15:8] ;
			alter_addr_cycl5[1] <= total_addr1[23:16] ;
		
			Bit#(24) total_addr2 = {'h0,block_addr_1,page_addr};
			alter_addr_cycl3[2] <= total_addr2[7:0] ;
			alter_addr_cycl4[2] <= total_addr2[15:8] ;
			alter_addr_cycl5[2] <= total_addr2[23:16] ;
			Bit#(24) total_addr3 = {'h0,block_addr_1-'h1,page_addr};
			alter_addr_cycl3[3] <= total_addr3[7:0] ;
			alter_addr_cycl4[3] <= total_addr3[15:8] ;
			alter_addr_cycl5[3] <= total_addr3[23:16] ;
		end
    default :	begin
    			Bit#(24) total_addr0 = {'h0,block_addr_0,page_addr};
			alter_addr_cycl3[1] <= total_addr0[7:0] ;
			alter_addr_cycl4[1] <= total_addr0[15:8] ;
			alter_addr_cycl5[1] <= total_addr0[23:16] ;
			Bit#(24) total_addr1 = {'h0,(block_addr_0-'h3),page_addr};
			alter_addr_cycl3[0] <= total_addr1[7:0] ;
			alter_addr_cycl4[0] <= total_addr1[15:8] ;
			alter_addr_cycl5[0] <= total_addr1[23:16] ;
		
			Bit#(24) total_addr2 = {'h0,block_addr_1,page_addr};
			alter_addr_cycl3[3] <= total_addr2[7:0] ;
			alter_addr_cycl4[3] <= total_addr2[15:8] ;
			alter_addr_cycl5[3] <= total_addr2[23:16] ;
			Bit#(24) total_addr3 = {'h0,block_addr_1-'h3,page_addr};
			alter_addr_cycl3[2] <= total_addr3[7:0] ;
			alter_addr_cycl4[2] <= total_addr3[15:8] ;
			alter_addr_cycl5[2] <= total_addr3[23:16] ;
    		end
endcase
block_erase_ongoing <= 1'b1 ;
cal_block_addr      <= 1'b0 ;
endrule

rule rl_block_erase_start(block_erase_ongoing == 1'b1 && timing_set == 1 && 
    wr_ready_busy_n[chip_sel] == 1) ;
case(erase_state)
ASK_STATUS1 : 	begin
/*Check status of the LUN*/
    wr_onfi_signal          <= 6'b011101;
    rg_data_to_flash[chip_sel]        <= 'h78 ;
    erase_state             <= WAIT_D1 ;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state ASK_STATUS1"); `endif
    end
WAIT_D1      :  begin
/*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count == 1)begin
        d_sync_count <= 2;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count    <= 0;
        lv_onfi[0]      = 0;
        lv_onfi[2]      = 0;
        erase_state             <= P_ADDR_S;
    end
    wr_onfi_signal    <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_D1"); `endif
    end
WAIT_NEW  :  begin
/*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count    <= 2;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            erase_state   <= B_ADDR_S ;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count <= 2;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            erase_state <= L_ADDR_S ;
            delay_count   <='h0; 
        end
    wr_onfi_signal <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_NEW"); `endif
    end
P_ADDR_S    :	begin
   wr_onfi_signal          <= 6'b011011;
   if(alter_cnt == 'h0)
       rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[0] ;
   else
       rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[2] ;
   delay_count    <= 1;
   erase_state    <= WAIT_NEW ;                  
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state P_ADDR_S"); `endif
    end
B_ADDR_S    : 	begin
   wr_onfi_signal          <= 6'b011011;
   if(alter_cnt == 'h0)
       rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[0] ;
   else
       rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[2] ;
   erase_state           <= WAIT_NEW ;
   delay_count           <= delay_count + 1;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state B_ADDR_S"); `endif
    end
L_ADDR_S    : 	begin
    wr_onfi_signal          <= 6'b011011;
    if(alter_cnt == 'h0)
        rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[0] ;
    else
        rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[2] ;
    erase_state           <= WAIT_RE_WE1;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state L_ADDR_S"); `endif
    end
WAIT_RE_WE1  :  begin
    /*Wait for tWHR time period*/
    Bit#(6) lv_onfi;
  	lv_onfi[5]              = 1'b0 ;
  	lv_onfi[4]              = 1'b1 ;
//  	lv_onfi[4]              = 1'b1 ;
  	lv_onfi[3]              = 1'b1 ;
  	lv_onfi[2]              = 1'b0 ;
    enable_dqs              <= 0;
//  	lv_onfi[0]              = 1'b0 ;
    if(delay_count  == 0) begin
         lv_onfi[1]      = 1'b1; //rg_onfi_ale         <= 1'b1;
         lv_onfi[0]      = 1'b1;
         delay_count     <= delay_count + 1;
    end
  	else if(delay_count == `TWHR_COUNT)
  	begin
  		delay_count  <= 'h0;
        lv_onfi[1]   = 1'b0; //rg_onfi_ale      <= 1'b0;
        lv_onfi[0]   = 1'b0;
  		erase_state   <= ENABLE_S_READ1 ;
  	end
    else begin
  	  delay_count <= delay_count + 'h1;
      lv_onfi[1]  = 1'b0;  //rg_onfi_ale <= 1'b0;
      lv_onfi[0]  = 1'b0;
    end
    wr_onfi_signal  <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_RE_WE1"); `endif
    end
ENABLE_S_READ1 :  begin
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b0 ;
    lv_onfi[2]        = 1'b1 ;
    lv_onfi[1]        = 1'b1 ; 
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal          <= lv_onfi;
    rg_en_rd_sync_data      <= 1;
/*    if(delay_count == 2) begin 
        delay_count       <= 0;
    end
    else 
        delay_count       <= delay_count + 1;*/
        erase_state             <= WAIT1 ;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state ENABLE_S_READ1"); `endif
    end
WAIT1       :   begin
    wr_onfi_signal          <= 6'b011000;
    rg_en_rd_sync_data      <= 0;
/*    if(delay_count == 2) begin 
        delay_count       <= 0;
    end
    else 
        delay_count       <= delay_count + 1;*/
        erase_state          <= WAIT_FOR_tRHW ;
        sreg                 <= unpack(chip_sel) ? wr_null_data1 : wr_null_data0;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT1"); `endif
    end
WAIT_FOR_tRHW : begin // vis: added for read to write high, timing issue.
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == 'h6) begin 
        erase_state   <= READ_STATUS1;
        delay_count <= 0;
    end
    else delay_count <= delay_count + 1;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_FOR_tRHW"); `endif
    end
READ_STATUS1 :	begin
    wr_onfi_signal          <= 6'b011000;
    if(last_status_r == 1'b1) begin//For checking if erase failed or not.
        if(sreg[5]==1'b1) begin 
            block_erase_ongoing <= 'h0 ;
            erase_state  <= ASK_STATUS1 ;
            if(sreg[0] ==1'b1)
                rg_erase_fail       <= 1'b1 ;
            else
                rg_erase_success    <= 1'b1 ;
        end
        else
            erase_state  <= ASK_STATUS1 ; 
    end
    else begin
        if(sreg[6] ==1'b1 && sreg[5] ==1'b1)
            erase_state  <= START_ERASE ;
        else
            erase_state  <= ASK_STATUS1 ; 
    end
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state READ_STATUS1"); `endif
    end
START_ERASE   :	begin
   /*Send 60h command series from here*/
   if(delay_count == 0) begin
       delay_count <= delay_count + 1;
       wr_onfi_signal          <= 6'b011000;
   end
   else if(delay_count == 1) begin   
       wr_onfi_signal          <= 6'b011101;
       rg_data_to_flash[chip_sel]        <= 'h60 ;
       erase_state             <= WAIT_D3 ;
       delay_count             <= 0;
   end
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state START_ERASE"); `endif
   end
WAIT_D3      :  begin
    /*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count == 1)begin
        d_sync_count <= 2;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count    <= 0;
        lv_onfi[0]      = 0;
        lv_onfi[2]      = 0;
        erase_state             <= P_ADDR ;
    end
    wr_onfi_signal    <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_D3"); `endif
    end
WAIT_NEW_1  :  begin
/*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count    <= 2;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            erase_state   <= B_ADDR;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count <= 2;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            erase_state <= L_ADDR;
            delay_count   <='h0; 
        end
    wr_onfi_signal <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_NEW_1"); `endif
    end
 P_ADDR  :  begin
    wr_onfi_signal          <= 6'b011011;
	if(alter_cnt == 'h0)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[0] ;
	else if(alter_cnt == 'h1)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[1] ;
	else if(alter_cnt == 'h2)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[2] ;
	else
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[3] ;
	erase_state             <= WAIT_NEW_1 ; 
    delay_count             <= 1;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state P_ADDR"); `endif
    end
B_ADDR  :  begin
    wr_onfi_signal          <= 6'b011011;
	if(alter_cnt == 'h0)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[0] ;
	else if(alter_cnt == 'h1)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[1] ;
	else if(alter_cnt == 'h2)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[2] ;
	else
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[3] ;
	erase_state             <= WAIT_NEW_1; 
    delay_count             <= delay_count + 1;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state B_ADDR"); `endif
    end
L_ADDR  :  begin
    wr_onfi_signal          <= 6'b011011;
	if(alter_cnt == 'h0)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[0] ;
	else if(alter_cnt == 'h1)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[1] ;
	else if(alter_cnt == 'h2)
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[2] ;
	else
		rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[3] ;
	erase_state             <= WAIT_D4 ; 
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state L_ADDR"); `endif
    end
WAIT_D4      :  begin
/*For tALH we spend one cycle here*/
    if(delay_count == 0) begin
        delay_count     <= 1;
        wr_onfi_signal  <= 6'b011011;
    end
    else if(delay_count == 1) begin
        delay_count <= 2;
        wr_onfi_signal     <= 6'b011000;
    end
    else begin
        erase_state             <= END_COMMAND ;
        wr_onfi_signal     <= 6'b011000;
        delay_count        <= 0;
    end
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_D4"); `endif
    end
END_COMMAND  : 	begin
    wr_onfi_signal          <= 6'b011101;
    if(alter_cnt[0] == 'h0) begin
        rg_data_to_flash[chip_sel]	 <= 'hD1 ;
        erase_state	       <= WAIT_D5 ;
        alter_cnt	       <= alter_cnt + 'h1 ;
    end
    else if(alter_cnt == 'h1) begin
        rg_data_to_flash[chip_sel]	 <= 'hD0 ;
        erase_state	       <= WAIT_TWB1 ;
        alter_cnt	       <= alter_cnt + 'h1 ;
    end
    else begin
        rg_data_to_flash[chip_sel]	 <= 'hD0 ;
        erase_state	       <= WAIT_TWB2 ;
        alter_cnt	       <= 'h0 ;
    end
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state END_COMMAND"); `endif
    end
WAIT_TWB1     :  begin
/*Wait for tWB time period*/
    Bit#(6)  lv_onfi;   
   	lv_onfi[5]          = 1'b0 ;
   	lv_onfi[4]          = 1'b1 ;
   	lv_onfi[3]          = 1'b1 ;
    lv_onfi[1]          = 1'b0 ;
    if(delay_count  == 0) begin
        lv_onfi[2]      = 1'b1; //rg_onfi_cle         <= 1'b1;
        lv_onfi[0]      = 1'b1 ;
        delay_count     <= delay_count + 1;
    end
    else if(delay_count == `TWB_COUNT) begin
       lv_onfi[2]   = 1'b0; //rg_onfi_cle     <= 1'b0;
        lv_onfi[0]          = 1'b0 ;
       delay_count <= 'h0;
       erase_state       <= ASK_STATUS1 ;
   	end
   	else begin
        lv_onfi[2]   = 1'b0; //rg_onfi_cle     <= 1'b0;
        lv_onfi[0]          = 1'b0 ;
   		delay_count <= delay_count + 'h1;
    end
    wr_onfi_signal <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_TWB1"); `endif
    end
WAIT_TWB2     :  begin
/*Wait for tWB time period*/
    Bit#(6)  lv_onfi;   
   	lv_onfi[5]          = 1'b0 ;
   	lv_onfi[4]          = 1'b1 ;
   	lv_onfi[3]          = 1'b1 ;
    lv_onfi[1]          = 1'b0 ;
    if(delay_count  == 0) begin
        lv_onfi[2]      = 1'b1; //rg_onfi_cle         <= 1'b1;
        lv_onfi[0]      = 1'b1 ;
        delay_count     <= delay_count + 1;
    end
    else if(delay_count == `TWB_COUNT) begin
        lv_onfi[0]  = 1'b0 ;
       lv_onfi[2]   = 1'b0; //rg_onfi_cle     <= 1'b0;
       delay_count  <= 'h0;
       erase_state       <= WAIT_D7;
   	end
   	else begin
        lv_onfi[2]   = 1'b0; //rg_onfi_cle     <= 1'b0;
        lv_onfi[0]   = 1'b0 ;
   		delay_count <= delay_count + 'h1;
    end
    wr_onfi_signal <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_TWB2"); `endif
    end
WAIT_D7      :  begin
/*Wait until busy goes high*/
    wr_onfi_signal          <= 6'b011000;
    if(wr_ready_busy_n[chip_sel] == 1'b0)
        erase_state             <= WAIT_D7 ;
    else
        erase_state             <= FINISH_STATUS ;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_D7"); `endif
    end
WAIT_D5      :  begin
/*For tCLH we spend one cycle here*/
    wr_onfi_signal          <= 6'b011101;
    erase_state             <= WAIT_TWB ;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_D5"); `endif
    end
WAIT_TWB     :  begin
	/*Wait for tWB time period*/
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == `TWB_COUNT) begin
		delay_count <= 'h0;
		erase_state       <= WAIT_STATUS_PLANE1 ;
	end
	else
		delay_count <= delay_count + 'h1;
    `ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_TWB"); `endif
    end
WAIT_STATUS_PLANE1   :   begin
/*Wait fot 2*tCCS time period.Because tDBSY and tCBSY values will be unkown from the basic timing 
information.Since tCCS is the highest value, we can use 2 times as the value of tCCS.This had to be
done because 70h and 78h operations does not work during the interval tDBSY and tCBSY in the model.
The ready busy pin cannot be checked for busy status because, in case of a genuine program , then 
we would end up waiting fore-ever till the program finishes */
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == `TCCS_COUNT*2) begin
        delay_count <= 'h0;
        erase_state  <= START_ERASE ;
    end
    else
        delay_count <= delay_count + 'h1;
    end
FINISH_STATUS  :  begin
/*Check status of the LUN*/
    wr_onfi_signal          <= 6'b011101;
    rg_data_to_flash[chip_sel]        <= 'h78 ;
    erase_state             <= WAIT_D6 ;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state FINISH_STATUS"); `endif
    end
WAIT_D6      :  begin
/*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count == 1)begin
        d_sync_count <= 2;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count    <= 0;
        lv_onfi[0]      = 0;
        lv_onfi[2]      = 0;
        erase_state   <= P_ADDR_S1 ;
    end
    wr_onfi_signal    <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_D6"); `endif
    end
WAIT_NEW_3  :  begin
    /*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count    <= 2;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            erase_state   <= B_ADDR_S1;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count <= 2;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            erase_state <= L_ADDR_S1;
            delay_count   <='h0; 
        end
    wr_onfi_signal <= lv_onfi;
`ifdef verbose    $display($stime(),"Nandflashcontroller: **********Erase state WAIT_NEW_3 State***********"); `endif
    end
 P_ADDR_S1    :	begin
    wr_onfi_signal          <= 6'b011011;
    rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[2] ;
    erase_state             <= WAIT_NEW_3;
    delay_count             <= 1;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state P_ADDR_S1"); `endif
    end
 B_ADDR_S1    : 	begin
    wr_onfi_signal          <= 6'b011011;
 	rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[2] ;
 	erase_state           <= WAIT_NEW_3 ;
    delay_count           <= 2;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state B_ADDR_S1"); `endif
    end
L_ADDR_S1    : 	begin
    wr_onfi_signal          <= 6'b011011;
	rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[2] ;
	erase_state           <= WAIT_RE_WE2 ;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state L_ADDR_S1"); `endif
    end
WAIT_RE_WE2  :  begin
   	/*Wait for tWHR time period*/
    Bit#(6) lv_onfi;
  	lv_onfi[5]              = 1'b0 ;
  	lv_onfi[4]              = 1'b1 ;
//  	lv_onfi[4]              = 1'b1 ;
  	lv_onfi[3]              = 1'b1 ;
  	lv_onfi[2]              = 1'b0 ;
    enable_dqs              <= 0;
//  	lv_onfi[0]              = 1'b0 ;
    if(delay_count  == 0) begin
         lv_onfi[1]      = 1'b1; //rg_onfi_ale         <= 1'b1;
         lv_onfi[0]      = 1'b1;
         delay_count     <= delay_count + 1;
    end
  	else if(delay_count == `TWHR_COUNT)
  	begin
  		delay_count  <= 'h0;
        lv_onfi[1]   = 1'b0; //rg_onfi_ale      <= 1'b0;
        lv_onfi[0]   = 1'b0;
  		erase_state   <= ENABLE_S_READ3 ;
  	end
    else begin
  	  delay_count <= delay_count + 'h1;
      lv_onfi[1]  = 1'b0;  //rg_onfi_ale <= 1'b0;
      lv_onfi[0]  = 1'b0;
    end
    wr_onfi_signal  <= lv_onfi;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_RE_WE2"); `endif
    end
ENABLE_S_READ3 :  begin
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b0 ;
    lv_onfi[2]        = 1'b1 ;
    lv_onfi[1]        = 1'b1 ; 
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal          <= lv_onfi;
    rg_en_rd_sync_data      <= 1;
/*    if(delay_count == 2) begin 
        delay_count       <= 0;
    end
    else 
        delay_count       <= delay_count + 1;*/
        erase_state             <= WAIT3 ; 
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state ENABLE_S_READ3"); `endif
    end
WAIT3    :  begin
    wr_onfi_signal          <= 6'b011000;
    rg_en_rd_sync_data      <= 0;
/*    if(delay_count == 2) begin 
        delay_count       <= 0;
    end
    else 
        delay_count       <= delay_count + 1;*/
        erase_state       <= WAIT_FOR_tRHW1 ;
        sreg              <= unpack(chip_sel) ? wr_null_data1 : wr_null_data0;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT3"); `endif
    end
READ_STATUS3 :	begin
    wr_onfi_signal          <= 6'b011000;
    if(sreg[5] ==1'b1) begin
		erase_state         <= ASK_STATUS1 ;
        if(sreg[0] ==1'b1) begin
			rg_erase_fail       <= 1'b1 ;
			block_erase_ongoing <= 'h0 ;
		end
		else
			last_status_r        <= 1'b1 ; 
	end
    else 
		erase_state  <= FINISH_STATUS ; 
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state READ_STATUS3"); `endif
    end
WAIT_FOR_tRHW1 : begin // vis: added for read to write high, timing issue.
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == 'h6) begin 
        erase_state   <= READ_STATUS3;
        delay_count <= 0;
    end
    else delay_count <= delay_count + 1;
`ifdef verbose $display($stime(),"Nandflashcontroller: : Erase state WAIT_FOR_tRHW"); `endif
    end
endcase
endrule
		
///////////////////////////////////////////////////////////////////////////////////////////////////	
//////////Rule for reset                                                                           
///////////////////////////////////////////////////////////////////////////////////////////////////
//Need to wait for VCC power ramp up first.
rule rl_wait_vcc_ramp (reset_wait_flag == 1'b1 && (wr_ready_busy_n[0] == 1 && wr_ready_busy_n[1] == 1)) ;
    if(delay_count == `VCC_DELAY) begin
		delay_count      <= 'h0;
		reset_wait_flag  <= 1'b0 ;
		reset_flag       <= 1'b1 ;
        wr_onfi_signal   <= 6'b001100;
        rg_data_to_flash[0]     <= 'hFF ;
        rg_data_to_flash[1]     <= 'hFF ;
        rg_onfi_we_n     <= 0;
`ifdef verbose $display("Nandflashcontroller: delay vcc done\n"); `endif
	end
    else if(delay_count == `VCC_DELAY - 1 ) begin
        wr_onfi_signal  <= 6'b001100;
        rg_onfi_we_n     <= 0;
        delay_count      <= delay_count + 1;
    end
    else begin
		delay_count <= delay_count + 'h1;
        rg_onfi_we_n        <= 1;
    end
endrule

rule rl_set_we_ce(reset_flag == 1 && (wr_ready_busy_n[0] == 1'b1 || wr_ready_busy_n[1] == 1'b1) &&
    reset_ongoing == 0 && set_we_ce == 0);
    we_flag         <= 1;
    wr_onfi_signal  <= 6'b001101;
    rg_data_to_flash[0]     <= 'hFF ;
    rg_data_to_flash[1]     <= 'hFF ;
    set_we_ce       <= 1;
    rg_onfi_we_n    <= 0;
//    $display($stime(),"Nandflashcontroller: reset case6\n");
endrule
	//Reset has to be sent only when the flash memory is not busy
rule rl_reset (reset_flag == 1'b1 && (wr_ready_busy_n[0] == 1'b1 || wr_ready_busy_n[1] == 1'b1) &&
    we_flag == 1) ; 
    if(reset_applied == 'h1) begin
/*If reset has been applied in prev cycle and the wr_ready_busy_n takes some time to update(3-4 cycles)*/
        if(reset_ongoing == 1'b1) begin 
			reset_ongoing <= 1'b0 ;
			reset_applied <= 'h0 ;
			reset_flag    <= 1'b0 ;
`ifdef verbose   $display($stime(),"Nandflashcontroller: reset is done\n"); `endif
			//Set the column address to first spare byte since after this we need to scan for BBM
			`ifdef COLUMN_WIDTH_LT_8
				addr_cycl1[`COLUMN_WIDTH] <= 1'b1;
			`elsif COLUMN_WIDTH_E_8 
				addr_cycl2[0]	  <= 1'b1;
			`else
				addr_cycl2[`COLUMN_WIDTH-8]   <= 1'b1;
			`endif
		end
        else begin
			reset_flag    <= 1'b1 ;
			//Disable all control signals 
            if(delay_count == 0) begin
                wr_onfi_signal      <= 6'b001101;
                rg_onfi_we_n        <= 0;
                delay_count         <= delay_count + 1;
 //   $display($stime(),"Nandflashcontroller: reset case1\n");
            end
            else if(delay_count == 1) begin
                wr_onfi_signal      <= 6'b111101;
                rg_onfi_we_n        <= 1;
                delay_count         <= delay_count + 1;
            end
            else if(delay_count == 2) begin
                wr_onfi_signal      <= 6'b111101;
                rg_onfi_we_n        <= 1;
                delay_count         <= delay_count + 1;
//    $display($stime(),"Nandflashcontroller: reset case2\n");
            end
            else begin
                wr_onfi_signal      <= 6'b111000;
                rg_onfi_we_n        <= 1;
//    $display($stime(),"Nandflashcontroller: reset case3\n");
            end

		end
	end
    else begin
        wr_onfi_signal      <= 6'b001101;
		rg_data_to_flash[0]     <= 'hFF ;
		rg_data_to_flash[1]     <= 'hFF ;
        reset_applied           <= 'h1 ;
        rg_onfi_we_n            <= 0;
//    $display($stime(),"Nandflashcontroller: reset case4\n");
	end
endrule

rule rl_reset_state_track (reset_flag == 1'b1 && wr_ready_busy_n[0] == 'h0 && 
    wr_ready_busy_n[1] == 'h0) ; //If reset has started and chip is busy.
	//Disable all control signals 
    wr_onfi_signal      <= 6'b111000;
    if(reset_applied == 'h1) begin
		reset_ongoing <= 1'b1 ;
        rg_onfi_we_n        <= 1;
//        $display($stime(),"Nandflashcontroller: reset case11\n");
    end
    else begin
       rg_onfi_we_n        <= 1;
		reset_ongoing       <= 1'b0 ;
//        $display($stime(),"Nandflashcontroller: reset case12\n");
    end
endrule

/*rule rl_debug_feature_state(reset_flag == 1'b0 && reset_wait_flag == 1'b0 && timing_set == 1'b0);
  `ifdef verbose      $display($stime(),"Nandflashcontroller: : FEATURE_STATE ",fshow(feature_state));  `endif
  endrule*/
///////////////////////////////////////////////////////////////////////////////////////////////////
//After Reset we need to go and set the timing mode for the operating frequency(close to it) using 
//SET FEATURES command.  /////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
rule rl_set_feature_timing (reset_flag == 1'b0 && reset_wait_flag == 1'b0 && timing_set == 1'b0) ;
case(feature_state)
IDLE : begin
    $display($stime(),"hey m started IDLE");
    wr_onfi_signal          <= 6'b001000;
    rg_onfi_we_n            <= 1;
    if(delay_count == 7) begin
        feature_state           <= START_PROGRAM;
        delay_count             <= 0;
    end
    else 
        delay_count             <= delay_count + 1;
end
START_PROGRAM :	begin
/*Send 80h command series from here*/	
    $display($stime(),"hey m started START_PROGRAM");
    wr_onfi_signal          <= 6'b001101;
    rg_onfi_we_n            <= 1;
    rg_data_to_flash[chip_sel]        <= 'hEF ;
    rg_data_to_flash[~chip_sel]        <= 'hEF ;
    feature_state           <= WAIT_D1 ;
    end
WAIT_D1      :	begin
/*Since initially the default timing mode is mode0, we need to wait for tWP w.r.t mode0 even if 
clk is higher*/	
    gate_we                 <= 1'b1;
    wr_onfi_signal          <= 6'b001101;
    if(delay_count == `TWP_MODE0) begin
        rg_onfi_we_n            <= 0;
        delay_count <= 'h0;
        feature_state       <= WAIT_D2 ;
    end
    else begin
        rg_onfi_we_n        <= 0;
        delay_count <= delay_count + 'h1;
    end
end
WAIT_D2  :  begin
			/*For tCLH we spend one cycle here*/
    wr_onfi_signal          <= 6'b001100;
    if(delay_count == 'h3) begin
        rg_onfi_we_n            <= 1;
		delay_count <= 'h0;
		feature_state       <= C_ADDR ;
	end
    else if(delay_count == 1) begin
        rg_onfi_we_n        <= 1;
        delay_count         <= 2;
    end
    else begin
        rg_onfi_we_n            <= 1;
		delay_count <= delay_count + 'h1;
    end
	gate_we                 <= 1'b0 ;
	end
C_ADDR  :  begin
    wr_onfi_signal          <= 6'b001011;
    rg_onfi_we_n            <= 1;
	rg_data_to_flash[chip_sel]        <= 'h01 ; //For timing mode set address is 'h01
	rg_data_to_flash[~chip_sel]        <= 'h01 ; //For timing mode set address is 'h01
	feature_state           <= WAIT_D4 ;
	end
WAIT_D4  :	begin
/*Since initially the default timing mode is mode0,we need to wait for tWP w.r.t mode0 even if clk 
    is higher*/
    Bit#(6) lv_onfi;
	lv_onfi[5] = 1'b0 ;
	lv_onfi[4] = 1'b0 ;
	lv_onfi[3] = 1'b1 ;
	lv_onfi[2] = 1'b0 ;
    lv_onfi[1] = 1'b1 ;
	lv_onfi[0] = 1'b1 ;
	gate_we                 <= 1'b1;
    if(delay_count == `TWP_MODE0) begin
		delay_count <= 'h0;
//        lv_onfi[4] = 1'b1 ; //   rg_onfi_we_n <= 1;  
        rg_onfi_we_n            <= 1;
		feature_state       <= WAIT_D3 ;
	end
    else begin
  //      lv_onfi[4] = 1'b0 ; // rg_onfi_we_n <= 0;
        rg_onfi_we_n            <= 0;
		delay_count <= delay_count + 'h1;
    end
    wr_onfi_signal  <= lv_onfi;
	end
WAIT_D3  :  begin
/*Spend tADL time here but wrt timing mode 0.Also takes care of tALH.*/
   Bit#(6) lv_onfi;
   gate_we           <= 1'b0;
   lv_onfi[5] = 1'b0 ;
   lv_onfi[4] = 1'b0 ;
//   lv_onfi[4] = 1'b1 ;
   lv_onfi[3] = 1'b1 ;
   lv_onfi[2] = 1'b0 ;
   lv_onfi[0] = 1'b0 ;
    rg_onfi_we_n            <= 1;
   if(delay_count == (`TADL_MODE0)) begin
       delay_count <= 'h0;
       lv_onfi[1] = 0; //rg_onfi_ale <= 0;
        feature_state       <= START_DATA ;
   end
   else if(delay_count < 3) begin
       delay_count <= delay_count + 1;
       lv_onfi[1] = 1; //rg_onfi_ale <= 1;
   end
   else begin
      delay_count <= delay_count + 'h1;
      lv_onfi[1] = 0; // rg_onfi_ale <= 0;
   end
   wr_onfi_signal  <= lv_onfi;
   end
START_DATA   :  begin
	/*Need to send 8 bits of data to flash everytime from the actual WDC bit data*/
    wr_onfi_signal          <= 6'b001001;
    rg_onfi_we_n            <= 0;
	gate_we                 <= 1'b0;
	feature_state           <= WAIT_D6 ;
    if(byte_count == 'h00) begin
		rg_data_to_flash[chip_sel]        <= rg_cfg_timing_mode ;
		rg_data_to_flash[~chip_sel]        <= rg_cfg_timing_mode ;
		byte_count    <= byte_count+'h1;
	end
    else if(byte_count == 'h3) begin
		rg_data_to_flash[chip_sel]        <= 'h00 ;
		rg_data_to_flash[~chip_sel]        <= 'h00 ;
		byte_count        <= 'h00;
		feature_data_done <= 1'b1 ;
	end
    else begin
		rg_data_to_flash[chip_sel]        <= 'h00 ;
		rg_data_to_flash[~chip_sel]        <= 'h00 ;
		byte_count <= byte_count+'h1;
	end
	end
WAIT_D6  :	begin
/*Since initially the deafault timing mode is mode0, we need to wait for tWP w.r.t mode0 even if 
    clk is higher*/	
    Bit#(6) lv_onfi;
    gate_we                 <= 1'b1;
    lv_onfi[5] = 1'b0 ;
    lv_onfi[4] = 1'b0 ;
    lv_onfi[3] = 1'b1 ;
    lv_onfi[2] = 1'b0 ;
    lv_onfi[1] = 1'b0 ;
    lv_onfi[0] = 1'b1 ;
    
    if(delay_count == `TWP_MODE0) begin
    	delay_count <= 'h0;
//        lv_onfi[4] = 1'b1; //rg_onfi_we_n <= 1;
        rg_onfi_we_n            <= 1;
    	feature_state       <= WAIT_D7 ;
    end
    else if(delay_count == `TWP_MODE0 - 1) begin
        delay_count <= delay_count + 1;
  //      lv_onfi[4]   = 1'b1; //rg_onfi_we_n <= 1;
        rg_onfi_we_n            <= 1;
    end
    else begin
    //    lv_onfi[4]      = 1'b0; //rg_onfi_we_n <= 0;
        rg_onfi_we_n            <= 0;
    	delay_count <= delay_count + 'h1;
    end
    wr_onfi_signal  <= lv_onfi;
	end
WAIT_D7  :  begin
/*To make sure tWH is satisfied.*/
    wr_onfi_signal          <= 6'b001000;
    rg_onfi_we_n            <= 1;
    if(feature_data_done == 1'b1) begin
		feature_state    <= WAIT_TWB ;
		feature_data_done <= 1'b0 ;
	end
	else begin 
	if(delay_count == 'h3) begin
		delay_count <= 'h0;
		feature_state       <= START_DATA ; end
	else begin
		delay_count <= delay_count + 'h1;
		feature_state       <= WAIT_D7 ; end
	end
	end
WAIT_TWB     :   begin
/*Wait for tWB time period*/
	gate_we                 <= 1'b0;
    wr_onfi_signal          <= 6'b001000;
    rg_onfi_we_n            <= 1;
    if(delay_count == `TWB_MODE0) begin
		delay_count <= 'h0;
		feature_state      <= WAIT_D5 ;
	end
	else
		delay_count <= delay_count + 'h1;
	end
WAIT_D5  :  begin
/*Wait here till tFEAT.*/
    wr_onfi_signal          <= 6'b001000;
    rg_onfi_we_n            <= 1;
    if(wr_ready_busy_n[chip_sel] == 1'b0)
    	feature_state           <= WAIT_D5 ;
    else 
    	feature_state           <= DUMMY ;
    end
DUMMY  :  begin
	timing_set              <= 1'b1 ;
//    timing_set_cc           <= 1'b1;
    wr_onfi_signal          <= 6'b011000;
    rg_onfi_we_n            <= 1;
	feature_state           <= START_PROGRAM ;
	end
endcase
endrule	
		
//////////////////////////////////////////////////////////////////////////////////////////////////
///////// Getting bad blocks on power on                                                          
//////////////////////////////////////////////////////////////////////////////////////////////////
/*For MLC flash the bad block info will be present in the first byte of the spare area of the last 
page in every block. Since block0, block 3 of lun0 and lun1 constitues 1 block because of the 
mapping, if any one of them have bad signature we need to put it in the list. Since there are 2 
chips, each chip has say B blocks, .Hence for 2 chips we have 2B working blocks. The remaining B 
blocks need to be stored in a list.*/

rule rl_start_badblock_scan (badblock_flag == 1'b1 && timing_set == 1'b1 && 
    wr_ready_busy_n[chip_sel] == 1) ;// tk: condition changed for badblock_flag to skip this rule
case(bbm_state)
GET_ADDR    :  begin
	Bit#(`PAGE_WIDTH) page_addr = 'h0 ;
	Bit#(TSub#(`MAX_ROW_BITS,`PAGE_WIDTH)) block_addr_1 = {~block_addr[`MAX_ROW_BITS-`PAGE_WIDTH-1]
    ,block_addr[`MAX_ROW_BITS-`PAGE_WIDTH-2:0]};
/*Each block is being split in 4 planes.Hence we can get an adress for last page in each block,
Below we are generating address*/
	Bit#(24) total_addr0 = {'h0,block_addr,~page_addr};
	alter_addr_cycl3[0] <= total_addr0[7:0] ;
	alter_addr_cycl4[0] <= total_addr0[15:8] ;
	alter_addr_cycl5[0] <= total_addr0[23:16] ;
	Bit#(24) total_addr2 = {'h0,block_addr_1,~page_addr};
	alter_addr_cycl3[2] <= total_addr2[7:0] ;
	alter_addr_cycl4[2] <= total_addr2[15:8] ;
	alter_addr_cycl5[2] <= total_addr2[23:16] ;
    if(block_addr[1:0] == 'h0) begin
		Bit#(24) total_addr1 = {'h0,(block_addr+'h3),~page_addr};
		alter_addr_cycl3[1] <= total_addr1[7:0] ;
		alter_addr_cycl4[1] <= total_addr1[15:8] ;
		alter_addr_cycl5[1] <= total_addr1[23:16] ;
		Bit#(24) total_addr3 = {'h0,block_addr_1+'h3,~page_addr};
		alter_addr_cycl3[3] <= total_addr3[7:0] ;
		alter_addr_cycl4[3] <= total_addr3[15:8] ;
		alter_addr_cycl5[3] <= total_addr3[23:16] ;
	end
    else begin
		Bit#(24) total_addr1 = {'h0,(block_addr-'h1),~page_addr};
		alter_addr_cycl3[1] <= total_addr1[7:0] ;
		alter_addr_cycl4[1] <= total_addr1[15:8] ;
		alter_addr_cycl5[1] <= total_addr1[23:16] ;
		Bit#(24) total_addr3 = {'h0,block_addr_1-'h1,~page_addr};
		alter_addr_cycl3[3] <= total_addr3[7:0] ;
		alter_addr_cycl4[3] <= total_addr3[15:8] ;
		alter_addr_cycl5[3] <= total_addr3[23:16] ;
	end
	bbm_state  <= START_READ ;
end
START_READ   : 	begin
 		/*Send 00h comand series from here*/
   if(delay_count == 0) begin
      wr_onfi_signal          <= 6'b011000;
      delay_count <= delay_count + 1;
   end
   else begin
      wr_onfi_signal          <= 6'b011101;
      rg_data_to_flash[chip_sel]        <= 'h0;
      delay_count             <= 0;
      bbm_state  <= WAIT_D1 ; 
   end
   end
WAIT_D1  :  begin
/*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count == 1)begin
        d_sync_count <= 2;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count   <= 0;
        lv_onfi[0]     =  0;
        lv_onfi[2]     =  0;
        bbm_state      <= C_ADDR ;
    end
    wr_onfi_signal          <= lv_onfi;
    end
WAIT_NEW  :  begin
/*For tCLH we spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count    <= 2;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            bbm_state   <= C_ADDR1 ;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else if(delay_count == 'h2)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count <= 2;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            bbm_state <= P_ADDR ;
        end
    else if(delay_count == 'h3) 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count <= 2;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            bbm_state <= B_ADDR ;
        end
    else  
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count <= 2;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            bbm_state <= L_ADDR ;
            delay_count   <='h0; 
        end
    wr_onfi_signal          <= lv_onfi;
 	end
C_ADDR       : 	begin
    wr_onfi_signal          <= 6'b011011;
    delay_count<= delay_count+'h1;
    rg_data_to_flash[chip_sel]        <= addr_cycl1 ;
    bbm_state               <= WAIT_NEW ;
    end
C_ADDR1      : 	begin
    wr_onfi_signal          <= 6'b011011;
	delay_count<=delay_count+'h1;
	rg_data_to_flash[chip_sel]        <= addr_cycl2;
	bbm_state              <= WAIT_NEW ;
	end
P_ADDR  :  begin
    wr_onfi_signal          <= 6'b011011;
   	delay_count<=delay_count+'h1;
   	if(alter_cnt == 'h0)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[0] ;
   	else if(alter_cnt == 'h1)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[1] ;
   	else if(alter_cnt == 'h2)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[2] ;
   	else
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[3] ;
   	bbm_state             <= WAIT_NEW ; 
   end
B_ADDR  :  begin
    wr_onfi_signal          <= 6'b011011;
   	delay_count<=delay_count+'h1;
   	if(alter_cnt == 'h0)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[0] ;
   	else if(alter_cnt == 'h1)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[1] ;
   	else if(alter_cnt == 'h2)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[2] ;
   	else
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[3] ;
   	bbm_state             <= WAIT_NEW ; 
   end
L_ADDR  :  begin
    wr_onfi_signal          <= 6'b011011;
   	if(alter_cnt == 'h0)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[0] ;
   	else if(alter_cnt == 'h1)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[1] ;
   	else if(alter_cnt == 'h2)
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[2] ;
   	else
   		rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[3] ;
   	bbm_state             <= WAIT_D2 ; 
   end
WAIT_D2  :  begin
/*For tALH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[2]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[1]   = 1;
    end
    else if(d_sync_count == 1)begin
        d_sync_count <= 2;
        lv_onfi[0] = 0;
        lv_onfi[1] = 0;
    end
    else begin
        d_sync_count   <= 0;
        lv_onfi[0]     =  0;
        lv_onfi[1]     =  0;
        bbm_state     <= END_COMMAND;
    end
    wr_onfi_signal          <= lv_onfi;
	end
END_COMMAND  : 	begin
    wr_onfi_signal          <= 6'b011101; 
    if(alter_cnt[0] == 'h0) begin
        rg_data_to_flash[chip_sel]	 <= 'h32 ;
        bbm_state	       <= WAIT_D3 ;
        alter_cnt	       <= alter_cnt + 'h1 ;
    end
    else if(alter_cnt == 'h1) begin
        rg_data_to_flash[chip_sel]	 <= 'h30 ;
        bbm_state	       <= WAIT_D3 ;
        alter_cnt	       <= alter_cnt + 'h1 ;
    end
    else begin
        rg_data_to_flash[chip_sel]	 <= 'h30 ;
        bbm_state	       <= WAIT_D7 ;
        alter_cnt	       <= 'h0 ;
    end
    end
WAIT_D3  :  begin
/*For tCLH spend one cycle here*/
    wr_onfi_signal          <= 6'b011101;
    bbm_state               <= WAIT_TWB ;
    end
WAIT_TWB     :   begin
	/*Wait for tWB time period*/
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == `TWB_COUNT) begin
		delay_count <= 'h0;
		bbm_state       <= WAIT_STATUS_PLANE1 ;
	end
	else
		delay_count <= delay_count + 'h1;
    end
WAIT_D7  :  begin
/*For tCLH spend one cycle here*/
    wr_onfi_signal          <= 6'b011101;
    bbm_state               <= WAIT_TWB1 ;
    end
WAIT_TWB1     :   begin
	/*Wait for tWB time period*/
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == `TWB_COUNT) begin
		delay_count <= 'h0;
		bbm_state       <= ASK_STATUS1 ;
	end
	else
		delay_count <= delay_count + 'h1;
end
WAIT_STATUS_PLANE1   :   begin
/*Wait fot 2*tCCS time period.Because tDBSY and tCBSY values will be unkown from the basic timing 
information.Since tCCS is the highest value, we can use 2 times as the value of tCCS.This had to be
done because 70h and 78h operations does not work during the interval tDBSY and tCBSY in the model.
The ready busy pin cannot be checked for busy status because, in case of a genuine program , then 
we would end up waiting fore-ever till the program finishes */
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == `TCCS_COUNT*2) begin
		delay_count <= 'h0;
		bbm_state  <= START_READ ;
	end
	else
		delay_count <= delay_count + 'h1;
    end
ASK_STATUS1    :  begin
/*Check status of the LUN*/
    wr_onfi_signal                  <= 6'b011101;
	rg_data_to_flash[chip_sel]      <= 'h78 ;
	bbm_state                       <= WAIT_D8 ;
	end
WAIT_D8  :  begin
/*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count == 1)begin
        d_sync_count <= 2;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count    <= 0;
        lv_onfi[0]      = 0;
        lv_onfi[2]      = 0;
        bbm_state   <= P_ADDR_S1;
    end
    wr_onfi_signal    <= lv_onfi;
end
WAIT_NEW_1  :  begin
/*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count    <= 2;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            bbm_state   <= B_ADDR_S1;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count <= 2;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            bbm_state <= L_ADDR_S1 ;
            delay_count   <='h0; 
        end
    wr_onfi_signal <= lv_onfi;
	end
P_ADDR_S1    :	begin
    wr_onfi_signal          <= 6'b011011;
    delay_count <= delay_count+'h1;
    rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[3] ;
    bbm_state              <= WAIT_NEW_1 ;
	end
B_ADDR_S1    : 	begin
    wr_onfi_signal          <= 6'b011011;
    delay_count<=delay_count+'h1;
    rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[3] ;
    bbm_state              <= WAIT_NEW_1 ;
	end
L_ADDR_S1    : 	begin
    wr_onfi_signal          <= 6'b011011;
    rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[3] ;
    bbm_state              <= WAIT_RE_WE1 ;
	end
WAIT_RE_WE1  :  begin
/*Wait for tWHR time period*/
    Bit#(6) lv_onfi;
  	lv_onfi[5]              = 1'b0 ;
  	lv_onfi[4]              = 1'b1 ;
  	lv_onfi[3]              = 1'b1 ;
  	lv_onfi[2]              = 1'b0 ;
    enable_dqs              <= 0;
    if(delay_count  == 0) begin
         lv_onfi[1]      = 1'b1; //rg_onfi_ale         <= 1'b1;
         lv_onfi[0]      = 1'b1;
         delay_count     <= delay_count + 1;
    end
  	else if(delay_count == `TWHR_COUNT)
  	begin
  		delay_count  <= 'h0;
        lv_onfi[1]   = 1'b0; //rg_onfi_ale      <= 1'b0;
        lv_onfi[0]   = 1'b0;
  		bbm_state   <= ENABLE_S_READ1 ;
  	end
    else begin
  	  delay_count <= delay_count + 'h1;
      lv_onfi[1]  = 1'b0;  //rg_onfi_ale <= 1'b0;
      lv_onfi[0]  = 1'b0;
    end
    wr_onfi_signal  <= lv_onfi;
    end
ENABLE_S_READ1 :  begin
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b0 ;
    lv_onfi[2]        = 1'b1 ;
    lv_onfi[1]        = 1'b1 ; 
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal          <= lv_onfi;
    rg_en_rd_sync_data      <= 1;
/*    if(delay_count == 2) begin 
        delay_count       <= 0;
    end
    else 
        delay_count       <= delay_count + 1;*/
        bbm_state               <= WAIT1 ; 
   	end
WAIT1    :  begin
    wr_onfi_signal          <= 6'b011000;
    rg_en_rd_sync_data      <= 0;
/*    if(delay_count == 2) begin 
        delay_count       <= 0;
    end
    else 
        delay_count       <= delay_count + 1;*/
        bbm_state               <= READ_STATUS1 ;
        sreg                    <= unpack(chip_sel) ? wr_null_data1 : wr_null_data0;
   	end
READ_STATUS1 :	begin
    wr_onfi_signal          <= 6'b011000;
    if(sreg[6]==1'b1) begin
        bbm_state         <= WAIT_FOR_tRHW;
    end
    else begin
        bbm_state  <= ASK_STATUS1 ; 
    end
   	end
WAIT_FOR_tRHW : begin // vis: added for read to write high, timing issue.
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == 'h6) begin 
        bbm_state   <= START_READ_C;
        delay_count <= 0;
    end
    else delay_count <= delay_count + 1;
    end
START_READ_C :	begin //Send 00 command to start reading
    wr_onfi_signal          <= 6'b010110;
    if(rg_dqs_to_controller[chip_sel] == 1) begin
        dreg                    <= unpack(chip_sel)? wr_null_data1 : wr_null_data0 ;
        bbm_state              <= CONT_READ1;
    end
    end
WAIT_TCCS2   :   begin
/*Wait fot tCCS time period(300ns)(tWHR issue is also solved because this time is big)*/
    Bit#(6) lv_onfi;
	lv_onfi[5]         = 1'b0 ;
	lv_onfi[4]         = 1'b1 ;
	lv_onfi[3]         = 1'b1 ;
	lv_onfi[1]         = 1'b0 ;
	lv_onfi[0]         = 1'b0 ;
    if(delay_count == 0) begin
    	lv_onfi[2]          = 1'b1; //rg_onfi_cle             <= 1'b1 ;
        delay_count             <= delay_count + 1;
    end
    else if(delay_count == `TCCS_COUNT) begin
		delay_count <= 'h0;
        lv_onfi[2]   = 1'b0; //rg_onfi_cle     <= 1'b0;
		bbm_state        <= WAIT3 ;
	end
    else begin
        lv_onfi[2]      = 1'b0; //rg_onfi_cle     <= 1'b0;
		delay_count <= delay_count + 'h1;
    end
    wr_onfi_signal <= lv_onfi;
    end
WAIT3        :  begin //Enable re_n 
    wr_onfi_signal          <= 6'b010000;
    bbm_state               <= CONT_READ ;
    end
CONT_READ    :	begin
   wr_onfi_signal          <= 6'b010000;
   if(delay_count == 1) begin
   	dreg                    <= unpack(chip_sel)? wr_null_data1 : wr_null_data0 ;
   	bbm_state               <= CONT_READ1 ;	
   delay_count             <= 0;
   end
   else delay_count            <= delay_count + 1;
   end
CONT_READ1   :  begin
   wr_onfi_signal          <= 6'b011000;
   Bit#(1) bbm_data;
   if(dreg !='hFF) //Anything other than FF is bad block
       bbm_data  = 1'b1 ;  // 1 indicates bad block
   else
       bbm_data = 1'b0 ;
   if(alter_cnt == 'h3 || bbm_data == 1'b1) begin //4 blocks in 4 diff plane constitute super block
      bbm_list.portA.request.put ( BRAMRequest{
        				write : True, 
        				address: bbm_list_addr , 
        				datain : bbm_data, 
        				responseOnWrite : False} 
     						   ) ;
      bbm_list_addr <= bbm_list_addr + 'h1 ;
      alter_cnt     <= 'h0 ;
      bbm_state     <= DUMMY ;
//`ifdef verbose $display("Nandflashcontroller: %d: Going to dummy state",$stime()); `endif
   end
   else begin
        if(alter_cnt == 'h0 || alter_cnt == 'h2)
			bbm_state <= WAIT_RE_WE3 ;
		else //if(alter_cnt == 'h1)
			bbm_state <= WAIT_RE_WE5;
		alter_cnt <= alter_cnt + 'h1 ;
	end
    end
WAIT_RE_WE3  :  begin
/*Wait for tRHW time period*/
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == `TCCS_COUNT) begin
		delay_count <= 'h0;
		bbm_state       <= SELECT_C_R ;
	end
	else
		delay_count <= delay_count + 'h1;
end
SELECT_C_R   :  begin
/*Send 06h comand series from here*/
    wr_onfi_signal          <= 6'b011101;
	rg_data_to_flash[chip_sel]        <= 'h06 ;
	bbm_state              <= WAIT_D4 ;
	end
WAIT_D4  :  begin
/*For tCLH spend one cycle here*/
    wr_onfi_signal          <= 6'b011100;
    bbm_state               <= C_ADDR_1 ;
    end
WAIT_NEW_2  :  begin
/*For tCLH we spend one cycle here*/
   wr_onfi_signal          <= 6'b011011;
   if (delay_count == 'h1) 
       bbm_state               <= C_ADDR_2 ;
   else if(delay_count =='h2)
        bbm_state               <= P_ADDR_1 ;
   else if(delay_count =='h3) 
        bbm_state               <= B_ADDR_1 ;
   else if(delay_count =='h4) begin
        bbm_state               <= L_ADDR_1 ;                         
        delay_count <='h0;
   end
   end
C_ADDR_1     : 	begin
/*Need to send column address here , in case of col offset. But read in our case happens for full 
page after first page in lengthy read, hence col addr is 0*/
   wr_onfi_signal       <= 6'b011011;
   delay_count          <= delay_count+'h1;
   rg_data_to_flash[chip_sel]        <= addr_cycl1 ;
   bbm_state              <= WAIT_NEW_2 ;
   end
C_ADDR_2     : 	begin
/* Need to send column address here , in case of col offset. But read in our case happens for full 
page after first page in lengthy read, hence col addr is 0*/
   wr_onfi_signal          <= 6'b011011;
   delay_count             <= delay_count+'h1;
   rg_data_to_flash[chip_sel] <= addr_cycl2 ;
   bbm_state              <= WAIT_NEW_2 ;
   end
P_ADDR_1     : 	begin
/*Send page address for read*/
   wr_onfi_signal          <= 6'b011011;
   delay_count<=delay_count+'h1;
   if(alter_cnt == 'h3)
   	rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[1] ;
   else
   	rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[2] ;
   bbm_state              <= WAIT_NEW_2 ; 
   end
B_ADDR_1     : 	begin
   wr_onfi_signal          <= 6'b011011;
   delay_count<=delay_count+'h1;
   if(alter_cnt == 'h3)
   	rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[1] ;
   else
   	rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[2] ;
   bbm_state              <= WAIT_NEW_2 ;
   end
L_ADDR_1     : 	begin
   wr_onfi_signal          <= 6'b011011;
   if(alter_cnt == 'h3)
   	rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[1] ;
   else
   	rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[2] ;
   bbm_state              <= WAIT_D5 ;
   end
WAIT_D5  :  begin
/*For tALH spend one cycle here*/
   wr_onfi_signal          <= 6'b011010;
   bbm_state               <= END_COMMAND_1 ;
   end
END_COMMAND_1 : begin
    wr_onfi_signal          <= 6'b011101;
    rg_data_to_flash[chip_sel]        <= 'hE0 ; //End the command and start getting data.
    bbm_state              <= WAIT_TCCS1 ;
    end
WAIT_TCCS1   :   begin
/*Wait fot tCCS time period(300ns)*/
    Bit#(6) lv_onfi;
	lv_onfi[5]         = 1'b0 ;
	lv_onfi[4]         = 1'b1 ;
//v_onfi[4]         = 1'b1 ;
	lv_onfi[3]         = 1'b1 ;
    lv_onfi[1]         = 1'b0 ;
    lv_onfi[0]         = 1'b0 ;
    if(delay_count == 0) begin
		lv_onfi[2]         = 1'b1 ; //rg_onfi_cle             <= 1'b1 ;
        delay_count             <= delay_count + 1;
    end
    else if(delay_count == `TCCS_COUNT) begin
		delay_count <= 'h0;
        lv_onfi[2]         = 1'b0 ; //rg_onfi_cle     <= 1'b0;
		bbm_state        <= WAIT3 ;
	end
    else begin
        lv_onfi[2]         = 1'b0 ; //rg_onfi_cle     <= 1'b0;
		delay_count <= delay_count + 'h1;
    end
    wr_onfi_signal <= lv_onfi;
	end
WAIT_RE_WE5  :  begin
/*Wait for tRHW time period*/
    wr_onfi_signal          <= 6'b011010;
	if(delay_count == `TCCS_COUNT)
	begin
		delay_count <= 'h0;
		bbm_state       <= ASK_STATUS2 ;
	end
	else
		delay_count <= delay_count + 'h1;
	end
ASK_STATUS2 : 	begin
/*Check status of the LUN*/
    wr_onfi_signal          <= 6'b011101;
  	rg_data_to_flash[chip_sel]        <= 'h78 ;
  	bbm_state              <= WAIT_D6 ;
    end 
WAIT_D6  :  begin
/*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]     = 1'b0 ;
    lv_onfi[4]     = 1'b1 ;
    lv_onfi[3]     = 1'b1 ;
    lv_onfi[1]     = 1'b0 ;
    if(d_sync_count == 0) begin
        d_sync_count <= 1;
        lv_onfi[0]   = 1;
        lv_onfi[2]   = 1;
    end
    else if(d_sync_count == 1)begin
        d_sync_count <= 2;
        lv_onfi[0] = 0;
        lv_onfi[2] = 0;
    end
    else begin
        d_sync_count    <= 0;
        lv_onfi[0]      = 0;
        lv_onfi[2]      = 0;
        bbm_state   <= P_ADDR_S;
    end
    wr_onfi_signal    <= lv_onfi;
	end
WAIT_NEW_3  :  begin
/*For tCLH spend one cycle here*/
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b1 ;
    lv_onfi[2]        = 1'b0 ;
    if (delay_count =='h1)
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count    <= 2;
            lv_onfi[0]      = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            bbm_state   <= B_ADDR_S;
            lv_onfi[0]      = 0;
            lv_onfi[1]      = 1'b0;
            d_sync_count    <= 0;
        end
    else 
        if(d_sync_count == 0) begin
            d_sync_count <= 1;
            lv_onfi[0]   = 1;
            lv_onfi[1]   = 1'b1 ;
        end
        else if(d_sync_count == 1) begin
            d_sync_count <= 2;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
        end
        else begin
            d_sync_count <= 0;
            lv_onfi[0]   = 0;
            lv_onfi[1]   = 1'b0 ;
            bbm_state <= L_ADDR_S ;
            delay_count   <='h0; 
        end
    wr_onfi_signal <= lv_onfi;
	end
P_ADDR_S    :	begin
    wr_onfi_signal          <= 6'b011011;
    delay_count <= delay_count + 'h1;
    rg_data_to_flash[chip_sel]        <= alter_addr_cycl3[0] ;
    bbm_state              <= WAIT_NEW_3 ;
	end
B_ADDR_S    : 	begin
    wr_onfi_signal          <= 6'b011011;
    delay_count <= delay_count + 'h1;
    rg_data_to_flash[chip_sel]        <= alter_addr_cycl4[0] ;
    bbm_state              <= WAIT_NEW_3 ;
	end
L_ADDR_S    : 	begin
    wr_onfi_signal          <= 6'b011011;
    rg_data_to_flash[chip_sel]        <= alter_addr_cycl5[0] ;
    bbm_state              <= WAIT_RE_WE4 ;
    end
WAIT_RE_WE4  :  begin
	/*Wait for tWHR time period*/
    Bit#(6) lv_onfi;
	lv_onfi[5]     = 1'b0 ;
	lv_onfi[4]     = 1'b1 ;
//v_onfi[4]     = 1'b1 ;
	lv_onfi[3]     = 1'b1 ;
	lv_onfi[2]     = 1'b0 ;
    if(delay_count == 0) begin
	    lv_onfi[0]     = 1'b1 ;//  enable_dataout <= True;
        lv_onfi[1]       = 1'b1 ; //rg_onfi_ale <= 1'b1;
    end
    else begin
        lv_onfi[1]       = 1'b0 ; //rg_onfi_ale <=1'b0;
        lv_onfi[0]     = 1'b0 ;
    end
    if(delay_count == `TWHR_COUNT) begin
		delay_count <= 'h0;
		bbm_state       <= ENABLE_S_READ2 ;
	end
	else
		delay_count <= delay_count + 'h1;
    wr_onfi_signal  <= lv_onfi;
    end
ENABLE_S_READ2 :  begin
    Bit#(6)     lv_onfi;
    lv_onfi[5]        = 1'b0 ;
    lv_onfi[4]        = 1'b1 ;
    lv_onfi[3]        = 1'b0 ;
    lv_onfi[2]        = 1'b1 ;
    lv_onfi[1]        = 1'b1 ; 
    lv_onfi[0]        = 1'b0 ;
    wr_onfi_signal          <= lv_onfi;
    bbm_state              <= WAIT2 ;
    end
WAIT2       :   begin
    wr_onfi_signal          <= 6'b011000;
    if(delay_count == 1) begin
   	 sreg                    <= unpack(chip_sel) ? wr_null_data1 : wr_null_data0 ;
   	 bbm_state               <= READ_STATUS2 ;
     delay_count <= 0;
    end
    else delay_count <= delay_count + 1;                                                
   	end
READ_STATUS2 :	begin
   wr_onfi_signal          <= 6'b011000;
   if(sreg[6]==1'b1)
       bbm_state      <= WAIT_FOR_tRHW ;
   else
       bbm_state      <= ASK_STATUS1 ;
end
DUMMY       :	begin
    Bit#(6) lv_onfi=0;
//	lv_onfi[4]     = 1'b1 ;
	lv_onfi[3]     = 1'b1 ;
	lv_onfi[2]     = 1'b0 ;
	lv_onfi[1]     = 1'b0 ;
	lv_onfi[0]     = 1'b0 ;
    if (delay_count == `TRHW_COUNT) begin 
        bbm_state  <= GET_ADDR ;
		Bit#(TAdd#(`BLOCK_WIDTH,`LUN_WIDTH)) comp = 'h0;
        if(block_addr == ({'h0,~comp}-'h1)) begin
			block_addr <= 'h0  ;
            if(chip_sel == 1'h0) begin
                lv_onfi[5]     = 1'b0 ;
                lv_onfi[4]     = 1'b1 ;
    			chip_sel   <= 1'b1 ;
			end
            else begin
				chip_sel      <= 1'b0 ;
				badblock_flag <= 1'b0 ;
	    		bbm_list_addr <= 'h0 ;
                lv_onfi[5]     = 1'b1 ; //Disable both chips after operation done.
                lv_onfi[4]     = 1'b1 ;
`ifdef verbose  $display("Nandflashcontroller: %d: Dummy state badblock flag is down ",$stime); `endif
            end
        end
        else begin
            lv_onfi[5]     = 1'b0 ;
            lv_onfi[4]     = 1'b1 ;
            block_addr <= block_addr + 'h2 ;
        end
        delay_count <='h0;
    end
    else begin 
        delay_count <= delay_count +'h1;
        bbm_state <=DUMMY;
    end
        wr_onfi_signal <= lv_onfi;
    end
endcase
endrule	
//////////////////////////////////////////////////////////////////////////////////////////////////
///////// Bad block req processing                                                                
//////////////////////////////////////////////////////////////////////////////////////////////////
rule rl_query_bbm_capture (wr_nand_ce_n == 1'b0 && wr_nand_bbm_n == 1'b0);
//LSB 32 bit of address corresponds to the offset window , till which block query needs to be done.
//Block_addr+offset must b < or = total blocks	
	rg_bbm_offset <= wr_address_from_nvm[31:0] ;
	rg_bbm_tempof <= wr_address_from_nvm[31:0] ;
	//Blocks are split equally in two chips, hence block width + 1 will be total block address
	bbm_list_addr <= wr_address_from_nvm[(32+`BLOCK_WIDTH):32] ;
	bb_search     <= 1'b1 ;
endrule
	
rule rl_search_bb_list (bb_search == 1'b1) ;
    if(bb_count_t == `WDC) begin
		bb_count_t <= 'h0;
		if(rg_bbm_offset == 'h1)
			bb_search     <= 'h0;
		else
			bb_search     <= 'h1;
	end
    else begin
		bbm_list.portA.request.put ( BRAMRequest{
        			      		write : False, 
        		      			address: bbm_list_addr , 
        		      			datain : ?, 
        		      			responseOnWrite : False} 
        	       	    	   ) ;
	
        if(rg_bbm_offset == 'h1) begin
			bbm_list_addr <= 'h0;
			bb_search     <= 'h0;
			bb_count_t    <= 'h0;
		end
        else begin
			bbm_list_addr <= bbm_list_addr + 'h1 ;
			bb_count_t    <= bb_count_t + 'h1 ;
		end
		rg_bbm_offset <= rg_bbm_offset - 'h1 ;
	end
endrule 
	
rule rl_read_bb_list(send_bbm_data == 1'b0);
    // fired 1 cycle after 'bbm_list.portA.request.put' is called in data read cycle.
	let data <- bbm_list.portA.response.get() ;
	Bit#(`WDC) temp_bit_map = {'h0,data};
	rg_bit_map <= (rg_bit_map|(temp_bit_map<<bb_count));
    if(bb_count == `WDC-1 || rg_bbm_tempof == 'h1) begin
		rg_interrupt  <= 1'b1 ;
		send_bbm_data <= 1'b1 ;
		bb_count      <= 'h0;
	end
	else
		bb_count      <= bb_count + 'h1;
    rg_bbm_tempof <= rg_bbm_tempof - 'h1 ;
endrule

rule rl_send_bbm_data(send_bbm_data == 1'b1);
	rg_data_to_nvm <= tagged Valid rg_bit_map ;
	rg_bit_map     <= 'h0 ;
	send_bbm_data  <= 1'b0 ;
endrule
	
///////////////////////////////////////////////////////////////////////////////////////////////////
///////// Busy flag updation                                                              /////////
///////////////////////////////////////////////////////////////////////////////////////////////////
rule rl_ready_busy_update ;
//For write , if both FIFOs are busy, busy will be enabled,  indicating no data can be taken during
// that time.
    rg_ready_busy_n <= (data_w_fifo_free)&(~(read_pending|block_erase_ongoing|badblock_flag|
    (reset_flag^reset_wait_flag)|bb_search | ~timing_set)) ;
 //`ifdef verbose     $display($stime(),"Nandflashcontroller: : rg_ready_busy_n %d\n", rg_ready_busy_n); `endif
//rg_ready_busy_n <= (data_w_fifo_free)&(~(read_pending|block_erase_ongoing|(reset_flag^reset_wait_flag)|bb_search|~timing_set)) ;//tk:badblock_flag removed from the condition
endrule

///////////////////////////////////////////////////////////////////////////////////////////////////


interface ONFi_cfg_reg_interface cfg_reg_interface;
    method Action _timing_mode(Bit#(8) data);
        rg_cfg_timing_mode <= data;
    endmethod
endinterface

interface NandFlashInterface nvm_nfc_interface;
                      
		      method Action _request_data(_address,_length) ;
                  wr_nand_re_n	      <= 1'b0;
                  wr_nand_ce_n        <= 1'b0;
                  wr_nand_we_n	      <= 1'b1;
                  wr_rd_addr_frm_nvm  <= pack(_address);
                  wr_r_length         <= pack(_length);
       	      endmethod: _request_data
		      
		      method Action _request_erase(_address) ;
                  wr_nand_erase	  <= 1'b1;
                  wr_nand_ce_n <= 1'b0;
                  wr_address_from_nvm <= pack(_address);
              endmethod: _request_erase

              method ActionValue#(Bit#(`WDC)) _get_data_() if(data_r_fifo_free  == 1'b0 && 
                  ff_data_r_fifo.notEmpty == True) ;
                 let lv_data_to_nvm	= ff_data_r_fifo.first ;
                `ifdef verbose $display("Nandflashcontroller: %d: rg_data_to_nvm %h",$stime(),ff_data_r_fifo.first); `endif
                	ff_data_r_fifo.deq ;
                	if(q_data_count_g < `PAGE_LENGTH)
                		q_data_count_g <= q_data_count_g + 'h1 ;
                    else begin
                		q_data_count_g    <= 'h0 ;
                		data_r_fifo_free  <= 1'b1 ; //Make the write FIFO free.
                		//This is to help ready/busy decision making
                        if(flag_end_read   == 1'b1) begin
                			flag_end_read   <= 1'b0 ;
                			read_pending    <= 1'b0 ;
                		end
                	end
                  return lv_data_to_nvm ;
              endmethod: _get_data_

              method Action _write_addr(_address, _length) ;
                  wr_address_from_nvm <= pack(_address) ;
                  wr_w_length         <= pack(_length) ;
              endmethod: _write_addr

              method Action _write_data(_data);
                  $display($stime(),"NandFlashController: rg_data_from_nvm %x",_data);
                  wr_data_from_nvm    <= _data;
                  wr_nand_ce_n <= 1'b0;
                  wr_nand_we_n	  <= 1'b0;
              endmethod

              method Action _enable(bit _nand_ce_l);
//                wr_nand_ce_n        <= _nand_ce_l;
              endmethod: _enable

              method bit interrupt_() ;
                  return rg_interrupt;           //Interrupt when read data is ready 
              endmethod: interrupt_

              method bit busy_() ;              	              
                  return (~rg_ready_busy_n);     //Return busy state of the NAND Controller
              endmethod: busy_
		      
		      method bit write_success_() ;              	              
  	              return (rg_write_success);     
       	      endmethod: write_success_
		      
		      method bit write_fail_() ;              	              
                  return (rg_write_fail);     
              endmethod: write_fail_
		      
		      method bit erase_success_() ;              	              
                  return (rg_erase_success);     
              endmethod: erase_success_
		      
		      method bit erase_fail_() ;              	              
                  return (rg_erase_fail);     
              endmethod: erase_fail_
		      
		      method Action _query_bad_block(_address) ;
                  wr_nand_bbm_n	  <= 1'b0     ;
                  wr_address_from_nvm <= pack(_address) ;
              endmethod: _query_bad_block
		
          endinterface

//interface nvm_nfc_interface  = fn_nvm_nfc_interface ( wr_address_from_nvm, wr_data_from_nvm, 
//    wr_w_length, wr_r_length,rg_data_to_nvm, wr_nand_ce_n, wr_nand_we_n, wr_nand_re_n, rg_interrupt
//    ,rg_ready_busy_n, wr_nand_erase, rg_write_success, rg_write_fail, rg_erase_success,
//    rg_erase_fail, wr_nand_bbm_n);

interface nfc_onfi_interface = fn_nfc_onfi_interface ( rg_onfi_ce_n[0], rg_onfi_ce_n[1], 
    rg_onfi_cle, rg_onfi_ale, rg_we_toggle, rg_onfi_we_n, timing_set , rg_onfi_re_n, rg_onfi_wp_n,wr_ready_busy_n[0], 
    wr_ready_busy_n[1], rg_dqs_to_flash[0], enable_dqs,rg_dqs_to_controller[0],
    rg_dqs_c_to_flash[0], enable_dqs, rg_dqs_c_to_controller[0], rg_dqs_to_flash[1], enable_dqs
    ,rg_dqs_to_controller[1], rg_dqs_c_to_flash[1], enable_dqs, rg_dqs_c_to_controller[1],
    rg_data_to_flash[0], enable_dataout, rg_data_to_controller[0], rg_data_to_flash[1],
    enable_dataout, rg_data_to_controller[1]);

endmodule: mkNandFlashController

endpackage 
