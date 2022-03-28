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

Author: Arjun Menon	
Email id: c.arjunmenon@gmail.com
Details: This is a 7-channel DMA controller whose spec is close to that of STM32L46xxx MCUs (refer 
Chapter 11 in ST Micro's RM0394 Reference Manual).

--------------------------------------------------------------------------------------------------
*/

//TODO 
//1. change CPAR, CMAR and CNDTR registers to conditionalWrite registers so that a write to these registers is possible only when the DMA channel is disabled.
//   DMA ISR register cannot be written by the software in any case. It can be reset by the software by writing into to DMA_IFCR.
//--DONE-- 2. Implement functionality of DMA_IFCR
//--DONE-- 3. Optimization. Remove the 1+ from 1 + (2*chanNum) for id because we are anyways checking for read and write responses in different FIFOs
//--DONE-- 4. What to do if the DMA channel needs to be disabled in between? How to clear the FIFOs, and what to do about the pending on going transaction?
//5. Opt. Try to remove currentReadRs and currentWriteRs and use !destAddrFs.notEmpty instead to detect transfer complete and generate exception. Is generateTransferDoneRules even needed then?
//6. Parameterize the number of channels and peripherals completely.
//7. Write an assertion if currentReadRs[chan]==currentWriteRs[chan] then destAddrFs and responseDataFs are both empty. Prove this formally?
//8. Implement burst mode readConfig and writeConfig registers
//9. While joining generateTransferDoneRules, try using rJoinConflictFree instead of rJoinDescendingUrgency and check if they are formally equivalent. The conflict is between finishWrite and this rule. But the condition currentReadRs==currentWriteRs will not be true untill finishWrite fires. So, these two rules will never actually fire together.
//10.What will happen if while disabling a channel, you go ahead and disable another channel? Mostly can't do because you have not issued a write response for the first request unless the channel has actually been disabled.


package DMA;

import FIFO::*;
import FIFOF::*;
import Vector::*;
import FShow::*;
import GetPut::*;
import DefaultValue::*;
import AXI4_Types::*;
import AXI4_Lite_Types::*;
import AXI4_Fabric::*;
import Semi_FIFOF::*;
import SpecialFIFOs::*;
import ConcatReg::*;
import ConfigReg::*;
import BUtils::*;
import device_common::*;

`define Burst_length_bits 8
`define byte_offset 2

  // project files to be imported/included
  `include "common_params.bsv"
// ================================================================
// DMA requests and responses parameters


function Bit#(1) fn_aligned_addr(Bit#(3) addr, Bit#(3) awsize);
   bit out = 0;
    case(awsize)
        'd3: begin
            if(addr == 0) out = 1;
        end
        'd2: begin
            if(addr[1:0] == 0) out = 1;
        end
        'd1: begin
            if(addr[0] == 0) out = 1;
        end
        'd0: begin
            out = 1;
        end
        default: out = 0;
    endcase
    return out;
endfunction




//The Req and Resp are of the same width in the current design
//typedef Bit#(10) RespInfo;
//typedef Bit#(1) RespAddr;
//typedef Bit#(32) RespData;

// ================================================================
// The DMA interface has two sub-interfaces
//  A AXI4 Slave interface for config
//  A AXI4 Master interface for data transfers

interface User_ifc#(numeric type addr_width, numeric type data_width, numeric type user_width, numeric type config_addr_width, numeric type config_data_width, numeric type numChannels, numeric type numPeripherals);//giving msipsize as a parameter 
	method Action read_req(Bit#(config_addr_width) addr, AccessSize size, Bool prot);
	method ActionValue#(Tuple2#(Bool, Bit#(config_data_width))) read_resp;
	method Action write_req(Bit#(config_addr_width) addr, Bit#(config_data_width) data, AccessSize size, Bool prot);
	method ActionValue#(Bool) write_resp;
	method Action interrupt_from_peripherals(Bit#(numPeripherals) pint);
	interface Get#(Bit#(1)) interrupt_to_proc;
	interface AXI4_Master_IFC#(addr_width, data_width, user_width) master;
endinterface


typedef UInt#(16)  DMACounts ;
// The number of channels is a parameter.
// typedef 2 NumChannels;


// Several configuration registers are included, and connected to the
// config socket/fifo.

// Along with the destination address, if we are writing into a peripheral, we need to pass the peripheral id 
// of the peripheral to check if the corresponding interrupt line is still high.
// Also, we need to send the destination transfer size for all the transactions.
typedef struct{
	Bit#(addr_width) addr;
	Bool is_dest_periph;
	Bit#(TLog#(numPeriphs)) periph_id;
} DestAddrFs_type#(numeric type numPeriphs, numeric type addr_width) deriving (Bits,Eq);

typedef struct{
	Bit#(TLog#(numChannels)) chanNum;
	Bit#(4) id;
} Disable_channel_type#(numeric type numChannels) deriving (Bits, Eq);

instance DefaultValue#(Disable_channel_type#(numChannels));
	defaultValue= Disable_channel_type { chanNum: 'd-1,
										 id: 'd-1 };
endinstance

(* descending_urgency = "writeConfig, handle_interrupts" *)
(* descending_urgency = "writeConfig, rl_finishRead" *)
(* descending_urgency = "writeConfig, rl_startWrite" *)
module mkDMA( User_ifc#(addr_width, data_width, user_width, config_addr_width, config_data_width, numChannels, numPeripherals) )
provisos (Add#(a__, TLog#(numPeripherals), 4),
	 				//Add#(numChannels, xyz__, 7),
	 				Add#(numChannels, 0, 3),
					//Add#(TMul#(numChannels, 4), a__, 64),
					Add#(b__, 8, config_addr_width),
					Add#(7, j__, addr_width),
					Add#(k__, 3, config_addr_width),	
					Add#(addr_width, g__, config_data_width),
					Add#(c__, 32, config_data_width),
			    Mul#(8, d__, config_data_width),
  			  Mul#(16, e__, config_data_width),
  				Mul#(32, f__, config_data_width),
					Add#(28, h__, config_data_width),
					Add#(16, i__, config_data_width),
					Add#(12, l__, config_data_width)	//for numChannels=3
);

	let val_numChannels= valueOf(numChannels);

	AXI4_Master_Xactor_IFC #(addr_width, data_width, user_width) m_xactor <- mkAXI4_Master_Xactor;
	Wire#(Bit#(config_addr_width)) wr_read_addr <- mkWire();
	Wire#(AccessSize) wr_read_access_size <- mkWire();
	Wire#(Bool) wr_read_prot <- mkWire();
	FIFO#(Tuple2#(Bool, Bit#(config_data_width))) ff_read_resp <- mkBypassFIFO();
	
	Wire#(Bit#(config_addr_width)) wr_write_addr <- mkWire();
	Wire#(Bit#(config_data_width)) wr_write_data <- mkWire();
	Wire#(AccessSize) wr_write_access_size <- mkWire();
	Wire#(Bool) wr_write_prot <- mkWire();
	FIFO#(Bool) ff_write_resp <- mkBypassFIFO();

	////////////////////////////////////////////////////////////////
	//////////////////////// DMA Registers /////////////////////////
	////////////////////////////////////////////////////////////////
	Vector#(numChannels,Reg#(Bit#(4))) dma_isr <- replicateM(mkReg(0));		//Interrupt Status Register
	Vector#(numChannels,Reg#(Bit#(4))) dma_ifcr <- replicateM(mkReg(0));	//Interrupt Flag Clear Register
	Vector#(numChannels,Reg#(Bit#(32))) dma_ccr <- replicateM(mkConfigReg(0));	//Channel Configuration Register
	Vector#(numChannels,Reg#(Bit#(16))) dma_cndtr <- replicateM(mkConfigReg(0));	//Channel Number of Data Transfer Register
	Vector#(numChannels,Reg#(Bit#(addr_width))) dma_cpar <- replicateM(mkReg(0));	//Channel Peripheral Address Register
	Vector#(numChannels,Reg#(Bit#(addr_width))) dma_cmar <- replicateM(mkReg(0));	//Channel Memory Address Register
	Vector#(numChannels,Reg#(Bit#(4))) dma1_cselr <- replicateM(mkReg(0));	//Channel SELection Register
	//We do not have dma2_cselr because there is only one DMA, and not 2 in this architecture

	//Registers to keep track if all the data read is wrtten
	//Vector#(numChannels, Array#(Reg#(DMACounts))) currentReadRs[2]  <- replicateM(mkCReg(2,0));
	//Vector#(numChannels, Array#(Reg#(DMACounts))) currentWriteRs[2] <- replicateM(mkCReg(2,0));
	Reg#(DMACounts) currentReadRs[valueOf(numChannels)][2];
	Reg#(DMACounts) currentWriteRs[valueOf(numChannels)][2];
	Reg#(Bool)		rg_is_cndtr_zero[valueOf(numChannels)][2];
  Reg#(Bit#(TDiv#(data_width,8))) rg_write_strobe <- mkRegU();
  Reg#(Bit#(2)) rg_tsize <- mkRegU();
  Reg#(Bit#(1)) rg_burst_type <- mkRegU();

	for(Integer i=0 ; i<valueOf(numChannels) ; i=i+1) begin
		currentReadRs[i] <- mkCReg(2,0);
		currentWriteRs[i] <- mkCReg(2,0);
		rg_is_cndtr_zero[i] <- mkCReg(2,True);
	end

	// Use a FIFO to pass the read response to the write "side",
	//  thus allowing pending transations and concurrency.
	// FIFOs can be replicated as well.
	Vector#(numChannels,FIFOF#(Bit#(data_width)))  
		  responseDataFs <- replicateM(mkSizedFIFOF(2)) ;  

	//Wire to pass the interrupt from peripheral to DMA
	Wire#(Vector#(numPeripherals,Bit#(1))) wr_peripheral_interrupt <- mkDWire(replicate(0));

	//Wire to set the TEIF
	Wire#(Maybe#(Bit#(4))) wr_bus_err <- mkDWire(tagged Invalid);

	// We also want to pass the destination address for each read over
	// to the write "side", along with some other metadata.
	// The depth of this fifo limits the number of outstanding reads
	// which may be pending before the write.  The maximum outstanding
	// reads depends on the overall latency of the read requests.
	Vector#(numChannels,FIFOF#(DestAddrFs_type#(numPeripherals,addr_width))) 
		destAddrFs <- replicateM( mkSizedFIFOF(2)) ;
	
	// This register stores the initial value of the CNDTR. It is used to restore the value back
	// when operating in circular mode.
	Vector#(numChannels,Reg#(Bit#(16))) rg_cndtr <- replicateM(mkReg(0));

	// The spec specifies that the CPAR and CMAR values when read in middle of a transaction should
	// still hold the original programmed value, and not the address of the current transaction.
	// Therefore, we have a copy of these registers which indicate the address of the current
	// ongoing transaction on that channel.
	Vector#(numChannels,Reg#(Bit#(addr_width))) rg_cpa <- replicateM(mkConfigReg(0));	// Local Channel Peripheral Address Register
	Vector#(numChannels,Reg#(Bit#(addr_width))) rg_cma <- replicateM(mkConfigReg(0));	// Local Channel Memory Address Register

	Reg#(Bit#(`Burst_length_bits)) rg_burst_count <- mkReg(0);
	Reg#(Bit#(TLog#(numChannels))) rg_current_trans_chan_id <- mkReg(0);
	Reg#(Tuple2#(Bool, Bit#(TLog#(numChannels)))) rg_disable_channel <- mkReg(tuple2(False, ?));
	Reg#(Tuple2#(Bit#(TLog#(numChannels)), Bit#(config_data_width))) rg_writeConfig_ccr <- mkReg(tuple2(0,0));
	Reg#(Bool) rg_finish_write[valueOf(numChannels)][2];
	Reg#(Bool) rg_finish_read[valueOf(numChannels)][2];
	for(Integer i=0 ; i<valueOf(numChannels) ; i=i+1) begin
		rg_finish_write[i]<- mkCReg(2,True);
		rg_finish_read[i]<- mkCReg(2,True);
	end

	// This function returns the id of the peripheral for which this channel is configured
	function Bit#(TLog#(numPeripherals)) fn_map_channel_to_periph(Integer chanNum);
		return truncate(dma1_cselr[chanNum]);
	endfunction

	// This function tells the maximum priority number amongst all the active peripherals
	// which want to initiate a transaction. If one of the peripherals want to initiate a transaction
	// i.e. it's interrupt line in high, we check if any other peripheral, whose interrupt line is high,
	// has a higher priority (bits 13:12 in dma_ccrX). If so, we set the max_priority_num to
	// the value of the highest interrupt.
	// Note that even after this there might be multiple peripheral which have the same priority level as
	// max_priority_num. Amongst them, the one with the lower channel number is given priority.
	function Tuple2#(Bit#(TLog#(numChannels)),Bit#(TLog#(numPeripherals))) fn_dma_priority_encoder();
		Bit#(2) max_priority_num= 0;
		//stores id of the periph whose channel has been granted to generate req in this cycle.
		Bit#(TLog#(numChannels)) grant_chan_id= 'd-1;
		Bit#(TLog#(numPeripherals)) grant_periph_id= 'd-1;
		for(Integer i=valueOf(numChannels)-1; i>=0; i=i-1) begin			//for every channel
			let periph_connected_to_channel_i=fn_map_channel_to_periph(i);	//identify the peripheral connected to this channel
			if(dma_ccr[i][0]==1 && dma_isr[i][1]==0 &&                                          //if the DMA channel is enabled
			(wr_peripheral_interrupt[periph_connected_to_channel_i]==1 	//if the peripheral has raised an interrupt
			|| dma_ccr[i][14]==1											//if M2M transfer, need not check for interrupt
			//|| (dma_ccr[4]==1 && !rg_burst)	//if not burst mode, the interrupt of periph needn't be high when reading from memory
				  								//if in burst mode, its better if the peripheral is ready, and then we fetch the word from memory? TODO
			)) begin 
				//check if its priority is greater than any of the other peripherals' priority
				//here we check for equality also as the channels with lower number have higher priority.
				//therefore if two requests have same "high" software priority, the channel whose channel number
				//is lower will be chosen
				if(dma_ccr[i][13:12]>=max_priority_num) begin
					max_priority_num= dma_ccr[i][13:12];	//if so, then set the current priority level to be that of peripheral[i]
					grant_chan_id= fromInteger(i);
					grant_periph_id= periph_connected_to_channel_i;
				end
			end
		end
		return tuple2(grant_chan_id,grant_periph_id);
	endfunction

	function Bit#(16) fn_decr_cndtr(Bit#(16) cndtr, Bit#(2) tsize, Bit#(`Burst_length_bits) bsize);
		Bit#(17) lv_to_sub= (zeroExtend(bsize)+1) << tsize;
		Bit#(17) lv_result= {1'b0,cndtr}-lv_to_sub;
		if(lv_result[16]==1)    //underflow. Can happen in burst mode when the bytes to be transferred is not an exact multiple of the burst length x burst size.
			return 0;
        else
		    return lv_result[15:0];
    endfunction

	// This function increments the source or destination address depending on
	// the size of the transfer.
	// Note that though STM's DMA defines tsize=2'b11 as reserved, we use it to
	// perform a 64-bit data transfer.
	function Bit#(addr_width) fn_incr_address(Bit#(addr_width) addr, Bit#(2) tsize, Bit#(`Burst_length_bits) bsize) provisos(Add#(addr_width,1, a),
											Add#(z__, 8, a));
		Bit#(a) lv_to_add= (zeroExtend(bsize)+1) << tsize;
		Bit#(a) lv_result= {1'b0,addr}+lv_to_add;
		return truncate(lv_result);
	endfunction

	// DMA rules //////////////////////////////////////////////////
	// We define a function inside the module so it can access some
	// of the registers without passing too many arguments.  
	// The function takes as arguments the conditions and fifos
	// (interfaces)
	// And returns a set a rules.
	// The rule are identical to the set used in the one mmu port case.
	function Rules generatePortDMARules (AXI4_Master_Xactor_IFC#(addr_width, data_width, user_width) xactor, Integer chanNum);
		return
		rules

	    /*rule display_stat(dma_ccr[chanNum][0]==1 || tpl_1(rg_disable_channel));
			Bit#(2) max_priority_num= 0;
			//stores id of the periph whose channel has been granted to generate req in this cycle.
			Bit#(TLog#(numChannels)) grant_chan_id= 'd-1;
			Bit#(TLog#(numPeripherals)) grant_periph_id= 'd-1;
			for(Integer i=valueOf(numChannels)-1; i>=0; i=i-1) begin			//for every channel
				let periph_connected_to_channel_i=fn_map_channel_to_periph(i);	//identify the peripheral connected to this channel
				//$display("\nChan %d interrupt: %b cndtr: 'h%h prio: %d", i, dma_isr[i][1], wr_peripheral_interrupt[periph_connected_to_channel_i], dma_cndtr[chanNum],tpl_1(fn_dma_priority_encoder()));
				if(dma_ccr[i][0]==1 && dma_isr[i][1]==0 &&                                          //if the DMA channel is enabled
				  (wr_peripheral_interrupt[periph_connected_to_channel_i]==1 	//if the peripheral has raised an interrupt
				  || dma_ccr[i][14]==1											//if M2M transfer, need not check for interrupt
				  //|| (dma_ccr[4]==1 && !rg_burst)
				  )) begin //if not burst mode, the interrupt of periph needn't be high when reading from memory
					//check if its priority is greater than any of the other peripherals' priority
					//here we check for equality also as the channels with lower number have higher priority.
					//therefore if two requests have same "high" software priority, the channel whose channel number
					//is lower will be chosen
					if(dma_ccr[i][13:12]>=max_priority_num) begin
						max_priority_num= dma_ccr[i][13:12];	//if so, then set the current priority level to be that of peripheral[i]
						grant_chan_id= fromInteger(i);
						grant_periph_id= periph_connected_to_channel_i;
						//$display("########## grant chan: %d grant periph_id: %d", grant_chan_id, grant_periph_id);
					end
				//$display("dma_ccr[%d][13:12]: %b max_prio: %b",i,dma_ccr[i][13:12],max_priority_num);
				end
			end
			if(pack(wr_peripheral_interrupt)!=0 || dma_ccr[chanNum][14]==1) begin
				Reg#(Bit#(64)) lv_dma_ifcr= regAToRegBitN( vectorToRegN( dma_ifcr ));
				$display($time,"Chan%d: cndtr: %h dma_isr: %h dma_ifcr: %h", chanNum, dma_cndtr[chanNum], dma_isr[chanNum], lv_dma_ifcr );
			end
			//$display($time,"grant chan_id: %d periph_id: %d",grant_chan_id,grant_periph_id);
			//$display("Channel no. %d user_id: %d", chanNum, xactor.o_rd_data.first.ruser); 
        endrule*/

		// To start a read, following conditions need to be met
		// 1. There is data to be transferred
		// 2. The dma is enabled (dma_ccr[chanNum][0]=1)
		// 3. The channel has the highest priority
		// 4. If multiple channels have same high priority, choose the one whose channel number is lowest
		// 5. A channel is disabled before the complete transfer is finished, do not initiate any more requests
		// The 2nd, 3rd and 4th conditions are handled by fn_dma_priority_encoder
		(* descending_urgency =  "rl_startWrite, rl_startRead" *)
		//(* preempts = "rl_finishRead, rl_can_change_channel" *)
		//(* preempts = "rl_startWrite, rl_can_change_channel" *)
		//(* preempts = "rl_send_burst_write_data, rl_can_change_channel" *)
		rule rl_startRead	( !rg_is_cndtr_zero[chanNum][0] &&	//no of bytes remaining to transfer is not 0
						  	  fromInteger(chanNum) == tpl_1(fn_dma_priority_encoder()) &&  //if the this channel has the highest priority
							  (!tpl_1(rg_disable_channel) ||  tpl_2(rg_disable_channel)!=fromInteger(chanNum))
						  		&& rg_finish_read[chanNum][1]);	//if the channel is not being disabled by the processor
			Bit#(addr_width) lv_araddr;
			Bit#(2) lv_arsize;
			bit lv_burst_type;
			let lv_dma_ccr= dma_ccr[chanNum];
			Bit#(`Burst_length_bits) lv_burst= lv_dma_ccr[31:32-`Burst_length_bits]; //Upper bits of CCR supports configurable bursts !! Added, not part of STMicro
			let lv_periph_id= tpl_2(fn_dma_priority_encoder());

			if(lv_dma_ccr[6]==1)	//peripheral increment mode is on
				rg_cpa[chanNum]<= fn_incr_address(rg_cpa[chanNum], lv_dma_ccr[9:8], dma_ccr[chanNum][31:32-`Burst_length_bits]);
			if(lv_dma_ccr[7]==1)	//memory increment mode is on
				rg_cma[chanNum]<= fn_incr_address(rg_cma[chanNum], lv_dma_ccr[11:10], dma_ccr[chanNum][31:32-`Burst_length_bits]);

			if(lv_dma_ccr[4]==0) begin		//read from peripheral
				lv_araddr= rg_cpa[chanNum];	//set the address to read from
				lv_arsize= lv_dma_ccr[9:8];	//set the transfer size
				lv_burst_type= lv_dma_ccr[6];	//0: Fixed, 1: INCR which is consistent with that of AXI4
				`ifdef verbosity>2 $display($time,"\tDMA: chan[%0d] starting read from peripheral address %h",chanNum, lv_araddr); `endif
				// Since the destination is memory, the write request needn't wait for any interrupt line to be high
				// Therefore, we send the first argument as Invalid
				destAddrFs[chanNum].enq( DestAddrFs_type {	addr: rg_cma[chanNum], // Enqueue the Write destination address
															is_dest_periph: False,
															periph_id: lv_periph_id});
			end
			else begin							//read from memory
				lv_araddr= rg_cma[chanNum];		//set the address to read from
				lv_arsize= lv_dma_ccr[11:10];	//set the transfer size
				lv_burst_type= lv_dma_ccr[7];	//0: Fixed, 1: INCR which is consistent with that of AXI4
				`ifdef verbosity>2 $display($time,"\tDMA: chan[%0d] starting read from memory address %h",chanNum, lv_araddr); `endif
				// Since the destination address is that of a peripheral, the write request can be issued only when
				// the corresponding peripheral's interrupt line is high. Therefore, we send the periph_id too.
				Bool lv_is_dest_periph;
				if(lv_dma_ccr[14]==0)
					lv_is_dest_periph= True;
				else
					lv_is_dest_periph= False;
                    `ifdef verbosity>2 $display("dest_is_periph: %h",lv_is_dest_periph); `endif
				destAddrFs[chanNum].enq( DestAddrFs_type { 	addr: rg_cpa[chanNum], // Enqueue the Write destination address
															is_dest_periph: lv_is_dest_periph,
															periph_id: lv_periph_id});

			end

			//TODO If transaction size is not a multiple of specified burst length
			//let lv_transaction_size= (lv_burst+1) * (1<<lv_arsize)
			//if(dma_cndtr<lv_transaction_size)
			//	lv_burst= (dma_cndtr-lv_transaction_size)/(1<<lv_arsize)
			//	rg_burst<= lv_burst;
			

			// Create a read request, and enqueue it
			// Since there can be multiple pending requests, either read or
			// writes, we use the arid field to mark these.
			let read_request = AXI4_Rd_Addr {araddr: lv_araddr, 
											 arid: {1'b1,fromInteger(chanNum)}, arlen: lv_burst,
											 arsize: zeroExtend(lv_arsize), arburst: zeroExtend(lv_burst_type), //arburst: 00-FIXED 01-INCR 10-WRAP
											 aruser: 0 };
				
			xactor.i_rd_addr.enq(read_request);
            `ifdef verbosity>2 $display("Sending a read request with araddr: %h arid: %h arlen: %h arsize: %h arburst: %h",lv_araddr,fromInteger(chanNum),lv_burst,lv_arsize,lv_burst_type); `endif

			//housekeeping. To be done when the transaction is complete.
			currentReadRs[chanNum][0]<= currentReadRs[chanNum][0] + 1;
            dma_cndtr[chanNum]<= fn_decr_cndtr(dma_cndtr[chanNum], lv_arsize, dma_ccr[chanNum][31:32-`Burst_length_bits]);
            rg_current_trans_chan_id<= fromInteger(chanNum);
			rg_finish_read[chanNum][1]<= False;
		endrule

		// We finish the read when we see the correct respInfo on the mmu response fifo
		rule rl_finishRead (xactor.o_rd_data.first.rid == {1'b1,fromInteger(chanNum)} && xactor.o_rd_data.first.rresp==AXI4_OKAY);
			// update cndtr register to keep track of remaining transactions
			let lv_dma_ccr= dma_ccr[chanNum];
			Bit#(2) lv_tsize;
			Bit#(2) lv_source_size;

			if(dma_ccr[chanNum][4]==0) begin		//if the source is peripheral
				lv_tsize= dma_ccr[chanNum][11:10];	//destination's tsize will be that of memory
				lv_source_size= dma_ccr[chanNum][9:8];
			end
			else begin
				lv_tsize= dma_ccr[chanNum][9:8];	//destination's tsize will be that of peripheral
				lv_source_size= dma_ccr[chanNum][11:10];
			end

			// grab the data from the mmu reponse fifo
			let resp <- pop_o(xactor.o_rd_data);
			`ifdef verbosity>2 $display("DMA: chan[%d] finish read. Got data: %h",chanNum, resp.rdata); `endif

			// Pass the read data to the write "side" of the dma
			responseDataFs[chanNum].enq( resp.rdata );
			rg_finish_read[chanNum][0]<= True;
		endrule

		//rule to handle circ mode
		rule rl_handle_circ_mode(dma_ccr[chanNum][5]==1 && rg_is_cndtr_zero[chanNum][1]); //if circular mode is enabled
			dma_cndtr[chanNum]<= rg_cndtr[chanNum];
		endrule

		// This rule start the write process
		// Note that this rule conflicts with rule startRead, so we make
		// this rule be more urgent. i.e., finish writing before you start
		// reading more.
		/*rule rl_startWrite111;
			$display("helloo.. intrpt: %b burst_count: %h finish_write: %b", wr_peripheral_interrupt[ destAddrFs[chanNum].first.periph_id ], rg_burst_count, rg_finish_write[chanNum][1]);
		endrule*/

		rule rl_startWrite( (destAddrFs[chanNum].first.is_dest_periph==False		//if the dest is memory
		|| wr_peripheral_interrupt[ destAddrFs[chanNum].first.periph_id ]==1)	//if dest is not memory, then check if the peripheral's interrupt line is active
		&& rg_burst_count==0 && rg_finish_write[chanNum][1]==True);
			let lv_data= destAddrFs[chanNum].first;

			Bit#(2) lv_tsize;
			bit lv_burst_type;
			let lv_dma_ccr= dma_ccr[chanNum];
			if(lv_dma_ccr[4]==0) begin			//if the source is peripheral
				lv_tsize= lv_dma_ccr[11:10];	//destination's tsize will be that of memory
				lv_burst_type= lv_dma_ccr[7];	//destination's burst type will be that of memory
			end
			else begin
				lv_tsize= lv_dma_ccr[9:8];		//destination's tsize will be that of peripheral
				lv_burst_type= lv_dma_ccr[6];	//destination's burst type will be that of peripheral
			end

			let actual_data= responseDataFs[chanNum].first;
			Bit#(`Burst_length_bits) lv_burst_len= lv_dma_ccr[31:32-`Burst_length_bits];
		//	Bit#(6) x = {3'b0,lv_data.addr[2:0]}<<3;
			Bit#(TDiv#(data_width,8)) write_strobe=lv_tsize==0?'b1:lv_tsize==1?'b11:lv_tsize==2?'hf:'hff;
			if(lv_tsize!=3 && lv_burst_type!=0)begin			// 8-byte write and burst mode is not FIXED;
				//actual_data=actual_data<<(x);
				write_strobe=write_strobe<<(lv_data.addr[`byte_offset:0]);
			end
			//lv_data.addr[2:0]=0; // also make the address 64-bit aligned
            `ifdef verbosity>2 $display("Start Write lv_burst_type: %b strb: %h",lv_burst_type,write_strobe); `endif

			
			Bool lv_last= True;
			if(lv_burst_len>0) begin // only enable the next rule when doing a write in burst mode.
				rg_burst_count<=rg_burst_count+1;
				lv_last= False;
				`ifdef verbosity>2 $display("Starting burst mode write...."); `endif
			end
			`ifdef verbosity>2
			else begin
				$display("Performing a single write...");
			end 
			`endif

      rg_write_strobe <= write_strobe; 	//Write strobe needs to be rotated so that burst writes are sent correctly, storing write_strobe in a register.
      rg_tsize <= lv_tsize; 						//Storing rg_tsize in a register.
			rg_burst_type<= lv_burst_type;		//FIXED or INCR
			// Generate a Write 
			let write_data = AXI4_Wr_Data { wdata: actual_data , wstrb: write_strobe, wlast: lv_last, wid: {1'b1,fromInteger(chanNum)}};
			let write_addr = AXI4_Wr_Addr {	awaddr: lv_data.addr, awuser: 0,
											awlen: lv_burst_len, awsize: zeroExtend(lv_tsize), awburst: zeroExtend(lv_burst_type),
											awid: {1'b1,fromInteger(chanNum)} };

			// enqueue the request.
			xactor.i_wr_data.enq(write_data);
			xactor.i_wr_addr.enq(write_addr);
			rg_finish_write[chanNum][1]<= False;

			// Some other house keeping - removing the data from the fifos
			responseDataFs[chanNum].deq;
			destAddrFs[chanNum].deq;	//dequeing this FIFO will cause startRead to fire.
			`ifdef verbosity>2 $display ($time,"\tDMA[%0d] startWrite addr: %h data: %h", chanNum,lv_data.addr,responseDataFs[chanNum].first); `endif
		endrule

		//This rule is used to send burst write data. The explicit condition ensures that we
		//send burst length number of data i.e. rg_burst_count>1. When rg_burst_count = the burst
		//length specified in dma_ccr, rg_burst_count becomes 0. Since rg_burst_count!=0 infers
		//lesser hardware compared to rg_burst_count>1, we write that as the explicit condition.
		rule rl_send_burst_write_data(rg_burst_count!=0);// && dma_ccr[chanNum][31:32-`Burst_length_bits]!='d0);
			Bool lv_last= rg_burst_count==dma_ccr[chanNum][31:32-`Burst_length_bits];
			/*==  Since this is going to always be a line write request in burst mode No need of shifting data and address=== */

			Bit#(TDiv#(data_width,8)) write_strobe;
			if(rg_burst_type!=0)	//If burst type is not FIXED, then generate the write strobe
      	write_strobe = rotateBitsBy(rg_write_strobe,1<<rg_tsize); //~Vinod Rotating write_strobe by awsize
			else	//Writing to the same address which implies that write strobe doesn't change
				write_strobe= rg_write_strobe;
			let w  = AXI4_Wr_Data {wdata:  responseDataFs[chanNum].first, wstrb: write_strobe , wlast: lv_last, wid: {1'b1, fromInteger(chanNum)} };
      		xactor.i_wr_data.enq(w);
			`ifdef verbosity>2 $display ($time,"\tDMA[%0d] startWrite Burst data: %h rg_burst_count: %d dma_ccr[31:24]: %d", chanNum,responseDataFs[chanNum].first,  rg_burst_count, dma_ccr[chanNum][31:24]); `endif
			if(lv_last)begin
				`ifdef verbosity>2 $display("Last data received..."); `endif
				rg_burst_count<=0;
			end
			else begin
				rg_burst_count<=rg_burst_count+1;
			end
			responseDataFs[chanNum].deq;
            rg_write_strobe <= write_strobe;
		endrule


		// This rule waits for the write to finish
		rule rl_finishWrite( (xactor.o_wr_resp.first.bid == {1'b1, fromInteger(chanNum)}) &&
		(xactor.o_wr_resp.first.bresp==AXI4_OKAY) );
			let x<- pop_o(xactor.o_wr_resp) ;			 // take the response data and finish
			currentWriteRs[chanNum][0]<= currentWriteRs[chanNum][0] + 1;
			`ifdef verbosity>2 $display ("DMA[%0d]: finishWrite", chanNum); `endif
			rg_finish_write[chanNum][0]<= True;
		endrule
		
		rule rl_cndtr_is_zero;
			rg_is_cndtr_zero[chanNum][0]<= (dma_cndtr[chanNum]==0);
		endrule

	endrules;
	endfunction

	function Rules generateTransferDoneRules( Integer chanNum );
		return
		rules
			// Conditions to mark when transfer is done.
			// This rule will not fire for when circular mode because dma_cndtr[chanNum] is updated 
			// in the next cycle, which is before currentWriteRs can possibly get updated.
			rule markTransferDone (	dma_ccr[chanNum][0]==1 &&	//if the channel is enabled
									rg_is_cndtr_zero[chanNum][0] &&	//if the remaining data to transfer is 0
									currentWriteRs[chanNum][0]== currentReadRs[chanNum][0]) ;	//the final write has finished
				//dmaEnabledRs[chanNum]._write (False) ; 
				currentWriteRs[chanNum][0] <= 0 ;
				currentReadRs[chanNum][0]  <= 0 ;
				`ifdef verbosity>1 $display ("DMA[%0d]: transfer done int_enable:%b dma_isr: %b", chanNum, dma_ccr[chanNum][3:1], dma_isr[chanNum]); `endif
			endrule
		endrules ;
	endfunction

	// Generate the rules, place them in priority order
	//
	Rules ruleset = emptyRules;

	for (Integer ch = 0; ch < valueof (numChannels); ch = ch + 1)
		ruleset = rJoinDescendingUrgency (ruleset,
					generatePortDMARules( m_xactor, ch));

	//
	for (Integer ch = 0; ch < valueof (numChannels); ch = ch + 1)
		ruleset = rJoinDescendingUrgency (ruleset,
					generateTransferDoneRules(ch));



	(* descending_urgency = "rl_write_response_error, rl_read_response_error" *)
	rule rl_write_response_error(m_xactor.o_wr_resp.first.bresp == AXI4_SLVERR || m_xactor.o_wr_resp.first.bresp == AXI4_DECERR);
		wr_bus_err<= tagged Valid m_xactor.o_wr_resp.first.bid;
	endrule

	rule rl_read_response_error(m_xactor.o_rd_data.first.rresp == AXI4_SLVERR || m_xactor.o_rd_data.first.rresp == AXI4_DECERR);
		wr_bus_err<= tagged Valid m_xactor.o_rd_data.first.rid;
	endrule

	rule handle_interrupts;	//interrupts will be raised only when channel is enabled
		for (Integer chanNum = 0; chanNum < valueOf (numChannels); chanNum = chanNum + 1) begin
			Bit#(4) chan_isr= 'd0;	//TEIF, HTIF, TCIF, GIF
			Bit#(1) lv_is_chan_enabled= dma_ccr[chanNum][0];
			//$display("*** chan: %d en: %d dma_cndtr: %h rg_cndtr: %h",chanNum, dma_ccr[chanNum][0], dma_cndtr[chanNum], rg_cndtr[chanNum]);
			if(wr_bus_err matches tagged Valid .chan_num &&& fromInteger(chanNum)=={1'b1, chan_num[2:0]}) begin
				chan_isr[3]=1;
				`ifdef verbosity>1 $display("Bus error on channel %d",chanNum); `endif
			end
			if(lv_is_chan_enabled==1) begin
				if(currentWriteRs[chanNum][1]==currentReadRs[chanNum][1]) begin	//once the read and write transactions are over
					if(rg_is_cndtr_zero[chanNum][0]) //TODO check what happens if you read from port 1 here
						chan_isr[1]= 1;
				end
				//The Half Transfer Complete will be set when half transfer is complete.
				//Since dma_cndtr is modified by startRead rule, though it would've become half,
				//the transaction wouldn't be complete till the write is over. Also, by the time write is over, 
				//another read request could've been issued. Therefore, we have the below condition.
				if(dma_cndtr[chanNum]<=(rg_cndtr[chanNum]>>1) && rg_finish_write[chanNum][1]) begin
					chan_isr[2]= 1;
				end
			end
			chan_isr[0]= chan_isr[3] | chan_isr[2] | chan_isr[1];	//Setting the GIF
			chan_isr= dma_isr[chanNum][3:0] | chan_isr; //Sticky nature of interrupts. Should be cleared by software using ifcr.

			`ifdef verbosity>2	
			if(dma_ifcr[chanNum]!=0) begin
				$display($time,"DMA[%d] IFCR:%b ISR:%b", chanNum, ~(dma_ifcr[chanNum]), chan_isr);
			end	
			`endif
			//The bits in IFCR represent the interrupts that need to be cleared
			chan_isr= chan_isr & ~(dma_ifcr[chanNum]);
			dma_isr[chanNum]<= chan_isr;
//            $display("chan_isr %b dma_cndtr[%d] :%h",chan_isr,chanNum,dma_cndtr[chanNum]);
		end
	endrule


	// Rules and other code to interface config port /////////////

	// Add a zero-size register as a default for invalid addresses
	Reg#(Bit#(0)) nullReg <- mkReg( ? ) ;

// ----------------------------------------------------------------
// At times it is best to consider registers as completely homogeneous,
// so that they can be accessed as a bit pattern with no internal
// structure.  These functions convert reg interfaces based on a
// structured type to reg interfaces based on a bit pattern of at
// least the same width.

function Reg#(Bit#(n)) regAToRegBitN( Reg#(a_type) rin )
	provisos ( Bits#( a_type, asize),
			   Add#(asize,xxx,n) ) ;

	return
	interface Reg
		method Bit#(n) _read ();
			a_type tmp =  rin._read()  ;
			return zeroExtend (pack( tmp )) ;
		endmethod
		method Action _write( Bit#(n) din );
			rin._write( unpack( truncate(din) )) ;
		endmethod
	endinterface ;
endfunction

// This function converts a Vector of (upto 7) Registers to a single Register
//TODO For now, this function has to be manually changed when num of channels change.
function Reg#(Bit#(TMul#(3,q))) vectorToRegN(Vector#(3,Reg#(Bit#(q))) inpV);
	return concatReg3(inpV[2], inpV[1], inpV[0]);
	//return asReg(zeroExtend(pack(inpV)));
endfunction
/*function Reg#(Bit#(TMul#(numChannels,q))) vectorToRegN(Vector#(numChannels,Reg#(Bit#(q))) inpV);
		return concatReg7(inpV[6], inpV[5], inpV[4], inpV[3], inpV[2], inpV[1], inpV[0]);
endfunction*/

//This doesn't work
/*function Reg#(Bit#(TMul#(param,q))) vectorToRegN(Vector#(param,Reg#(Bit#(q))) inpV)
	provisos(Add#(param,a__,7));
	let val_param= valueOf(param);
	if(val_param==1)
		return regAToRegBitN(concatReg7(nullReg, nullReg, nullReg, nullReg, nullReg, nullReg, inpV[0]));
	else if(val_param==2)
		return regAToRegBitN(concatReg7(nullReg, nullReg, nullReg, nullReg, nullReg, inpV[1], inpV[0]));
	else if(val_param==3)
		return regAToRegBitN(concatReg7(nullReg, nullReg, nullReg, nullReg, inpV[2], inpV[1], inpV[0]));
	else if(val_param==4)
		return regAToRegBitN(concatReg7(nullReg, nullReg, nullReg, inpV[3], inpV[2], inpV[1], inpV[0]));
	else if(val_param==5)
		return regAToRegBitN(concatReg7(nullReg, nullReg, inpV[4], inpV[3], inpV[2], inpV[1], inpV[0]));
	else if(val_param==6)
		return regAToRegBitN(concatReg7(nullReg, inpV[5], inpV[4], inpV[3], inpV[2], inpV[1], inpV[0]));
	else if(val_param==7)
		return regAToRegBitN(concatReg7(inpV[6], inpV[5], inpV[4], inpV[3], inpV[2], inpV[1], inpV[0]));
	else 
		return regAToRegBitN( nullReg );
	//return asReg(zeroExtend(pack(inpV)));
endfunction*/

//This doesn't work
/*function Reg#(Bit#(TMul#(param,q))) vectorToRegN(Vector#(param,Reg#(Bit#(q))) inpV)
	provisos(Add#(param,a__,7));
	let val_param= valueOf(param);
	if(val_param==1)
		return inpV[0];
	else if(val_param==2)
		return concatReg2(inpV[1], inpV[0]);
	else if(val_param==3)
		return concatReg3(inpV[2], inpV[1], inpV[0]);
	else if(val_param==4)
		return concatReg4(inpV[3], inpV[2], inpV[1], inpV[0]);
	else if(val_param==5)
		return concatReg5(inpV[4], inpV[3], inpV[2], inpV[1], inpV[0]);
	else if(val_param==6)
		return concatReg6(inpV[5], inpV[4], inpV[3], inpV[2], inpV[1], inpV[0]);
	else if(val_param==7)
		return concatReg7(inpV[6], inpV[5], inpV[4], inpV[3], inpV[2], inpV[1], inpV[0]);
	else 
		return regAToRegBitN( nullReg );
	//return asReg(zeroExtend(pack(inpV)));
endfunction*/

//This function converts any Reg type, including vectors of registers to a single register.
//The first parameter is a Reg type derivative, and the second parameter indicates if the register
//can be read in the user mode or not. If dma_ccr[chanNum][16]=1, it implies that all registers of
//channel "chanNum" can be accessed in the User mode; =0 implies a AXI_SLVERR.
//However, the attributes of the following 3 registers are fixed:
//	dma_isr: User read only
//	dma_ifcr: User cannot access
//	dma_cselr: User read only
	function Tuple2#(Reg#(Bit#(config_data_width)), Bool) can_return( Reg#(b_type) inp, Bit#(1) user_readable)
	provisos (Bits#(b_type, bsize),
						Add#(bsize,xxx,config_data_width));
//		if(valueOf(numChannels)>channel)
			return tuple2(regAToRegBitN(inp), unpack(user_readable));
//		else
//			return tuple2(regAToRegBitN( nullReg ), False);
	endfunction


	// For ease of development we want all registers to look like 64
	// bit resister-- the data size of the config socket.
	// Create function to map from address to specific registers

	//TODO Make sure that writes to cndtr, cpar and cmar do not happen when channel is enabled.
	function Tuple2#(Reg#(Bit#(config_data_width)), Bool) selectReg( Bit#(config_addr_width) addr);
    Bit#(5) taddr= addr[7:3];
    return
    case ( taddr )
 
	  	//8'h08 : if(valueOf(numChannels)>1) begin return tuple2(regAToRegBitN( dma_ccr[0] ), True); end  //32-bit
	  	//				else return tuple2(regAToRegBitN( nullReg ), False);
	  	'd0 : return can_return(dma_ccr[0], dma_ccr[0][15]);   //32-bit
      'd1 : return can_return(dma_cndtr[0], dma_ccr[0][15]); //16-bit -- 32-bit Addr 
      'd2 : return can_return(dma_cpar[0], dma_ccr[0][15]); //64-bit
      'd3 : return can_return(dma_cmar[0], dma_ccr[0][15]); //64-bit
 
      'd4 : return can_return(dma_ccr[1], dma_ccr[1][15]);
      'd5 : return can_return(dma_cndtr[1], dma_ccr[1][15]);
      'd6 : return can_return(dma_cpar[1], dma_ccr[1][15]);
      'd7 : return can_return(dma_cmar[1], dma_ccr[1][15]);
 
      'd8 : return can_return(dma_ccr[2], dma_ccr[2][15]);
      'd9 : return can_return(dma_cndtr[2], dma_ccr[2][15]);
      'd10 : return can_return(dma_cpar[2], dma_ccr[2][15]);
      'd11 : return can_return(dma_cmar[2], dma_ccr[2][15]);
/* 
      'd12 : return can_return(dma_ccr[3], dma_ccr[3][15]);
      'd13 : return can_return(dma_cndtr[3], dma_ccr[3][15]);
      'd14 : return can_return(dma_cpar[3], dma_ccr[3][15]);
      'd15 : return can_return(dma_cmar[3], dma_ccr[3][15]);
 
      'd16 : return can_return(dma_ccr[4], dma_ccr[4][15]);
      'd17 : return can_return(dma_cndtr[4], dma_ccr[4][15]);
      'd18 : return can_return(dma_cpar[4], dma_ccr[4][15]);
      'd19 : return can_return(dma_cmar[4], dma_ccr[4][15]);
 
      'd20 : return can_return(dma_ccr[5], dma_ccr[5][15]);
      'd21 : return can_return(dma_cndtr[5], dma_ccr[5][15]);
      'd22 : return can_return(dma_cpar[5], dma_ccr[5][15]);
      'd23 : return can_return(dma_cmar[5], dma_ccr[5][15]);
 
      'd24 : return can_return(dma_ccr[6], dma_ccr[6][15]);
      'd25 : return can_return(dma_cndtr[6], dma_ccr[6][15]);
      'd26 : return can_return(dma_cpar[6], dma_ccr[6][15]);
      'd27 : return can_return(dma_cmar[6], dma_ccr[6][15]);
*/ 
      'd28 : return can_return(vectorToRegN( dma_isr ), 1'b1);
      'd29 : return can_return(vectorToRegN( dma_ifcr ), 1'b0);
      'd30 : return can_return(vectorToRegN( dma1_cselr ), 1'b1);

      default: return tuple2(regAToRegBitN( nullReg ), False);
    endcase ;
  endfunction

	function Tuple2#(Bit#(TLog#(numChannels)), Bool) ccr_channel_number (Bit#(config_addr_width) addr);
    Bit#(8) taddr= addr[7:0];
		let val_numChannels= valueOf(numChannels);
		if(val_numChannels >0 && taddr=='h0)
			return tuple2('d0, True);
		else if(val_numChannels >1 && taddr =='h20)
			return tuple2('d1, True);
		else if(val_numChannels >2 && taddr =='h40)
			return tuple2('d2, True);
		else if(val_numChannels >3 && taddr =='h60)
			return tuple2('d3, True);
		else if(val_numChannels >4 && taddr =='h80)
			return tuple2('d4, True);
		else if(val_numChannels >5 && taddr =='hA0)
			return tuple2('d5, True);
		else if(val_numChannels >6 && taddr =='hC0)
			return tuple2('d6, True);
		else
			return tuple2(?, False);
	endfunction
  /*  case ( taddr )
      8'h08 : return 0;
      8'h20 : return 1;
      8'h38 : return 2;
      8'h50 : return 3;
      8'h68 : return 4;
      8'h80 : return 5;
      8'h98 : return 6;
      default: return 'd-1;
    endcase;
  endfunction*/
	
	Rules writeConfig = (rules
		rule writeConfig;
			let data= wr_write_data;
			let size= wr_write_access_size;
			let addr= wr_write_addr;
			let prot= wr_write_prot;

			Bool lv_valid_access= True;
			Bool lv_send_response= True;

			// Select and write the register
			let selectReg_address= addr&{'1,3'd0};	//Generating a 64-bit aligned address
			let lv1= selectReg(selectReg_address);
			let thisReg = tpl_1(lv1);
			

			Bit#(config_data_width) mask=size==Byte?'hff:size==HWord?'hFFF:size==Word?'hFFFFFFFF:'1;	
			/*data= case (size)
          		Byte: duplicate(data[7:0]);
          		HWord: duplicate(data[15:0]);
          		Word: duplicate(data[31:0]);
        		endcase;*/

			Bit#(6) shift_amt=zeroExtend(addr[2:0])<<3;
      mask=mask<<shift_amt;
      Bit#(config_data_width) datamask=data & mask;	
      let notmask=~mask;
			data= (thisReg & notmask)|datamask; 

			let lv_ccr_channel_number_tuple= ccr_channel_number(selectReg_address);
			let lv_ccr_channel_number=tpl_1(lv_ccr_channel_number_tuple);
			if(prot==False) begin		//If unprivileged access
				`ifdef verbosity>1 $display("DMA: Unpriviliged access trying to change config registers of channel %d",lv_ccr_channel_number); `endif
				lv_valid_access= False;
			end
			else if(selectReg_address=='hE0) begin 	//If writing to DMA_ISR
				`ifdef verbosity>1 $display("DMA: Trying to change config registers of channel %d when the channel is active",lv_ccr_channel_number); `endif
				lv_valid_access= False;
			end
			//If channel is enabled and write happening other than disabling current channel
			else if( ((dma_ccr[lv_ccr_channel_number] & 'd1) == 1) && !(tpl_2(lv_ccr_channel_number_tuple) && data[0]==0) ) begin
				`ifdef verbosity>1 $display("DMA: Trying to change config registers of channel %d when the channel is active",lv_ccr_channel_number); `endif
				lv_valid_access= False;
			end
			else begin
				lv_valid_access= True;
			end

			`ifdef verbosity>1 $display ($time,"\tDMA writeConfig addr: %0h data: %0h ccr_chan_num: %d", addr, data, lv_ccr_channel_number); `endif
			if(lv_valid_access && tpl_2(lv_ccr_channel_number_tuple)==True) begin 	//if trans is valid, and the current write is happening to one of the channel's CCR.
				if(data[0]==1) begin			//if the channel is being enabled
					  if((dma_ccr[lv_ccr_channel_number] & 'd1)!=1) begin	//If the channel is not already enabled
					  	  rg_cpa[lv_ccr_channel_number] <= dma_cpar[lv_ccr_channel_number];	//peripheral address is copied
					  	  rg_cma[lv_ccr_channel_number] <= dma_cmar[lv_ccr_channel_number];	//memory address is copied
					  	  rg_cndtr[lv_ccr_channel_number]<= dma_cndtr[lv_ccr_channel_number];	//the cndtr value is saved
					  	  rg_disable_channel<= tuple2(False,?);
								`ifdef verbosity>1 $display("----------------------- ENABLING DMA CHANNEL %d", lv_ccr_channel_number," -----------------------"); `endif
            	            
        	  	  		/*Bit#(3) cmar_align = dma_cmar[lv_ccr_channel_number][2:0]; 
        	  	  		Bit#(3) cpar_align = dma_cpar[lv_ccr_channel_number][2:0]; 
        	  	  		
        	  	  		//data[9:8] and data[11:10] gives transfer size supposedly. Using K-Maps --Possibility of a bug?
        	  	  		bit cmar_is_aligned =  fn_aligned_addr(cmar_align, data[11:10]);
        	  	  		bit cpar_is_aligned =  fn_aligned_addr(cpar_align, data[9:8]); 

        	  	  		if((cmar_is_aligned & cpar_is_aligned)==0) begin
        	  	  		  lv_bresp = AXI4_DECERR; //DECERR for Unaligned addresses
											`ifdef verbosity>1 $display("\tAXI4_DECERR\n"); `endif
        	  	  		end

										`ifdef verbosity>2 $display("cmar_is_aligned: %b cpar_is_aligned: %b isr: %b",cmar_is_aligned,cpar_is_aligned, dma_isr[lv_ccr_channel_number]); `endif
					  	    	*/	
        	  	  	`ifdef verbosity>1 
        	  	  		if(data[4]==0) begin
					  	  	$display("SOURCE: Peripheral  Addr: 'h%0h Transfer size: 'b%b Incr: %b",dma_cpar[lv_ccr_channel_number], data[9:8], data[6]);
					  	  	$display("DEST  : Memory      Addr: 'h%0h Transfer size: 'b%b Incr: %b",dma_cmar[lv_ccr_channel_number], data[11:10], data[7]);
					  	  end
					  	  else if(data[14]==0) begin
					  	  	$display("SOURCE: Memory      Addr: 'h%0h Transfer size: 'b%b Incr: %b",dma_cmar[lv_ccr_channel_number], data[11:10], data[7]);
					  	  	$display("DEST  : Peripheral  Addr: 'h%0h Transfer size: 'b%b Incr: %b",dma_cpar[lv_ccr_channel_number], data[9:8], data[6]);
					  	  end
					  	  else begin
					  	  	$display("SOURCE: Memory  Addr: 'h%0h Transfer size: 'b%b Incr: %b",dma_cmar[lv_ccr_channel_number], data[11:10], data[7]);
					  	  	$display("DEST  : Memory  Addr: 'h%0h Transfer size: 'b%b Incr: %b",dma_cpar[lv_ccr_channel_number], data[9:8], data[6]);
					  	  end
					  	  $display("Priority level: 'b%b Circular mode: %b CNDTR: 'h%h", data[13:12], data[5], dma_cndtr[lv_ccr_channel_number]);
							`endif
					  end
					  else if(data[31:0]==dma_ccr[lv_ccr_channel_number])begin	//Enabling a channel that is already enabled with same config
					  	  lv_valid_access= True;
					  end
					  else begin	//Enabling a channel that is already enabled with different config
					  	  lv_valid_access= False;
					  end
				end
				else begin //the channel is being disabled
					//TODO since it is a CReg, what if we check in port [1]?
					if(currentReadRs[lv_ccr_channel_number][0]!=currentWriteRs[lv_ccr_channel_number][0]) begin	//there is an on going transaction
						rg_disable_channel<= tuple2(True, lv_ccr_channel_number);
						lv_send_response= False;
						rg_writeConfig_ccr<= tuple2(lv_ccr_channel_number, data);
						`ifdef verbosity>1 $display("----------------------- DISABLING DMA CHANNEL %d before transactions are over", lv_ccr_channel_number," -----------------------"); `endif
					end
					else begin	// no pending transaction
						lv_valid_access= True;
						`ifdef verbosity>2 $display("----------------------- DISABLING DMA CHANNEL %d", lv_ccr_channel_number," -----------------------"); `endif
						//clear the local registers
						rg_is_cndtr_zero[lv_ccr_channel_number][0]<= True;
					end
				end
			end

			// Now generate the response and enqueue
			if(lv_send_response) begin
				thisReg <= data;
				ff_write_resp.enq(lv_valid_access);
			end
		endrule
	endrules);

	//Rule to send response to processor that the dma channel has been disabled after the on going transactions are over
	Rules rl_send_chan_disabled_to_proc = (rules
		rule rl_send_chan_disabled_to_proc(tpl_1(rg_disable_channel) && currentReadRs[tpl_2(rg_disable_channel)][1]==currentWriteRs[tpl_2(rg_disable_channel)][1]);
			rg_disable_channel<= tuple2(False,?);
			dma_ccr[tpl_1(rg_writeConfig_ccr)]<= truncate(tpl_2(rg_writeConfig_ccr));
			ff_write_resp.enq(True);
		endrule
	endrules);

	//writeConfig gets highest priority since we do not want the core to stall 
	ruleset= rJoinDescendingUrgency(writeConfig,ruleset); 

	//writeConfig if more urgent than rl_send_chan_disabled_to_proc
	ruleset= rJoinDescendingUrgency(ruleset, rl_send_chan_disabled_to_proc); 

	// A rule for reading a configuration register
	//TODO need to add preempts with writeConfig? or mutually_exclusive? Because both these rules will never fire together.
	// If we do not put any attributes, won't two instances of selectReg get synthesized?
	rule readConfig;
		// Select the register
		let lv1= selectReg(wr_read_addr);
		let thisReg = tpl_1(lv1);
		let read_resp= tpl_2(lv1);
		let lv_prot= wr_read_prot;
		Bit#(config_data_width) lv_data;
		Bool lv_valid_access;

		if(lv_prot || read_resp)	begin //If privileged access; or in unprivileged access, if the register access is granted
			lv_valid_access= True;
			if(wr_read_access_size==Byte)
				lv_data=duplicate(thisReg[7:0]);
			else if(wr_read_access_size==HWord)
				lv_data=duplicate(thisReg[15:0]);
			else if(wr_read_access_size==DWord)
				lv_data=duplicate(thisReg[31:0]);
			else
				lv_data= thisReg;
		end
		else begin
			lv_valid_access= False;
			lv_data= 'd0;
		end

		ff_read_resp.enq(tuple2(lv_valid_access, lv_data));
	endrule


	// Add the rules to this module
	addRules (ruleset);

	//Note that unknownConfig rule is not needed here because all the FIFOs will
	//be empty and hence neither writeConfig nor readConfig will fire.

	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	//
	// Create the interfaces by connecting the axi side interfaces
	// of the transactors to it.

	
	method Action read_req(Bit#(config_addr_width) addr, AccessSize size, Bool prot);
		wr_read_addr<= addr;
		wr_read_access_size<= size;
		wr_read_prot<= prot;
	endmethod

	method ActionValue#(Tuple2#(Bool, Bit#(config_data_width))) read_resp;
		ff_read_resp.deq;
		return ff_read_resp.first;
	endmethod

	method Action write_req(Bit#(config_addr_width) addr, Bit#(config_data_width) data, AccessSize size, Bool prot);
		wr_write_addr<= addr;
		wr_write_data<= data;
		wr_write_access_size<= size;
		wr_write_prot<= prot;
	endmethod

	method ActionValue#(Bool) write_resp;
		ff_write_resp.deq;
		return ff_write_resp.first;
	endmethod
	
	//This method receives various interrupts from the peripheral devices and gives it to the DMA
	method Action interrupt_from_peripherals(Bit#(numPeripherals) pint);
		wr_peripheral_interrupt<= unpack(pint);
	endmethod

	//This method returns the interrupt generated by the DMA if any of the channels raise an
	//interrupt. This can be found by checking the Interrupt Status Register (ISR) of every channel, 
	//and checking if that corresponding interrupt is not masked.
	//Raise the interrupt of a particular channel if any of the interrupts are active (in ISR) and 
	//are not masked(in CCR).
  interface  interrupt_to_proc= interface Get
  method ActionValue#(Bit#(1)) get();
			Bit#(numChannels) lv_interrupt_to_processor;
			for(Integer chanNum= 0; chanNum < valueof(numChannels); chanNum= chanNum + 1) begin
				let lv_dma_ccr= dma_ccr[chanNum];
				//The bits in IFCR represent which need to be cleared
				let lv_intr_TE_HT_TC_enable= lv_dma_ccr[3:1];

				//The bits in ISR represent which interrupts are active right now
				Bit#(3) active_interrupts= {lv_intr_TE_HT_TC_enable} & dma_isr[chanNum][3:1];
				//if(lv_dma_ccr[0]==1) begin
					//$display("DMA chanNum: %d int_enable: %b dma_isr: %b\n",chanNum,lv_intr_TE_HT_TC_enable, dma_isr);
				//end
				lv_interrupt_to_processor[chanNum]= |(active_interrupts);	//TODO change this to | of all
			end
			`ifdef verbosity>2  
			if(lv_interrupt_to_processor!=0) begin
				$display("intrrr: %b",lv_interrupt_to_processor);
			end
			`endif
			return |(lv_interrupt_to_processor);
	endmethod
  endinterface;
	interface master= m_xactor.axi_side;
endmodule

interface Ifc_DMA_AXI4#(numeric type addr_width, numeric type data_width, numeric type user_width, numeric type config_addr_width, numeric type config_data_width, numeric type numChannels, numeric type numPeripherals);
	interface AXI4_Master_IFC#(addr_width, data_width, user_width) master;
	interface AXI4_Slave_IFC#(config_addr_width, config_data_width, user_width) slave;
	method Action interrupt_from_peripherals(Bit#(numPeripherals) pint);
	interface Get#(Bit#(1)) interrupt_to_proc;
endinterface

module mkDMA_AXI4(Ifc_DMA_AXI4#(addr_width, data_width, user_width, config_addr_width, config_data_width, numChannels, numPeripherals))
provisos (Add#(a__, TLog#(numPeripherals), 4),
	 				//Add#(numChannels, xyz__, 7),
	 				Add#(numChannels, 0, 3),
					//Add#(TMul#(numChannels, 4), a__, 64),
					Add#(b__, 8, config_addr_width),
					Add#(7, j__, addr_width),
					Add#(k__, 3, config_addr_width),	
					Add#(addr_width, g__, config_data_width),
					Add#(c__, 32, config_data_width),
			    Mul#(8, d__, config_data_width),
  			  Mul#(16, e__, config_data_width),
  				Mul#(32, f__, config_data_width),
					Add#(28, h__, config_data_width),
					Add#(16, i__, config_data_width),
					Add#(12, l__, config_data_width)	//for numChannels=3
);
		User_ifc#(addr_width, data_width, user_width, config_addr_width, config_data_width, numChannels, numPeripherals) dma <- mkDMA;
		AXI4_Slave_Xactor_IFC#(config_addr_width, config_data_width, user_width)  s_xactor <- mkAXI4_Slave_Xactor();

		Reg#(Bool) rg_is_rdburst[2] <- mkCReg(2,False);
		Reg#(Bit#(4)) rg_arid[2] <- mkCReg(2,?);
		Reg#(Bit#(8)) rg_rdburst_count <- mkReg(0);
		
		Reg#(Bool) rg_is_wrburst[2] <- mkCReg(2,False);
		Reg#(Bit#(4)) rg_awid[2] <- mkCReg(2,?);
		Reg#(Bit#(8)) rg_wrburst_count <- mkReg(0);

//	method Action read_req(Bit#(addr_width) addr, AccessSize size);
//	method Tuple2#(Bool, Bit#(data_width)) read_resp;
//	method Action write_req(Bit#(addr_width) addr, Bit#(data_width) data, AccessSize size);
//	method Bool write_resp;

		rule read_req(rg_is_rdburst[0]==False);
      let req <- pop_o(s_xactor.o_rd_addr);
			if(req.arlen!=0)
				rg_is_rdburst[0]<= True;
			else begin
				rg_is_rdburst[0]<= False;
      	dma.read_req(req.araddr, unpack(truncate(req.arsize)), unpack(req.arprot[0]));
      	rg_arid[0]<= req.arid;
				rg_rdburst_count<= req.arlen;
			end
		endrule

		rule read_resp(!rg_is_rdburst[1]);
      let {succ,data}<- dma.read_resp;
      let r = AXI4_Rd_Data {rresp: succ? AXI4_OKAY:AXI4_SLVERR, rid: rg_arid[1],
														rlast: True, rdata: data, ruser: ?};
      s_xactor.i_rd_data.enq(r);
		endrule
	
		rule burst_read_resp(rg_is_rdburst[1]);
			Bool lv_rlast;

    	if(rg_rdburst_count==0) begin
				lv_rlast= True;
				rg_is_rdburst[1]<= False;
			end
			else begin
				lv_rlast= False;
				rg_rdburst_count<= rg_rdburst_count-1;
			end

      let r = AXI4_Rd_Data {rresp: AXI4_SLVERR, rid: rg_arid[1],
														rlast: lv_rlast, rdata: ?, ruser: ?};
      s_xactor.i_rd_data.enq(r);
    endrule

    rule write_req(rg_wrburst_count==0);
      let aw <- pop_o(s_xactor.o_wr_addr);
      let w <- pop_o(s_xactor.o_wr_data);
      if(aw.awlen!=0)
        rg_is_wrburst[0]<= True;
		  else begin
				rg_is_wrburst[0]<= False;
      	dma.write_req(aw.awaddr,w.wdata,unpack(truncate(aw.awsize)),unpack(aw.awprot[0]));
				rg_awid[0]<= aw.awid;
				rg_wrburst_count<= aw.awlen;
			end
		endrule

		rule write_resp(!rg_is_wrburst[1]);
      let succ<- dma.write_resp;
      let r = AXI4_Wr_Resp {bresp: succ?AXI4_OKAY:AXI4_SLVERR, buser: 0 , bid:rg_awid[1]};
      s_xactor.i_wr_resp.enq (r);
		endrule

		rule burst_write_resp(rg_is_wrburst[1]);
      let dummy<-pop_o(s_xactor.o_wr_data);
			Bool lv_rlast;
    	if(rg_wrburst_count==0) begin
					lv_rlast= True;
					rg_is_wrburst[1]<= False;
			end
			else begin
				lv_rlast= False;
				rg_wrburst_count<= rg_wrburst_count-1;
			end

			if(lv_rlast) begin
      	let r = AXI4_Wr_Resp {bresp: AXI4_SLVERR, buser: 0 , bid:rg_awid[1]};
      	s_xactor.i_wr_resp.enq(r);
			end
    endrule

		interface master= dma.master;
    interface slave= s_xactor.axi_side;
		//method interrupt_from_peripherals=dma.interrupt_from_peripherals;	//TODO see if this works
		method Action interrupt_from_peripherals(Bit#(numPeripherals) pint);
			dma.interrupt_from_peripherals(pint);
		endmethod
		interface interrupt_to_proc= dma.interrupt_to_proc;
endmodule

interface Ifc_DMA_AXI4_Lite#(numeric type addr_width, numeric type data_width, numeric type user_width,
                             numeric type config_addr_width, numeric type config_data_width,
                             numeric type numChannels, numeric type numPeripherals);
	interface AXI4_Master_IFC#(addr_width, data_width, user_width) master;
 	interface AXI4_Lite_Slave_IFC#(config_addr_width, config_data_width, user_width) slave;
	method Action interrupt_from_peripherals(Bit#(numPeripherals) pint);
	interface Get#(Bit#(1)) interrupt_to_proc;
endinterface

module mkDMA_AXI4_Lite(Ifc_DMA_AXI4_Lite#(addr_width, data_width, user_width, config_addr_width, config_data_width, numChannels, numPeripherals))
provisos (Add#(a__, TLog#(numPeripherals), 4),
	 				//Add#(numChannels, xyz__, 7),
	 				Add#(numChannels, 0, 3),
					//Add#(TMul#(numChannels, 4), a__, 64),
					Add#(b__, 8, config_addr_width),
					Add#(7, j__, addr_width),
					Add#(k__, 3, config_addr_width),	
					Add#(addr_width, g__, config_data_width),
					Add#(c__, 32, config_data_width),
			    Mul#(8, d__, config_data_width),
  			  Mul#(16, e__, config_data_width),
  				Mul#(32, f__, config_data_width),
					Add#(28, h__, config_data_width),
					Add#(16, i__, config_data_width),
					Add#(12, l__, config_data_width)	//for numChannels=3
);
		User_ifc#(addr_width, data_width, user_width, config_addr_width, config_data_width, numChannels, numPeripherals) dma <- mkDMA;
		AXI4_Lite_Slave_Xactor_IFC#(config_addr_width, config_data_width, user_width)  s_xactor <- mkAXI4_Lite_Slave_Xactor();

	 	rule axi_read_req;
	 		let req <- pop_o(s_xactor.o_rd_addr);
			dma.read_req(req.araddr,unpack(truncate(req.arsize)), unpack(req.arprot[0]));
	 	endrule

		rule axi_read_resp;
      let {succ,data}<- dma.read_resp;
	 		let r = AXI4_Lite_Rd_Data {rresp: succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, rdata: data, ruser: 0};
	 		s_xactor.i_rd_data.enq(r);
		endrule
		
	 	rule axi_write_req;
	 		let aw <- pop_o(s_xactor.o_wr_addr);
	 		let w <- pop_o(s_xactor.o_wr_data);
	 		dma.write_req(aw.awaddr,w.wdata,unpack(truncate(aw.awsize)), unpack(aw.awprot[0]));
		endrule

		rule axi_write_resp;
			let succ<- dma.write_resp;
	 		let r = AXI4_Lite_Wr_Resp {bresp: succ?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, buser: 0 };
	 		s_xactor.i_wr_resp.enq (r);
	 	endrule

		interface master= dma.master;
    interface slave= s_xactor.axi_side;
		//method interrupt_from_peripherals=dma.interrupt_from_peripherals;	//TODO see if this works
		method Action interrupt_from_peripherals(Bit#(numPeripherals) pint);
			dma.interrupt_from_peripherals(pint);
		endmethod
		interface interrupt_to_proc= dma.interrupt_to_proc;
	endmodule
endpackage
