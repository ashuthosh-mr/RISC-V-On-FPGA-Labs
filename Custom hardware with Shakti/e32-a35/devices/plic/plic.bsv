package plic;
 	import Vector::*;
	import ConfigReg::*;
	import Semi_FIFOF::*;
	import AXI4_Lite_Types::*;
	import AXI4_Lite_Fabric::*;
	import AXI4_Types::*;
	import AXI4_Fabric::*;	
	import BUtils ::*;
	import ConcatReg ::*;
	import encoder ::*;
	import device_common::*;
	import GetPut ::*;

	`include "plic.defines"

/*Platform level interrupt controller:
	Refer to RISC-V privilege spec-v-1.10 chapter 7
	Memory maps of Registers
		Interrupt enable registers :
							rg_ie_0 :0C002000 
							rg_ie_1 :0C002000 
							rg_ie_2 :0C002000 
							.
							.
							.
							rg_ie_7 :0C002000 
							rg_ie_8 :0C002001 
							rg_ie_9 :0C002001 
							.
							.
		Interrupt priority registers :
							rg_priority_0 : 0C000000     //32 bits per source
							rg_priority_1 : 0C000004
							rg_priority_2 : 0C000008
							.
							.
							.
		Priority Threshold register : 
							rg_priority_threshold : 0C200000
		Claim register : 
							rg_interrupt_id : 0C200004
*/

	typedef enum {Load, Store, Atomic, Fence} Access_type deriving (Bits,Eq,FShow);

	typedef struct {
		Bit#(paddr) 			 address;
		Bit#(TLog#(TDiv#(data_width,8)))  transfer_size;	
		Bit#(1)					 u_signed;
		Bit#(TLog#(TDiv#(data_width,8)))						byte_offset;
		Bit#(data_width) write_data;
		Access_type			 ld_st;
	} UncachedMemReq#(numeric type paddr, numeric type data_width) deriving(Bits, Eq);

	interface IFC_GLOBAL_INTERRUPT_IO;
		method Action irq_frm_gateway(Bool ir);
	endinterface

	interface IFC_PROGRAM_REGISTERS#(numeric type addr_width,numeric type data_width);
		method ActionValue#(Tuple2#(Bit#(data_width), Bool)) prog_reg(UncachedMemReq#(addr_width, data_width) mem_req, AccessSize size);
	endinterface 	


interface User_ifc#(numeric type addr_width,numeric type data_width,
      numeric type no_of_ir_pins, numeric type no_of_ir_levels, numeric type no_nmi);
	method Action ifc_external_irq_io(Bit#(no_of_ir_pins) irq) ;
	interface IFC_PROGRAM_REGISTERS#(addr_width,data_width) ifc_prog_reg;
	interface Get#(Tuple2#(Bool,Bool)) intrpt_note_sb;
endinterface

//(*conflict_free = "rl_prioritise, prog_reg"*)
module mkplic#(parameter Integer slave_base)(User_ifc#(addr_width,data_width,no_of_ir_pins,no_of_ir_levels,no_nmi))
	provisos(
	Log#(no_of_ir_pins, ir_bits),
	Log#(no_of_ir_levels, priority_bits),
	Add#(1,ir_bits,x_ir_bits),
	Add#(msb_ir_bits,1,ir_bits),
	Add#(msb_ir_pins,1,no_of_ir_pins),
	Add#(msb_priority_levels,1,no_of_ir_levels),
	Add#(msb_priority_bits,1,priority_bits),
	Add#(a__, no_of_ir_levels, data_width),
	Add#(c__, no_of_ir_pins, 1024),
	Add#(d__, ir_bits, 10),
	Add#(f__, no_of_ir_levels, 1024),
	Add#(g__, priority_bits, 10),
	Add#(h__, no_of_ir_levels, 32),
	Add#(i__, data_width, 64),
	Add#(j__, no_of_ir_levels, 64),
	Add#(k__, ir_bits, 64)
	);

	let v_no_of_ir_pins = valueOf(no_of_ir_pins);
	let v_ir_bits = valueOf(ir_bits);
	let v_msb_ir_bits = valueOf(msb_ir_bits);
	let v_msb_ir_pins = valueOf(msb_ir_pins);
	let v_msb_priority = valueOf(msb_priority_levels);
	let v_data_width = valueOf(data_width);
	Bit#(addr_width) base_address = fromInteger(slave_base);


	Vector#(no_of_ir_pins,Reg#(Bool)) rg_gateway <- replicateM(mkReg(False));
	Vector#(no_of_ir_pins,Array#(Reg#(Bool))) rg_ip <- replicateM(mkCReg(2,False));


	Reg#(Bool) rg_ie[v_no_of_ir_pins];//interrupt enable 
	for(Integer i = 0; i < v_no_of_ir_pins;i=i+1)
		begin
			if(i<valueOf(no_nmi))//these are for the non maskable interrupt
				rg_ie[i] = readOnlyReg(True);// interrupt is always enabled for these interrupts 
			else
				rg_ie[i] <- mkReg(False);
		end		

	Reg#(Bit#(no_of_ir_levels)) rg_priority_low[v_no_of_ir_pins];
	for(Integer i =0; i < v_no_of_ir_pins; i=i+1)
		begin
			if(i<valueOf(no_nmi))//for non maskable interrupt
				rg_priority_low[i] = readOnlyReg(1);// these interrupts have the highest priority
			else
				rg_priority_low[i] <- mkConfigReg(0);
		end	

	Reg#(Bit#(32)) rg_priority[v_no_of_ir_pins];
	for(Integer i=0;i < v_no_of_ir_pins;i=i+1)
		rg_priority[i] = concatReg2(readOnlyReg(0), rg_priority_low[i]);

	Reg#(Bit#(no_of_ir_levels))	 rg_priority_threshold <- mkReg(0);
	Reg#(Bit#(ir_bits))	 rg_interrupt_id <- mkConfigReg(0);
	Reg#(Bool)	 rg_interrupt_valid <- mkConfigReg(False);
	Reg#(Maybe#(Bit#(ir_bits))) rg_completion_id <- mkReg(tagged Invalid);
	Reg#(Bit#(no_of_ir_pins)) rg_total_priority <- mkReg(0);
	Reg#(Bit#(1)) rg_plic_state <- mkReg(0); //TODO put an enum later
	Reg#(Bit#(no_of_ir_levels)) rg_winner_priority <- mkReg(0);
	Ifc_encoder#(no_of_ir_levels) ir_priority_encoder <- mkencoder();
	Ifc_encoder#(no_of_ir_pins) irencoder <- mkencoder();

	rule rl_prioritise(rg_plic_state==0);
		Bit#(priority_bits) winner_priority = 0;
		Bit#(ir_bits) winner_interrupts = 0;
		Bit#(x_ir_bits) ir_id_valid = 0;
		Bit#(no_of_ir_levels) lv_priority = 0;
		Bit#(no_of_ir_pins) lv_total_priority = 0;
		Bool i_valid = False;
		for(Integer i = 0; i < v_no_of_ir_pins; i = i + 1)
		 begin
			if(rg_ip[i][1] && rg_ie[i]) begin
				lv_priority = lv_priority | truncate(rg_priority[i]);//taking the OR of multiple priority registers
				`ifdef verbose $display($time,"\tInterrupt id %d and priority is %d", i, lv_priority);`endif
			end
		end
		winner_priority = ir_priority_encoder.encode(lv_priority);
		`ifdef verbose $display($time,"\t winner priority is  %d", winner_priority);`endif
		for(Integer i = 0; i < v_no_of_ir_pins; i = i + 1) begin
			if(rg_priority[i][winner_priority] == 1 && rg_ip[i][1] && rg_ie[i])
				lv_total_priority[i] = 1;
		end
		if(lv_total_priority!=0) begin
			rg_total_priority <= lv_total_priority;
			rg_plic_state <= 1;
			Bit#(no_of_ir_levels) lv_winner_priority = 0;
			lv_winner_priority[winner_priority] = 1;
			rg_winner_priority <= lv_winner_priority;
			if(rg_priority_threshold <= lv_winner_priority)
				i_valid = True;
		end
		if(!i_valid)
			rg_interrupt_valid <= False;
				
	endrule

	rule rl_encoder(rg_plic_state==1);
		`ifdef verbose $display("Interrupt valid");`endif
		Bit#(ir_bits) interrupt_id = irencoder.encode(rg_total_priority);
		if(interrupt_id!=0 && rg_priority_threshold <= rg_winner_priority) begin
			`ifdef verbose $display("Interrupt valid");`endif
			rg_interrupt_id <= interrupt_id;
			rg_interrupt_valid <= True;
			$display($time,"\t The highest priority interrupt is  %d and the priority is ", interrupt_id, rg_winner_priority);
		end
		rg_plic_state <= 0;
			
		
	endrule

	rule rl_clear_gateway(rg_completion_id matches tagged Valid .id);
		rg_gateway[id] <= False;
		rg_completion_id <= tagged Invalid;
	endrule

	method Action ifc_external_irq_io(Bit#(no_of_ir_pins) irq);
	for(Integer i = 0; i < v_no_of_ir_pins; i = i + 1) begin
  		if(irq[i]==1 && !rg_gateway[i]) begin
									rg_gateway[i] <= True;
									rg_ip[i][0] <= True;
								end
	end
	endmethod

interface ifc_prog_reg = interface IFC_PROGRAM_REGISTERS;

							method ActionValue#(Tuple2#(Bit#(data_width),Bool)) prog_reg(UncachedMemReq#(addr_width, data_width) mem_req,AccessSize size);
								//update memory mapped registers
								`ifdef verbose $display($time,"\tPLIC : programming registers for address %h", mem_req.address);`endif
								Bool success= True;
								let address = mem_req.address;
								Bit#(ir_bits) source_id=0;
								Bit#(data_width) data_return = 0;
								let loop=(size==Byte)?8:(size==HWord)?16:(size==Word)?32:64;// to get the correct number of concatination for rg_ip

								Bit#(64) temp=0;
								let dvalue=valueOf(data_width);
								Bit#(6) shift_amt=zeroExtend(address[2:0]);

			if(address < base_address+'h1000) begin
									address = address >> 2;
									if(mem_req.ld_st == Load) begin
										source_id = address[v_msb_ir_bits:0];
										`ifdef verbose $display($time,"\tPLIC : source %d Priority set to %h", source_id, mem_req.write_data);`endif
										temp = zeroExtend(rg_priority[source_id]);
									end
									else if(mem_req.ld_st == Store) begin
					Bit#(64) store_data;
					store_data=zeroExtend(mem_req.write_data);
										source_id = address[v_msb_ir_bits:0];
										$display($time,"\tPLIC : source %d Priority set to %h", source_id, store_data);
										rg_priority[source_id] <= truncate(store_data);
									end
								end
			else if(address < base_address+'h2000) begin
									if(mem_req.ld_st == Load) begin
										source_id = address[v_msb_ir_bits:0];
										// let shift=(loop==8)?3:(loop==16)?4:(loop==32)?5:6;
										// source_id = source_id << shift;
										// for(Integer i = 0; i < loop; i = i+1)
										// 		temp[i] = pack(rg_ip[source_id + fromInteger(i)][1]);
										if(loop==8) begin
											source_id = source_id << 3;
											for(Integer i = 0; i < 8; i = i+1)
												temp[i] = pack(rg_ip[source_id + fromInteger(i)][1]);
										end
										else if(loop==16 && v_no_of_ir_pins >= 16) begin
											source_id = source_id << 4;
											for(Integer i = 0; i < 16; i = i+1)
												temp[i] = pack(rg_ip[source_id + fromInteger(i)][1]);
										end
										else if(loop==32 && v_no_of_ir_pins >= 32) begin
											source_id = source_id << 5;
											for(Integer i = 0; i < 32; i = i+1)
												temp[i] = pack(rg_ip[source_id + fromInteger(i)][1]);
										end
									end
									else if(mem_req.ld_st == Store) begin
										source_id = address[v_msb_ir_bits:0];
										// let shift=(loop==8)?3:(loop==16)?4:(loop==32)?5:6;
										// source_id = source_id << shift;										
										if(loop==8)begin
											for(Integer i = 0; i < 8; i = i+1) begin
											`ifdef verbose $display($time,"\tPLIC : pending interrupt  %b id %d", mem_req.write_data[i], source_id);`endif
											rg_ip[source_id + fromInteger(i)][1] <= unpack(mem_req.write_data[i]); 
											end	
										end
										else if(loop==16 && v_no_of_ir_pins >= 16)begin
											for(Integer i = 0; i < 16; i = i+1) begin
											`ifdef verbose $display($time,"\tPLIC : pending interrupt  %b id %d", mem_req.write_data[i], source_id);`endif
											rg_ip[source_id + fromInteger(i)][1] <= unpack(mem_req.write_data[i]); 
											end
										end
										else if(loop==32 && v_no_of_ir_pins >= 32)begin
											for(Integer i = 0; i < 32; i = i+1) begin
											`ifdef verbose $display($time,"\tPLIC : pending interrupt  %b id %d", mem_req.write_data[i], source_id);`endif
											rg_ip[source_id + fromInteger(i)][1] <= unpack(mem_req.write_data[i]); 
											end
										end
									end
								end
			else if(address <base_address+'h3000)
								begin
									if(mem_req.ld_st == Load) 
									begin
										source_id = address[v_msb_ir_bits:0];
										// let shift=(loop==8)?3:(loop==16)?4:(loop==32)?5:6;
										// source_id = source_id << shift;	
										if(loop==8)	
										begin								
											source_id = source_id << 3;	
											for(Integer i = 0; i < 8; i = i+1)
											temp[i] = pack(rg_ie[source_id + fromInteger(i)]);
										end
										if(loop==16 && v_no_of_ir_pins >= 16)	
										begin								
											source_id = source_id << 4;	
											for(Integer i = 0; i < 16; i = i+1)
											temp[i] = pack(rg_ie[source_id + fromInteger(i)]);
										end
										if(loop==32 && v_no_of_ir_pins >= 32)	
										begin								
											source_id = source_id << 5;	
											for(Integer i = 0; i < 32; i = i+1)
											temp[i] = pack(rg_ie[source_id + fromInteger(i)]);
										end
                    `ifdef verbose $display($time,"PLIC: Printing Source Enable Interrupt: %h data_return: %h",source_id,temp); `endif
									end
									else if(mem_req.ld_st == Store) begin
										source_id = address[v_msb_ir_bits:0];
										// let shift=(loop==8)?3:(loop==16)?4:(loop==32)?5:6;
										// source_id = source_id << shift;			
										if(loop==8)begin
											source_id=source_id<<3;
											for(Integer i = 0; i < 8; i = i+1) begin
											`ifdef verbose $display($time,"\tPLIC : enabled interrupt  %b id %d", mem_req.write_data[i], source_id);`endif
											rg_ie[source_id + fromInteger(i)] <= unpack(mem_req.write_data[i]); 
											end
										end
										if(loop==16 && v_no_of_ir_pins >= 16)begin
											source_id=source_id<<4;
											for(Integer i = 0; i < 16; i = i+1) begin
											`ifdef verbose $display($time,"\tPLIC : enabled interrupt  %b id %d", mem_req.write_data[i], source_id);`endif
											rg_ie[source_id + fromInteger(i)] <= unpack(mem_req.write_data[i]); 
											end
										end
										if(loop==32 && v_no_of_ir_pins >= 32)begin
											source_id=source_id<<5;
											for(Integer i = 0; i < 32; i = i+1) begin
											`ifdef verbose $display($time,"\tPLIC : enabled interrupt  %b id %d", mem_req.write_data[i], source_id);`endif
											rg_ie[source_id + fromInteger(i)] <= unpack(mem_req.write_data[i]); 
											end
										end
									
									end
								end
			else if(address == base_address+'h10000) begin
									if(mem_req.ld_st == Load) begin
										temp = zeroExtend(rg_priority_threshold); 
									end
									else if(mem_req.ld_st == Store)
										rg_priority_threshold <= mem_req.write_data[v_msb_priority:0];
								end
			else if(address == base_address+'h10010) begin
									if(mem_req.ld_st == Load) begin
										temp = zeroExtend(rg_interrupt_id); 
										rg_ip[rg_interrupt_id][1] <= False;
                                       `ifdef verbose $display($time,"rg_ip is made false here"); `endif
									end
									else if(mem_req.ld_st == Store) begin
										source_id = mem_req.write_data[v_msb_ir_bits:0];
										rg_completion_id <= tagged Valid source_id;
                                        `ifdef verbose $display("rg_completion_id is made tagged valid and completion is signaled-- source_id: %d",source_id); `endif
									end
								end

								//temp=temp>>shift_amt;

								if(size==Byte && dvalue%8==0)
							           temp = duplicate(temp[7:0]);
							    else if(size==HWord && dvalue%16==0)
							           temp = duplicate(temp[15:0]);
							    else if(size==Word && dvalue%32==0)
							           temp = duplicate(temp[31:0]);
						        else 
					                   success=False;

					            data_return=truncate(temp);
								return tuple2(data_return,success);
							endmethod
						endinterface;

							interface intrpt_note_sb= interface Get
								method ActionValue#(Tuple2#(Bool,Bool)) get;
									let v_no_nmi=valueOf(no_nmi);
									Bool if_nmi = (rg_interrupt_id < fromInteger(v_no_nmi));
									Bool valid_interrupt = rg_interrupt_valid;
									//rg_interrupt_valid <= False;
									return tuple2(valid_interrupt, if_nmi);
								endmethod
							endinterface;
endmodule

	interface Ifc_plic_axi4lite#(numeric type addr_width,numeric type data_width,numeric type
      user_width, numeric type no_of_ir_pins, numeric type no_of_ir_levels, numeric type no_nmi);
			interface AXI4_Lite_Slave_IFC#(addr_width, data_width, user_width) slave;
	    method Action ifc_external_irq_io(Bit#(no_of_ir_pins) irq) ;
			interface Get#(Tuple2#(Bool,Bool)) intrpt_note_sb;
	endinterface

	module mkplic_axi4lite#(parameter Integer slave_base)(Ifc_plic_axi4lite#(addr_width, data_width, user_width, no_of_ir_pins, 
      no_of_ir_levels,no_nmi))
		provisos(
				    Add#(a__, data_width, 64),
				    Add#(b__, 8, data_width),
		        Add#(c__, 32, data_width),
            Add#(d__, 1, TLog#(no_of_ir_pins)),
            Add#(e__, 1, no_of_ir_pins),
            Add#(f__, 1, no_of_ir_levels),
            Add#(g__, 1, TLog#(no_of_ir_levels)),
            Add#(h__, no_of_ir_levels, data_width),
            Add#(i__, no_of_ir_pins, 1024),
            Add#(j__, TLog#(no_of_ir_pins), 10),
            Add#(k__, no_of_ir_levels, 1024),
            Add#(l__, TLog#(no_of_ir_levels), 10),
            Add#(m__, no_of_ir_levels, 32),
            Add#(n__, no_of_ir_levels, 64),
            Add#(o__, TLog#(no_of_ir_pins), 64)
			);

		AXI4_Lite_Slave_Xactor_IFC #(addr_width, data_width, user_width)  s_xactor <- mkAXI4_Lite_Slave_Xactor;
		User_ifc#(addr_width, data_width, no_of_ir_pins, no_of_ir_levels, no_nmi) plic <- mkplic(slave_base);

		(*preempts="rl_config_plic_reg_read, rl_config_plic_reg_write"*)
			rule rl_config_plic_reg_write;
				let aw <- pop_o(s_xactor.o_wr_addr);
				let w <- pop_o(s_xactor.o_wr_data);
				let w_strobe = w.wstrb;
				Bit#(TLog#(TDiv#(data_width,8))) byte_offset=0;
				for(Integer i=3; i >= 0; i=i-1) begin 
					if(w_strobe[i]==1)
						byte_offset=fromInteger(i);
				end
				let {x,success} <- plic.ifc_prog_reg.prog_reg(UncachedMemReq{address : aw.awaddr, transfer_size : 'd3, 
														u_signed : 0, byte_offset : byte_offset, write_data : w.wdata, ld_st : Store},unpack(truncate(aw.awsize))); 

				let w_resp = AXI4_Lite_Wr_Resp {bresp: success?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, buser: 0 }; //TODO user value is null
				s_xactor.i_wr_resp.enq(w_resp);
			endrule

 			rule rl_config_plic_reg_read;

				let ar <- pop_o(s_xactor.o_rd_addr);
				let {x,success} <- plic.ifc_prog_reg.prog_reg(UncachedMemReq{address : ar.araddr, transfer_size : 'd3, 
			    														u_signed : 0, byte_offset : 0, ld_st : Load, write_data:?},unpack(truncate(ar.arsize))); 
		        

				let r = AXI4_Lite_Rd_Data {rresp: success?AXI4_LITE_OKAY:AXI4_LITE_SLVERR, rdata: duplicate(x), ruser: 0};
				s_xactor.i_rd_data.enq(r);
			endrule

			interface slave = s_xactor.axi_side;
			method ifc_external_irq_io = plic.ifc_external_irq_io;
			interface intrpt_note_sb = plic.intrpt_note_sb;
	endmodule

	interface Ifc_plic_axi4#(numeric type addr_width, numeric type data_width, numeric type
      user_width, numeric type no_of_ir_pins, numeric type no_of_ir_levels, numeric type no_nmi);
		interface AXI4_Slave_IFC#(addr_width,data_width,user_width) slave;
	  method Action ifc_external_irq_io(Bit#(no_of_ir_pins) irq) ;
		interface Get#(Tuple2#(Bool,Bool)) intrpt_note_sb;
	endinterface

	module mkplic_axi4#(parameter Integer slave_base)(Ifc_plic_axi4#(addr_width,data_width,user_width, no_of_ir_pins,no_of_ir_levels,
      no_nmi))
			provisos(
				    Add#(a__, data_width, 64),
				    Add#(b__, 8, data_width),
				    Add#(c__, 32, data_width),
            Add#(d__, 1, TLog#(no_of_ir_pins)),
            Add#(e__, 1, no_of_ir_pins),
            Add#(f__, 1, no_of_ir_levels),
            Add#(g__, 1, TLog#(no_of_ir_levels)),
            Add#(h__, no_of_ir_levels, data_width),
            Add#(i__, no_of_ir_pins, 1024),
            Add#(j__, TLog#(no_of_ir_pins), 10),
            Add#(k__, no_of_ir_levels, 1024),
            Add#(l__, TLog#(no_of_ir_levels), 10),
            Add#(m__, no_of_ir_levels, 32),
            Add#(n__, no_of_ir_levels, 64),
            Add#(o__, TLog#(no_of_ir_pins), 64)
			);

		let strb_size = valueOf(TSub#(TDiv#(data_width,8),1));

		AXI4_Slave_Xactor_IFC #(addr_width, data_width, user_width)  s_xactor <- mkAXI4_Slave_Xactor;
		User_ifc#(addr_width, data_width, no_of_ir_pins, no_of_ir_levels, no_nmi) plic <- mkplic(slave_base);

	 	Reg#(Bit#(8)) rg_rdburst_count <- mkReg(0);
		Reg#(Bit#(8)) rg_wrburst_count <- mkReg(0);

		Reg#(AXI4_Rd_Addr#(addr_width,user_width)) rg_rdpacket <- mkReg(?);
 		Reg#(AXI4_Wr_Addr#(addr_width,user_width)) rg_wrpacket <- mkReg(?);


		 (*preempts="rl_config_plic_reg_read,rl_config_plic_reg_write"*)
		 //because of the read method side effects (there is also a write to IP bits during the read method)
		 (*preempts="rl_config_plic_reg_read_burst,rl_config_plic_reg_write_burst"*)
		 (*preempts="rl_config_plic_reg_read,rl_config_plic_reg_write_burst"*)
		 (*preempts="rl_config_plic_reg_read_burst,rl_config_plic_reg_write"*)

			rule rl_config_plic_reg_write(rg_wrburst_count==0);
				let aw <- pop_o(s_xactor.o_wr_addr);
				let w <- pop_o(s_xactor.o_wr_data);
				let w_strobe = w.wstrb;
				Bit#(TLog#(TDiv#(data_width,8))) byte_offset=0;
				for(Integer i=strb_size; i >= 0; i=i-1) begin 
					if(w_strobe[i]==1)
						byte_offset=fromInteger(i);
				end
				
				rg_wrpacket<=aw;
		 		if(aw.awlen!=0)
		 			rg_wrburst_count<=1;
				let {x,success} <- plic.ifc_prog_reg.prog_reg(UncachedMemReq{address : aw.awaddr, transfer_size : 'd3, 
														u_signed : 0, byte_offset : byte_offset, write_data : w.wdata, ld_st : Store},unpack(truncate(aw.awsize))); 

				let w_resp = AXI4_Wr_Resp {bresp: success?AXI4_OKAY:AXI4_SLVERR, buser: 0,bid:aw.awid }; //TODO user value is null
				s_xactor.i_wr_resp.enq(w_resp);
			endrule

			rule rl_config_plic_reg_write_burst(rg_wrburst_count!=0);
				let wr_req=rg_wrpacket;
				let w <- pop_o(s_xactor.o_wr_data);
				let w_strobe = w.wstrb;
				Bit#(TLog#(TDiv#(data_width,8))) byte_offset=0;
				for(Integer i=strb_size; i >= 0; i=i-1) begin 
					if(w_strobe[i]==1)
						byte_offset=fromInteger(i);
				end
				let {data,success}<-plic.ifc_prog_reg.prog_reg(UncachedMemReq{address : wr_req.awaddr, transfer_size : 'd3, 
														u_signed : 0, byte_offset : byte_offset, write_data : w.wdata, ld_st : Store},unpack(truncate(wr_req.awsize))); 
				if(rg_wrburst_count==wr_req.awlen)
					rg_wrburst_count<=0;
				else
					rg_wrburst_count<=rg_wrburst_count+1;
				let resp= AXI4_Wr_Resp {bresp: success?AXI4_OKAY:AXI4_SLVERR, buser: 0,bid:wr_req.awid };
				s_xactor.i_wr_resp.enq(resp);
			endrule

 			rule rl_config_plic_reg_read(rg_rdburst_count==0);


				let ar <- pop_o(s_xactor.o_rd_addr);
				rg_rdpacket<=ar;
				if(ar.arlen!=0)
		 			rg_rdburst_count<=1;
				let {x,success} <- plic.ifc_prog_reg.prog_reg(UncachedMemReq{address : ar.araddr, transfer_size : 'd3, 
			    													write_data:?,	u_signed : 0, byte_offset : 0, ld_st : Load},unpack(truncate(ar.arsize))); 

				let r = AXI4_Rd_Data {rresp: success?AXI4_OKAY:AXI4_SLVERR, rdata: duplicate(x), ruser: 0,rid:ar.arid,rlast:(ar.arlen==0)};
				s_xactor.i_rd_data.enq(r);
			endrule

			rule rl_config_plic_reg_read_burst(rg_rdburst_count!=0);
				let rd_req=rg_rdpacket;
				let {x,success} <- plic.ifc_prog_reg.prog_reg(UncachedMemReq{address : rd_req.araddr, transfer_size : 'd3, 
			    														write_data:?, u_signed : 0, byte_offset : 0, ld_st : Load},unpack(truncate(rd_req.arsize))); 			
		        success=False;
		        if(rg_rdburst_count==rd_req.arlen)
					rg_rdburst_count<=0;
				else
					rg_rdburst_count<=rg_rdburst_count+1;
				let r = AXI4_Rd_Data {rresp: success?AXI4_OKAY:AXI4_SLVERR, rdata: duplicate(x), ruser: 0,rid:rd_req.arid,rlast:(rd_req.arlen==0)};
				s_xactor.i_rd_data.enq(r);	
			endrule

			interface slave = s_xactor.axi_side;
			method ifc_external_irq_io = plic.ifc_external_irq_io;
			interface intrpt_note_sb = plic.intrpt_note_sb;
	endmodule
endpackage	
