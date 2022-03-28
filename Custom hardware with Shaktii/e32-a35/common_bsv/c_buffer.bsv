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

Author: Rahul Bodduna 
Email id: rahul.bodduna@gmail.com 
Details:

--------------------------------------------------------------------------------------------------
*/


package c_buffer;

import Vector ::*;
import CWire ::*;
import types ::*;
`include "parameters.bsv"


interface Ifc_read_buffer#(type a);

	method a read_buffer;

	method Action incr_tail;

	method Bool b_not_empty;

endinterface

interface Ifc_write_buffer#(type a);

	method Action write_buffer(a x);

	method Bool b_not_full;

endinterface

interface Ifc_fill_buffer#(numeric type w_ports, type a);
	interface Vector#(w_ports, Ifc_write_buffer#(a)) w;
endinterface

interface Ifc_free_buffer#(numeric type c_buffer_bsize, numeric type r_ports, type a);
	interface Vector#(r_ports, Ifc_read_buffer#(a)) r;
	method Bit#(TLog#(c_buffer_bsize)) b_filled;
endinterface

interface Ifc_c_buffer#(numeric type c_buffer_bsize, 
																numeric type w_ports, 
																numeric type r_ports, 
																								type a);
interface Ifc_fill_buffer#(w_ports,a) fill;
interface Ifc_free_buffer#(c_buffer_bsize, r_ports,a) free;

method Action flush;

method Bit#(TLog#(c_buffer_bsize)) empty_slots;

endinterface

	
module mkc_buffer#(parameter String instant, function a fn_gen(Integer i), Integer a) 
																 (Ifc_c_buffer#(c_buffer_bsize, w_ports, r_ports, a))
				provisos( Bits#(a, a__),
									Log#(c_buffer_bsize, c_buf_index),
									Add#(r_ports,1,incr_tail),
									Add#(w_ports,1,incr_head));

let v_incr_tail = valueOf(incr_tail);
let v_incr_head = valueOf(incr_head);
let v_r_ports = valueOf(r_ports);
let v_w_ports = valueOf(w_ports);
let v_c_buf = valueOf(c_buffer_bsize);

Reg#(a) buffer[valueOf(c_buffer_bsize)];

for(Integer i = 0; i < valueOf(c_buffer_bsize); i = i+1)
	buffer[i] <- mkReg(fn_gen(i));
	//Vector#(c_buffer_bsize, Reg#(a)) buffer <- monad_func(compose(mkReg(), fromInteger)); 


Reg#(Bit#(TLog#(c_buffer_bsize))) rg_head <- mkReg(fromInteger(a));
Reg#(Bit#(TLog#(c_buffer_bsize))) rg_tail <- mkReg(-1);

CWire#(incr_tail, Bit#(TLog#(c_buffer_bsize))) wr_tail_ctr <- mkCWire();
CWire#(incr_head, Bit#(TLog#(c_buffer_bsize))) wr_head_ctr <- mkCWire();

Vector#(r_ports, Ifc_read_buffer#(a)) temp_read_buffer;

Vector#(w_ports, Ifc_write_buffer#(a)) temp_write_buffer;

Wire#(Bool) wr_flush <- mkDWire(False);

function Bit#(TLog#(c_buffer_bsize)) free_slots;
	return rg_tail - rg_head;
endfunction

rule rl_head_increment(!wr_flush);
	rg_head <= rg_head + wr_head_ctr.read[v_w_ports];
endrule

rule rl_tail_increment(!wr_flush);
	rg_tail <= rg_tail + wr_tail_ctr.read[v_r_ports];
endrule

rule rl_flush_head(wr_flush);
	rg_head <= fromInteger(a);
endrule

rule rl_flush_tail(wr_flush);
	rg_tail <= '1;
endrule

rule rl_display_no_of_free_slots;
	if(`VERBOSITY>=3)
		$display($time,"The number of free slots in the %s buffer %d", instant, free_slots);
endrule

for(Integer i =0; i<valueOf(r_ports); i =i+1) begin

	temp_read_buffer[i] = interface Ifc_read_buffer 
													method a read_buffer;
														return buffer[rg_tail+wr_tail_ctr.read[i]+1];
													endmethod 
													method Action incr_tail if(free_slots<(fromInteger(v_c_buf-1)-wr_tail_ctr.read[i]));
														wr_tail_ctr.write[i] <= wr_tail_ctr.read[i] + 1;
													endmethod

													method Bool b_not_empty;
														if(free_slots <fromInteger(v_c_buf-1)-wr_tail_ctr.read[i])
															return True;
														else
															return False;
													endmethod
												endinterface;

end

for(Integer i =0; i<valueOf(w_ports); i =i+1) begin

	temp_write_buffer[i] = interface Ifc_write_buffer
													method Action write_buffer(a x) if(free_slots>wr_head_ctr.read[i]);
														buffer[rg_head+wr_head_ctr.read[i]] <= x;
														wr_head_ctr.write[i] <= wr_head_ctr.read[i]+1;
														if(`VERBOSITY!=0)
															$display("The writing into %s buffer %d value %d", 
																										instant, rg_head+wr_head_ctr.read[i], x);
													endmethod

													method Bool b_not_full;
														if(free_slots > wr_head_ctr.read[i]) 
															return True;
														else
															return False;
													endmethod

												endinterface;
end

interface fill = interface Ifc_fill_buffer;

	interface w = temp_write_buffer;
	
endinterface;

interface free = interface Ifc_free_buffer;

	interface r = temp_read_buffer;
	
	method Bit#(TLog#(c_buffer_bsize)) b_filled;
		Bit#(TLog#(c_buffer_bsize)) no_packets = rg_head - rg_tail;
		return no_packets;
	endmethod

endinterface;

method Action flush;
	wr_flush <= True;
	if(`VERBOSITY>=1)
		$display("Flushing %s", instant); 
endmethod

method Bit#(TLog#(c_buffer_bsize)) empty_slots;
	return free_slots;
endmethod

endmodule

endpackage
