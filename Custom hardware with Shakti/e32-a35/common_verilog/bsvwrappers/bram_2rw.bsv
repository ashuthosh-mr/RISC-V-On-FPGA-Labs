// Bluespec wrapper, created by Import BVI Wizard
// Created on: Thu Nov 22 18:16:30 IST 2018
// Created by: neel
// Bluespec version: 2018.10.beta1 2018-10-17 e1df8052c

package bram_2rw;
interface Ifc_bram_2rw#(numeric type addr_width, numeric type data_width, numeric type memsize);
	(*always_enabled*)
	method Action request_a (Bit#(1) wea, Bit#(addr_width) addra, Bit#(data_width) dina);
	(*always_enabled*)
	method Bit#(data_width) response_a ();
	(*always_enabled*)
	method Action request_b (Bit#(1) web, Bit#(addr_width) addrb, Bit#(data_width) dinb);
	(*always_enabled*)
	method Bit#(data_width) response_b ();
endinterface

import "BVI" bram_2rw =
module mkbram_2rw  (Ifc_bram_2rw#(addr_width, data_width, memsize));

	parameter ADDR_WIDTH = valueOf(addr_width);
	parameter DATA_WIDTH = valueOf(data_width);
	parameter MEMSIZE = valueOf(memsize);

	default_clock clk_clka;
	default_reset no_reset;

	input_clock clk_clka (clka)  <- exposeCurrentClock;
	input_clock clk_clkb (clkb)  <- exposeCurrentClock;


  	method request_a (wea , addra /*ADDR_WIDTH-1:0*/, dina /*DATA_WIDTH-1:0*/)
	  	 enable(ena) clocked_by(clk_clka);
  	method douta /* DATA_WIDTH-1 : 0 */ response_a ()
	  	 clocked_by(clk_clka);

  	method request_b (web , addrb /*ADDR_WIDTH-1:0*/, dinb /*DATA_WIDTH-1:0*/)
	  	 enable(enb) clocked_by(clk_clkb);
  	method doutb /* DATA_WIDTH-1 : 0 */ response_b ()
	  	 clocked_by(clk_clkb);

	schedule request_a C request_a;
	schedule request_a CF response_a;
	schedule response_a CF response_a;

	schedule request_b C request_b;
	schedule request_b CF response_b;
	schedule response_b CF response_b;

	schedule request_b CF request_a;
	schedule request_b CF response_a;
	schedule response_b CF request_a;
	schedule response_b CF response_a;
endmodule

endpackage
