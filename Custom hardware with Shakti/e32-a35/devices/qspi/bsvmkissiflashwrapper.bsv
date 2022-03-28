// Bluespec wrapper, created by Import BVI Wizard
// Created on: Wed Jan 16 11:38:07 IST 2019
// Created by: vishvesh
// Bluespec version: 2018.10.beta1 2018-10-17 e1df8052c


interface Ifc_issiflashwrapper;
	interface Inout#(bit) si;
	interface Inout#(bit) so;
	interface Inout#(bit) wp;
	interface Inout#(bit) sio3;
	(*always_ready , always_enabled*)
	method Action isclk (bit sclk);
	(*always_ready , always_enabled*)
	method Action ics (bit cs);
endinterface

import "BVI" issiflashwrapper =
module mkissiflashwrapper  (Ifc_issiflashwrapper);

	default_clock clk_clk;
	default_reset rst_rst;

	input_clock clk_clk (clk)  <- exposeCurrentClock;
	input_reset rst_rst (rst) clocked_by(clk_clk)  <- exposeCurrentReset;

	ifc_inout si(si);
	ifc_inout so(so);
	ifc_inout wp(wp);
	ifc_inout sio3(sio3);

	method isclk (sclk )
		 enable((*inhigh*)isclk_enable) clocked_by(clk_clk) reset_by(rst_rst);
	method ics (cs )
		 enable((*inhigh*)ics_enable) clocked_by(clk_clk) reset_by(rst_rst);

	schedule isclk C isclk;
	schedule isclk CF ics;
	schedule ics C ics;
endmodule


