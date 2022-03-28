// Bluespec wrapper, created by Import BVI Wizard
// Created on: Fri Jan 04 12:38:05 IST 2019
// Created by: vishvesh
// Bluespec version: 2018.10.beta1 2018-10-17 e1df8052c


interface Ifc_issi;
	interface Inout#(Bit#(32)) dq;
	(*always_ready , always_enabled*)
	method Action icke (bit cke);
	(*always_ready , always_enabled*)
	method Action iaddr (Bit#(13) addr);
	(*always_ready , always_enabled*)
	method Action iba (Bit#(2) ba);
	(*always_ready , always_enabled*)
	method Action ics_n (bit cs_n);
	(*always_ready , always_enabled*)
	method Action iras_n (bit ras_n);
	(*always_ready , always_enabled*)
	method Action icas_n (bit cas_n);
	(*always_ready , always_enabled*)
	method Action iwe_n (bit we_n);
	(*always_ready , always_enabled*)
	method Action idqm (Bit#(4) dqm);
endinterface

import "BVI" issiwrapper =
module mkissiwrapper  (Ifc_issi);

	parameter ADDR_BITS = 13;
	parameter ROW_BITS = 13;
	parameter DQ_BITS = 32;
	parameter DM_BITS = 4;
	parameter COL_BITS = 9;
	parameter BA_BITS = 2;

	default_clock old_Clk;
	default_reset rst;

	input_clock old_Clk (old_Clk)  <- exposeCurrentClock;
	input_reset rst (/* empty */) clocked_by(old_Clk)  <- exposeCurrentReset;

	ifc_inout   dq(dq);

	method icke (cke )
		 enable((*inhigh*)icke_enable) clocked_by(old_Clk) reset_by(rst);
	method iaddr (addr /*ADDR_BITS-1:0*/)
		 enable((*inhigh*)iaddr_enable) clocked_by(old_Clk) reset_by(rst);
	method iba (ba /*BA_BITS-1:0*/)
		 enable((*inhigh*)iba_enable) clocked_by(old_Clk) reset_by(rst);
	method ics_n (cs_n )
		 enable((*inhigh*)ics_n_enable) clocked_by(old_Clk) reset_by(rst);
	method iras_n (ras_n )
		 enable((*inhigh*)iras_n_enable) clocked_by(old_Clk) reset_by(rst);
	method icas_n (cas_n )
		 enable((*inhigh*)icas_n_enable) clocked_by(old_Clk) reset_by(rst);
	method iwe_n (we_n )
		 enable((*inhigh*)iwe_n_enable) clocked_by(old_Clk) reset_by(rst);
	method idqm (dqm /*DM_BITS-1:0*/)
		 enable((*inhigh*)idqm_enable) clocked_by(old_Clk) reset_by(rst);

	schedule icke C icke;
	schedule icke CF iaddr;
	schedule icke CF iba;
	schedule icke CF ics_n;
	schedule icke CF iras_n;
	schedule icke CF icas_n;
	schedule icke CF iwe_n;
	schedule icke CF idqm;
	schedule iaddr C iaddr;
	schedule iaddr CF iba;
	schedule iaddr CF ics_n;
	schedule iaddr CF iras_n;
	schedule iaddr CF icas_n;
	schedule iaddr CF iwe_n;
	schedule iaddr CF idqm;
	schedule iba C iba;
	schedule iba CF ics_n;
	schedule iba CF iras_n;
	schedule iba CF icas_n;
	schedule iba CF iwe_n;
	schedule iba CF idqm;
	schedule ics_n C ics_n;
	schedule ics_n CF iras_n;
	schedule ics_n CF icas_n;
	schedule ics_n CF iwe_n;
	schedule ics_n CF idqm;
	schedule iras_n C iras_n;
	schedule iras_n CF icas_n;
	schedule iras_n CF iwe_n;
	schedule iras_n CF idqm;
	schedule icas_n C icas_n;
	schedule icas_n CF iwe_n;
	schedule icas_n CF idqm;
	schedule iwe_n C iwe_n;
	schedule iwe_n CF idqm;
	schedule idqm C idqm;
endmodule


