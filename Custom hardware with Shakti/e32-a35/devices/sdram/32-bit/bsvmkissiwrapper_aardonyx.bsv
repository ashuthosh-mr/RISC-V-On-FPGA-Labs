// Bluespec wrapper, created by Import BVI Wizard
// Created on: Fri Jan 04 12:38:05 IST 2019
// Created by: vishvesh
// Bluespec version: 2018.10.beta1 2018-10-17 e1df8052c


interface Ifc_issi_aardonyx;
	interface Inout#(Bit#(1)) dq_0;
	interface Inout#(Bit#(1)) dq_1;
	interface Inout#(Bit#(1)) dq_2;
	interface Inout#(Bit#(1)) dq_3;
	interface Inout#(Bit#(1)) dq_4;
	interface Inout#(Bit#(1)) dq_5;
	interface Inout#(Bit#(1)) dq_6;
	interface Inout#(Bit#(1)) dq_7;
	interface Inout#(Bit#(1)) dq_8;
	interface Inout#(Bit#(1)) dq_9;
	interface Inout#(Bit#(1)) dq_10;
	interface Inout#(Bit#(1)) dq_11;
	interface Inout#(Bit#(1)) dq_12;
	interface Inout#(Bit#(1)) dq_13;
	interface Inout#(Bit#(1)) dq_14;
	interface Inout#(Bit#(1)) dq_15;
	interface Inout#(Bit#(1)) dq_16;
	interface Inout#(Bit#(1)) dq_17;
	interface Inout#(Bit#(1)) dq_18;
	interface Inout#(Bit#(1)) dq_19;
	interface Inout#(Bit#(1)) dq_20;
	interface Inout#(Bit#(1)) dq_21;
	interface Inout#(Bit#(1)) dq_22;
	interface Inout#(Bit#(1)) dq_23;
	interface Inout#(Bit#(1)) dq_24;
	interface Inout#(Bit#(1)) dq_25;
	interface Inout#(Bit#(1)) dq_26;
	interface Inout#(Bit#(1)) dq_27;
	interface Inout#(Bit#(1)) dq_28;
	interface Inout#(Bit#(1)) dq_29;
	interface Inout#(Bit#(1)) dq_30;
	interface Inout#(Bit#(1)) dq_31;
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

import "BVI" issiwrapper_aardonyx =
module mkissiwrapper_aardonyx  (Ifc_issi_aardonyx);

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

  ifc_inout dq_0        ( dq_0);
  ifc_inout	dq_1        ( dq_1);
  ifc_inout	dq_2        ( dq_2);
  ifc_inout	dq_3        ( dq_3);
  ifc_inout	dq_4        ( dq_4);
  ifc_inout	dq_5        ( dq_5);
  ifc_inout	dq_6        ( dq_6);
  ifc_inout	dq_7        ( dq_7);
  ifc_inout	dq_8        ( dq_8);
  ifc_inout	dq_9        ( dq_9);
  ifc_inout	dq_10       ( dq_10);
  ifc_inout	dq_11       ( dq_11);
  ifc_inout	dq_12       ( dq_12);
  ifc_inout	dq_13       ( dq_13);
  ifc_inout	dq_14       ( dq_14);
  ifc_inout	dq_15       ( dq_15);
  ifc_inout	dq_16       ( dq_16);
  ifc_inout	dq_17       ( dq_17);
  ifc_inout	dq_18       ( dq_18);
  ifc_inout	dq_19       ( dq_19);
  ifc_inout	dq_20       ( dq_20);
  ifc_inout	dq_21       ( dq_21);
  ifc_inout	dq_22       ( dq_22);
  ifc_inout	dq_23       ( dq_23);
  ifc_inout	dq_24       ( dq_24);
  ifc_inout	dq_25       ( dq_25);
  ifc_inout	dq_26       ( dq_26);
  ifc_inout	dq_27       ( dq_27);
  ifc_inout	dq_28       ( dq_28);
  ifc_inout	dq_29       ( dq_29);
  ifc_inout	dq_30       ( dq_30);
  ifc_inout	dq_31       ( dq_31);
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


