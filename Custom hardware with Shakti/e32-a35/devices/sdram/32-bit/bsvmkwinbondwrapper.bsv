// Bluespec wrapper, created by Import BVI Wizard
// Created on: Wed Dec 13 15:55:41 IST 2017
// Created by: vishvesh
// Bluespec version: 2017.03.beta1 2017-03-16 35049

	`define ADDR_BITS 13
	`define ROW_BITS  13
	`define DQ_BITS   32
	`define DM_BITS   4
	`define COL_BITS  9
	`define BA_BITS   2

interface Ifc_windbond;
	interface Inout#(Bit#(`DQ_BITS)) dq;
	(*always_ready , always_enabled*)
	method Action iCke (bit cke);
	(*always_ready , always_enabled*)
	method Action iAddr (Bit#(`ADDR_BITS) addr);
	(*always_ready , always_enabled*)
	method Action iBa (Bit#(2) ba);
	(*always_ready , always_enabled*)
	method Action iCs_n (bit cs_n);
	(*always_ready , always_enabled*)
	method Action iRas_n (bit ras_n);
	(*always_ready , always_enabled*)
	method Action iCas_n (bit cas_n);
	(*always_ready , always_enabled*)
	method Action iWe_n (bit we_n);
	(*always_ready , always_enabled*)
	method Action iDqm (Bit#(`DM_BITS) dqm);
endinterface

import "BVI" winbondwrapper =
module mkwinbondwrapper  (Ifc_windbond);

	parameter ADDR_BITS = 13;
	parameter ROW_BITS =  13;
	parameter DQ_BITS =   32;
	parameter DM_BITS =   4;
	parameter COL_BITS =  9;
	parameter BA_BITS =   2;

	default_clock clk_old_clk;
	default_reset rst;

	input_clock clk_old_clk (old_Clk)  <- exposeCurrentClock;
	input_reset rst (/* empty */) clocked_by(clk_old_clk)  <- exposeCurrentReset;
	
	ifc_inout   dq(Dq);
	//inout Dq clocked_by (clk_old_clk) reset_by (rst) = inout;

	method iCke (Cke )
		 enable((*inhigh*)iCke_enable) clocked_by(clk_old_clk) reset_by(rst);
	method iAddr (Addr /*ADDR_BITS-1:0*/)
		 enable((*inhigh*)iAddr_enable) clocked_by(clk_old_clk) reset_by(rst);
	method iBa (Ba /*BA_BITS-1:0*/)
		 enable((*inhigh*)iBa_enable) clocked_by(clk_old_clk) reset_by(rst);
	method iCs_n (Cs_n )
		 enable((*inhigh*)iCs_n_enable) clocked_by(clk_old_clk) reset_by(rst);
	method iRas_n (Ras_n )
		 enable((*inhigh*)iRas_n_enable) clocked_by(clk_old_clk) reset_by(rst);
	method iCas_n (Cas_n )
		 enable((*inhigh*)iCas_n_enable) clocked_by(clk_old_clk) reset_by(rst);
	method iWe_n (We_n )
		 enable((*inhigh*)iWe_n_enable) clocked_by(clk_old_clk) reset_by(rst);
	method iDqm (Dqm /*DM_BITS-1:0*/)
		 enable((*inhigh*)iDqm_enable) clocked_by(clk_old_clk) reset_by(rst);

	schedule iCke C iCke;
	schedule iCke CF iAddr;
	schedule iCke CF iBa;
	schedule iCke CF iCs_n;
	schedule iCke CF iRas_n;
	schedule iCke CF iCas_n;
	schedule iCke CF iWe_n;
	schedule iCke CF iDqm;
	schedule iAddr C iAddr;
	schedule iAddr CF iBa;
	schedule iAddr CF iCs_n;
	schedule iAddr CF iRas_n;
	schedule iAddr CF iCas_n;
	schedule iAddr CF iWe_n;
	schedule iAddr CF iDqm;
	schedule iBa C iBa;
	schedule iBa CF iCs_n;
	schedule iBa CF iRas_n;
	schedule iBa CF iCas_n;
	schedule iBa CF iWe_n;
	schedule iBa CF iDqm;
	schedule iCs_n C iCs_n;
	schedule iCs_n CF iRas_n;
	schedule iCs_n CF iCas_n;
	schedule iCs_n CF iWe_n;
	schedule iCs_n CF iDqm;
	schedule iRas_n C iRas_n;
	schedule iRas_n CF iCas_n;
	schedule iRas_n CF iWe_n;
	schedule iRas_n CF iDqm;
	schedule iCas_n C iCas_n;
	schedule iCas_n CF iWe_n;
	schedule iCas_n CF iDqm;
	schedule iWe_n C iWe_n;
	schedule iWe_n CF iDqm;
	schedule iDqm C iDqm;
endmodule


