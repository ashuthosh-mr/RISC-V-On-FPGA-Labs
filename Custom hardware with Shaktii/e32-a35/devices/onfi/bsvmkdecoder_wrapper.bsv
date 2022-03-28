// Bluespec wrapper, created by Import BVI Wizard
// Created on: Mon Jun 04 11:39:28 IST 2018
// Created by: vishvesh
// Bluespec version: 2017.07.A 2017-07-21 1da80f1


interface Ifc_decode_wrapper;
	(*always_ready*)
	method Action idata (Bit#(8) data);
	(*always_ready*)
	method Action isyn_start (Bit#(1) syn_start);
	method Bit#(1) osyn_ready ();
	method Bit#(8) oerr_out ();
	method Bit#(1) ofirst_out ();
endinterface

import "BVI" decoder_wrapper =
module mkdecoder_wrapper  (Ifc_decode_wrapper);

	parameter T = 3;
	parameter DATA_BITS = 4096;
	parameter BITS = 8;
	parameter SYN_REG_RATIO = 1;
	parameter ERR_REG_RATIO = 1;
	parameter SYN_PIPELINE_STAGES = 0;
	parameter ERR_PIPELINE_STAGES = 0;
	parameter ACCUM = 1;
	parameter NCHANNEL = 1;
	parameter NKEY = 2;
	parameter NCHIEN = 1;

	default_clock clk(clk);
	default_reset no_reset;


	method idata (data /*8*/)
		 enable((*inhigh*)idata_enable) ;
	method isyn_start (syn_start /*1*/)
		 enable((*inhigh*)isyn_start_enable) ;
	method syn_ready /* 1 */ osyn_ready ();
	method err_out /* 8 */ oerr_out ();
	method first_out ofirst_out ();

	schedule idata C idata;
	schedule idata CF isyn_start;
	schedule osyn_ready CF idata;
	schedule oerr_out CF idata;
	schedule ofirst_out CF idata;
	schedule isyn_start C isyn_start;
	schedule osyn_ready CF isyn_start;
	schedule oerr_out CF isyn_start;
	schedule ofirst_out CF isyn_start;
	schedule osyn_ready CF osyn_ready;
	schedule osyn_ready CF oerr_out;
	schedule osyn_ready CF ofirst_out;
	schedule oerr_out CF oerr_out;
	schedule oerr_out CF ofirst_out;
	schedule ofirst_out CF ofirst_out;
endmodule


