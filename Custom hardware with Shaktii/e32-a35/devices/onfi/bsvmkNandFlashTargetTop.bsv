// Bluespec wrapper, created by Import BVI Wizard
// Created on: Mon Jan 09 19:30:15 IST 2017
// Created by: reshmi
// Bluespec version: 2015.09.beta2 2015-09-07 34689
package bsvmkNandFlashTargetTop;

interface ONFI_Target_Interface_top;
	interface Inout#(Bit#(8)) data;
	interface Inout#(Bit#(8)) data2;
	interface Inout#(Bool) dqs_open;
	interface Inout#(Bool) dqs_c_open;
	interface Inout#(Bool) dqs2_open;
	interface Inout#(Bool) dqs2_c_open;
	(*always_ready*)
	method Action _Ce_n (bit ce_n);
	(*always_ready*)
	method Action _We_n (bit clk_we_n);
	(*always_ready*)
	method Action _Re_n (bit wr_re_n);
	(*always_ready*)
	method Action _Wp_n (bit wp_n);
	(*always_ready*)
	method Action _Cle (bit cle);
	(*always_ready*)
	method Action _Ale (bit ale);
	method bit _Rb_n ();
	(*always_ready*)
	method Action _Ce2_n (bit ce2_n);
	(*always_ready*)
	method Action _We2_n (bit clk_we2_n);
	(*always_ready*)
	method Action _Re2_n (bit wr_re2_n);
	(*always_ready*)
	method Action _Wp2_n (bit wp2_n);
	(*always_ready*)
	method Action _Cle2 (bit cle2);
	(*always_ready*)
	method Action _Ale2 (bit ale2);
	method bit _Rb2_n ();
	(*always_ready*)
	method Action _ENi (bit eni);
	method bit _ENo ();
	(*always_ready*)
	method Action _Re_c (bit re_c);
endinterface

import "BVI" nand_model =
module mkNandFlashTargetTop  (ONFI_Target_Interface_top ifc);

	parameter MAX_LUN_PER_TAR = 2;

	default_clock clk();
	default_reset no_reset;

	ifc_inout data(Dq_Io);
	ifc_inout data2(Dq_Io2);
	
	ifc_inout dqs_open(Dqs);
	ifc_inout dqs_c_open(Dqs_c);
	ifc_inout dqs2_open(Dqs2);
	ifc_inout dqs2_c_open(Dqs2_c);

	method _Ce_n (Ce_n /*0:0*/)
		 enable((*inhigh*)_Ce_n_enable);
	method _We_n (Clk_We_n /*0:0*/)
		 enable((*inhigh*)_We_n_enable);
	method _Re_n (Wr_Re_n /*0:0*/)
		 enable((*inhigh*)_Re_n_enable);
	method _Wp_n (Wp_n /*0:0*/)
		 enable((*inhigh*)_Wp_n_enable);
	method _Cle (Cle /*0:0*/)
		 enable((*inhigh*)_Cle_enable);
	method _Ale (Ale /*0:0*/)
		 enable((*inhigh*)_Ale_enable);
	method Rb_n _Rb_n ()
		;
	method _Ce2_n (Ce2_n /*0:0*/)
		 enable((*inhigh*)_Ce2_n_enable);
	method _We2_n (Clk_We2_n /*0:0*/)
		 enable((*inhigh*)_We2_n_enable);
	method _Re2_n (Wr_Re2_n /*0:0*/)
		 enable((*inhigh*)_Re2_n_enable);
	method _Wp2_n (Wp2_n /*0:0*/)
		 enable((*inhigh*)_Wp2_n_enable);
	method _Cle2 (Cle2 /*0:0*/)
		 enable((*inhigh*)_Cle2_enable);
	method _Ale2 (Ale2 /*0:0*/)
		 enable((*inhigh*)_Ale2_enable);
	method Rb2_n _Rb2_n ()
		;
	method _ENi (ENi /*0:0*/)
		 enable((*inhigh*)_ENi_enable);
	method ENo _ENo ()
		;
	method _Re_c (Re_c /*0:0*/)
		 enable((*inhigh*)_Re_c_enable);

	schedule _Ce_n C _Ce_n;
	schedule _Ce_n CF _We_n;
	schedule _Ce_n CF _Re_n;
	schedule _Ce_n CF _Wp_n;
	schedule _Ce_n CF _Cle;
	schedule _Ce_n CF _Ale;
	schedule _Ce_n CF _Rb_n;
	schedule _Ce_n CF _Ce2_n;
	schedule _Ce_n CF _We2_n;
	schedule _Ce_n CF _Re2_n;
	schedule _Ce_n CF _Wp2_n;
	schedule _Ce_n CF _Cle2;
	schedule _Ce_n CF _Ale2;
	schedule _Ce_n CF _Rb2_n;
	schedule _Ce_n CF _ENi;
	schedule _Ce_n CF _ENo;
	schedule _Ce_n CF _Re_c;
	schedule _We_n C _We_n;
	schedule _We_n CF _Re_n;
	schedule _We_n CF _Wp_n;
	schedule _We_n CF _Cle;
	schedule _We_n CF _Ale;
	schedule _We_n CF _Rb_n;
	schedule _We_n CF _Ce2_n;
	schedule _We_n CF _We2_n;
	schedule _We_n CF _Re2_n;
	schedule _We_n CF _Wp2_n;
	schedule _We_n CF _Cle2;
	schedule _We_n CF _Ale2;
	schedule _We_n CF _Rb2_n;
	schedule _We_n CF _ENi;
	schedule _We_n CF _ENo;
	schedule _We_n CF _Re_c;
	schedule _Re_n C _Re_n;
	schedule _Re_n CF _Wp_n;
	schedule _Re_n CF _Cle;
	schedule _Re_n CF _Ale;
	schedule _Re_n CF _Rb_n;
	schedule _Re_n CF _Ce2_n;
	schedule _Re_n CF _We2_n;
	schedule _Re_n CF _Re2_n;
	schedule _Re_n CF _Wp2_n;
	schedule _Re_n CF _Cle2;
	schedule _Re_n CF _Ale2;
	schedule _Re_n CF _Rb2_n;
	schedule _Re_n CF _ENi;
	schedule _Re_n CF _ENo;
	schedule _Re_n CF _Re_c;
	schedule _Wp_n C _Wp_n;
	schedule _Wp_n CF _Cle;
	schedule _Wp_n CF _Ale;
	schedule _Wp_n CF _Rb_n;
	schedule _Wp_n CF _Ce2_n;
	schedule _Wp_n CF _We2_n;
	schedule _Wp_n CF _Re2_n;
	schedule _Wp_n CF _Wp2_n;
	schedule _Wp_n CF _Cle2;
	schedule _Wp_n CF _Ale2;
	schedule _Wp_n CF _Rb2_n;
	schedule _Wp_n CF _ENi;
	schedule _Wp_n CF _ENo;
	schedule _Wp_n CF _Re_c;
	schedule _Cle C _Cle;
	schedule _Cle CF _Ale;
	schedule _Cle CF _Rb_n;
	schedule _Cle CF _Ce2_n;
	schedule _Cle CF _We2_n;
	schedule _Cle CF _Re2_n;
	schedule _Cle CF _Wp2_n;
	schedule _Cle CF _Cle2;
	schedule _Cle CF _Ale2;
	schedule _Cle CF _Rb2_n;
	schedule _Cle CF _ENi;
	schedule _Cle CF _ENo;
	schedule _Cle CF _Re_c;
	schedule _Ale C _Ale;
	schedule _Ale CF _Rb_n;
	schedule _Ale CF _Ce2_n;
	schedule _Ale CF _We2_n;
	schedule _Ale CF _Re2_n;
	schedule _Ale CF _Wp2_n;
	schedule _Ale CF _Cle2;
	schedule _Ale CF _Ale2;
	schedule _Ale CF _Rb2_n;
	schedule _Ale CF _ENi;
	schedule _Ale CF _ENo;
	schedule _Ale CF _Re_c;
	schedule _Rb_n C _Rb_n;
	schedule _Rb_n CF _Ce2_n;
	schedule _Rb_n CF _We2_n;
	schedule _Rb_n CF _Re2_n;
	schedule _Rb_n CF _Wp2_n;
	schedule _Rb_n CF _Cle2;
	schedule _Rb_n CF _Ale2;
	schedule _Rb_n CF _Rb2_n;
	schedule _Rb_n CF _ENi;
	schedule _Rb_n CF _ENo;
	schedule _Rb_n CF _Re_c;
	schedule _Ce2_n C _Ce2_n;
	schedule _Ce2_n CF _We2_n;
	schedule _Ce2_n CF _Re2_n;
	schedule _Ce2_n CF _Wp2_n;
	schedule _Ce2_n CF _Cle2;
	schedule _Ce2_n CF _Ale2;
	schedule _Ce2_n CF _Rb2_n;
	schedule _Ce2_n CF _ENi;
	schedule _Ce2_n CF _ENo;
	schedule _Ce2_n CF _Re_c;
	schedule _We2_n C _We2_n;
	schedule _We2_n CF _Re2_n;
	schedule _We2_n CF _Wp2_n;
	schedule _We2_n CF _Cle2;
	schedule _We2_n CF _Ale2;
	schedule _We2_n CF _Rb2_n;
	schedule _We2_n CF _ENi;
	schedule _We2_n CF _ENo;
	schedule _We2_n CF _Re_c;
	schedule _Re2_n C _Re2_n;
	schedule _Re2_n CF _Wp2_n;
	schedule _Re2_n CF _Cle2;
	schedule _Re2_n CF _Ale2;
	schedule _Re2_n CF _Rb2_n;
	schedule _Re2_n CF _ENi;
	schedule _Re2_n CF _ENo;
	schedule _Re2_n CF _Re_c;
	schedule _Wp2_n C _Wp2_n;
	schedule _Wp2_n CF _Cle2;
	schedule _Wp2_n CF _Ale2;
	schedule _Wp2_n CF _Rb2_n;
	schedule _Wp2_n CF _ENi;
	schedule _Wp2_n CF _ENo;
	schedule _Wp2_n CF _Re_c;
	schedule _Cle2 C _Cle2;
	schedule _Cle2 CF _Ale2;
	schedule _Cle2 CF _Rb2_n;
	schedule _Cle2 CF _ENi;
	schedule _Cle2 CF _ENo;
	schedule _Cle2 CF _Re_c;
	schedule _Ale2 C _Ale2;
	schedule _Ale2 CF _Rb2_n;
	schedule _Ale2 CF _ENi;
	schedule _Ale2 CF _ENo;
	schedule _Ale2 CF _Re_c;
	schedule _Rb2_n C _Rb2_n;
	schedule _Rb2_n CF _ENi;
	schedule _Rb2_n CF _ENo;
	schedule _Rb2_n CF _Re_c;
	schedule _ENi C _ENi;
	schedule _ENi CF _ENo;
	schedule _ENi CF _Re_c;
	schedule _ENo C _ENo;
	schedule _ENo CF _Re_c;
	schedule _Re_c C _Re_c;
endmodule

endpackage
