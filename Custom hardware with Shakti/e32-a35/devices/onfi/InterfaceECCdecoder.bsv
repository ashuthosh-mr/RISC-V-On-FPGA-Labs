package InterfaceECCdecoder ;

interface ECCdecoder_NFCInterface   ;
 
	method Action _data_from_nfc (Bit#(8) data_from_nfc ) ;
	method Action _ecc_decoder_ce (Bit#(1) ecc_decoder_ce) ;
	method Action _ecc_decoder_start (Bit#(1) ecc_decoder_ce) ;

endinterface


import "BVI" decoder_wrapper =
module mkECCdecoder(ECCdecoder_NFCInterface) ;

	default_clock clk(clk); 
   	default_reset no_reset;
	
	method _data_from_nfc(din) enable((*inhigh*) en1);
	method _ecc_decoder_ce(ce) enable((*inhigh*) en2);
	method _ecc_decoder_start(start) enable((*inhigh*) en3);
	
	schedule (_data_from_nfc) CF (_ecc_decoder_ce);
	schedule (_data_from_nfc) CF (_data_from_nfc);
	schedule (_ecc_decoder_ce) CF (_ecc_decoder_start);
	schedule (_ecc_decoder_ce) CF (_ecc_decoder_ce);
	schedule (_data_from_nfc) CF (_ecc_decoder_start);
   	schedule (_ecc_decoder_start) CF (_ecc_decoder_start);

endmodule
endpackage
