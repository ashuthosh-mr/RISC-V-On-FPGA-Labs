package InterfaceECCencoder ;

interface ECCencoder_NFCInterface   ;
 
	method Bit#(8) data_to_nfc ( ) ;
	method Action _data_from_nfc (Bit#(8) data_from_nfc) ;
	method Action _ecc_encoder_ce (Bit#(1) ecc_encoder_ce) ;
	method Action _ecc_encoder_start (Bit#(1) ecc_encoder_ce) ;

endinterface


import "BVI" encoder_wrapper =
module mkECCencoder(ECCencoder_NFCInterface) ;

	default_clock clk(clk); 
   	default_reset no_reset;
	
	method dout data_to_nfc ();
	method _data_from_nfc(din) enable((*inhigh*) en1);
	method _ecc_encoder_ce(ce) enable((*inhigh*) en2);
	method _ecc_encoder_start(start) enable((*inhigh*) en3);
	
	schedule (data_to_nfc) CF (data_to_nfc);
	schedule (data_to_nfc) CF (_ecc_encoder_ce);
	schedule (data_to_nfc) CF (_ecc_encoder_start);
	schedule (data_to_nfc) CF (_data_from_nfc);
	schedule (_data_from_nfc) CF (_ecc_encoder_ce);
	schedule (_data_from_nfc) CF (_data_from_nfc);
	schedule (_ecc_encoder_ce) CF (_ecc_encoder_start);
	schedule (_ecc_encoder_ce) CF (_ecc_encoder_ce);
	schedule (_data_from_nfc) CF (_ecc_encoder_start);
   	schedule (_ecc_encoder_start) CF (_ecc_encoder_start);

endmodule
endpackage
