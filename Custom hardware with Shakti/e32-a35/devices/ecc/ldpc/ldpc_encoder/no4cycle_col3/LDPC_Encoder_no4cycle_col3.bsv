package LDPC_Encoder_no4cycle_col3;
import Vector :: *;
import FIFOF::*;
//(*doc= "note: LDPC Encoder Code" *)
//(*doc = "note: =========================================================================================" *)
//(*doc = "note: In this code, we are considering 16 bytes (128 bits) of message with 7 bits of parity. In each clock cycle, 16 bytes (128bits) of message data is received which is referred to as "message_in". Now, the encoder starts performing encoding and spits of out codeword of 135 bits after 1 cycle. This is referred to as "codeword_out". The encoder has a generator matrix of size 7x128. c=Gm where c is matrix of parity bits (7x1), G is generator matrix (7x128) and m is matrix of message bits (128x1). The codeword so generator is of the form  c = {p,m} where p is sub matrix of 7 parity bits and m is the matrix of message of 128 bits." *)
//(*doc = "note: =========================================================================================" *)

//(*doc = "note: The architecture is as follows- There is an overall interface "Encoder". It has to sub-interfaces, one to receive input (message) and one to send output (codeword). There is a module to which the interface "encoder" is tied to. Using this module, valid codewords are generated. " *)

//(*doc = "ifc: Overall block of encoder" *)
interface Ifc_Enc;
	
	//(*doc = "subifc: It receives 128 bits of data (message_in) in each clock cycle" *)
	interface Subifc_Enc_In subifc_in;
	
	//(*doc = "subifc: It spits out 135 bits of data (codeword_out) after 1 clock cycle" *)
	interface Subifc_Enc_Out subifc_out;
	
endinterface: Ifc_Enc

interface Subifc_Enc_In;
	
	//(*doc = "method: To store all the 128 bits in a vector. " *)
	method Action ma_get_message(Bit#(128) message_in);
	
endinterface: Subifc_Enc_In

interface Subifc_Enc_Out;
	
	//(*doc = "method: To transmit the codeword 135 bits after the channel is ready to accept encoded message bits" *)
	method ActionValue#(Bit#(135)) mav_tx_codeword;
	
endinterface: Subifc_Enc_Out

//(*doc = "func:  This function performs matrix multiplication. It multiplies the row of matrix with the column vector message. It return each parity bit. " *)
function Bit#(1) fn_mul(Bit#(128) row, Bit#(128) col);

  //(*doc = "note: To store the parity bit and return it" *)
  Bit#(1) p = 0;
  
  //(*doc = "note: To iterate 128 times as there are 128 columns in the generator matrix. " *)
  for(Integer i=0;i<128;i=i+1)
  begin
	if(row[i] == 1)
	  p = p ^ col[i];
  end
  return p;
endfunction

(*synthesize*)
module mkEncoder (Ifc_Enc);
	
	(*doc = "fifo:  To store the codeword after the parity bits are generated " *)
	FIFOF#(Bit#(135)) ff_output <- mkFIFOF();
	
	//(*doc = "note: To store the rows of generator matrix as vector of 7 elements, each with size of 128 bits. " *)
	Vector#(7,Bit#(128)) gen;
		
gen[0]=128'b11010100001010001001001100000000000000000000100000000010011000000000000000101100100000000000110100001010000000000000001000000000;
gen[1]=128'b00100000010100100000000000010000100100000000011011000001000000000011100100010011000001000000000000000000001000100100000000100000;
gen[2]=128'b00000000000000000000010000000010001000001010000100111100000101000000000000000000010000100001000001010000100000011010000001000100;
gen[3]=128'b00000000001000000000000000000000101000000000000000010010000000000000000100000001000000000000000000000000000000000000000000000000;
gen[4]=128'b00001001000000000000100000000100010011000100000000000000000000111000000000000000001000000000000000000001010111000001110100001011;
gen[5]=128'b00000000000100000000000000000000000000000000000000000100000010000000000000000000000000000000000100001000000000000000000000000000;
gen[6]=128'b00000010101101010110000011101001101000110001000000010110100000000100011111000001000110011110001110101100000000000000000010010000;		
	
	interface Subifc_Enc_In subifc_in;	
		method Action ma_get_message(Bit#(128) message_in);	
		
		 //$display($stime," message_in: %b ",message_in);
 			
 			//(*doc = "note: To generate each parity bit, fn_mul is called by passing each row vector of generator matrix and incoming message as column. " *)
			let p0 = fn_mul(gen[0],message_in);
			let p1 = fn_mul(gen[1],message_in);
			let p2 = fn_mul(gen[2],message_in);
			let p3 = fn_mul(gen[3],message_in);
			let p4 = fn_mul(gen[4],message_in);
			let p5 = fn_mul(gen[5],message_in);
			let p6 = fn_mul(gen[6],message_in);
			
			//(*doc = "note: To store the codeword in FIFO. " *)
			ff_output.enq({p0,p1,p2,p3,p4,p5,p6,message_in});
		endmethod: ma_get_message
	endinterface

	interface Subifc_Enc_Out subifc_out;
		method ActionValue#(Bit#(135)) mav_tx_codeword;
			//(*doc = "note: To check if the channel is ready to receive the codeword. " *)
			
			begin
			//(*doc = "note: To remove the contents of the FIFO so that the next set of codeword can be stored. " *)
			ff_output.deq;
			
			//(*doc = "note: To return the contents of FIFO which has the codeword." *)
			return ff_output.first();
			end
			
		endmethod: mav_tx_codeword
	endinterface

endmodule: mkEncoder

(*synthesize*)
module mkTb (Empty);
	
	//(*doc = "ifc: To declare an instance name and instantiate with mkEncoder" *)
	Ifc_Enc enc<-mkEncoder;
	let message = 128'b01011001001111011000101011100011111111101010111000111101111000101010011101011110001100100000011010011001010110000100001011000011; 
	
	(*doc = "rule:  To keep sending input message of 128 bits" *)
	rule in ;
			$display($stime," message: %b",message);
			enc.subifc_in.ma_get_message(message);
	endrule: in
	
	(*doc = "rule:  To receive coddeword from Encoder" *)
	rule out;
		let x <- enc.subifc_out.mav_tx_codeword;
		 $display($stime, " codeword: %b ",x);
		 
		let time1 <- $stime;
		if(time1 >20)	
			$finish(0);
		
	endrule: out		
endmodule: mkTb

endpackage: LDPC_Encoder_no4cycle_col3

