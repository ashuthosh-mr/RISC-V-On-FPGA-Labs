package rm_decoder;
import Vector :: *;
import FIFOF::*;
//(*doc = "note: The architecture is as follows- There is an overall interface "decoder". It has to sub-interfaces, one to receive input (cipher) and one to send output (decoded valid codeword). There is a module to which the interface "decoder" is tied to. Using this module, any single-error ciphers are corrected. If there is no or two error, the received cipher is printed as it is, without nay modification " *)

//(*doc = "ifc: Overall decoder interface. " *)
interface Ifc_rm_decoder;
	//(*doc = "subifc: Subifc_Dec_In takes input(cipher) from the channel. " *)
	interface Subifc_Dec_In subifc_in;
	//(*doc = "subifc: Subifc_Dec_Out sends output(decoded valid codeword) to the channel. " *)
	interface Subifc_Dec_Out subifc_out;
endinterface: Ifc_rm_decoder

interface Subifc_Dec_In;
	//(*doc = "method: This Value method returns bit 1 if the decoder is ready to receive cipher and bit 0 if the decoder is not ready. " *)
	method bit mv_decoder_rdy;
	//(*doc = "method: This Action method receives cipher(256 bits) from the channel. " *)
	method Action ma_get_cipher(Bit#(256) lv_cipher_in);
endinterface: Subifc_Dec_In

interface Subifc_Dec_Out;
	//(*doc = "method: This ActionValue method sends decoded codeword(256 bits) to the channel. " *)
	method ActionValue#(Bit#(256)) mav_tx_decoded;
endinterface: Subifc_Dec_Out

//(*doc = "func: This function decodes a cipher with single errors. It takes the parity check matrix(9x256) and cipher (256 bits) as arguements. If there are no or more than one errors, it will simply return the received cipher. Hence, it returns a 256 bit codeword/cipher. " *)
function Bit#(256) fn_decode(Vector#(9,Bit#(256)) v_h_matrix, Bit#(256) lv_codeword);
		//(*doc = "note: syndrome is the result obtained after multiplying the parity check matrix and cipher vector. In this case, it will be a 9 bit vector. It is important to initialise it to 0. " *)
		Bit#(9) lv_syndrome = 0;
	//(*doc = "note: The matrix multiplication is performed as follows: " *)
	//(*doc = "note: <-------------------------------------------------- " *)
	//(*doc = "note: h255,0 | h254,0 | h253,0 |... | h2,0 | h1,0 | h0,0 | 	^ c255 " *)
	//(*doc = "note: h255,1 | h254,1 | h253,1 |... | h2,1 | h1,1 | h0,1 |	| c254 " *)
	//(*doc = "note: h255,2 | h254,2 | h253,2 |... | h2,2 | h1,2 | h0,2 | 	| c253 " *)
	//(*doc = "note: h255,3 | h254,3 | h253,3 |... | h2,3 | h1,3 | h0,3 | 	| c252 " *)
	//(*doc = "note: h255,4 | h254,4 | h253,4 |... | h2,4 | h1,4 | h0,4 | 	| c251 " *)
	//(*doc = "note: h255,5 | h254,5 | h253,5 |... | h2,5 | h1,5 | h0,5 | 	|  .   " *)
	//(*doc = "note: h255,6 | h254,6 | h253,6 |... | h2,6 | h1,6 | h0,6 | 	|  .   " *)
	//(*doc = "note: h255,7 | h254,7 | h253,7 |... | h2,7 | h1,7 | h0,7 | 	|  .   " *)
	//(*doc = "note: h255,8 | h254,8 | h253,8 |... | h2,8 | h1,8 | h0,8 | 	|  .   " *)
	//(*doc = "note:                                                      	|  .   " *)
	//(*doc = "note:                                                      	|  .   " *)
	//(*doc = "note:                                                      	|  .   " *)
	//(*doc = "note:                                                      	|  c2  " *)
	//(*doc = "note:                                                      	|  c1  " *)
	//(*doc = "note:                                                      	|  c0  " *)
	for(Integer lv_i=0; lv_i<9; lv_i=lv_i+1)begin
		for(Integer lv_j=0; lv_j<256; lv_j=lv_j+1)begin
		//(*doc = "note: When an element of row of the pchk matrix is 1, we will XOR the value of 8-i th bit of syndrome vector with the previously stored value of codeword in the jth location. We need to XOR to do mod2 addition. "*)
		//(*doc = "note: We are considering 8-i because we are starting from row 0 to row 8. However we need to store it in MSB to LSB format in syndrome. " *)
			if(v_h_matrix[lv_i][lv_j]==1)
				lv_syndrome[8-lv_i]=lv_syndrome[8-lv_i]^lv_codeword[lv_j];
			end
		end
		//(*doc = "note: The value of the syndrome is the position of the bit in which single error occured. Thus a syndrome '0' would mean that an error occured in the zeroth position. When there is no error in the codeword, syndrome is 9'b100000000. When more than two errors occur, syndrome lies in the remaining possible values. ")
		if(lv_syndrome<256)
		//(*doc : "note: Flip the bit in the location specified by the syndrome's value. " *)
			lv_codeword[lv_syndrome] = ~(lv_codeword[lv_syndrome]);
			
	return lv_codeword;	
endfunction: fn_decode

(*synthesize*)
(*doc = "module: Decoder module " *)
module mk_rm_decoder (Ifc_rm_decoder);

	(*doc = "reg: This register is the ready signal sent by the decoder before it can accept any cipher from the channel. It is set to one." *)
	Reg#(bit) rg_dec_rdy <- mkReg(1);
	
	(*doc = "fifo: This FIFO stores the decoded value. " *)
	FIFOF#(Bit#(256)) ff_decoded <- mkFIFOF();
	
	(*doc = "reg: This register stores the cipher received from the channel through the Action method ma_get_cipher. " *)
	Reg#(Bit#(256)) rg_cipher <- mkReg(0);
	
	(*doc = "reg: This register stores the syndrome from the function fn_decode, if the function is made to return the syndrome. Otherwise, we will not be using it in the code. It is set to one for debugging purposes. " *)
	Reg#(Bit#(9)) rg_syndrome <- mkReg(1);
	
	(*doc = "reg: This register is used to store the codeword returned by the function fn_decode. It is initialised to zero. " *)
	Reg#(Bit#(256)) rg_codeword<-mkReg(0);
	
	//(*doc = "note: To store the rows of partiy check matrix as a vector of 9 elements, each with a size of 256 bits. " *)
	Vector#(9,Bit#(256)) v_h_matrix;

//(*doc = "note: The parity check matrix has been generated using the c++ program. "*)
v_h_matrix[0]=256'b1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111;
v_h_matrix[1]=256'b1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
v_h_matrix[2]=256'b1111111111111111111111111111111111111111111111111111111111111111000000000000000000000000000000000000000000000000000000000000000011111111111111111111111111111111111111111111111111111111111111110000000000000000000000000000000000000000000000000000000000000000;
v_h_matrix[3]=256'b1111111111111111111111111111111100000000000000000000000000000000111111111111111111111111111111110000000000000000000000000000000011111111111111111111111111111111000000000000000000000000000000001111111111111111111111111111111100000000000000000000000000000000;
v_h_matrix[4]=256'b1111111111111111000000000000000011111111111111110000000000000000111111111111111100000000000000001111111111111111000000000000000011111111111111110000000000000000111111111111111100000000000000001111111111111111000000000000000011111111111111110000000000000000;
v_h_matrix[5]=256'b1111111100000000111111110000000011111111000000001111111100000000111111110000000011111111000000001111111100000000111111110000000011111111000000001111111100000000111111110000000011111111000000001111111100000000111111110000000011111111000000001111111100000000;
v_h_matrix[6]=256'b1111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000111100001111000011110000;
v_h_matrix[7]=256'b1100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100110011001100;
v_h_matrix[8]=256'b1010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010;

	(*doc = "rule: To decode the received cipher if there is any single bit error. " *)
	rule decoding;
		//(*doc = "reg: To store the codeword(256 bits) returned by the function 'fn_decode'. " *)
		rg_codeword <= fn_decode(v_h_matrix,rg_cipher);	
		$display($stime,":decoded1: %b, \n cipher1: %b",rg_codeword,rg_cipher);
		
		//(*doc = "reg: To make the decoder ready for receiving the next set of codewords. " *)
		rg_dec_rdy<=1;
	endrule: decoding

	interface Subifc_Dec_In subifc_in;
		//(*doc = "method: This value method returns the ready signal of the decoder. " *)
		method bit mv_decoder_rdy;
			return rg_dec_rdy;
		endmethod: mv_decoder_rdy
		
		//(*doc = "method: This Action method receives the cipher(256 bits) from the channel. " *)
		method Action ma_get_cipher(Bit#(256) lv_cipher_in);
			//(*doc = "reg: The ready signal is made zero as it has just received the cipher from the channel. " *)
			rg_dec_rdy <= 0;
			
			//(*doc = "reg: The received cipher is stored in rg_codeword. " *)
			rg_cipher <= lv_cipher_in;
		endmethod: ma_get_cipher
	endinterface: subifc_in
	
	interface Subifc_Dec_Out subifc_out;
		//(*doc = "method: This ActionValue method returns the decoded value to the channel only if the ready signal is 1. " *)
		method ActionValue#(Bit#(256)) mav_tx_decoded if(rg_dec_rdy==1);
		
			//(*doc = "fifo: To remove the contents of the FIFO so that the next set of decoded codeword can be stored. " *)
			ff_decoded.deq();
			//(*doc = "fifo: To return the contents of FIFO which has the codeword." *)
			return ff_decoded.first();
			
		endmethod: mav_tx_decoded
	endinterface: subifc_out
endmodule: mk_rm_decoder

(*synthesize*)
(*doc = "module: Testbench module" *)
module mkTb (Empty);
	//(*doc = "Ifc:  To declare an instance name and instantiate with mk_rm_decoder" *)
	Ifc_rm_decoder ifc_dec <- mk_rm_decoder;
	
	(*doc = "reg: To declare a register to execute the rule 'send_cipher' once. " *)
	Reg#(bit) rg_flag <- mkReg(0);
	//(*doc = "note: To declare a variable to store the cipher before passing into the appropriate method. " *)
	let lv_cipher = 256'b0110100011100010100110100101110100001111011001111001011001111101001111011110000101001010111000100101110101100111111000010001100101101101001100011010011100011100101111111001001100100011000110101111111010101000011111100111001011100010011101001100001010111000;
	(*doc = "rule:  To send cipher of 256 bits" *)
	rule send_cipher(rg_flag==0);
		//(*doc = "method: The Action method "ma_get_cipher" is called by passing "lv_cipher" as argument. " *)
		ifc_dec.subifc_in.ma_get_cipher(lv_cipher); 
		$display($stime," : cipher= %b",lv_cipher);
		
		//(*doc = "reg: To change the flag value to 1 so that this rule won't be fired again. " *)
		rg_flag <=1;
	endrule: send_cipher

	(*doc = "rule: To receive valid codeword from decoder" *)
	rule receive_decoded;
	//(*doc = "note: The following lines may be used to call the method 'mav_tx_codeword'. However, since we are already displayed the correct codeword in the rule decoding, it has been commented. Appropriate changes may be made. " *)
//		let x <- ifc_dec.subifc_out.mav_tx_decoded;
//		if(x!=0)
//			$display($stime,": decoded: %b ",x);

		//(*doc = "note: The following code lines are to finish the simulation. " *)
		let time1<-$stime;
		if(time1>20)
			$finish(0);
	endrule: receive_decoded	
endmodule: mkTb
endpackage: rm_decoder
