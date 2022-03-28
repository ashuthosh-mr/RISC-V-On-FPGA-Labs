package LDPC_Decoder_no4cycle_col3;
import Vector :: *;
import FIFOF::*;
//(*doc = "note: The architecture is as follows- There is an overall interface "decoder". It has to sub-interfaces, one to receive input (cipher) and one to send output (decoded valid codeword). There is a module to which the interface "decoder" is tied to. Using this module, any single-error ciphers are corrected. If there is no or two error, the received cipher is printed as it is, without nay modification. However, this will not be able to correct any errors as n> 2^r - 1, where n is the codeword length, r is the number of rows. Nevertheless, the bit-flipping algorithm has been coded and it can used for a valid codeword length and message length." *)

//(*doc = "ifc: Overall block of decoder " *)
interface Ifc_Dec;

	//(*doc = "subifc: It receives 135 bits of cipher (cipher_in) in each clock cycle. " *)
	interface Subifc_Dec_In subifc_in;
	//(*doc = "subifc: It spits out 135 bits of codeword. " *)
	interface Subifc_Dec_Out subifc_out;
endinterface: Ifc_Dec

interface Subifc_Dec_In;
	
	//(*doc = "method: To tell the channel whether the decoder is ready to receive cipher. " *)
	method bit mv_decoder_rdy;
	//(*doc = "method: To store all the 135 bits in a vector" *)
	method Action ma_get_cipher(Bit#(135) cipher_in);
endinterface: Subifc_Dec_In

interface Subifc_Dec_Out;
	//(*doc = "method: To transmit the decoded codeword 135 bits. " *)
	method ActionValue#(Bit#(135)) mav_tx_decoded;
endinterface: Subifc_Dec_Out

//(*doc = "func:  This function performs matrix multiplication. It multiplies the row of matrix with the column vector message. It return each parity bit. " *)
function Bit#(7) fn_mul(Vector#(7,Bit#(135)) h_matrix, Bit#(135) codeword);
	
	//(*doc = "note: To store the syndrome(7 bits) and return it" *)	
	Bit#(7) result = 0;
	for(Integer i=0; i<7; i=i+1)begin
		for(Integer j=0; j<135; j=j+1)begin
			if(h_matrix[i][j]==1)
				result[i]=result[i]^codeword[j];
			end
		end
		return result;		
endfunction: fn_mul

//(*doc = "func: This function calculates the most likely value of the bit. " *)
function bit fn_m(Bit#(135) eqn,Bit#(135) codeword,Integer node);
	bit val = 0;
	for(Integer i=0;i<135;i=i+1) 
	begin
		//(*doc = "note: We must XOR all the specified bits of the parity equation except the that bit in consideration. " *)
		if(eqn[i]==1 && i!=node) 
			val = val^codeword[i];
	end
	return val;
endfunction: fn_m

//(*doc = "func: This function returns a vector consisting of the bit equations of each bit. " *)
//(*doc = "note: The bit equations were generated using c++. " *)
function Vector#(135,Vector#(3,Bit#(3))) fn_define_num;
	Vector#(135,Vector#(3,Bit#(3))) eqn_num= replicate(replicate(0));

eqn_num[134][0]= 0; eqn_num[134][1]= 5; eqn_num[134][2]= 6;//Bit134 equations: 0 ,5 ,6 ,
eqn_num[133][0]= 1; eqn_num[133][1]= 5; eqn_num[133][2]= 6;//Bit133 equations: 1 ,5 ,6 ,
eqn_num[132][0]= 2; eqn_num[132][1]= 5; eqn_num[132][2]= 6;//Bit132 equations: 2 ,5 ,6 ,
eqn_num[131][0]= 3; eqn_num[131][1]= 4; eqn_num[131][2]= 5;//Bit131 equations: 3 ,4 ,5 ,
eqn_num[130][0]= 4; eqn_num[130][1]= 5; eqn_num[130][2]= 6;//Bit130 equations: 4 ,5 ,6 ,
eqn_num[129][0]= 3; eqn_num[129][1]= 4; eqn_num[129][2]= 6;//Bit129 equations: 3 ,4 ,6 ,
eqn_num[128][0]= 3; eqn_num[128][1]= 5; eqn_num[128][2]= 6;//Bit128 equations: 3 ,5 ,6 ,
eqn_num[127][0]= 0; eqn_num[127][1]= 5; eqn_num[127][2]= 6;//Bit127 equations: 0 ,5 ,6 ,
eqn_num[126][0]= 0; eqn_num[126][1]= 5; eqn_num[126][2]= 6;//Bit126 equations: 0 ,5 ,6 ,
eqn_num[125][0]= 1; eqn_num[125][1]= 5; eqn_num[125][2]= 6;//Bit125 equations: 1 ,5 ,6 ,
eqn_num[124][0]= 0; eqn_num[124][1]= 5; eqn_num[124][2]= 6;//Bit124 equations: 0 ,5 ,6 ,
eqn_num[123][0]= 4; eqn_num[123][1]= 5; eqn_num[123][2]= 6;//Bit123 equations: 4 ,5 ,6 ,
eqn_num[122][0]= 0; eqn_num[122][1]= 5; eqn_num[122][2]= 6;//Bit122 equations: 0 ,5 ,6 ,
eqn_num[121][0]= 3; eqn_num[121][1]= 5; eqn_num[121][2]= 6;//Bit121 equations: 3 ,5 ,6 ,
eqn_num[120][0]= 4; eqn_num[120][1]= 5; eqn_num[120][2]= 6;//Bit120 equations: 4 ,5 ,6 ,
eqn_num[119][0]= 3; eqn_num[119][1]= 5; eqn_num[119][2]= 6;//Bit119 equations: 3 ,5 ,6 ,
eqn_num[118][0]= 1; eqn_num[118][1]= 5; eqn_num[118][2]= 6;//Bit118 equations: 1 ,5 ,6 ,
eqn_num[117][0]= 0; eqn_num[117][1]= 5; eqn_num[117][2]= 6;//Bit117 equations: 0 ,4 ,5 ,
eqn_num[116][0]= 1; eqn_num[116][1]= 4; eqn_num[116][2]= 6;//Bit116 equations: 1 ,4 ,6 ,
eqn_num[115][0]= 0; eqn_num[115][1]= 5; eqn_num[115][2]= 6;//Bit115 equations: 0 ,5 ,6 ,
eqn_num[114][0]= 3; eqn_num[114][1]= 5; eqn_num[114][2]= 6;//Bit114 equations: 3 ,5 ,6 ,
eqn_num[113][0]= 1; eqn_num[113][1]= 5; eqn_num[113][2]= 6;//Bit113 equations: 1 ,5 ,6 ,
eqn_num[112][0]= 3; eqn_num[112][1]= 5; eqn_num[112][2]= 6;//Bit112 equations: 3 ,5 ,6 ,
eqn_num[111][0]= 0; eqn_num[111][1]= 5; eqn_num[111][2]= 6;//Bit111 equations: 0 ,5 ,6 ,
eqn_num[110][0]= 3; eqn_num[110][1]= 5; eqn_num[110][2]= 6;//Bit110 equations: 3 ,5 ,6 ,
eqn_num[109][0]= 3; eqn_num[109][1]= 5; eqn_num[109][2]= 6;//Bit109 equations: 3 ,5 ,6 ,
eqn_num[108][0]= 0; eqn_num[108][1]= 5; eqn_num[108][2]= 6;//Bit108 equations: 0 ,5 ,6 ,
eqn_num[107][0]= 4; eqn_num[107][1]= 5; eqn_num[107][2]= 6;//Bit107 equations: 4 ,5 ,6 ,
eqn_num[106][0]= 2; eqn_num[106][1]= 5; eqn_num[106][2]= 6;//Bit106 equations: 2 ,5 ,6 ,
eqn_num[105][0]= 0; eqn_num[105][1]= 5; eqn_num[105][2]= 6;//Bit105 equations: 0 ,5 ,6 ,
eqn_num[104][0]= 0; eqn_num[104][1]= 5; eqn_num[104][2]= 6;//Bit104 equations: 0 ,5 ,6 ,
eqn_num[103][0]= 3; eqn_num[103][1]= 5; eqn_num[103][2]= 6;//Bit103 equations: 3 ,5 ,6 ,
eqn_num[102][0]= 3; eqn_num[102][1]= 5; eqn_num[102][2]= 6;//Bit102 equations: 3 ,5 ,6 ,
eqn_num[101][0]= 3; eqn_num[101][1]= 5; eqn_num[101][2]= 6;//Bit101 equations: 3 ,5 ,6 ,
eqn_num[100][0]= 1; eqn_num[100][1]= 5; eqn_num[100][2]= 6;//Bit100 equations: 1 ,5 ,6 ,
eqn_num[99][0]= 3; eqn_num[99][1]= 5; eqn_num[99][2]= 6;//Bit99 equations: 3 ,5 ,6 ,
eqn_num[98][0]= 4; eqn_num[98][1]= 5; eqn_num[98][2]= 6;//Bit98 equations: 4 ,5 ,6 ,
eqn_num[97][0]= 2; eqn_num[97][1]= 5; eqn_num[97][2]= 6;//Bit97 equations: 2 ,5 ,6 ,
eqn_num[96][0]= 3; eqn_num[96][1]= 5; eqn_num[96][2]= 6;//Bit96 equations: 3 ,5 ,6 ,
eqn_num[95][0]= 1; eqn_num[95][1]= 4; eqn_num[95][2]= 5;//Bit95 equations: 1 ,4 ,5 ,
eqn_num[94][0]= 4; eqn_num[94][1]= 5; eqn_num[94][2]= 6;//Bit94 equations: 4 ,5 ,6 ,
eqn_num[93][0]= 2; eqn_num[93][1]= 4; eqn_num[93][2]= 5;//Bit93 equations: 2 ,4 ,5 ,
eqn_num[92][0]= 1; eqn_num[92][1]= 5; eqn_num[92][2]= 6;//Bit92 equations: 1 ,5 ,6 ,
eqn_num[91][0]= 4; eqn_num[91][1]= 5; eqn_num[91][2]= 6;//Bit91 equations: 4 ,5 ,6 ,
eqn_num[90][0]= 4; eqn_num[90][1]= 5; eqn_num[90][2]= 6;//Bit90 equations: 4 ,5 ,6 ,
eqn_num[89][0]= 3; eqn_num[89][1]= 5; eqn_num[89][2]= 6;//Bit89 equations: 3 ,5 ,6 ,
eqn_num[88][0]= 3; eqn_num[88][1]= 5; eqn_num[88][2]= 6;//Bit88 equations: 3 ,5 ,6 ,
eqn_num[87][0]= 2; eqn_num[87][1]= 5; eqn_num[87][2]= 6;//Bit87 equations: 2 ,5 ,6 ,
eqn_num[86][0]= 4; eqn_num[86][1]= 5; eqn_num[86][2]= 6;//Bit86 equations: 4 ,5 ,6 ,
eqn_num[85][0]= 2; eqn_num[85][1]= 5; eqn_num[85][2]= 6;//Bit85 equations: 2 ,5 ,6 ,
eqn_num[84][0]= 3; eqn_num[84][1]= 5; eqn_num[84][2]= 6;//Bit84 equations: 3 ,5 ,6 ,
eqn_num[83][0]= 0; eqn_num[83][1]= 5; eqn_num[83][2]= 6;//Bit83 equations: 0 ,5 ,6 ,
eqn_num[82][0]= 1; eqn_num[82][1]= 5; eqn_num[82][2]= 6;//Bit82 equations: 1 ,5 ,6 ,
eqn_num[81][0]= 1; eqn_num[81][1]= 5; eqn_num[81][2]= 6;//Bit81 equations: 1 ,5 ,6 ,
eqn_num[80][0]= 2; eqn_num[80][1]= 5; eqn_num[80][2]= 6;//Bit80 equations: 2 ,5 ,6 ,
eqn_num[79][0]= 1; eqn_num[79][1]= 5; eqn_num[79][2]= 6;//Bit79 equations: 1 ,5 ,6 ,
eqn_num[78][0]= 1; eqn_num[78][1]= 5; eqn_num[78][2]= 6;//Bit78 equations: 1 ,5 ,6 ,
eqn_num[77][0]= 2; eqn_num[77][1]= 5; eqn_num[77][2]= 6;//Bit77 equations: 2 ,5 ,6 ,
eqn_num[76][0]= 2; eqn_num[76][1]= 5; eqn_num[76][2]= 6;//Bit76 equations: 2 ,4 ,5 ,
eqn_num[75][0]= 2; eqn_num[75][1]= 5; eqn_num[75][2]= 6;//Bit75 equations: 2 ,5 ,6 ,
eqn_num[74][0]= 2; eqn_num[74][1]= 5; eqn_num[74][2]= 6;//Bit74 equations: 2 ,4 ,6 ,
eqn_num[73][0]= 0; eqn_num[73][1]= 4; eqn_num[73][2]= 5;//Bit73 equations: 0 ,4 ,5 ,
eqn_num[72][0]= 1; eqn_num[72][1]= 5; eqn_num[72][2]= 6;//Bit72 equations: 1 ,5 ,6 ,
eqn_num[71][0]= 3; eqn_num[71][1]= 5; eqn_num[71][2]= 6;//Bit71 equations: 3 ,5 ,6 ,
eqn_num[70][0]= 0; eqn_num[70][1]= 5; eqn_num[70][2]= 6;//Bit70 equations: 0 ,5 ,6 ,
eqn_num[69][0]= 0; eqn_num[69][1]= 5; eqn_num[69][2]= 6;//Bit69 equations: 0 ,5 ,6 ,
eqn_num[68][0]= 2; eqn_num[68][1]= 5; eqn_num[68][2]= 6;//Bit68 equations: 2 ,5 ,6 ,
eqn_num[67][0]= 3; eqn_num[67][1]= 4; eqn_num[67][2]= 6;//Bit67 equations: 3 ,4 ,6 ,
eqn_num[66][0]= 2; eqn_num[66][1]= 5; eqn_num[66][2]= 6;//Bit66 equations: 2 ,5 ,6 ,
eqn_num[65][0]= 4; eqn_num[65][1]= 5; eqn_num[65][2]= 6;//Bit65 equations: 4 ,5 ,6 ,
eqn_num[64][0]= 4; eqn_num[64][1]= 5; eqn_num[64][2]= 6;//Bit64 equations: 4 ,5 ,6 ,
eqn_num[63][0]= 4; eqn_num[63][1]= 5; eqn_num[63][2]= 6;//Bit63 equations: 4 ,5 ,6 ,
eqn_num[62][0]= 3; eqn_num[62][1]= 5; eqn_num[62][2]= 6;//Bit62 equations: 3 ,5 ,6 ,
eqn_num[61][0]= 1; eqn_num[61][1]= 5; eqn_num[61][2]= 6;//Bit61 equations: 1 ,5 ,6 ,
eqn_num[60][0]= 1; eqn_num[60][1]= 5; eqn_num[60][2]= 6;//Bit60 equations: 1 ,5 ,6 ,
eqn_num[59][0]= 1; eqn_num[59][1]= 5; eqn_num[59][2]= 6;//Bit59 equations: 1 ,5 ,6 ,
eqn_num[58][0]= 3; eqn_num[58][1]= 5; eqn_num[58][2]= 6;//Bit58 equations: 3 ,5 ,6 ,
eqn_num[57][0]= 3; eqn_num[57][1]= 5; eqn_num[57][2]= 6;//Bit57 equations: 3 ,5 ,6 ,
eqn_num[56][0]= 1; eqn_num[56][1]= 4; eqn_num[68][2]= 5;//Bit56 equations: 1 ,4 ,5 ,
eqn_num[55][0]= 3; eqn_num[55][1]= 5; eqn_num[68][2]= 6;//Bit55 equations: 3 ,5 ,6 ,
eqn_num[54][0]= 3; eqn_num[54][1]= 5; eqn_num[54][2]= 6;//Bit54 equations: 3 ,5 ,6 ,
eqn_num[53][0]= 0; eqn_num[53][1]= 5; eqn_num[53][2]= 6;//Bit53 equations: 0 ,5 ,6 ,
eqn_num[52][0]= 1; eqn_num[52][1]= 5; eqn_num[52][2]= 6;//Bit52 equations: 1 ,5 ,6 ,
eqn_num[51][0]= 0; eqn_num[51][1]= 5; eqn_num[51][2]= 6;//Bit51 equations: 0 ,5 ,6 ,
eqn_num[50][0]= 0; eqn_num[50][1]= 5; eqn_num[50][2]= 6;//Bit50 equations: 0 ,5 ,6 ,
eqn_num[49][0]= 1; eqn_num[49][1]= 5; eqn_num[68][2]= 6;//Bit49 equations: 1 ,5 ,6 ,
eqn_num[48][0]= 1; eqn_num[48][1]= 4; eqn_num[48][2]= 6;//Bit48 equations: 1 ,4 ,5 ,
eqn_num[47][0]= 0; eqn_num[47][1]= 5; eqn_num[47][2]= 6;//Bit47 equations: 0 ,5 ,6 ,
eqn_num[46][0]= 2; eqn_num[46][1]= 5; eqn_num[46][2]= 6;//Bit46 equations: 2 ,5 ,6 ,
eqn_num[45][0]= 4; eqn_num[45][1]= 5; eqn_num[45][2]= 6;//Bit45 equations: 4 ,5 ,6 ,
eqn_num[44][0]= 3; eqn_num[44][1]= 5; eqn_num[44][2]= 6;//Bit44 equations: 3 ,5 ,6 ,
eqn_num[43][0]= 3; eqn_num[43][1]= 5; eqn_num[43][2]= 6;//Bit43 equations: 3 ,5 ,6 ,
eqn_num[42][0]= 1; eqn_num[42][1]= 5; eqn_num[42][2]= 6;//Bit42 equations: 1 ,5 ,6 ,
eqn_num[41][0]= 2; eqn_num[41][1]= 5; eqn_num[41][2]= 6;//Bit41 equations: 2 ,5 ,6 ,
eqn_num[40][0]= 3; eqn_num[40][1]= 5; eqn_num[40][2]= 6;//Bit40 equations: 3 ,5 ,6 ,
eqn_num[39][0]= 3; eqn_num[39][1]= 5; eqn_num[39][2]= 6;//Bit39 equations: 3 ,5 ,6 ,
eqn_num[38][0]= 3; eqn_num[38][1]= 5; eqn_num[38][2]= 6;//Bit38 equations: 3 ,5 ,6 ,
eqn_num[37][0]= 3; eqn_num[37][1]= 5; eqn_num[37][2]= 6;//Bit37 equations: 3 ,5 ,6 ,
eqn_num[36][0]= 2; eqn_num[36][1]= 5; eqn_num[36][2]= 6;//Bit36 equations: 2 ,5 ,6 ,
eqn_num[35][0]= 0; eqn_num[35][1]= 5; eqn_num[35][2]= 6;//Bit35 equations: 0 ,5 ,6 ,
eqn_num[34][0]= 0; eqn_num[34][1]= 5; eqn_num[34][2]= 6;//Bit34 equations: 0 ,5 ,6 ,
eqn_num[33][0]= 3; eqn_num[33][1]= 5; eqn_num[33][2]= 6;//Bit33 equations: 3 ,5 ,6 ,
eqn_num[32][0]= 0; eqn_num[32][1]= 4; eqn_num[68][2]= 6;//Bit32 equations: 0 ,4 ,6 ,
eqn_num[31][0]= 3; eqn_num[31][1]= 5; eqn_num[31][2]= 6;//Bit31 equations: 3 ,5 ,6 ,
eqn_num[30][0]= 2; eqn_num[30][1]= 5; eqn_num[30][2]= 6;//Bit30 equations: 2 ,5 ,6 ,
eqn_num[29][0]= 3; eqn_num[29][1]= 5; eqn_num[29][2]= 6;//Bit29 equations: 3 ,5 ,6 ,
eqn_num[28][0]= 2; eqn_num[28][1]= 5; eqn_num[28][2]= 6;//Bit28 equations: 2 ,5 ,6 ,
eqn_num[27][0]= 0; eqn_num[27][1]= 4; eqn_num[27][2]= 6;//Bit27 equations: 0 ,4 ,6 ,
eqn_num[26][0]= 3; eqn_num[26][1]= 5; eqn_num[26][2]= 6;//Bit26 equations: 3 ,5 ,6 ,
eqn_num[25][0]= 0; eqn_num[25][1]= 5; eqn_num[25][2]= 6;//Bit25 equations: 0 ,5 ,6 ,
eqn_num[24][0]= 4; eqn_num[24][1]= 5; eqn_num[24][2]= 6;//Bit24 equations: 4 ,5 ,6 ,
eqn_num[23][0]= 2; eqn_num[23][1]= 5; eqn_num[23][2]= 6;//Bit23 equations: 2 ,5 ,6 ,
eqn_num[22][0]= 4; eqn_num[22][1]= 5; eqn_num[22][2]= 6;//Bit22 equations: 4 ,5 ,6 ,
eqn_num[21][0]= 1; eqn_num[21][1]= 5; eqn_num[21][2]= 6;//Bit21 equations: 1 ,5 ,6 ,
eqn_num[20][0]= 4; eqn_num[20][1]= 5; eqn_num[20][2]= 6;//Bit20 equations: 4 ,5 ,6 ,
eqn_num[19][0]= 4; eqn_num[19][1]= 5; eqn_num[19][2]= 6;//Bit19 equations: 4 ,5 ,6 ,
eqn_num[18][0]= 4; eqn_num[18][1]= 5; eqn_num[18][2]= 6;//Bit18 equations: 4 ,5 ,6 ,
eqn_num[17][0]= 1; eqn_num[17][1]= 5; eqn_num[17][2]= 6;//Bit17 equations: 1 ,5 ,6 ,
eqn_num[16][0]= 2; eqn_num[16][1]= 5; eqn_num[16][2]= 6;//Bit16 equations: 2 ,5 ,6 ,
eqn_num[15][0]= 2; eqn_num[15][1]= 5; eqn_num[15][2]= 6;//Bit15 equations: 2 ,5 ,6 ,
eqn_num[14][0]= 1; eqn_num[14][1]= 5; eqn_num[14][2]= 6;//Bit14 equations: 1 ,5 ,6 ,
eqn_num[13][0]= 2; eqn_num[13][1]= 5; eqn_num[13][2]= 6;//Bit13 equations: 2 ,5 ,6 ,
eqn_num[12][0]= 4; eqn_num[12][1]= 5; eqn_num[12][2]= 6;//Bit12 equations: 4 ,5 ,6 ,
eqn_num[11][0]= 4; eqn_num[11][1]= 5; eqn_num[11][2]= 6;//Bit11 equations: 4 ,5 ,6 ,
eqn_num[10][0]= 4; eqn_num[10][1]= 5; eqn_num[10][2]= 6;//Bit10 equations: 4 ,5 ,6 ,
eqn_num[09][0]= 0; eqn_num[09][1]= 5; eqn_num[09][2]= 6;//Bit9 equations: 0 ,5 ,6 ,
eqn_num[08][0]= 4; eqn_num[08][1]= 5; eqn_num[08][2]= 6;//Bit8 equations: 4 ,5 ,6 ,
eqn_num[07][0]= 3; eqn_num[07][1]= 5; eqn_num[07][2]= 6;//Bit7 equations: 3 ,5 ,6 ,
eqn_num[06][0]= 2; eqn_num[06][1]= 5; eqn_num[06][2]= 6;//Bit6 equations: 2 ,5 ,6 ,
eqn_num[05][0]= 1; eqn_num[05][1]= 5; eqn_num[05][2]= 6;//Bit5 equations: 1 ,5 ,6 ,
eqn_num[04][0]= 3; eqn_num[04][1]= 5; eqn_num[04][2]= 6;//Bit4 equations: 3 ,5 ,6 ,
eqn_num[03][0]= 4; eqn_num[03][1]= 5; eqn_num[03][2]= 6;//Bit3 equations: 4 ,5 ,6 ,
eqn_num[02][0]= 2; eqn_num[02][1]= 5; eqn_num[02][2]= 6;//Bit2 equations: 2 ,5 ,6 ,
eqn_num[01][0]= 4; eqn_num[01][1]= 5; eqn_num[01][2]= 6;//Bit1 equations: 4 ,5 ,6 ,
eqn_num[00][0]= 4; eqn_num[00][1]= 5; eqn_num[00][2]= 6;//Bit0 equations: 4 ,5 ,6 ,

	return eqn_num;

endfunction: fn_define_num

//(*doc = "func: This function performs bit flipping algorithm. " *)
function Bit#(135) fn_bit_flipping(Vector#(7,Bit#(135)) h_matrix, Bit#(135) codeword);
	Vector#(135,Bit#(3)) m = replicate(0);
	Vector#(135,Vector#(3,Bit#(3))) num = replicate(replicate(0)); // 3 bit
	Integer i = 0;
	bit flag =0;
	
	num = fn_define_num();
	for(i=0;i<135&&flag==0;i=i+1) begin
		//(*doc = "note: To get the most likely bit values from the three parity equations in which the bit is present. " *)
		m[i][0] = fn_m(h_matrix[num[i][0]],codeword,i);
		m[i][1] = fn_m(h_matrix[num[i][1]],codeword,i);
		m[i][2] = fn_m(h_matrix[num[i][2]],codeword,i);
	end
	//for(i=0;i<135&&flag==0;i=i+1) begin
	//	m[i][2] = fn_m(h_matrix[num[i][2]],codeword,i);
	//end
	
	//(*doc = "note: The following codes of line perform bit flipping. " *)
	for(i=0;i<135;i=i+1) begin 
		if((m[i][0] == 0 && m[i][1] == 0 && m[i][2] == 0) || (m[i][0] == 1 && m[i][1] == 1 && m[i][2] == 1))
		begin
			codeword[i]=m[i][0];
			//let syndrome=fn_mul(h_matrix,codeword);
			//if(syndrome==0)	flag=1;
		end
	end
	
	return codeword;
endfunction: fn_bit_flipping


(*synthesize*)
module mkDecoder (Ifc_Dec);
	(*doc = "reg: To generate ready signal for decoder. " *)
	Reg#(bit) dec_rdy <- mkReg(1);
	
	(*doc = "fifo: To store the decoded value." *)
	FIFOF#(Bit#(135)) ff_decoded <- mkFIFOF();
	
	(*doc = "fifo: To store the syndrome. " *)
	FIFOF#(Bit#(7)) ff_syndrome <- mkFIFOF();
	
	(*doc = "reg: To store the received cipher. " *)
	Reg#(Bit#(135)) cipher <- mkReg(0);
	
	(*doc = "reg: To store the syndrome returned by the function. " *)
	Reg#(Bit#(7)) syndrome <- mkReg(1);
	
	(*doc = "reg: To keep track of the number of iterations. " *)
	Reg#(Bit#(32)) counter <- mkReg(0);

	//(*doc = "note: To store the rows of h matrix" *)
	Vector#(7,Bit#(135)) h_matrix;
	
//(*doc = "note: The matrix has been generated using the software. " *)
	h_matrix[0]=135'b100000011010100001010001001001100000000000000000000100000000010011000000000000000101100100000000000110100001010000000000000001000000000; 
	h_matrix[1]=135'b010000000100000010100100000000000010000100100000000011011000001000000000011100100010011000001000000000000000000001000100100000000100000;
	h_matrix[2]=135'b001000000000000000000000000010000000010001000001010000100111100000101000000000000000000010000100001000001010000100000011010000001000100;	
	h_matrix[3]=135'b000101100000010100001010110000011101001000000110001000000000000100010000100011011000000000110011110001010100100000000000000000010010000; 
	h_matrix[4]=135'b000111000001001001100000000100000000100111011000100000000010110000010111000000100000001001000000000000100001001010111000001110100001011;      
	h_matrix[5]=135'b111110111111111111011111111111111111111111111111111111111111011111101111111111111111111111111111111111011110111111111111111111111111111; 
	h_matrix[6]=135'b111011111111111110111111111111111111111010111111111111111101101111111111111111011111110111111111111111111111111111111111111111111111111;
	
	(*doc = "rule: To decode the incoming cipher data. " *)
	rule decoding(syndrome!=0&&cipher!=0);
		//(*doc = "note: To get the value of syndrome. " *)
		syndrome <= fn_mul(h_matrix,cipher);	
		//counter<= counter+1;
		$display($stime,": syndrome: %b, cipher1: %b ",syndrome,cipher);
			cipher <= fn_bit_flipping(h_matrix,cipher);
			
	endrule: decoding
	
	(*doc = "rule: To enque the fifo after decoding. " *)
	rule decoding_over(syndrome==0&&cipher!=0);
		//if(counter<200)	ff_decoded.enq(cipher);
		//else 	begin ff_decoded.enq(0); end
		//counter<=0;
		$display($stime," cipher2: %h,syndrome= %b ",cipher,syndrome);
		//(*doc = "note: To enque the value stored in fifo. " *)
		ff_decoded.enq(cipher);
		//(*doc = "note: To make the ready signal 1 as it has finished decoding. " *)
		dec_rdy<=1;
	endrule: decoding_over
	
	interface Subifc_Dec_In subifc_in;
		//(*doc = "method: To inform the channel whether it is ready to decode. " *)
		method bit mv_decoder_rdy;
			return dec_rdy;
		endmethod: mv_decoder_rdy
	
		method Action ma_get_cipher(Bit#(135) cipher_in);
			//(*doc = "note: The ready signal must be made 0. " *)
			dec_rdy <= 0;
			//(*doc = "note: To store the received cipher. " *)
			cipher <= cipher_in;
			//syndrome <= (fn_syndrome(h_matrix,cipher_in));
			//$display($stime,"in method:- cipher: %h, syndrome: %b ",cipher,syndrome);
					
		endmethod: ma_get_cipher
	endinterface: subifc_in
	
	interface Subifc_Dec_Out subifc_out;
		//(*doc = "method: To transmit the decoded codeword. " *)
		method ActionValue#(Bit#(135)) mav_tx_decoded if(dec_rdy==1);
		
			ff_decoded.deq();
			return ff_decoded.first();
				//else
			//return 0;
			endmethod: mav_tx_decoded
	endinterface: subifc_out
endmodule: mkDecoder

(*synthesize*)
module mkTest (Empty);
	//(*doc = "ifc: To create an instance of interface and instantiate it with mkDecoder. " *)
	Ifc_Dec ifc_dec <- mkDecoder;
	(*doc = "reg: To have a flag to make sure rule send_cipher is fired once. " *)
	Reg#(bit) flag <-mkReg(0);
	
	Reg#(Bit#(4)) flagd <- mkReg(0);
	
	//Reg#(Bit#(135)) codeword <- mkReg(135'b000000000000000000000000000000000000000000000000000000000000000000000000000000000000000010000000000000000000000000010000000001010000000);
	Reg#(bit) m0 <- mkReg(0);
	Reg#(bit) m1 <-mkReg(0);
	Vector#(7,Bit#(135)) h_matrix;
	// give values for h matrix
	h_matrix[0]=135'b100000011010100001010001001001100000000000000000000100000000010011000000000000000101100100000000000110100001010000000000000001000000000; 
	h_matrix[1]=135'b010000000100000010100100000000000010000100100000000011011000001000000000011100100010011000001000000000000000000001000100100000000100000;
	h_matrix[2]=135'b001000000000000000000000000010000000010001000001010000100111100000101000000000000000000010000100001000001010000100000011010000001000100;	
	h_matrix[3]=135'b000101100000010100001010110000011101001000000110001000000000000100010000100011011000000000110011110001010100100000000000000000010010000; 
	h_matrix[4]=135'b000111000001001001100000000100000000100111011000100000000010110000010111000000100000001001000000000000100001001010111000001110100001011;      
	h_matrix[5]=135'b111110111111111111011111111111111111111111111111111111111111011111101111111111111111111111111111111111011110111111111111111111111111111; 
	h_matrix[6]=135'b111011111111111110111111111111111111111010111111111111111101101111111111111111011111110111111111111111111111111111111111111111111111111;
	
	let cipher = 135'b111111101011001001111011000101011100011111111101010111000111101111000101010011101011110001100100000011010011001010110000100001011000011;
//135'b010001101111011110010001110110011110010000011101010001011010100010111010101111111110100001000000100011010101100100010010011001111100000 
//135'b010001101111011110010001110110011110010000011101010001011010100010111010101111111110100001000000100011010101100100010010011001111100000 
//135'b010011001110000010011011000010000011111000000011111001011010100110111101001100100110100000100011010010000001001110101010010111001111011 
//135'b011111101011001001111011000101011100011111111101010111000111101111000101010011101011110001100100000011010011001010110000100001011000011
//135'b011111101011001001111011000111011100001111111100000111100010101111101101010011101011110011100000001011011001001110110011110001010000111 
//135'b011111110001101001101010001100111100011111111101010011000111101100000101010011101110010101100100000101010011011010110000100000011000011 
//135'b010111101011001001111011000101011100011111111101010111000111101111000101010011101011110001100100000011010011001010110000100001011000011

(*doc = "rule: To send cipher"*)
rule send_cipher(flag==0);
ifc_dec.subifc_in.ma_get_cipher(cipher); 

$display($stime," : cipher= %b",cipher);
	flag <=1;
	endrule: send_cipher
	
	//rule debug (flagd<=2);
	//let x = fn_mul(h_matrix,cipher);
	//flagd<=flagd+1;
	//$display($stime,": syndrome: %b ",x);
	//endrule	
	
	(*doc = "rule: To receive decoded. " *)
	rule receive_decoded;
		let x <- ifc_dec.subifc_out.mav_tx_decoded;
		if(x!=0)
			$display($stime,": decoded: %b ",x);
		else
			$display($stime,": cant correct");
		let time1<-$stime;
		if(time1>200)
		$finish(0);
		
	endrule: receive_decoded	
endmodule: mkTest
endpackage: LDPC_Decoder_no4cycle_col3
