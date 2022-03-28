/*
Copyright (c) 2018, IIT Madras All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions
  and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of
  conditions and the following disclaimer in the documentation and/or other materials provided
 with the distribution.
* Neither the name of IIT Madras  nor the names of its contributors may be used to endorse or
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------

Author: Usha Kumari Krishnamurthy
Email id: usha.krishnamurthy@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/

package ecc_hamming; // Hamming codec for Error Detection and Correction

import Vector::*;
//import Randomizable ::*;
import LFSR::*;
`include "Logger.bsv"

  function Bit#(TAdd#(2,TLog#(databits))) fn_ecc_encode (Bit#(databits) data_word_in)
  		provisos(
  		  Add#(a__, TAdd#(1, TLog#(databits)), 7),
  		  Add#(b__, databits, 64)
  		);

    Bit#(7) p = 0;
    Bit#(64) data = zeroExtend(data_word_in);
    let v_p_size = valueOf(TAdd#(1,TLog#(databits)));
    //8
    p[0] = data[0] ^ data[1] ^ data[3] ^ data[4] ^ data[6];
    p[1] = data[0] ^ data[2] ^ data[3] ^ data[5] ^ data[6];
    p[2] = data[1] ^ data[2] ^ data[3] ^ data[7];
    p[3] = data[4] ^ data[5] ^ data[6] ^ data[7];
    //16
    if( v_p_size > 4) begin
      p[0] = p[0] ^ data[8] ^ data[10] ^ data[11] ^ data[13] ^ data[15];
      p[1] = p[1] ^ data[9] ^ data[10] ^ data[12] ^ data[13];
      p[2] = p[2] ^ data[8] ^ data[9] ^ data[10] ^ data[14] ^ data[15];
      p[3] = p[3] ^ data[8] ^ data[9] ^ data[10];
      p[4] = data[11] ^ data[12] ^ data[13] ^ data[14] ^ data[15];
    end
    //32
    if(v_p_size > 5) begin
      p[0] = p[0] ^ data[17] ^ data[19] ^ data[21] ^ data[23] ^ data[25] ^ data[26] ^ data[28] ^ 
                    data[30];
      p[1] = p[1] ^ data[16] ^ data[17] ^ data[20] ^ data[21] ^ data[24] ^ data[25] ^ data[27] ^ 
                    data[28] ^ data[31];
      p[2] = p[2] ^ data[16] ^ data[17] ^ data[22] ^ data[23] ^ data[24] ^ data[25] ^ data[29] ^ 
                    data[30] ^ data[31];
      p[3] = p[3] ^ data[18] ^ data[19] ^ data[20] ^ data[21] ^ data[22] ^ data[23] ^ data[24] ^ 
                    data[25];
      p[4] = p[4] ^ data[16] ^ data[17] ^ data[18] ^ data[19] ^ data[20] ^ data[21] ^ data[22] ^ 
                    data[23] ^ data[24] ^ data[25];
      p[5] = data[26] ^ data[27] ^ data[28] ^ data[29] ^ data[30] ^ data[31];
    end
    //64
    if(v_p_size > 6) begin
      p[0] = p[0] ^ data[32] ^ data[34] ^ data[36] ^ data[38] ^ data[40] ^ data[42] ^ data[44] ^ 
                    data[46] ^ data[48] ^ data[50] ^ data[52] ^ data[54] ^ data[56] ^ data[57] ^ 
                    data[59] ^ data[61] ^ data[63];
      p[1] = p[1] ^ data[32] ^ data[35] ^ data[36] ^ data[39] ^ data[40] ^ data[43] ^ data[44] ^ 
                    data[47] ^ data[48] ^ data[51] ^ data[52] ^ data[55] ^ data[56] ^ data[58] ^ 
                    data[59] ^ data[62] ^ data[63];
      p[2] = p[2] ^ data[32] ^ data[37] ^ data[38] ^ data[39] ^ data[40] ^ data[45] ^ data[46] ^ 
                    data[47] ^ data[48] ^ data[53] ^ data[54] ^ data[55] ^ data[56] ^ data[60] ^ 
                    data[61] ^ data[62] ^ data[63];
      p[3] = p[3] ^ data[33] ^ data[34] ^ data[35] ^ data[36] ^ data[37] ^ data[38] ^ data[39] ^ 
                    data[40] ^ data[49] ^ data[50] ^ data[51] ^ data[52] ^ data[53] ^ data[54] ^ 
                    data[55] ^ data[56];
      p[4] = p[4] ^ data[41] ^ data[42] ^ data[43] ^ data[44] ^ data[45] ^ data[46] ^ data[47] ^ 
                    data[48] ^ data[49] ^ data[50] ^ data[51] ^ data[52] ^ data[53] ^ data[54] ^ 
                    data[55] ^ data[56];
      p[5] = p[5] ^ data[32] ^ data[33] ^ data[34] ^ data[35] ^ data[36] ^ data[37] ^ data[38] ^ 
                    data[39] ^ data[40] ^ data[41] ^ data[42] ^ data[43] ^ data[44] ^ data[45] ^ 
                    data[46] ^ data[47] ^ data[48] ^ data[49] ^ data[50] ^ data[51] ^ data[52] ^ 
                    data[53] ^ data[54] ^ data[55] ^ data[56];
      p[6] = data[57] ^ data[58] ^ data[59] ^ data[60] ^ data[61] ^ data[62] ^ data[63];
    end

    Bit#(TAdd#(1,TLog#(databits))) actual_parity = truncate(p);
    Bit#(1) msb_parity = (^data_word_in) ^ (^actual_parity);
    return {msb_parity, actual_parity};
  endfunction

  function Tuple2#(Bit#(TAdd#(2,TLog#(databits))),Bit#(2)) fn_ecc_detect 
      (Bit#(databits) data, Bit#(TAdd#(2,TLog#(databits))) ecc_in)
      provisos(Add#(a__, databits, 64), Add#(TLog#(databits), b__, 6));
    Bit#(1) sed = 'b0;
    Bit#(1) ded = 'b0;

    Bit#(TAdd#(2,TLog#(databits))) enc_ecc = fn_ecc_encode(data);
    Bit#(TAdd#(1,TLog#(databits))) parity_check = truncate(ecc_in) ^ truncate(enc_ecc);
    Bit#(1) msb_parity_check = (^data) ^ (^ecc_in);
 
    sed = pack(({msb_parity_check,parity_check} != 0) && (msb_parity_check == 1));
    ded = pack(({msb_parity_check,parity_check} != 0) && (msb_parity_check == 0));
    return tuple2({msb_parity_check,parity_check}, {sed, ded});

  endfunction

  function Bit#(databits) fn_ecc_correct(Bit#(TAdd#(2,TLog#(databits))) ecc_check, 
                                         Bit#(TAdd#(2,TLog#(databits))) eccin,
                                         Bit#(databits)                 datain)
    provisos( Add#(databits, TAdd#(2,TLog#(databits)), encodebits));
    
    let v_encodebits = valueOf(encodebits);
    Bit#(TLog#(TAdd#(1,TLog#(databits)))) k=0;
    Bit#(TLog#(databits)) j=0;
    Bit#(TLog#(databits)) m = 0;

    Bit#(encodebits) encoded_word = ?;
    Bit#(TAdd#(1,TLog#(databits))) _temp = 1;
    Bit#(TAdd#(1,TLog#(databits))) index = truncate(ecc_check);
    Bit#(encodebits) error_mask = 0;
    error_mask[index] = 1'b1;
    error_mask = error_mask >> 1;
    for (Integer i = 0; i < v_encodebits-1; i = i + 1) begin
      if ( fromInteger(i+1) == _temp ) begin
    //    encoded_word[i] = eccin[k];
        _temp = _temp << 1;
    //    k = k+1;
      end
      else begin
        encoded_word[i] = datain[j];
        j=j+1;
      end
    end
    encoded_word = encoded_word ^ error_mask;

    Bit#(databits) corrected_word = ?;
    _temp = 1;
    for (Integer i = 0; i < v_encodebits-1; i = i + 1) begin
      if ( fromInteger(i+1) == _temp ) begin
        _temp = _temp << 1;
      end
      else begin
        corrected_word[m] = encoded_word[i];
        m=m+1;
      end
    end
    return corrected_word;
  endfunction



  function Bit#(TAdd#(2,TLog#(databits))) ecc_hamming_encode (Bit#(databits) data_word_in)
  		provisos(Add#(databits, TAdd#(1,TLog#(databits)), encodedbits)
  		);
		let v_paritybits = valueOf(TAdd#(1,TLog#(databits)));

		Bit#(TAdd#(2,databits)) data_word = {2'b00,data_word_in};
		let v_encodedbits = valueOf(encodedbits);
		Bit#(TAdd#(2,encodedbits)) encoded_word = '0;
		Bit#(TAdd#(1,TLog#(databits))) parity_word_index = 1;
		Bit#(TAdd#(1,TLog#(databits))) parity_word = '0;
		Bit#(TLog#(databits)) parity_word_lsb = '0;
		Bit#(TAdd#(1,TLog#(databits))) j = '0;
		Bit#(TAdd#(1,TLog#(TLog#(databits)))) k = '0;
		Bit#(1) extra_parity_ded = 1'b0;

		/* Fill a temp register with word to be encoded and with parity word(initially zero)at 
			bit positions which are powers of 2 like 1,2,4,8 etc.*/

		//$display("Encodedbits is = %d", v_encodedbits);
		for(Integer i=0; i <= v_encodedbits ; i=i+1) begin
			if (fromInteger(i+1) == parity_word_index) begin
				encoded_word [i+1] = parity_word[k];
				parity_word_index = parity_word_index << 1;
				k = k + 1;  
			end
			else begin
				encoded_word [i+1] = data_word [j];
				j = j +1;
			end
		end
		
		/*Compute the encoded parity as follows:
			* for bit position 1 of parity bits XOR all bits in the temp register with 
				the position's binary index having a 1 in bit 1
			  viz. Parity bit 1 covers all the bits positions whose binary representation 
				includes a 1 in the least significant position (1, 3, 5, 7, 9, 11, etc).
			* Parity bit 2 covers all the bits positions whose binary representation includes 
				a 1 in the second position from the least significant bit (2, 3, 6, 7, 10, 11, etc).	
		*/
		for (Integer m=0; fromInteger(m) <= k; m=m+1) begin
			for (Integer n=0; n <= v_encodedbits; n=n+1) begin
				Bit#(TAdd#(1,TLog#(encodedbits))) index_into_enc= fromInteger(n+1);
				if (index_into_enc[m] == 1'b1) begin
					parity_word[m] = parity_word[m] ^ encoded_word[index_into_enc];
				end
			end 
		end

		/* Add an extra parity bit over the above to enable double error detection */
		extra_parity_ded = ((^data_word_in)^(^parity_word));

		return {extra_parity_ded,parity_word};
  endfunction

  function  Tuple3#(Bit#(databits),Bit#(TAdd#(1,TLog#(databits))), Bool) ecc_hamming_decode_correct 
            (Bit#(databits) data_word_in, Bit#(TAdd#(2,TLog#(databits))) 
            parity_word_in, Bit#(1) det_only)
            provisos(Add#(databits, TAdd#(1,TLog#(databits)), encodedbits));

		let data_word = {2'b00,data_word_in};

		let v_encodedbits = valueOf(encodedbits);
		let v_databits = valueOf(databits);
		let v_paritybits = valueOf(TAdd#(1,TLog#(databits)));
		Bit#(TAdd#(2,encodedbits)) encoded_word = '0;
		Bit#(TAdd#(1,TLog#(databits))) parity_word_index = 1;
		Bit#(TAdd#(1,TLog#(databits))) decoded_parity_word = '0;
		Bit#(TAdd#(1,TLog#(databits))) decoded_parity_word_lsb = '0;
		Bit#(TAdd#(1,TLog#(databits))) decoded_parity_word_correct = '0;
		Bit#(TAdd#(1,TLog#(databits))) j = '0;
		Bit#(TAdd#(1,TLog#(TLog#(databits)))) k = '0;
		Bit#(TAdd#(2,databits)) correct_data_word = '0;
		Bit#(1) extra_decoded_parity_ded = 1'b0; 
		Bool ecc_error_detect_only_trap = False;
		Bit#(TAdd#(2,TLog#(databits)))parity_word = {1'b0,parity_word_in[v_paritybits-1:0]};


		/* Do the same as encoder to recover the encoded parity word which when no error should all be zeroes */
		for(Integer i=0; i <= v_encodedbits ; i=i+1) begin
			if (fromInteger(i+1) == parity_word_index) begin
				encoded_word [i+1] = parity_word[k];
				parity_word_index = parity_word_index << 1;
				k = k + 1;  
			end
			else begin
				encoded_word [i+1] = data_word [j];
				j = j +1;
			end
		end

                for (Integer m=0; fromInteger(m) <= k; m=m+1) begin
                        for (Integer n=0; n <= v_encodedbits; n=n+1) begin
                                Bit#(TAdd#(1,TLog#(encodedbits))) index_into_enc= fromInteger(n+1);
                                if (index_into_enc[m] == 1'b1) begin
                                        decoded_parity_word[m] = decoded_parity_word[m] ^ encoded_word[index_into_enc];
                                end
                        end
                end
		decoded_parity_word_lsb = decoded_parity_word[v_paritybits-1:0];
		
		/* Compute the extra parity bit over the above to detect double error - 
			when det_only is set single errors are also not corrected but only detected */
		extra_decoded_parity_ded = ((^data_word_in)^(^parity_word_in)) & ~det_only;

		/* When decoded parity word is not zero it is the index of the bit position which has been errored so toggle it
			This is the single error case when not detect-only; the extra parity bit is for double error detection  */
		if (({extra_decoded_parity_ded,decoded_parity_word_lsb} != 0) && (extra_decoded_parity_ded == 1'b1)) begin
			parity_word_index = 1;
			j = '0;
			for(Integer i=0; i <= v_encodedbits; i=i+1) begin
				if (fromInteger(i+1) == decoded_parity_word) begin
					encoded_word[i+1] = ~encoded_word[i+1];
				end
				if (fromInteger(i+1) == parity_word_index) begin
					parity_word_index = parity_word_index << 1;
				end
				else begin
					correct_data_word [j] = encoded_word [i+1];
					j = j +1;
				end
			end
		end
		
		/* in case of double error detection take a trap */
		else if (({extra_decoded_parity_ded,decoded_parity_word_lsb} != 0) && (extra_decoded_parity_ded == 1'b0)) begin
			ecc_error_detect_only_trap = True;
		end
		else begin
			correct_data_word = data_word;
		end
		return tuple3 (correct_data_word[v_databits-1:0], decoded_parity_word, ecc_error_detect_only_trap);
  endfunction
 


  /* (*noinline*)
  function Bit#(32) inst_correct (Bit#(7) ecc_check, Bit#(7) eccin, Bit#(32) datain);
    return fn_ecc_correct(ecc_check, eccin, datain);
  endfunction
  (*noinline*)
  function Vector#(16,Bit#(8)) ecc_inst (Vector#(16, Bit#(64)) data_word_in);
    Vector#(16, Bit#(8)) temp;
    for (Integer i = 0; i<16; i = i + 1) begin
      temp[i] = fn_ecc_encode(data_word_in[i]);
    end
    return temp;
  endfunction
  (*noinline*)
  function Vector#(16,Tuple2#(Bit#(8),Bit#(2))) ecc_detect_inst 
        (Vector#(16, Bit#(64)) data_word_in, Vector#(16,Bit#(8)) parity);
    Vector#(16,Tuple2#(Bit#(8),Bit#(2))) temp ;
    for (Integer i = 0; i<16; i = i + 1) begin
      temp[i] = fn_ecc_detect(data_word_in[i], parity[i]);
    end
    return temp;
  endfunction
  (*noinline*)
  function Vector#(16,Bit#(8)) ecc_inst2 (Vector#(16, Bit#(64)) data_word_in);
    Vector#(16, Bit#(8)) temp;
    for (Integer i = 0; i<16; i = i + 1) begin
      temp[i] = ecc_hamming_encode(data_word_in[i]);
    end
    return temp;
  endfunction */

 
//  (*synthesize*)
//  module mkTb(Empty);
//    /*doc:reg: */
//    Reg#(Bit#(20)) rg_data <- mkReg('h0001);
//    /*doc:rule: */
//    rule rl_check;
//      let parity = fn_ecc_encode(rg_data);
//      let {chparity, sed_ded} = fn_ecc_detect(rg_data ^ 'b00100, parity);
//      Bit#(32) _t = zeroExtend(rg_data);
//      let corrected_word <- fn_ecc_correct(chparity, parity, _t ^ 'b00100);
//      `logLevel( tb, 0, $format("TB: data:%h parity:%h chparity:%h sed_ded:%b cw:%h",
//                                     rg_data, parity, chparity, sed_ded, corrected_word ))
//      if(rg_data == 5)
//        $finish(0);
//      else
//        rg_data <= rg_data + 1;
//    endrule
//  endmodule


  /*(*noinline*)
  function Tuple3#(Bit#(`DATABITS ),Bit#(TAdd#(1,TLog#(`DATABITS ))), Bool) ecc_decode 
            (Bit#(`DATABITS) data_word_in, Bit#(TAdd#(2,TLog#(`DATABITS))) 
            parity_word_in, Bit#(1) det_only) = 
    ecc_hamming_decode_correct(data_word_in, parity_word_in, det_only);*/
/*
module mkTbECCHammingVerfy (Empty);

    //Randomize#(Bit#(`DATABITS)) rand_data <- mkGenericRandomizer;
    Reg#(Bit#(TLog#((`DATABITS)))) count <- mkReg(0);
    Reg#(Bool) rg_init <- mkReg(True);
    LFSR#(Bit#(32)) lfsr1 <- mkLFSR_32 ;
    LFSR#(Bit#(32)) lfsr2 <- mkLFSR_32 ;

    rule rl_init_randomizers(rg_init);
      rg_init <= False;
      //rand_data.cntrl.init();	
      lfsr1.seed('h99);
      lfsr2.seed('h11);	
    endrule

    rule rl_encode_and_decode(!rg_init);
      //let lv_rand_data <- rand_data.next;	
      //Bit#(`DATABITS) data_word = lv_rand_data;
      //Bit#(`DATABITS) data_word = 64'h0C0C0D0D0E0EF0F0;
      Bit#(`DATABITS) enc_data_word = '0;
      Bit#(TAdd#(2,TLog#(`DATABITS))) parity_word = '0;
      Bit#(`DATABITS) corrected_data = '0;
      Bit#(TAdd#(1,TLog#(`DATABITS))) decoded_parity = '0;
      Bool detect_trap = False;
      count <= count + 1;
       Bit#(`DATABITS) data_word = truncate({lfsr1.value,lfsr2.value});
      Bit#(`DATABITS) data_word_err = data_word;
      $display("Input Data Word ifor Encoding = %h", data_word );
      parity_word = ecc_encode (data_word);
      $display("Hamming encoded_word = %h, %b", enc_data_word, parity_word);
      //enc_data_word[33] = ~enc_data_word[33];
      data_word_err[count] = ~data_word_err[count];
      $display("Corrupted Data Word with injected Error = %h", data_word_err);
      {corrected_data,decoded_parity, detect_trap} = ecc_decode (data_word_err, parity_word, 1'b0);
      $display("Hamming decoded_word = %h, %b %b\n", corrected_data, decoded_parity, detect_trap);
      if (corrected_data != data_word)
        $display ("*************** Decoded Word Mismatch!!!! Not corrected properly");
                        enc_data_word[count] = ~enc_data_word[count];
      lfsr1.next;
      lfsr2.next;
    if (count == fromInteger(`DATABITS-1)) 
                  $finish(0);
    endrule

endmodule*/

endpackage: ecc_hamming
