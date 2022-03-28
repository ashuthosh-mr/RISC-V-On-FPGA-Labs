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

Author: Neel Gala
Email id: neelgala@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/

package device_common;

  import BUtils::*;

  typedef enum {Byte=0, HWord=1, Word=2, DWord=3} AccessSize deriving(Bits,Eq,FShow);
  instance Ord#(AccessSize);
  	function \< (e1,e2);
  		return (pack(e1)<pack(e2));
  	endfunction
  	function \<= (e1,e2);
  		return (pack(e1)<=pack(e2));
  	endfunction
  	function \> (e1,e2);
  		return (pack(e1)>pack(e2));
  	endfunction
  	function \>= (e1,e2);
  		return (pack(e1)>=pack(e2));
  	endfunction
  endinstance
	
  function Bit#(awidth) axi4burst_addrgen(Bit#(8) arlen, Bit#(3) arsize, Bit#(2) arburst, 
                                                                            Bit#(awidth) address);

		// this variable will decide the index above which part of the address should
		// not change in WRAP mode. Bits below this index value be incremented according
		// to the value of arlen and arsize;
		Bit#(3) wrap_size;
		case(arlen)
			3: wrap_size= 2;
			7: wrap_size= 3;
			15: wrap_size=4;
			default:wrap_size=1;
		endcase
  
    // this is address will directly be used for INCR mode
		Bit#(awidth) new_address=address+(('b1)<<arsize);
		Bit#(awidth) mask;
		mask=('1)<<(zeroExtend(arsize)+wrap_size);	// create a mask for bits which will remain constant in WRAP.
		Bit#(awidth) temp1=address& mask;	  // capture the constant part of the addr in WRAP.
		Bit#(awidth) temp2=new_address& (~mask);//capture the incremental part of the addr in WRAP.

		if(arburst==0) // FIXED
			return address;
		else if(arburst==1) // INCR
			return new_address;
		else // WRAP
			return temp1|temp2; // create the new address in the wrap mode by ORing the masked values.
	endfunction

  function Bit#(dwidth) data_shift(Bit#(dwidth) data, AccessSize size, 
                                                            Bit#(TAdd#(1,TDiv#(dwidth,32))) offset)
    provisos(Mul#(8, a__, dwidth),
             Mul#(16, b__, dwidth),
             Mul#(32, c__, dwidth));
    let shift_amount = {3'b0, offset}<<3;
    Bit#(dwidth) return_data=data>>shift_amount;
    if(size==Byte)
      return duplicate(return_data[7:0]);
    else if(size==HWord)
      return duplicate(return_data[15:0]);
    else if(size==Word)
      return duplicate(return_data[31:0]);
    else 
      return return_data;
  endfunction
endpackage: device_common
