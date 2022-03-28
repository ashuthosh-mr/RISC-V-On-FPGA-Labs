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

Author: Rahul Bodduna 
Email id: rahul.bodduna@gmail.com 
Details:

--------------------------------------------------------------------------------------------------
*/

package CWire;

import Vector ::*;
import DefaultValue ::*;

interface CWire_read#(type a); 
	method a _read;
endinterface

interface CWire_write#(type a); 
	method Action _write(a x);
endinterface

interface CWire#(numeric type ports, type a);
	interface Vector#(ports, CWire_read#(a)) read;
	interface Vector#(ports, CWire_write#(a)) write;
endinterface

module mkCWire(CWire#(ports, a)) provisos(Bits#(a, a__),
													DefaultValue::DefaultValue#(a));

	Vector#(ports, RWire#(a)) wr_in <- replicateM(mkRWire());
	Vector#(ports, Wire#(a)) wr_out <- replicateM(mkDWire(defaultValue));
	
	for(Integer i=0; i<valueOf(ports)-1; i=i+1) begin
		rule rl_mux_wires;
			if(wr_in[i].wget matches tagged Valid .data) begin
				wr_out[i+1] <= data;
			end
			else 
				wr_out[i+1] <= wr_out[i];
		endrule
	end

Vector#(ports, CWire_read#(a)) temp_read;
Vector#(ports, CWire_write#(a)) temp_write;

for(Integer i=0; i<valueOf(ports); i=i+1) begin
	
	temp_read[i] = interface CWire_read
										method a _read;
											return wr_out[i];
										endmethod
								 endinterface;

	temp_write[i] = interface CWire_write
										method Action _write(a x);
											wr_in[i].wset(x);
										endmethod
									endinterface;
end

interface read = temp_read;
interface write = temp_write;

endmodule
endpackage
