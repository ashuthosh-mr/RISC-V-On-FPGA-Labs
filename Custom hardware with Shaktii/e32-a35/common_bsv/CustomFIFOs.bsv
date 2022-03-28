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

Author: Neel Gala
Email id: neelgala@gmail.com
Details:

--------------------------------------------------------------------------------------------------
*/
package CustomFIFOs;
  import Vector::*;
  import FIFOF::*;
  import DReg::*;
  import SpecialFIFOs::*;
  import FIFO::*;

  module mkUGBypassFIFOF (FIFOF#(a))
    provisos (Bits#(a,sa));

   // STATE ----------------

    Reg#(Maybe#(a)) rv[3] <- mkCReg(3, tagged Invalid);

    // INTERFACE ----------------

    Bool enq_ok = ! isValid(rv[0]);
    Bool deq_ok = isValid(rv[1]);

    method notFull = enq_ok;

    method Action enq(v);
       rv[0] <= tagged Valid v;
    endmethod

    method notEmpty = deq_ok;

    method Action deq();
       rv[1] <= tagged Invalid;
    endmethod

    method first(); // deq_ok
       return fromMaybe(unpack(0),rv[1]);
    endmethod

    method Action clear();
       rv[2] <= tagged Invalid;
    endmethod

  endmodule

  interface Ifc_PipeFIFOF#(type a);
    interface FIFOF#(a) fifo;
    method Tuple2#(Bool, a) first_data;
    method Tuple2#(Bool, a) second_data;
  endinterface

  module mkPipeFIFOF(Ifc_PipeFIFOF#(a))
    provisos(Bits#(a, a__));
    Reg#(Bit#(1)) head <- mkReg(0);
    Reg#(Bit#(1)) tail <- mkReg(0);
    FIFOF#(a) ff [2];
    for (Integer i=0;i<2;i=i+1)
      ff[i]<-mkUGFIFOF1;
    interface fifo = interface FIFOF
      method Action enq(v) if(ff[head].notFull);
        ff[head].enq(v);
        head<= ~head;
      endmethod
      method first() if(ff[tail].notEmpty);
        return ff[tail].first;
      endmethod
      method Action deq() if (ff[tail].notEmpty);
        tail<= ~tail;
        ff[tail].deq;
      endmethod
      method Action clear;
        ff[0].clear;
        ff[1].clear;
        tail<= head;
      endmethod
      method notEmpty = ff[tail].notEmpty;
      method notFull  = ff[head].notFull;
    endinterface;
    method first_data = tuple2(ff[tail].notEmpty, ff[tail].first);
    method second_data = tuple2(ff[tail-1].notEmpty, ff[tail-1].first);
  endmodule

//  (*synthesize*)
//  module mkTb(Empty);
//    Reg#(Bit#(32)) counter <- mkReg(0);
//    Ifc_PipeFIFOF#(Bit#(32)) ff <- mkPipeFIFOF;
//    rule term;
//      if(counter==50)
//        $finish(0);
//      else
//        $display("\n");
//    endrule
//    rule enq;
//      ff.fifo.enq(counter);
//      counter<= counter+1;
//      $display($time, "\tENQ: counter: %d", counter);
//    endrule
//
//    rule deq(counter%2==0);
//      ff.fifo.deq;
//      $display($time, "\tDEQ. First: %d", ff.fifo.first);
//    endrule
//
//    rule display;
//      $display($time, "\tFIRST: ", fshow(ff.first_data), " SECOND: ", fshow(ff.second_data));
//    endrule
//  endmodule
//
//  (*synthesize*)
//  module mktemp1(FIFOF#(Bit#(32)));
//    let ifc();
//    mkFIFOF _temp(ifc);
//    return (ifc);
//  endmodule
//  (*synthesize*)
//  module mktemp2(Ifc_PipeFIFOF#(Bit#(32)));
//    let ifc();
//    mkPipeFIFOF _temp(ifc);
//    return (ifc);
//  endmodule
endpackage

