
/*
   Copyright (c) 2013, IIT Madras
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 *  Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *  Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *  Neither the name of IIT Madras  nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

/*

Author: Deepa N Sarma
Email id: deepans.88@gmail.com
 */

    package mcpu_defines;

    typedef enum{Little,Big} End deriving (Eq, Bits);

    function End endian(Bit#(32) addr,Bool sram_big);
    begin
         End type_end=Big;  
        
        `ifndef simulate
              if(addr>=32'h00000000 && addr<=32'h201FFFFF && !sram_big)
                type_end=Little;
              else
                if(addr>=32'h20000000 && addr<=32'h201FFFFF)
         `endif

         `ifdef simulate
            if(addr >=32'h20000000 && addr<=32'h201FFFFF)
         `endif
         type_end=Little;
         return type_end;
     end
    endfunction

    typedef struct
    {
    Bool endian_big;
    Bit #(32) addr;
    Bit #(32) wr_data;
    Bit #(2)  mode;
    Bit #(3) fun_code;
    Bit #(1) rd_req;//HIGH FOR READ,LOW FOR WRITE
    }Req_mcpu deriving (Bits,Eq);//Request from cpu

    typedef struct
    {
    Bool endian_big;
    Bit#(2) port_type;
    Bit#(32) data;
    Bit#(1)berr ; //r0 for bus_error 1 for no error 
    }Resp_mcpu deriving (Bits,Eq);//Response to cpu






    endpackage
