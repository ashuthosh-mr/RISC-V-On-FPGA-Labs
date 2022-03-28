package encoder;

import Vector ::* ;
`define INPUT 1024 
`define OUTPUT 10 
interface Ifc_encoder#(numeric type irpins);
	method Bit#(TLog#(irpins)) encode(Bit#(irpins) ip);
endinterface

module mkencoder(Ifc_encoder#(irpins))
		provisos(Log#(irpins, irid),
				 Add#(a__, irpins, `INPUT),
				 Add#(b__, irid, `OUTPUT));
	function Bit#(irid) fn_encoder(Bit#(irpins) irp);
		Bit#(`INPUT) ip = zeroExtend(irp);
		Bit#(`OUTPUT) result=0;
		Vector#(TDiv#(`INPUT,2),Bit#(1)) ip1;
		Vector#(TDiv#(`INPUT,4),Bit#(1)) ip2;
		Vector#(TDiv#(`INPUT,8),Bit#(1)) ip3;
		Vector#(TDiv#(`INPUT,16),Bit#(1)) ip4;
		Vector#(TDiv#(`INPUT,32),Bit#(1)) ip5;
		Vector#(TDiv#(`INPUT,64),Bit#(1)) ip6;
		Vector#(TDiv#(`INPUT,128),Bit#(1)) ip7;
		Vector#(TDiv#(`INPUT,256),Bit#(1)) ip8;
		Vector#(TDiv#(`INPUT,512),Bit#(1)) ip9;
		Vector#(TDiv#(`INPUT,2),Bit#(1)) pp1;
		Vector#(TDiv#(`INPUT,4),Bit#(1)) pp2;
		Vector#(TDiv#(`INPUT,8),Bit#(1)) pp3;
		Vector#(TDiv#(`INPUT,16),Bit#(1)) pp4;
		Vector#(TDiv#(`INPUT,32),Bit#(1)) pp5;
		Vector#(TDiv#(`INPUT,64),Bit#(1)) pp6;
		Vector#(TDiv#(`INPUT,128),Bit#(1)) pp7;
		Vector#(TDiv#(`INPUT,256),Bit#(1)) pp8;
		Vector#(TDiv#(`INPUT,512),Bit#(1)) pp9;
		Bit#(1) ip10;
		Bit#(1) pp10;
		
		for(Integer i=0;i<`INPUT/2;i=i+1) begin
			ip1[i]=ip[i*2+1] | ip[i*2];
		end
		for(Integer i=0;i<`INPUT/4;i=i+1) begin
			ip2[i]=ip1[i*2+1] | ip1[i*2];
		end
		for(Integer i=0;i<`INPUT/8;i=i+1) begin
			ip3[i]=ip2[i*2+1] | ip2[i*2];
		end
		for(Integer i=0;i<`INPUT/16;i=i+1) begin
			ip4[i]=ip3[i*2+1] | ip3[i*2];
		end
		for(Integer i=0;i<`INPUT/32;i=i+1) begin
			ip5[i]=ip4[i*2+1] | ip4[i*2];
		end
		for(Integer i=0;i<`INPUT/64;i=i+1) begin
			ip6[i]=ip5[i*2+1] | ip5[i*2];
		end
		for(Integer i=0;i<`INPUT/128;i=i+1) begin
			ip7[i]=ip6[i*2+1] | ip6[i*2];
		end
		for(Integer i=0;i<`INPUT/256;i=i+1) begin
			ip8[i]=ip7[i*2+1] | ip7[i*2];
		end
		for(Integer i=0;i<`INPUT/512;i=i+1) begin
			ip9[i]=ip8[i*2+1] | ip8[i*2];
		end

		for(Integer i=0;i<`INPUT/2;i=i+1) begin
			pp1[i]=ip[i*2+1]==1?1:ip[i*2]==1?0:0;
		end
		for(Integer i=0;i<`INPUT/4;i=i+1) begin
			pp2[i]=ip1[i*2+1]==1?1:ip1[i*2]==1?0:0;
		end
		for(Integer i=0;i<`INPUT/8;i=i+1) begin
			pp3[i]=ip2[i*2+1]==1?1:ip2[i*2]==1?0:0;
		end
		for(Integer i=0;i<`INPUT/16;i=i+1) begin
			pp4[i]=ip3[i*2+1]==1?1:ip3[i*2]==1?0:0;
		end
		for(Integer i=0;i<`INPUT/32;i=i+1) begin
			pp5[i]=ip4[i*2+1]==1?1:ip4[i*2]==1?0:0;
		end
		for(Integer i=0;i<`INPUT/64;i=i+1) begin
			pp6[i]=ip5[i*2+1]==1?1:ip5[i*2]==1?0:0;
		end
		for(Integer i=0;i<`INPUT/128;i=i+1) begin
			pp7[i]=ip6[i*2+1]==1?1:ip6[i*2]==1?0:0;
		end
		for(Integer i=0;i<`INPUT/256;i=i+1) begin
			pp8[i]=ip7[i*2+1]==1?1:ip7[i*2]==1?0:0;
		end
		for(Integer i=0;i<`INPUT/512;i=i+1) begin
			pp9[i]=ip8[i*2+1]==1?1:ip8[i*2]==1?0:0;
		end

		pp10=ip9[1]==1?1:ip9[0]==1?0:0;
		ip10=ip9[1] | ip9[0];

		result[0] = pp10;
		let op9 = pp9[result[1:0]];
		result = {result[8:0],op9};
		let op8 = pp8[result[2:0]];
		result = {result[8:0],op8};
		let op7 = pp7[result[3:0]];
		result = {result[8:0],op7};
		let op6 = pp6[result[4:0]];
		result = {result[8:0],op6};
		let op5 = pp5[result[5:0]];
		result = {result[8:0],op5};
		let op4 = pp4[result[6:0]];
		result = {result[8:0],op4};
		let op3 = pp3[result[7:0]];
		result = {result[8:0],op3};
		let op2 = pp2[result[8:0]];
		result = {result[8:0],op2};
		let op1 = pp1[result[9:0]];
		result = {result[8:0],op1};
		return truncate(result);
	endfunction
	
method Bit#(irid) encode(Bit#(irpins) ip);
	return fn_encoder(ip);
endmethod
endmodule

(*synthesize*)
module mkSencoder(Ifc_encoder#(512));
	let ifc();
	mkencoder inst(ifc);
	return ifc();
endmodule

endpackage
