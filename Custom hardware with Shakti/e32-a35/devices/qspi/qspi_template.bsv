package qspi_template;
import qspi::*;
`include "qspi.defines"
import device_common::*;

(*synthesize*)
module mkdummy#(Clock slow_clock, Reset slow_reset)(Empty);
	let core_clock<-exposeCurrentClock;
	let core_reset<-exposeCurrentReset;
	Ifc_qspi_axi4lite#(32,64,0) qspi <- mkqspi_axi4lite(slow_clock,slow_reset);

endmodule

endpackage
