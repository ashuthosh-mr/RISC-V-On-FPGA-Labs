ISA=RV32IMAC
MUL=fpga
SYNTH=FPGA
VERBOSITY=0
USERTRAPS=enable
USER=enable
COREFABRIC=AXI4Lite
MULSTAGES=2
DIVSTAGES=32
PADDR=32
RESETPC=4096
ARITHTRAP=disable

CAUSESIZE=6
DEBUG=enable
OPENOCD=disable
DTVEC_BASE=16

# Verilator options
COVERAGE=none
TRACE=disable
THREADS=1
VERILATESIM=fast

# DebugOptions
RTLDUMP=disable
ASSERTIONS=disable

# Trigger Setup
TRIGGERS=0

# performance counters
COUNTERS=0

# pmp
PMP=disable
PMPSIZE=2

# For Vivado Synthesis
FPGA=xc7a35ticsg324-1l
SYNTHTOP=fpga_top
BSCAN2E=enable
JOBS=6
