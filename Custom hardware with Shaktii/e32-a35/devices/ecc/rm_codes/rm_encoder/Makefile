BSC_PATHS = -p .:%/Prelude:%/Libraries

.PHONY: default
default: compile link

# ----------------------------------------------------------------
# Bluesim compile/link/simulate

BSIM_DIRS =

.PHONY: compile
compile:
	@echo Compiling for Bluesim ...
	bsc -sim -g mkTb  rm_encoder.bsv 
	@echo Compiling for Bluesim finished

.PHONY: link
link:
	@echo Linking for Bluesim ...
	bsc -sim -e mkTb -o ./out
	@echo Linking for Bluesim finished

.PHONY: simulate
simulate:
	@echo Bluesim simulation ...
	./out -V
	@echo Bluesim simulation finished

# ----------------------------------------------------------------
# Verilog compile/link/sim

.PHONY: rtl
rtl:
	@echo Compiling for Verilog ...
	bsc -verilog -g mkTb  rm_encoder.bsv
	@echo Compiling for Verilog finished

.PHONY: v_link
v_link:
	@echo Linking for Verilog sim ...
	bsc -verilog -e mkTb -o out_v -vsim iverilog  mkTb.v
	@echo Linking for Verilog sim finished

.PHONY: v_simulate
v_simulate:
	@echo Verilog simulation...
	./out_v
	@echo Verilog simulation finished

# ----------------------------------------------------------------

.PHONY: clean
clean:
	rm -f  *.bo *.ba mk* *.cxx *.h *.o

.PHONY: full_clean
full_clean:
	rm -f  *.bo *.ba mk* *.cxx *.h *.o
	rm -f  out  out.so  out_v *.v
