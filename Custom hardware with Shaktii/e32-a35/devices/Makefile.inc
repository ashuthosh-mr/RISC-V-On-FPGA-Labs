############ DO NOT CHANGE THESE #################
SUPPORTED:=./uart:./pwm:./bootrom:./clint:./common_bsv:./fabrics/axi4:./fabrics/axi4lite:./bram:./gpio:./sdram/32-bit:./jtagdtm:./onfi:./plic:./i2c:./qspi:./riscvDebug013:./spi:./watchdog
TOP_MODULE?=mkdummy

############# User changes to be done below #################
TOP_DIR?= riscvDebug013
TOP_FILE?= riscvDebug013_template.bsv

