
# Devices

This repo contains the following devices:

1. BootROM
2. UART - simple (Rx - Tx mechanism)
3. CLINT (Core level interruptor)
4. PWM (Pulse Width Modulator)
5. BRAM 
6. SDRAM
7. 1149.1 JTAG TAP
8. RISCV 0.13.1 Debug Module

## QuickStart

    $ git clone git@gitlab.com:incoresemi/devices.git --recursive
    
Each device has its own Makefile to generate verilog and synthesize using VIVADO

