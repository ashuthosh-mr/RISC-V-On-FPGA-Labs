create_clock -period 60.000 -name tck -waveform {0.000 30.000} -add [get_nets pin_tck]

set_multicycle_path -setup 4 -from [get_clocks tck] -to [get_clocks clk_out1_clk_divider] -end
set_multicycle_path -hold 3 -from [get_clocks tck] -to [get_clocks clk_out1_clk_divider] -end
set_multicycle_path -setup 4 -from [get_clocks clk_out1_clk_divider] -to [get_clocks tck] -start
set_multicycle_path -hold 4 -from [get_clocks clk_out1_clk_divider] -to [get_clocks tck] -start

set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets pin_tck_IBUF]
set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets sys_clk_IBUF]

##Pmod Header JA for JTAG
set_property -dict { PACKAGE_PIN G13   IOSTANDARD LVCMOS33 } [get_ports { pin_tms             }]; #IO_0_15 Sch=ja[1]
set_property PULLUP   true [get_ports { pin_tms  }]; 
set_property -dict { PACKAGE_PIN B11   IOSTANDARD LVCMOS33 } [get_ports { pin_tdi             }]; #IO_L4P_T0_15 Sch=ja[2]
set_property -dict { PACKAGE_PIN D12   IOSTANDARD LVCMOS33 } [get_ports { pin_trst            }]; #IO_L6P_T0_15 Sch=ja[4]
set_property PULLDOWN true [get_ports { pin_trst }]; 
set_property -dict { PACKAGE_PIN D13   IOSTANDARD LVCMOS33 } [get_ports { pin_tdo             }]; #IO_L6N_T0_VREF_15 Sch=ja[7]
set_property -dict { PACKAGE_PIN B18   IOSTANDARD LVCMOS33 } [get_ports { pin_tck             }]; #IO_L10P_T1_AD11P_15 Sch=ja[8]

