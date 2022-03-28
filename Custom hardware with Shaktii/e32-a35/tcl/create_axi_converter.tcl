#set curdir [ file dirname [ file normalize [ info script ] ] ]
#source $curdir/env.tcl

#open_project $ip_project_dir/$ip_project.xpr
#set_property "simulator_language" "Mixed" [current_project]
#set_property "target_language" "Verilog" [current_project]

puts "\nDEBUG: Creating AXI Clock converter IP\n"

if { [get_ips -quiet clk_converter] eq "" } {
    create_ip -name axi_clock_converter -vendor xilinx.com -library ip -module_name clk_converter
} else {
    reset_run clk_converter_synth_1
}

set_property -dict [list \
  CONFIG.DATA_WIDTH {64} \
  CONFIG.ID_WIDTH {4} \
  CONFIG.ADDR_WIDTH {28} ] [get_ips clk_converter]
generate_target {instantiation_template} [get_ips clk_converter]
create_ip_run [get_ips clk_converter]
#launch_run clk_converter_synth_1
#wait_on_run clk_converter_synth_1
#exit

