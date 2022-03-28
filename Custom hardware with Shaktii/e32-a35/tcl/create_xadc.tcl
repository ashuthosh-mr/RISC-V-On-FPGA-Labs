#Generate an XADC instance

puts "\nDEBUG: Creating XADC IP instance \n"

if { [get_ips -quiet xadc_wiz_0] eq "" } {
	create_ip -name xadc_wiz -vendor xilinx.com -library ip -version 3.3 -module_name xadc_wiz_0
} else {
    reset_run xadc_wiz_0_synth_1
}

set_property -dict [list \
CONFIG.INTERFACE_SELECTION {Enable_AXI} \
CONFIG.XADC_STARUP_SELECTION {channel_sequencer} \
CONFIG.DCLK_FREQUENCY {50} \
CONFIG.ADC_CONVERSION_RATE {800} \
CONFIG.OT_ALARM {false} \
CONFIG.USER_TEMP_ALARM {false} \
CONFIG.VCCINT_ALARM {false} \
CONFIG.ENABLE_TEMP_BUS {false} \
CONFIG.VCCAUX_ALARM {false} \
CONFIG.CHANNEL_ENABLE_VP_VN {true} \
CONFIG.CHANNEL_ENABLE_VAUXP0_VAUXN0 {true} \
CONFIG.CHANNEL_ENABLE_VAUXP4_VAUXN4 {true} \
CONFIG.CHANNEL_ENABLE_VAUXP5_VAUXN5 {true} \
CONFIG.CHANNEL_ENABLE_VAUXP6_VAUXN6 {true} \
CONFIG.CHANNEL_ENABLE_VAUXP7_VAUXN7 {true} \
CONFIG.CHANNEL_ENABLE_VAUXP12_VAUXN12 {true} \
CONFIG.CHANNEL_ENABLE_VAUXP13_VAUXN13 {true} \
CONFIG.CHANNEL_ENABLE_VAUXP14_VAUXN14 {true} \
CONFIG.CHANNEL_ENABLE_VAUXP15_VAUXN15 {true} \
CONFIG.BIPOLAR_VAUXP12_VAUXN12 {true} \
CONFIG.BIPOLAR_VAUXP13_VAUXN13 {true} \
CONFIG.BIPOLAR_VAUXP14_VAUXN14 {true} \
CONFIG.SEQUENCER_MODE {Continuous} \
CONFIG.ENABLE_RESET {false} \
CONFIG.EXTERNAL_MUX_CHANNEL {VP_VN} \
CONFIG.SINGLE_CHANNEL_SELECTION {TEMPERATURE} \
CONFIG.CHANNEL_ENABLE_TEMPERATURE {true} ] [get_ips xadc_wiz_0]
generate_target {instantiation_template} [get_ips xadc_wiz_0]
create_ip_run [get_ips xadc_wiz_0]

