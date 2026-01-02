#create_clock -name in_clk100 -period 10 [get_ports in_clk100]
#set_property PACKAGE_PIN AL15 [get_ports in_clk100]
#set_property IOSTANDARD LVCMOS18 [get_ports in_clk100]
set_property PACKAGE_PIN BJ43 [get_ports sys_clk_p]
set_property PACKAGE_PIN BJ44 [get_ports sys_clk_n]
set_property IOSTANDARD  DIFF_SSTL12 [get_ports sys_clk_p]
set_property IOSTANDARD  DIFF_SSTL12 [get_ports sys_clk_n]
create_clock -name sys_clk -period 10.000 [get_ports sys_clk_p]
#create_clock -name sys_clk -period 10 [get_ports sys_clk_p]
#set_property LOC [get_package_pins -of_objects [get_bels [get_sites -filter {NAME =~ *COMMON*} -of_objects [get_iobanks -of_objects [get_sites GTYE4_CHANNEL_X1Y7]]]/REFCLK0P]] [get_ports sys_clk_p]
#set_property LOC [get_package_pins -of_objects [get_bels [get_sites -filter {NAME =~ *COMMON*} -of_objects [get_iobanks -of_objects [get_sites GTYE4_CHANNEL_X1Y7]]]/REFCLK0N]] [get_ports sys_clk_n]
set_false_path -from [get_ports sys_rst_n]
set_property PULLUP true [get_ports sys_rst_n]
set_property IOSTANDARD LVCMOS18 [get_ports sys_rst_n]
set_property PACKAGE_PIN BH26 [get_ports sys_rst_n]
set_property CONFIG_VOLTAGE 1.8 [current_design]
#set_property PACKAGE_PIN D32 [get_ports bbq_out]
#set_property IOSTANDARD  LVCMOS18 [get_ports bbq_out]

## QSFP28 Interfaces
#set_property -dict {LOC L53 } [get_ports {gt0_rxp_in[0]}] ;# MGTYRXP0_134 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC L54 } [get_ports {gt0_rxn_in[0]}] ;# MGTYRXN0_134 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC L48 } [get_ports {gt0_txp_out[0]}] ;# MGTYTXP0_134 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC L49 } [get_ports {gt0_txn_out[0]}] ;# MGTYTXN0_134 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC K51 } [get_ports {gt0_rxp_in[1]}] ;# MGTYRXP1_134 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC K52 } [get_ports {gt0_rxn_in[1]}] ;# MGTYRXN1_134 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC L44 } [get_ports {gt0_txp_out[1]}] ;# MGTYTXP1_134 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC L45 } [get_ports {gt0_txn_out[1]}] ;# MGTYTXN1_134 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC J53 } [get_ports {gt0_rxp_in[2]}] ;# MGTYRXP2_134 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC J54 } [get_ports {gt0_rxn_in[2]}] ;# MGTYRXN2_134 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC K46 } [get_ports {gt0_txp_out[2]}] ;# MGTYTXP2_134 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC K47 } [get_ports {gt0_txn_out[2]}] ;# MGTYTXN2_134 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC H51 } [get_ports {gt0_rxp_in[3]}] ;# MGTYRXP3_134 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC H52 } [get_ports {gt0_rxn_in[3]}] ;# MGTYRXN3_134 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC J48 } [get_ports {gt0_txp_out[3]}] ;# MGTYTXP3_134 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y10
#set_property -dict {LOC J49 } [get_ports {gt0_txn_out[3]}] ;# MGTYTXN3_134 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y10
##set_property -dict {LOC T42 } [get_ports qsfp0_mgt_refclk_0_p] ;# MGTREFCLK0P_134 from SI570
##set_property -dict {LOC T43 } [get_ports qsfp0_mgt_refclk_0_n] ;# MGTREFCLK0N_134 from SI570
#set_property -dict {LOC R40 } [get_ports gt0_ref_clk_p] ;# MGTREFCLK1P_134 from SI546
#set_property -dict {LOC R41 } [get_ports gt0_ref_clk_n] ;# MGTREFCLK1N_134 from SI546

## 161.1328125 MHz MGT reference clock (from SI546, fs = 1)
#create_clock -period 6.206 -name gt0_ref_clk [get_ports gt0_ref_clk_p]

#set_property -dict {LOC G53 } [get_ports {gt1_rxp_in[0]}]  ;# MGTYRXP0_135 GTYE4_CHANNEL_X0Y44 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC G54 } [get_ports {gt1_rxn_in[0]}]  ;# MGTYRXN0_135 GTYE4_CHANNEL_X0Y44 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC G48 } [get_ports {gt1_txp_out[0]}] ;# MGTYTXP0_135 GTYE4_CHANNEL_X0Y44 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC G49 } [get_ports {gt1_txn_out[0]}] ;# MGTYTXN0_135 GTYE4_CHANNEL_X0Y44 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC F51 } [get_ports {gt1_rxp_in[1]}]  ;# MGTYRXP1_135 GTYE4_CHANNEL_X0Y45 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC F52 } [get_ports {gt1_rxn_in[1]}]  ;# MGTYRXN1_135 GTYE4_CHANNEL_X0Y45 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC E48 } [get_ports {gt1_txp_out[1]}] ;# MGTYTXP1_135 GTYE4_CHANNEL_X0Y45 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC E49 } [get_ports {gt1_txn_out[1]}] ;# MGTYTXN1_135 GTYE4_CHANNEL_X0Y45 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC E53 } [get_ports {gt1_rxp_in[2]}]  ;# MGTYRXP2_135 GTYE4_CHANNEL_X0Y46 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC E54 } [get_ports {gt1_rxn_in[2]}]  ;# MGTYRXN2_135 GTYE4_CHANNEL_X0Y46 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC C48 } [get_ports {gt1_txp_out[2]}] ;# MGTYTXP2_135 GTYE4_CHANNEL_X0Y46 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC C49 } [get_ports {gt1_txn_out[2]}] ;# MGTYTXN2_135 GTYE4_CHANNEL_X0Y46 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC D51 } [get_ports {gt1_rxp_in[3]}]  ;# MGTYRXP3_135 GTYE4_CHANNEL_X0Y47 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC D52 } [get_ports {gt1_rxn_in[3]}]  ;# MGTYRXN3_135 GTYE4_CHANNEL_X0Y47 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC A49 } [get_ports {gt1_txp_out[3]}] ;# MGTYTXP3_135 GTYE4_CHANNEL_X0Y47 / GTYE4_COMMON_X0Y11
#set_property -dict {LOC A50 } [get_ports {gt1_txn_out[3]}] ;# MGTYTXN3_135 GTYE4_CHANNEL_X0Y47 / GTYE4_COMMON_X0Y11
##set_property -dict {LOC P42 } [get_ports qsfp1_mgt_refclk_0_p] ;# MGTREFCLK0P_135 from SI570
##set_property -dict {LOC P43 } [get_ports qsfp1_mgt_refclk_0_n] ;# MGTREFCLK0N_135 from SI570
#set_property -dict {LOC M42 } [get_ports gt1_ref_clk_p] ;# MGTREFCLK1P_135 from SI546
#set_property -dict {LOC M43 } [get_ports gt1_ref_clk_n] ;# MGTREFCLK1N_135 from SI546

## 156.25 MHz MGT reference clock (from SI570)
##create_clock -period 6.400 -name qsfp1_mgt_refclk_0 [get_ports qsfp1_mgt_refclk_0_p]

## 156.25 MHz MGT reference clock (from SI546, fs = 0)
##create_clock -period 6.400 -name qsfp1_mgt_refclk_1 [get_ports qsfp1_mgt_refclk_1_p]

## 161.1328125 MHz MGT reference clock (from SI546, fs = 1)
#create_clock -period 6.206 -name gt1_ref_clk [get_ports gt1_ref_clk_p]