# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct D:\FMCOMMS4_AD9364\ZedFSK_ilaFmcomms4.ide\Ws\system_top\platform.tcl
# 
# OR launch xsct and run below command.
# source D:\FMCOMMS4_AD9364\ZedFSK_ilaFmcomms4.ide\Ws\system_top\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {system_top}\
-hw {D:\FMCOMMS4_AD9364\ZedFSK_ilaFmcomms4.ide\system_top\export\system_top\hw\system_top.xsa}\
-out {D:/FMCOMMS4_AD9364/ZedFSK_ilaFmcomms4.ide/Ws}

platform write
domain create -name {standalone_ps7_cortexa9_0} -display-name {standalone_ps7_cortexa9_0} -os {standalone} -proc {ps7_cortexa9_0} -runtime {cpp} -arch {32-bit} -support-app {hello_world}
platform generate -domains 
platform active {system_top}
domain active {zynq_fsbl}
domain active {standalone_ps7_cortexa9_0}
platform generate -quick
platform generate
platform config -updatehw {D:/FMCOMMS4_AD9364/ZedFSK_ilaFmcomms4.ide/Ws/system_top.xsa}
platform generate -domains 
platform active {system_top}
platform config -updatehw {D:/FMCOMMS4_AD9364_SG/FMCOMMS4_AD9364/ZedFSK_ilaFmcomms4.ide/Ws/system_top.xsa}
platform generate
platform clean
platform generate
domain active {zynq_fsbl}
domain active {standalone_ps7_cortexa9_0}
bsp reload
bsp reload
bsp setlib -name lwip213 -ver 1.0
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains standalone_ps7_cortexa9_0 
bsp write
domain active {zynq_fsbl}
bsp setlib -name lwip213 -ver 1.0
bsp write
bsp reload
catch {bsp regenerate}
bsp config lwip_dhcp "false"
bsp config lwip_dhcp "false"
bsp config lwip_dhcp "true"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains zynq_fsbl 
bsp removelib -name lwip213
bsp write
bsp reload
catch {bsp regenerate}
domain active {standalone_ps7_cortexa9_0}
bsp reload
domain active {zynq_fsbl}
bsp setlib -name lwip213 -ver 1.0
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains zynq_fsbl 
platform clean
platform generate
bsp reload
domain active {standalone_ps7_cortexa9_0}
bsp reload
domain active {zynq_fsbl}
bsp removelib -name lwip213
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains zynq_fsbl 
domain active {standalone_ps7_cortexa9_0}
bsp removelib -name lwip213
bsp write
bsp reload
catch {bsp regenerate}
bsp setlib -name lwip213 -ver 1.0
bsp config dhcp_does_arp_check "true"
bsp config lwip_dhcp "true"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains standalone_ps7_cortexa9_0 
bsp config api_mode "SOCKET_API"
bsp write
bsp reload
catch {bsp regenerate}
bsp write
platform clean
bsp config api_mode "RAW_API"
bsp write
bsp reload
catch {bsp regenerate}
bsp write
platform generate
bsp removelib -name lwip213
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains standalone_ps7_cortexa9_0 
bsp setlib -name lwip213 -ver 1.0
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains standalone_ps7_cortexa9_0 
bsp config dhcp_does_arp_check "true"
bsp config lwip_dhcp "true"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains standalone_ps7_cortexa9_0 
bsp write
bsp write
platform generate -domains 
platform active {system_top}
bsp reload
bsp reload
platform generate -domains 
bsp config api_mode "RAW_API"
bsp config pbuf_pool_size "512"
bsp config tcp_snd_buf "65535"
bsp write
bsp reload
catch {bsp regenerate}
bsp reload
platform clean
bsp config tcp_snd_buf "8192"
bsp config pbuf_pool_size "256"
bsp write
bsp reload
catch {bsp regenerate}
platform clean
platform generate
platform config -updatehw {D:/FMCOMMS4_AD9364_SG/FMCOMMS4_AD9364/ZedFSK_ilaFmcomms4.ide/Ws/system_top.xsa}
platform generate -domains 
bsp reload
bsp config pbuf_pool_size "512"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains standalone_ps7_cortexa9_0 
bsp config api_mode "SOCKET_API"
bsp write
bsp reload
catch {bsp regenerate}
bsp write
platform clean
bsp config api_mode "RAW_API"
bsp write
bsp reload
catch {bsp regenerate}
platform clean
platform generate
bsp config api_mode "SOCKET_API"
bsp write
bsp reload
catch {bsp regenerate}
bsp config api_mode "RAW_API"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains standalone_ps7_cortexa9_0 
platform clean
platform generate
platform clean
platform generate
bsp write
platform generate -domains 
platform active {system_top}
platform generate
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform active {system_top}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform active {system_top}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top_Gpio.xsa}
platform generate -domains 
platform clean
platform generate
platform clean
platform generate
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top_Gpio.xsa}
platform generate -domains 
platform active {system_top}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform active {system_top}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform clean
platform generate
platform clean
platform generate
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform active {system_top}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
platform active {system_top}
platform config -updatehw {D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top.xsa}
platform generate -domains 
