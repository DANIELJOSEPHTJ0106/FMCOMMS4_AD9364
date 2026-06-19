# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: D:\FMCOMMS4_AD9364_Fifo_Tlast_FFT\Ws\FMCOMMS4_AD9364_system\_ide\scripts\debugger_fmcomms4_ad9364-emulation.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source D:\FMCOMMS4_AD9364_Fifo_Tlast_FFT\Ws\FMCOMMS4_AD9364_system\_ide\scripts\debugger_fmcomms4_ad9364-emulation.tcl
# 
connect -url tcp:127.0.0.1:3121
targets -set -nocase -filter {name =~"APU*"}
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "Digilent Zed 210248BE0D85" && level==0 && jtag_device_ctx=="jsn-Zed-210248BE0D85-23727093-0"}
fpga -file D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/FMCOMMS4_AD9364/_ide/bitstream/system_top.bit
targets -set -nocase -filter {name =~"APU*"}
loadhw -hw D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/system_top/export/system_top/hw/system_top.xsa -mem-ranges [list {0x40000000 0xbfffffff}] -regs
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*"}
source D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/FMCOMMS4_AD9364/_ide/psinit/ps7_init.tcl
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "*A9*#0"}
dow D:/FMCOMMS4_AD9364_Fifo_Tlast_FFT/Ws/FMCOMMS4_AD9364/Debug/FMCOMMS4_AD9364.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "*A9*#0"}
con
