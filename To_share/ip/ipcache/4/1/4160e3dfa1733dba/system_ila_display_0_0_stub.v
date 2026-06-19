// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 12:25:20 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_ila_display_0_0/system_ila_display_0_0_stub.v
// Design      : system_ila_display_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "ila_display,Vivado 2023.1" *)
module system_ila_display_0_0(clock_61, clock_100, reset_n, 
  pattern_gen_dat_out, pattern_gen_val_out, bsync_dat, bsync_val, ila_pattern_gen_dat, 
  ila_pattern_gen_val, ila_bsync_dat, ila_bsync_val)
/* synthesis syn_black_box black_box_pad_pin="reset_n,pattern_gen_dat_out[7:0],pattern_gen_val_out,bsync_dat[7:0],bsync_val,ila_pattern_gen_dat[7:0],ila_pattern_gen_val,ila_bsync_dat[7:0],ila_bsync_val" */
/* synthesis syn_force_seq_prim="clock_61" */
/* synthesis syn_force_seq_prim="clock_100" */;
  input clock_61 /* synthesis syn_isclock = 1 */;
  input clock_100 /* synthesis syn_isclock = 1 */;
  input reset_n;
  input [7:0]pattern_gen_dat_out;
  input pattern_gen_val_out;
  input [7:0]bsync_dat;
  input bsync_val;
  output [7:0]ila_pattern_gen_dat;
  output ila_pattern_gen_val;
  output [7:0]ila_bsync_dat;
  output ila_bsync_val;
endmodule
