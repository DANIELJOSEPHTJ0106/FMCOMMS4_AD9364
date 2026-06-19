// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Wed Jan  7 16:05:45 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_2_Data_repack_0_1_stub.v
// Design      : design_2_Data_repack_0_1
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "Data_repack,Vivado 2023.1" *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix(clock, reset_n, dat_in, dat_in_val, in_start, 
  dat_out_dat, dat_out_val)
/* synthesis syn_black_box black_box_pad_pin="reset_n,dat_in,dat_in_val,in_start,dat_out_dat[7:0],dat_out_val" */
/* synthesis syn_force_seq_prim="clock" */;
  input clock /* synthesis syn_isclock = 1 */;
  input reset_n;
  input dat_in;
  input dat_in_val;
  input in_start;
  output [7:0]dat_out_dat;
  output dat_out_val;
endmodule
