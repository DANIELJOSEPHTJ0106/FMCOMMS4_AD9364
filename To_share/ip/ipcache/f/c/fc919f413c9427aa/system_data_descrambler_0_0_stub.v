// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Wed Jan  7 18:04:02 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_data_descrambler_0_0_stub.v
// Design      : system_data_descrambler_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "data_descrambler,Vivado 2023.1" *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix(clk, rst_n, datain, validin, data_start, data_end, 
  dataout1, validout)
/* synthesis syn_black_box black_box_pad_pin="datain,validin,data_start,data_end,dataout1,validout" */
/* synthesis syn_force_seq_prim="clk" */
/* synthesis syn_force_seq_prim="rst_n" */;
  input clk /* synthesis syn_isclock = 1 */;
  input rst_n /* synthesis syn_isclock = 1 */;
  input datain;
  input validin;
  input data_start;
  input data_end;
  output dataout1;
  output validout;
endmodule
