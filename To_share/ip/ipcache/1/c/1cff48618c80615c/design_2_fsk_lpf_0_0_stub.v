// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Wed Jan  7 13:01:35 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_2_fsk_lpf_0_0_stub.v
// Design      : design_2_fsk_lpf_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "fsk_lpf,Vivado 2023.1" *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix(ap_clk, ap_rst_n, in_r_TVALID, in_r_TREADY, 
  in_r_TDATA, in_r_TLAST, in_r_TKEEP, in_r_TSTRB, out_r_TVALID, out_r_TREADY, out_r_TDATA, 
  out_r_TLAST, out_r_TKEEP, out_r_TSTRB)
/* synthesis syn_black_box black_box_pad_pin="ap_rst_n,in_r_TVALID,in_r_TREADY,in_r_TDATA[15:0],in_r_TLAST[0:0],in_r_TKEEP[1:0],in_r_TSTRB[1:0],out_r_TVALID,out_r_TREADY,out_r_TDATA[15:0],out_r_TLAST[0:0],out_r_TKEEP[1:0],out_r_TSTRB[1:0]" */
/* synthesis syn_force_seq_prim="ap_clk" */;
  input ap_clk /* synthesis syn_isclock = 1 */;
  input ap_rst_n;
  input in_r_TVALID;
  output in_r_TREADY;
  input [15:0]in_r_TDATA;
  input [0:0]in_r_TLAST;
  input [1:0]in_r_TKEEP;
  input [1:0]in_r_TSTRB;
  output out_r_TVALID;
  input out_r_TREADY;
  output [15:0]out_r_TDATA;
  output [0:0]out_r_TLAST;
  output [1:0]out_r_TKEEP;
  output [1:0]out_r_TSTRB;
endmodule
