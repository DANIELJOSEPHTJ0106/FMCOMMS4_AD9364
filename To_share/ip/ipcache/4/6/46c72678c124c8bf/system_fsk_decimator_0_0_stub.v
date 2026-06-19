// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Tue Feb  3 16:25:28 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_fsk_decimator_0_0_stub.v
// Design      : system_fsk_decimator_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "fsk_decimator,Vivado 2023.1" *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix(ap_clk, ap_rst_n, rx_in_TVALID, rx_in_TREADY, 
  rx_in_TDATA, rx_in_TLAST, rx_in_TKEEP, rx_in_TSTRB, dec_out_TVALID, dec_out_TREADY, 
  dec_out_TDATA, dec_out_TLAST, dec_out_TKEEP, dec_out_TSTRB)
/* synthesis syn_black_box black_box_pad_pin="ap_rst_n,rx_in_TVALID,rx_in_TREADY,rx_in_TDATA[31:0],rx_in_TLAST[0:0],rx_in_TKEEP[3:0],rx_in_TSTRB[3:0],dec_out_TVALID,dec_out_TREADY,dec_out_TDATA[31:0],dec_out_TLAST[0:0],dec_out_TKEEP[3:0],dec_out_TSTRB[3:0]" */
/* synthesis syn_force_seq_prim="ap_clk" */;
  input ap_clk /* synthesis syn_isclock = 1 */;
  input ap_rst_n;
  input rx_in_TVALID;
  output rx_in_TREADY;
  input [31:0]rx_in_TDATA;
  input [0:0]rx_in_TLAST;
  input [3:0]rx_in_TKEEP;
  input [3:0]rx_in_TSTRB;
  output dec_out_TVALID;
  input dec_out_TREADY;
  output [31:0]dec_out_TDATA;
  output [0:0]dec_out_TLAST;
  output [3:0]dec_out_TKEEP;
  output [3:0]dec_out_TSTRB;
endmodule
