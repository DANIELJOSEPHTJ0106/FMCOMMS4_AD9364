// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Tue Feb  3 16:25:29 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_fsk_phase_corrector_0_0_stub.v
// Design      : system_fsk_phase_corrector_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "fsk_phase_corrector,Vivado 2023.1" *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix(ap_clk, ap_rst_n, in_stream_TVALID, 
  in_stream_TREADY, in_stream_TDATA, in_stream_TLAST, in_stream_TKEEP, in_stream_TSTRB, 
  out_stream_TVALID, out_stream_TREADY, out_stream_TDATA, out_stream_TLAST, 
  out_stream_TKEEP, out_stream_TSTRB)
/* synthesis syn_black_box black_box_pad_pin="ap_rst_n,in_stream_TVALID,in_stream_TREADY,in_stream_TDATA[31:0],in_stream_TLAST[0:0],in_stream_TKEEP[3:0],in_stream_TSTRB[3:0],out_stream_TVALID,out_stream_TREADY,out_stream_TDATA[31:0],out_stream_TLAST[0:0],out_stream_TKEEP[3:0],out_stream_TSTRB[3:0]" */
/* synthesis syn_force_seq_prim="ap_clk" */;
  input ap_clk /* synthesis syn_isclock = 1 */;
  input ap_rst_n;
  input in_stream_TVALID;
  output in_stream_TREADY;
  input [31:0]in_stream_TDATA;
  input [0:0]in_stream_TLAST;
  input [3:0]in_stream_TKEEP;
  input [3:0]in_stream_TSTRB;
  output out_stream_TVALID;
  input out_stream_TREADY;
  output [31:0]out_stream_TDATA;
  output [0:0]out_stream_TLAST;
  output [3:0]out_stream_TKEEP;
  output [3:0]out_stream_TSTRB;
endmodule
