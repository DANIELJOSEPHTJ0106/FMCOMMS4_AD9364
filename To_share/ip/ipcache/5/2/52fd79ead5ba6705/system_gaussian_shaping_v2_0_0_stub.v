// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 12:24:47 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_gaussian_shaping_v2_0_0/system_gaussian_shaping_v2_0_0_stub.v
// Design      : system_gaussian_shaping_v2_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "gaussian_shaping_v2,Vivado 2023.1" *)
module system_gaussian_shaping_v2_0_0(ap_clk, ap_rst_n, in_stream_TVALID, 
  in_stream_TREADY, in_stream_TDATA, in_stream_TLAST, in_stream_TKEEP, in_stream_TSTRB, 
  in_stream_TUSER, fcw_out_TVALID, fcw_out_TREADY, fcw_out_TDATA, fcw_out_TLAST, 
  fcw_out_TKEEP, fcw_out_TSTRB)
/* synthesis syn_black_box black_box_pad_pin="ap_rst_n,in_stream_TVALID,in_stream_TREADY,in_stream_TDATA[7:0],in_stream_TLAST[0:0],in_stream_TKEEP[0:0],in_stream_TSTRB[0:0],in_stream_TUSER[12:0],fcw_out_TVALID,fcw_out_TREADY,fcw_out_TDATA[31:0],fcw_out_TLAST[0:0],fcw_out_TKEEP[3:0],fcw_out_TSTRB[3:0]" */
/* synthesis syn_force_seq_prim="ap_clk" */;
  input ap_clk /* synthesis syn_isclock = 1 */;
  input ap_rst_n;
  input in_stream_TVALID;
  output in_stream_TREADY;
  input [7:0]in_stream_TDATA;
  input [0:0]in_stream_TLAST;
  input [0:0]in_stream_TKEEP;
  input [0:0]in_stream_TSTRB;
  input [12:0]in_stream_TUSER;
  output fcw_out_TVALID;
  input fcw_out_TREADY;
  output [31:0]fcw_out_TDATA;
  output [0:0]fcw_out_TLAST;
  output [3:0]fcw_out_TKEEP;
  output [3:0]fcw_out_TSTRB;
endmodule
