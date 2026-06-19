// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 12:24:46 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_bit_sync_0_0/system_bit_sync_0_0_stub.v
// Design      : system_bit_sync_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "bit_sync,Vivado 2023.1" *)
module system_bit_sync_0_0(ap_clk, ap_rst_n, in_stream_TVALID, 
  in_stream_TREADY, in_stream_TDATA, in_stream_TLAST, in_stream_TKEEP, in_stream_TSTRB, 
  out_stream_TVALID, out_stream_TREADY, out_stream_TDATA, out_stream_TLAST, 
  out_stream_TKEEP, out_stream_TSTRB)
/* synthesis syn_black_box black_box_pad_pin="ap_rst_n,in_stream_TVALID,in_stream_TREADY,in_stream_TDATA[15:0],in_stream_TLAST[0:0],in_stream_TKEEP[1:0],in_stream_TSTRB[1:0],out_stream_TVALID,out_stream_TREADY,out_stream_TDATA[7:0],out_stream_TLAST[0:0],out_stream_TKEEP[0:0],out_stream_TSTRB[0:0]" */
/* synthesis syn_force_seq_prim="ap_clk" */;
  input ap_clk /* synthesis syn_isclock = 1 */;
  input ap_rst_n;
  input in_stream_TVALID;
  output in_stream_TREADY;
  input [15:0]in_stream_TDATA;
  input [0:0]in_stream_TLAST;
  input [1:0]in_stream_TKEEP;
  input [1:0]in_stream_TSTRB;
  output out_stream_TVALID;
  input out_stream_TREADY;
  output [7:0]out_stream_TDATA;
  output [0:0]out_stream_TLAST;
  output [0:0]out_stream_TKEEP;
  output [0:0]out_stream_TSTRB;
endmodule
