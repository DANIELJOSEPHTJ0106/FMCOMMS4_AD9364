// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 12:24:45 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_fsk_ddc_0_0/system_fsk_ddc_0_0_stub.v
// Design      : system_fsk_ddc_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "fsk_ddc,Vivado 2023.1" *)
module system_fsk_ddc_0_0(ap_clk, ap_rst_n, rx_in_TVALID, rx_in_TREADY, 
  rx_in_TDATA, rx_in_TLAST, rx_in_TKEEP, rx_in_TSTRB, dds_lo_TVALID, dds_lo_TREADY, 
  dds_lo_TDATA, dds_lo_TLAST, dds_lo_TKEEP, dds_lo_TSTRB, baseband_out_TVALID, 
  baseband_out_TREADY, baseband_out_TDATA, baseband_out_TLAST, baseband_out_TKEEP, 
  baseband_out_TSTRB)
/* synthesis syn_black_box black_box_pad_pin="ap_rst_n,rx_in_TVALID,rx_in_TREADY,rx_in_TDATA[31:0],rx_in_TLAST[0:0],rx_in_TKEEP[3:0],rx_in_TSTRB[3:0],dds_lo_TVALID,dds_lo_TREADY,dds_lo_TDATA[31:0],dds_lo_TLAST[0:0],dds_lo_TKEEP[3:0],dds_lo_TSTRB[3:0],baseband_out_TVALID,baseband_out_TREADY,baseband_out_TDATA[31:0],baseband_out_TLAST[0:0],baseband_out_TKEEP[3:0],baseband_out_TSTRB[3:0]" */
/* synthesis syn_force_seq_prim="ap_clk" */;
  input ap_clk /* synthesis syn_isclock = 1 */;
  input ap_rst_n;
  input rx_in_TVALID;
  output rx_in_TREADY;
  input [31:0]rx_in_TDATA;
  input [0:0]rx_in_TLAST;
  input [3:0]rx_in_TKEEP;
  input [3:0]rx_in_TSTRB;
  input dds_lo_TVALID;
  output dds_lo_TREADY;
  input [31:0]dds_lo_TDATA;
  input [0:0]dds_lo_TLAST;
  input [3:0]dds_lo_TKEEP;
  input [3:0]dds_lo_TSTRB;
  output baseband_out_TVALID;
  input baseband_out_TREADY;
  output [31:0]baseband_out_TDATA;
  output [0:0]baseband_out_TLAST;
  output [3:0]baseband_out_TKEEP;
  output [3:0]baseband_out_TSTRB;
endmodule
