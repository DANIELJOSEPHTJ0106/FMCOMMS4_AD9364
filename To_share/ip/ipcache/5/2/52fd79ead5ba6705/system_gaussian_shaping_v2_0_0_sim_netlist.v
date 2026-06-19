// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 12:24:47 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_gaussian_shaping_v2_0_0/system_gaussian_shaping_v2_0_0_sim_netlist.v
// Design      : system_gaussian_shaping_v2_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "system_gaussian_shaping_v2_0_0,gaussian_shaping_v2,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "HLS" *) 
(* X_CORE_INFO = "gaussian_shaping_v2,Vivado 2023.1" *) (* hls_module = "yes" *) 
(* NotValidForBitStream *)
module system_gaussian_shaping_v2_0_0
   (ap_clk,
    ap_rst_n,
    in_stream_TVALID,
    in_stream_TREADY,
    in_stream_TDATA,
    in_stream_TLAST,
    in_stream_TKEEP,
    in_stream_TSTRB,
    in_stream_TUSER,
    fcw_out_TVALID,
    fcw_out_TREADY,
    fcw_out_TDATA,
    fcw_out_TLAST,
    fcw_out_TKEEP,
    fcw_out_TSTRB);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ap_clk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ap_clk, ASSOCIATED_BUSIF in_stream:fcw_out, ASSOCIATED_RESET ap_rst_n, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input ap_clk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 ap_rst_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ap_rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input ap_rst_n;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TVALID" *) input in_stream_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TREADY" *) output in_stream_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TDATA" *) input [7:0]in_stream_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TLAST" *) input [0:0]in_stream_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TKEEP" *) input [0:0]in_stream_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TSTRB" *) input [0:0]in_stream_TSTRB;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TUSER" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME in_stream, TDATA_NUM_BYTES 1, TUSER_WIDTH 13, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input [12:0]in_stream_TUSER;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 fcw_out TVALID" *) output fcw_out_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 fcw_out TREADY" *) input fcw_out_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 fcw_out TDATA" *) output [31:0]fcw_out_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 fcw_out TLAST" *) output [0:0]fcw_out_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 fcw_out TKEEP" *) output [3:0]fcw_out_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 fcw_out TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME fcw_out, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) output [3:0]fcw_out_TSTRB;

  wire \<const0> ;
  wire \<const1> ;
  wire ap_clk;
  wire ap_rst_n;
  wire [20:0]\^fcw_out_TDATA ;
  wire [0:0]fcw_out_TLAST;
  wire fcw_out_TREADY;
  wire fcw_out_TVALID;
  wire [7:0]in_stream_TDATA;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TREADY;
  wire [12:0]in_stream_TUSER;
  wire in_stream_TVALID;
  wire [31:21]NLW_inst_fcw_out_TDATA_UNCONNECTED;
  wire [3:0]NLW_inst_fcw_out_TKEEP_UNCONNECTED;
  wire [3:0]NLW_inst_fcw_out_TSTRB_UNCONNECTED;

  assign fcw_out_TDATA[31] = \<const0> ;
  assign fcw_out_TDATA[30] = \<const0> ;
  assign fcw_out_TDATA[29] = \<const0> ;
  assign fcw_out_TDATA[28] = \<const0> ;
  assign fcw_out_TDATA[27] = \<const0> ;
  assign fcw_out_TDATA[26] = \<const1> ;
  assign fcw_out_TDATA[25] = \<const0> ;
  assign fcw_out_TDATA[24] = \<const0> ;
  assign fcw_out_TDATA[23] = \<const0> ;
  assign fcw_out_TDATA[22] = \<const0> ;
  assign fcw_out_TDATA[21] = \<const1> ;
  assign fcw_out_TDATA[20:0] = \^fcw_out_TDATA [20:0];
  assign fcw_out_TKEEP[3] = \<const1> ;
  assign fcw_out_TKEEP[2] = \<const1> ;
  assign fcw_out_TKEEP[1] = \<const1> ;
  assign fcw_out_TKEEP[0] = \<const1> ;
  assign fcw_out_TSTRB[3] = \<const0> ;
  assign fcw_out_TSTRB[2] = \<const0> ;
  assign fcw_out_TSTRB[1] = \<const0> ;
  assign fcw_out_TSTRB[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  VCC VCC
       (.P(\<const1> ));
  (* SDX_KERNEL = "true" *) 
  (* SDX_KERNEL_SYNTH_INST = "inst" *) 
  (* SDX_KERNEL_TYPE = "hls" *) 
  (* ap_ST_fsm_pp0_stage0 = "1'b1" *) 
  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2 inst
       (.ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .fcw_out_TDATA({NLW_inst_fcw_out_TDATA_UNCONNECTED[31:21],\^fcw_out_TDATA }),
        .fcw_out_TKEEP(NLW_inst_fcw_out_TKEEP_UNCONNECTED[3:0]),
        .fcw_out_TLAST(fcw_out_TLAST),
        .fcw_out_TREADY(fcw_out_TREADY),
        .fcw_out_TSTRB(NLW_inst_fcw_out_TSTRB_UNCONNECTED[3:0]),
        .fcw_out_TVALID(fcw_out_TVALID),
        .in_stream_TDATA({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,in_stream_TDATA[0]}),
        .in_stream_TKEEP(1'b0),
        .in_stream_TLAST(in_stream_TLAST),
        .in_stream_TREADY(in_stream_TREADY),
        .in_stream_TSTRB(1'b0),
        .in_stream_TUSER(in_stream_TUSER),
        .in_stream_TVALID(in_stream_TVALID));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2" *) (* ap_ST_fsm_pp0_stage0 = "1'b1" *) (* hls_module = "yes" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2
   (ap_clk,
    ap_rst_n,
    in_stream_TDATA,
    in_stream_TVALID,
    in_stream_TREADY,
    in_stream_TKEEP,
    in_stream_TSTRB,
    in_stream_TUSER,
    in_stream_TLAST,
    fcw_out_TDATA,
    fcw_out_TVALID,
    fcw_out_TREADY,
    fcw_out_TKEEP,
    fcw_out_TSTRB,
    fcw_out_TLAST);
  input ap_clk;
  input ap_rst_n;
  input [7:0]in_stream_TDATA;
  input in_stream_TVALID;
  output in_stream_TREADY;
  input [0:0]in_stream_TKEEP;
  input [0:0]in_stream_TSTRB;
  input [12:0]in_stream_TUSER;
  input [0:0]in_stream_TLAST;
  output [31:0]fcw_out_TDATA;
  output fcw_out_TVALID;
  input fcw_out_TREADY;
  output [3:0]fcw_out_TKEEP;
  output [3:0]fcw_out_TSTRB;
  output [0:0]fcw_out_TLAST;

  wire \<const0> ;
  wire B_V_data_1_sel0;
  wire GAUSS_LUT_U_n_0;
  wire GAUSS_LUT_U_n_1;
  wire GAUSS_LUT_U_n_10;
  wire GAUSS_LUT_U_n_11;
  wire GAUSS_LUT_U_n_12;
  wire GAUSS_LUT_U_n_13;
  wire GAUSS_LUT_U_n_14;
  wire GAUSS_LUT_U_n_15;
  wire GAUSS_LUT_U_n_16;
  wire GAUSS_LUT_U_n_17;
  wire GAUSS_LUT_U_n_18;
  wire GAUSS_LUT_U_n_19;
  wire GAUSS_LUT_U_n_2;
  wire GAUSS_LUT_U_n_20;
  wire GAUSS_LUT_U_n_3;
  wire GAUSS_LUT_U_n_4;
  wire GAUSS_LUT_U_n_5;
  wire GAUSS_LUT_U_n_6;
  wire GAUSS_LUT_U_n_7;
  wire GAUSS_LUT_U_n_8;
  wire GAUSS_LUT_U_n_9;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter2;
  wire ap_enable_reg_pp0_iter3;
  wire ap_enable_reg_pp0_iter4;
  wire ap_enable_reg_pp0_iter5;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [20:0]\^fcw_out_TDATA ;
  wire [0:0]fcw_out_TLAST;
  wire fcw_out_TREADY;
  wire fcw_out_TVALID;
  wire [2:0]grp_fu_198_p0;
  wire \history_reg_n_0_[2] ;
  wire [7:0]in_stream_TDATA;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TLAST_int_regslice;
  wire in_stream_TREADY;
  wire [12:0]in_stream_TUSER;
  wire in_stream_TVALID;
  wire in_stream_TVALID_int_regslice;
  wire \input_pkt_last_V_reg_212_pp0_iter2_reg_reg[0]_srl3_n_0 ;
  wire input_pkt_last_V_reg_212_pp0_iter3_reg;
  wire [12:0]input_pkt_user_V_reg_207;
  wire [2:0]or_ln_fu_157_p3;
  wire regslice_both_fcw_out_V_data_V_U_n_10;
  wire regslice_both_fcw_out_V_data_V_U_n_11;
  wire regslice_both_fcw_out_V_data_V_U_n_12;
  wire regslice_both_fcw_out_V_data_V_U_n_13;
  wire regslice_both_fcw_out_V_data_V_U_n_14;
  wire regslice_both_fcw_out_V_data_V_U_n_15;
  wire regslice_both_fcw_out_V_data_V_U_n_16;
  wire regslice_both_fcw_out_V_data_V_U_n_17;
  wire regslice_both_fcw_out_V_data_V_U_n_18;
  wire regslice_both_fcw_out_V_data_V_U_n_19;
  wire regslice_both_fcw_out_V_data_V_U_n_2;
  wire regslice_both_fcw_out_V_data_V_U_n_20;
  wire regslice_both_fcw_out_V_data_V_U_n_21;
  wire regslice_both_fcw_out_V_data_V_U_n_22;
  wire regslice_both_fcw_out_V_data_V_U_n_23;
  wire regslice_both_fcw_out_V_data_V_U_n_24;
  wire regslice_both_fcw_out_V_data_V_U_n_25;
  wire regslice_both_fcw_out_V_data_V_U_n_26;
  wire regslice_both_fcw_out_V_data_V_U_n_27;
  wire regslice_both_fcw_out_V_data_V_U_n_28;
  wire regslice_both_fcw_out_V_data_V_U_n_29;
  wire regslice_both_fcw_out_V_data_V_U_n_3;
  wire regslice_both_fcw_out_V_data_V_U_n_30;
  wire regslice_both_fcw_out_V_data_V_U_n_31;
  wire regslice_both_fcw_out_V_data_V_U_n_32;
  wire regslice_both_fcw_out_V_data_V_U_n_33;
  wire regslice_both_fcw_out_V_data_V_U_n_34;
  wire regslice_both_fcw_out_V_data_V_U_n_35;
  wire regslice_both_fcw_out_V_data_V_U_n_36;
  wire regslice_both_fcw_out_V_data_V_U_n_37;
  wire regslice_both_fcw_out_V_data_V_U_n_38;
  wire regslice_both_fcw_out_V_data_V_U_n_39;
  wire regslice_both_fcw_out_V_data_V_U_n_4;
  wire regslice_both_fcw_out_V_data_V_U_n_40;
  wire regslice_both_fcw_out_V_data_V_U_n_41;
  wire regslice_both_fcw_out_V_data_V_U_n_42;
  wire regslice_both_fcw_out_V_data_V_U_n_5;
  wire regslice_both_fcw_out_V_data_V_U_n_6;
  wire regslice_both_fcw_out_V_data_V_U_n_7;
  wire regslice_both_fcw_out_V_data_V_U_n_8;
  wire regslice_both_fcw_out_V_data_V_U_n_9;
  wire regslice_both_in_stream_V_data_V_U_n_2;
  wire regslice_both_in_stream_V_user_V_U_n_0;
  wire regslice_both_in_stream_V_user_V_U_n_1;
  wire regslice_both_in_stream_V_user_V_U_n_2;
  wire [12:0]sel0;
  wire [15:0]zext_ln45_2_fu_179_p1;

  assign fcw_out_TDATA[31] = \<const0> ;
  assign fcw_out_TDATA[30] = \<const0> ;
  assign fcw_out_TDATA[29] = \<const0> ;
  assign fcw_out_TDATA[28] = \<const0> ;
  assign fcw_out_TDATA[27] = \<const0> ;
  assign fcw_out_TDATA[26] = \<const0> ;
  assign fcw_out_TDATA[25] = \<const0> ;
  assign fcw_out_TDATA[24] = \<const0> ;
  assign fcw_out_TDATA[23] = \<const0> ;
  assign fcw_out_TDATA[22] = \<const0> ;
  assign fcw_out_TDATA[21] = \<const0> ;
  assign fcw_out_TDATA[20:0] = \^fcw_out_TDATA [20:0];
  assign fcw_out_TKEEP[3] = \<const0> ;
  assign fcw_out_TKEEP[2] = \<const0> ;
  assign fcw_out_TKEEP[1] = \<const0> ;
  assign fcw_out_TKEEP[0] = \<const0> ;
  assign fcw_out_TSTRB[3] = \<const0> ;
  assign fcw_out_TSTRB[2] = \<const0> ;
  assign fcw_out_TSTRB[1] = \<const0> ;
  assign fcw_out_TSTRB[0] = \<const0> ;
  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_GAUSS_LUT_ROM_AUTO_1R GAUSS_LUT_U
       (.B_V_data_1_payload_A0({GAUSS_LUT_U_n_0,GAUSS_LUT_U_n_1,GAUSS_LUT_U_n_2,GAUSS_LUT_U_n_3,GAUSS_LUT_U_n_4,GAUSS_LUT_U_n_5,GAUSS_LUT_U_n_6,GAUSS_LUT_U_n_7,GAUSS_LUT_U_n_8,GAUSS_LUT_U_n_9,GAUSS_LUT_U_n_10,GAUSS_LUT_U_n_11,GAUSS_LUT_U_n_12,GAUSS_LUT_U_n_13,GAUSS_LUT_U_n_14,GAUSS_LUT_U_n_15,GAUSS_LUT_U_n_16,GAUSS_LUT_U_n_17,GAUSS_LUT_U_n_18,GAUSS_LUT_U_n_19,GAUSS_LUT_U_n_20}),
        .P(zext_ln45_2_fu_179_p1),
        .RDEN(regslice_both_fcw_out_V_data_V_U_n_3),
        .ap_clk(ap_clk),
        .q0_reg_0_0_0(regslice_both_fcw_out_V_data_V_U_n_42),
        .q0_reg_0_10_0(regslice_both_fcw_out_V_data_V_U_n_22),
        .q0_reg_0_11_0(regslice_both_fcw_out_V_data_V_U_n_20),
        .q0_reg_0_12_0(regslice_both_fcw_out_V_data_V_U_n_18),
        .q0_reg_0_13_0(regslice_both_fcw_out_V_data_V_U_n_16),
        .q0_reg_0_14_0(regslice_both_fcw_out_V_data_V_U_n_14),
        .q0_reg_0_15_0(regslice_both_fcw_out_V_data_V_U_n_12),
        .q0_reg_0_16_0(regslice_both_fcw_out_V_data_V_U_n_10),
        .q0_reg_0_17_0(regslice_both_fcw_out_V_data_V_U_n_8),
        .q0_reg_0_18_0(regslice_both_fcw_out_V_data_V_U_n_6),
        .q0_reg_0_19_0(regslice_both_fcw_out_V_data_V_U_n_4),
        .q0_reg_0_1_0(regslice_both_fcw_out_V_data_V_U_n_40),
        .q0_reg_0_2_0(regslice_both_fcw_out_V_data_V_U_n_38),
        .q0_reg_0_3_0(regslice_both_fcw_out_V_data_V_U_n_36),
        .q0_reg_0_4_0(regslice_both_fcw_out_V_data_V_U_n_34),
        .q0_reg_0_5_0(regslice_both_fcw_out_V_data_V_U_n_32),
        .q0_reg_0_6_0(regslice_both_fcw_out_V_data_V_U_n_30),
        .q0_reg_0_7_0(regslice_both_fcw_out_V_data_V_U_n_28),
        .q0_reg_0_8_0(regslice_both_fcw_out_V_data_V_U_n_26),
        .q0_reg_0_9_0(regslice_both_fcw_out_V_data_V_U_n_24),
        .q0_reg_1_0_0(regslice_both_fcw_out_V_data_V_U_n_41),
        .q0_reg_1_10_0(regslice_both_fcw_out_V_data_V_U_n_21),
        .q0_reg_1_11_0(regslice_both_fcw_out_V_data_V_U_n_19),
        .q0_reg_1_12_0(regslice_both_fcw_out_V_data_V_U_n_17),
        .q0_reg_1_13_0(regslice_both_fcw_out_V_data_V_U_n_15),
        .q0_reg_1_14_0(regslice_both_fcw_out_V_data_V_U_n_13),
        .q0_reg_1_15_0(regslice_both_fcw_out_V_data_V_U_n_11),
        .q0_reg_1_16_0(regslice_both_fcw_out_V_data_V_U_n_9),
        .q0_reg_1_17_0(regslice_both_fcw_out_V_data_V_U_n_7),
        .q0_reg_1_18_0(regslice_both_fcw_out_V_data_V_U_n_5),
        .q0_reg_1_1_0(regslice_both_fcw_out_V_data_V_U_n_39),
        .q0_reg_1_2_0(regslice_both_fcw_out_V_data_V_U_n_37),
        .q0_reg_1_3_0(regslice_both_fcw_out_V_data_V_U_n_35),
        .q0_reg_1_4_0(regslice_both_fcw_out_V_data_V_U_n_33),
        .q0_reg_1_5_0(regslice_both_fcw_out_V_data_V_U_n_31),
        .q0_reg_1_6_0(regslice_both_fcw_out_V_data_V_U_n_29),
        .q0_reg_1_7_0(regslice_both_fcw_out_V_data_V_U_n_27),
        .q0_reg_1_8_0(regslice_both_fcw_out_V_data_V_U_n_25),
        .q0_reg_1_9_0(regslice_both_fcw_out_V_data_V_U_n_23));
  GND GND
       (.G(\<const0> ));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter1_reg
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(1'b1),
        .Q(ap_enable_reg_pp0_iter1),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter2_reg
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(ap_enable_reg_pp0_iter1),
        .Q(ap_enable_reg_pp0_iter2),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter3_reg
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(ap_enable_reg_pp0_iter2),
        .Q(ap_enable_reg_pp0_iter3),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter4_reg
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(ap_enable_reg_pp0_iter3),
        .Q(ap_enable_reg_pp0_iter4),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter5_reg
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(ap_enable_reg_pp0_iter4),
        .Q(ap_enable_reg_pp0_iter5),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b1)) 
    \history_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_in_stream_V_user_V_U_n_2),
        .Q(or_ln_fu_157_p3[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b1)) 
    \history_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_in_stream_V_user_V_U_n_1),
        .Q(or_ln_fu_157_p3[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b1)) 
    \history_reg[2] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_in_stream_V_user_V_U_n_0),
        .Q(\history_reg_n_0_[2] ),
        .R(1'b0));
  (* srl_bus_name = "inst/\\input_pkt_last_V_reg_212_pp0_iter2_reg_reg " *) 
  (* srl_name = "inst/\\input_pkt_last_V_reg_212_pp0_iter2_reg_reg[0]_srl3 " *) 
  SRL16E \input_pkt_last_V_reg_212_pp0_iter2_reg_reg[0]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(B_V_data_1_sel0),
        .CLK(ap_clk),
        .D(in_stream_TLAST_int_regslice),
        .Q(\input_pkt_last_V_reg_212_pp0_iter2_reg_reg[0]_srl3_n_0 ));
  FDRE \input_pkt_last_V_reg_212_pp0_iter3_reg_reg[0]__0 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(\input_pkt_last_V_reg_212_pp0_iter2_reg_reg[0]_srl3_n_0 ),
        .Q(input_pkt_last_V_reg_212_pp0_iter3_reg),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[0]),
        .Q(input_pkt_user_V_reg_207[0]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[10]),
        .Q(input_pkt_user_V_reg_207[10]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[11]),
        .Q(input_pkt_user_V_reg_207[11]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[12]),
        .Q(input_pkt_user_V_reg_207[12]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[1]),
        .Q(input_pkt_user_V_reg_207[1]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[2]),
        .Q(input_pkt_user_V_reg_207[2]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[3]),
        .Q(input_pkt_user_V_reg_207[3]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[4]),
        .Q(input_pkt_user_V_reg_207[4]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[5]),
        .Q(input_pkt_user_V_reg_207[5]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[6]),
        .Q(input_pkt_user_V_reg_207[6]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[7]),
        .Q(input_pkt_user_V_reg_207[7]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[8]),
        .Q(input_pkt_user_V_reg_207[8]),
        .R(1'b0));
  FDRE \input_pkt_user_V_reg_207_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(sel0[9]),
        .Q(input_pkt_user_V_reg_207[9]),
        .R(1'b0));
  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_mac_muladd_3ns_13ns_13ns_16_4_1 mac_muladd_3ns_13ns_13ns_16_4_1_U1
       (.A(grp_fu_198_p0),
        .B_V_data_1_sel0(B_V_data_1_sel0),
        .P(zext_ln45_2_fu_179_p1),
        .Q(input_pkt_user_V_reg_207),
        .ap_clk(ap_clk));
  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both__parameterized2 regslice_both_fcw_out_V_data_V_U
       (.B_V_data_1_sel0(B_V_data_1_sel0),
        .B_V_data_1_sel_rd_reg_0(regslice_both_fcw_out_V_data_V_U_n_2),
        .B_V_data_1_sel_rd_reg_1(regslice_both_in_stream_V_data_V_U_n_2),
        .\B_V_data_1_state_reg[0]_0 (fcw_out_TVALID),
        .D({GAUSS_LUT_U_n_0,GAUSS_LUT_U_n_1,GAUSS_LUT_U_n_2,GAUSS_LUT_U_n_3,GAUSS_LUT_U_n_4,GAUSS_LUT_U_n_5,GAUSS_LUT_U_n_6,GAUSS_LUT_U_n_7,GAUSS_LUT_U_n_8,GAUSS_LUT_U_n_9,GAUSS_LUT_U_n_10,GAUSS_LUT_U_n_11,GAUSS_LUT_U_n_12,GAUSS_LUT_U_n_13,GAUSS_LUT_U_n_14,GAUSS_LUT_U_n_15,GAUSS_LUT_U_n_16,GAUSS_LUT_U_n_17,GAUSS_LUT_U_n_18,GAUSS_LUT_U_n_19,GAUSS_LUT_U_n_20}),
        .RDEN(regslice_both_fcw_out_V_data_V_U_n_3),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter3(ap_enable_reg_pp0_iter3),
        .ap_enable_reg_pp0_iter3_reg(regslice_both_fcw_out_V_data_V_U_n_4),
        .ap_enable_reg_pp0_iter3_reg_0(regslice_both_fcw_out_V_data_V_U_n_5),
        .ap_enable_reg_pp0_iter3_reg_1(regslice_both_fcw_out_V_data_V_U_n_6),
        .ap_enable_reg_pp0_iter3_reg_10(regslice_both_fcw_out_V_data_V_U_n_15),
        .ap_enable_reg_pp0_iter3_reg_11(regslice_both_fcw_out_V_data_V_U_n_16),
        .ap_enable_reg_pp0_iter3_reg_12(regslice_both_fcw_out_V_data_V_U_n_17),
        .ap_enable_reg_pp0_iter3_reg_13(regslice_both_fcw_out_V_data_V_U_n_18),
        .ap_enable_reg_pp0_iter3_reg_14(regslice_both_fcw_out_V_data_V_U_n_19),
        .ap_enable_reg_pp0_iter3_reg_15(regslice_both_fcw_out_V_data_V_U_n_20),
        .ap_enable_reg_pp0_iter3_reg_16(regslice_both_fcw_out_V_data_V_U_n_21),
        .ap_enable_reg_pp0_iter3_reg_17(regslice_both_fcw_out_V_data_V_U_n_22),
        .ap_enable_reg_pp0_iter3_reg_18(regslice_both_fcw_out_V_data_V_U_n_23),
        .ap_enable_reg_pp0_iter3_reg_19(regslice_both_fcw_out_V_data_V_U_n_24),
        .ap_enable_reg_pp0_iter3_reg_2(regslice_both_fcw_out_V_data_V_U_n_7),
        .ap_enable_reg_pp0_iter3_reg_20(regslice_both_fcw_out_V_data_V_U_n_25),
        .ap_enable_reg_pp0_iter3_reg_21(regslice_both_fcw_out_V_data_V_U_n_26),
        .ap_enable_reg_pp0_iter3_reg_22(regslice_both_fcw_out_V_data_V_U_n_27),
        .ap_enable_reg_pp0_iter3_reg_23(regslice_both_fcw_out_V_data_V_U_n_28),
        .ap_enable_reg_pp0_iter3_reg_24(regslice_both_fcw_out_V_data_V_U_n_29),
        .ap_enable_reg_pp0_iter3_reg_25(regslice_both_fcw_out_V_data_V_U_n_30),
        .ap_enable_reg_pp0_iter3_reg_26(regslice_both_fcw_out_V_data_V_U_n_31),
        .ap_enable_reg_pp0_iter3_reg_27(regslice_both_fcw_out_V_data_V_U_n_32),
        .ap_enable_reg_pp0_iter3_reg_28(regslice_both_fcw_out_V_data_V_U_n_33),
        .ap_enable_reg_pp0_iter3_reg_29(regslice_both_fcw_out_V_data_V_U_n_34),
        .ap_enable_reg_pp0_iter3_reg_3(regslice_both_fcw_out_V_data_V_U_n_8),
        .ap_enable_reg_pp0_iter3_reg_30(regslice_both_fcw_out_V_data_V_U_n_35),
        .ap_enable_reg_pp0_iter3_reg_31(regslice_both_fcw_out_V_data_V_U_n_36),
        .ap_enable_reg_pp0_iter3_reg_32(regslice_both_fcw_out_V_data_V_U_n_37),
        .ap_enable_reg_pp0_iter3_reg_33(regslice_both_fcw_out_V_data_V_U_n_38),
        .ap_enable_reg_pp0_iter3_reg_34(regslice_both_fcw_out_V_data_V_U_n_39),
        .ap_enable_reg_pp0_iter3_reg_35(regslice_both_fcw_out_V_data_V_U_n_40),
        .ap_enable_reg_pp0_iter3_reg_36(regslice_both_fcw_out_V_data_V_U_n_41),
        .ap_enable_reg_pp0_iter3_reg_37(regslice_both_fcw_out_V_data_V_U_n_42),
        .ap_enable_reg_pp0_iter3_reg_4(regslice_both_fcw_out_V_data_V_U_n_9),
        .ap_enable_reg_pp0_iter3_reg_5(regslice_both_fcw_out_V_data_V_U_n_10),
        .ap_enable_reg_pp0_iter3_reg_6(regslice_both_fcw_out_V_data_V_U_n_11),
        .ap_enable_reg_pp0_iter3_reg_7(regslice_both_fcw_out_V_data_V_U_n_12),
        .ap_enable_reg_pp0_iter3_reg_8(regslice_both_fcw_out_V_data_V_U_n_13),
        .ap_enable_reg_pp0_iter3_reg_9(regslice_both_fcw_out_V_data_V_U_n_14),
        .ap_enable_reg_pp0_iter4(ap_enable_reg_pp0_iter4),
        .ap_enable_reg_pp0_iter5(ap_enable_reg_pp0_iter5),
        .ap_rst_n_inv(ap_rst_n_inv),
        .fcw_out_TDATA(\^fcw_out_TDATA ),
        .fcw_out_TREADY(fcw_out_TREADY),
        .in_stream_TVALID_int_regslice(in_stream_TVALID_int_regslice));
  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both__parameterized0 regslice_both_fcw_out_V_last_V_U
       (.B_V_data_1_sel0(B_V_data_1_sel0),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter4(ap_enable_reg_pp0_iter4),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .fcw_out_TLAST(fcw_out_TLAST),
        .fcw_out_TREADY(fcw_out_TREADY),
        .input_pkt_last_V_reg_212_pp0_iter3_reg(input_pkt_last_V_reg_212_pp0_iter3_reg));
  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both regslice_both_in_stream_V_data_V_U
       (.B_V_data_1_sel0(B_V_data_1_sel0),
        .B_V_data_1_sel_rd_reg_0(regslice_both_in_stream_V_data_V_U_n_2),
        .B_V_data_1_sel_rd_reg_1(regslice_both_fcw_out_V_data_V_U_n_2),
        .\B_V_data_1_state_reg[1]_0 (in_stream_TREADY),
        .ap_clk(ap_clk),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_stream_TDATA(in_stream_TDATA[0]),
        .in_stream_TVALID(in_stream_TVALID),
        .in_stream_TVALID_int_regslice(in_stream_TVALID_int_regslice),
        .or_ln_fu_157_p3(or_ln_fu_157_p3[0]));
  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both__parameterized0_0 regslice_both_in_stream_V_last_V_U
       (.B_V_data_1_sel0(B_V_data_1_sel0),
        .ap_clk(ap_clk),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_stream_TLAST(in_stream_TLAST),
        .in_stream_TLAST_int_regslice(in_stream_TLAST_int_regslice),
        .in_stream_TVALID(in_stream_TVALID));
  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both__parameterized1 regslice_both_in_stream_V_user_V_U
       (.A(grp_fu_198_p0),
        .B_V_data_1_sel0(B_V_data_1_sel0),
        .D(sel0),
        .ap_clk(ap_clk),
        .ap_rst_n_inv(ap_rst_n_inv),
        .\history_reg[0] (regslice_both_in_stream_V_user_V_U_n_1),
        .\history_reg[0]_0 (regslice_both_in_stream_V_user_V_U_n_2),
        .\history_reg[1] (regslice_both_in_stream_V_user_V_U_n_0),
        .\history_reg[2] (\history_reg_n_0_[2] ),
        .in_stream_TUSER(in_stream_TUSER),
        .in_stream_TVALID(in_stream_TVALID),
        .or_ln_fu_157_p3(or_ln_fu_157_p3));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2_GAUSS_LUT_ROM_AUTO_1R" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_GAUSS_LUT_ROM_AUTO_1R
   (B_V_data_1_payload_A0,
    ap_clk,
    q0_reg_0_0_0,
    P,
    q0_reg_1_0_0,
    q0_reg_0_1_0,
    q0_reg_1_1_0,
    q0_reg_0_2_0,
    q0_reg_1_2_0,
    q0_reg_0_3_0,
    q0_reg_1_3_0,
    q0_reg_0_4_0,
    q0_reg_1_4_0,
    q0_reg_0_5_0,
    q0_reg_1_5_0,
    q0_reg_0_6_0,
    q0_reg_1_6_0,
    q0_reg_0_7_0,
    q0_reg_1_7_0,
    q0_reg_0_8_0,
    q0_reg_1_8_0,
    q0_reg_0_9_0,
    q0_reg_1_9_0,
    q0_reg_0_10_0,
    q0_reg_1_10_0,
    q0_reg_0_11_0,
    q0_reg_1_11_0,
    q0_reg_0_12_0,
    q0_reg_1_12_0,
    q0_reg_0_13_0,
    q0_reg_1_13_0,
    q0_reg_0_14_0,
    q0_reg_1_14_0,
    q0_reg_0_15_0,
    q0_reg_1_15_0,
    q0_reg_0_16_0,
    q0_reg_1_16_0,
    q0_reg_0_17_0,
    q0_reg_1_17_0,
    q0_reg_0_18_0,
    q0_reg_1_18_0,
    q0_reg_0_19_0,
    RDEN);
  output [20:0]B_V_data_1_payload_A0;
  input ap_clk;
  input q0_reg_0_0_0;
  input [15:0]P;
  input q0_reg_1_0_0;
  input q0_reg_0_1_0;
  input q0_reg_1_1_0;
  input q0_reg_0_2_0;
  input q0_reg_1_2_0;
  input q0_reg_0_3_0;
  input q0_reg_1_3_0;
  input q0_reg_0_4_0;
  input q0_reg_1_4_0;
  input q0_reg_0_5_0;
  input q0_reg_1_5_0;
  input q0_reg_0_6_0;
  input q0_reg_1_6_0;
  input q0_reg_0_7_0;
  input q0_reg_1_7_0;
  input q0_reg_0_8_0;
  input q0_reg_1_8_0;
  input q0_reg_0_9_0;
  input q0_reg_1_9_0;
  input q0_reg_0_10_0;
  input q0_reg_1_10_0;
  input q0_reg_0_11_0;
  input q0_reg_1_11_0;
  input q0_reg_0_12_0;
  input q0_reg_1_12_0;
  input q0_reg_0_13_0;
  input q0_reg_1_13_0;
  input q0_reg_0_14_0;
  input q0_reg_1_14_0;
  input q0_reg_0_15_0;
  input q0_reg_1_15_0;
  input q0_reg_0_16_0;
  input q0_reg_1_16_0;
  input q0_reg_0_17_0;
  input q0_reg_1_17_0;
  input q0_reg_0_18_0;
  input q0_reg_1_18_0;
  input q0_reg_0_19_0;
  input RDEN;

  wire [20:0]B_V_data_1_payload_A0;
  wire \B_V_data_1_payload_A[11]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[11]_i_3_n_0 ;
  wire \B_V_data_1_payload_A[15]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[15]_i_3_n_0 ;
  wire \B_V_data_1_payload_A[19]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[19]_i_3_n_0 ;
  wire \B_V_data_1_payload_A[3]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[3]_i_3_n_0 ;
  wire \B_V_data_1_payload_A[7]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[7]_i_3_n_0 ;
  wire \B_V_data_1_payload_A_reg[11]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[11]_i_1_n_1 ;
  wire \B_V_data_1_payload_A_reg[11]_i_1_n_2 ;
  wire \B_V_data_1_payload_A_reg[11]_i_1_n_3 ;
  wire \B_V_data_1_payload_A_reg[15]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[15]_i_1_n_1 ;
  wire \B_V_data_1_payload_A_reg[15]_i_1_n_2 ;
  wire \B_V_data_1_payload_A_reg[15]_i_1_n_3 ;
  wire \B_V_data_1_payload_A_reg[19]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[19]_i_1_n_1 ;
  wire \B_V_data_1_payload_A_reg[19]_i_1_n_2 ;
  wire \B_V_data_1_payload_A_reg[19]_i_1_n_3 ;
  wire \B_V_data_1_payload_A_reg[20]_i_2_n_0 ;
  wire \B_V_data_1_payload_A_reg[20]_i_2_n_1 ;
  wire \B_V_data_1_payload_A_reg[20]_i_2_n_2 ;
  wire \B_V_data_1_payload_A_reg[20]_i_2_n_3 ;
  wire \B_V_data_1_payload_A_reg[3]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[3]_i_1_n_1 ;
  wire \B_V_data_1_payload_A_reg[3]_i_1_n_2 ;
  wire \B_V_data_1_payload_A_reg[3]_i_1_n_3 ;
  wire \B_V_data_1_payload_A_reg[7]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[7]_i_1_n_1 ;
  wire \B_V_data_1_payload_A_reg[7]_i_1_n_2 ;
  wire \B_V_data_1_payload_A_reg[7]_i_1_n_3 ;
  wire [19:0]GAUSS_LUT_q0;
  wire [15:0]P;
  wire RDEN;
  wire ap_clk;
  wire q0_reg_0_0_0;
  wire q0_reg_0_0_n_0;
  wire q0_reg_0_10_0;
  wire q0_reg_0_10_n_0;
  wire q0_reg_0_11_0;
  wire q0_reg_0_11_n_0;
  wire q0_reg_0_12_0;
  wire q0_reg_0_12_n_0;
  wire q0_reg_0_13_0;
  wire q0_reg_0_13_n_0;
  wire q0_reg_0_14_0;
  wire q0_reg_0_14_n_0;
  wire q0_reg_0_15_0;
  wire q0_reg_0_15_n_0;
  wire q0_reg_0_16_0;
  wire q0_reg_0_16_n_0;
  wire q0_reg_0_17_0;
  wire q0_reg_0_17_n_0;
  wire q0_reg_0_18_0;
  wire q0_reg_0_18_n_0;
  wire q0_reg_0_19_0;
  wire q0_reg_0_19_n_0;
  wire q0_reg_0_1_0;
  wire q0_reg_0_1_n_0;
  wire q0_reg_0_2_0;
  wire q0_reg_0_2_n_0;
  wire q0_reg_0_3_0;
  wire q0_reg_0_3_n_0;
  wire q0_reg_0_4_0;
  wire q0_reg_0_4_n_0;
  wire q0_reg_0_5_0;
  wire q0_reg_0_5_n_0;
  wire q0_reg_0_6_0;
  wire q0_reg_0_6_n_0;
  wire q0_reg_0_7_0;
  wire q0_reg_0_7_n_0;
  wire q0_reg_0_8_0;
  wire q0_reg_0_8_n_0;
  wire q0_reg_0_9_0;
  wire q0_reg_0_9_n_0;
  wire q0_reg_1_0_0;
  wire q0_reg_1_10_0;
  wire q0_reg_1_11_0;
  wire q0_reg_1_12_0;
  wire q0_reg_1_13_0;
  wire q0_reg_1_14_0;
  wire q0_reg_1_15_0;
  wire q0_reg_1_16_0;
  wire q0_reg_1_17_0;
  wire q0_reg_1_18_0;
  wire q0_reg_1_1_0;
  wire q0_reg_1_2_0;
  wire q0_reg_1_3_0;
  wire q0_reg_1_4_0;
  wire q0_reg_1_5_0;
  wire q0_reg_1_6_0;
  wire q0_reg_1_7_0;
  wire q0_reg_1_8_0;
  wire q0_reg_1_9_0;
  wire [3:1]\NLW_B_V_data_1_payload_A_reg[20]_i_2_O_UNCONNECTED ;
  wire NLW_q0_reg_0_0_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_0_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_0_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_0_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_0_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_0_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_0_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_0_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_0_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_0_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_0_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_0_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_1_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_1_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_1_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_1_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_1_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_1_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_1_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_1_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_1_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_1_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_1_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_1_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_10_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_10_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_10_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_10_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_10_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_10_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_10_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_10_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_10_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_10_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_10_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_10_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_11_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_11_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_11_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_11_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_11_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_11_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_11_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_11_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_11_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_11_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_11_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_11_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_12_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_12_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_12_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_12_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_12_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_12_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_12_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_12_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_12_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_12_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_12_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_12_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_13_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_13_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_13_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_13_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_13_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_13_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_13_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_13_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_13_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_13_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_13_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_13_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_14_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_14_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_14_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_14_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_14_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_14_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_14_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_14_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_14_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_14_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_14_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_14_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_15_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_15_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_15_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_15_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_15_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_15_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_15_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_15_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_15_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_15_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_15_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_15_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_16_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_16_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_16_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_16_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_16_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_16_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_16_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_16_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_16_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_16_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_16_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_16_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_17_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_17_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_17_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_17_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_17_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_17_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_17_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_17_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_17_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_17_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_17_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_17_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_18_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_18_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_18_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_18_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_18_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_18_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_18_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_18_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_18_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_18_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_18_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_18_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_19_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_19_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_19_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_19_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_19_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_19_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_19_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_19_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_19_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_19_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_19_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_19_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_2_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_2_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_2_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_2_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_2_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_2_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_2_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_2_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_2_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_2_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_2_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_2_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_3_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_3_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_3_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_3_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_3_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_3_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_3_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_3_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_3_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_3_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_3_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_3_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_4_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_4_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_4_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_4_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_4_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_4_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_4_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_4_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_4_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_4_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_4_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_4_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_5_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_5_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_5_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_5_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_5_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_5_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_5_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_5_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_5_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_5_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_5_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_5_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_6_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_6_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_6_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_6_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_6_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_6_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_6_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_6_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_6_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_6_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_6_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_6_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_7_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_7_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_7_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_7_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_7_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_7_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_7_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_7_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_7_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_7_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_7_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_7_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_8_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_8_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_8_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_8_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_8_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_8_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_8_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_8_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_8_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_8_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_8_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_8_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_0_9_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_0_9_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_9_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_9_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_0_9_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_0_9_DIPADIP_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_9_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_0_9_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_9_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_0_9_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_0_9_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_0_9_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_0_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_0_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_0_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_0_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_0_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_0_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_0_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_0_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_0_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_0_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_0_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_0_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_0_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_1_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_1_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_1_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_1_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_1_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_1_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_1_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_1_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_1_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_1_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_1_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_1_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_1_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_10_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_10_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_10_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_10_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_10_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_10_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_10_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_10_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_10_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_10_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_10_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_10_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_10_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_11_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_11_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_11_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_11_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_11_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_11_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_11_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_11_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_11_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_11_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_11_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_11_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_11_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_12_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_12_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_12_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_12_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_12_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_12_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_12_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_12_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_12_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_12_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_12_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_12_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_12_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_13_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_13_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_13_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_13_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_13_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_13_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_13_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_13_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_13_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_13_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_13_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_13_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_13_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_14_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_14_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_14_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_14_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_14_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_14_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_14_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_14_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_14_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_14_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_14_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_14_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_14_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_15_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_15_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_15_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_15_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_15_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_15_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_15_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_15_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_15_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_15_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_15_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_15_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_15_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_16_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_16_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_16_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_16_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_16_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_16_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_16_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_16_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_16_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_16_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_16_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_16_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_16_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_17_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_17_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_17_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_17_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_17_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_17_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_17_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_17_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_17_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_17_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_17_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_17_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_17_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_18_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_18_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_18_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_18_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_18_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_18_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_18_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_18_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_18_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_18_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_18_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_18_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_18_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_19_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_19_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_19_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_19_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_19_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_19_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_19_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_19_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_19_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_19_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_19_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_19_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_19_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_2_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_2_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_2_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_2_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_2_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_2_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_2_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_2_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_2_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_2_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_2_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_2_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_2_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_3_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_3_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_3_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_3_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_3_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_3_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_3_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_3_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_3_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_3_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_3_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_3_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_3_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_4_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_4_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_4_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_4_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_4_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_4_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_4_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_4_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_4_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_4_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_4_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_4_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_4_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_5_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_5_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_5_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_5_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_5_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_5_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_5_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_5_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_5_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_5_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_5_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_5_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_5_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_6_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_6_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_6_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_6_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_6_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_6_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_6_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_6_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_6_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_6_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_6_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_6_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_6_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_7_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_7_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_7_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_7_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_7_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_7_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_7_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_7_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_7_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_7_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_7_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_7_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_7_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_8_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_8_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_8_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_8_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_8_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_8_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_8_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_8_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_8_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_8_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_8_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_8_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_8_RDADDRECC_UNCONNECTED;
  wire NLW_q0_reg_1_9_CASCADEOUTA_UNCONNECTED;
  wire NLW_q0_reg_1_9_CASCADEOUTB_UNCONNECTED;
  wire NLW_q0_reg_1_9_DBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_9_INJECTDBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_9_INJECTSBITERR_UNCONNECTED;
  wire NLW_q0_reg_1_9_SBITERR_UNCONNECTED;
  wire [0:0]NLW_q0_reg_1_9_DIPADIP_UNCONNECTED;
  wire [31:1]NLW_q0_reg_1_9_DOADO_UNCONNECTED;
  wire [31:0]NLW_q0_reg_1_9_DOBDO_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_9_DOPADOP_UNCONNECTED;
  wire [3:0]NLW_q0_reg_1_9_DOPBDOP_UNCONNECTED;
  wire [7:0]NLW_q0_reg_1_9_ECCPARITY_UNCONNECTED;
  wire [8:0]NLW_q0_reg_1_9_RDADDRECC_UNCONNECTED;

  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[11]_i_2 
       (.I0(GAUSS_LUT_q0[11]),
        .O(\B_V_data_1_payload_A[11]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[11]_i_3 
       (.I0(GAUSS_LUT_q0[9]),
        .O(\B_V_data_1_payload_A[11]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[15]_i_2 
       (.I0(GAUSS_LUT_q0[15]),
        .O(\B_V_data_1_payload_A[15]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[15]_i_3 
       (.I0(GAUSS_LUT_q0[13]),
        .O(\B_V_data_1_payload_A[15]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[19]_i_2 
       (.I0(GAUSS_LUT_q0[19]),
        .O(\B_V_data_1_payload_A[19]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[19]_i_3 
       (.I0(GAUSS_LUT_q0[17]),
        .O(\B_V_data_1_payload_A[19]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[3]_i_2 
       (.I0(GAUSS_LUT_q0[3]),
        .O(\B_V_data_1_payload_A[3]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[3]_i_3 
       (.I0(GAUSS_LUT_q0[1]),
        .O(\B_V_data_1_payload_A[3]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[7]_i_2 
       (.I0(GAUSS_LUT_q0[7]),
        .O(\B_V_data_1_payload_A[7]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[7]_i_3 
       (.I0(GAUSS_LUT_q0[5]),
        .O(\B_V_data_1_payload_A[7]_i_3_n_0 ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[11]_i_1 
       (.CI(\B_V_data_1_payload_A_reg[7]_i_1_n_0 ),
        .CO({\B_V_data_1_payload_A_reg[11]_i_1_n_0 ,\B_V_data_1_payload_A_reg[11]_i_1_n_1 ,\B_V_data_1_payload_A_reg[11]_i_1_n_2 ,\B_V_data_1_payload_A_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({GAUSS_LUT_q0[11],1'b0,GAUSS_LUT_q0[9],1'b0}),
        .O(B_V_data_1_payload_A0[11:8]),
        .S({\B_V_data_1_payload_A[11]_i_2_n_0 ,GAUSS_LUT_q0[10],\B_V_data_1_payload_A[11]_i_3_n_0 ,GAUSS_LUT_q0[8]}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[15]_i_1 
       (.CI(\B_V_data_1_payload_A_reg[11]_i_1_n_0 ),
        .CO({\B_V_data_1_payload_A_reg[15]_i_1_n_0 ,\B_V_data_1_payload_A_reg[15]_i_1_n_1 ,\B_V_data_1_payload_A_reg[15]_i_1_n_2 ,\B_V_data_1_payload_A_reg[15]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({GAUSS_LUT_q0[15],1'b0,GAUSS_LUT_q0[13],1'b0}),
        .O(B_V_data_1_payload_A0[15:12]),
        .S({\B_V_data_1_payload_A[15]_i_2_n_0 ,GAUSS_LUT_q0[14],\B_V_data_1_payload_A[15]_i_3_n_0 ,GAUSS_LUT_q0[12]}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[19]_i_1 
       (.CI(\B_V_data_1_payload_A_reg[15]_i_1_n_0 ),
        .CO({\B_V_data_1_payload_A_reg[19]_i_1_n_0 ,\B_V_data_1_payload_A_reg[19]_i_1_n_1 ,\B_V_data_1_payload_A_reg[19]_i_1_n_2 ,\B_V_data_1_payload_A_reg[19]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,GAUSS_LUT_q0[17],1'b0}),
        .O(B_V_data_1_payload_A0[19:16]),
        .S({\B_V_data_1_payload_A[19]_i_2_n_0 ,GAUSS_LUT_q0[18],\B_V_data_1_payload_A[19]_i_3_n_0 ,GAUSS_LUT_q0[16]}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[20]_i_2 
       (.CI(\B_V_data_1_payload_A_reg[19]_i_1_n_0 ),
        .CO({\B_V_data_1_payload_A_reg[20]_i_2_n_0 ,\B_V_data_1_payload_A_reg[20]_i_2_n_1 ,\B_V_data_1_payload_A_reg[20]_i_2_n_2 ,\B_V_data_1_payload_A_reg[20]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_B_V_data_1_payload_A_reg[20]_i_2_O_UNCONNECTED [3:1],B_V_data_1_payload_A0[20]}),
        .S({1'b0,1'b0,1'b1,1'b0}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[3]_i_1 
       (.CI(1'b0),
        .CO({\B_V_data_1_payload_A_reg[3]_i_1_n_0 ,\B_V_data_1_payload_A_reg[3]_i_1_n_1 ,\B_V_data_1_payload_A_reg[3]_i_1_n_2 ,\B_V_data_1_payload_A_reg[3]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({GAUSS_LUT_q0[3],1'b0,GAUSS_LUT_q0[1],1'b0}),
        .O(B_V_data_1_payload_A0[3:0]),
        .S({\B_V_data_1_payload_A[3]_i_2_n_0 ,GAUSS_LUT_q0[2],\B_V_data_1_payload_A[3]_i_3_n_0 ,GAUSS_LUT_q0[0]}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[7]_i_1 
       (.CI(\B_V_data_1_payload_A_reg[3]_i_1_n_0 ),
        .CO({\B_V_data_1_payload_A_reg[7]_i_1_n_0 ,\B_V_data_1_payload_A_reg[7]_i_1_n_1 ,\B_V_data_1_payload_A_reg[7]_i_1_n_2 ,\B_V_data_1_payload_A_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({GAUSS_LUT_q0[7],1'b0,GAUSS_LUT_q0[5],1'b0}),
        .O(B_V_data_1_payload_A0[7:4]),
        .S({\B_V_data_1_payload_A[7]_i_2_n_0 ,GAUSS_LUT_q0[6],\B_V_data_1_payload_A[7]_i_3_n_0 ,GAUSS_LUT_q0[4]}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_0" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "0" *) 
  (* ram_slice_end = "0" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h000007FFFFFFC0000007FFFFFC000003FFFFF800003FFFFE00001FFFFC0000FF),
    .INIT_01(256'h000000000001FFFFFFFFFFFE00000000003FFFFFFFFF8000000007FFFFFFFC00),
    .INIT_02(256'h000003FFFFFFFFFFFFFFFFFFFFFF0000000000000000003FFFFFFFFFFFFFFF00),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000),
    .INIT_04(256'h000000000000000000000000000000000000000000000000000000000000001F),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'h00000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hE000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'h000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'h03FFFFFFFFFFFFFFF0000000000000000003FFFFFFFFFFFFFFFFFFFFFF000000),
    .INIT_17(256'h00FFFFFFFF8000000007FFFFFFFFF00000000001FFFFFFFFFFFE000000000000),
    .INIT_18(256'hFC0000FFFFE00001FFFFF000007FFFFF000000FFFFFF8000000FFFFFFF800000),
    .INIT_19(256'hF81F80FC07E03F807F00FF00FF00FF803FF003FF001FFC003FFE0007FFE0000F),
    .INIT_1A(256'h8E3C71E3871E3C78F0E1E1C3C3C3C3C3E1E1F0F87C1E0F83E0F83F07C0F81F81),
    .INIT_1B(256'hCC999333366666666633333998CCE63398CE7318C639CE31CE31C738E38E38E3),
    .INIT_1C(256'h55AA954AB56AD4AD6B5AD6B4B4A5B4B496D24B6DB6DB6DB249B64D9364D9B266),
    .INIT_1D(256'h64CC99364DB2492492DB4B4B4B4A5295A95AB55AAD552AAAA555555555AAAAA5),
    .INIT_1E(256'h3C78783E0FE03FF00003FFC00007FE01F81F87C3C3C78E38E39C6319CCC66666),
    .INIT_1F(256'hCCCC9926DA4B6B4AD4AB5555555554AB56A5AD2DA4924D936664C666339C638E),
    .INIT_20(256'h6B52A955556AB529692493664C66318E1C3C1F803FFFFFFC01F83E1C38E318CC),
    .INIT_21(256'h8C666C9B4B5A955555AB5A5B6CD998CE38787E001FE003F07871CE6332649B49),
    .INIT_22(256'hD4B6D9B98C783FC00FF078E73326DA5AB55552B5A49B3339C70F803FFC01F0F1),
    .INIT_23(256'hAAAB524999CF0FFFFF878C66C96B55556B493331C780FFF81F1CE66C92D4AAAA),
    .INIT_24(256'hCCE3E000F8E664B52AD5B6CC61F0003E3999B4AD54AD26CC71F8001F0E6664B5),
    .INIT_25(256'h256AD49B9C3FFE1CEC94AA94999C3FFF87332D2AA96999C3F07F0C66DAD556B6),
    .INIT_26(256'h76D2AD6CC700079996AAB6CC700038CDA555A4CE1FFF8E6694AD4931C1FF073B),
    .INIT_27(256'hAAB4CE1FC199296B6C7801E66D55699C3FE1CDB554B3381E0E36D555B338000E),
    .INIT_28(256'hFF8CDAAA4CF007196AAD98FFF19B555B38FF8665AA5B383C1CC955499C000E6D),
    .INIT_29(256'hFFCCD2B4CC1C1CDAAB663FE39296B31FFE335AD263FFC64AAA4C7FFCEDAAB663),
    .INIT_2A(256'hA4E000CDAAB238071B554987F864A6B67003892A933803892AB661FC334AB6C7),
    .INIT_2B(256'h39A54B31FF19B55B30FE192D299C00E6D553300066D55B3C03CDAAA660F0E4B5),
    .INIT_2C(256'h3FE192D4B31FFC6695A4E3F87255533078336AA4CE00E64AA48E00736AA9983C),
    .INIT_2D(256'h79B6AA49C7FF1995B5B30FF8E4955B33FFF1B6AAD9C3F8E6D55B31FF8E4AAA4E),
    .INIT_2E(256'h3652AD64E3E0F8E49555A6707E0E64A5293387E0E64AAA49C7FF0CDAD2933800),
    .INIT_2F(256'hAAB49B187C00F8E6496AAB49B9C1FF8399B6AAAD36387FF0E6695B56CCE0FF06),
    .INIT_30(256'h78E73326D2D6AAAAD6924CC63C3FFFFC1C7326D29555ADB6631F0000F1CCDB4A),
    .INIT_31(256'h49B6DB6DB6DA4B69696B5A952AB5555556AB529692493367338E3C1F800001F8),
    .INIT_32(256'hCE1E0FF0003FC1E1C7198C99B249696B5AA5554AAD5556AB56AD6A5296B4A5A4),
    .INIT_33(256'hA556A4999C3F00FC38CD92955552DB2638E07FFE070C666DB4A55556A5A4D999),
    .INIT_34(256'h5B663E07E336D555B231FFFE3992D552D9187FFF8E66D2AAB4931C3FFF871934),
    .INIT_35(256'hAD330FF8E69556CC7FFC66D556D8E003CC96A96CC3FFC736B56B271FFF1CDB55),
    .INIT_36(256'h5498FFE335B5B3801E6D55263FF8EDAA933800E652B49C7FC336AA931E03CCD2),
    .INIT_37(256'h1FF1DA52CC7F865AA4CF03CDAAA670079255B31FE392AA4E3FC3255498FFE325),
    .INIT_38(256'hCFFF32D691C071A54B3C079A54B3C079B55B3801C9696C7FE3255267FF192AD3),
    .INIT_39(256'h299E0F329AD8FF8CD5599FF8CB52CE00E6D56CE01CCAA930F86694B3000CDAA4),
    .INIT_3A(256'h6AA4C3F0C95499FFC6D55B1FF8CB52CE00725549C0066D5263FF1B55663FE36B),
    .INIT_3B(256'h34A96700732AACC7F8CD6D270019955B38039B55B38039B55B3C07125699FFE3),
    .INIT_3C(256'h2556C701C4956DC3E1DB5491E0F3255B3020E4B6B31FE3B6ADB8FE192AB6700F),
    .INIT_3D(256'hAACC7FE3694B71FF192AB67003925493801CDAA931FF1DA5ACC3F0CD696E3FE3),
    .INIT_3E(256'h27000E4AAB31FF19A52CC7FC64AAD980066D5498FF8CD29663FE335549C00392),
    .INIT_3F(256'h1FF1DA5ACC3F0CD696E3FE32556CE00724A9270039B55263FE3B4A5B1FF8CD55),
    .INIT_40(256'hC039B55261FC76D5B71FE335B49C10336A933C1E24AB6E1F0EDAA48E038DAA93),
    .INIT_41(256'h1FFE65A92380F36AB6700736AB6700736AA6600392DACC7F8CD55338039A54B3),
    .INIT_42(256'h5B1FF19AAB63FF192AD9800E4AA93801CD2B4C7FE36AAD8FFE64AA4C3F0C955B),
    .INIT_43(256'h956CC00334A5987C32554CE01CDAAD9C01CD2B4C7FE66AACC7FC6D6533C1E653),
    .INIT_44(256'h2D5263FF992A931FF8DA5A4E00736AB6780F34A96780F34A96380E25AD33FFCC),
    .INIT_45(256'h931FFC64AA930FF1C955271FE336A92780399556CF03CC956987F8CD296E3FE3),
    .INIT_46(256'h2CCF01E32555B30FF8E4B5299C00732556DC7FF192AAD9E00736B6B31FFC64AA),
    .INIT_47(256'hAB6CE3FFE3935AB5B38FFF0CDA55A4CF001C6DAAAD98FFF8CDAAA59C7FC332D5),
    .INIT_48(256'hB26387FFF0E324B5552D99C7FFF8626D2AAD2671FFFE3136AAADB31F81F19B6A),
    .INIT_49(256'h666C9695AAAA94B6D998C381FFF81C71936D2AAAA526CC70FC03F0E66495AA94),
    .INIT_4A(256'h9694B5A5295AD5AB55AAAAD54AAA956B5A5A493664C6638E1E0FF0003FC1E1CE),
    .INIT_4B(256'h7E000007E0F1C7339B324925A52B55AAAAAAB552A56B5A5A5B496DB6DB6DB648),
    .INIT_4C(256'h4B6CCE3C0003E319B6D6AAA52D9338E0FFFFF0F18CC925AD5555AD2D93339C78),
    .INIT_4D(256'h83FC1CCDAB6A599C3FF871B2D555B66707FE0E764B555A499C7C00F86364B555),
    .INIT_4E(256'h0073252D6CC3FF8E4955499C1F8732529499C1F83996AAA49C7C1F1C9AD529B1),
    .INIT_4F(256'hC95549C7FE336AAD9C7F0E6D55B63FFF336AA49C7FC336B6A663FF8E4955B678),
    .INIT_50(256'hF066555B3801C495499C01CC955B3078332AA9387F1C96A598FFE334AD261FF1),
    .INIT_51(256'hB49C3C199556CF00F36AAD9800332AAD9C00E652D261FC336AB663FE334A9670),
    .INIT_52(256'h8DB54B30FE19B5524700732552470039B594987F864AAB6380713556CC001C96),
    .INIT_53(256'h19B556DCFFF8C955498FFF192D6B31FFE335A5271FF19B556CE0E0CCB52CCFFF),
    .INIT_54(256'hD9C000E64AAA4CE0F07369569987FC736AAB663FFC66D55A63803CC9556CC7FF),
    .INIT_55(256'hC0007336AAADB1C1E07334AAB6CE1FF0E65AAAD99E0078DB5A52660FE1CCB556),
    .INIT_56(256'h7383FE0E324AD4A599C7FFE1CC96AA96CC700038CDB555A66780038CDAD52DB9),
    .INIT_57(256'hB5AAAD6D98C3F83F0E665A5552D33387FFF0E664A554A4DCE1FFF0E764AD5A93),
    .INIT_58(256'hB49999C3E0007E38CD92D4AAD4B66671F0003E18CDB6AD52B4999C7C001F1CCD),
    .INIT_59(256'h5554AD24D99CE3E07FFC078E33324B5AAAAB5A4D98C787FFFFC3CE66492B5556),
    .INIT_5A(256'h3C3E00FFF007C38E73336496B52AAAB5696D93339C783FC00FF078C6766DB4AD),
    .INIT_5B(256'h4B64993319CE38783F001FE001F87871CC666CDB696B56AAAAA56B4B64D998C6),
    .INIT_5C(256'hCC631C70E1F07E00FFFFFFF007E0F0E1C63198C99B24925A52B55AAAAA552B5A),
    .INIT_5D(256'hC718E731998C999B26C92496D2D695AB54AAAAAAAAAB54AD4B5B496D9264CCCC),
    .INIT_5E(256'h99998CCE6318E71C71C78F0F0F87E07E01FF80000FFF00003FF01FC1F07878F1),
    .INIT_5F(256'h955556AAAAAAAA955552AAD56AB56A56A5294B4B4B4B6D24924936C9B264CC99),
    .INIT_60(256'h99366C9B26C9B64936DB6DB6DB492DA4B4B694B4B5AD6B5AD4AD5AB54AA556AA),
    .INIT_61(256'h1C71C71C738E31CE31CE718C6339CC67319CCC6673333199999999B3332664CD),
    .INIT_62(256'h07E07C0F83F07C1F07C1E0F87C3E1E1F0F0F0F0F0E1E1C3C78F1E3871E38F1C7),
    .INIT_63(256'hC0001FFF8001FFF000FFE003FF003FF007FC03FC03FC03F807F01F80FC07E07E),
    .INIT_64(256'h7E000007E0F1C7339B324925A52B55AAAAAAB552A56B5A5A5B496DB6DB6DB648),
    .INIT_65(256'h4B6CCE3C0003E319B6D6AAA52D9338E0FFFFF0F18CC925AD5555AD2D93339C78),
    .INIT_66(256'h83FC1CCDAB6A599C3FF871B2D555B66707FE0E764B555A499C7C00F86364B555),
    .INIT_67(256'h0073252D6CC3FF8E4955499C1F8732529499C1F83996AAA49C7C1F1C9AD529B1),
    .INIT_68(256'hC95549C7FE336AAD9C7F0E6D55B63FFF336AA49C7FC336B6A663FF8E4955B678),
    .INIT_69(256'hF066555B3801C495499C01CC955B3078332AA9387F1C96A598FFE334AD261FF1),
    .INIT_6A(256'hB49C3C199556CF00F36AAD9800332AAD9C00E652D261FC336AB663FE334A9670),
    .INIT_6B(256'h8DB54B30FE19B5524700732552470039B594987F864AAB6380713556CC001C96),
    .INIT_6C(256'h19B556DCFFF8C955498FFF192D6B31FFE335A5271FF19B556CE0E0CCB52CCFFF),
    .INIT_6D(256'hD9C000E64AAA4CE0F07369569987FC736AAB663FFC66D55A63803CC9556CC7FF),
    .INIT_6E(256'hC0007336AAADB1C1E07334AAB6CE1FF0E65AAAD99E0078DB5A52660FE1CCB556),
    .INIT_6F(256'h7383FE0E324AD4A599C7FFE1CC96AA96CC700038CDB555A66780038CDAD52DB9),
    .INIT_70(256'hB5AAAD6D98C3F83F0E665A5552D33387FFF0E664A554A4DCE1FFF0E764AD5A93),
    .INIT_71(256'hB49999C3E0007E38CD92D4AAD4B66671F0003E18CDB6AD52B4999C7C001F1CCD),
    .INIT_72(256'h5554AD24D99CE3E07FFC078E33324B5AAAAB5A4D98C787FFFFC3CE66492B5556),
    .INIT_73(256'h3C3E00FFF007C38E73336496B52AAAB5696D93339C783FC00FF078C6766DB4AD),
    .INIT_74(256'h4B64993319CE38783F001FE001F87871CC666CDB696B56AAAAA56B4B64D998C6),
    .INIT_75(256'hCC631C70E1F07E00FFFFFFF007E0F0E1C63198C99B24925A52B55AAAAA552B5A),
    .INIT_76(256'hC718E731998C999B26C92496D2D695AB54AAAAAAAAAB54AD4B5B496D9264CCCC),
    .INIT_77(256'h99998CCE6318E71C71C78F0F0F87E07E01FF80000FFF00003FF01FC1F07878F1),
    .INIT_78(256'h955556AAAAAAAA955552AAD56AB56A56A5294B4B4B4B6D24924936C9B264CC99),
    .INIT_79(256'h99366C9B26C9B64936DB6DB6DB492DA4B4B694B4B5AD6B5AD4AD5AB54AA556AA),
    .INIT_7A(256'h1C71C71C738E31CE31CE718C6339CC67319CCC6673333199999999B3332664CD),
    .INIT_7B(256'h07E07C0F83F07C1F07C1E0F87C3E1E1F0F0F0F0F0E1E1C3C78F1E3871E38F1C7),
    .INIT_7C(256'hC0001FFF8001FFF000FFE003FF003FF007FC03FC03FC03F807F01F80FC07E07E),
    .INIT_7D(256'hCE1E0FF0003FC1E1C7198C99B249696B5AA5554AAD5556AB56AD6A5296B4A5A4),
    .INIT_7E(256'hA556A4999C3F00FC38CD92955552DB2638E07FFE070C666DB4A55556A5A4D999),
    .INIT_7F(256'h5B663E07E336D555B231FFFE3992D552D9187FFF8E66D2AAB4931C3FFF871934),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_0
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_0_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_0_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_0_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_0_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_0_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_0_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_0_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_0_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_0_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_0_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_0_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_0_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_0_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_0_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_1" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "1" *) 
  (* ram_slice_end = "1" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFC0000000000003FFFFFFFFFFF80000000001FFFFFFFFFC000000),
    .INIT_01(256'h000000000000000000000001FFFFFFFFFFFFFFFFFFFF800000000000000003FF),
    .INIT_02(256'h0000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'h00000000000000000000000000000000000000000000000000003FFFFFFFFFFF),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'hFFFFFFFFFFF00000000000000000000000000000000000000000000000000000),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'h000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'h03FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000),
    .INIT_17(256'hFF000000000000000007FFFFFFFFFFFFFFFFFFFE000000000000000000000000),
    .INIT_18(256'h000000FFFFFFFFFE00000000007FFFFFFFFFFF0000000000000FFFFFFFFFFFFF),
    .INIT_19(256'h001FFF0007FFC0007FFF0000FFFF00003FFFFC00001FFFFFC0000007FFFFFFF0),
    .INIT_1A(256'hF03F81FC07E03F80FF01FE03FC03FC03FE01FF007FE00FFC00FFC007FF001FFE),
    .INIT_1B(256'h0F1E1C3C38787878783C3C3E1F0F07C3E0F07C1F07C1F03E0FC1F83F03F03F03),
    .INIT_1C(256'h6633198CC67318CE739CE738C739C738E71C738E38E38E3C71C78E1C78E1C387),
    .INIT_1D(256'hB696D25B6924924924926D926D9364D9326CD99336664CCCC999999999CCCCC6),
    .INIT_1E(256'h6AD52A955AAA95555556AA95555554AAAD552A956A952B52B5294A529694B4B4),
    .INIT_1F(256'h3C3C78E1C638E739CC67333333333266CD93649B6DB6DB492D2DAD2D694AD6A5),
    .INIT_20(256'hB264CD99998CC6318E38E3878F87C1F01FC01FFFC0000003FFF801FC07E0F83C),
    .INIT_21(256'h83E1E38738C673333366C936DA4B4A5A952AD5554AB5555AAD5A94B5A4B6D26D),
    .INIT_22(256'h4D924B6B5AD56AAAAAA552B5A5B4936CD9999CC638E3C3C1F80FFFC003FFF00F),
    .INIT_23(256'h999936DB4B5AA555552AD6B4924D99998C71C3C1F800FFF800FC1E1C71CC6666),
    .INIT_24(256'h0F03FFFFF81E1C7319B36DA52B555554AD2D26C998CE38F07E000000FE1E1C73),
    .INIT_25(256'h1CE64DB6B56AAB56B6D93318E1E03FFF80F0E3199B24B4A95555A94B6C9998C7),
    .INIT_26(256'h0E319B25AD55552D24CCC70F800007C39CCC925AB5552B4B26CE71C1FE00FF07),
    .INIT_27(256'hCCC70FE03F8718D9252AAAB4B66671E03FE03C7332696AB4AB5B66663C3FFFFE),
    .INIT_28(256'h007C399925AAAA524CCE1F000F87333695AAD4B6CC63C03C03C73324B55554B6),
    .INIT_29(256'h003C31925AB6A96CCC783FE0718D96B554A5931C7C003E3999252AA94933387C),
    .INIT_2A(256'h38FFFFC399969555AD998E07F81C626D2AAAD24CE3C00078E66D2B56A59338F8),
    .INIT_2B(256'h52C98C3E00F8733695AB5249CE1FFFE1CCC96AAAB4999C3FFFC39992D5A54926),
    .INIT_2C(256'h954ADB673C1FFC1E736DA952A4999C3F87F0E66DA5554B6CC70FFFF0E664B569),
    .INIT_2D(256'h52DB338E07FF078C6C96A552B6D99C3C000F8E664B56AD4B6663C1FF81C666DA),
    .INIT_2E(256'h0E319B2DA95552B6D999C7807E01E39C9B6952B54B6CCC71F800FC39C9B69555),
    .INIT_2F(256'h9992494AD55552B4924CCC71C1FE007F878E666492952AA54B4D9398F0FF00FE),
    .INIT_30(256'hF81F0F1E31CE66664DB6DA52956AAAA956A5B49B266631C783E000000FC3C739),
    .INIT_31(256'h6D24924924936DB24DB26CD9B326666667339CE71C71C387C3F03FE000000007),
    .INIT_32(256'hC1FE000FFFC001FE07E1F0E1C38E718C6339998CC9999B3264C9B364DB26C937),
    .INIT_33(256'h6CCE638783FF00FFC0F1E319999B6DB4AD4AAAAAAD5AD2DB6D9333319C63C787),
    .INIT_34(256'h38E1FE07FC38E666DB5AAAAA94B64CCE38F800000F871CCCD925A9555552B492),
    .INIT_35(256'h365A5555ADB331C3FFFF87199B6D4AA95A4D98E3C00007C739B24A55554A4933),
    .INIT_36(256'h992D555693738F801F8E664B5555A4998F07FF079CD9295556926670FE03F0E3),
    .INIT_37(256'h0001E39B695552C99C3F03F1CCCB5AAD49338F0003E3336B5556933387FFFC39),
    .INIT_38(256'h6AAA964E703F81C66DA952B6CC703F81C66DAD54A4D8E3FFFC399B4AAAB499CF),
    .INIT_39(256'hE781F03CEC92AAA5B3387FFF0C6494AA524CE3E01F0CCDA5552DB38F000F1CC9),
    .INIT_3A(256'h73369555A4CC78000719925555A6CE3E007C666D6AAD24CE1FFFE399B4AAA926),
    .INIT_3B(256'h6D98E0FF83CCC96AAA5B231F001E199252A949338F803E399256AD493187FFFC),
    .INIT_3C(256'hC66495AB524CE3C001E399254A569338F000F8C4DA5556926387FFE1CCDB5AA5),
    .INIT_3D(256'h663C00038E6DA5554B6671F003E399252AB5B6670FFFE1C6C96AAA5B271E0003),
    .INIT_3E(256'hB5AAA526670FFFE1C6496AAAD266387FF871992D555A498E1FFFC399929556B6),
    .INIT_3F(256'h0001E39369555A4D8E1FFFC399B6B55292671F003E399B4AAA96D9C70000F199),
    .INIT_40(256'h956B6CCE1FFF871925AAA96C8C7C003C7325A94A92671E000F1CC92B56A4998F),
    .INIT_41(256'hFFFF86324AD5A92671F007C7324A55292661E003E31369555A4CCF07FC1C66DA),
    .INIT_42(256'h925554B6671FFFE1CC92D55AD998F801F1CD96AAA92663800078CC96AAA5B338),
    .INIT_43(256'h4CE3C003C736D2AA96CCC3E01F1CC92954A498C3FFF87336955524DCF03E079D),
    .INIT_44(256'hCE64B5554B6670FFFF1C6C94AAD6D98E07F038CDB52A56D98E07F039C9A5555A),
    .INIT_45(256'h70FFFF873325AAAB5B331F0003C7324AD56B4CCE3F03F0E64D2AAA5B671E0003),
    .INIT_46(256'h1C3F01FC399925AAAA526CE783FF83C66496AAAB4999C7E007C73B25AAAAD266),
    .INIT_47(256'h32494AAAA94936738F80000F1C66C96A554ADB666387FFFF0E3336D6AAA969B3),
    .INIT_48(256'h24B52AAAAA56926CCCE387C000007C71CCC9B4A555556B6D999C70FF81FE1C73),
    .INIT_49(256'h878F18E6333326DB6D2D6AD555554AD4B6DB6666631E3C0FFC03FF078719CCD9),
    .INIT_4A(256'hB24D936C9B364C993366664CC6667318C639C70E1C3E1F81FE000FFFC001FE0F),
    .INIT_4B(256'hFE000007FF01F83C1C3C71C639CC663333332664C9B26C936D92492492492493),
    .INIT_4C(256'h38E3C1FC0003FC1E38E733364925AD4AAAAAAA54A5A49364CCCC631C70F07C07),
    .INIT_4D(256'h7FFFE0F1CC4C92D6AAAAD4964CCC71E0FFFFF0787399936D2956AA55292D9333),
    .INIT_4E(256'hAAD693631C3FFFF0719992D6AAAD69367387C0003E18CCC92956B54A49B31870),
    .INIT_4F(256'hA4CCC7C0003C7336D6AAA524CC71FFFFC38CC9295556926E61E0000F8E66DB52),
    .INIT_50(256'hAAD2CCC707FE071992D6AB5A4CC70FFFC3CCCDAD554A4D9C780003C7364B5554),
    .INIT_51(256'h26D6AAB4B331C0FF038CC92D5569666383FF079C9B4AAA96D98E1FFFC38CDB5A),
    .INIT_52(256'hF1C66DA5554B6CCE3F007C399B6A556B6C8C7800078CCDB52AD49331C3FFE0E7),
    .INIT_53(256'hE1C66496AAAA5B33387FFFE1CE4DA5555693631F0001E399B6B555A5931C3FFF),
    .INIT_54(256'h6D6AAA52D999C3E0007C719B2D2AAAD6D998E1FFFF8719934AD56A5B331C3FFF),
    .INIT_55(256'h3FFF83C73336DA9555296D998E3E0000F863336D2B552A493631E1FFFE0F399B),
    .INIT_56(256'hF07FFFF03C731936D2955554A5B26671C3F0003F0E399934B52AA95A49B31C78),
    .INIT_57(256'h39CCC9B6D29552955AD2C93331CF0F800000F878C666C9694AAAAA52D264C670),
    .INIT_58(256'h8C78783FE0007FC0F1E318CC9924B4A55AAA954A5B6D9B318C787C03FFE01F0E),
    .INIT_59(256'hCCCC631C387C1FE0000007F03C3C739CCCCD93692D6AD5555556A52D24993331),
    .INIT_5A(256'hFC01FFFFFFF803F07C3C78E739CCCCD9B24925A52952AA955AAAD5AD2D249264),
    .INIT_5B(256'h92492DA5AD6B52AD55AAAAAAAB552AD4A52D25B6DB26CD99999CE738E3C787C1),
    .INIT_5C(256'h3C1F03F01FF001FFFFFFFFFFF800FF01F83E1F0E1C38E39C63399CCCCC99B26C),
    .INIT_5D(256'h52B5AD6B4B5A4B496DA4924DB64DB366CD9999999998CC6338C738E38E1C3C3C),
    .INIT_5E(256'h2D2D296B4A52B5A95A952A55AAD54AAB55552AAAAAAAAAAA95554AAB552AD5AB),
    .INIT_5F(256'h199998CCCCCCCCD9999B33664CD9B364C9B26D926D9249B6DB6DA492DB49692D),
    .INIT_60(256'h1E3870E3C70E3871C71C71C71C71CE38C738E738C6318C6318CE63398CC66733),
    .INIT_61(256'h1F81F81F83F03E0FC1F07E0F83C1F0783E1F0F8783C3C1E1E1E1E1C3C3C7870E),
    .INIT_62(256'hF8007FF003FF801FF801FF007FC01FE00FF00FF00FE01FC07F01FC07E03F01F8),
    .INIT_63(256'hFFFFE0000001FFFFFF000003FFFFC00007FFFC0003FFFC0007FFE000FFF8007F),
    .INIT_64(256'h800000001FF03F0F870E38E39CE73399999993366CD936C936DB2492492492DB),
    .INIT_65(256'h738F0FC000001F078E31999364B695AA55555AA5296DB6C99999CE31E3C3E07F),
    .INIT_66(256'hFC03FC3C6726CB4A9552A5249999C787F801FE0E38CCC924B52AAAAD4A492666),
    .INIT_67(256'hAAA5B64E70FC007E38CCDB4AB52A5B64E71E01F8078E666DB52AAA56D36631C1),
    .INIT_68(256'h6D998E07FE0F199B4AD5AB4999C7C000F0E66DB52A95A4D8C783FF81C7336D2A),
    .INIT_69(256'h5AB4999C3FFFC38CDB4AAA96D99C3F87F0E664952A56DB39E0FFE0F39B6D4AA5),
    .INIT_6A(256'h924A96AD26670FFFF0E664B5555A4CCE1FFFE1CE492B56A5B3387C01F0C64D2A),
    .INIT_6B(256'h7C732695AB52D99C78000F1CC92D5552D918E07F81C666D6AAA5A6670FFFFC71),
    .INIT_6C(256'hF873324A5552926671F000F8E32694AAB5A6C6381FF078CCDA55B5692630F000),
    .INIT_6D(256'hB4AAAAB493338F00F00F18CDB4AD56A5B33387C003E1CCC9295556926670F800),
    .INIT_6E(256'hFFFFF0F1999B6B54B55A593338F01FF01E3999B4B55552926C6387F01FC38CCD),
    .INIT_6F(256'h83FC01FE0E39CD934B52AAB56924CCE70F800007C38CCC92D2AAAAD6936631C1),
    .INIT_70(256'h8C6664DB4A56AAAA54B49366631C3C07FFF01E1C63326DB5AB555AB5B6C99CE3),
    .INIT_71(256'h38E1E1FC000001F83C71CC664D92D2D4AAAAAB5296DB366338E1E07FFFFF03C3),
    .INIT_72(256'h9998CE38E1E0FC007FFC007E0F0E38C66666C924B5AD52AAAA956B4B6DB26667),
    .INIT_73(256'hC03FFF000FFFC07E0F0F1C718CE6666CDB24B696B52A9555555AAD6B5B4926C9),
    .INIT_74(256'hD92DB496B4A56AD56AAAB54AAAAD52A5694B496DB24D9B3333398C73871E1F07),
    .INIT_75(256'hF07C1F80FE007FFF0000000FFFE00FE03E0F87C7871C71C6318CC66666CC9936),
    .INIT_76(256'h95AD4A5AD2D6D2D24B6DB6DB649B26CD99333333333398CE739C718E1C78F0F0),
    .INIT_77(256'hB4B4A5A5294A52B52B52A55AA552AAD554AAAAAAA555AAAAAAA5556AA552AD5A),
    .INIT_78(256'h8CCCCE666666664CCCC999B3266CD9326C9B26D926D924924924925B692DA5B4),
    .INIT_79(256'h870E1C78E1C78E38F1C71C71C738E39C738E738C739CE739CC63398CC6633199),
    .INIT_7A(256'h03F03F03F07E0FC1F03E0F83E0F83C1F0F83C3E1F0F0F07878787870F0E1E3C3),
    .INIT_7B(256'hFFE003FF800FFC00FFC01FF803FE01FF00FF00FF01FE03FC07F01F80FE07F03F),
    .INIT_7C(256'h3FFFFFFF8000000FFFFFE00000FFFFF00003FFFC0003FFF8000FFF8003FFE001),
    .INIT_7D(256'h0FE00FFFFFFFC01FC0F87C7871C718E7399CCCC664CCCD993264D9364D926C93),
    .INIT_7E(256'hC998C71E1FC00003F83C718CCCC9B69295AAD554AA56B4B6D936666739C71E1E),
    .INIT_7F(256'h6387C0001F0E3333696B5554AD24999CE1E07FFF81E1CE666DB6B56AAAD5ADA6),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_1
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_1_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_1_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_1_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_1_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_1_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_1_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_1_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_1_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_1_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_1_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_1_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_1_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_1_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_1_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_10" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "10" *) 
  (* ram_slice_end = "10" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC000000000000000000000000),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'h0000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFE000000000000000000000000000000000000000),
    .INIT_21(256'hFF80000000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'hFFFFFFFFE000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'h00000000007FFFFFFFFFFFFFFFFFF000000000000000000003FFFFFFFFFFFFFF),
    .INIT_24(256'hC0000000000007FFFFFFFFFFFFF000000000000003FFFFFFFFFFFFFFF8000000),
    .INIT_25(256'h0000000003FFFFFFFFFC0000000000FFFFFFFFFFE000000000007FFFFFFFFFFF),
    .INIT_26(256'h0000007FFFFFFE00000007FFFFFFF800000003FFFFFFFF000000000FFFFFFFFF),
    .INIT_27(256'h000007FFFFF000000FFFFFF8000003FFFFFE0000007FFFFFF0000001FFFFFFF0),
    .INIT_28(256'hFFC00007FFFF00001FFFFC00003FFFFC00001FFFFE00000FFFFFC00000FFFFFC),
    .INIT_29(256'h80007FFF0001FFFE0001FFFE0001FFFE0000FFFF80003FFFE0000FFFFC0000FF),
    .INIT_2A(256'hFF8003FFE000FFF8003FFE0007FFC000FFFC000FFFC000FFFC0007FFF0003FFF),
    .INIT_2B(256'hFC007FF001FFC003FF800FFF001FFE001FFC003FFC003FFC001FFE001FFF000F),
    .INIT_2C(256'h03FE007FE00FFC00FFC00FFC00FFC007FE007FE003FF001FF800FFE007FF001F),
    .INIT_2D(256'h07FC01FF00FF803FE00FF803FE00FF803FE00FFC01FF003FE00FFC01FF803FF0),
    .INIT_2E(256'hE01FE01FE01FE01FE00FF00FF007F807FC03FC01FE00FF007F803FC01FE00FF8),
    .INIT_2F(256'h3F807F00FF01FE01FC03FC07F807F80FF00FF00FE01FE01FE01FE01FE01FE01F),
    .INIT_30(256'h07F01FE03F807F01FE03FC07F00FE01FC03F80FF01FE03FC07F80FF01FE01FC0),
    .INIT_31(256'hFC07F80FE03FC07F01FE03F80FF01FC07F80FE03FC07F00FE03F807F01FE03F8),
    .INIT_32(256'h01FE03F807F01FE03F80FF01FC03F80FE01FC07F80FE03FC07F01FE03F80FF01),
    .INIT_33(256'hE03FC03F807F00FE01FC03F807F00FE01FC07F80FF01FC03F807F01FE03F807F),
    .INIT_34(256'h7F807F807F807F807F807F807F80FF00FF00FE01FE01FC03FC07F807F00FF01F),
    .INIT_35(256'h03FE00FF007FC03FE01FF00FF807FC03FC01FE01FF00FF00FF007F807F807F80),
    .INIT_36(256'h07FC00FFC01FF803FF007FE00FF803FF007FC01FF007FC01FF007FC01FF007F8),
    .INIT_37(256'h800FFE003FF800FFC007FF003FF800FFC00FFE007FE007FE007FE007FE007FE0),
    .INIT_38(256'h1FFF0007FFC003FFE000FFF000FFF8007FF8007FF8007FF000FFE001FFC007FF),
    .INIT_39(256'hFFF0000FFFF0000FFFE0003FFF8000FFFC0007FFE0007FFE0007FFC000FFF800),
    .INIT_3A(256'hFFFE00001FFFFE00001FFFFC00007FFFE00007FFFE00007FFFC0001FFFF0000F),
    .INIT_3B(256'h3FFFFFFE0000001FFFFFFC000001FFFFFF000000FFFFFF000003FFFFF000003F),
    .INIT_3C(256'h00000000000FFFFFFFFFFE0000000003FFFFFFFFC00000000FFFFFFFF0000000),
    .INIT_3D(256'h000000000003FFFFFFFFFFFFFFFFFFFFFC0000000000000001FFFFFFFFFFFFF8),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h7FFFFFFFFFFFFE0000000000000000FFFFFFFFFFFFFFFFFFFFFF000000000000),
    .INIT_40(256'h0000003FFFFFFFC00000000FFFFFFFFF0000000001FFFFFFFFFFC00000000000),
    .INIT_41(256'hF000003FFFFF000003FFFFFC000003FFFFFE000000FFFFFFE0000001FFFFFFF0),
    .INIT_42(256'hC0003FFFE0000FFFF80001FFFF80001FFFF80000FFFFE00001FFFFE00001FFFF),
    .INIT_43(256'h007FFC000FFF8001FFF8001FFF8000FFFC0007FFF0001FFFC0003FFFC0003FFF),
    .INIT_44(256'hFF800FFE001FFC003FF8007FF8007FF8007FFC003FFC001FFF000FFF8003FFE0),
    .INIT_45(256'h1FF801FF801FF801FF801FF801FFC00FFC007FF003FF800FFC007FF001FFC007),
    .INIT_46(256'h7F803FE00FF803FE00FF803FE00FF803FF007FC01FF803FF007FE00FFC00FF80),
    .INIT_47(256'h07F807F807F803FC03FC03FE01FE00FF00FF807FC03FE01FF00FF803FC01FF00),
    .INIT_48(256'hE03FC03F807F80FF00FE01FE01FC03FC03FC07F807F807F807F807F807F807F8),
    .INIT_49(256'hF807F01FE03F807F00FE03FC07F80FE01FC03F807F00FE01FC03F807F00FF01F),
    .INIT_4A(256'h03FC07F01FE03F80FF01FC07F80FE01FC07F00FE03FC07F01FE03F807F01FE03),
    .INIT_4B(256'h80FE01FC07F80FE03FC07F00FE03F807F01FC03F80FE01FC07F00FE03F807F01),
    .INIT_4C(256'hF00FE01FC03F807F00FE01FC03F80FF01FE03FC07F00FE01FC07F80FE01FC07F),
    .INIT_4D(256'h1FE01FE01FE01FE01FE01FE03FC03FC03F807F807F00FF01FE01FC03FC07F80F),
    .INIT_4E(256'h803FE01FF00FF807FC03FE01FF00FF007F807FC03FC03FE01FE01FE01FE01FE0),
    .INIT_4F(256'hC00FF801FF003FE00FFC01FF003FE00FF803FE00FF803FE00FF803FC01FF007F),
    .INIT_50(256'h1FFC007FE003FF801FFC00FFE007FE007FF003FF003FF003FF003FE007FE00FF),
    .INIT_51(256'h3FFC001FFE001FFF000FFF000FFF001FFE001FFC003FF800FFF001FFC007FF00),
    .INIT_52(256'h000FFFC0007FFF0003FFF0003FFF0003FFF0007FFE000FFF8003FFE000FFF800),
    .INIT_53(256'h03FFFF00003FFFE0000FFFF80003FFFE0001FFFE0001FFFE0001FFFC0007FFF8),
    .INIT_54(256'h000003FFFFF000003FFFFE00000FFFFF00000FFFFF00001FFFFC00007FFFF000),
    .INIT_55(256'hC0000001FFFFFFC0000007FFFFFE0000007FFFFF8000003FFFFFC000007FFFFF),
    .INIT_56(256'h000000003FFFFFFFFC00000000FFFFFFFF800000007FFFFFFE00000007FFFFFF),
    .INIT_57(256'h000000000007FFFFFFFFFFE00000000003FFFFFFFFFF0000000000FFFFFFFFFC),
    .INIT_58(256'hFFFFFF8000000000000000FFFFFFFFFFFFFFC00000000000007FFFFFFFFFFFF0),
    .INIT_59(256'h00000000000000FFFFFFFFFFFFFFFFFFFFC0000000000000000007FFFFFFFFFF),
    .INIT_5A(256'h00000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFE000000000),
    .INIT_5B(256'h000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800),
    .INIT_5C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000000000000),
    .INIT_5D(256'h000000000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'hFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000000000000000000000000000),
    .INIT_60(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_61(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_62(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_63(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_64(256'h7F01FE03F807F01FC03F80FF01FC07F80FE03FC07F01FE03F80FF01FC07F80FE),
    .INIT_65(256'h0FE01FE03FC07F80FF01FE03FC07F00FE01FC03F80FF01FE03F807F01FE03F80),
    .INIT_66(256'hE01FE01FE01FE01FE01FE01FC03FC03FC07F807F80FF00FE01FE03FC03F807F0),
    .INIT_67(256'h7FC01FE00FF007F803FC01FE00FF00FF807F803FC03FC01FE01FE01FE01FE01F),
    .INIT_68(256'h3FF007FE00FFC01FF003FE00FFC01FF007FC01FF007FC01FF007FC03FE00FF80),
    .INIT_69(256'hE003FF801FFC007FE003FF001FF801FF800FFC00FFC00FFC00FFC01FF801FF00),
    .INIT_6A(256'hC003FFE001FFE000FFF000FFF000FFE001FFE003FFC007FF000FFE003FF800FF),
    .INIT_6B(256'hFFF0003FFF8000FFFC000FFFC000FFFC000FFF8001FFF0007FFC001FFF0007FF),
    .INIT_6C(256'hFC0000FFFFC0001FFFF00007FFFC0001FFFE0001FFFE0001FFFE0003FFF80007),
    .INIT_6D(256'hFFFFFC00000FFFFFC00001FFFFE00000FFFFF00000FFFFE00003FFFF80000FFF),
    .INIT_6E(256'h3FFFFFFE0000003FFFFFF8000001FFFFFF0000007FFFFFC000003FFFFF800000),
    .INIT_6F(256'hFFFFFFFFC000000003FFFFFFFF000000007FFFFFFF80000001FFFFFFF8000000),
    .INIT_70(256'hFFFFFFFFFFF800000000001FFFFFFFFFFC0000000000FFFFFFFFFF0000000003),
    .INIT_71(256'h0000007FFFFFFFFFFFFFFF000000000000003FFFFFFFFFFFFF8000000000000F),
    .INIT_72(256'hFFFFFFFFFFFFFF000000000000000000003FFFFFFFFFFFFFFFFFF80000000000),
    .INIT_73(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000001FFFFFFFFF),
    .INIT_74(256'hFFFFFFFFFFFFFFFFFFFFFFFC00000000000000000000000000000000000007FF),
    .INIT_75(256'h000000000000000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_76(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000000),
    .INIT_77(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_78(256'h000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'hFE01FC07F80FE01FC07F00FE03FC07F01FE03F807F01FC03F80FE01FC07F00FE),
    .INIT_7E(256'h1FC03FC07F80FF01FE03FC07F80FF01FE03F807F00FE03FC07F80FE01FC07F80),
    .INIT_7F(256'h807F807F807F807F807F807F807F00FF00FF01FE01FE03FC03F807F80FF00FE0),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_10
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_10_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_10_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_10_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_10_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_10_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_10_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_10_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_10_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_10_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_10_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_10_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_10_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_10_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_10_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_11" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "11" *) 
  (* ram_slice_end = "11" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'h0000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000000),
    .INIT_22(256'h000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000000),
    .INIT_24(256'h00000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000),
    .INIT_25(256'hFFFFFFFFFC00000000000000000000FFFFFFFFFFFFFFFFFFFFFF800000000000),
    .INIT_26(256'hFFFFFF8000000000000007FFFFFFFFFFFFFFFC00000000000000000FFFFFFFFF),
    .INIT_27(256'h000007FFFFFFFFFFF0000000000003FFFFFFFFFFFF80000000000001FFFFFFFF),
    .INIT_28(256'h00000007FFFFFFFFE0000000003FFFFFFFFFE0000000000FFFFFFFFFFF000000),
    .INIT_29(256'h00007FFFFFFE00000001FFFFFFFE00000000FFFFFFFFC00000000FFFFFFFFF00),
    .INIT_2A(256'hFFFFFC000000FFFFFFC0000007FFFFFF0000000FFFFFFF00000007FFFFFFC000),
    .INIT_2B(256'h00007FFFFE000003FFFFF000001FFFFFE000003FFFFFC000001FFFFFE000000F),
    .INIT_2C(256'hFC00007FFFF00000FFFFF00000FFFFF800007FFFFC00001FFFFF000007FFFFE0),
    .INIT_2D(256'hF80001FFFF00003FFFF00003FFFF00003FFFF00001FFFFC0000FFFFE00003FFF),
    .INIT_2E(256'h001FFFE0001FFFE0000FFFF00007FFF80003FFFE0000FFFF80003FFFE0000FFF),
    .INIT_2F(256'hC0007FFF0001FFFE0003FFF80007FFF0000FFFF0001FFFE0001FFFE0001FFFE0),
    .INIT_30(256'h07FFE0003FFF8001FFFC0007FFF0001FFFC000FFFE0003FFF8000FFFE0001FFF),
    .INIT_31(256'hFFF8000FFFC0007FFE0003FFF0001FFF8000FFFC0007FFF0003FFF8001FFFC00),
    .INIT_32(256'hFE0003FFF8001FFFC000FFFE0003FFF0001FFF8000FFFC0007FFE0003FFF0001),
    .INIT_33(256'h003FFFC0007FFF0001FFFC0007FFF0001FFF8000FFFE0003FFF8001FFFC0007F),
    .INIT_34(256'h80007FFF80007FFF80007FFF8000FFFF0000FFFE0001FFFC0007FFF8000FFFE0),
    .INIT_35(256'h03FFFF00007FFFC0001FFFF00007FFFC0001FFFE0000FFFF00007FFF80007FFF),
    .INIT_36(256'h07FFFF00001FFFFC00007FFFF00003FFFF80001FFFF80001FFFF80001FFFF800),
    .INIT_37(256'h000FFFFFC00000FFFFF800003FFFFF00000FFFFF800007FFFF800007FFFF8000),
    .INIT_38(256'hE0000007FFFFFC000000FFFFFF0000007FFFFF8000007FFFFF000001FFFFF800),
    .INIT_39(256'hFFFFFFF00000000FFFFFFFC0000000FFFFFFF80000007FFFFFF8000000FFFFFF),
    .INIT_3A(256'h000000001FFFFFFFFFE0000000007FFFFFFFF8000000007FFFFFFFE00000000F),
    .INIT_3B(256'h3FFFFFFFFFFFFFE0000000000001FFFFFFFFFFFF000000000003FFFFFFFFFFC0),
    .INIT_3C(256'hFFFFFFFFFFF000000000000000000003FFFFFFFFFFFFFFFFF000000000000000),
    .INIT_3D(256'hFFFFFFFFFFFC00000000000000000000000000000000000001FFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFE00000000000000000000000000000000000000FFFFFFFFFFFF),
    .INIT_40(256'h000000000000003FFFFFFFFFFFFFFFFF000000000000000000003FFFFFFFFFFF),
    .INIT_41(256'h0FFFFFFFFFFF000000000003FFFFFFFFFFFE0000000000001FFFFFFFFFFFFFF0),
    .INIT_42(256'hC00000001FFFFFFFF8000000007FFFFFFFF8000000001FFFFFFFFFE000000000),
    .INIT_43(256'hFFFFFC0000007FFFFFF80000007FFFFFFC0000000FFFFFFFC00000003FFFFFFF),
    .INIT_44(256'h007FFFFE000003FFFFF8000007FFFFF8000003FFFFFC000000FFFFFF8000001F),
    .INIT_45(256'h0007FFFF800007FFFF800007FFFFC00003FFFFF000007FFFFC00000FFFFFC000),
    .INIT_46(256'h007FFFE00007FFFE00007FFFE00007FFFF00003FFFF80000FFFFE00003FFFF80),
    .INIT_47(256'hFFF80007FFF80003FFFC0001FFFE0000FFFF80003FFFE0000FFFF80003FFFF00),
    .INIT_48(256'h1FFFC0007FFF8000FFFE0001FFFC0003FFFC0007FFF80007FFF80007FFF80007),
    .INIT_49(256'hF8000FFFE0007FFF0001FFFC0007FFE0003FFF8000FFFE0003FFF8000FFFF000),
    .INIT_4A(256'h0003FFF0001FFF8000FFFC0007FFE0003FFF0001FFFC000FFFE0007FFF0001FF),
    .INIT_4B(256'hFF0001FFF8000FFFC0007FFF0003FFF8001FFFC000FFFE0007FFF0003FFF8001),
    .INIT_4C(256'h000FFFE0003FFF8000FFFE0003FFF0001FFFC0007FFF0001FFF8000FFFE0007F),
    .INIT_4D(256'hE0001FFFE0001FFFE0001FFFC0003FFFC0007FFF8000FFFE0001FFFC0007FFF0),
    .INIT_4E(256'h003FFFE0000FFFF80003FFFE0000FFFF80007FFFC0003FFFE0001FFFE0001FFF),
    .INIT_4F(256'h000FFFFE00003FFFF00001FFFFC0000FFFFC0000FFFFC0000FFFFC0001FFFF80),
    .INIT_50(256'hE000007FFFFC00001FFFFF000007FFFF800003FFFFC00003FFFFC00007FFFF00),
    .INIT_51(256'h3FFFFFE000001FFFFFF000000FFFFFE000001FFFFFC00000FFFFFE000007FFFF),
    .INIT_52(256'hFFF00000007FFFFFFC0000003FFFFFFC0000007FFFFFF0000003FFFFFF000000),
    .INIT_53(256'hFC000000003FFFFFFFF000000003FFFFFFFE00000001FFFFFFFE00000007FFFF),
    .INIT_54(256'hFFFFFC00000000003FFFFFFFFFF0000000000FFFFFFFFFE0000000007FFFFFFF),
    .INIT_55(256'h00000001FFFFFFFFFFFFF80000000000007FFFFFFFFFFFC000000000007FFFFF),
    .INIT_56(256'h000000003FFFFFFFFFFFFFFFFF00000000000000007FFFFFFFFFFFFFF8000000),
    .INIT_57(256'hFFFFFFFFFFF80000000000000000000003FFFFFFFFFFFFFFFFFFFF0000000000),
    .INIT_58(256'hFFFFFFFFFFFFFFFFFFFFFF00000000000000000000000000007FFFFFFFFFFFFF),
    .INIT_59(256'hFFFFFFFFFFFFFF0000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_5A(256'h00000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5B(256'hFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000000000000000),
    .INIT_5C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5D(256'h000000000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h00FFFE0007FFF0003FFF8000FFFC0007FFE0003FFF0001FFF8000FFFC0007FFE),
    .INIT_65(256'hFFE0001FFFC0007FFF0001FFFC000FFFE0003FFF8000FFFE0007FFF0001FFF80),
    .INIT_66(256'h1FFFE0001FFFE0001FFFE0003FFFC0003FFF80007FFF0001FFFE0003FFF8000F),
    .INIT_67(256'hFFC0001FFFF00007FFFC0001FFFF00007FFF80003FFFC0001FFFE0001FFFE000),
    .INIT_68(256'hFFF00001FFFFC0000FFFFE00003FFFF00003FFFF00003FFFF00003FFFE00007F),
    .INIT_69(256'h1FFFFF800003FFFFE00000FFFFF800007FFFFC00003FFFFC00003FFFF80000FF),
    .INIT_6A(256'hC000001FFFFFE000000FFFFFF000001FFFFFE000003FFFFF000001FFFFF80000),
    .INIT_6B(256'h000FFFFFFF80000003FFFFFFC0000003FFFFFF8000000FFFFFFC000000FFFFFF),
    .INIT_6C(256'h03FFFFFFFFC00000000FFFFFFFFC00000001FFFFFFFE00000001FFFFFFF80000),
    .INIT_6D(256'h000003FFFFFFFFFFC0000000001FFFFFFFFFF0000000001FFFFFFFFF80000000),
    .INIT_6E(256'hFFFFFFFE00000000000007FFFFFFFFFFFF0000000000003FFFFFFFFFFF800000),
    .INIT_6F(256'hFFFFFFFFC00000000000000000FFFFFFFFFFFFFFFF8000000000000007FFFFFF),
    .INIT_70(256'h000000000007FFFFFFFFFFFFFFFFFFFFFC00000000000000000000FFFFFFFFFF),
    .INIT_71(256'h0000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000),
    .INIT_72(256'h00000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_73(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000),
    .INIT_74(256'h000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000000),
    .INIT_77(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_79(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7D(256'h01FFFC0007FFE0003FFF0001FFFC000FFFE0007FFF0003FFF8001FFFC000FFFE),
    .INIT_7E(256'hFFC0003FFF8000FFFE0003FFF8000FFFE0007FFF0001FFFC0007FFE0003FFF80),
    .INIT_7F(256'h7FFF80007FFF80007FFF80007FFF0000FFFF0001FFFE0003FFF80007FFF0001F),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_11
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_11_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_11_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_11_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_11_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_11_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_11_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_11_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_11_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_11_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_11_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_11_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_11_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_11_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_11_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_12" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "12" *) 
  (* ram_slice_end = "12" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000000000000000),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'h000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'hFFFFFFFFFF800000000000000000000000000000000000000000000000000000),
    .INIT_24(256'h00000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000000000000000000000),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFF8000000000000000000000000000000000FFFFFFFFF),
    .INIT_27(256'h000007FFFFFFFFFFFFFFFFFFFFFFFC00000000000000000000000001FFFFFFFF),
    .INIT_28(256'hFFFFFFF80000000000000000003FFFFFFFFFFFFFFFFFFFF00000000000000000),
    .INIT_29(256'hFFFF8000000000000001FFFFFFFFFFFFFFFF00000000000000000FFFFFFFFFFF),
    .INIT_2A(256'hFFFFFFFFFFFF00000000000007FFFFFFFFFFFFF000000000000007FFFFFFFFFF),
    .INIT_2B(256'hFFFF800000000003FFFFFFFFFFE000000000003FFFFFFFFFFFE000000000000F),
    .INIT_2C(256'hFFFFFF8000000000FFFFFFFFFF00000000007FFFFFFFFFE00000000007FFFFFF),
    .INIT_2D(256'h000001FFFFFFFFC000000003FFFFFFFFC000000001FFFFFFFFF0000000003FFF),
    .INIT_2E(256'h001FFFFFFFE00000000FFFFFFFF800000003FFFFFFFF000000003FFFFFFFF000),
    .INIT_2F(256'h00007FFFFFFE00000003FFFFFFF80000000FFFFFFFE00000001FFFFFFFE00000),
    .INIT_30(256'hF80000003FFFFFFE00000007FFFFFFE0000000FFFFFFFC0000000FFFFFFFE000),
    .INIT_31(256'hFFFFFFF00000007FFFFFFC0000001FFFFFFF00000007FFFFFFC0000001FFFFFF),
    .INIT_32(256'h000003FFFFFFE0000000FFFFFFFC0000001FFFFFFF00000007FFFFFFC0000001),
    .INIT_33(256'hFFC00000007FFFFFFE00000007FFFFFFE0000000FFFFFFFC0000001FFFFFFF80),
    .INIT_34(256'hFFFF800000007FFFFFFF80000000FFFFFFFF00000001FFFFFFF80000000FFFFF),
    .INIT_35(256'hFC000000007FFFFFFFE000000007FFFFFFFE00000000FFFFFFFF800000007FFF),
    .INIT_36(256'h07FFFFFFFFE0000000007FFFFFFFFC000000001FFFFFFFFE000000001FFFFFFF),
    .INIT_37(256'h000FFFFFFFFFFF00000000003FFFFFFFFFF00000000007FFFFFFFFF800000000),
    .INIT_38(256'hFFFFFFF8000000000000FFFFFFFFFFFF8000000000007FFFFFFFFFFE00000000),
    .INIT_39(256'hFFFFFFFFFFFFFFF000000000000000FFFFFFFFFFFFFF80000000000000FFFFFF),
    .INIT_3A(256'hFFFFFFFFE00000000000000000007FFFFFFFFFFFFFFFFF80000000000000000F),
    .INIT_3B(256'h3FFFFFFFFFFFFFFFFFFFFFFFFFFE000000000000000000000003FFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000000000000000000000000),
    .INIT_3D(256'h00000000000000000000000000000000000000000000000001FFFFFFFFFFFFFF),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'hFFFFFFFFFFFFFE00000000000000000000000000000000000000000000000000),
    .INIT_40(256'h00000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFF000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFF0),
    .INIT_42(256'hC00000000000000007FFFFFFFFFFFFFFFFF80000000000000000001FFFFFFFFF),
    .INIT_43(256'hFFFFFC00000000000007FFFFFFFFFFFFFC000000000000003FFFFFFFFFFFFFFF),
    .INIT_44(256'h00000001FFFFFFFFFFF8000000000007FFFFFFFFFFFC0000000000007FFFFFFF),
    .INIT_45(256'h000000007FFFFFFFFF80000000003FFFFFFFFFF00000000003FFFFFFFFFFC000),
    .INIT_46(256'hFFFFFFE000000001FFFFFFFFE000000000FFFFFFFFF8000000001FFFFFFFFF80),
    .INIT_47(256'hFFF800000007FFFFFFFC00000001FFFFFFFF800000001FFFFFFFF800000000FF),
    .INIT_48(256'hFFFFC00000007FFFFFFE00000003FFFFFFFC00000007FFFFFFF800000007FFFF),
    .INIT_49(256'h07FFFFFFE0000000FFFFFFFC0000001FFFFFFF80000001FFFFFFF80000000FFF),
    .INIT_4A(256'h0000000FFFFFFF80000003FFFFFFE0000000FFFFFFFC0000001FFFFFFF000000),
    .INIT_4B(256'h000001FFFFFFF00000007FFFFFFC0000001FFFFFFF00000007FFFFFFC0000001),
    .INIT_4C(256'hFFF00000003FFFFFFF00000003FFFFFFE00000007FFFFFFE0000000FFFFFFF80),
    .INIT_4D(256'hFFFFE00000001FFFFFFFE00000003FFFFFFF80000000FFFFFFFE00000007FFFF),
    .INIT_4E(256'hFFC00000000FFFFFFFFC00000000FFFFFFFF800000003FFFFFFFE00000001FFF),
    .INIT_4F(256'h000FFFFFFFFFC000000001FFFFFFFFF000000000FFFFFFFFF000000001FFFFFF),
    .INIT_50(256'h0000007FFFFFFFFFE00000000007FFFFFFFFFC0000000003FFFFFFFFF8000000),
    .INIT_51(256'h3FFFFFFFFFFFE000000000000FFFFFFFFFFFE00000000000FFFFFFFFFFF80000),
    .INIT_52(256'h00000000007FFFFFFFFFFFFFC00000000000007FFFFFFFFFFFFC000000000000),
    .INIT_53(256'h00000000003FFFFFFFFFFFFFFFFC0000000000000001FFFFFFFFFFFFFFF80000),
    .INIT_54(256'hFFFFFFFFFFFFFFFFC00000000000000000000FFFFFFFFFFFFFFFFFFF80000000),
    .INIT_55(256'h00000001FFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000000000007FFFFF),
    .INIT_56(256'h000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000000000),
    .INIT_57(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC000000000000000000000000000000),
    .INIT_58(256'h000000000000000000000000000000000000000000000000007FFFFFFFFFFFFF),
    .INIT_59(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_5A(256'h00000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE0000000000000000000000000000000),
    .INIT_5E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_60(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_61(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_62(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_63(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_64(256'hFFFFFE0000000FFFFFFF80000003FFFFFFE0000000FFFFFFF80000003FFFFFFE),
    .INIT_65(256'h001FFFFFFFC0000000FFFFFFFC0000001FFFFFFF80000001FFFFFFF00000007F),
    .INIT_66(256'h00001FFFFFFFE00000001FFFFFFFC00000007FFFFFFF00000001FFFFFFF80000),
    .INIT_67(256'h003FFFFFFFF000000003FFFFFFFF000000007FFFFFFFC00000001FFFFFFFE000),
    .INIT_68(256'hFFF0000000003FFFFFFFFE000000000FFFFFFFFF000000000FFFFFFFFE000000),
    .INIT_69(256'hFFFFFF80000000001FFFFFFFFFF80000000003FFFFFFFFFC0000000007FFFFFF),
    .INIT_6A(256'hC000000000001FFFFFFFFFFFF000000000001FFFFFFFFFFF000000000007FFFF),
    .INIT_6B(256'hFFFFFFFFFF800000000000003FFFFFFFFFFFFF80000000000003FFFFFFFFFFFF),
    .INIT_6C(256'hFFFFFFFFFFC00000000000000003FFFFFFFFFFFFFFFE0000000000000007FFFF),
    .INIT_6D(256'h00000000000000003FFFFFFFFFFFFFFFFFFFF00000000000000000007FFFFFFF),
    .INIT_6E(256'hFFFFFFFE00000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF800000),
    .INIT_6F(256'hFFFFFFFFC0000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFF),
    .INIT_70(256'h0000000000000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_71(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000),
    .INIT_72(256'h000000000000000000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_73(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000),
    .INIT_74(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_75(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_76(256'h000000000000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'hFFFFFC0000001FFFFFFF00000003FFFFFFE0000000FFFFFFF80000003FFFFFFE),
    .INIT_7E(256'h003FFFFFFF80000001FFFFFFF80000001FFFFFFF00000003FFFFFFE00000007F),
    .INIT_7F(256'h00007FFFFFFF800000007FFFFFFF00000000FFFFFFFE00000007FFFFFFF00000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_12
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_12_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_12_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_12_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_12_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_12_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_12_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_12_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_12_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_12_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_12_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_12_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_12_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_12_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_12_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_13" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "13" *) 
  (* ram_slice_end = "13" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'h000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_24(256'hFFFFFFFFFFFFF800000000000000000000000000000000000000000000000000),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'h0000000000000000000000000000000000000000000000000000000FFFFFFFFF),
    .INIT_27(256'h000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000),
    .INIT_28(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000000000000),
    .INIT_29(256'hFFFFFFFFFFFFFFFFFFFE000000000000000000000000000000000FFFFFFFFFFF),
    .INIT_2A(256'hFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000007FFFFFFFFFF),
    .INIT_2B(256'h0000000000000003FFFFFFFFFFFFFFFFFFFFFFC000000000000000000000000F),
    .INIT_2C(256'hFFFFFFFFFFFFFFFF000000000000000000007FFFFFFFFFFFFFFFFFFFF8000000),
    .INIT_2D(256'hFFFFFE000000000000000003FFFFFFFFFFFFFFFFFE0000000000000000003FFF),
    .INIT_2E(256'hFFE0000000000000000FFFFFFFFFFFFFFFFC00000000000000003FFFFFFFFFFF),
    .INIT_2F(256'hFFFF8000000000000003FFFFFFFFFFFFFFF0000000000000001FFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFC000000000000007FFFFFFFFFFFFFF000000000000000FFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFF800000000000001FFFFFFFFFFFFFF800000000000001FFFFFF),
    .INIT_32(256'h000003FFFFFFFFFFFFFF000000000000001FFFFFFFFFFFFFF800000000000001),
    .INIT_33(256'h00000000007FFFFFFFFFFFFFF800000000000000FFFFFFFFFFFFFFE000000000),
    .INIT_34(256'h0000000000007FFFFFFFFFFFFFFF0000000000000001FFFFFFFFFFFFFFF00000),
    .INIT_35(256'h00000000007FFFFFFFFFFFFFFFF80000000000000000FFFFFFFFFFFFFFFF8000),
    .INIT_36(256'h07FFFFFFFFFFFFFFFFFF8000000000000000001FFFFFFFFFFFFFFFFFE0000000),
    .INIT_37(256'hFFF0000000000000000000003FFFFFFFFFFFFFFFFFFFF8000000000000000000),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFF0000000000000000000000007FFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000000000000000FFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000000000F),
    .INIT_3B(256'hC000000000000000000000000000000000000000000000000003FFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'h00000000000000000000000000000000000000000000000001FFFFFFFFFFFFFF),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'hFFFFFFFFFFFFFE00000000000000000000000000000000000000000000000000),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFF000000000000000000000000000000000000000000000000000F),
    .INIT_42(256'hC00000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFC0000000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFF8000000000000000000000003FFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'h0000000000000000007FFFFFFFFFFFFFFFFFFFF0000000000000000000003FFF),
    .INIT_46(256'h0000001FFFFFFFFFFFFFFFFFE0000000000000000007FFFFFFFFFFFFFFFFFF80),
    .INIT_47(256'h0007FFFFFFFFFFFFFFFC00000000000000007FFFFFFFFFFFFFFFF80000000000),
    .INIT_48(256'h00003FFFFFFFFFFFFFFE0000000000000003FFFFFFFFFFFFFFF8000000000000),
    .INIT_49(256'h000000001FFFFFFFFFFFFFFC000000000000007FFFFFFFFFFFFFF80000000000),
    .INIT_4A(256'h000000000000007FFFFFFFFFFFFFE000000000000003FFFFFFFFFFFFFF000000),
    .INIT_4B(256'h000001FFFFFFFFFFFFFF800000000000001FFFFFFFFFFFFFF800000000000001),
    .INIT_4C(256'h00000000003FFFFFFFFFFFFFFC000000000000007FFFFFFFFFFFFFF000000000),
    .INIT_4D(256'h0000000000001FFFFFFFFFFFFFFFC000000000000000FFFFFFFFFFFFFFF80000),
    .INIT_4E(256'h00000000000FFFFFFFFFFFFFFFFF00000000000000003FFFFFFFFFFFFFFFE000),
    .INIT_4F(256'h000FFFFFFFFFFFFFFFFFFE000000000000000000FFFFFFFFFFFFFFFFFE000000),
    .INIT_50(256'hFFFFFF8000000000000000000007FFFFFFFFFFFFFFFFFFFC0000000000000000),
    .INIT_51(256'h3FFFFFFFFFFFFFFFFFFFFFFFF00000000000000000000000FFFFFFFFFFFFFFFF),
    .INIT_52(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000),
    .INIT_53(256'h00000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000000000),
    .INIT_54(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000000000),
    .INIT_55(256'hFFFFFFFE000000000000000000000000000000000000000000000000007FFFFF),
    .INIT_56(256'h000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000),
    .INIT_59(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5A(256'h00000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'hFFFFFE000000000000007FFFFFFFFFFFFFE000000000000007FFFFFFFFFFFFFE),
    .INIT_65(256'hFFFFFFFFFFC000000000000003FFFFFFFFFFFFFF800000000000000FFFFFFFFF),
    .INIT_66(256'hFFFFFFFFFFFFE0000000000000003FFFFFFFFFFFFFFF0000000000000007FFFF),
    .INIT_67(256'hFFFFFFFFFFF00000000000000000FFFFFFFFFFFFFFFFC0000000000000001FFF),
    .INIT_68(256'hFFF0000000000000000001FFFFFFFFFFFFFFFFFF000000000000000001FFFFFF),
    .INIT_69(256'h0000007FFFFFFFFFFFFFFFFFFFF800000000000000000003FFFFFFFFFFFFFFFF),
    .INIT_6A(256'hC000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFF0000000000000000),
    .INIT_6B(256'hFFFFFFFFFF80000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6C(256'hFFFFFFFFFFC000000000000000000000000000000001FFFFFFFFFFFFFFFFFFFF),
    .INIT_6D(256'h0000000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6E(256'h00000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000),
    .INIT_6F(256'hFFFFFFFFC0000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_71(256'h000000000000000000000000000000000000000000000000007FFFFFFFFFFFFF),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000),
    .INIT_74(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_75(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_76(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_77(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_79(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7D(256'hFFFFFC00000000000000FFFFFFFFFFFFFFE000000000000007FFFFFFFFFFFFFE),
    .INIT_7E(256'hFFFFFFFFFF8000000000000007FFFFFFFFFFFFFF000000000000001FFFFFFFFF),
    .INIT_7F(256'hFFFFFFFFFFFF8000000000000000FFFFFFFFFFFFFFFE000000000000000FFFFF),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_13
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_13_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_13_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_13_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_13_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_13_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_13_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_13_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_13_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_13_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_13_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_13_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_13_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_13_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_13_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_14" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "14" *) 
  (* ram_slice_end = "14" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'h000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_24(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_25(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000),
    .INIT_27(256'h000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_28(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_29(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000),
    .INIT_2A(256'h000000000000000000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_2B(256'h0000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0),
    .INIT_2C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000000000000000),
    .INIT_2D(256'hFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000000000003FFF),
    .INIT_2E(256'hFFFFFFFFFFFFFFFFFFF0000000000000000000000000000000003FFFFFFFFFFF),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000001FFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000000000FFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000000000000000001FFFFFF),
    .INIT_32(256'h000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000000000000000001),
    .INIT_33(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000000),
    .INIT_34(256'h0000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000000000),
    .INIT_35(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000000000000),
    .INIT_36(256'h07FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE0000000000000000000000000),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFC000000000000000000000000000000000000000),
    .INIT_38(256'h000000000000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000),
    .INIT_3A(256'h000000000000000000000000000000000000000000000000000000000000000F),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC000000000000),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'h00000000000000000000000000000000000000000000000001FFFFFFFFFFFFFF),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'hFFFFFFFFFFFFFE00000000000000000000000000000000000000000000000000),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'h000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hC000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000000000000),
    .INIT_45(256'h000000000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'h0000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80),
    .INIT_47(256'h00000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_48(256'h00000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000),
    .INIT_49(256'h000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_4A(256'h00000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000),
    .INIT_4B(256'h000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000000000000000001),
    .INIT_4C(256'h00000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000),
    .INIT_4D(256'h0000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000000000000),
    .INIT_4E(256'h00000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000),
    .INIT_4F(256'h000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000000),
    .INIT_50(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000000000000000000000000),
    .INIT_51(256'hC00000000000000000000000000000000000000000000000FFFFFFFFFFFFFFFF),
    .INIT_52(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_53(256'hFFFFFFFFFFC00000000000000000000000000000000000000000000000000000),
    .INIT_54(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_55(256'h00000000000000000000000000000000000000000000000000000000007FFFFF),
    .INIT_56(256'hFFFFFFFFC0000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_58(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_59(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5A(256'h00000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'hFFFFFE00000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_65(256'hFFFFFFFFFFC000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_66(256'hFFFFFFFFFFFFE0000000000000000000000000000000FFFFFFFFFFFFFFFFFFFF),
    .INIT_67(256'hFFFFFFFFFFF0000000000000000000000000000000003FFFFFFFFFFFFFFFFFFF),
    .INIT_68(256'hFFF0000000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_69(256'h0000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6A(256'h3FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000),
    .INIT_6B(256'hFFFFFFFFFF800000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h00000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000),
    .INIT_6F(256'h000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000),
    .INIT_74(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_75(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_76(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_77(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_79(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7D(256'hFFFFFC00000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_7E(256'hFFFFFFFFFF800000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7F(256'hFFFFFFFFFFFF80000000000000000000000000000001FFFFFFFFFFFFFFFFFFFF),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_14
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_14_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_14_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_14_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_14_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_14_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_14_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_14_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_14_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_14_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_14_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_14_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_14_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_14_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_14_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_15" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "15" *) 
  (* ram_slice_end = "15" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_20(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_21(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_22(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000),
    .INIT_23(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_24(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_27(256'h000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_28(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_29(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_2B(256'h0000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC000),
    .INIT_2E(256'h00000000000000000000000000000000000000000000000000003FFFFFFFFFFF),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE0000000000000),
    .INIT_30(256'h00000000000000000000000000000000000000000000000000000FFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE000000),
    .INIT_32(256'hFFFFFC0000000000000000000000000000000000000000000000000000000001),
    .INIT_33(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFF8000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hF800000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'h000000000000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'h00000000000000000000000000000000000000000000000001FFFFFFFFFFFFFF),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'hFFFFFFFFFFFFFE00000000000000000000000000000000000000000000000000),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'h3FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000000000000),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'h000000000000000000000000000000000000000000000000000000000000007F),
    .INIT_47(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000007FFFFFFFFFFFF),
    .INIT_49(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000FFFFFF),
    .INIT_4B(256'hFFFFFE0000000000000000000000000000000000000000000000000000000001),
    .INIT_4C(256'h00000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_4D(256'hFFFFFFFFFFFFE000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h00000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_4F(256'hFFF0000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_51(256'h000000000000000000000000000000000000000000000000FFFFFFFFFFFFFFFF),
    .INIT_52(256'hFFFFFFFFFF800000000000000000000000000000000000000000000000000000),
    .INIT_53(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_54(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_55(256'h00000000000000000000000000000000000000000000000000000000007FFFFF),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000),
    .INIT_5B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_60(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_61(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_62(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_63(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_64(256'h000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_65(256'hFFFFFFFFFFC00000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_67(256'hFFFFFFFFFFF00000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000),
    .INIT_6B(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000),
    .INIT_6F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_70(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_71(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_72(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_73(256'h00000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_7E(256'hFFFFFFFFFF800000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_15
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_15_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_15_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_15_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_15_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_15_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_15_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_15_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_15_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_15_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_15_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_15_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_15_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_15_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_15_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_16" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "16" *) 
  (* ram_slice_end = "16" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_24(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_27(256'h000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_28(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_29(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2B(256'hFFFFFFFFFFFFFFFC000000000000000000000000000000000000000000000000),
    .INIT_2C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2E(256'h00000000000000000000000000000000000000000000000000003FFFFFFFFFFF),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000001),
    .INIT_33(256'hFFFFFFFFFF800000000000000000000000000000000000000000000000000000),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'h00000000000000000000000000000000000000000000000001FFFFFFFFFFFFFF),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'hFFFFFFFFFFFFFE00000000000000000000000000000000000000000000000000),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'h00000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_48(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_49(256'h000000000000000000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000001),
    .INIT_4C(256'hFFFFFFFFFFC00000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_4E(256'h00000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000),
    .INIT_52(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_53(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_54(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_55(256'h00000000000000000000000000000000000000000000000000000000007FFFFF),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_65(256'h00000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'hFFFFFFFFFFF00000000000000000000000000000000000000000000000000000),
    .INIT_68(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_69(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6A(256'h000000000000000000000000000000000000000000000000FFFFFFFFFFFFFFFF),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000),
    .INIT_6F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_70(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_71(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_72(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_73(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_74(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_75(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_76(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_77(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_79(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_7E(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_16
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_16_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_16_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_16_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_16_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_16_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_16_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_16_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_16_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_16_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_16_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_16_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_16_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_16_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_16_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_17" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "17" *) 
  (* ram_slice_end = "17" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_24(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_27(256'h000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_28(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_29(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000001),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'hFFFFFFFFFF800000000000000000000000000000000000000000000000000000),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'h00000000000000000000000000000000000000000000000001FFFFFFFFFFFFFF),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'hFFFFFFFFFFFFFE00000000000000000000000000000000000000000000000000),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_47(256'h000000000000000000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000001),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'hFFFFFFFFFFF00000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_50(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_51(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_52(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_53(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_54(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_55(256'h00000000000000000000000000000000000000000000000000000000007FFFFF),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_65(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_66(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_67(256'h00000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000),
    .INIT_6F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_70(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_71(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_72(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_73(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_74(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_75(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_76(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_77(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_79(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_7E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_17
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_17_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_17_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_17_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_17_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_17_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_17_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_17_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_17_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_17_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_17_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_17_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_17_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_17_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_17_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_18" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "18" *) 
  (* ram_slice_end = "18" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_20(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_21(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_22(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_23(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_24(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_25(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_26(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_27(256'hFFFFF80000000000000000000000000000000000000000000000000000000000),
    .INIT_28(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_29(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000001),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'h00000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000001),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000),
    .INIT_56(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_57(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_58(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_59(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_60(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_61(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_62(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_63(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_64(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_65(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_66(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_67(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_68(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_69(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6E(256'h00000000000000000000000000000000000000000000000000000000007FFFFF),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_7E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_18
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_18_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_18_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_18_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_18_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_18_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_18_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_18_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_18_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_18_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_18_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_18_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_18_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_18_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_18_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_19" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "19" *) 
  (* ram_slice_end = "19" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_24(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_27(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_28(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_29(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000001),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000001),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_65(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_66(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_67(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_68(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_69(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_6F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_70(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_71(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_72(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_73(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_74(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_75(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_76(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_77(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_79(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE),
    .INIT_7E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_19
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_19_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_19_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_19_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_19_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_19_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_19_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_19_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_19_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_19_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_19_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_19_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_19_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_19_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_19_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_2" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "2" *) 
  (* ram_slice_end = "2" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFC0000000000000000000000007FFFFFFFFFFFFFFFFFFFC000000),
    .INIT_01(256'h000000000000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00),
    .INIT_03(256'h00000000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000000000000000),
    .INIT_16(256'h03FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000000000000),
    .INIT_18(256'h000000FFFFFFFFFFFFFFFFFFFF8000000000000000000000000FFFFFFFFFFFFF),
    .INIT_19(256'hFFE0000007FFFFFF80000000FFFFFFFFC0000000001FFFFFFFFFFFF800000000),
    .INIT_1A(256'h003FFE0007FFC000FFFE0003FFFC0003FFFE00007FFFF00000FFFFF800001FFF),
    .INIT_1B(256'h0FE01FC03F807F807FC03FC01FF007FC00FF801FF801FFC00FFE003FFC003FFC),
    .INIT_1C(256'h87C3E1F0F87C1F0F83E0F83F07C1F83F07E07C0FC0FC0FC07E07F01F80FE03F8),
    .INIT_1D(256'h38E71C638E38E38E38E38E1C71E3871E3C70E1E3C7878F0F0E1E1E1E1E0F0F07),
    .INIT_1E(256'h4C99B3266CCCD999999B3319999998CCCE6633198CE6339CC6318C6318E738C7),
    .INIT_1F(256'h56A952B56B52B5AD694A5A5A5A5A5B4B6925B6D24924926DB6493649B26C9B36),
    .INIT_20(256'h96D25B4B4B5A5294A56A56AD5AAD54AAB5554AAAAAAAAAAAAAAD5556AAB552A9),
    .INIT_21(256'h801FE07F07C1F0F0F0E1C70E39C739C67319CCCCC66CCCC99B364D936D924924),
    .INIT_22(256'h96DB6DB26C99B33333399CC639C71C70E1E1E0F83F03FC01FFF0000000000FFF),
    .INIT_23(256'h87870E38C7399CCCCC99B26DB6DB4B4B5AD4A954AAAA5552AAA954A95A96B4B4),
    .INIT_24(256'h5AA9555552AB56A5AD25B6C9B266666731CE38F1E0F03F007FFFFFFFFE01FC0F),
    .INIT_25(256'h56B496DB264CCC6738E1C3E0FE003FFF800FE0F878E38C67333364D925B4B5AD),
    .INIT_26(256'hAB5AD24936666631C70F07F00000003F83C38E398CCC9926925AD4AB555555AA),
    .INIT_27(256'h5A52A555552A529249B33338C78781FFC01FFC0F0E18E67266C92D2D6A955554),
    .INIT_28(256'hFFFC07871C6666C925A54AAAAAD5A5A4D93318C70F83FFC3FFC0F0E38CCCCD92),
    .INIT_29(256'hAAA95ADB6CD8CE70F07FC01FF07C718CCD93494AD55554AD2DB64CCE71C3C07F),
    .INIT_2A(256'h3F00003F878E733364B4A552AD56B4B64CCCE38F03FFFFF81E1CE7326CB695AA),
    .INIT_2B(256'h365B5A955552A5A4D9339C71F01FFFE03C38E6666DB4B56AAA952D2499398E38),
    .INIT_2C(256'hD98CE387C01FFC01F0E398C9924B4A95555AB4B6C9998C70F80FFFF01E1C731B),
    .INIT_2D(256'hC9B696A552AA55294924C99CC71E1FC000007E1E38CE64D92D2954AAD56B4B6C),
    .INIT_2E(256'hFE0F871C6733366DB4B4AD552B554AD6D24D9B398C70F07E000003F838718CCC),
    .INIT_2F(256'h7871C739CCCCC9924925A52B54AAAAAAD52B4B4924D9B3398C71E3E0FF000001),
    .INIT_30(256'h52AA55AB5A94B4B496DB6C9B264CCCCE6739C71C38783E07FC000000003FC0F8),
    .INIT_31(256'hDB6DB6DB6DB6DB692496DA4B696D2D2D2D694A52B52B56AD56AA955555555555),
    .INIT_32(256'h6AAB5555555554AAAD54AA54A95AD4A5296B4B5A5B4B49692DA496D2496DA492),
    .INIT_33(256'hE3C1E07F8000FF0000FE03E1E1E38E38CE733333366C9B6DB6DA5A5AD6B56AD5),
    .INIT_34(256'h07E001F8003F07871C63333326DB696B52AD55555AAD4A5A4B6C9B3333318C71),
    .INIT_35(256'h6D36CCCC638F0FC0000007E1E38E73326C96D2B56AAAAD529496D9333339C70F),
    .INIT_36(256'h1E31999B25A52AD54AA52D26CCCC638780FFFFF81F1E31999B24B4A554A955A9),
    .INIT_37(256'h5554A94924CCCE387C00FC01F0F39CC9925AD5AAA95696D933318F0F8000003E),
    .INIT_38(256'hD9998E3E0FFFFE078E319B2496A55554AD249B339C381FFFFFC1E38CCCD92D6A),
    .INIT_39(256'h1F80003F0F1CCCC925AD55555AD24D99CE3C1FE01FF0F1C66649252A555AB5A4),
    .INIT_3A(256'h83C719993696AD5552B4B6CCCC61C1FE007F878E7336496B555556B49266671E),
    .INIT_3B(256'h492D4AAAA95A5B266638E0FF001FE1E39CCD925AD52A956B49326338F07FFFFF),
    .INIT_3C(256'h52D24C98CE3C1FC001FC1E398C9B25AD5AAA55ADB6CCCE71E07FFFFE0F1C6336),
    .INIT_3D(256'hE1FC0003F071C6666DB4A55AA956B49366738E1F000001F8F18CCC924A54AAA9),
    .INIT_3E(256'h39CCC9B4B5AAAAAB52DB266631E1F800007E1E3199936D2B555556B4B64CCE71),
    .INIT_3F(256'h5554A94924CCC63C7E000003E1C7399B24B5AA556A94B6D9998E383F0000FE1E),
    .INIT_40(256'hB318E3C1FFFFF81E39CCCDB6D6A9556AD69364C671E0FE000FE0F1CC64C92D2A),
    .INIT_41(256'hFFFFF83C7319324B5AA552AD6926CCE71E1FE003FC1C719993696A55554AD249),
    .INIT_42(256'hE3999924B5AAAAAB5A49B339C787F801FE0E18CCCDB4B52AAAD5A5B266638F07),
    .INIT_43(256'h96B56AA9529249998E3C3FE01FE0F1CE66C92D6AAAAAD6924CCCE3C3F00007E1),
    .INIT_44(256'h5AD26CCCC71E0FFFFFE070E7336492D4AAAA95A4936631C781FFFFC1F1C6666C),
    .INIT_45(256'hF0000007C3C633326DA5AA5556AD69264CE73C3E00FC00F871CCCC924A54AAA9),
    .INIT_46(256'h56AA54AA94B493666631E3E07FFFFC078718CCCD92D2954AAD529693666631E1),
    .INIT_47(256'hC38E7333326DA4A52AD5555AB52DA4D93339C71E1F8000000FC3C718CCCDB2DA),
    .INIT_48(256'h38C633333364DB49694AD56AAAAAD52B5A5B6D93333318E38783F0007E001F83),
    .INIT_49(256'hAD5AB5AD69696DB6DB64D9B3333339CC71C71E1E1F01FC0003FC0007F81E0F1E),
    .INIT_4A(256'h2496DA492DA496D25A4B4B696B4B5A5294AD6A54A954AAD554AAAAAAAAAB555A),
    .INIT_4B(256'h54AAAAAD5554AA954A952B5294A52D6969696D2DA496DA4924B6DB6DB6DB6DB6),
    .INIT_4C(256'h07E03FFC0003FFE03F07C3C78E39CE7333333366C936DA4969694A56A55AA955),
    .INIT_4D(256'hFFFFFF01F070E318CCCC99249696A54AAAAAAAD5294B49249B32663318E38F0F),
    .INIT_4E(256'h66318F1F03FFFFFF81E1E318CCC9B25B5AD56AAA954A5A5B64CD8CC63870F80F),
    .INIT_4F(256'h3696956AAA95296DB266631C3C0FFFFFFC0F0E31999B24B4B54AAAA55AD24936),
    .INIT_50(256'h99CE3C3F000007E1E318CD93696A555556A5A49B3339C383F80003F838739999),
    .INIT_51(256'hC718CCD925A56AAAA95A5B64CCE71E1F800007E0E38CCCDB6D2B555556A5B6C9),
    .INIT_52(256'hAB52DB6CCCC71C3E00FF803E1C7399B2492952AAAD5A5B6C99CC70F03FFFFF07),
    .INIT_53(256'h01F878E733336DA5AD555554A5249333318F1F00FFFE03E1C7399936DA56AAAA),
    .INIT_54(256'hDB266631C787C01FFF807E1C31CCCC9B6D2D4AAAAAAD4B49264CE638F0FC0000),
    .INIT_55(256'h555556AD696DB64CCCE71C787E01FFFF007C3C71CC664C925B5AB555555A94B6),
    .INIT_56(256'hA55555556AD6B49249B333339C71E1F03FF0003FF03E1E38C633326C92DA56AD),
    .INIT_57(256'h6B5A5B6DB64CC98CC631C70F0FC0FF800000FF80F878F18E733333649B496B5A),
    .INIT_58(256'h7C07F8001FFF8000FE03E0F0E1C738C66333266C92492DA52952A95555554AA5),
    .INIT_59(256'h69694A56AD56AAB5555552AA956AD6B5A5A4B6DB64D9B33333319CE31C78F0F0),
    .INIT_5A(256'h03FFFFFFFFFFFC007FC07F07C1F0F0E1C38E39C6319CCCE66CCC993649B6DB49),
    .INIT_5B(256'h49249B6C9B26C99B33666666673319CC631CE38E38E1C3878783E0F81FC07FC0),
    .INIT_5C(256'hA955AAA5555AAAAAAAAAAAAAAAAA5554AA954AA54A95A94AD694B5A5A5B496DA),
    .INIT_5D(256'h64D9364D926C926DB6C92496DB6925B496D2D2D2D2D296B5AD6A52B52B56A956),
    .INIT_5E(256'h31CE318C739CC6319CE63399CCE673339999CCCCCCCCCCCCD9999332664C9932),
    .INIT_5F(256'h1E1E1F0F0F0F0F1E1E1C3C7870E1C3870E3C71E38E1C71C71C71C71CE38E71CE),
    .INIT_60(256'hE03F80FC07F03F81F81F81F81F81F03F07C0F83F07C1F07C1F0F83C1F0F8783C),
    .INIT_61(256'hE001FFE003FFC00FFE007FF003FE007FC01FF007FC03FE01FE01FE03FC07F80F),
    .INIT_62(256'h00007FFFFC00001FFFFE00007FFFE0000FFFF0000FFFE0007FFE0007FFC001FF),
    .INIT_63(256'hFFFFFFFFFFFE000000000003FFFFFFFFF800000003FFFFFFF8000000FFFFFF80),
    .INIT_64(256'hAAAAAAAAAAA555AAD5AB52B5294A5AD2D2D2DA5B496DA4925B6DB6DB6DB6DB6D),
    .INIT_65(256'h7C0FF000000000FF81F07870E38E7399CCCCC99364DB6DA4B4B4A56B56A9552A),
    .INIT_66(256'h000003FC1F1E38C673366C924B4B52AD555554AB52969249264CCCCE738E3878),
    .INIT_67(256'hCCC638707F000001F83C38C67366C92DAD4AAB52AAD4B4B6D9B33398E387C1FE),
    .INIT_68(256'hDB4B5AAD54AA52D26C99CC71E1F800000FE1E38CE64C924A52A9552A95A5B64C),
    .INIT_69(256'h6338E1E03FFFC07C38C6664DB4B56AAAA54B49264C671C3E00FFE00F871CC66C),
    .INIT_6A(256'h71C6726492D2A5555AB4B6D9999C70F01FFFE03E38E7326C96952AAAA56B69B3),
    .INIT_6B(256'h56A5B4D9339CE1E07FFFFF03C71CCCC9B4B5AAD52A94B49B3339C787F00003F0),
    .INIT_6C(256'hF80F0E39CCC9B6D2D4AAAAAD4A4B26CCC638F83FE00FF83C39CC6CDB6D6A5555),
    .INIT_6D(256'h26CCCCC71C3C0FFF0FFF07C38C63326C9696AD55554A96924D9998E38780FFFF),
    .INIT_6E(256'hAAAAA55AD2D24D99399C61C3C0FFE00FFE07878C73333649252952AAAA952969),
    .INIT_6F(256'h56AAAAAB54AD69259264CCC671C70F07F00000003F83C38E319999B2492D6B54),
    .INIT_70(256'hD6B4B6926C9B333398C71C787C1FC007FFF001FC1F0E1C7398CCC9936DA4B5A9),
    .INIT_71(256'hC0FE01FFFFFFFFF803F03C1E3C71CE33999999364DB692D695AB552AAAAA556A),
    .INIT_72(256'hB4B5A56A54AA55552AA95554AA54AD6B4B4B6DB6D93664CCCCE6738C71C38787),
    .INIT_73(256'hFFC0000000003FFE00FF03F07C1E1E1C38E38E718CE67333333664D936DB6DA4),
    .INIT_74(256'h924926DB26C9B3664CCCD98CCCCE63398E738E71C38E1C3C3C3E0F83F81FE007),
    .INIT_75(256'h552AB555AAAAD55555555555554AAAB554AAD56AD5A95A94A5296B4B4B692DA4),
    .INIT_76(256'hB364D93649B249B6D92492492DB6925B4B69696969694A5AD6B52B5AB52A55AA),
    .INIT_77(256'h8C739C6318C6318CE7319CC6633199CCCC66666663336666666CCCD9933664C9),
    .INIT_78(256'h83C3C1E1E1E1E1C3C3C7878F1E1C38F1E3871E38E1C71C71C71C71C718E39C73),
    .INIT_79(256'h7F01FC07E03F81F80FC0FC0FC0F81F83F07E0F83F07C1F07C3E0F87C3E1F0F87),
    .INIT_7A(256'hFFF000FFF001FFC00FFE007FE007FC00FF803FE00FF00FF807F807F00FE01FC0),
    .INIT_7B(256'hFFE000007FFFFC00003FFFF80001FFFF0000FFFF0001FFFC000FFF8001FFF000),
    .INIT_7C(256'h000000007FFFFFFFFFFFE0000000000FFFFFFFFC00000007FFFFFF8000001FFF),
    .INIT_7D(256'hA5555AAAAAAA95556AAD56AD5A95AD4A52D69694B69696D25B496DA496DB4925),
    .INIT_7E(256'h0E1F07E01FFFFFFFF803F07C3C38718E7399CCCD99326D924B6D2D2D6B52B54A),
    .INIT_7F(256'h7C07FFFFFF01F0F0E718CCCD9B6DB4B5AB552AAAD54A94B4B6DB264CCCE631C7),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_2
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_2_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_2_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_2_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_2_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_2_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_2_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_2_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_2_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_2_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_2_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_2_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_2_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_2_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_2_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_3" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "3" *) 
  (* ram_slice_end = "3" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFC000000000000000000000000000000000000000000003FFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'h00000000000000000000000000000000000000000000000000000000000000FF),
    .INIT_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_16(256'hFC00000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFF000000000000000000000000000000000000000000000FFFFFFFFFFFFF),
    .INIT_19(256'h0000000007FFFFFFFFFFFFFF0000000000000000001FFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'h003FFFFFF8000000FFFFFFFC00000003FFFFFFFF8000000000FFFFFFFFFFE000),
    .INIT_1B(256'h0FFFE0003FFF80007FFFC0001FFFF80000FFFFE00001FFFFF000003FFFFFC000),
    .INIT_1C(256'hF803FE00FF801FF003FF003FF801FFC007FF800FFF000FFF8007FFE000FFFC00),
    .INIT_1D(256'hC0F81F83F03F03F03F03F01F81FC07E03F80FE03F807F00FF01FE01FE00FF007),
    .INIT_1E(256'h8F1E3C3870F0E1E1E1E3C3E1E1E1E0F0F0783C1E0F07C3E0F83E0F83E0F83F07),
    .INIT_1F(256'h98CE63398C6339CE718C639C639C638C71C638E38E38E38E3871C78E3C70E3C7),
    .INIT_20(256'hDB64926D926C9B26C9B364C9933666CCD99993333333333333319998CCC66331),
    .INIT_21(256'hD5554AAA556AA55AA54A95AB5295AD6B5A52969694B69692D25B6925B6DB6DB6),
    .INIT_22(256'hB2492496DA4B6969696B4A5294AD4AD5AB54AA556AA95554AAAAAAAAAAAAAAAA),
    .INIT_23(256'h7F80FE07C0F87C3C3C7871E38E38C738C63398CC666633366664CD9B364D926D),
    .INIT_24(256'h93326666633398C631C638F1C3878787C1F03F01FF003FFF8000000001FFFC00),
    .INIT_25(256'hCD924DB692DA5AD295AB56AA55556AAAD5554AAD52B5294A5A5A496DB6D926C9),
    .INIT_26(256'h339CE38E3878783E07F007FFFFFFFFFF803F81F87C3C78E18E39CC6733333366),
    .INIT_27(256'hC6319CCCCC99364924969695AD52AB55555556AA54AD4B5B4B6DB649B3266667),
    .INIT_28(256'hAAA9552A56B4B49249366CCCCCE639C71E3C1F07F003FFFFFFC00FE07C3C3C71),
    .INIT_29(256'hCCCE631C70E0F07F007FFFFFF003F07C3C70C739CCCCCD9B6492DA5AD4A9552A),
    .INIT_2A(256'h3FFFFFFF807E0F0F1C739CCE64CD926D25A5A95AA9555552AB56B5A4B6DB26CC),
    .INIT_2B(256'h0E38C67333366C924B694AD4AAB5554AA952B4B4B6D9264CCCE631C71E3E0FC0),
    .INIT_2C(256'h4B5A56AD554AA9555AB52D6D24926CD9999CC738F1E1F07F000FFFF001FC0F07),
    .INIT_2D(256'h92DB24C99B3399CE71C70E1F07E01FFFFFFFFE01F83E1C38E318CC664CD926DA),
    .INIT_2E(256'h01FF80FC1F0F0E1C738C63331933264DB6DB496B5AD5AAD555555552AD5AD696),
    .INIT_2F(256'hAD5A95AD69696D249249364D9933333319CC738E38E1C3C1F07E03FF00000000),
    .INIT_30(256'h3666CC99364D926DB24925B692DA5A5AD294AD4A952A9552AAAAAAAAAAAA9552),
    .INIT_31(256'h38E38E38E38E38E71C71C638E71CE31CE318C6318CE7319CCE66733333333333),
    .INIT_32(256'h2666CCCCCCCCCC66633399CC6739CC6318E738C638C738E71C638E31C71C638E),
    .INIT_33(256'h4A954AAAD555555555AAA954AB56A56A5AD6969692DA4924924936C9B26CD9B3),
    .INIT_34(256'h001FFFFFFFC007F81F83C3C3C71C718C6331999993366C936DB6D25A5A5AD6A5),
    .INIT_35(256'hB65B69694AD5AA95555552AB56A5296925B24993266663318C71C70F0F07C0FF),
    .INIT_36(256'h4A94B4B6936C99B3399CE31E3C3C1F807FFFFFFFE01FC1E1E3C738C6673266CD),
    .INIT_37(256'h333398C71C3C3E07FC000001FF03E0F1E39CE6333264DB6DA5A52A552AAAAA95),
    .INIT_38(256'h38787E01FFFFFFF80FC1E3C718C66666C9B6D25AD6AD55555554A95A5A4B64D9),
    .INIT_39(256'hFF80003FF01F0F0E39CE66666C9B692D6B56AAB54AAA54AD2D249366CCC6739C),
    .INIT_3A(256'hFC07E1E1C718CE6664D9249696B56AAB552AAD5AD692DB26CCCCCE738E1E1F01),
    .INIT_3B(256'h71CE7333326C924B4B52B555AAB554A94A5B49364C998CE738F1E0F80FFFFFFF),
    .INIT_3C(256'h9B6496D294A9556AAB554A94A5B69364C999CC638E3C3E0FE00000000FE07C38),
    .INIT_3D(256'hB556AAA9552B52D2DB6D933667318C70E1F07E00FFFFFE00FE0F0F1C7398CCCD),
    .INIT_3E(256'h6B5A5B6D9366666731C71E1E0FE007FFFF801FC1E1E38E3399999B26DB696B5A),
    .INIT_3F(256'hCCCC6738E3C3C1FC01FFFFFC01F83E1C38C63399B326DB6D2D2B52AA5555AAB5),
    .INIT_40(256'h70F81FC00000001FC1F0F1C718CE664C9B25B694A54AAB555AAA54A52DA49B66),
    .INIT_41(256'hFFFFFFC07C1E3C739CC664C9B24B694A54AAB556AAB52B4B4924D9333339CE38),
    .INIT_42(256'h03E1E1C739CCCCCD936D25AD6AD552AB555AB5A5A4926C9999CC638E1E1F80FF),
    .INIT_43(256'hE7398CCD9B2492D2D4A9554AB555AB5AD25B64D99999CE71C3C3E03FF00007FE),
    .INIT_44(256'h6C9B49696A54AAAAAAAAD5AD692DB64D99998C638F1E0FC07FFFFFFE01F87870),
    .INIT_45(256'hA5555552A9529696DB6C9933319CE71E3C1F03FE000000FF81F0F0E38C673332),
    .INIT_46(256'hCD9933998C738F1E1E0FE01FFFFFFFF807E0F0F1E31CE6733664DB25B4B4A54A),
    .INIT_47(256'hFC0F83C3C38E38C633199993264936925A5295AB552AAAAAA556AD4A5A5B69B6),
    .INIT_48(256'h95AD6969692DB6DB24D9B32666663318C638E38F0F0F07E07F800FFFFFFFE003),
    .INIT_49(256'h366CD9364DB2492492496D25A5A5AD695A95AB54AA5556AAAAAAAAAD554AA54A),
    .INIT_4A(256'hC718E38E31C718E39C738C718C739C6318CE7398CE67331998CCCCCCCCCD9993),
    .INIT_4B(256'hCD99999CCCCC6673398CE7318C631CE718E71CE39C71C638E38E38E38E38E38E),
    .INIT_4C(256'h554AAAA95556AAAA9552A952A56B5AD6969696D25B6DB6DB24DB26CD933664CC),
    .INIT_4D(256'hFFFFFFFE007F03E0F0F0E1C718E7398CCCCCCC99B26D92492DA4B4A5AD4AD5AA),
    .INIT_4E(256'h4B5AD5AA5555555554AB56B5A5A496C9364CD9998CC639C71C3C7C3E07F007FF),
    .INIT_4F(256'h924DB326667318E38E1E1F03FC000000000FF03E1E1C38C7398CCCC99364925B),
    .INIT_50(256'h783E03FF000007FE03E0F1E38E7399999B36C92DA5AD6AD552AAA9556AD6B4B4),
    .INIT_51(256'hF81F0F1E39C67333326C9249694A54AAD55552AA56A5A5B6DB66CCCCCE638E38),
    .INIT_52(256'h32649249696A56AB5555556AB5294B69249B36666339C71C783C0FF000000007),
    .INIT_53(256'hAB552A529696DB6C9B3333339CE38F0F0F80FF00000003FE07C1E1C71C673333),
    .INIT_54(256'hC71E1E0FC07FC00000007FE03E0F0F1C71CE733333366D924B694B52A556AAAA),
    .INIT_55(256'h666664C9B249249696B5A952AB555555552A952B5AD2DA4936C9933333398C71),
    .INIT_56(256'h6CCCCCCCE6318C71C78F0F0F83F01FF0000FFFC0003FE03F07C3C38F1CE398CE),
    .INIT_57(256'hB26C9249249692D694A56A55AA95552AAAAA5555AAD5AB5AD69696D24924D936),
    .INIT_58(256'hFC0007FFFFFFFFFF0003FF00FE07C0F87C3C3870E38E31C6319CCE6666666CC9),
    .INIT_59(256'h24DB26CD9B32666CCCCCCE667319CE739C638E38E3C78F0F0F0F83E0FC07F00F),
    .INIT_5A(256'hAAAAAAAAAAAAAAAAD5552AAD54AA55AB56A56B5294B5A5AD25A5B492DB6DB6DB),
    .INIT_5B(256'h6DB6D2492DB492D25A4B4B4B4A5A5296B5A94AD4AD4A952AD52AB552AA95556A),
    .INIT_5C(256'hCE663339999CCCCCCCCCCCCCCCCC999933266CC99326CD9364D926C936D92493),
    .INIT_5D(256'h78E1C78E1C70E38E38F1C718E38E39C718E31CE31CE318C6318C6339CC673198),
    .INIT_5E(256'h3E0FC1F07C1F07C1E0F83C1E0F0783C3E1E1F0F0F0F0F0F0E1E1E3C3878F1E3C),
    .INIT_5F(256'h1FE01FF00FF00FE01FE03F807F01FC07F03F81FC0FE07E07E07E07E0FC0F81F0),
    .INIT_60(256'hFFC000FFF8003FFE001FFE001FFE003FF800FFC007FE007FE00FFC01FF007FC0),
    .INIT_61(256'hFFFE000003FFFFF000007FFFFC00007FFFE00007FFFC0001FFFE0003FFF8000F),
    .INIT_62(256'hFFFF80000000001FFFFFFFFF800000000FFFFFFFF00000007FFFFFF8000001FF),
    .INIT_63(256'h000000000000000000000003FFFFFFFFFFFFFFFFFC00000000000000FFFFFFFF),
    .INIT_64(256'h33333333333999CCE6339CC6318C631CE31CE39C718E38E39C71C71C71C71C71),
    .INIT_65(256'h2AA55555555555552AA552A54AD4A52D69696D25B6924936D926C9B264CD99B3),
    .INIT_66(256'h00000003FF01F83E0F0E1C71C738CE6333333266C9B2492492DA5A5AD6A56AD5),
    .INIT_67(256'hA5AD6AD52AAAAAAAAD56AD6B5A4B6DB6C99332633318C738E1C3C3E0FC07FE00),
    .INIT_68(256'h6D926CC998CC631C70E1F07E01FFFFFFFFE01F83E1C38E39CE6733664C936D25),
    .INIT_69(256'h83C0FE003FFFC003F83E1E3C738CE6666CD92492DAD2B56AAA554AAAD5A96B49),
    .INIT_6A(256'h0FC1F1E38E319CCCC9926DB4B4B52A554AAAB554AD4A5B4924D9B333398C71C3),
    .INIT_6B(256'hCD936DB496B5AB552AAAAA556A569692D926CC99CCE738E3C3C1F807FFFFFFF0),
    .INIT_6C(256'h52AA54AD696D249B66CCCCCE738C38F0F83F003FFFFFF803F83C1C38E319CCCC),
    .INIT_6D(256'h38F0F0F81FC00FFFFFFF003F83E0F1E38E719CCCCCD9B24924B4B5A952AA5555),
    .INIT_6E(256'h9999933649B6DB4B6B4AD4A955AAAAAAAB552AD6A5A5A49249B264CCCCE6318E),
    .INIT_6F(256'h9B33333398CE71C61C78F0F87E07F007FFFFFFFFFF803F81F0787871C71CE733),
    .INIT_70(256'h4D926DB6DA4969694A52B52AD54AAAAD555AAAA955AB56A52D696D25B6C926CD),
    .INIT_71(256'h00FFFE0000000007FFF003FE03F03E0F8787870E3C718E318C67331999993326),
    .INIT_72(256'hD926C9B366CC9999B3319998CC67318C738C71C71E3878F0F0F87C0F81FC07F8),
    .INIT_73(256'h5555555555555554AAAA555AA954AB56AD4AD4A5294B5A5A5A5B496DA4924936),
    .INIT_74(256'hB6DB6DB6925B692D25A5B4A5A5A5296B5AD6A52B56A54A956A955AA9554AAAAD),
    .INIT_75(256'h33198CCC66663333333333333326666CCD99B3264C9B364D9364D926D9249B6D),
    .INIT_76(256'h8F1C38F1C78E3871C71C71C71C718E38C718E718E718C639CE7318C67319CC66),
    .INIT_77(256'h83F07C1F07C1F07C1F0F83C1E0F0783C3C1E1E1E1F0F1E1E1E1C3C3870F1E3C7),
    .INIT_78(256'h803FC01FE01FE03FC03F807F01FC07F01F80FE07E03F03F03F03F03F07E07C0F),
    .INIT_79(256'h00FFFC001FFF8007FFC003FFC007FF800FFE007FF003FF003FE007FC01FF007F),
    .INIT_7A(256'h000FFFFFF000003FFFFE00001FFFFC00007FFFE0000FFFF80007FFF0001FFFC0),
    .INIT_7B(256'h001FFFFFFFFFFC0000000007FFFFFFFF00000000FFFFFFFC0000007FFFFFF000),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFE0000000000000000003FFFFFFFFFFFFFF8000000000),
    .INIT_7D(256'hC9999333333319998CCE67319CE6318C6318E718C718E71C638E71C718E38E39),
    .INIT_7E(256'hA54AAD554AAAAAAAAD555AA956AD5AD4A52D69692DA4B6DB6DB649B64D9B266C),
    .INIT_7F(256'h7FF8000000FFF00FE0F83C3C78E38C7398CCE6664CD9B26D92496D25A5AD6B52),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_3
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_3_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_3_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_3_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_3_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_3_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_3_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_3_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_3_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_3_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_3_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_3_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_3_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_3_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_3_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_4" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "4" *) 
  (* ram_slice_end = "4" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFC000000000000000000000000000000000000000000000000000),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'h000000000000000000000000000000000000000000000000000FFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFF8000000000000000000000000000000001FFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFC0000000000000FFFFFFFFFFFFFFFC000000000000000000FFFFFFFFFFFFFF),
    .INIT_1B(256'h0FFFFFFFC00000007FFFFFFFE000000000FFFFFFFFFE00000000003FFFFFFFFF),
    .INIT_1C(256'hFFFC0000FFFFE00003FFFFC00001FFFFF800000FFFFFF0000007FFFFFF000000),
    .INIT_1D(256'h00FFE003FFC003FFC003FFE001FFF8003FFF0003FFF8000FFFE0001FFFF00007),
    .INIT_1E(256'h0FE03FC07F00FE01FE03FC01FE01FF00FF803FE00FF803FF003FF003FF003FF8),
    .INIT_1F(256'h1F0F83C1F07C3E0F81F07C1F83E07C0F81F83F03F03F03F03F81F80FC07F03F8),
    .INIT_20(256'hE3871C71E38F1C38F1C3870E1C3878F0E1E1E3C3C3C3C3C3C3C1E1E0F0F87C3E),
    .INIT_21(256'hE6667333998CC663398CE6339CE6318C639CE718E738E71CE39C71C638E38E38),
    .INIT_22(256'h24924924936DB24DB24D9364D9366C993266CC99B3326666CCCCCCCCCCCCCCCC),
    .INIT_23(256'hAAD554AA9552A956A952A54AD4AD6A5294A52D694B4B5A5B4B49692DA496DB49),
    .INIT_24(256'hB696D2D2D696B5AD6B5295AB56AD52AD54AA9554AAAA955555555555555556AA),
    .INIT_25(256'hC38E3C718E39C6318C673199CCCCE6664CCCD99B366C9B26C936DB24924B6DA4),
    .INIT_26(256'hC3E0FC0FC07F803FF80007FFFFFFFFFF80007FF803FC07E07E07C3E0F0F0F0E1),
    .INIT_27(256'h94A52969692DA4924924DB26C99B32666666673398CE739C738E3871C3C78787),
    .INIT_28(256'h6664CC99326D92492492DA5A5A5294AD4A954AAD5556AAAAAA95554AA956A95A),
    .INIT_29(256'hF0F07C1F80FF007FFF8000000FFFF003FC0FC0F83C3C3C78E38E39C63398CCE6),
    .INIT_2A(256'h955555552AAB55AA56A5296B496924B64936CD93326666633398C638C71C38F0),
    .INIT_2B(256'h01F83E0F0F0E1C71C718C633998CCCD99B366D926DB492DA5A5294AD4A955AAA),
    .INIT_2C(256'h926C9B36666CCE666339CE71C71C70E1E1E0F83F01FE007FFFF0000FFFFC00FF),
    .INIT_2D(256'hB6496DA4B696B4A52B52A54AAD554AAAAAAAAB5552AB56AD4A5296B496924B6C),
    .INIT_2E(256'hFFFF8003FF00FE03F07C1F0F070F1E3C71C738E739CC66333333333664C9B24D),
    .INIT_2F(256'hCE6319CE718E71C71C71C78E1E3C3C3C1E0F83F03F01FC01FF8003FFFFFFFFFF),
    .INIT_30(256'h5B4B692DA496DB4924924924DB6C936C9B26C993264CD99B333333333333199C),
    .INIT_31(256'hAD4AD4AD4AD4AD4A56A56B52B5A94A56B5AD6B5AD6B5A5296B4B5A5A5A5A5A5A),
    .INIT_32(256'h4B4B69696969694B4A5AD296B5AD694A52B5AD6B5295AD4A56B52B5A95A94AD4),
    .INIT_33(256'h8CE673331999999999333266CD9B364C9364DB24DB6C924924925B6D24B692DA),
    .INIT_34(256'h00000000000007FFE003FC03F81F81F07C3E1E1E1C3870E38E38E39C639CE739),
    .INIT_35(256'h92C924DB264C99B333333198CE6318E71C71C78F1E1E1F0F83F03F00FF003FFF),
    .INIT_36(256'h9326D924DA492D25AD294A54A956AAD555555555554AAB54A95295AD2D692DA4),
    .INIT_37(256'hF0F0783F03FC01FFFC000001FFFC00FE03E0F83C3C78E38E39C63399CCCCCCD9),
    .INIT_38(256'h52AD54AAAAAAAAAAA554A952B5AD2D2DA49249364D9B3333333398C639C71C38),
    .INIT_39(256'h007FFFC0001FF00FC1F0787870E38E318C673339933366C9B64925B49694A529),
    .INIT_3A(256'hAAAD54AB52B5A52D2DB4924DB26CD99933199CC6318E38E1C3C3C1F07E01FF00),
    .INIT_3B(256'h7E0F83C3C38F1C738C633999CCD999326C926DA496D2D6B5AD5AB552AAAAAAAA),
    .INIT_3C(256'hB6D24DB64D9B33266733398C638E70E3C787C3E07E03FE001FFFFFFFF0007FC0),
    .INIT_3D(256'h3998CCCD99B2649B6DB6DA5B4A5AD6A54AA554AAAAAAAAAA555AA54AD6B5A5A4),
    .INIT_3E(256'h18C638E38F1E1E1F0FC0FE01FFE0000000001FFE01FC0FC3E1E1E3C71C718C63),
    .INIT_3F(256'h9696B5AD4A956AA95555555554AA954A95AD694B696DB6DB64993666CCCC6673),
    .INIT_40(256'h0FF8003FFFFFFFE001FF01F81F0F878F1C39C718C6733399933366C9B6C92DB4),
    .INIT_41(256'h555555552AB56AD6B5AD2DA496D924D932666CCE667318C738E3C70F0F07C1F8),
    .INIT_42(256'h03FE01F83E0F0F0E1C71C6318CE66332666CD936C924B6D2D296B52B54AAD555),
    .INIT_43(256'h5294A5A4B69249B64D9B3326733398C631C71C3878783E0FC03FE0000FFFF800),
    .INIT_44(256'h70E38E718C673333333366C9B2492496D2D2D6B52A54AA955555555554AAD52A),
    .INIT_45(256'h6CCCCCCE67318E71C71C78F0F07C1F01FC00FFFE000000FFFE00FF03F0783C3C),
    .INIT_46(256'h96D25AD2D6A52A54AB554AAAAAAAAAAAAD55AA54A94A52D692D2496C926D9326),
    .INIT_47(256'hFFF003FC03F03F07C3E1E1E3C78E38E39C6319CC663333333664C9936C924D24),
    .INIT_48(256'h739CE718E71C71C71C3870E1E1E1F0F83E07E07F00FF001FFF80000000000003),
    .INIT_49(256'h6D25B492DB6924924924DB6C936C9B24C9B366CD993332666666666333399CC6),
    .INIT_4A(256'hAD4A56A56B52B5A94AD6A52B5AD6B5294A5AD6B5A52D694B4A5A5A5A5A5B4B49),
    .INIT_4B(256'h96D2D2D69696B4A5AD294A5AD6B5A94A52B5A94AD6A56B52B52B52B52B52B52B),
    .INIT_4C(256'h998CCCCE6664CCCCD99B3264C9B26C9B24DB249B6DB6DB6DB6924B6925A4B696),
    .INIT_4D(256'h00000000007FFC00FF00FE07E0F83E0F0F0F0F1E3C71E38E31C738C6318CE633),
    .INIT_4E(256'h26C9B366CCCCCCCCCC67318C639C71C70E3C38787C3E07C0FC03FC01FFF00000),
    .INIT_4F(256'h2496DA4B4B5A52B52B54AA5556AAAAAAAAA5556AB54A95AD6B5A5A5B492DB6C9),
    .INIT_50(256'h07FE0000FFFFF80003FF01FC0F83E1E1E3C70E31C6318CE6633332664C9B26D9),
    .INIT_51(256'hAAB55AB56B52D69696DA4924DB26CD99B3333199CE639C71C71E3C3C3E1F81F8),
    .INIT_52(256'hC3871C718E7398CC6666664CD9B26DB2492DA4B4B5AD6A56AD56AAA555555552),
    .INIT_53(256'h673319CE718E38E3870F0F0F83E07F00FF8000FFFFFFFC0007FE01F81F87C3C3),
    .INIT_54(256'h95AB54AA95556AAAAAAAD5556AA55AB52B5AD6969692DB4926DB26C993326666),
    .INIT_55(256'h787878F1C38E38E718C6319CCC666666664CD9B26C9B6C925B6D25A5A5AD295A),
    .INIT_56(256'hE3C3C3C3E1F07C0FC07F00FF800FFFF000000000003FFFC007FC03F01F03E0F0),
    .INIT_57(256'h6925B6DB6DB249B24D9326CC99B333666666333399CC6739CE718E31C71C38F1),
    .INIT_58(256'hA9555555555555555556AAAA5552AA552A956AD5A95A94AD6B4A5AD2D2D2DA5B),
    .INIT_59(256'hB6924B692DA4B4B6969694B4A5AD6B5AD6B52B52B56AD5AA55AAD54AA9555AAA),
    .INIT_5A(256'h333333333333333366664CC9993366CD9B364D9B26D936C9B6C926DB6DB6DB6D),
    .INIT_5B(256'h8E38E38E31C71CE39C738C738C639CE739CE7318CE7319CCE633399CCCE66673),
    .INIT_5C(256'h0F87C3C1E1E0F0F0F0F0F0F0F0F0E1E1C3C78F0E1C38F1E3871E38F1C71E38E3),
    .INIT_5D(256'h80FE07F01F80FC0FC0FE07E0FC0FC1F81F03E0FC1F03E0F83E0F83C1F0783E1F),
    .INIT_5E(256'hC00FFE007FE007FE00FFC01FF007FC03FE01FF00FF00FF00FE01FC03F80FE03F),
    .INIT_5F(256'h1FFFE0000FFFF0001FFFC0007FFE0007FFC001FFF0007FF8007FF800FFF001FF),
    .INIT_60(256'hFFFFFF0000003FFFFFE000001FFFFFC00000FFFFF800007FFFF00001FFFF8000),
    .INIT_61(256'h0000000003FFFFFFFFFF80000000007FFFFFFFF800000001FFFFFFFC0000000F),
    .INIT_62(256'h000000000000001FFFFFFFFFFFFFFFFFF0000000000000007FFFFFFFFFFFFE00),
    .INIT_63(256'h000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000),
    .INIT_64(256'h69696969696B4B5A5296B5AD6B5AD6B5A94A56B52B5A95A94AD4AD4AD4AD4AD4),
    .INIT_65(256'hE663333333333333666CC993264D9364DB24DB6C924924924B6DA496D25B4B69),
    .INIT_66(256'hFFFFFFFFFF0007FE00FE03F03F07C1E0F0F0F1E1C78E38E38E39C639CE6319CC),
    .INIT_67(256'hC9364C99B33333333198CE739C738E38F1E3C383C3E0F83F01FC03FF0007FFFF),
    .INIT_68(256'hDB4925A4B5A5294AD5AB552AAB555555554AAAD54A952B5294B5A5B496DA49B6),
    .INIT_69(256'hFC00FFFFC0003FFFF801FE03F07C1E1E1C38E38E39CE731999CCD999B364D924),
    .INIT_6A(256'h556AA54AD4A529696D24B6D926D9B3666CCCC667318C638E38E1C3C3C1F07E03),
    .INIT_6B(256'h3C70E38C718C67331999993326CDB249B4925A4B5A5295A956AB5552AAAAAAA5),
    .INIT_6C(256'h9CCC67318E71C71C78F0F0F07C0FC0FF003FFFC0000007FFF803FC07E0F83C3C),
    .INIT_6D(256'h6A55AA554AAAA5555555AAAAD54AA54AD4A52969696D24924926D93264CC9999),
    .INIT_6E(256'h87878F0E3871C738E739CC67339999999933664D936C92492496D25A5A5294A5),
    .INIT_6F(256'h1C3C3C3C1F0F81F81F80FF007FF80007FFFFFFFFFF80007FF007F80FC0FC1F0F),
    .INIT_70(256'h96DB4924936DB24D9364D9B3666CCCC9999CCCCE663398C6318E71C638F1C70E),
    .INIT_71(256'h55AAAAAAAAAAAAAAAAA55554AAA554AAD52AD5AB56A52B5AD6B5A5AD2D2DA5B4),
    .INIT_72(256'h4B6DA496D25A4B4B696B4B4A5AD294A5295AD4AD4A952A55AA552AA554AAAD55),
    .INIT_73(256'hCCCCCCCCCCCCCCCD9999333664CD993264D9B26C9B26C936C936DB2492492492),
    .INIT_74(256'h71C71C718E38E71CE39C739C639CE718C6319CE7319CC673198CC6673339999C),
    .INIT_75(256'hF0F87C3C1E1E0F0F0F0F0F0F0F1E1E1C3C7870E1C3870E3C70E3C71E38E3871C),
    .INIT_76(256'h7F03F80FC07E07F03F03F03F03F07E07C0F81F07E0F83E07C1F0F83E0F07C3E1),
    .INIT_77(256'h7FF003FF003FF003FF007FC01FF007FC03FE01FE00FF01FE01FC03F80FF01FC0),
    .INIT_78(256'h80003FFFE0001FFFC0007FFF0003FFF0007FFE001FFF000FFF000FFF001FFC00),
    .INIT_79(256'h000003FFFFFF8000003FFFFFC000007FFFFE00000FFFFF00001FFFFC0000FFFF),
    .INIT_7A(256'hFFFFFFFFF00000000001FFFFFFFFFC000000001FFFFFFFF80000000FFFFFFFC0),
    .INIT_7B(256'hFFFFFFFFFFFFFC000000000000000000FFFFFFFFFFFFFFFC0000000000000FFF),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFE0000000000000000000000000000000007FFFFFFFFF),
    .INIT_7D(256'hA4B4B6969696B4B4A5A52D6B4A5294A5294A52B5AD4A52B5295AD4AD4A56A56B),
    .INIT_7E(256'h63399CCCC666666664CCC99B3264C9B26C9B24DB64926DB6DB6D2492DB496D25),
    .INIT_7F(256'h7FFFFFFFFFFFF0001FF803FC07E07C0F87C3E1E1C3C78E1C71C71CE39C6318CE),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_4
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_4_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_4_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_4_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_4_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_4_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_4_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_4_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_4_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_4_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_4_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_4_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_4_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_4_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_4_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_5" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "5" *) 
  (* ram_slice_end = "5" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h0000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE000000000000000000000),
    .INIT_1A(256'hFFFFFFFFFFFFFFFF0000000000000000000000000000000000FFFFFFFFFFFFFF),
    .INIT_1B(256'hF0000000000000007FFFFFFFFFFFFFFFFF000000000000000000003FFFFFFFFF),
    .INIT_1C(256'h00000000FFFFFFFFFC0000000001FFFFFFFFFFF0000000000007FFFFFFFFFFFF),
    .INIT_1D(256'h00FFFFFC000003FFFFFC000001FFFFFFC0000003FFFFFFF00000001FFFFFFFF8),
    .INIT_1E(256'hF0003FFF8000FFFE0003FFFE0001FFFF00003FFFF00003FFFFC00003FFFFC000),
    .INIT_1F(256'h1FF003FE007FC00FFE007FE003FF800FFE003FFC003FFC003FFE000FFF8003FF),
    .INIT_20(256'h03F81F81FC0FE03F01FC07F01FC07F00FE01FC03FC03FC03FC01FE00FF007FC0),
    .INIT_21(256'hF8787C3C1E0F0783C1F0F83C1F07C1F07C1F07E0F83F07E0FC1F81F83F03F03F),
    .INIT_22(256'hC71C71C71C71C38E3C71E3871E3870E1C3870F1E3C3C7878F0F0F0F0F0F0F0F0),
    .INIT_23(256'h331998CCE6633198CE63398CE7318C6318C6318E738C639C738E71CE38E71C71),
    .INIT_24(256'h24DB649B64DB26C9B264D93264C99B3666CCD999333326666666666666666733),
    .INIT_25(256'h952B56A52B5294A5294A5AD29696B4B4969692D25B492DB4925B6DB6DB6DB6C9),
    .INIT_26(256'h56AA555AAAD5556AAAAAAD55555555552AAAAAAD5556AAB554AA954AA55AA54A),
    .INIT_27(256'h4D9364DB249B6DB6DB6DB6925B49692D2D2D2D694A5AD6B5295A952B56AD52AD),
    .INIT_28(256'hE1E3C3870E1C71C71C71C639C6318C63398CC6633331999999B3332664CD9B36),
    .INIT_29(256'hFF007FE000FFFF800000000000000FFFFC003FF803FC03F81F81F83E0F87C3E1),
    .INIT_2A(256'hD9999999CCCC663398C6318C718E38C78E38F1E3C3878783C3E0F83F07E03F00),
    .INIT_2B(256'h5552AB55AA54A95A95AD6B5AD2D69692D25B4924B6D924936C9B26C993266CCC),
    .INIT_2C(256'h4925B692D2DA5AD2D694A52B52B52A54AB55AA9554AAAAD5555555555556AAAA),
    .INIT_2D(256'h8E38E39C718E739CE7319CC663333999999999333666CD9B26C9B26DB24926DA),
    .INIT_2E(256'h00007FFFFF0001FFF003FF00FF00FE03F03F07E0F83C1E0F0F0F0F0E1C3871C3),
    .INIT_2F(256'h0F83E1F07E0F81F81F81F80FE03FC03FE00FFC003FFE0001FFFFFC0000000000),
    .INIT_30(256'h638C71CE38E71C71C71C71C71C70E38F1C38F1E3C78F1E1C3C3C3C3C3C3C1E1F),
    .INIT_31(256'h318CE7318CE7318C67398C6339CE7398C6318C6318C639CE738C639C639C639C),
    .INIT_32(256'h738C718E718E718C739CE318C6318E739CC6318C6319CE7398C6339CE6318CE7),
    .INIT_33(256'hF0F87C3C1E1E1E1E1E3C3C78F1E3C78F1C78E3C71C70E38E38E39C71C738E31C),
    .INIT_34(256'hFFFFFFFFFFFFF8000003FFFC001FFE007FC01FE01FC07F03F03F03E07C1F07C1),
    .INIT_35(256'h71C71C38E1C3878F0F0F0F87C1E0F81F03F03F80FE01FF007FF000FFFF000000),
    .INIT_36(256'hB6924B6DB6DB6493649B26CD9B32664CCCCCCCCCCCC6673398CE739CE318E39C),
    .INIT_37(256'h5AA552AA5556AAAAA9555554AAAAAA5556AA556A952A56A56B5296B4A5A5A5B4),
    .INIT_38(256'h9CCE6733333333333666CD9B26C9B64936DB6DA496D25A5A5A5AD294AD6A56AD),
    .INIT_39(256'hFFFFFFFFFFE0000FFE007F807F03F03E0F87C3C1E3C3870E3871C638E718C631),
    .INIT_3A(256'h999CCC67318C631CE38C71C38E1C3878F0F87C3E0F81F81FC03FC00FFE0000FF),
    .INIT_3B(256'h2AA556A956A54AD6A5296B4B5A4B4B6925B6DB6DB249B26C9B366CC999999999),
    .INIT_3C(256'hDB6496DB692DA5B4B5A5AD294AD4A54A952A954AAB5554AAAAAAAAAAAAAAD555),
    .INIT_3D(256'hC1E0F0F1E1C3871C71C71C638C6318C6733998CCCCCCCCCC9993366C9B26C936),
    .INIT_3E(256'h07C1F81F80FE01FF003FFE00001FFFFFFFFFE00001FFF003FE01FC07E07E0F83),
    .INIT_3F(256'hB24D9364D9B32664CCCCCCCCCC6673398C6318C718E38E38E3870E1E3C3C1E0F),
    .INIT_40(256'hAAAD55555555555554AAAB554AA552A54A94AD4A52D696B4B696D25B6DA49B6D),
    .INIT_41(256'h666666664CD9B364D9364936DB6DB6925B4B496B4B5A5295AD4A95AA55AA9552),
    .INIT_42(256'hFC0001FFC00FF00FE07E07C1F0F87C3C7870E1C70E38C71CE318C63398CCE666),
    .INIT_43(256'h318C639C718E3871C3870F1E0F0F87C1F03F03F807F801FFC0001FFFFFFFFFFF),
    .INIT_44(256'hD5A95AD4A52D696969692DA496DB6DB249B64D9366CD99B3333333333399CCE6),
    .INIT_45(256'hB6969694B5A52B5A95A952A55AA955AAA9555554AAAAAA555555AAA9552A956A),
    .INIT_46(256'hE71C631CE739CC6733998CCCCCCCCCCCC9993366CD93649B249B6DB6DB4925B4),
    .INIT_47(256'h000003FFFC003FF803FE01FC07F03F03E07C1E0F87C3C3C3C7870E1C70E38E38),
    .INIT_48(256'h0F83E0F81F03F03F03F80FE01FE00FF801FFE000FFFF0000007FFFFFFFFFFFFC),
    .INIT_49(256'hE31C738E38E71C71C71C38E38F1C78E3C78F1E3C78F0F1E1E1E1E1E0F0F87C3E),
    .INIT_4A(256'h9CC6319CE7318C6739CE6318C6318CE739C6318C631CE738C639C639C638C738),
    .INIT_4B(256'h18E31CE718E738C631CE739CE739CE739CC6318CE7398C6339CC6339CC6339CC),
    .INIT_4C(256'h1E0F0F0F87870F0F1E1C3C78F1C38F1C38E3C71C71C71C71C71C738E39C738E7),
    .INIT_4D(256'hFFFFFFFFFF800000FFFF0007FF003FF00FF00FE03F81FC0FC1F83F07C1F0F83C),
    .INIT_4E(256'hE1C78F1E3C3C3C3C3C1F0F83E07C0FC0FE03F807FC01FFC003FFFC00000FFFFF),
    .INIT_4F(256'h924DB6D926C9366C993266CCCD999999999CCCE673398C6318C639C738E38E38),
    .INIT_50(256'hAAAB55555555555556AAAB555AA954AB56AD5A94AD6B5A52D69696D2DA496DB4),
    .INIT_51(256'h33266CD9B2649B24DB6C92496DB496D2DA5A5AD294B5295A95AB56A954AAD552),
    .INIT_52(256'hFC07E07E0F83E0F078787870E1C38E3C71CE38C739CE7398CE6733399999999B),
    .INIT_53(256'hE0F0F83E0F81F81F80FF00FF801FFF00007FFFFFFFFFFFFFF80001FFE007FC03),
    .INIT_54(256'h4C9932664CCCD9999999CCCCE663398CE739CE718E71C738E1C71E3870F1E1E1),
    .INIT_55(256'h2AD52A54A95A95AD4A5294B5A52D2D2D2D25B496DA4925B6C924936C93649B36),
    .INIT_56(256'hB56A956AB55AA9556AAA55552AAAAAA555555555556AAAAAAD5556AAB556AA55),
    .INIT_57(256'hB2492492492492DB6925B496D2DA5A4B4B4B5A5AD296B5AD6B5AD4A56A56AD5A),
    .INIT_58(256'hCE666666666666666664CCCC999B33664CD9B366CD9326C9B26C93649B64936D),
    .INIT_59(256'hC71C738E31C738C718E718C739CE739CE739CC63398CE63399CCE67331999CCC),
    .INIT_5A(256'hC3C3C3C3C3C3C3C387878F0E1E3C78F1E3C78E1C38E1C70E38F1C71C71C71C71),
    .INIT_5B(256'h0FC0FC0FC1F81F03E07C0F83F07C1F07C1F07C1F0F83E1F0F83C3E1F0F078783),
    .INIT_5C(256'hF007FC01FE00FF00FF00FF00FF00FE01FC07F00FE03F01FC07E03F01F81FC0FC),
    .INIT_5D(256'h00FFF8001FFF000FFF0007FF000FFE001FFC00FFE003FF003FF003FE007FC01F),
    .INIT_5E(256'hFFF000007FFFF80000FFFFE00007FFFC0001FFFF0000FFFF0001FFFC000FFFC0),
    .INIT_5F(256'hE00000000FFFFFFFE00000007FFFFFF8000001FFFFFF8000007FFFFF000001FF),
    .INIT_60(256'h0000000000003FFFFFFFFFFFE00000000000FFFFFFFFFF8000000001FFFFFFFF),
    .INIT_61(256'h0000000003FFFFFFFFFFFFFFFFFFFF800000000000000001FFFFFFFFFFFFFFF0),
    .INIT_62(256'h000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000),
    .INIT_63(256'hFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000000000000000),
    .INIT_64(256'hE718E718E718C739CE718C6318C6318C6739CE7318C67398C6339CC6339CC633),
    .INIT_65(256'hE1E0F0F0F0F0F0F0E1E3C78F1E3C70E3C71C38E38E38E38E38E39C71CE38C718),
    .INIT_66(256'h0000000000FFFFFE0001FFF000FFC01FF00FF01FC07E07E07E07C1F83E1F07C3),
    .INIT_67(256'h0E3870E1C3C3C3C3C1E0F07C1F83F03F01FC03FC03FF003FFE0003FFFFF80000),
    .INIT_68(256'h6D924936D9364D9366CD99B332666666667333198CE6339CE739C638E71C71C7),
    .INIT_69(256'h5555AAAAAAAAAAAAAD5554AAA556AB54A952B52B5294A5AD2D696D2D25B69249),
    .INIT_6A(256'hCCD993264D9364DB24926DB4924B692D25A5AD2D6B5AD6A56A54A956AB552AA9),
    .INIT_6B(256'h03F01F83F07C1F0F0787870F1E3C71C78C71C638C6318C673198CCCE6666666C),
    .INIT_6C(256'h1F0F87C1F07E07E07F00FF007FF000FFFFC000000000000007FFFC001FF803FC),
    .INIT_6D(256'hB366CC999333366666663333198CC67318C6318E718E38E38E38E1C3870F1E1E),
    .INIT_6E(256'hD52AD5AB52A56A52B5AD694A5AD2D2D2D25A4B6925B6DB6DB6DB64936C9B26C9),
    .INIT_6F(256'h4A956A954AA554AAB555AAAAD5555552AAAAAAAAAAD555555AAAAD556AA955AA),
    .INIT_70(256'h4DB6DB6DB6DB6924B6D24B692D25A5A4B4B5A5A52D694A5294A52B5295AB52A5),
    .INIT_71(256'h339999999999999999933332666CCD99B3664C99326C99364D936C9B649B6C92),
    .INIT_72(256'h38E39C71CE39C738E718C739C6318C6318C6339CC67319CC6633199CCC666333),
    .INIT_73(256'h3C3C3C3C3C3C3C3C7878F0F1E3C3870E1C3871E3871E38F1C70E38E38E38E38E),
    .INIT_74(256'hF03F03F07E07E0FC1F83F07C1F83E0F83E0F83E0F07C3E0F0783C1E0F0F8787C),
    .INIT_75(256'h0FF803FC01FE00FF00FF00FF00FE01FC03F80FE03F80FE03F01FC0FE07E07F03),
    .INIT_76(256'hFF0007FFC001FFF000FFF000FFF001FFC007FF001FF801FFC00FF801FF003FE0),
    .INIT_77(256'h000FFFFF00000FFFFF00003FFFF00003FFFE0001FFFF0001FFFC0007FFF0003F),
    .INIT_78(256'h7FFFFFFFE00000003FFFFFFF0000000FFFFFFE000000FFFFFF000000FFFFFC00),
    .INIT_79(256'hFFFFFFFFFFFF8000000000003FFFFFFFFFFE0000000000FFFFFFFFFC00000000),
    .INIT_7A(256'hFFFFFFFFF000000000000000000003FFFFFFFFFFFFFFFFF8000000000000003F),
    .INIT_7B(256'hFFFFFFFFFFFFFC0000000000000000000000000000000003FFFFFFFFFFFFFFFF),
    .INIT_7C(256'h000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7D(256'h9C738E718E718C739C631CE739CE739CE739CE739CC6318CE739CC6339CE6318),
    .INIT_7E(256'h1F0783C3C1E1E1E1E3C3C7870E1C3871E3871C38E38E1C71C71CE38E38C71CE3),
    .INIT_7F(256'h8000000000000FFFFFF80003FFE003FF803FE01FC03F81FC0FC0FC1F83E0F83E),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_5
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_5_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_5_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_5_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_5_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_5_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_5_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_5_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_5_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_5_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_5_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_5_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_5_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_5_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_5_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_6" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "6" *) 
  (* ram_slice_end = "6" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000000),
    .INIT_1B(256'hFFFFFFFFFFFFFFFF800000000000000000000000000000000000003FFFFFFFFF),
    .INIT_1C(256'h00000000FFFFFFFFFFFFFFFFFFFE000000000000000000000007FFFFFFFFFFFF),
    .INIT_1D(256'hFF000000000003FFFFFFFFFFFE00000000000003FFFFFFFFFFFFFFE000000000),
    .INIT_1E(256'h00003FFFFFFF00000003FFFFFFFE000000003FFFFFFFFC0000000003FFFFFFFF),
    .INIT_1F(256'h1FFFFC00007FFFF000007FFFFC00000FFFFFC000003FFFFFC000000FFFFFFC00),
    .INIT_20(256'hFC001FFE000FFFC001FFF8001FFF8000FFFE0003FFFC0003FFFE0000FFFF8000),
    .INIT_21(256'h007F803FE00FF803FE00FFC01FF801FF801FF800FFC007FF001FFE003FFC003F),
    .INIT_22(256'hF81F81F81F81FC0FC07E03F81FC07F01FC07F01FC03F807F00FF00FF00FF00FF),
    .INIT_23(256'hC3E1E0F0F87C3E1F0F83C1F0F83E0F83E0F83E0F83F07C1F83F07E0FC0F81F81),
    .INIT_24(256'h38E3871C78E3C70E3C78E1C3870E1C3878F0E1E1C3C3C78787878787878787C3),
    .INIT_25(256'hE63398C6339CE739CE739CE318E738C718E71CE39C71CE38E39C71C71C71C70E),
    .INIT_26(256'h9B33666CCC9999B33333366666666666333333319998CCC66733198CC663398C),
    .INIT_27(256'h6925B692492DB6DB6DB6DB24926DB249B649B64D936C9B264D93264D9B3664C9),
    .INIT_28(256'h4AB56AD5AB56A56A56A56B5294A5294A52D694B5A5A52D2D2D25A5B4B696D25B),
    .INIT_29(256'h55552AAAAA555555555555555555555556AAAAAD5556AAAD552AAD54AAD56AB5),
    .INIT_2A(256'h4B4B4B4B5A5AD296B5AD6B5AD4A56A52A56A54A956AD52A956AA556AAD556AAA),
    .INIT_2B(256'h333666CC993264C9B364D93649B24DB64936DB6D924B6DB6DA496DA4B692DA5A),
    .INIT_2C(256'h38E38E71CE39C631CE739CE7318CE63398CC6673339999CCCCCCCCCCCCCD9999),
    .INIT_2D(256'h7E07E07C0F81F07C1F0F83C1E0F0F878787878F0F1E1C3871E3871E38E38E1C6),
    .INIT_2E(256'h0000000000FFFFFFF00000FFFF0001FFF000FFE007FC01FF00FF00FE03F80FC0),
    .INIT_2F(256'h0FFC01FF800FFE001FFE000FFFC0003FFFF000003FFFFFFE0000000000000000),
    .INIT_30(256'h83F07E0FC0F81F81F81F81F81F80FC0FE03F01FC07F01FE03FC03FC03FC01FE0),
    .INIT_31(256'h3E0F07C1F0F83E0F87C1F07C3E0F83E0F83E0F83E0F83E0F83F07C1F83E07C1F),
    .INIT_32(256'h83F07E0F81F07E0F83E0FC1F07C1F07C1F07C1F07C1E0F83E0F83C1F07C1F0F8),
    .INIT_33(256'h00FF803FE01FE01FE03FC07F01FC07F01F80FC07E07F03F03F03E07E07C0FC1F),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFC0000001FFFFF80001FFFE0007FFC003FFC007FE007FE),
    .INIT_35(256'h0FC0FC07E03F807F00FF007FC01FF800FFF0007FFE0000FFFFF0000000FFFFFF),
    .INIT_36(256'h718E38E38E38E38F1C78E1C3870E1E3C3C3C3C3C3C3E1F0F87C1F07C1F07E07C),
    .INIT_37(256'h366CC9993332666664CCCCCC666666333199CCE67319CE6318CE718C639C638C),
    .INIT_38(256'h4A5AD2969696969692D25B496DA492DB6DB6DB6DB24936C936C9B64D9B26CD9B),
    .INIT_39(256'hAAAAAAAAAAAAAAA555552AAAD556AA955AAD56AB56A952A56AD4AD6A52B5AD6B),
    .INIT_3A(256'h2D29694A5AD6B5A94AD6A56AD4A952AD5AAD56AB552AAD556AAA955554AAAAAA),
    .INIT_3B(256'hB33664CD9B366C9B364DB26D936D924DB6DB6DB6DB6D24B6D25B496D2D2D2D2D),
    .INIT_3C(256'h1C78E71C71CE39C739C631CE7318C67319CCE673339998CCCCCCCCCCCCCC9999),
    .INIT_3D(256'hFE00FF01FE03F81F81F81F83F07C1F0783C1E0F0F0F0F0F0E1E3C78F1C38F1C7),
    .INIT_3E(256'h003FF8007FFE0000FFFFFE00000000000000000001FFFFFC0001FFF8007FF003),
    .INIT_3F(256'h8E3C70E3C78F1E1C3C3C3C3C3C1E0F0783E0F83F07E07E07E07F01FE03FC01FF),
    .INIT_40(256'h6664CCCCCCCCCCCCCC666733399CCE63398C6339CE318E738E71CE38E39C78E3),
    .INIT_41(256'hD2D2D2D2DA4B692DB492DB6DB6DB6DB6C926DB26D936C9B364D9B366CC99B336),
    .INIT_42(256'h555554AAAAA5555AAAD552AB55AAD56AD52A54AD5A95AD4A56B5AD694A5A52D2),
    .INIT_43(256'h5AD6B5295AD4AD5A952A55AB55AAD56AA555AAAD5552AAAA9555555555555555),
    .INIT_44(256'h66CD9366C9B64DB24DB24936DB6DB6DB6D2496DA4B692D25A5A5A5A5A52D694B),
    .INIT_45(256'hC718E718C639CC6319CE63399CCE663331999998CCCCCC9999993332664CD9B3),
    .INIT_46(256'hF81F83E0F83E0F87C3E1F0F0F0F0F0F0F1E1C3870E1C78E3C71C71C71C71C638),
    .INIT_47(256'hFFFFFC0000003FFFFC0001FFF8003FFC007FE00FF803FC03F807F01F80FC0FC0),
    .INIT_48(256'hFF801FF800FFF000FFF8001FFFE00007FFFFE0000000FFFFFFFFFFFFFFFFFFFF),
    .INIT_49(256'hE0FC0F81F81F03F03F03F81F80FC07E03F80FE03F80FF01FE01FE01FF007FC01),
    .INIT_4A(256'h7C3E0F83E0F07C1F07C1E0F83E0F83E0F83E0F83E0FC1F07C1F83E07C1F83F07),
    .INIT_4B(256'h1F03E0F81F07C0F83E0F83E0F83E0F83E0F83E0F07C1F07C3E0F83C1F07C3E0F),
    .INIT_4C(256'hE00FF00FF807F00FE01FC07F01FC0FE03F03F81F81F81F81F81F83F03E07C0F8),
    .INIT_4D(256'hFFFFFFFFFFFFFFFF00000007FFFFC0000FFFF0003FFE000FFE003FF801FF003F),
    .INIT_4E(256'hE03F80FE03FC03FC03FF007FE003FFC001FFF80003FFFFC0000003FFFFFFFFFF),
    .INIT_4F(256'h71C38E38E1C70E1C78F1E1C3C38787878783C3E1F0F87C1F07C1F83F07E07E07),
    .INIT_50(256'h9999333333333333319998CCC6673398CE63398C6318C631CE718E31C638E38C),
    .INIT_51(256'h9692DA4B692DB6924925B6DB24924DB64936C9B64D9364C9B366CD9B32664CC9),
    .INIT_52(256'hAAAD552AA556AA552AD52AD5AB56A56AD4A56A5294A5294A5AD29694B4B4B4B6),
    .INIT_53(256'h4AA552AB552AAD552AAA55552AAAAA555555555555555555555554AAAAAD5556),
    .INIT_54(256'h96D25B4B69696D2D2D2D69694B4A52D6B5AD6B5AD4A56A52B56A54AD5AA54AB5),
    .INIT_55(256'hB3664C99326CD9366C9B26D93649B649B64926DB6C924924924925B6DA492DA4),
    .INIT_56(256'h398CE673399CCE6673339999CCCCCCC666666666664CCCCCC9999B332664CC99),
    .INIT_57(256'h3C71C71C71C71CE38E39C718E31C638C738C639CE318C6318C6318C67398CE63),
    .INIT_58(256'hF0787878787878787878F0F0E1E3C3878F1E3C78F1E3C70E3C70E3871C78E38E),
    .INIT_59(256'hF81F83F03E07C0F81F07E0F83E0F83E0F83E0F83C1F0F83C1E0F0783C1E1E0F0),
    .INIT_5A(256'h03FC03FC03FC03FC07F80FF01FC07F01FC07F01FC0FE07F03F01F81F81F81F81),
    .INIT_5B(256'h0FFF000FFE001FFC007FF003FF801FF801FF801FF003FE00FFC03FE00FF807FC),
    .INIT_5C(256'hFFF80001FFFF0000FFFF0000FFFF0001FFF8000FFFC001FFF8003FFE001FFF00),
    .INIT_5D(256'hFF0000001FFFFFF0000007FFFFF000001FFFFF000003FFFFC00003FFFF80001F),
    .INIT_5E(256'h000000007FFFFFFFFF0000000007FFFFFFFE00000000FFFFFFFE0000000FFFFF),
    .INIT_5F(256'hFFFFFFFFF0000000000000007FFFFFFFFFFFFE0000000000007FFFFFFFFFFE00),
    .INIT_60(256'h0000000000003FFFFFFFFFFFFFFFFFFFFFFF00000000000000000001FFFFFFFF),
    .INIT_61(256'h0000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE0000000000000000),
    .INIT_62(256'hFFFFFFFFFFFFFFE0000000000000000000000000000000000000000000000000),
    .INIT_63(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_64(256'hE0F81F07E0F83F07C1F07C1F07C1F07C1F07C1F0F83E0F87C1F07C3E0F83C1F0),
    .INIT_65(256'h1FE00FF00FF00FF01FE03F80FE03F01FC0FC07E07E07E07E07E07C0FC1F83F07),
    .INIT_66(256'h0000000000000001FFFFFFF000003FFFF0000FFFC001FFE001FFC007FE00FFC0),
    .INIT_67(256'h0FC07F01FC03FC03FE00FF801FFC003FFE0003FFFC00003FFFFFFC0000000000),
    .INIT_68(256'h8E1C71C71E3871E3870E1E3C3C787878787C3C1E0F07C3E0F83E07C0F81F81F8),
    .INIT_69(256'h6666CCCCCCCCCCCCCE6667333998CC67319CC6339CE739CE318E71CE39C71C71),
    .INIT_6A(256'h696D25B496DA496DB6DB4926DB6DB249B6C93649B26C9B364C993264CD99B332),
    .INIT_6B(256'h555AAAD55AA955AA552AD5AA54A95A95295A94AD6B5AD6B5A52D696B4B4B4B49),
    .INIT_6C(256'hB55AAD54AAD552AAD555AAAAD55555AAAAAAAAAAAAAAAAAAAAAAA9555552AAA9),
    .INIT_6D(256'h692DA5B4B69692D2D2D29696B4A5AD294A5294A52B5A95A95A95AB56AD5AB54A),
    .INIT_6E(256'h4C99B366C99326C99364DB26C9B649B64936D924936DB6DB6DB6D24925B6925B),
    .INIT_6F(256'hC673198CC66333998CCC6666333333319999999999B33333366664CCD99B3366),
    .INIT_70(256'hC38E38E38E38E71C71CE38E71CE39C638C739C631CE739CE739CE7318C67319C),
    .INIT_71(256'h0F87878787878787878F0F0E1E1C3C7870E1C3870E1C78F1C38F1C78E3871C71),
    .INIT_72(256'h07E07C0FC1F83F07E0F83F07C1F07C1F07C1F07C3E0F07C3E1F0F87C3C1E1F0F),
    .INIT_73(256'hFC03FC03FC03FC03F807F00FE03F80FE03F80FE07F01F80FC0FE07E07E07E07E),
    .INIT_74(256'hF000FFF001FFE003FF800FFC007FE007FE007FE00FFC01FF007FC01FF007F803),
    .INIT_75(256'h0007FFFC0001FFFF0000FFFF0001FFFC0007FFE0007FFE000FFFC001FFE000FF),
    .INIT_76(256'h00FFFFFFC000000FFFFFF000000FFFFFC00000FFFFF800003FFFF80000FFFFE0),
    .INIT_77(256'hFFFFFFFF0000000000FFFFFFFFF000000001FFFFFFFF00000003FFFFFFF00000),
    .INIT_78(256'h000000001FFFFFFFFFFFFFFF00000000000001FFFFFFFFFFFF000000000003FF),
    .INIT_79(256'hFFFFFFFFFFFF800000000000000000000001FFFFFFFFFFFFFFFFFFFC00000000),
    .INIT_7A(256'hFFFFFFFFF000000000000000000000000000000000000007FFFFFFFFFFFFFFFF),
    .INIT_7B(256'h00000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h7C0F81F07E0F83F07C1F03E0F83E0F83E0F83E0F83C1F07C1F07C3E0F83E1F07),
    .INIT_7E(256'hFF007FC03FE01FE01FC03F80FE03F80FE07F03F81F81FC0FC0FC1F81F83F03E0),
    .INIT_7F(256'h00000000000000000007FFFFFFE000007FFFE0003FFF8003FFC003FF801FF801),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_6
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_6_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_6_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_6_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_6_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_6_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_6_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_6_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_6_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_6_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_6_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_6_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_6_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_6_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_6_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_7" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "7" *) 
  (* ram_slice_end = "7" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC000000000),
    .INIT_1C(256'hFFFFFFFF00000000000000000000000000000000000000000007FFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFC00000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFC000000000000003FFFFFFFFFFFFFFFFC0000000000000000003FFFFFFFF),
    .INIT_1F(256'hE0000000007FFFFFFFFF80000000000FFFFFFFFFFFC000000000000FFFFFFFFF),
    .INIT_20(256'hFFFFE000000FFFFFFE0000001FFFFFFF00000003FFFFFFFC00000000FFFFFFFF),
    .INIT_21(256'h007FFFC0000FFFFC0000FFFFE00001FFFFE00000FFFFF800001FFFFFC000003F),
    .INIT_22(256'h001FFE001FFE000FFF8003FFE0007FFE0007FFE0003FFF8000FFFF0000FFFF00),
    .INIT_23(256'h03FE00FF007FC01FF003FE00FFC00FFC00FFC00FFC007FE003FF800FFF001FFE),
    .INIT_24(256'h3F03F81F80FC07F03F80FE03F80FE03F80FF01FE03FC07F807F807F807F807FC),
    .INIT_25(256'hF83C1F07C3E0F83E0F83E0FC1F07C0F81F07E0FC1F81F03F03E07E07E07E07F0),
    .INIT_26(256'h1C3C7870F0E1E1C3C3C3C78787878787C3C3C3C1E1E0F0F8783C1E0F0783C1F0),
    .INIT_27(256'h8E39C71C71CE38E38E38E3C71C71C38E3871C78E1C70E3C78E1C3871E3C7870E),
    .INIT_28(256'h73398CE63398C67398C6739CE739CE739CE718C639C631CE31C639C738E71C63),
    .INIT_29(256'h66664CCCCC999999999999999999999998CCCCCE6667333199CCCE6733198CC6),
    .INIT_2A(256'h926D926D936C9B24D9364D9366C9B364C9B366CD9B3664CD9B33664CC999B333),
    .INIT_2B(256'h5A5B4B692DA4B692DA496DA492DB6924925B6DB6DB6DB6DB6C924936DB24936C),
    .INIT_2C(256'h52B52B5A94AD6B5A94A5294A5AD6B4A52D694B5A5AD2D296969696969696D2D2),
    .INIT_2D(256'h54AAB556AAD55AA955AAD56AB55AAD52AD52AD5AA54A952A54AD5AB52B52B56B),
    .INIT_2E(256'h55555555555555555AAAAAAAAA5555555AAAAAB55556AAAA5555AAAB5552AA95),
    .INIT_2F(256'hA55554AAAAA555554AAAAAA55555556AAAAAAAAA955555555555555555555555),
    .INIT_30(256'h56AAD55AAA554AAB554AAB554AAA555AAA9554AAAD554AAA95556AAA95554AAA),
    .INIT_31(256'h6AA552AB55AA955AAD54AAD56AA556AA556AA556AA556AA556AAD54AA9552AB5),
    .INIT_32(256'hA9552AA554AAD55AA955AAB552AB552AB552AB552AB55AA955AA954AAD54AA55),
    .INIT_33(256'h55AAAA95554AAAB5556AAAD554AAAD554AAA5552AAD556AA9556AAD552AA554A),
    .INIT_34(256'h555555555555555555555555554AAAAAAAAAB55555552AAAAA9555552AAAAD55),
    .INIT_35(256'hAA9556AAB5552AAA5555AAAA955552AAAAA5555554AAAAAAAAA5555555555555),
    .INIT_36(256'hA52B52B52B52B52A56AD4A952A54AB56A956A956A954AA552A955AA955AAB556),
    .INIT_37(256'h5B496D2DA5A4B4B4B6969696B4B4B4A5A52D694B5A5294B5AD6B5AD6B5294AD6),
    .INIT_38(256'h936C9B24DB24DB24DB64926DB6C924924924924924925B6DA492DB692DB496D2),
    .INIT_39(256'hCCCCCCCCCCCCCCC99999B3336664CCD9933664CD9B3264C9B366C9B364D9364D),
    .INIT_3A(256'hCE318E739CE739CE7318C67318CE63319CCE673399CCCE667333199998CCCCCC),
    .INIT_3B(256'h3C3878F1E3C78F1C3871C38E1C71E38E38E38E38E38E38C71C638E71CE31CE31),
    .INIT_3C(256'hE07F07E07E0FC1F83E07C1F07C1F0783E1F0F87C3C1E1F0F0F0F0F0F0F0F1E1E),
    .INIT_3D(256'h0000FFFE0003FFE001FFE003FF801FF803FE00FF00FF00FF01FC07F01FC0FE07),
    .INIT_3E(256'hFFFFF8000001FFFFFFFFFE00000000000000000001FFFFFFFFFE0000007FFFFC),
    .INIT_3F(256'h81FC0FE03F80FE03FC03FC03FC01FF007FE007FF001FFE001FFF0001FFFC0000),
    .INIT_40(256'hE1E3C3C3C3C3C3C3C3E1E0F0F87C3E1F0783E0F83E0F81F07E0FC1F81F83F81F),
    .INIT_41(256'h31CE31CE39C718E38C71C71C71C71C71C71E38E1C70E3870E3C78F1E3C7870F1),
    .INIT_42(256'hCCCCCC666663333999CCCE673399CCE63319CC63398C6339CE739CE739C631CE),
    .INIT_43(256'hC9B26C9B364D9B364C993366CC99B3266CCC999B333666664CCCCCCCCCCCCCCC),
    .INIT_44(256'h2DA4B6D25B6D2496DB6924924924924924924DB6D9249B6C936C936C9364DB26),
    .INIT_45(256'hAD4A52B5AD6B5AD6B4A5296B4A5AD29694B4B4B5A5A5A5B4B4B49696D2DA4B69),
    .INIT_46(256'hAAB556AA556AA552A954AA55AA55AA55AB54A952A54AD5A952B52B52B52B5295),
    .INIT_47(256'hAAAAAAAAAAAA9555555554AAAAAA9555552AAAA55556AAA95552AAB555AAA555),
    .INIT_48(256'hAAD55552AAAAA5555552AAAAAAB5555555554AAAAAAAAAAAAAAAAAAAAAAAAAAA),
    .INIT_49(256'h4AA9552AAD55AAA555AAAD552AA9554AAAD554AAAD555AAAB5554AAAA55556AA),
    .INIT_4A(256'hA954AAD54AA556AA556AB552AB552AB552AB552AB556AA556AAD54AA9552AA55),
    .INIT_4B(256'h4AA955AAB552AA556AA556AA556AA556AA556AA552AB552A955AA954AAD56AA5),
    .INIT_4C(256'hAAA5555AAAAD555AAAB5552AAB555AAA9556AAB554AAB554AAB556AA9552AA55),
    .INIT_4D(256'h555555555555555555555552AAAAAAAAA55555556AAAAAA555556AAAAB55556A),
    .INIT_4E(256'h4AAAD554AAA95556AAAA55554AAAAA95555552AAAAAAAA955555555555555555),
    .INIT_4F(256'hA56AD4AD4A95AB56AD5AB56A952AD52AD52A954AA552A955AA9552AA554AAB55),
    .INIT_50(256'hD2D25A5A5A5A5A5A5AD2D29694B5A52D6B4A52D6B5AD6B5A94A52B5A94AD4AD6),
    .INIT_51(256'h24DB6C924DB6DB2492492492492496DB6DA492DB6925B692DA4B692DA4B49692),
    .INIT_52(256'hCCC999B33664CC99B3664C993264C9B366C9B364D9364D936C9B24D926D926DB),
    .INIT_53(256'h73399CCC66333199CCCC66663333339999999999999999999999993333366664),
    .INIT_54(256'hE71C638C718E71CE31CE718E738C6318C6318C6318C6739CC67398CE63398CC6),
    .INIT_55(256'h3C7870E1C38F1E3870E3C71E3871C78E3871C71C70E38E38E38E39C71C71CE38),
    .INIT_56(256'hC1F0F87C3E1F0F8783C3E1E1F0F0F0F8787878787870F0F0F1E1E3C3C7870F1E),
    .INIT_57(256'hC07E07E07E07E0FC0FC1F81F03E07C0F83F07C1F03E0F83E0F83E0F87C1F0F83),
    .INIT_58(256'h007F807F807F807F807F00FF01FC03F80FE03F80FE03F80FC07F03F81F80FC0F),
    .INIT_59(256'h001FFC003FF800FFE007FF003FF003FF003FF003FE00FFC01FF007FC01FE00FF),
    .INIT_5A(256'hFC0003FFFC0003FFF8000FFFE0007FFE0007FFE000FFF8003FFE001FFE001FFE),
    .INIT_5B(256'h0FFFFFF000001FFFFF800003FFFFE00001FFFFE00003FFFF00003FFFF00007FF),
    .INIT_5C(256'h00000001FFFFFFFF00000000FFFFFFFE0000000FFFFFFE0000003FFFFFE00000),
    .INIT_5D(256'h000000001FFFFFFFFFFFF800000000001FFFFFFFFFFC0000000003FFFFFFFFE0),
    .INIT_5E(256'h000000007FFFFFFFFFFFFFFFFFF80000000000000000FFFFFFFFFFFFFFF00000),
    .INIT_5F(256'h0000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFF80000000000000),
    .INIT_60(256'h0000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000),
    .INIT_61(256'hFFFFFFFFFC000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_63(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_64(256'hB552AA554AAD55AA955AA955AA955AA955AA955AAD54AAD56AA556AB552A955A),
    .INIT_65(256'h554AAAA5555AAAA5554AAAD554AAA5556AA9554AAB554AAB554AA9556AAD55AA),
    .INIT_66(256'hAAAAAAAAAAAAAAAAAAAAAAA5555555555AAAAAAA9555554AAAAA955554AAAA95),
    .INIT_67(256'hA5552AAB5556AAA95555AAAAB555556AAAAAA9555555556AAAAAAAAAAAAAAAAA),
    .INIT_68(256'h5AB52B52B56AD4A952A54A956AD52AD52AD56AB55AAD56AA556AAD55AAB554AA),
    .INIT_69(256'h2D2DA5A5A5A5A5A5A52D2D696B4A5AD294B5AD694A5294A56B5AD4A56B52B52B),
    .INIT_6A(256'hDB24936DB24924DB6DB6DB6DB6DB6924925B6D2496DA496D25B496D25B4B6969),
    .INIT_6B(256'h3336664CC99B3366CC99B366CD9B364C9B364D9B26C9B26C9364DB26D926D924),
    .INIT_6C(256'h8CC6633399CCCE6633339999CCCCCC666666666666666666666664CCCCC9999B),
    .INIT_6D(256'h18E39C738E718E31CE318E718C639CE739CE739CE7398C67398C67319CC67339),
    .INIT_6E(256'hC3878F1E3870E1C78F1C38E1C78E3871C70E38E38F1C71C71C71CE38E38E71C7),
    .INIT_6F(256'h3E0F0783C1E0F0787C3C1E1E0F0F0F0F87878787878F0F0F0E1E1C3C3878F0E1),
    .INIT_70(256'h3F81F81F81F81F03F03E07E0FC1F83E07C0F83E0FC1F07C1F07C1F0F83E0F07C),
    .INIT_71(256'hFF807F807F807F807F80FF01FE03FC07F01FC07F01FC07F03F80FC07E07F03F0),
    .INIT_72(256'hFFE003FFC007FF001FF800FFC00FFC00FFC00FFC01FF003FE00FF803FC01FF00),
    .INIT_73(256'h03FFFC0003FFFC0007FFF0001FFF8001FFF8001FFF0007FFC001FFE001FFE001),
    .INIT_74(256'hF000000FFFFFE000007FFFFC00001FFFFE00001FFFFC0000FFFFC0000FFFF800),
    .INIT_75(256'hFFFFFFFC00000000FFFFFFFF00000003FFFFFFE0000001FFFFFFC000001FFFFF),
    .INIT_76(256'hFFFFFFFFC000000000000FFFFFFFFFFFC00000000007FFFFFFFFF8000000001F),
    .INIT_77(256'hFFFFFFFF0000000000000000000FFFFFFFFFFFFFFFFF000000000000000FFFFF),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFF00000000000000000000000000FFFFFFFFFFFFFF),
    .INIT_79(256'hFFFFFFFFFFFF80000000000000000000000000000000000000000003FFFFFFFF),
    .INIT_7A(256'h000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h56AAD55AAB552AA556AA554AAD54AAD54AAD54AAD56AA556AA556AB552AB55AA),
    .INIT_7E(256'hAA55556AAAB5554AAA95552AAB5552AAB555AAAD552AA9556AA9552AAD55AAB5),
    .INIT_7F(256'hAAAAAAAAAAAAAAAAAAAAAAAAAAB5555555554AAAAAAAD555556AAAAAD55552AA),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_7
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_7_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_7_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_7_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_7_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_7_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_7_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_7_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_7_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_7_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_7_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_7_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_7_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_7_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_7_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_8" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "8" *) 
  (* ram_slice_end = "8" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'h00000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000),
    .INIT_1F(256'hFFFFFFFFFF800000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF000000000),
    .INIT_20(256'hFFFFFFFFFFF00000000000001FFFFFFFFFFFFFFC0000000000000000FFFFFFFF),
    .INIT_21(256'h007FFFFFFFF000000000FFFFFFFFFE0000000000FFFFFFFFFFE000000000003F),
    .INIT_22(256'hFFE000001FFFFFF0000003FFFFFF80000007FFFFFFC0000000FFFFFFFF000000),
    .INIT_23(256'h03FFFF00007FFFE00003FFFF00000FFFFF00000FFFFF800003FFFFF000001FFF),
    .INIT_24(256'h3FFC001FFF0007FFC000FFFC000FFFC000FFFE0003FFF80007FFF80007FFF800),
    .INIT_25(256'hFFC01FF803FF003FF003FF001FF800FFE007FF001FFE003FFC007FF8007FF800),
    .INIT_26(256'hE03F807F00FE01FC03FC07F807F807F803FC03FE01FF00FF803FE00FF803FE00),
    .INIT_27(256'hF03E07E07E0FC0FC0FC0FC07E07E03F03F81F80FE07F03F80FE03F81FC07F80F),
    .INIT_28(256'h7C3E0F07C3E0F87C1F0783E0F83E0F83E0F81F07C1F83E0FC1F83E07C0F81F83),
    .INIT_29(256'h787870F0F0E1E1E1E1E1E1E1E1E1E1E1E0F0F0F078783C3E1E0F0F87C3E1F0F8),
    .INIT_2A(256'h1C71E38E1C70E3C71E3871E3870E3C78F1C3870E1C3878F1E3C3878F0E1E3C3C),
    .INIT_2B(256'h639C738E31C738E31C718E38E31C71C71C638E38E38E38E38F1C71C71C38E38F),
    .INIT_2C(256'h6339CC6318CE739CE739CE739CE738C6318E739C631CE318E718E718E718E31C),
    .INIT_2D(256'h67333998CCE6633199CCE673399CCE63319CCE63398CE63398CE6339CC63398C),
    .INIT_2E(256'h99999999999999999CCCCCCCCC666666633333399998CCCC66663333999CCCE6),
    .INIT_2F(256'h366666CCCCC99999933333366666664CCCCCCCCCD99999999999999999999999),
    .INIT_30(256'h64CC999333666CCD999332666CCC9993332666CCC999933326664CCCD9999333),
    .INIT_31(256'hB33664CD9933266CC99933664CC99B33664CC99B33664CC99B33666CCD99B326),
    .INIT_32(256'hCD99B33666CC99933266CCD99B32664CD99B32664CD9933266CCD9933666CC99),
    .INIT_33(256'h99333326666CCCD999B3336666CCC99993336664CC999B332664CC999B33666C),
    .INIT_34(256'h666666666666666666666666666CCCCCCCCCD9999999B333332666664CCCC999),
    .INIT_35(256'h331998CCC66633339999CCCCE66663333339999998CCCCCCCCC6666666666666),
    .INIT_36(256'hC6339CC6339CC63398CE7319CC673398CE673198CE673399CCE6633199CCC667),
    .INIT_37(256'h638E71CE39C738C738E718E738C738C639CE718C639CE739CE739CE739CE7318),
    .INIT_38(256'h1C70E3C71C38E3C71C78E38E38F1C71C71C71C71C71C638E38E31C71CE38E71C),
    .INIT_39(256'h0F0F0F0F0F0F0F0E1E1E3C3C7878F0E1E3C7870E1C3C78F1C3870E3C78E1C78E),
    .INIT_3A(256'h0FC1F07C1F07C1F07C1F0783E0F07C3E1F0F87C3E1F0F0787C3C1E1E1F0F0F0F),
    .INIT_3B(256'h3FC07F01FC07F01FC07E03F01F81FC0FC0FC0FC0FC0FC0F81F83F07E0FC1F03E),
    .INIT_3C(256'hFF8007FF800FFE003FF801FF801FF803FE00FF803FE01FF00FF00FF00FF01FE0),
    .INIT_3D(256'hFFFF00000003FFFFFE000003FFFFE00003FFFF0000FFFF0001FFF8001FFF0007),
    .INIT_3E(256'hFFFFF80000000000000001FFFFFFFFFFFFFFFFFFFE00000000000000007FFFFF),
    .INIT_3F(256'h8003FFE0007FFE0003FFFC0003FFFF00001FFFFF000001FFFFFF00000003FFFF),
    .INIT_40(256'h1FE03FC03FC03FC03FE01FF007FC01FF007FE007FE007FF001FFC007FF8007FF),
    .INIT_41(256'hF03E0FC1F83F07E07C0FC0FC0FC0FC0FC0FE07E03F01F80FE03F80FE03F80FF0),
    .INIT_42(256'hC3C3C3E1E1E0F0F8783C3E1F0F87C3E1F0F83C1F0783E0F83E0F83E0F83E0FC1),
    .INIT_43(256'hC78E1C78F1C3870E3C78F0E1C3878F1E1C3C7878F0F1E1E1C3C3C3C3C3C3C3C3),
    .INIT_44(256'hE39C71CE38E31C71C718E38E38E38E38E38E3C71C71C78E38F1C70E38F1C38E1),
    .INIT_45(256'h6339CE739CE739CE739CE718C639CE718C738C739C639C738C738E71CE39C718),
    .INIT_46(256'h998CCE6633199CCE673399CC663399CC673398CE6339CC67318CE7318CE7318C),
    .INIT_47(256'h9999999999998CCCCCCCCC66666673333319999CCCCE66673331998CCC666333),
    .INIT_48(256'h664CCCC99999933333366666666CCCCCCCCCD999999999999999999999999999),
    .INIT_49(256'hD99B336664CC9993336664CC999B3326664CCD999B3336666CCCD99993333266),
    .INIT_4A(256'h64CD99B3266CCD9933266CC99933666CC99933666CCD99332664CD99B33666CC),
    .INIT_4B(256'h6CCD99332664CC99B33664CC99B33664CC99B33664CD99B3266CCD9933664CC9),
    .INIT_4C(256'hCCC999933336666CCCD999B332666CCCD99B332666CCD999332664CCD99B3366),
    .INIT_4D(256'h99999999999999999999999B33333333366666664CCCCCC99999B3333266664C),
    .INIT_4E(256'h73331998CCCE6667333399998CCCCCE666666333333333199999999999999999),
    .INIT_4F(256'h398CE7318CE63398CE63398CE63319CCE633198CC6633199CCE66333998CCC66),
    .INIT_50(256'h1CE39C639C639C639CE31CE718C639CE738C6318C6318C6318C6339CE7318CE7),
    .INIT_51(256'h38E38F1C71C71C38E38E38E38E38E71C71C71CE38E39C71CE38C71CE38C718E3),
    .INIT_52(256'h0F0E1E3C3878F0E1C3878F1E3C78F1C3870E3C78E1C78E1C70E3C71E38E1C71C),
    .INIT_53(256'h83C1E0F0783C3E1E0F0F8787C3C3C3E1E1E1E1E1E1E1E1E1E1E1E1C3C3C78787),
    .INIT_54(256'hF81F83F07E0F81F03E0F81F07C0F83E0F83E0F83E0F87C1F0783E0F07C3E0F07),
    .INIT_55(256'h3F807F01FC0FE03F80FC07E03F81F80FC07E07E07F03F03F03F03E07E07E0FC0),
    .INIT_56(256'hFE00FF803FE00FF803FC01FE00FF00FF807F807F807F00FF01FE03FC07F80FE0),
    .INIT_57(256'hFF8007FF8007FF000FFE001FFC007FF003FF801FFC00FFC00FFC00FF801FF003),
    .INIT_58(256'hFF80007FFF80007FFF8000FFFE0003FFF0003FFF0003FFF0007FFC001FFF000F),
    .INIT_59(256'h001FFFFFC00000FFFFF800003FFFFC00003FFFFC0000FFFFE00007FFFE0000FF),
    .INIT_5A(256'hFFFFFC00000003FFFFFFF00000007FFFFFF8000000FFFFFFC000001FFFFFE000),
    .INIT_5B(256'h0FFFFFFFFFFFE00000000003FFFFFFFFFE0000000003FFFFFFFFC000000007FF),
    .INIT_5C(256'h00000001FFFFFFFFFFFFFFFF000000000000000FFFFFFFFFFFFFC00000000000),
    .INIT_5D(256'hFFFFFFFFE000000000000000000000001FFFFFFFFFFFFFFFFFFFFC0000000000),
    .INIT_5E(256'hFFFFFFFF800000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFF),
    .INIT_5F(256'h0000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_60(256'hFFFFFFFFFFFFC000000000000000000000000000000000000000000000000000),
    .INIT_61(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_62(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_63(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_64(256'h933666CCD99B33664CC99B33664CC99B33664CC99B32664CD9933266CC99B336),
    .INIT_65(256'h3326666CCCC999933326664CCD9993332664CCD999332666CCD99B332664CC99),
    .INIT_66(256'h66666666666666666666666CCCCCCCCCC9999999B333332666664CCCCD9999B3),
    .INIT_67(256'h9CCCE66733319998CCCC666673333319999998CCCCCCCCE66666666666666666),
    .INIT_68(256'hC67318CE7319CC67319CC67319CCE63319CCE673399CCE6633199CCC66733399),
    .INIT_69(256'hE31C639C639C639C631CE318E739C6318C739CE739CE739CE739CC6318CE7318),
    .INIT_6A(256'hC71C70E38E38E3C71C71C71C71C718E38E38E31C71C638E31C738E31C738E718),
    .INIT_6B(256'hF0F1E1C3C7870F1E3C7870E1C3870E3C78F1C3871E3871E38F1C38E1C71E38E3),
    .INIT_6C(256'h7C3E1F0F87C3C1E1F0F078783C3C3C1E1E1E1E1E1E1E1E1E1E1E1C3C3C387878),
    .INIT_6D(256'h07E07C0F81F07E0FC1F07E0F83E07C1F07C1F07C1F0783E0F87C1F0F83C1F0F8),
    .INIT_6E(256'hC07F80FE07F01FC07F03F81FC07E07F03F01F81F80FC0FC0FC0FC1F81F81F03F),
    .INIT_6F(256'h01FF007FC01FF007FC03FE01FF00FF007F807F807F80FF00FE01FC03F807F01F),
    .INIT_70(256'h007FF8007FF800FFF001FFE003FF801FFC007FE003FF003FF003FF007FE00FFC),
    .INIT_71(256'h007FFF80007FFF80007FFF0001FFFC000FFFC000FFFC000FFF8003FFE000FFF0),
    .INIT_72(256'hFFE000003FFFFF000007FFFFC00003FFFFC00003FFFF00001FFFF80003FFFF00),
    .INIT_73(256'h000003FFFFFFFC0000000FFFFFFF80000007FFFFFF0000003FFFFFE000001FFF),
    .INIT_74(256'hF000000000001FFFFFFFFFFC0000000001FFFFFFFFFC000000003FFFFFFFF800),
    .INIT_75(256'hFFFFFFFC0000000000000000FFFFFFFFFFFFFFE00000000000003FFFFFFFFFFF),
    .INIT_76(256'h000000003FFFFFFFFFFFFFFFFFFFFFFFC000000000000000000007FFFFFFFFFF),
    .INIT_77(256'h00000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000000000000),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h32664CC99933666CCD99332664CD99B32664CD99B3266CCD9933266CC9993366),
    .INIT_7E(256'h66CCCCD999933326664CCC99993336666CCC999B336664CCD99B336664CC9993),
    .INIT_7F(256'h999999999999999999999999999333333333266666664CCCCCD99999B3333666),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_8
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_8_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_8_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_8_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_8_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_8_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_8_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_8_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_8_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_8_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_8_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_8_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_8_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_8_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_8_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_0_9" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "0" *) 
  (* ram_addr_end = "32767" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "9" *) 
  (* ram_slice_end = "9" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_15(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'h0000000000000000000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFC00000000000000000000000000000000000000000000),
    .INIT_1F(256'h0000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'h0000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000),
    .INIT_21(256'h007FFFFFFFFFFFFFFFFF00000000000000000000FFFFFFFFFFFFFFFFFFFFFFC0),
    .INIT_22(256'h000000001FFFFFFFFFFFFC00000000000007FFFFFFFFFFFFFF00000000000000),
    .INIT_23(256'hFC000000007FFFFFFFFC000000000FFFFFFFFFF00000000003FFFFFFFFFFE000),
    .INIT_24(256'h3FFFFFE0000007FFFFFF0000000FFFFFFF00000003FFFFFFF800000007FFFFFF),
    .INIT_25(256'hFFFFE00003FFFFC00003FFFFE00000FFFFF800001FFFFFC000007FFFFF800000),
    .INIT_26(256'hFFC0007FFF0001FFFC0007FFF80007FFFC0003FFFE0000FFFFC0000FFFFC0000),
    .INIT_27(256'hFFC007FF800FFF000FFF0007FF8003FFC001FFF0007FFC000FFFC001FFF8000F),
    .INIT_28(256'h803FF007FC00FF801FF803FF003FF003FF001FF801FFC00FFE003FF800FFE003),
    .INIT_29(256'h7F807F00FF01FE01FE01FE01FE01FE01FF00FF007F803FC01FF00FF803FE00FF),
    .INIT_2A(256'hE07E03F01F80FC07E03F81FC07F03F80FE03F80FE03F80FE03FC07F00FE03FC0),
    .INIT_2B(256'h83E07C0FC1F83F03E07E0FC0FC1F81F81F83F03F03F03F03F01F81F81FC0FC0F),
    .INIT_2C(256'h83C1F07C1F0F83E0F83E0F83E0F83F07C1F07C1F83E0FC1F07E0F81F07E0FC1F),
    .INIT_2D(256'h87C3C1E0F0F87C3E1E0F0783C1E0F07C3E1F0F83C1F0F83C1F0F83C1F07C3E0F),
    .INIT_2E(256'h1E1E1E1E1E1E1E1E1F0F0F0F0F87878783C3C3C1E1E0F0F078783C3C1E1F0F07),
    .INIT_2F(256'h387878F0F0F1E1E1E3C3C3C78787878F0F0F0F0F1E1E1E1E1E1E1E1E1E1E1E1E),
    .INIT_30(256'h870F1E1C3C7870F1E1E3C3878F0F1E1C3C3878F0F1E1E3C3C7878F0F1E1E1C3C),
    .INIT_31(256'hC3C7870E1E3C3870F1E1C3878F0E1C3C7870F1E3C3878F0E1C3C7870F1E1C3C7),
    .INIT_32(256'hF1E1C3C7870F1E1C3C78F0E1E3C3878F1E1C3C7870E1E3C3870F1E1C3878F0E1),
    .INIT_33(256'h1E3C3C387870F0E1E1C3C387870F0E1E1C3C7878F0E1E3C3C7870F1E1C3C7870),
    .INIT_34(256'h7878787878787878787878787870F0F0F0F0E1E1E1E1C3C3C3C787878F0F0E1E),
    .INIT_35(256'hC3E1E0F0F8783C3C1E1E0F0F078783C3C3C1E1E1E0F0F0F0F0F8787878787878),
    .INIT_36(256'h07C3E0F83C1F07C3E0F07C1E0F87C3E0F0783E1F0F87C3E1F0F87C3E1E0F0787),
    .INIT_37(256'h7C0F81F03E07C0F83F07E0F83F07C0F83E0F81F07C1F07C1F07C1F07C1F07C1F),
    .INIT_38(256'h1F80FC07E03F03F81F80FC0FC0FE07E07E07E07E07E07C0FC0FC1F81F03F07E0),
    .INIT_39(256'hF00FF00FF00FF00FE01FC03F807F00FE03F807F01FC07F01FC07F03F80FE07F0),
    .INIT_3A(256'hF001FF801FF801FF801FF803FF007FC01FF007FC01FF007F803FE01FE00FF00F),
    .INIT_3B(256'h3FFF8001FFF8001FFF8003FFE001FFF000FFF000FFF000FFE003FF800FFE003F),
    .INIT_3C(256'hFFFFF800000FFFFFC00001FFFFE00003FFFF00003FFFE0000FFFF0000FFFE000),
    .INIT_3D(256'h000000000003FFFFFFFFFFFC0000000003FFFFFFFF00000001FFFFFFE0000007),
    .INIT_3E(256'h000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000),
    .INIT_3F(256'h8000001FFFFFFE00000003FFFFFFFF0000000000FFFFFFFFFFFF000000000000),
    .INIT_40(256'h001FFFC0003FFFC0001FFFF00003FFFF00001FFFFE00000FFFFFC000007FFFFF),
    .INIT_41(256'hF001FFC007FF001FFC003FFC003FFC003FFE001FFF0007FFE0007FFE0007FFF0),
    .INIT_42(256'hC03FC01FE01FF007F803FE00FF803FE00FF803FF007FE007FE007FE007FE003F),
    .INIT_43(256'h3F81FC07F03F80FE03F80FE03F807F01FC03F807F00FE01FC03FC03FC03FC03F),
    .INIT_44(256'h1F83F03E07E0FC0FC0F81F81F81F81F81F81FC0FC0FC07E07F03F01F80FC07E0),
    .INIT_45(256'hE0F83E0F83E0F83E0F83E0F83E07C1F07C0F83F07C1F83F07C0F81F03E07C0F8),
    .INIT_46(256'h8783C1E1F0F87C3E1F0F87C3E1F0783C1F0F87C1E0F83C1F0F83E0F07C1F0F83),
    .INIT_47(256'h7878787878787C3C3C3C3C1E1E1E0F0F0F078783C3C1E1E0F0F0787C3C1E1F0F),
    .INIT_48(256'hE1C3C3C787878F0F0F0E1E1E1E1C3C3C3C3C3878787878787878787878787878),
    .INIT_49(256'h3878F0E1E3C3878F0F1E1C3C7878F0E1E1C3C387870F0E1E1C3C387870F0F1E1),
    .INIT_4A(256'h1C3C7870E1E3C3870F1E1C3878F0E1E3C7870F1E1C3C78F0E1E3C3878F0E1E3C),
    .INIT_4B(256'h70F1E1C3C7870F1E3C3878F0E1C3C7870F1E3C3878F1E1C3C78F0E1E3C7870F1),
    .INIT_4C(256'h0F0E1E1C3C387870F0E1E1C3C3878F0F1E1C3C3878F0E1E1C3C7870F1E1C3C78),
    .INIT_4D(256'h1E1E1E1E1E1E1E1E1E1E1E1C3C3C3C3C3878787870F0F0F1E1E1C3C3C387878F),
    .INIT_4E(256'h7C3C1E1F0F0F8787C3C3E1E1F0F0F0F878787C3C3C3C3C1E1E1E1E1E1E1E1E1E),
    .INIT_4F(256'h3E0F07C1F0F83C1F0F83C1F0F83C1E0F07C3E1F0F87C3E1E0F0783C3E1F0F078),
    .INIT_50(256'h1F03E07C1F83E07C1F03E0F81F07C1F07C0F83E0F83E0F83E0F83C1F07C1F0F8),
    .INIT_51(256'h3F03F01F81F81FC0FC0FC0FC0FC0F81F81F81F03F03E07E0FC0F81F03F07E0FC),
    .INIT_52(256'hF00FE03FC07F00FE03F80FE03F80FE03F80FC07F01F80FE07F03F81FC0FE07E0),
    .INIT_53(256'h03FE00FF803FC01FF00FF807FC03FC01FE01FE01FE01FE01FE01FE03FC07F807),
    .INIT_54(256'hFFE003FF800FFE003FF001FF800FFC00FFC00FFC00FF801FF803FF007FC00FF8),
    .INIT_55(256'h3FFF8001FFF0003FFF0007FFC001FFF0007FF8007FFC003FFC003FF8007FF000),
    .INIT_56(256'hFFFF00003FFFF00003FFFE0000FFFF00007FFF80007FFF0001FFFC0007FFF000),
    .INIT_57(256'hFFFFF8000007FFFFF000001FFFFF800003FFFFE00000FFFFF00000FFFFE00003),
    .INIT_58(256'h0000007FFFFFFF80000000FFFFFFFC0000003FFFFFFC0000007FFFFFE000000F),
    .INIT_59(256'hFFE00000000000FFFFFFFFFFC0000000003FFFFFFFFF0000000007FFFFFFFF00),
    .INIT_5A(256'hFFFFFFFFFFFFFC000000000000007FFFFFFFFFFFFF0000000000001FFFFFFFFF),
    .INIT_5B(256'hF00000000000000000000003FFFFFFFFFFFFFFFFFFFC000000000000000007FF),
    .INIT_5C(256'hFFFFFFFE0000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5D(256'h000000000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_5E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000000000000),
    .INIT_5F(256'h0000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h8F0E1E3C3878F0E1C3C7870F1E3C3878F0E1C3C7870E1E3C3870F1E1C3878F0E),
    .INIT_65(256'hF0E1E1E3C3C7878F0F1E1E3C3C7870F0E1E3C3C7870F1E1E3C3878F0E1E3C387),
    .INIT_66(256'hE1E1E1E1E1E1E1E1E1E1E1E3C3C3C3C3C78787878F0F0F1E1E1E3C3C3C787870),
    .INIT_67(256'h83C3E1E0F0F078783C3C1E1E0F0F0F07878787C3C3C3C3E1E1E1E1E1E1E1E1E1),
    .INIT_68(256'hC1F0F83E0F07C3E0F07C3E0F07C3E1F0F83C1E0F0783C1E1F0F87C3C1E0F0F87),
    .INIT_69(256'hE0FC1F83E07C1F83E0FC1F07E0F83E0F83F07C1F07C1F07C1F07C3E0F83E0F07),
    .INIT_6A(256'hC0FC0FE07E07E03F03F03F03F03F07E07E07E0FC0FC1F81F03F07E0FC0F81F07),
    .INIT_6B(256'h0FF01FC03F80FF01FC07F01FC07F01FC07F03F80FE07F01F80FC07E03F01F81F),
    .INIT_6C(256'hFC01FF007FC03FE00FF007F803FC03FE01FE01FE01FE01FE01FE03FC03F807F8),
    .INIT_6D(256'h001FFC007FF001FFC00FFE007FE003FF003FF003FF007FE007FC00FF803FF007),
    .INIT_6E(256'hC0007FFE000FFFC000FFF8003FFE000FFF0007FF8003FFC003FFC007FF800FFF),
    .INIT_6F(256'h0000FFFFC0000FFFFC0001FFFF0000FFFF80007FFF8000FFFE0003FFF8000FFF),
    .INIT_70(256'h000007FFFFF800000FFFFFE000007FFFFC00001FFFFF00000FFFFF00001FFFFC),
    .INIT_71(256'hFFFFFF800000007FFFFFFF00000003FFFFFFC0000003FFFFFF8000001FFFFFF0),
    .INIT_72(256'h001FFFFFFFFFFF00000000003FFFFFFFFFC000000000FFFFFFFFF800000000FF),
    .INIT_73(256'h00000000000003FFFFFFFFFFFFFF80000000000000FFFFFFFFFFFFE000000000),
    .INIT_74(256'h0FFFFFFFFFFFFFFFFFFFFFFC00000000000000000003FFFFFFFFFFFFFFFFF800),
    .INIT_75(256'h00000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE0000000000000000000000000),
    .INIT_76(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000000),
    .INIT_77(256'h00000000000000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFF),
    .INIT_78(256'hFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000000000000000000000000000),
    .INIT_79(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_7D(256'h0E1E3C3878F0E1E3C3870F1E1C3C7870E1E3C3878F1E1C3C78F0E1E3C7870F1E),
    .INIT_7E(256'hE1C3C3C7878F0F1E1E3C3C7878F0F1E1E3C387870F1E1C3C3878F0E1E3C3878F),
    .INIT_7F(256'h878787878787878787878787878F0F0F0F0F1E1E1E1E3C3C3C38787870F0F1E1),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("LOWER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_0_9
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(1'b1),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(q0_reg_0_9_n_0),
        .CASCADEOUTB(NLW_q0_reg_0_9_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_0_9_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_0_9_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO(NLW_q0_reg_0_9_DOADO_UNCONNECTED[31:0]),
        .DOBDO(NLW_q0_reg_0_9_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_0_9_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_0_9_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_0_9_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_0_9_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_0_9_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_0_9_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_0_9_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_0_9_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_0" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "0" *) 
  (* ram_slice_end = "0" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hAD330FF8E69556CC7FFC66D556D8E003CC96A96CC3FFC736B56B271FFF1CDB55),
    .INIT_01(256'h5498FFE335B5B3801E6D55263FF8EDAA933800E652B49C7FC336AA931E03CCD2),
    .INIT_02(256'h1FF1DA52CC7F865AA4CF03CDAAA670079255B31FE392AA4E3FC3255498FFE325),
    .INIT_03(256'hCFFF32D691C071A54B3C079A54B3C079B55B3801C9696C7FE3255267FF192AD3),
    .INIT_04(256'h299E0F329AD8FF8CD5599FF8CB52CE00E6D56CE01CCAA930F86694B3000CDAA4),
    .INIT_05(256'h6AA4C3F0C95499FFC6D55B1FF8CB52CE00725549C0066D5263FF1B55663FE36B),
    .INIT_06(256'h34A96700732AACC7F8CD6D270019955B38039B55B38039B55B3C07125699FFE3),
    .INIT_07(256'h2556C701C4956DC3E1DB5491E0F3255B3020E4B6B31FE3B6ADB8FE192AB6700F),
    .INIT_08(256'hAACC7FE3694B71FF192AB67003925493801CDAA931FF1DA5ACC3F0CD696E3FE3),
    .INIT_09(256'h27000E4AAB31FF19A52CC7FC64AAD980066D5498FF8CD29663FE335549C00392),
    .INIT_0A(256'h1FF1DA5ACC3F0CD696E3FE32556CE00724A9270039B55263FE3B4A5B1FF8CD55),
    .INIT_0B(256'hC039B55261FC76D5B71FE335B49C10336A933C1E24AB6E1F0EDAA48E038DAA93),
    .INIT_0C(256'h1FFE65A92380F36AB6700736AB6700736AA6600392DACC7F8CD55338039A54B3),
    .INIT_0D(256'h5B1FF19AAB63FF192AD9800E4AA93801CD2B4C7FE36AAD8FFE64AA4C3F0C955B),
    .INIT_0E(256'h956CC00334A5987C32554CE01CDAAD9C01CD2B4C7FE66AACC7FC6D6533C1E653),
    .INIT_0F(256'h2D5263FF992A931FF8DA5A4E00736AB6780F34A96780F34A96380E25AD33FFCC),
    .INIT_10(256'h931FFC64AA930FF1C955271FE336A92780399556CF03CC956987F8CD296E3FE3),
    .INIT_11(256'h2CCF01E32555B30FF8E4B5299C00732556DC7FF192AAD9E00736B6B31FFC64AA),
    .INIT_12(256'hAB6CE3FFE3935AB5B38FFF0CDA55A4CF001C6DAAAD98FFF8CDAAA59C7FC332D5),
    .INIT_13(256'hB26387FFF0E324B5552D99C7FFF8626D2AAD2671FFFE3136AAADB31F81F19B6A),
    .INIT_14(256'h666C9695AAAA94B6D998C381FFF81C71936D2AAAA526CC70FC03F0E66495AA94),
    .INIT_15(256'h9694B5A5295AD5AB55AAAAD54AAA956B5A5A493664C6638E1E0FF0003FC1E1CE),
    .INIT_16(256'hF81F80FC07E03F807F00FF00FF00FF803FF003FF001FFC003FFE0007FFE0000F),
    .INIT_17(256'h8E3C71E3871E3C78F0E1E1C3C3C3C3C3E1E1F0F87C1E0F83E0F83F07C0F81F81),
    .INIT_18(256'hCC999333366666666633333998CCE63398CE7318C639CE31CE31C738E38E38E3),
    .INIT_19(256'h55AA954AB56AD4AD6B5AD6B4B4A5B4B496D24B6DB6DB6DB249B64D9364D9B266),
    .INIT_1A(256'h64CC99364DB2492492DB4B4B4B4A5295A95AB55AAD552AAAA555555555AAAAA5),
    .INIT_1B(256'h3C78783E0FE03FF00003FFC00007FE01F81F87C3C3C78E38E39C6319CCC66666),
    .INIT_1C(256'hCCCC9926DA4B6B4AD4AB5555555554AB56A5AD2DA4924D936664C666339C638E),
    .INIT_1D(256'h6B52A955556AB529692493664C66318E1C3C1F803FFFFFFC01F83E1C38E318CC),
    .INIT_1E(256'h8C666C9B4B5A955555AB5A5B6CD998CE38787E001FE003F07871CE6332649B49),
    .INIT_1F(256'hD4B6D9B98C783FC00FF078E73326DA5AB55552B5A49B3339C70F803FFC01F0F1),
    .INIT_20(256'hAAAB524999CF0FFFFF878C66C96B55556B493331C780FFF81F1CE66C92D4AAAA),
    .INIT_21(256'hCCE3E000F8E664B52AD5B6CC61F0003E3999B4AD54AD26CC71F8001F0E6664B5),
    .INIT_22(256'h256AD49B9C3FFE1CEC94AA94999C3FFF87332D2AA96999C3F07F0C66DAD556B6),
    .INIT_23(256'h76D2AD6CC700079996AAB6CC700038CDA555A4CE1FFF8E6694AD4931C1FF073B),
    .INIT_24(256'hAAB4CE1FC199296B6C7801E66D55699C3FE1CDB554B3381E0E36D555B338000E),
    .INIT_25(256'hFF8CDAAA4CF007196AAD98FFF19B555B38FF8665AA5B383C1CC955499C000E6D),
    .INIT_26(256'hFFCCD2B4CC1C1CDAAB663FE39296B31FFE335AD263FFC64AAA4C7FFCEDAAB663),
    .INIT_27(256'hA4E000CDAAB238071B554987F864A6B67003892A933803892AB661FC334AB6C7),
    .INIT_28(256'h39A54B31FF19B55B30FE192D299C00E6D553300066D55B3C03CDAAA660F0E4B5),
    .INIT_29(256'h3FE192D4B31FFC6695A4E3F87255533078336AA4CE00E64AA48E00736AA9983C),
    .INIT_2A(256'h79B6AA49C7FF1995B5B30FF8E4955B33FFF1B6AAD9C3F8E6D55B31FF8E4AAA4E),
    .INIT_2B(256'h3652AD64E3E0F8E49555A6707E0E64A5293387E0E64AAA49C7FF0CDAD2933800),
    .INIT_2C(256'hAAB49B187C00F8E6496AAB49B9C1FF8399B6AAAD36387FF0E6695B56CCE0FF06),
    .INIT_2D(256'h78E73326D2D6AAAAD6924CC63C3FFFFC1C7326D29555ADB6631F0000F1CCDB4A),
    .INIT_2E(256'h49B6DB6DB6DA4B69696B5A952AB5555556AB529692493367338E3C1F800001F8),
    .INIT_2F(256'h000007FFFFFFC0000007FFFFFC000003FFFFF800003FFFFE00001FFFFC0000FF),
    .INIT_30(256'h000000000001FFFFFFFFFFFE00000000003FFFFFFFFF8000000007FFFFFFFC00),
    .INIT_31(256'h000003FFFFFFFFFFFFFFFFFFFFFF0000000000000000003FFFFFFFFFFFFFFF00),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF800000000000000000000000),
    .INIT_33(256'h000000000000000000000000000000000000000000000000000000000000001F),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'h00000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hE000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'h03FFFFFFFFFFFFFFF0000000000000000003FFFFFFFFFFFFFFFFFFFFFF000000),
    .INIT_46(256'h00FFFFFFFF8000000007FFFFFFFFF00000000001FFFFFFFFFFFE000000000000),
    .INIT_47(256'hFC0000FFFFE00001FFFFF000007FFFFF000000FFFFFF8000000FFFFFFF800000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_0
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_0_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_0_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_0_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_0_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_0_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_0_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[0]}),
        .DOBDO(NLW_q0_reg_1_0_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_0_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_0_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_0_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_0_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_0_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_0_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_0_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_0_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_1" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "1" *) 
  (* ram_slice_end = "1" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h9B695AAD4B26670F8003E1CCCDB5AAAA96DB318F03FFC0F18CD96D4AAA569266),
    .INIT_01(256'hCDB5AAB5A6C63C0001E3336D6AAD49331C3FFFE1CE6DB52A95A4CCE3E0003C31),
    .INIT_02(256'h1FF039C9A52AD49338F0003C666D2AAADB663C1FE07199256A95B6671F001F1C),
    .INIT_03(256'hA555A498E1FFF0632695552C98C3FFF8733695556DB18F801F1CC92D55ADB31C),
    .INIT_04(256'hCE1FFF0E764A55296661E007C7365AAAB4998F0003C66495AD4B273C0003C66D),
    .INIT_05(256'h199256A56D98E1FFC1CCC94AAD6D9CF0000E3324AAAB499C7C00F8CCD2954A4D),
    .INIT_06(256'h593187FFF0E665AD52964E3800078CC96AAAD2663C00078CC96AAA5B671E001F),
    .INIT_07(256'hE33252AA96D98E03E038CDB4AAA5B663C0201C72694AB524CE3F01F8E66D2AAA),
    .INIT_08(256'hCCF07FE0E726D4AA524CC7800071CDB6AAA96CCE3E00FC6365A95A964E703FE0),
    .INIT_09(256'h92AAAB6CCC3E00F86365AD56B6CCE1FFFE1CCDB5AAD69B187C01F0CCDB555524),
    .INIT_0A(256'h1FF039C9A56A569B18FC01F1CCDA5555B6CE3800078CC92954AD939C1FF83CCC),
    .INIT_0B(256'h5552D99C7E03F1CC92B54A5938E0100F19B69554B6CC701F01C66DA55529331C),
    .INIT_0C(256'hE001E39B69555A4CC78000F1992D555A4CC7800071C9A52AD6999C3FFF863269),
    .INIT_0D(256'hC94AA52CCC7C00F8E64B55549331C0003CE6DAD54A4CCE0FFE1C66DA95A92663),
    .INIT_0E(256'hD98F0000F3934AD6A4998F0003C664B55569B38F801E199A52A949B9C3FFE1CE),
    .INIT_0F(256'hE336D6AAD24CE3E007C636DAAAA5B3387FFF0C64D2AAA593183FFE1C6496AA96),
    .INIT_10(256'hE3E003E399B6A55A9266381FE0F19B6D5552D998F0003C7324AD52964E703FE0),
    .INIT_11(256'h30F0001F1CCC96A552B6D9CE1FFFF0E3324AD55ADB331E0000F18D96B556B6CC),
    .INIT_12(256'h9925A9554ADA6CC63C0FFF03C6336DA55556B6CCCE1F0007C399934AD56A5B66),
    .INIT_13(256'h96D6AD555AB5B6D999CE1E07FFF81E1CE66492D4AAAB5A5B3331C3E0000F8719),
    .INIT_14(256'hE1E38E739999B26DB4B5A954AAAD56A525B64CCCC638F07F00000FE1E38C664D),
    .INIT_15(256'h24D926C9B26C993266CCCC998CCCE6739C638E3878F87C0FE00FFFFFFFC01FC1),
    .INIT_16(256'hF8007FFC001FFF8000FFFF0000FFFF80000FFFFF000003FFFFFE0000001FFFFF),
    .INIT_17(256'h7E03F01F80FE03F80FE01FC03FC03FC01FE00FF803FE007FE007FF003FF8007F),
    .INIT_18(256'hC3878F0F0E1E1E1E1E0F0F0787C3E1F0783E0F07C1F83E0FC1F03F07E07E07E0),
    .INIT_19(256'h33998CC67319CC6318C6318C739C738C71CE38E38E38E38E3871C38F1C3871E1),
    .INIT_1A(256'hD25A4B6D2496DB6DB64926D926D9364C9B366CC99B3366666CCCCCCCCC666663),
    .INIT_1B(256'h56AD52AB554AAAA5555555555552AAAB554AAD56A952A56A56B5294B5A52D2D2),
    .INIT_1C(256'hF0F0E1C71C738C7318CC6666666666CD9B36C9B6C92496DA4B496B4B5AD6B52B),
    .INIT_1D(256'hD93664CCCCE67318E71C70E1C3E1F07E03FC007FFFFFFFFFFE003FE03F03E0F0),
    .INIT_1E(256'h0F878F1C739CE66666CD936DB692D294AD52AB55555556AAD52B5AD696D24924),
    .INIT_1F(256'h992492D2D6AD556AA5552A52969249366CCCCE739C78F0F83F007FFFFFFE00FE),
    .INIT_20(256'h33326492D295AAAAAAAD5AD25B26CCCCE738F0F03F8000001FE0F870E318CCCC),
    .INIT_21(256'hC3E01FFF00F878C63366DB694AA5556A94B49264CC631E3C0FF8001FF07878C6),
    .INIT_22(256'h398C992D2955554A5A4D998C787C000007C3CE33324D2D6AA52AA52DB64CCE71),
    .INIT_23(256'h78E336496A5552B4B26671C3F0003F0E39993694AAAAA52DB26338F03FFFF83C),
    .INIT_24(256'h6673C1FFFE1E31B24952AB52DB33187C0001F1C666DA52AAA56DB3338F07FFF0),
    .INIT_25(256'hFFF0E333695AAD4B266387FFFE1C666DAD5552D36638F8001F0E666D29555ADB),
    .INIT_26(256'hFFF0E32696AAB5B6671E0003E31B25AAAA96C9CE1FFFF87333695555A4998E1F),
    .INIT_27(256'h9C1FFF0E3324AD52B6CCC7800078C4DB5AA95B6670F803F1CCDB4AAA96D98E3F),
    .INIT_28(256'h6B6CC70FFFE1C66DA5554B64E783FF07199A5AAAD24CC703FC0E3334B555AD93),
    .INIT_29(256'hAAAB49B38F000078E6C94AAAD6CCCF0FFFC38CC96B55AD266381FF838CCD2D55),
    .INIT_2A(256'h2B6D99C7C0001E19D925AAAA524CC70FFFFE38CC929555ADB338F0000F8CCC94),
    .INIT_2B(256'h386336494AB5AA524CCC61F0000F8739B25AD555AD2666383FFFF0E31B25AD55),
    .INIT_2C(256'h3326D252A955AA52DB266738783FFFFC1E38CCC9A4AD5555AD24C8CE3C1FFFF8),
    .INIT_2D(256'h80F83C38E318CCCC9B249694A95555554AD69249B3339C71E0FF0000FE0F1C73),
    .INIT_2E(256'h24924924924926DB24D9364C999333333198CE718E38F0E0F07E03FF800001FF),
    .INIT_2F(256'hFFFFF800000000000007FFFFFFFFFFFC00000000003FFFFFFFFFE000000000FF),
    .INIT_30(256'h000000000001FFFFFFFFFFFFFFFFFFFFFFC0000000000000000007FFFFFFFFFF),
    .INIT_31(256'h000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE0),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'h1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h00000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000),
    .INIT_46(256'hFFFFFFFFFF8000000000000000000FFFFFFFFFFFFFFFFFFFFFFE000000000000),
    .INIT_47(256'hFC000000001FFFFFFFFFF00000000000FFFFFFFFFFFF800000000000007FFFFF),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_1
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_1_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_1_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_1_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_1_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_1_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_1_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[1]}),
        .DOBDO(NLW_q0_reg_1_1_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_1_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_1_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_1_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_1_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_1_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_1_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_1_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_1_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_10" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "10" *) 
  (* ram_slice_end = "10" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFC01FF00FF803FC01FE00FF007F807FC03FE01FE00FF00FF00FF807F807F807F),
    .INIT_01(256'hF803FF003FE007FC00FF801FF007FC00FF803FE00FF803FE00FF803FE00FF807),
    .INIT_02(256'h7FF001FFC007FF003FF800FFC007FE003FF001FF801FF801FF801FF801FF801F),
    .INIT_03(256'hE000FFF8003FFC001FFF000FFF0007FF8007FF8007FF800FFF001FFE003FF800),
    .INIT_04(256'h000FFFF0000FFFE0001FFFC0007FFF0003FFF8001FFF8001FFF8003FFF0007FF),
    .INIT_05(256'h0001FFFFE00001FFFFE00003FFFF80001FFFF80001FFFF80003FFFE0000FFFF0),
    .INIT_06(256'hC0000001FFFFFFE0000003FFFFFE000000FFFFFF000000FFFFFC00000FFFFFC0),
    .INIT_07(256'hFFFFFFFFFFF00000000001FFFFFFFFFC000000003FFFFFFFF00000000FFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFC0000000000000000000003FFFFFFFFFFFFFFFE00000000000007),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'h80000000000001FFFFFFFFFFFFFFFF0000000000000000000000FFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFC00000003FFFFFFFF000000000FFFFFFFFFE00000000003FFFFFFFFFFF),
    .INIT_0C(256'h0FFFFFC00000FFFFFC000003FFFFFC000001FFFFFF0000001FFFFFFE0000000F),
    .INIT_0D(256'h3FFFC0001FFFF00007FFFE00007FFFE00007FFFF00001FFFFE00001FFFFE0000),
    .INIT_0E(256'hFF8003FFF0007FFE0007FFE0007FFF0003FFF8000FFFE0001FFFC0003FFFC000),
    .INIT_0F(256'h007FF001FFE003FFC007FF8007FF8007FF8003FFC003FFE000FFF0007FFC001F),
    .INIT_10(256'hE007FE007FE007FE007FE007FE003FF001FF800FFC007FF003FF800FFE003FF8),
    .INIT_11(256'h807FC01FF007FC01FF007FC01FF007FC00FF803FE007FC00FF801FF003FF007F),
    .INIT_12(256'hF807F807F807FC03FC03FC01FE01FF00FF807F803FC01FE00FF007FC03FE00FF),
    .INIT_13(256'h1FC03FC07F807F00FF01FE01FE03FC03FC03F807F807F807F807F807F807F807),
    .INIT_14(256'h07F80FE01FC07F80FF01FC03F807F01FE03FC07F80FF01FE03FC07F80FF00FE0),
    .INIT_15(256'hFC03F80FE01FC07F00FE03F807F01FE03F80FF01FC03F80FE01FC07F80FE01FC),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'h0000000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'h007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000000),
    .INIT_1F(256'h000000001FFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000),
    .INIT_20(256'hFFFFFFFFFF8000000000000000000FFFFFFFFFFFFFFFFFFFFC00000000000000),
    .INIT_21(256'h3FFFFFFFFFFFF80000000000000FFFFFFFFFFFFFFC0000000000000007FFFFFF),
    .INIT_22(256'hFFFFFFFFFC0000000003FFFFFFFFFF00000000001FFFFFFFFFFF800000000000),
    .INIT_23(256'hFFFFFF80000001FFFFFFF800000007FFFFFFFC00000000FFFFFFFFF000000000),
    .INIT_24(256'hFFFFF800000FFFFFF0000007FFFFF8000001FFFFFF8000000FFFFFFE0000000F),
    .INIT_25(256'h003FFFF80000FFFFE00003FFFFC00003FFFFC00001FFFFF000003FFFFF000003),
    .INIT_26(256'h7FFF8000FFFE0001FFFE0001FFFE0001FFFF00007FFFC0001FFFF00003FFFF00),
    .INIT_27(256'h007FFC001FFF0007FFC001FFF8003FFF0003FFF0003FFF0003FFF8000FFFC000),
    .INIT_28(256'h03FF800FFE003FFC007FF000FFE001FFE003FFC003FFC003FFE001FFE000FFF0),
    .INIT_29(256'hFC01FF801FF003FF003FF003FF003FF801FF801FFC00FFE007FF001FF800FFE0),
    .INIT_2A(256'hF803FE00FF007FC01FF007FC01FF007FC01FF003FE00FFC01FF003FE007FC00F),
    .INIT_2B(256'h1FE01FE01FE01FE01FF00FF00FF807F803FC03FE01FF00FF807FC03FE01FF007),
    .INIT_2C(256'hC07F80FF00FE01FE03FC03F807F807F00FF00FF01FE01FE01FE01FE01FE01FE0),
    .INIT_2D(256'hF80FE01FC07F80FE01FC03F80FF01FE03FC07F00FE01FC03F807F00FE01FC03F),
    .INIT_2E(256'h03F807F01FC03F80FE01FC07F00FE03F807F01FC03F80FF01FC07F80FE01FC07),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_47(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_10
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_10_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_10_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_10_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_10_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_10_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_10_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[10]}),
        .DOBDO(NLW_q0_reg_1_10_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_10_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_10_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_10_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_10_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_10_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_10_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_10_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_10_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_11" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "11" *) 
  (* ram_slice_end = "11" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFC0000FFFF80003FFFE0000FFFF80003FFFE0001FFFF0000FFFF80007FFF8000),
    .INIT_01(256'hF80000FFFFE00003FFFF80000FFFFC00007FFFE00007FFFE00007FFFE00007FF),
    .INIT_02(256'hFFF000003FFFFF000007FFFFC00001FFFFF000007FFFF800007FFFF800007FFF),
    .INIT_03(256'h1FFFFFF8000003FFFFFF000000FFFFFF8000007FFFFF800000FFFFFE000007FF),
    .INIT_04(256'h0000000FFFFFFFE00000003FFFFFFF00000007FFFFFF80000007FFFFFF000000),
    .INIT_05(256'hFFFFFFFFE0000000001FFFFFFFFF8000000007FFFFFFFF800000001FFFFFFFF0),
    .INIT_06(256'hC00000000000001FFFFFFFFFFFFE000000000000FFFFFFFFFFFC00000000003F),
    .INIT_07(256'h00000000000FFFFFFFFFFFFFFFFFFFFC00000000000000000FFFFFFFFFFFFFFF),
    .INIT_08(256'h000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'h00000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000),
    .INIT_0B(256'hFFFFFFFFFFFFFFC00000000000000000FFFFFFFFFFFFFFFFFFFFC00000000000),
    .INIT_0C(256'hF00000000000FFFFFFFFFFFC000000000001FFFFFFFFFFFFE00000000000000F),
    .INIT_0D(256'h3FFFFFFFE000000007FFFFFFFF8000000007FFFFFFFFE0000000001FFFFFFFFF),
    .INIT_0E(256'h000003FFFFFF80000007FFFFFF80000003FFFFFFF00000001FFFFFFFC0000000),
    .INIT_0F(256'hFF800001FFFFFC000007FFFFF8000007FFFFFC000003FFFFFF0000007FFFFFE0),
    .INIT_10(256'hFFF800007FFFF800007FFFF800003FFFFE00000FFFFF800003FFFFF000003FFF),
    .INIT_11(256'hFF80001FFFF80001FFFF80001FFFF80000FFFFC00007FFFF00001FFFFC00007F),
    .INIT_12(256'h0007FFF80007FFFC0003FFFE0001FFFF00007FFFC0001FFFF00007FFFC0000FF),
    .INIT_13(256'hE0003FFF80007FFF0001FFFE0003FFFC0003FFF80007FFF80007FFF80007FFF8),
    .INIT_14(256'h07FFF0001FFF8000FFFE0003FFF8001FFFC0007FFF0001FFFC0007FFF0000FFF),
    .INIT_15(256'hFFFC000FFFE0007FFF0003FFF8001FFFC000FFFE0003FFF0001FFF8000FFFE00),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE000000000000000000000000000000000),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'h0000000000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000),
    .INIT_20(256'hFFFFFFFFFF8000000000000000000000000000000000000003FFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFF80000000000000000000000000003FFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'h0000000003FFFFFFFFFFFFFFFFFFFF00000000000000000000007FFFFFFFFFFF),
    .INIT_23(256'h0000007FFFFFFFFFFFFFF80000000000000003FFFFFFFFFFFFFFFFF000000000),
    .INIT_24(256'hFFFFF800000000000FFFFFFFFFFFF80000000000007FFFFFFFFFFFFE00000000),
    .INIT_25(256'hFFFFFFF8000000001FFFFFFFFFC0000000003FFFFFFFFFF00000000000FFFFFF),
    .INIT_26(256'hFFFF80000001FFFFFFFE00000001FFFFFFFF000000003FFFFFFFF000000000FF),
    .INIT_27(256'h000003FFFFFF0000003FFFFFF8000000FFFFFFF0000000FFFFFFF80000003FFF),
    .INIT_28(256'hFFFF800001FFFFFC00000FFFFFE000001FFFFFC000003FFFFFE000001FFFFFF0),
    .INIT_29(256'h03FFFF80000FFFFF00000FFFFF000007FFFF800003FFFFE00000FFFFF800001F),
    .INIT_2A(256'h07FFFE0000FFFFC0000FFFFC0000FFFFC0000FFFFE00003FFFF00001FFFFC000),
    .INIT_2B(256'hFFE0001FFFE0001FFFF0000FFFF80007FFFC0001FFFF00007FFFC0001FFFF000),
    .INIT_2C(256'h3FFF8000FFFE0001FFFC0007FFF8000FFFF0000FFFE0001FFFE0001FFFE0001F),
    .INIT_2D(256'hF8001FFFC0007FFE0003FFF8000FFFE0003FFF0001FFFC0007FFF0001FFFC000),
    .INIT_2E(256'h0007FFF0003FFF8001FFFC000FFFE0007FFF0003FFF8000FFFC0007FFE0003FF),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_11
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_11_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_11_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_11_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_11_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_11_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_11_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[11]}),
        .DOBDO(NLW_q0_reg_1_11_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_11_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_11_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_11_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_11_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_11_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_11_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_11_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_11_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_12" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "12" *) 
  (* ram_slice_end = "12" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h03FFFFFFFF800000001FFFFFFFF800000001FFFFFFFF000000007FFFFFFF8000),
    .INIT_01(256'hF8000000001FFFFFFFFF8000000003FFFFFFFFE000000001FFFFFFFFE0000000),
    .INIT_02(256'hFFF00000000000FFFFFFFFFFC0000000000FFFFFFFFFF80000000007FFFFFFFF),
    .INIT_03(256'h00000007FFFFFFFFFFFF0000000000007FFFFFFFFFFF800000000001FFFFFFFF),
    .INIT_04(256'h000000000000001FFFFFFFFFFFFFFF000000000000007FFFFFFFFFFFFF000000),
    .INIT_05(256'h000000001FFFFFFFFFFFFFFFFFFF8000000000000000007FFFFFFFFFFFFFFFF0),
    .INIT_06(256'hC000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFC000000000000),
    .INIT_07(256'h00000000000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'h00000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000000000000000000000000),
    .INIT_0C(256'h000000000000FFFFFFFFFFFFFFFFFFFFFFFE000000000000000000000000000F),
    .INIT_0D(256'h3FFFFFFFFFFFFFFFF8000000000000000007FFFFFFFFFFFFFFFFFFE000000000),
    .INIT_0E(256'h000003FFFFFFFFFFFFF800000000000003FFFFFFFFFFFFFFE000000000000000),
    .INIT_0F(256'hFFFFFFFE000000000007FFFFFFFFFFF8000000000003FFFFFFFFFFFF80000000),
    .INIT_10(256'hFFFFFFFF80000000007FFFFFFFFFC0000000000FFFFFFFFFFC00000000003FFF),
    .INIT_11(256'h0000001FFFFFFFFE000000001FFFFFFFFF0000000007FFFFFFFFE0000000007F),
    .INIT_12(256'h0007FFFFFFF800000003FFFFFFFE000000007FFFFFFFE000000007FFFFFFFF00),
    .INIT_13(256'h00003FFFFFFF80000001FFFFFFFC00000003FFFFFFF800000007FFFFFFF80000),
    .INIT_14(256'hF80000001FFFFFFF00000003FFFFFFE00000007FFFFFFE00000007FFFFFFF000),
    .INIT_15(256'hFFFFFFF00000007FFFFFFC0000001FFFFFFF00000003FFFFFFE0000000FFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'h0000000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000),
    .INIT_20(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFF800000000000000000000000000000000000000000000000000),
    .INIT_22(256'h000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'h0000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000),
    .INIT_24(256'hFFFFF8000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFE00000000),
    .INIT_25(256'h00000007FFFFFFFFFFFFFFFFFFC00000000000000000000FFFFFFFFFFFFFFFFF),
    .INIT_26(256'h00007FFFFFFFFFFFFFFE0000000000000000FFFFFFFFFFFFFFFFF00000000000),
    .INIT_27(256'h000000000000FFFFFFFFFFFFF80000000000000FFFFFFFFFFFFFF80000000000),
    .INIT_28(256'h00007FFFFFFFFFFC00000000001FFFFFFFFFFFC000000000001FFFFFFFFFFFF0),
    .INIT_29(256'h0000007FFFFFFFFF0000000000FFFFFFFFFF80000000001FFFFFFFFFF8000000),
    .INIT_2A(256'hFFFFFE000000003FFFFFFFFC000000003FFFFFFFFE000000000FFFFFFFFFC000),
    .INIT_2B(256'hFFE00000001FFFFFFFF000000007FFFFFFFC00000000FFFFFFFFC00000000FFF),
    .INIT_2C(256'hFFFF80000001FFFFFFFC00000007FFFFFFF00000001FFFFFFFE00000001FFFFF),
    .INIT_2D(256'h07FFFFFFC0000001FFFFFFF80000001FFFFFFF00000003FFFFFFF00000003FFF),
    .INIT_2E(256'h0000000FFFFFFF80000003FFFFFFE0000000FFFFFFF80000003FFFFFFE000000),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_47(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_12
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_12_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_12_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_12_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_12_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_12_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_12_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[12]}),
        .DOBDO(NLW_q0_reg_1_12_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_12_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_12_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_12_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_12_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_12_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_12_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_12_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_12_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_13" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "13" *) 
  (* ram_slice_end = "13" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFF800000000000000007FFFFFFFFFFFFFFFF00000000000000007FFF),
    .INIT_01(256'hF80000000000000000007FFFFFFFFFFFFFFFFFE000000000000000001FFFFFFF),
    .INIT_02(256'h000FFFFFFFFFFFFFFFFFFFFFC000000000000000000007FFFFFFFFFFFFFFFFFF),
    .INIT_03(256'h00000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000),
    .INIT_04(256'h000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFF000000),
    .INIT_05(256'h00000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0),
    .INIT_06(256'h3FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'h00000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0),
    .INIT_0D(256'h3FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000),
    .INIT_0E(256'h000003FFFFFFFFFFFFFFFFFFFFFFFFFFFC000000000000000000000000000000),
    .INIT_0F(256'h00000000000000000007FFFFFFFFFFFFFFFFFFFFFFFC00000000000000000000),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFF800000000000000000000FFFFFFFFFFFFFFFFFFFFFC000),
    .INIT_11(256'hFFFFFFE000000000000000001FFFFFFFFFFFFFFFFFF80000000000000000007F),
    .INIT_12(256'hFFF80000000000000003FFFFFFFFFFFFFFFF800000000000000007FFFFFFFFFF),
    .INIT_13(256'hFFFFC000000000000001FFFFFFFFFFFFFFFC0000000000000007FFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFE000000000000003FFFFFFFFFFFFFF8000000000000007FFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFF800000000000001FFFFFFFFFFFFFFC00000000000000FFFFFF),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'h00000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_23(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000),
    .INIT_24(256'hFFFFF800000000000000000000000000000000000000000000000001FFFFFFFF),
    .INIT_25(256'h000000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'h00000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000),
    .INIT_27(256'h00000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_28(256'hFFFFFFFFFFFFFFFC00000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFF0),
    .INIT_29(256'h0000000000000000FFFFFFFFFFFFFFFFFFFF8000000000000000000007FFFFFF),
    .INIT_2A(256'h000001FFFFFFFFFFFFFFFFFC000000000000000001FFFFFFFFFFFFFFFFFFC000),
    .INIT_2B(256'h001FFFFFFFFFFFFFFFF00000000000000003FFFFFFFFFFFFFFFFC00000000000),
    .INIT_2C(256'h00007FFFFFFFFFFFFFFC000000000000000FFFFFFFFFFFFFFFE0000000000000),
    .INIT_2D(256'h000000003FFFFFFFFFFFFFF800000000000000FFFFFFFFFFFFFFF00000000000),
    .INIT_2E(256'h000000000000007FFFFFFFFFFFFFE000000000000007FFFFFFFFFFFFFE000000),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_13
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_13_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_13_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_13_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_13_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_13_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_13_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[13]}),
        .DOBDO(NLW_q0_reg_1_13_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_13_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_13_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_13_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_13_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_13_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_13_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_13_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_13_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_14" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "14" *) 
  (* ram_slice_end = "14" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFF8000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hF80000000000000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'h0000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000FFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000003FFFFFFFFFFFF),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'h00000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'hFFFFFFFFFFFF0000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h3FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFC0000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h00000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000000000000000),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000000000000000000000000007F),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000000007FFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFE00000000000000000000000000000007FFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFC000000000000000000000000000007FFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000000000000000000FFFFFF),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000000000000),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'h0000000000000000000000000000000000000000000000000000000FFFFFFFFF),
    .INIT_24(256'hFFFFF80000000000000000000000000000000000000000000000000000000000),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'h00000000000000000000000000000000000000000000000000000FFFFFFFFFFF),
    .INIT_27(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_28(256'hFFFFFFFFFFFFFFFC00000000000000000000000000000000000000000000000F),
    .INIT_29(256'h0000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2A(256'h000000000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC000),
    .INIT_2B(256'h0000000000000000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000),
    .INIT_2C(256'h00000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE0000000000000),
    .INIT_2D(256'h000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000),
    .INIT_2E(256'h00000000000000000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFE000000),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_14
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_14_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_14_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_14_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_14_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_14_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_14_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[14]}),
        .DOBDO(NLW_q0_reg_1_14_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_14_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_14_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_14_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_14_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_14_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_14_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_14_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_14_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_15" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "15" *) 
  (* ram_slice_end = "15" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFF800000000000000000000000000000000000000000000000000000),
    .INIT_01(256'h07FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'h000000000000000000000000000000000000000000000000000000000000000F),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'h00000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'hC000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'h00000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80),
    .INIT_12(256'h000000000000000000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000),
    .INIT_14(256'h000000000000000000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'h000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_21(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_22(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_23(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_24(256'hFFFFF80000000000000000000000000000000000000000000000000000000000),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_27(256'h000000000000000000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_28(256'hFFFFFFFFFFFFFFFC000000000000000000000000000000000000000000000000),
    .INIT_29(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2A(256'h0000000000000000000000000000000000000000000000000000000000003FFF),
    .INIT_2B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000),
    .INIT_2C(256'h000000000000000000000000000000000000000000000000001FFFFFFFFFFFFF),
    .INIT_2D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000000),
    .INIT_2E(256'h0000000000000000000000000000000000000000000000000000000001FFFFFF),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_47(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_15
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_15_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_15_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_15_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_15_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_15_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_15_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[15]}),
        .DOBDO(NLW_q0_reg_1_15_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_15_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_15_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_15_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_15_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_15_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_15_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_15_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_15_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_16" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "16" *) 
  (* ram_slice_end = "16" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFF800000000000000000000000000000000000000000000000000000),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'h000000000000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'h00000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000000000000),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'h000000000000000000000000000000000000000000000000000007FFFFFFFFFF),
    .INIT_13(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_20(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_21(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_22(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_23(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_24(256'hFFFFF80000000000000000000000000000000000000000000000000000000000),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_27(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_28(256'h0000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_29(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC00000000000),
    .INIT_2C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2D(256'h00000000000000000000000000000000000000000000000000000FFFFFFFFFFF),
    .INIT_2E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_16
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_16_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_16_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_16_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_16_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_16_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_16_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[16]}),
        .DOBDO(NLW_q0_reg_1_16_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_16_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_16_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_16_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_16_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_16_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_16_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_16_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_16_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_17" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "17" *) 
  (* ram_slice_end = "17" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE00000000000000),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'h00000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_10(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_11(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_20(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_21(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_22(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_23(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_24(256'hFFFFF80000000000000000000000000000000000000000000000000000000000),
    .INIT_25(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_26(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_27(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_28(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_29(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_2B(256'h00000000000000000000000000000000000000000000000000003FFFFFFFFFFF),
    .INIT_2C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_17
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_17_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_17_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_17_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_17_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_17_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_17_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[17]}),
        .DOBDO(NLW_q0_reg_1_17_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_17_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_17_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_17_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_17_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_17_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_17_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_17_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_17_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_18" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "18" *) 
  (* ram_slice_end = "18" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'h00000000000000000000000000000000000000000000000001FFFFFFFFFFFFFF),
    .INIT_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_0A(256'hFFFFFFFFFFFFFE00000000000000000000000000000000000000000000000000),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_20(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_21(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_22(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_23(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_24(256'h000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_25(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_26(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_27(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_28(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_29(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_47(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_18
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_18_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_18_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_18_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_18_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_18_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_18_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[18]}),
        .DOBDO(NLW_q0_reg_1_18_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_18_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_18_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_18_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_18_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_18_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_18_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_18_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_18_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_19" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "19" *) 
  (* ram_slice_end = "19" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_01(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_02(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_03(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_04(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_05(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_06(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_07(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_08(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_09(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_0F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_10(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_11(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_12(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_13(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_14(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_15(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_20(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_21(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_22(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_23(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_24(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_25(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_26(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_27(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_28(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_29(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_19
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_19_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_19_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_19_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_19_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_19_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_19_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[19]}),
        .DOBDO(NLW_q0_reg_1_19_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_19_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_19_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_19_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(RDEN),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_19_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_19_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_19_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_19_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_2" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "2" *) 
  (* ram_slice_end = "2" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hD24D93318C38780FFFFFE03C3C739999B2496B5AA9556AA5296DB66CCC671C78),
    .INIT_01(256'hC38C666C92529555554A5A49B3318E3C1FC0001FC1E38CE64C925A56AAAAA95A),
    .INIT_02(256'h4AA55292C9B318E3C0FFFFFC1E1CE666492D6AB54AA52DB64CE638781FFFFF03),
    .INIT_03(256'h366638E0FE000FE0E18CCC9A4A56AAAD5A5B266671C1F00000FC38E3336496B5),
    .INIT_04(256'hF01FFF01F1C63364D2D4AAAA95A4933338E1F000003E1C739B26929555556B49),
    .INIT_05(256'hF871CE6CDB4A54AA95696D93318E1F000001F0E3999924B52AAAAD6964D98C71),
    .INIT_06(256'h3494AD555AB4B6C99CE78FC000007C38E666492D6AAAAAD6924CCC6387E00000),
    .INIT_07(256'hB5A49B3318E1F003E007C38C666C92D6AA8AA95B4D9339C70FC00007E1E31999),
    .INIT_08(256'h0F007FE01F1E339936DA52AAAAA56924CCCE70F03FFFFC1F1C67364D252A954A),
    .INIT_09(256'h8E6666DA5A955552B5B6C998C70F01FFFE03C38C664DB6B52AAAA5696D9999C7),
    .INIT_0A(256'h4AA55292C9B398E3E0FFFFF03C39CCCC925A955555296DB26731E3E01FF803C3),
    .INIT_0B(256'h66631E1F80000FC38E7326CB6A554555AD24D998C70F801F003E1C63336496B5),
    .INIT_0C(256'h00001F8718CCC925AD55555AD249999C70F800000FC79CE64DB4B56AAAD4A4B2),
    .INIT_0D(256'h38C66C9A5AD55552B49266671C3E000003E1C63326DA5AA554A94B6CD9CE387C),
    .INIT_0E(256'h4B5AAAAAA525936738E1F000003E1C73332496A55554AD2C9B318E3E03FFE03E),
    .INIT_0F(256'hB5A49B331C70FC00003E0E399993696AD555A94964CCC61C1FC001FC1C7199B2),
    .INIT_10(256'h03FFFFE078719CC9B6D2954AB55AD249999CE1E0FFFFFC0F1C63364D252A954A),
    .INIT_11(256'h6A555555A96924C99CC71E0FE0000FE0F1C6333649694AAAAAA52924D998C70F),
    .INIT_12(256'h78E398CCD9B6DA52955AAA556B5A4936666738F0F01FFFFFC07870C63326C92D),
    .INIT_13(256'h8E319CCCC9936DB4B4A54AAD5552AB56B4B6DB66CCCC639C3C3E03FFFFFF80F8),
    .INIT_14(256'h4AB52B5AD2D2DB4926D93266CCCE6739C63870F0F83F007FFFFFFFE01F83E1C3),
    .INIT_15(256'h924B6DA496DA4B692DA5A5B4A5A5AD294AD6A56AD5AAD55AAAA55555556AAA95),
    .INIT_16(256'h07FFFFFC0000007FFFFFFF000000007FFFFFFFFF000000000001FFFFFFFFFFFF),
    .INIT_17(256'hFE000FFF8001FFF8001FFFC0003FFFC0001FFFF80001FFFFE00000FFFFF80000),
    .INIT_18(256'hC07F80FF01FE01FE01FF00FF803FE00FF801FF003FF801FFC00FFF001FFE001F),
    .INIT_19(256'hF0787C3E0F07C3E0F83E0F83F07C0F83F03E07E07E07E07E07F03F80FC07F01F),
    .INIT_1A(256'hCE39C71CE38E38E38E38E1C71E38F1C3870E1C3878F0E1E1E3C3C3C3C3E1E1E0),
    .INIT_1B(256'h3264C9993326666CCCCCCCCCCCCE666733399CCE67319CE6318CE738C631CE31),
    .INIT_1C(256'hAA55AB52B5295AD6B5A52D2D2D2D2DA4B6925B6DA4924DB6D924D926C9B26C99),
    .INIT_1D(256'h6DA4B69696B4A5AD4A56A54A954AA554AAA955555555555555556AAA9556AA55),
    .INIT_1E(256'h0FF80FE07C1F0787870E1C71C71CE318CE63339999999B33664D9364DB649249),
    .INIT_1F(256'h4B6DB649B264CCD99CCCE6318E71C70E1C3C3E0F83F80FF800FFFFFFFFFFFF00),
    .INIT_20(256'h3C3C78E31CE6333333366C9B6DB49696B5AD5AA5552AAAAAB555AAD5A94A5A5A),
    .INIT_21(256'h954AAAAAAA552A5296D24924D99333198C738E1C3C1F01FC0007FFE0007F80F8),
    .INIT_22(256'h6B5A4B649B333339C63C787C07FC000007FC0FC3C38E318CC64CC9B6DB696B5A),
    .INIT_23(256'hD5A96D24D933318C71E1F03FF0003FF03E1E38E73333364924B5AD5AAAAAAA95),
    .INIT_24(256'hB4A56AAAAAB56B6924C998CE38F0F803FFFE01F878E39CCCC9B6DA5AD5AAAAAA),
    .INIT_25(256'h0000FC3C719CC9924B4AD555554AD2DB64CCCE30E1F807FFE00F878E3199936D),
    .INIT_26(256'h5555A96DB266738E1F01FFFC03E3C63333249294AAAAAAD696DB33339C787E00),
    .INIT_27(256'h83FFFFF03C38CE64DB696AD5552A5249366738E1F007FC01F0E38CCCDB6D2B55),
    .INIT_28(256'h4DB695AAAAAB52DB6CCCC71C1F800007E1E39CCC9B696A55555A96926CCC638F),
    .INIT_29(256'h666738707F00007F070E7333649695AAAAA95A5B26CC631E1F800003F0F1CE66),
    .INIT_2A(256'hB2492D6A95554AB4B493666631C3C0FFFFFFC0F0E3199936DA52A5555AA5A5B2),
    .INIT_2B(256'hC07C38718CC6CC9B69694AA5555AAD6B69364CCC631E1E07FFFFFF03E3C63199),
    .INIT_2C(256'hC3C71C6331993364924B4A52AD5555554A95A5A49264CCCC631C383E03FFFFFF),
    .INIT_2D(256'hAA556A95A94A5A5A496DB24D9B33333339CE71C78F0F83F01FFF0000FFF01F83),
    .INIT_2E(256'hB6DB6DB6DB6DB492496DA496D2DA5A5A5AD294A52B52A54AA554AAAAD55554AA),
    .INIT_2F(256'h00000000000000000007FFFFFFFFFFFFFFFFFFFFFFC0000000000000000000FF),
    .INIT_30(256'h000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000),
    .INIT_31(256'hFFFFFC0000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000FFFFFF),
    .INIT_46(256'h00000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE000000000000),
    .INIT_47(256'hFC0000000000000000000FFFFFFFFFFFFFFFFFFFFFFF80000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_2
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_2_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_2_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_2_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_2_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_2_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_2_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[2]}),
        .DOBDO(NLW_q0_reg_1_2_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_2_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_2_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_2_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_2_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_2_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_2_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_2_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_2_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_3" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "3" *) 
  (* ram_slice_end = "3" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h4924B694A56AD55AAAAAB556A95AD2D2DB6DB26CCD998CC6318E3870F0781F80),
    .INIT_01(256'h95294B49249B266666739C71C3C1F03FE00000003FE07C1E3C71C63199999B36),
    .INIT_02(256'h6CC6631CF1C3E0FC00FFFFFC01FC1E1E38E3198CD9936492DA52952AB55555AA),
    .INIT_03(256'hC787C0FF0000001FE07C3C79C631999B36C96D2D2B54AAAAAAA952B5A5B6DB26),
    .INIT_04(256'h001FFF000FC1F0E3CE339999B36DB69695AB55555554A95AD24B24D999998C71),
    .INIT_05(256'h07F03E1C38C63399B324DB496B5AB55555555AB52D2DB6D9B333318E78E1F07E),
    .INIT_06(256'h0C739CCCC9926DA4B5AD5AAAAAAAA952B4B49249B3333318E38F0F83F8000000),
    .INIT_07(256'h6C9249694A54AAA94AAA95294B49249B3333319C71E3C1F80FFFFFFFE01F0787),
    .INIT_08(256'h5AAAD54AAA54A52DA49364CCCCC671C70F0F80FFC00003FF03E0F1C31CE67326),
    .INIT_09(256'hD4B4B4936CD9999CC638F1E0F80FFE0001FFC07C1E3C718CE6666CDB24B4B4AD),
    .INIT_0A(256'h93399CE30E3C1F03FF00000FFC07C3C38E398CCCCC9B2496D294A9554AAD556A),
    .INIT_0B(256'h8783E01FFFFFFFC07E0F1E38E633333364924B4A52A5554A5554A94A5A4924D9),
    .INIT_0C(256'h0000007F07C3C71C633333364924B4B52A555555556AD6B496D9264CCCE738C3),
    .INIT_0D(256'hF83E1C79C63333366DB6D2D2B56AAAAAAAB56B5A4B6C933667318C70E1F03F80),
    .INIT_0E(256'h38C666666C93492D6A54AAAAAAAB56A5A5B6DB36666731CF1C3E0FC003FFE001),
    .INIT_0F(256'h936DB696B52A55555554AB52D2DA4DB36666318E78F0F81FE0000003FC0F878E),
    .INIT_10(256'h56AAAAB552A5296D249B266CC6631C71E1E0FE00FFFFFC00FC1F0E3CE3198CD9),
    .INIT_11(256'hB3666666318E38F1E0F81FF00000001FF03E0F0E38E73999999364924B4A52A5),
    .INIT_12(256'h07E0783C3871C6318CC666CCD936DB6D2D2D6A55AAB555556AAD5A94A5B49249),
    .INIT_13(256'h2B5AD69692DA4926D9366CC9999CCC6738C71C78F0F07C1FC03FFC0000007FF8),
    .INIT_14(256'hD99366C9B649B6DB6DB496D25A5AD294AD6AD5AA556AAAD55555554AAAD54A95),
    .INIT_15(256'h71C71C638E39C718E39C638C639C6318C6319CE63399CCC6666333333326664C),
    .INIT_16(256'hFFFFFFFC00000000000000FFFFFFFFFFFFFFFFFF000000000000000000000000),
    .INIT_17(256'hFE0000007FFFFFF80000003FFFFFFFC000000007FFFFFFFFE00000000007FFFF),
    .INIT_18(256'hC0007FFF0001FFFE0000FFFF80001FFFF80000FFFFF800003FFFFF000001FFFF),
    .INIT_19(256'h0FF803FE00FFC01FF801FF800FFC007FF001FFE001FFE001FFF0007FFC000FFF),
    .INIT_1A(256'h3E07C0FC1F81F81F81F81FC0FE07F03F80FE03F807F01FE01FC03FC03FE01FE0),
    .INIT_1B(256'hF1E3C7870F1E1E1C3C3C3C3C3C3E1E1F0F0783C1E0F07C1E0F83E0F83E0FC1F0),
    .INIT_1C(256'h663398CE7318C6318C631CE31CE31C638E71C71C638E3C71C71C38E1C78E1C78),
    .INIT_1D(256'h24926DB24D926C9B26CD93264CD993326664CCCCCCCCCCCCCCCCE666733199CC),
    .INIT_1E(256'h5AAAA5552AB552AD52A54AD4AD4A56B5A529694B4B4B49692D24B6D2492DB6DB),
    .INIT_1F(256'h6DB6DB6D24B69692D696B4A52B5A95AB56A954AAD552AAAD5555555555555555),
    .INIT_20(256'hC03F80FC1F07C3C3C3C78F1C71C718E739CE633999CCCCCCD9993366CD936C93),
    .INIT_21(256'h4CD9999999CCE6318E31C71C3870F0F87C0F81FC03FF0003FFFFFFFFFF8000FF),
    .INIT_22(256'hB26C92492DA5A5AD6B56AD56AAA9555552AAA556A95A94A5AD25A4924924D936),
    .INIT_23(256'hCC671CE3C70F0F83F01FF0000FFFC0003FE03F07C3C3C78E38C6319CCCCCCCD9),
    .INIT_24(256'h38C6733333264DB2496D2D6B52A552AAAAAAAB552A56B5A5A49249364C999999),
    .INIT_25(256'h5555AA952B4A5B4926D9B3333339CE38E3C3C1F01FF80000000FF80FC1E1E38E),
    .INIT_26(256'h333398E38E1E0F81FF00000003FC07C3C3C71CE733333364DB6DA5A52952AB55),
    .INIT_27(256'h800000003FC0F078E38E731999B364925B4A52B55AAAAAAB55A95A5A49249933),
    .INIT_28(256'h71C719CCCCCD9B6DB69695A9552AAAAD54A94A5A4924D93333398E71E3C3E07F),
    .INIT_29(256'hB4B5AD5AAA55552AAD5AD696D24DB366666739C71E3C1F01FF800003FF01F078),
    .INIT_2A(256'h69249B264CCCC6738C70E1E1F03FC000000000FF03E1E1C71C6339999336C924),
    .INIT_2B(256'hFF803F81F0F8F0E38E718CC6666CC9B24DA49696B5AB54AAAAAAAAA956AD6B4B),
    .INIT_2C(256'h56AD4AD694B496D24926D93664CCCCCCC6739C638E1C3C3C1F03F801FFFFFFFF),
    .INIT_2D(256'hCC99B326CD936C936DB6DB692DA5A5A5AD6B5A952A552AA55555AAAA55554AA9),
    .INIT_2E(256'hC71C71C71C71C71C718E38E71CE39C639CE318C6339CC6733998CCCCE66666CC),
    .INIT_2F(256'h00000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00),
    .INIT_30(256'hFFFFFFFFFFFE0000000000000000000000000000000000000000000000000000),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000001FFFFFFFFFFFF),
    .INIT_47(256'h03FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_3
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_3_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_3_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_3_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_3_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_3_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_3_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[3]}),
        .DOBDO(NLW_q0_reg_1_3_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_3_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_3_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_3_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_3_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_3_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_3_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_3_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_3_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_4" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "4" *) 
  (* ram_slice_end = "4" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h6DB6DB26C9B3666CCCCCC667319CE31CE38E3C70F1E1F0F83E0FC07F007FE000),
    .INIT_01(256'h4C9B26DB6DB692D2D2D6B52B56AB556AAAAAAAAAAAB556AB56A56B5AD2D2D25B),
    .INIT_02(256'h8F0783E0FE03FF0000FFFFFC0003FE01F81F0783C78F1C71C6318CE673333366),
    .INIT_03(256'hAD52AA55555555554AA956AD6B5AD2D25B6DB649B266CCCCCCCE6339C638E3C7),
    .INIT_04(256'hFFE000FFFFC00FE03E0F87878F1C718E7398CCCCCCCD9B364926924B4B4B5AD4),
    .INIT_05(256'h555AAB56AD6B5AD2DA496D924D93266666666339CE31C71E3C3C3E0F80FE007F),
    .INIT_06(256'h03F07C3C3871E39C739CC666666664C9926DB6DB6969694A56A55AA955555555),
    .INIT_07(256'h4924924D9366CCCD8CCCE6318C71C71C3C3C3E1F81FC01FFF00000001FFF007F),
    .INIT_08(256'hC666332666CD936492492DA5A5AD2B52A55AAA5555555555AAB55A95A94B5A4B),
    .INIT_09(256'hE738C71C70E1E1E0F83F01FF000FFFFFFFFFC003FE03F07C1E1E1C38E38C739C),
    .INIT_0A(256'h496B4A56A56AB556AAAAAAAAA9556A952B52D69696D249249B26CD999331998C),
    .INIT_0B(256'hF803FFE00000003FFE00FE07E1F0F0F0E38E38C6319CCCC6CCCD9B26C924924B),
    .INIT_0C(256'hAAAAAAAA556A95A94A5A5A5B6DB6D9264C999999998CE738E71E3870F0F83F03),
    .INIT_0D(256'hF801FC07C1F0F0F1E38E31CE73199999999326C926DA496D2D6B5AD5AB556AAA),
    .INIT_0E(256'hAD6B4B4B49259249B366CCCCCCCC6739C638E3C78787C1F01FC00FFFFC001FFF),
    .INIT_0F(256'h8F1C718E7319CCCCCCCD993649B6DB692D2D6B5AD5AA554AAAAAAAAAA9552AD4),
    .INIT_10(256'h9B3333399CC6318E38E3C78F0783E07E01FF0000FFFFFC0003FF01FC1F0783C7),
    .INIT_11(256'h692D2D2D6B5A95AB55AAB555555555555AAB55AB52B5AD2D2D25B6DB6D9364C9),
    .INIT_12(256'h001FF803F80FC1F07C3E1E3C38F1C71CE31CE633998CCCCCD99B364D936DB6DB),
    .INIT_13(256'hCC6318E71CE38E38E1C78F0E1E1F0F87C0F81F80FF007FE0003FFFFFFFFFFFF8),
    .INIT_14(256'h92DA4B6D2492DB6DB6D9249B6C9364D9364C9933664CCC999999998CCCE67319),
    .INIT_15(256'h5A95A94AD4AD6A52B5294AD6B5294A5294A5294B5AD29694B4B5A5A5A5B4B496),
    .INIT_16(256'h00000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000000),
    .INIT_17(256'h01FFFFFFFFFFFFF8000000000000003FFFFFFFFFFFFFFFFFE000000000000000),
    .INIT_18(256'hC0000000FFFFFFFE000000007FFFFFFFF80000000007FFFFFFFFFF0000000000),
    .INIT_19(256'h0007FFFE00003FFFF800007FFFFC00000FFFFFE000001FFFFFF0000003FFFFFF),
    .INIT_1A(256'hFE003FFC007FF8007FF8003FFE000FFF8001FFF8000FFFE0003FFFC0001FFFE0),
    .INIT_1B(256'hF01FC07F00FE01FC03FC03FC03FE01FF00FF803FE00FFC01FF801FF801FFC00F),
    .INIT_1C(256'hE1F0783E0F07C1F07C1F03E0FC1F03E07E0FC0FC1F81FC0FC0FC07E03F81FC07),
    .INIT_1D(256'h1C71E38E3C71E3871E3C70E1C3C78F0E1E1C3C3C3C3C3C3C3C3C1E1E0F0F87C3),
    .INIT_1E(256'h39999CCCE673319CCE6339CC6339CE739CE718C738C738E71CE38E31C71C71C7),
    .INIT_1F(256'hDB6DB6DB6D924DB64DB26D9366C9B366CD9B32664CC9999B3333333333333333),
    .INIT_20(256'h556AAA554AAD56A956AD5AB52B52B5AD6B5AD694B4A5A5A5B4B496D25B4925B6),
    .INIT_21(256'h696D2D2D2D694B5AD4A56A56AD5AA552A9552AA95555AAAAAAAAAAAAAAAAAA55),
    .INIT_22(256'h3C70E38E31C639CE7398CE67333199999B333664CD9326C9364936DB6DB6925B),
    .INIT_23(256'h3C1F03E03F00FF800FFFF000000000003FFFC007FC03F80FC0F83E1F0F0F0F1E),
    .INIT_24(256'h6A52D6969692DB6924DB64D9366CC999999998CCE6318C639C71C70E3C787878),
    .INIT_25(256'h999933264D936D924B6D25A5A5AD6B52B56A955AAAAD5555555AAAA554AB56A5),
    .INIT_26(256'h0F0F87E07E01FF8000FFFFFFFC0007FC03F81F07C3C3C3871C71C639CE633399),
    .INIT_27(256'h2AAAAAAA9555AAD5A95AD6B4B496D24936D9366CC9999998CC6739C638E3870F),
    .INIT_28(256'h7E07E1F0F0F1E38E38E719CE6633333666CD936C92496DA5A5AD2B5AB56AB555),
    .INIT_29(256'h6D9364C9993333199CC6318E31C38F1E1E1F07C0FE03FF00007FFFFC0001FF80),
    .INIT_2A(256'h4DB6D24B69696B5AD6A54AB55AAA9555555555AAA954AB52B5296B4B496DA492),
    .INIT_2B(256'h00003FFE00FF00FC0F81F0F87870F1C38E38E718C63398CCCCCCCCCD9B364D92),
    .INIT_2C(256'h319CC6318C738E31C71E38F1E3C3C3C3C1F07C1F81FC03FC00FFF80000000000),
    .INIT_2D(256'hA5B496925B4925B6DB6DB6DB64936C9364D9364C9933666CCCCC9999CCCCC667),
    .INIT_2E(256'h52B52B52B52B52B52B5A95AD4A56B5294A56B5AD694A52D694B5A5A5AD2D2DA5),
    .INIT_2F(256'h00000000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF80000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_4
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_4_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_4_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_4_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_4_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_4_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_4_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[4]}),
        .DOBDO(NLW_q0_reg_1_4_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_4_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_4_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_4_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_4_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_4_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_4_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_4_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_4_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_5" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "5" *) 
  (* ram_slice_end = "5" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h8E38E3C70E3C7870F0F0F8783E1F03E0FC0FC07F01FE00FFC00FFF80007FFFFF),
    .INIT_01(256'h692DB4924924DB649B64D9B264CD99B333333333333998CC67398C631CE31C63),
    .INIT_02(256'hA552A955AAA9555555AAAAA9555554AAAD55AAD56AD5A95A94A5294B5A5A5A4B),
    .INIT_03(256'h633199CCCCCCCCCCD99B3264D93649B6C92492DB692DA5A5A5A5296B5295A952),
    .INIT_04(256'h00000000003FFFE001FF807F80FC0F81F0783C3C3C3C78F1C71E71C738C739CC),
    .INIT_05(256'h66633398CE739CE31C718E1C71E3C787878783C1F03E07E03FC03FF000FFFF80),
    .INIT_06(256'h555AA956AD5AB5295AD694B4B4B4B692DB4924924DB24D9364C9933266666666),
    .INIT_07(256'h24924924B6D25A5B5A5A5294A52B52B56A956AB554AAAB55555555555555AAAA),
    .INIT_08(256'h3E1E0F1E1E3C70E38E38E39C639CE7319CC6663333333333666CC9B364D936D9),
    .INIT_09(256'hF83F07E07F01FE00FFC001FFFFF0000000003FFFFE000FFC01FE03F81F83F07C),
    .INIT_0A(256'h6DB26C9B364CD99B3333333331998CE6339CE718E71C71C71C38F1E1E3C1E1F0),
    .INIT_0B(256'h5556AAAAAAAAAAAAAB5554AAB55AA55AB52B5294A529696B69692DB492492492),
    .INIT_0C(256'h9999999933264C9B26C936C924924B6D25B4B4B4B4A5AD6A52B56AD5AA556AA9),
    .INIT_0D(256'h07FFFC003FF00FF01F81F03E0F078787878F1E38E1C638E31CE739CC67331999),
    .INIT_0E(256'hCE738C738E39E38E3C78F0F0F0F0783E07C0FC07F807FE001FFFF00000000000),
    .INIT_0F(256'h2A56A52B5A5296969696D25B6D24924DB649B26C9933666CCCCCCCCCCE663318),
    .INIT_10(256'h4969696B4A5294A56A56AD5AAD56AAD554AAAAAA555556AAAAAA5556AA552A95),
    .INIT_11(256'h18E31CE318C67398CC667333333333333666CC99366C9B649B6C924924B6D25B),
    .INIT_12(256'hFFFFF80007FFC00FFC01FE03F80FC0FC1F03E1F0787C3C3C3878F1C38F1C71C7),
    .INIT_13(256'hF07C1F07E0FC0FC0FE07F00FE01FF007FF001FFF00007FFFFFC0000000000007),
    .INIT_14(256'h1CE38C71C71CE38E38E1C71C70E3871E3870E1C3878F0F1E1E1E1E0F0F0783E1),
    .INIT_15(256'h6319CE7318CE739CC6318CE739CE739CE739CE739CE318E738C639C639C738E7),
    .INIT_16(256'h0000000000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'h0000000000000007FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE000000000000000),
    .INIT_18(256'h3FFFFFFFFFFFFFFE000000000000000007FFFFFFFFFFFFFFFFFFFF0000000000),
    .INIT_19(256'hFFFFFFFE0000000007FFFFFFFFFC00000000001FFFFFFFFFFFF0000000000000),
    .INIT_1A(256'hFE000003FFFFF8000007FFFFFE0000007FFFFFF80000001FFFFFFFC00000001F),
    .INIT_1B(256'h0FFFC000FFFE0003FFFC0003FFFE0000FFFF80001FFFFC00007FFFF800003FFF),
    .INIT_1C(256'hE00FF801FF003FF003FF001FFC00FFE001FFC003FF8003FFC003FFE0007FFC00),
    .INIT_1D(256'hFC0FE07E03F01F80FE03F01FC03F80FE01FC03FC03FC03FC03FC01FE00FF803F),
    .INIT_1E(256'h078783C3E1F0F07C3E1F07C3E0F83E0F83E0F83F07C0F81F03E07E0FC0FC0FC0),
    .INIT_1F(256'h38E38E38E38E3C71C38E1C70E1C78F1E3C78F1E1C3C787870F0F0F0F0F0F0F0F),
    .INIT_20(256'hCCE66633399CCE67319CC67318CE739CE739CE738C639C638C738E31C738E38E),
    .INIT_21(256'hDB249B649B24D9364D9326CD9B366CC99B336664CCCC999999999999999999CC),
    .INIT_22(256'h6AD5A95A94AD6B5AD6B5A52D696B4B4B49696D2DA4B6925B6D24924924924936),
    .INIT_23(256'hA955AAB555AAAAD555555AAAAAAAAAAA95555552AAA9555AAA556AB55AA55AB5),
    .INIT_24(256'hB3649B24DB24924DB692496DA4B692D2D2D2D296B4A5294AD6A56A54A952AD52),
    .INIT_25(256'h1E1E3C3871E38E1C738E39C639CE739CC673199CCCCE6666666CCCC9993264C9),
    .INIT_26(256'h00FF801FFE00007FFFFFFFFFFFFFF80003FFE007FC03FC07E07E07C1F07C3C1E),
    .INIT_27(256'h66666666733399CC6739CE738C71CE38F1C70E1C387878783C1F07C1F81F80FF),
    .INIT_28(256'h2AAD54AA55AB56A56A52B4A52D69696D2DA4B6DA4924DB6C936499366CD99333),
    .INIT_29(256'hB6DA496D2DA5A5AD296B5AD4A56AD5AB54AA556AAB5555AAAAAAAAAAAAAB5555),
    .INIT_2A(256'h71C71C738E718C6318C673399CCCE666666666CCCD993264D9B24D926DB6C924),
    .INIT_2B(256'hFFFFC00000FFFF000FFE00FF807F01FC0FC0F81F07C3E0F0F0F0F0F1E3C78E1C),
    .INIT_2C(256'hF07C3E0F83F07E0FC0FE07F01FC03FC03FF003FF8003FFFC000007FFFFFFFFFF),
    .INIT_2D(256'h9C738E71C738E38E38E38E38E38F1C70E3C70E3C78F0E1E3C3C38787C3C3C1E0),
    .INIT_2E(256'hCE7318CE7318CE7318C6739CC6318CE739CE739CE739CE318C739C639CE31C63),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFF800000000000000000000000000000000000000000000),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_47(256'h000000000000000000000000000000000000000000007FFFFFFFFFFFFFFFFFFF),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_5
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_5_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_5_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_5_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_5_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_5_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_5_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[5]}),
        .DOBDO(NLW_q0_reg_1_5_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_5_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_5_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_5_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_5_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_5_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_5_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_5_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_5_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_6" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "6" *) 
  (* ram_slice_end = "6" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hF03F03F80FC07F80FF00FF803FE003FF000FFF8001FFFF00000FFFFFFF800000),
    .INIT_01(256'h8E31C71C71C71C78E3871E3C78F1E1C3C3C3C3C3C3C1E0F0783E0F83E0FC1F83),
    .INIT_02(256'hC99B3266CCCD999999333331999998CCCE6633198CE6319CE739CE739C639C73),
    .INIT_03(256'hB5A52D69696969696D2DA4B6925B6D24924924924DB6C936C9364DB264D93264),
    .INIT_04(256'h555555555555554AAAAAD5552AA9552AA552A956A956AD5A95AB5A95AD6A5296),
    .INIT_05(256'hD2D696B5A5294A56B52B5AB52B56AD52AD52A954AA9552AA95556AAAAA555555),
    .INIT_06(256'hCCC99B3264C99364C9B24D926D926DB6492492492496DB492DA4B696D2D2D2D2),
    .INIT_07(256'hE38E38E38E31C638C639CE739CE7318CE673198CCC6667333333333333336666),
    .INIT_08(256'h01FE00FE01FC0FE07E07E07C1F83E0F07C3E1E0F0F0F0F0F1E1C3870E3C70E38),
    .INIT_09(256'hFFC007FF8001FFFF000001FFFFFFFFFFFFFFFFFFFE000003FFFE0007FF800FFC),
    .INIT_0A(256'h71C38F1C3870E1E3C3C3C3C3C1E1F0F83C1F07E0F81F81F81FC0FE01FC01FE00),
    .INIT_0B(256'h999B333333333333339998CCC663399CC6339CE739CE718C718E31C71C71C71C),
    .INIT_0C(256'h2D2D2D2DA5B496D24B6DA49249249249B6D926D926C9364C9B264C9933664CCD),
    .INIT_0D(256'hAAAAA955555AAAA5552AA554AA552AD52AD5AB52B56B52B5A94A5296B5A5AD2D),
    .INIT_0E(256'hA5295AD6A56B56A56AD5AA55AA552A9552AA5552AAAD55554AAAAAAAAAAAAAAA),
    .INIT_0F(256'h99326C9936C9B24DB24DB6C92492492492DB6925B496D2DA5A5A5A5A5AD296B5),
    .INIT_10(256'h38E718E739CE739CE6319CC6633199CCCC666666333332666666CCCD9933664C),
    .INIT_11(256'h07E0FC1F07C1F0783C1E0F0F0F0F0F0F0E1E3C78F1E3871C78E38E38E38E31C7),
    .INIT_12(256'h000007FFFFFFC00003FFFE0007FFC003FF001FF007FC03FC07F80FC07F03F03F),
    .INIT_13(256'h007FE007FF000FFF0007FFF0001FFFF800001FFFFFFF80000000000000000000),
    .INIT_14(256'h1F03F07E07E0FC0FC0FE07E07F03F81FC07F01FC07F00FE01FE01FF00FF803FE),
    .INIT_15(256'h83E1F07C1F0F83E0F83E0F07C1F07C1F07C1F07C1F03E0F83F07C1F83E07C0F8),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'h0000000000000000000000000000000000000000000000001FFFFFFFFFFFFFFF),
    .INIT_18(256'h0000000000000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000),
    .INIT_19(256'hFFFFFFFE00000000000000000003FFFFFFFFFFFFFFFFFFFFFFF0000000000000),
    .INIT_1A(256'h01FFFFFFFFFFF8000000000001FFFFFFFFFFFFF8000000000000003FFFFFFFFF),
    .INIT_1B(256'hFFFFC0000001FFFFFFFC00000001FFFFFFFF8000000003FFFFFFFFF800000000),
    .INIT_1C(256'hE00007FFFF00000FFFFF000003FFFFE000003FFFFF8000003FFFFFE0000003FF),
    .INIT_1D(256'h03FFE001FFF0007FFE000FFFC0007FFE0003FFFC0003FFFC0003FFFE00007FFF),
    .INIT_1E(256'hFF807FC01FF00FFC01FF003FE007FE007FE007FF003FF800FFE001FFC003FFC0),
    .INIT_1F(256'h07E07E07E07E03F03F81FC0FE03F80FE03F80FE03FC07F80FF00FF00FF00FF00),
    .INIT_20(256'h3C1E1E0F0783C1E0F07C3E0F07C1F07C1F07C1F07C1F83E07C0F81F03F07E07E),
    .INIT_21(256'hC71C78E3871C38F1C38F1E3C78F1E3C7870F1E1C3C3C7878787878787878783C),
    .INIT_22(256'h19CC67398C6318C6318C631CE718C738C718E31C638E71C71CE38E38E38E38F1),
    .INIT_23(256'h64CC99933366664CCCCCC999999999998CCCCCCE6667333999CCE673399CC673),
    .INIT_24(256'h96D2496DB6924924924924DB6D9249B649B649B26D9364D9B26CD93264C99B36),
    .INIT_25(256'hB54A956AD4A95AB5295A94AD6B5AD6B5AD294B4A5A5AD2D2D2DA5A5B4B692DA4),
    .INIT_26(256'hAAAAD55554AAAAAAAAAAAAAAAAAAAAAAA9555552AAA95552AAD552AB552A954A),
    .INIT_27(256'hB4B4B4B4A5A52D694A5294A5295A94AD5A95AB56AD52AD52A955AA9552AAD555),
    .INIT_28(256'h4CC9993366CD9B364C9B26C9B64DB249B6C924936DB6924925B6D25B496D25A5),
    .INIT_29(256'hC71C718E31C639CE318C6318C67319CC6733998CCC6666333333333333326666),
    .INIT_2A(256'h81F81F83F07E0F83E0F87C3E1F0F07878787870F0E1E3C78E1C38E1C71C70E38),
    .INIT_2B(256'hFFFFFFFFFF0000000FFFFF00007FFE000FFF001FF803FF00FF00FF01FC07F01F),
    .INIT_2C(256'hF003FE007FF001FFC001FFF0003FFFC0000FFFFF80000003FFFFFFFFFFFFFFFF),
    .INIT_2D(256'h7C0F81F03F07E07E07E07E07E07F03F01FC0FE03F80FE01FC03F807FC03FC01F),
    .INIT_2E(256'hC1F0F83E0F07C1F0F83E0F83C1F07C1F07C1F07C1F07C1F07C0F83E07C1F03E0),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_6
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_6_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_6_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_6_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_6_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_6_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_6_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[6]}),
        .DOBDO(NLW_q0_reg_1_6_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_6_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_6_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_6_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_6_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_6_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_6_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_6_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_6_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_7" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "7" *) 
  (* ram_slice_end = "7" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h556AA9555AAAD555AAAA55556AAAA955555AAAAAAB555555555AAAAAAAAAAAAA),
    .INIT_01(256'h5A94AD4AD4AD4AD5A952B56AD5AB54A956A956A956AB55AAD56AA556AA554AA9),
    .INIT_02(256'hA4B696D25A5B4B4B4B69696B4B4B4A5A5AD296B4A5AD6B4A5294A5294AD6B529),
    .INIT_03(256'h6C9364DB24DB24DB249B6D924936DB6DB6DB6DB6DB6DA4925B6D2496D24B692D),
    .INIT_04(256'h333333333333332666664CCC999B33666CC99B3264CD9B364C99364C9B26C9B2),
    .INIT_05(256'h31CE718C6318C6318CE7398CE7319CCE633198CC667331998CCCE66666333333),
    .INIT_06(256'hC3C7870E1C3870E3C78E3C71E38E1C71C71C71C71C71C738E39C718E31CE31CE),
    .INIT_07(256'h1F81F81F81F03E07C1F83E0F83E0F07C1E0F0783C3E1E0F0F0F0F0F0F0F0E1E1),
    .INIT_08(256'hFFFE0001FFFC001FFE001FFC007FE00FFC01FE00FF00FF00FE03F80FE03F01F8),
    .INIT_09(256'h000007FFFFFE0000000001FFFFFFFFFFFFFFFFFFFE0000000001FFFFFF800003),
    .INIT_0A(256'h7E03F01FC07F01FC03FC03FC01FE00FFC01FF800FFE001FFE000FFFE0001FFFF),
    .INIT_0B(256'h1E1C3C3C3C3C3C3C3C1E1F0F0783C1E0F83C1F07C1F07E0F81F03E07E07E07E0),
    .INIT_0C(256'hCE31CE31C638E71C738E38E38E38E38E38E1C71E38F1C78F1C3870E1C3878F0E),
    .INIT_0D(256'h33333199999CCCC666333998CC663319CCE6339CC6739CC6318C6318C639CE31),
    .INIT_0E(256'h364D9364C9B264C9B366CC9933664CD99B336664CCC999999333333333333333),
    .INIT_0F(256'hD25B492DA492DB692496DB6DB6DB6DB6DB6DB24926DB64936C936C936C9B24D9),
    .INIT_10(256'h52B5AD4A5294A5294B5AD694B5A52D69694B4B4B5A5A5B4B4B4B69692DA5B496),
    .INIT_11(256'h554AA955AA955AAD56AB55AA55AA55AA54AB56AD5AB52A56AD4AD4AD4AD4A56A),
    .INIT_12(256'h5555555555556AAAAAAAAB5555556AAAAA55555AAAA95556AAAD556AAA555AAA),
    .INIT_13(256'h552AAAAD55555AAAAAAD5555554AAAAAAAAAB555555555555555555555555555),
    .INIT_14(256'hB556AAD552AA555AAA5552AAD556AAB5552AAB5552AAA5554AAAB5555AAAA955),
    .INIT_15(256'h56AB552AB55AA955AA955AAD54AAD54AAD54AAD54AA955AA9552AB556AAD55AA),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'h000000000000000000000000000000000000000000000000000000FFFFFFFFFF),
    .INIT_19(256'h00000001FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000000000),
    .INIT_1A(256'h00000000000007FFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000000000000),
    .INIT_1B(256'h00003FFFFFFFFFFFFFFC00000000000000007FFFFFFFFFFFFFFFFFF800000000),
    .INIT_1C(256'h1FFFFFFFFF0000000000FFFFFFFFFFE000000000007FFFFFFFFFFFE000000000),
    .INIT_1D(256'h00001FFFFFF0000001FFFFFFC0000001FFFFFFFC00000003FFFFFFFE00000000),
    .INIT_1E(256'hFF80003FFFF00003FFFF00001FFFFE00001FFFFF000007FFFFE000003FFFFFC0),
    .INIT_1F(256'hFFE001FFE001FFF0007FFC001FFF8001FFF8001FFFC0007FFF0000FFFF0000FF),
    .INIT_20(256'hFC01FE00FF803FE00FFC01FF003FF003FF003FF003FF801FFC007FF000FFE001),
    .INIT_21(256'hC0FC07E07F03F80FC07F01FC07F01FC07F00FE03FC03F807F807F807F807F803),
    .INIT_22(256'h07C3E0F87C1F07C1F07C1F03E0F83F07C0F81F03E07E0FC0FC1F81F81F81F80F),
    .INIT_23(256'hE3C3878F0F1E1E3C3C3C3878787878787C3C3C3E1E1F0F0787C3E1F0F87C3E0F),
    .INIT_24(256'h71CE38E38E71C71C71C71C38E38E3871C78E3871E38F1C3871E3C70E1C3878F1),
    .INIT_25(256'h8CC67319CC67398CE7398C6318C6318C6318C739C639CE31CE39C638C718E39C),
    .INIT_26(256'h9999B333326666666666666666666666673333319998CCCE66333198CCE67339),
    .INIT_27(256'h6D926D926C9364DB26C9B26C9B364D9B364C993264C99B3664CC99B336664CCC),
    .INIT_28(256'h25A4B496D25B496D25B6925B6D2496DB6DA4924924924924936DB6C924DB6C93),
    .INIT_29(256'hAD4AD4A56B5294A56B5AD6B5AD294B5AD296B4A5A52D2D696969696969692D2D),
    .INIT_2A(256'hAB554AA9552AA556AA552A954AA552AD52AD52A55AB56AD5AB56A54AD4AD5A95),
    .INIT_2B(256'hAAAAAAAAAAAAAAAAA5555555552AAAAAA555554AAAA95555AAAA5554AAAD554A),
    .INIT_2C(256'h5AAAAB55555AAAAA9555555AAAAAAA95555555552AAAAAAAAAAAAAAAAAAAAAAA),
    .INIT_2D(256'hA9552AA555AAB554AAB554AAB555AAA5556AAB5552AAB5556AAAD5556AAA9555),
    .INIT_2E(256'h955AAD54AA556AA552AB552A955AA955AA955AA955AA955AA9552AB556AA554A),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_7
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_7_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_7_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_7_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_7_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_7_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_7_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[7]}),
        .DOBDO(NLW_q0_reg_1_7_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_7_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_7_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_7_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_7_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_7_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_7_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_7_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_7_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_8" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "8" *) 
  (* ram_slice_end = "8" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'hCCE667333999CCCC66663333199998CCCCC66666673333333339999999999999),
    .INIT_01(256'h398C6339CC6339CC67318CE63398CC673198CE673198CC6633199CCE66333998),
    .INIT_02(256'h9C718E31C638C738C718E718C738C639C6318E739C6318C6318C6318C6318CE7),
    .INIT_03(256'hE38F1C38E3C71C38E3871C71C70E38E38E38E38E38E39C71C71CE38E31C718E3),
    .INIT_04(256'hF0F0F0F0F0F0F0E1E1E1C3C387870F1E1C3878F1E3C3870E3C78F1C3871E3871),
    .INIT_05(256'hF03E0F83E0F83E0F83E0F87C1F0F83C1E0F0783C1E0F0F8783C3E1E1E1F0F0F0),
    .INIT_06(256'hC03F80FE03F80FE03F81FC0FE07E03F03F03F03F03F03F07E07C0F81F03E0FC1),
    .INIT_07(256'h007FF8007FF001FFC007FE007FE00FFC01FF007FC01FE00FF00FF00FF00FE01F),
    .INIT_08(256'h0001FFFFFFFC000001FFFFFC00001FFFFC0001FFFF0000FFFE0007FFE000FFF8),
    .INIT_09(256'h000007FFFFFFFFFFFFFFFE00000000000000000001FFFFFFFFFFFFFFFF800000),
    .INIT_0A(256'h7FFC001FFF8001FFFC0003FFFE0000FFFFE00000FFFFFE000000FFFFFFFE0000),
    .INIT_0B(256'hE01FC03FC03FC03FC01FE00FF803FE00FFC01FF801FF800FFE003FF8007FF800),
    .INIT_0C(256'h0FC1F03E07C0F81F83F03F03F03F03F03F01F81FC0FE07F01FC07F01FC07F00F),
    .INIT_0D(256'h3C3C3E1E1E1F0F0787C3C1E0F0783C1E0F07C3E0F87C1F07C1F07C1F07C1F03E),
    .INIT_0E(256'h3871E3870E3C78F1C3870F1E3C7870E1E3C387870F0E1E1E1C3C3C3C3C3C3C3C),
    .INIT_0F(256'h1C638E31C71CE38E38E71C71C71C71C71C71C38E38E3871C70E38F1C70E3C71E),
    .INIT_10(256'h9CC6318C6318C6318C6318E739C6318E718C738C639C638C738C718E31C638E7),
    .INIT_11(256'h66733199CCE6633198CC663399CC663398CC67319CC63398CE7318CE7318C673),
    .INIT_12(256'h66666666666673333333339999998CCCCC66666333319998CCCE667333999CCC),
    .INIT_13(256'h99B3333666666CCCCCC999999993333333332666666666666666666666666666),
    .INIT_14(256'h2664CC999B33666CCC999B336664CCD999B3326664CCC999933326666CCCCD99),
    .INIT_15(256'h9B32664CD9933266CCD9933666CC99933666CC99933266CCD99B32664CC99933),
    .INIT_16(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_17(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_18(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_19(256'h000000000000000000000000000000000000000000000000000FFFFFFFFFFFFF),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000000000000),
    .INIT_1B(256'hFFFFFFFFFFFFFFFFFFFC000000000000000000000000000000000007FFFFFFFF),
    .INIT_1C(256'h0000000000FFFFFFFFFFFFFFFFFFFFE000000000000000000000001FFFFFFFFF),
    .INIT_1D(256'h00000000000FFFFFFFFFFFFFC000000000000003FFFFFFFFFFFFFFFE00000000),
    .INIT_1E(256'hFF800000000FFFFFFFFF0000000001FFFFFFFFFF00000000001FFFFFFFFFFFC0),
    .INIT_1F(256'h001FFFFFE000000FFFFFFC0000007FFFFFF80000003FFFFFFF00000000FFFFFF),
    .INIT_20(256'hFC0001FFFF80001FFFFC0000FFFFF00000FFFFF000007FFFFC00000FFFFFE000),
    .INIT_21(256'hC003FFE000FFF8003FFF0003FFF0003FFF0001FFFC0007FFF80007FFF80007FF),
    .INIT_22(256'h003FE007FC00FFC00FFC00FFE007FF003FF800FFE001FFC003FF8007FF8007FF),
    .INIT_23(256'h1FC07F80FF01FE03FC03F807F807F807FC03FC01FE00FF007FC01FF007FC01FF),
    .INIT_24(256'h0FC1F81F81F03F03F03F03F81F81F80FC07E07F01F80FC07F01FC0FE03F807F0),
    .INIT_25(256'h83C1F0F83C1F0783E0F87C1F07C1F07C1F07C0F83E07C1F03E07C1F83F07E07C),
    .INIT_26(256'h87878F0F0E1E1E1E1E1E1E1E1E1E1E1E1F0F0F0F8787C3C1E1F0F0783C1E0F07),
    .INIT_27(256'hE38E1C71E38F1C38E1C78E1C78F1C3870E3C78F1E3C7870E1C3C7870F1E1C3C3),
    .INIT_28(256'h1C638C71CE38C71CE38E71C71CE38E38E39C71C71C71C71C70E38E38E3C71C70),
    .INIT_29(256'h9CC6339CE7318C6318C6318C6318C739CE718C639CE31CE718E718E718E71CE3),
    .INIT_2A(256'h98CCC66733199CCE6633198CC663319CCE63319CC67319CC67319CC6339CC673),
    .INIT_2B(256'h666666666666666663333333331999999CCCCCC6666733339999CCCC66633339),
    .INIT_2C(256'hC9999933333666664CCCCCC9999999B333333333666666666666666666666666),
    .INIT_2D(256'h9B33666CCC999332666CCD999333666CCCD9993336666CCCD999B33326664CCC),
    .INIT_2E(256'h4CC99B3266CCD9933666CC99B33664CC99B33664CC99B33664CC99933266CCD9),
    .INIT_2F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_30(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_31(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_32(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_33(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_34(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_35(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_36(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_37(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_38(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_39(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_3F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_40(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_41(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_42(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_43(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_44(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_45(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_46(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_47(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_8
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_8_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_8_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_8_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_8_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_8_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_8_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[8]}),
        .DOBDO(NLW_q0_reg_1_8_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_8_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_8_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_8_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_8_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_8_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_8_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_8_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_8_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
  (* \MEM.PORTA.DATA_BIT_LAYOUT  = "p0_d1" *) 
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-6 {cell *THIS*}}" *) 
  (* RTL_RAM_BITS = "1310720" *) 
  (* RTL_RAM_NAME = "inst/GAUSS_LUT_U/q0_reg_1_9" *) 
  (* RTL_RAM_TYPE = "RAM_SP" *) 
  (* ram_addr_begin = "32768" *) 
  (* ram_addr_end = "65535" *) 
  (* ram_offset = "0" *) 
  (* ram_slice_begin = "9" *) 
  (* ram_slice_end = "9" *) 
  RAMB36E1 #(
    .DOA_REG(0),
    .DOB_REG(0),
    .EN_ECC_READ("FALSE"),
    .EN_ECC_WRITE("FALSE"),
    .INITP_00(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_01(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_02(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_03(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_04(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_05(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_06(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_07(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_08(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_09(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INITP_0F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_00(256'h3C1E1F0F0787C3C3E1E1F0F0F878783C3C3E1E1E1F0F0F0F0F07878787878787),
    .INIT_01(256'hF87C1F07C3E0F83C1F0F83E1F0783C1F0F87C1E0F0783C1E0F0783C1E1F0F878),
    .INIT_02(256'h83F07E0FC1F83F07C0F81F07C0F83E07C1F07E0F83E0F83E0F83E0F83E0F83E0),
    .INIT_03(256'hE07F03F81FC0FC07E07F03F03F01F81F81F81F81F81F83F03F03E07E0FC0F81F),
    .INIT_04(256'h0FF00FF00FF00FE01FE03FC07F80FF01FC07F80FE03F80FE03F80FC07F01F80F),
    .INIT_05(256'h0FFE007FE007FE007FE007FC00FF803FE00FF803FE00FF807FC01FE01FF00FF0),
    .INIT_06(256'hC0007FFE0007FFE0007FFC001FFE000FFF000FFF000FFF001FFC007FF001FFC0),
    .INIT_07(256'h000007FFFFF000003FFFFE00001FFFFC0000FFFFC0001FFFF0000FFFF0001FFF),
    .INIT_08(256'hFFFFFFFFFFFC000000000003FFFFFFFFFC00000000FFFFFFFE0000001FFFFFF8),
    .INIT_09(256'hFFFFF800000000000000000000000000000000000000000000000000007FFFFF),
    .INIT_0A(256'h7FFFFFE0000001FFFFFFFC00000000FFFFFFFFFF000000000000FFFFFFFFFFFF),
    .INIT_0B(256'hFFE0003FFFC0003FFFE0000FFFFC0000FFFFE00001FFFFF000003FFFFF800000),
    .INIT_0C(256'h0FFE003FF800FFE003FFC003FFC003FFC001FFE000FFF8001FFF8001FFF8000F),
    .INIT_0D(256'h3FC03FE01FE00FF807FC01FF007FC01FF007FC00FF801FF801FF801FF801FFC0),
    .INIT_0E(256'hC07E03F80FC07F01FC07F01FC07F80FE03FC07F80FF01FE01FC03FC03FC03FC0),
    .INIT_0F(256'hE07C0FC1F81F03F03F07E07E07E07E07E07E03F03F03F81F80FC0FE07F03F81F),
    .INIT_10(256'h1F07C1F07C1F07C1F07C1F07C1F83E0F81F07C0F83E07C0F83F07E0FC1F83F07),
    .INIT_11(256'h787C3E1E0F0783C1E0F0783C1E0F87C3E0F0783E1F07C3E0F07C1F0F83E0F87C),
    .INIT_12(256'h87878787878783C3C3C3C3E1E1E1F0F0F078787C3C3E1E1F0F0F8783C3E1E0F0),
    .INIT_13(256'h1E3C3C38787870F0F0F1E1E1E1E3C3C3C3C3C787878787878787878787878787),
    .INIT_14(256'hC7870F1E1C3C7870F0E1E3C387870F1E1E3C3C7878F0F1E1E3C3C7878F0F0E1E),
    .INIT_15(256'hE3C3878F1E1C3C78F0E1E3C7870F1E1C3878F0E1E3C3870F1E1C3C7870F1E1C3),
    .INIT_16(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_17(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_18(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_19(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_1A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8000000000000000000000000),
    .INIT_1B(256'h00000000000000000003FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_1C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE000000000000000000000000000000000),
    .INIT_1D(256'hFFFFFFFFFFFFFFFFFFFFFFFFC0000000000000000000000000000001FFFFFFFF),
    .INIT_1E(256'hFF800000000000000000FFFFFFFFFFFFFFFFFFFF00000000000000000000003F),
    .INIT_1F(256'hFFFFFFFFE0000000000003FFFFFFFFFFFFF800000000000000FFFFFFFFFFFFFF),
    .INIT_20(256'h03FFFFFFFF8000000003FFFFFFFFF0000000000FFFFFFFFFFC00000000001FFF),
    .INIT_21(256'hC000001FFFFFF8000000FFFFFFF0000000FFFFFFFC00000007FFFFFFF8000000),
    .INIT_22(256'h00001FFFFC00003FFFFC00001FFFFF000007FFFFE000003FFFFF8000007FFFFF),
    .INIT_23(256'h003FFF8000FFFE0003FFF80007FFF80003FFFC0001FFFF00003FFFF00003FFFF),
    .INIT_24(256'h003FF8007FF000FFF000FFF8007FF8003FFE000FFF8003FFF0003FFE0007FFF0),
    .INIT_25(256'h7FC00FF803FF007FE007FC00FFC00FFC00FFC007FE003FF001FFC007FF001FFC),
    .INIT_26(256'h807F80FF01FE01FE01FE01FE01FE01FE00FF00FF807FC03FE00FF007FC01FF00),
    .INIT_27(256'h1F81FC0FE07F03F81FC07E03F80FC07F01FC07F01FC07F01FC03F80FF01FC03F),
    .INIT_28(256'hFC1F83F03E07C0FC1F81F03F03E07E07E07C0FC0FC0FC0FC0FE07E07E03F03F0),
    .INIT_29(256'h7C3E0F83E0F07C1F07C1F07C1F07C0F83E0F83E07C1F03E0F81F07E0F81F03E0),
    .INIT_2A(256'h783C3E1F0F0783C1E1F0F87C3E1F0F83C1E0F07C3E0F07C3E0F07C3E0F83C1F0),
    .INIT_2B(256'hE1E1E1E1E1E1E1E1E0F0F0F0F0F878787C3C3C3E1E1F0F0F8787C3C3E1E0F0F8),
    .INIT_2C(256'hC787870F0F0E1E1E3C3C3C3878787870F0F0F0F0E1E1E1E1E1E1E1E1E1E1E1E1),
    .INIT_2D(256'h78F0E1E3C3878F0E1E1C3C7870F0E1E3C3C7870F0E1E1C3C387870F0E1E1C3C3),
    .INIT_2E(256'h3C3878F1E1C3C78F0E1E3C7870F1E3C3878F0E1C3C7870F1E3C3878F0E1E3C38),
    .INIT_2F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_30(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_31(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_32(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_33(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_34(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_35(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_36(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_37(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_38(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_39(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3A(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3B(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3C(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3D(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3E(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_3F(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_40(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_41(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_42(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_43(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_44(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_45(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_46(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_47(256'hFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF),
    .INIT_48(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_49(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_4F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_50(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_51(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_52(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_53(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_54(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_55(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_56(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_57(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_58(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_59(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_5F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_60(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_61(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_62(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_63(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_64(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_65(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_66(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_67(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_68(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_69(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_6F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_70(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_71(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_72(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_73(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_74(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_75(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_76(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_77(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_78(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_79(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7A(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7B(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7C(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7D(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7E(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_7F(256'h0000000000000000000000000000000000000000000000000000000000000000),
    .INIT_A(36'h000000000),
    .INIT_B(36'h000000000),
    .RAM_EXTENSION_A("UPPER"),
    .RAM_EXTENSION_B("NONE"),
    .RAM_MODE("TDP"),
    .RDADDR_COLLISION_HWCONFIG("PERFORMANCE"),
    .READ_WIDTH_A(1),
    .READ_WIDTH_B(0),
    .RSTREG_PRIORITY_A("RSTREG"),
    .RSTREG_PRIORITY_B("RSTREG"),
    .SIM_COLLISION_CHECK("ALL"),
    .SIM_DEVICE("7SERIES"),
    .SRVAL_A(36'h000000000),
    .SRVAL_B(36'h000000000),
    .WRITE_MODE_A("WRITE_FIRST"),
    .WRITE_MODE_B("WRITE_FIRST"),
    .WRITE_WIDTH_A(1),
    .WRITE_WIDTH_B(0)) 
    q0_reg_1_9
       (.ADDRARDADDR(P),
        .ADDRBWRADDR({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CASCADEINA(q0_reg_0_9_n_0),
        .CASCADEINB(1'b0),
        .CASCADEOUTA(NLW_q0_reg_1_9_CASCADEOUTA_UNCONNECTED),
        .CASCADEOUTB(NLW_q0_reg_1_9_CASCADEOUTB_UNCONNECTED),
        .CLKARDCLK(ap_clk),
        .CLKBWRCLK(1'b0),
        .DBITERR(NLW_q0_reg_1_9_DBITERR_UNCONNECTED),
        .DIADI({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .DIBDI({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .DIPADIP({1'b0,1'b0,1'b0,NLW_q0_reg_1_9_DIPADIP_UNCONNECTED[0]}),
        .DIPBDIP({1'b1,1'b1,1'b1,1'b1}),
        .DOADO({NLW_q0_reg_1_9_DOADO_UNCONNECTED[31:1],GAUSS_LUT_q0[9]}),
        .DOBDO(NLW_q0_reg_1_9_DOBDO_UNCONNECTED[31:0]),
        .DOPADOP(NLW_q0_reg_1_9_DOPADOP_UNCONNECTED[3:0]),
        .DOPBDOP(NLW_q0_reg_1_9_DOPBDOP_UNCONNECTED[3:0]),
        .ECCPARITY(NLW_q0_reg_1_9_ECCPARITY_UNCONNECTED[7:0]),
        .ENARDEN(q0_reg_1_9_0),
        .ENBWREN(1'b0),
        .INJECTDBITERR(NLW_q0_reg_1_9_INJECTDBITERR_UNCONNECTED),
        .INJECTSBITERR(NLW_q0_reg_1_9_INJECTSBITERR_UNCONNECTED),
        .RDADDRECC(NLW_q0_reg_1_9_RDADDRECC_UNCONNECTED[8:0]),
        .REGCEAREGCE(1'b0),
        .REGCEB(1'b0),
        .RSTRAMARSTRAM(1'b0),
        .RSTRAMB(1'b0),
        .RSTREGARSTREG(1'b0),
        .RSTREGB(1'b0),
        .SBITERR(NLW_q0_reg_1_9_SBITERR_UNCONNECTED),
        .WEA({1'b0,1'b0,1'b0,1'b0}),
        .WEBWE({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2_mac_muladd_3ns_13ns_13ns_16_4_1" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_mac_muladd_3ns_13ns_13ns_16_4_1
   (P,
    B_V_data_1_sel0,
    ap_clk,
    A,
    Q);
  output [15:0]P;
  input B_V_data_1_sel0;
  input ap_clk;
  input [2:0]A;
  input [12:0]Q;

  wire [2:0]A;
  wire B_V_data_1_sel0;
  wire [15:0]P;
  wire [12:0]Q;
  wire ap_clk;

  system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_mac_muladd_3ns_13ns_13ns_16_4_1_DSP48_0 gaussian_shaping_v2_mac_muladd_3ns_13ns_13ns_16_4_1_DSP48_0_U
       (.A(A),
        .B_V_data_1_sel0(B_V_data_1_sel0),
        .P(P),
        .Q(Q),
        .ap_clk(ap_clk));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2_mac_muladd_3ns_13ns_13ns_16_4_1_DSP48_0" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_mac_muladd_3ns_13ns_13ns_16_4_1_DSP48_0
   (P,
    B_V_data_1_sel0,
    ap_clk,
    A,
    Q);
  output [15:0]P;
  input B_V_data_1_sel0;
  input ap_clk;
  input [2:0]A;
  input [12:0]Q;

  wire [2:0]A;
  wire B_V_data_1_sel0;
  wire [15:0]P;
  wire [12:0]Q;
  wire ap_clk;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:16]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(1),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(1),
    .AUTORESET_PATDET("NO_RESET"),
    .A_INPUT("DIRECT"),
    .BCASCREG(0),
    .BREG(0),
    .B_INPUT("DIRECT"),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .CREG(1),
    .DREG(1),
    .INMODEREG(0),
    .MASK(48'h3FFFFFFFFFFF),
    .MREG(1),
    .OPMODEREG(0),
    .PATTERN(48'h000000000000),
    .PREG(1),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_DPORT("FALSE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,A}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,Q}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(1'b0),
        .CEA2(B_V_data_1_sel0),
        .CEAD(1'b0),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(B_V_data_1_sel0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(1'b0),
        .CEINMODE(1'b0),
        .CEM(B_V_data_1_sel0),
        .CEP(B_V_data_1_sel0),
        .CLK(ap_clk),
        .D({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .INMODE({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b1,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:16],P}),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT(NLW_p_reg_reg_PCOUT_UNCONNECTED[47:0]),
        .RSTA(1'b0),
        .RSTALLCARRYIN(1'b0),
        .RSTALUMODE(1'b0),
        .RSTB(1'b0),
        .RSTC(1'b0),
        .RSTCTRL(1'b0),
        .RSTD(1'b0),
        .RSTINMODE(1'b0),
        .RSTM(1'b0),
        .RSTP(1'b0),
        .UNDERFLOW(NLW_p_reg_reg_UNDERFLOW_UNCONNECTED));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2_regslice_both" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both
   (\B_V_data_1_state_reg[1]_0 ,
    in_stream_TVALID_int_regslice,
    B_V_data_1_sel_rd_reg_0,
    or_ln_fu_157_p3,
    ap_rst_n_inv,
    ap_clk,
    B_V_data_1_sel_rd_reg_1,
    B_V_data_1_sel0,
    in_stream_TVALID,
    in_stream_TDATA);
  output \B_V_data_1_state_reg[1]_0 ;
  output in_stream_TVALID_int_regslice;
  output B_V_data_1_sel_rd_reg_0;
  output [0:0]or_ln_fu_157_p3;
  input ap_rst_n_inv;
  input ap_clk;
  input B_V_data_1_sel_rd_reg_1;
  input B_V_data_1_sel0;
  input in_stream_TVALID;
  input [0:0]in_stream_TDATA;

  wire \B_V_data_1_payload_A[0]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg_n_0_[0] ;
  wire \B_V_data_1_payload_B[0]_i_1_n_0 ;
  wire \B_V_data_1_payload_B_reg_n_0_[0] ;
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_reg_0;
  wire B_V_data_1_sel_rd_reg_1;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__2_n_0 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire ap_clk;
  wire ap_rst_n_inv;
  wire [0:0]in_stream_TDATA;
  wire in_stream_TVALID;
  wire in_stream_TVALID_int_regslice;
  wire [0:0]or_ln_fu_157_p3;

  LUT5 #(
    .INIT(32'hEFEE2022)) 
    \B_V_data_1_payload_A[0]_i_1 
       (.I0(in_stream_TDATA),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(in_stream_TVALID_int_regslice),
        .I4(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .O(\B_V_data_1_payload_A[0]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[0]_i_1_n_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  LUT5 #(
    .INIT(32'hBFBB8088)) 
    \B_V_data_1_payload_B[0]_i_1 
       (.I0(in_stream_TDATA),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(in_stream_TVALID_int_regslice),
        .I4(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .O(\B_V_data_1_payload_B[0]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[0]_i_1_n_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_reg_1),
        .Q(B_V_data_1_sel_rd_reg_0),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__1
       (.I0(\B_V_data_1_state_reg[1]_0 ),
        .I1(in_stream_TVALID),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT4 #(
    .INIT(16'hD8F8)) 
    \B_V_data_1_state[0]_i_1__2 
       (.I0(\B_V_data_1_state_reg[1]_0 ),
        .I1(in_stream_TVALID),
        .I2(in_stream_TVALID_int_regslice),
        .I3(B_V_data_1_sel0),
        .O(\B_V_data_1_state[0]_i_1__2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT4 #(
    .INIT(16'hDFDD)) 
    \B_V_data_1_state[1]_i_2 
       (.I0(in_stream_TVALID_int_regslice),
        .I1(B_V_data_1_sel0),
        .I2(in_stream_TVALID),
        .I3(\B_V_data_1_state_reg[1]_0 ),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__2_n_0 ),
        .Q(in_stream_TVALID_int_regslice),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg[1]_0 ),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'hB8)) 
    \history[0]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(B_V_data_1_sel_rd_reg_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .O(or_ln_fu_157_p3));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2_regslice_both" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both__parameterized0
   (ap_rst_n_inv,
    fcw_out_TLAST,
    ap_clk,
    fcw_out_TREADY,
    ap_enable_reg_pp0_iter4,
    B_V_data_1_sel0,
    ap_rst_n,
    input_pkt_last_V_reg_212_pp0_iter3_reg);
  output ap_rst_n_inv;
  output [0:0]fcw_out_TLAST;
  input ap_clk;
  input fcw_out_TREADY;
  input ap_enable_reg_pp0_iter4;
  input B_V_data_1_sel0;
  input ap_rst_n;
  input input_pkt_last_V_reg_212_pp0_iter3_reg;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__1_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1__1_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_i_1__3_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__0_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__1_n_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter4;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [0:0]fcw_out_TLAST;
  wire fcw_out_TREADY;
  wire input_pkt_last_V_reg_212_pp0_iter3_reg;

  LUT5 #(
    .INIT(32'hEFEE2022)) 
    \B_V_data_1_payload_A[0]_i_1__1 
       (.I0(input_pkt_last_V_reg_212_pp0_iter3_reg),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg_n_0_[0] ),
        .I4(B_V_data_1_payload_A),
        .O(\B_V_data_1_payload_A[0]_i_1__1_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[0]_i_1__1_n_0 ),
        .Q(B_V_data_1_payload_A),
        .R(1'b0));
  LUT5 #(
    .INIT(32'hBFBB8088)) 
    \B_V_data_1_payload_B[0]_i_1__1 
       (.I0(input_pkt_last_V_reg_212_pp0_iter3_reg),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg_n_0_[0] ),
        .I4(B_V_data_1_payload_B),
        .O(\B_V_data_1_payload_B[0]_i_1__1_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[0]_i_1__1_n_0 ),
        .Q(B_V_data_1_payload_B),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__3
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(fcw_out_TREADY),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__3_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__3_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT4 #(
    .INIT(16'h7F80)) 
    B_V_data_1_sel_wr_i_1__0
       (.I0(ap_enable_reg_pp0_iter4),
        .I1(B_V_data_1_sel0),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__0_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__0_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT5 #(
    .INIT(32'h88F8F0F0)) 
    \B_V_data_1_state[0]_i_1__1 
       (.I0(ap_enable_reg_pp0_iter4),
        .I1(B_V_data_1_sel0),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(fcw_out_TREADY),
        .I4(\B_V_data_1_state_reg_n_0_[1] ),
        .O(\B_V_data_1_state[0]_i_1__1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT5 #(
    .INIT(32'hBBFBFBFB)) 
    \B_V_data_1_state[1]_i_1__1 
       (.I0(fcw_out_TREADY),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(ap_enable_reg_pp0_iter4),
        .I4(B_V_data_1_sel0),
        .O(B_V_data_1_state));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_state[1]_i_1__3 
       (.I0(ap_rst_n),
        .O(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__1_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \fcw_out_TLAST[0]_INST_0 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(fcw_out_TLAST));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2_regslice_both" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both__parameterized0_0
   (in_stream_TLAST_int_regslice,
    ap_rst_n_inv,
    ap_clk,
    B_V_data_1_sel0,
    in_stream_TVALID,
    in_stream_TLAST);
  output in_stream_TLAST_int_regslice;
  input ap_rst_n_inv;
  input ap_clk;
  input B_V_data_1_sel0;
  input in_stream_TVALID;
  input [0:0]in_stream_TLAST;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__0_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1__0_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_i_1_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__3_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1_n_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n_inv;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TLAST_int_regslice;
  wire in_stream_TVALID;

  LUT5 #(
    .INIT(32'hEFEE2022)) 
    \B_V_data_1_payload_A[0]_i_1__0 
       (.I0(in_stream_TLAST),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg_n_0_[0] ),
        .I4(B_V_data_1_payload_A),
        .O(\B_V_data_1_payload_A[0]_i_1__0_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[0]_i_1__0_n_0 ),
        .Q(B_V_data_1_payload_A),
        .R(1'b0));
  LUT5 #(
    .INIT(32'hBFBB8088)) 
    \B_V_data_1_payload_B[0]_i_1__0 
       (.I0(in_stream_TLAST),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg_n_0_[0] ),
        .I4(B_V_data_1_payload_B),
        .O(\B_V_data_1_payload_B[0]_i_1__0_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[0]_i_1__0_n_0 ),
        .Q(B_V_data_1_payload_B),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1
       (.I0(B_V_data_1_sel0),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__3
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(in_stream_TVALID),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__3_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__3_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT4 #(
    .INIT(16'hD8F8)) 
    \B_V_data_1_state[0]_i_1 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(in_stream_TVALID),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(B_V_data_1_sel0),
        .O(\B_V_data_1_state[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT4 #(
    .INIT(16'hDFDD)) 
    \B_V_data_1_state[1]_i_1 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(B_V_data_1_sel0),
        .I2(in_stream_TVALID),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_last_V_reg_212_pp0_iter2_reg_reg[0]_srl3_i_1 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(in_stream_TLAST_int_regslice));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2_regslice_both" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both__parameterized1
   (\history_reg[1] ,
    \history_reg[0] ,
    \history_reg[0]_0 ,
    A,
    D,
    ap_rst_n_inv,
    ap_clk,
    or_ln_fu_157_p3,
    B_V_data_1_sel0,
    \history_reg[2] ,
    in_stream_TVALID,
    in_stream_TUSER);
  output \history_reg[1] ;
  output \history_reg[0] ;
  output \history_reg[0]_0 ;
  output [2:0]A;
  output [12:0]D;
  input ap_rst_n_inv;
  input ap_clk;
  input [2:0]or_ln_fu_157_p3;
  input B_V_data_1_sel0;
  input \history_reg[2] ;
  input in_stream_TVALID;
  input [12:0]in_stream_TUSER;

  wire [2:0]A;
  wire B_V_data_1_load_A;
  wire B_V_data_1_load_B;
  wire \B_V_data_1_payload_A_reg_n_0_[0] ;
  wire \B_V_data_1_payload_A_reg_n_0_[10] ;
  wire \B_V_data_1_payload_A_reg_n_0_[11] ;
  wire \B_V_data_1_payload_A_reg_n_0_[12] ;
  wire \B_V_data_1_payload_A_reg_n_0_[1] ;
  wire \B_V_data_1_payload_A_reg_n_0_[2] ;
  wire \B_V_data_1_payload_A_reg_n_0_[3] ;
  wire \B_V_data_1_payload_A_reg_n_0_[4] ;
  wire \B_V_data_1_payload_A_reg_n_0_[5] ;
  wire \B_V_data_1_payload_A_reg_n_0_[6] ;
  wire \B_V_data_1_payload_A_reg_n_0_[7] ;
  wire \B_V_data_1_payload_A_reg_n_0_[8] ;
  wire \B_V_data_1_payload_A_reg_n_0_[9] ;
  wire \B_V_data_1_payload_B_reg_n_0_[0] ;
  wire \B_V_data_1_payload_B_reg_n_0_[10] ;
  wire \B_V_data_1_payload_B_reg_n_0_[11] ;
  wire \B_V_data_1_payload_B_reg_n_0_[12] ;
  wire \B_V_data_1_payload_B_reg_n_0_[1] ;
  wire \B_V_data_1_payload_B_reg_n_0_[2] ;
  wire \B_V_data_1_payload_B_reg_n_0_[3] ;
  wire \B_V_data_1_payload_B_reg_n_0_[4] ;
  wire \B_V_data_1_payload_B_reg_n_0_[5] ;
  wire \B_V_data_1_payload_B_reg_n_0_[6] ;
  wire \B_V_data_1_payload_B_reg_n_0_[7] ;
  wire \B_V_data_1_payload_B_reg_n_0_[8] ;
  wire \B_V_data_1_payload_B_reg_n_0_[9] ;
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_i_1__0_n_0;
  wire B_V_data_1_sel_rd_reg_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__2_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__0_n_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire [12:0]D;
  wire ap_clk;
  wire ap_rst_n_inv;
  wire \history[2]_i_2_n_0 ;
  wire \history[2]_i_3_n_0 ;
  wire \history[2]_i_4_n_0 ;
  wire \history[2]_i_5_n_0 ;
  wire \history[2]_i_6_n_0 ;
  wire \history[2]_i_7_n_0 ;
  wire \history[2]_i_8_n_0 ;
  wire \history_reg[0] ;
  wire \history_reg[0]_0 ;
  wire \history_reg[1] ;
  wire \history_reg[2] ;
  wire [12:0]in_stream_TUSER;
  wire in_stream_TVALID;
  wire [2:0]or_ln_fu_157_p3;
  wire p_reg_reg_i_4_n_0;
  wire p_reg_reg_i_5_n_0;

  LUT3 #(
    .INIT(8'h45)) 
    \B_V_data_1_payload_A[12]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[0]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[10]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[11]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[12]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[1]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[3]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[4]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[5]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[6]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[7]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[8]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TUSER[9]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .R(1'b0));
  LUT3 #(
    .INIT(8'h8A)) 
    \B_V_data_1_payload_B[12]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[0]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[10]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[11]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[12]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[1]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[3]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[4]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[5]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[6]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[7]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[8]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TUSER[9]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__0
       (.I0(B_V_data_1_sel0),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(B_V_data_1_sel_rd_i_1__0_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__0_n_0),
        .Q(B_V_data_1_sel_rd_reg_n_0),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__2
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(in_stream_TVALID),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__2_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__2_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT4 #(
    .INIT(16'hD8F8)) 
    \B_V_data_1_state[0]_i_1__0 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(in_stream_TVALID),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(B_V_data_1_sel0),
        .O(\B_V_data_1_state[0]_i_1__0_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT4 #(
    .INIT(16'hDFDD)) 
    \B_V_data_1_state[1]_i_1__0 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(B_V_data_1_sel0),
        .I2(in_stream_TVALID),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__0_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'hFFFFFBFF00000800)) 
    \history[0]_i_1 
       (.I0(or_ln_fu_157_p3[0]),
        .I1(B_V_data_1_sel0),
        .I2(\history[2]_i_2_n_0 ),
        .I3(\history[2]_i_3_n_0 ),
        .I4(\history[2]_i_4_n_0 ),
        .I5(or_ln_fu_157_p3[1]),
        .O(\history_reg[0]_0 ));
  LUT6 #(
    .INIT(64'hFFFFFBFF00000800)) 
    \history[1]_i_1 
       (.I0(or_ln_fu_157_p3[1]),
        .I1(B_V_data_1_sel0),
        .I2(\history[2]_i_2_n_0 ),
        .I3(\history[2]_i_3_n_0 ),
        .I4(\history[2]_i_4_n_0 ),
        .I5(or_ln_fu_157_p3[2]),
        .O(\history_reg[0] ));
  LUT6 #(
    .INIT(64'hFFFFFBFF00000800)) 
    \history[2]_i_1 
       (.I0(or_ln_fu_157_p3[2]),
        .I1(B_V_data_1_sel0),
        .I2(\history[2]_i_2_n_0 ),
        .I3(\history[2]_i_3_n_0 ),
        .I4(\history[2]_i_4_n_0 ),
        .I5(\history_reg[2] ),
        .O(\history_reg[1] ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFEFEA)) 
    \history[2]_i_2 
       (.I0(\history[2]_i_5_n_0 ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .I3(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I4(\history[2]_i_6_n_0 ),
        .I5(\history[2]_i_7_n_0 ),
        .O(\history[2]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT5 #(
    .INIT(32'h00053305)) 
    \history[2]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I2(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I3(B_V_data_1_sel_rd_reg_n_0),
        .I4(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .O(\history[2]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFBBFCB8)) 
    \history[2]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I4(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I5(\history[2]_i_8_n_0 ),
        .O(\history[2]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT5 #(
    .INIT(32'hFFFACCFA)) 
    \history[2]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I2(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I3(B_V_data_1_sel_rd_reg_n_0),
        .I4(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .O(\history[2]_i_5_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT5 #(
    .INIT(32'hFFFACCFA)) 
    \history[2]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I2(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I3(B_V_data_1_sel_rd_reg_n_0),
        .I4(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .O(\history[2]_i_6_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT5 #(
    .INIT(32'hFFFACCFA)) 
    \history[2]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I2(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I3(B_V_data_1_sel_rd_reg_n_0),
        .I4(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .O(\history[2]_i_7_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT5 #(
    .INIT(32'hFFFACCFA)) 
    \history[2]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I2(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I3(B_V_data_1_sel_rd_reg_n_0),
        .I4(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .O(\history[2]_i_8_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[0]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .O(D[0]));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[10]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .O(D[10]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[11]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .O(D[11]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[12]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .O(D[12]));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[1]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .O(D[1]));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[2]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .O(D[2]));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[3]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .O(D[3]));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[4]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .O(D[4]));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[5]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .O(D[5]));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[6]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .O(D[6]));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[7]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .O(D[7]));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[8]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .O(D[8]));
  LUT3 #(
    .INIT(8'hB8)) 
    \input_pkt_user_V_reg_207[9]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .O(D[9]));
  LUT6 #(
    .INIT(64'hFFFFFFEF00000020)) 
    p_reg_reg_i_1
       (.I0(or_ln_fu_157_p3[2]),
        .I1(\history[2]_i_4_n_0 ),
        .I2(\history[2]_i_3_n_0 ),
        .I3(p_reg_reg_i_4_n_0),
        .I4(p_reg_reg_i_5_n_0),
        .I5(\history_reg[2] ),
        .O(A[2]));
  LUT6 #(
    .INIT(64'hFFFFFFEF00000020)) 
    p_reg_reg_i_2
       (.I0(or_ln_fu_157_p3[1]),
        .I1(\history[2]_i_4_n_0 ),
        .I2(\history[2]_i_3_n_0 ),
        .I3(p_reg_reg_i_4_n_0),
        .I4(p_reg_reg_i_5_n_0),
        .I5(or_ln_fu_157_p3[2]),
        .O(A[1]));
  LUT6 #(
    .INIT(64'hFFFFFFEF00000020)) 
    p_reg_reg_i_3
       (.I0(or_ln_fu_157_p3[0]),
        .I1(\history[2]_i_4_n_0 ),
        .I2(\history[2]_i_3_n_0 ),
        .I3(p_reg_reg_i_4_n_0),
        .I4(p_reg_reg_i_5_n_0),
        .I5(or_ln_fu_157_p3[1]),
        .O(A[0]));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFBBFCB8)) 
    p_reg_reg_i_4
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I4(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I5(\history[2]_i_6_n_0 ),
        .O(p_reg_reg_i_4_n_0));
  LUT6 #(
    .INIT(64'hFFFCFFFFFFFCFAFA)) 
    p_reg_reg_i_5
       (.I0(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I2(D[2]),
        .I3(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I4(B_V_data_1_sel_rd_reg_n_0),
        .I5(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .O(p_reg_reg_i_5_n_0));
endmodule

(* ORIG_REF_NAME = "gaussian_shaping_v2_regslice_both" *) 
module system_gaussian_shaping_v2_0_0_gaussian_shaping_v2_regslice_both__parameterized2
   (\B_V_data_1_state_reg[0]_0 ,
    B_V_data_1_sel0,
    B_V_data_1_sel_rd_reg_0,
    RDEN,
    ap_enable_reg_pp0_iter3_reg,
    ap_enable_reg_pp0_iter3_reg_0,
    ap_enable_reg_pp0_iter3_reg_1,
    ap_enable_reg_pp0_iter3_reg_2,
    ap_enable_reg_pp0_iter3_reg_3,
    ap_enable_reg_pp0_iter3_reg_4,
    ap_enable_reg_pp0_iter3_reg_5,
    ap_enable_reg_pp0_iter3_reg_6,
    ap_enable_reg_pp0_iter3_reg_7,
    ap_enable_reg_pp0_iter3_reg_8,
    ap_enable_reg_pp0_iter3_reg_9,
    ap_enable_reg_pp0_iter3_reg_10,
    ap_enable_reg_pp0_iter3_reg_11,
    ap_enable_reg_pp0_iter3_reg_12,
    ap_enable_reg_pp0_iter3_reg_13,
    ap_enable_reg_pp0_iter3_reg_14,
    ap_enable_reg_pp0_iter3_reg_15,
    ap_enable_reg_pp0_iter3_reg_16,
    ap_enable_reg_pp0_iter3_reg_17,
    ap_enable_reg_pp0_iter3_reg_18,
    ap_enable_reg_pp0_iter3_reg_19,
    ap_enable_reg_pp0_iter3_reg_20,
    ap_enable_reg_pp0_iter3_reg_21,
    ap_enable_reg_pp0_iter3_reg_22,
    ap_enable_reg_pp0_iter3_reg_23,
    ap_enable_reg_pp0_iter3_reg_24,
    ap_enable_reg_pp0_iter3_reg_25,
    ap_enable_reg_pp0_iter3_reg_26,
    ap_enable_reg_pp0_iter3_reg_27,
    ap_enable_reg_pp0_iter3_reg_28,
    ap_enable_reg_pp0_iter3_reg_29,
    ap_enable_reg_pp0_iter3_reg_30,
    ap_enable_reg_pp0_iter3_reg_31,
    ap_enable_reg_pp0_iter3_reg_32,
    ap_enable_reg_pp0_iter3_reg_33,
    ap_enable_reg_pp0_iter3_reg_34,
    ap_enable_reg_pp0_iter3_reg_35,
    ap_enable_reg_pp0_iter3_reg_36,
    ap_enable_reg_pp0_iter3_reg_37,
    fcw_out_TDATA,
    ap_rst_n_inv,
    ap_clk,
    ap_enable_reg_pp0_iter4,
    in_stream_TVALID_int_regslice,
    fcw_out_TREADY,
    ap_enable_reg_pp0_iter5,
    B_V_data_1_sel_rd_reg_1,
    ap_enable_reg_pp0_iter3,
    D);
  output \B_V_data_1_state_reg[0]_0 ;
  output B_V_data_1_sel0;
  output B_V_data_1_sel_rd_reg_0;
  output RDEN;
  output ap_enable_reg_pp0_iter3_reg;
  output ap_enable_reg_pp0_iter3_reg_0;
  output ap_enable_reg_pp0_iter3_reg_1;
  output ap_enable_reg_pp0_iter3_reg_2;
  output ap_enable_reg_pp0_iter3_reg_3;
  output ap_enable_reg_pp0_iter3_reg_4;
  output ap_enable_reg_pp0_iter3_reg_5;
  output ap_enable_reg_pp0_iter3_reg_6;
  output ap_enable_reg_pp0_iter3_reg_7;
  output ap_enable_reg_pp0_iter3_reg_8;
  output ap_enable_reg_pp0_iter3_reg_9;
  output ap_enable_reg_pp0_iter3_reg_10;
  output ap_enable_reg_pp0_iter3_reg_11;
  output ap_enable_reg_pp0_iter3_reg_12;
  output ap_enable_reg_pp0_iter3_reg_13;
  output ap_enable_reg_pp0_iter3_reg_14;
  output ap_enable_reg_pp0_iter3_reg_15;
  output ap_enable_reg_pp0_iter3_reg_16;
  output ap_enable_reg_pp0_iter3_reg_17;
  output ap_enable_reg_pp0_iter3_reg_18;
  output ap_enable_reg_pp0_iter3_reg_19;
  output ap_enable_reg_pp0_iter3_reg_20;
  output ap_enable_reg_pp0_iter3_reg_21;
  output ap_enable_reg_pp0_iter3_reg_22;
  output ap_enable_reg_pp0_iter3_reg_23;
  output ap_enable_reg_pp0_iter3_reg_24;
  output ap_enable_reg_pp0_iter3_reg_25;
  output ap_enable_reg_pp0_iter3_reg_26;
  output ap_enable_reg_pp0_iter3_reg_27;
  output ap_enable_reg_pp0_iter3_reg_28;
  output ap_enable_reg_pp0_iter3_reg_29;
  output ap_enable_reg_pp0_iter3_reg_30;
  output ap_enable_reg_pp0_iter3_reg_31;
  output ap_enable_reg_pp0_iter3_reg_32;
  output ap_enable_reg_pp0_iter3_reg_33;
  output ap_enable_reg_pp0_iter3_reg_34;
  output ap_enable_reg_pp0_iter3_reg_35;
  output ap_enable_reg_pp0_iter3_reg_36;
  output ap_enable_reg_pp0_iter3_reg_37;
  output [20:0]fcw_out_TDATA;
  input ap_rst_n_inv;
  input ap_clk;
  input ap_enable_reg_pp0_iter4;
  input in_stream_TVALID_int_regslice;
  input fcw_out_TREADY;
  input ap_enable_reg_pp0_iter5;
  input B_V_data_1_sel_rd_reg_1;
  input ap_enable_reg_pp0_iter3;
  input [20:0]D;

  wire B_V_data_1_load_A;
  wire B_V_data_1_load_B;
  wire \B_V_data_1_payload_A_reg_n_0_[0] ;
  wire \B_V_data_1_payload_A_reg_n_0_[10] ;
  wire \B_V_data_1_payload_A_reg_n_0_[11] ;
  wire \B_V_data_1_payload_A_reg_n_0_[12] ;
  wire \B_V_data_1_payload_A_reg_n_0_[13] ;
  wire \B_V_data_1_payload_A_reg_n_0_[14] ;
  wire \B_V_data_1_payload_A_reg_n_0_[15] ;
  wire \B_V_data_1_payload_A_reg_n_0_[16] ;
  wire \B_V_data_1_payload_A_reg_n_0_[17] ;
  wire \B_V_data_1_payload_A_reg_n_0_[18] ;
  wire \B_V_data_1_payload_A_reg_n_0_[19] ;
  wire \B_V_data_1_payload_A_reg_n_0_[1] ;
  wire \B_V_data_1_payload_A_reg_n_0_[20] ;
  wire \B_V_data_1_payload_A_reg_n_0_[2] ;
  wire \B_V_data_1_payload_A_reg_n_0_[3] ;
  wire \B_V_data_1_payload_A_reg_n_0_[4] ;
  wire \B_V_data_1_payload_A_reg_n_0_[5] ;
  wire \B_V_data_1_payload_A_reg_n_0_[6] ;
  wire \B_V_data_1_payload_A_reg_n_0_[7] ;
  wire \B_V_data_1_payload_A_reg_n_0_[8] ;
  wire \B_V_data_1_payload_A_reg_n_0_[9] ;
  wire \B_V_data_1_payload_B_reg_n_0_[0] ;
  wire \B_V_data_1_payload_B_reg_n_0_[10] ;
  wire \B_V_data_1_payload_B_reg_n_0_[11] ;
  wire \B_V_data_1_payload_B_reg_n_0_[12] ;
  wire \B_V_data_1_payload_B_reg_n_0_[13] ;
  wire \B_V_data_1_payload_B_reg_n_0_[14] ;
  wire \B_V_data_1_payload_B_reg_n_0_[15] ;
  wire \B_V_data_1_payload_B_reg_n_0_[16] ;
  wire \B_V_data_1_payload_B_reg_n_0_[17] ;
  wire \B_V_data_1_payload_B_reg_n_0_[18] ;
  wire \B_V_data_1_payload_B_reg_n_0_[19] ;
  wire \B_V_data_1_payload_B_reg_n_0_[1] ;
  wire \B_V_data_1_payload_B_reg_n_0_[20] ;
  wire \B_V_data_1_payload_B_reg_n_0_[2] ;
  wire \B_V_data_1_payload_B_reg_n_0_[3] ;
  wire \B_V_data_1_payload_B_reg_n_0_[4] ;
  wire \B_V_data_1_payload_B_reg_n_0_[5] ;
  wire \B_V_data_1_payload_B_reg_n_0_[6] ;
  wire \B_V_data_1_payload_B_reg_n_0_[7] ;
  wire \B_V_data_1_payload_B_reg_n_0_[8] ;
  wire \B_V_data_1_payload_B_reg_n_0_[9] ;
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_i_1__2_n_0;
  wire B_V_data_1_sel_rd_reg_0;
  wire B_V_data_1_sel_rd_reg_1;
  wire B_V_data_1_sel_rd_reg_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__3_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire [20:0]D;
  wire RDEN;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter3;
  wire ap_enable_reg_pp0_iter3_reg;
  wire ap_enable_reg_pp0_iter3_reg_0;
  wire ap_enable_reg_pp0_iter3_reg_1;
  wire ap_enable_reg_pp0_iter3_reg_10;
  wire ap_enable_reg_pp0_iter3_reg_11;
  wire ap_enable_reg_pp0_iter3_reg_12;
  wire ap_enable_reg_pp0_iter3_reg_13;
  wire ap_enable_reg_pp0_iter3_reg_14;
  wire ap_enable_reg_pp0_iter3_reg_15;
  wire ap_enable_reg_pp0_iter3_reg_16;
  wire ap_enable_reg_pp0_iter3_reg_17;
  wire ap_enable_reg_pp0_iter3_reg_18;
  wire ap_enable_reg_pp0_iter3_reg_19;
  wire ap_enable_reg_pp0_iter3_reg_2;
  wire ap_enable_reg_pp0_iter3_reg_20;
  wire ap_enable_reg_pp0_iter3_reg_21;
  wire ap_enable_reg_pp0_iter3_reg_22;
  wire ap_enable_reg_pp0_iter3_reg_23;
  wire ap_enable_reg_pp0_iter3_reg_24;
  wire ap_enable_reg_pp0_iter3_reg_25;
  wire ap_enable_reg_pp0_iter3_reg_26;
  wire ap_enable_reg_pp0_iter3_reg_27;
  wire ap_enable_reg_pp0_iter3_reg_28;
  wire ap_enable_reg_pp0_iter3_reg_29;
  wire ap_enable_reg_pp0_iter3_reg_3;
  wire ap_enable_reg_pp0_iter3_reg_30;
  wire ap_enable_reg_pp0_iter3_reg_31;
  wire ap_enable_reg_pp0_iter3_reg_32;
  wire ap_enable_reg_pp0_iter3_reg_33;
  wire ap_enable_reg_pp0_iter3_reg_34;
  wire ap_enable_reg_pp0_iter3_reg_35;
  wire ap_enable_reg_pp0_iter3_reg_36;
  wire ap_enable_reg_pp0_iter3_reg_37;
  wire ap_enable_reg_pp0_iter3_reg_4;
  wire ap_enable_reg_pp0_iter3_reg_5;
  wire ap_enable_reg_pp0_iter3_reg_6;
  wire ap_enable_reg_pp0_iter3_reg_7;
  wire ap_enable_reg_pp0_iter3_reg_8;
  wire ap_enable_reg_pp0_iter3_reg_9;
  wire ap_enable_reg_pp0_iter4;
  wire ap_enable_reg_pp0_iter5;
  wire ap_rst_n_inv;
  wire [20:0]fcw_out_TDATA;
  wire fcw_out_TREADY;
  wire in_stream_TVALID_int_regslice;

  LUT3 #(
    .INIT(8'h45)) 
    \B_V_data_1_payload_A[20]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[0]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[10]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[11]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[12]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[13]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[14]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[15] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[15]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[16] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[16]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[17] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[17]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[18] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[18]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[19] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[19]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[1]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[20] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[20]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[3]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[4]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[5]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[6]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[7]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[8]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[9]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .R(1'b0));
  LUT3 #(
    .INIT(8'h8A)) 
    \B_V_data_1_payload_B[20]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[0]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[10]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[11]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[12]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[13]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[14]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[15] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[15]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[16] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[16]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[17] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[17]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[18] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[18]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[19] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[19]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[1]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[20] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[20]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[3]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[4]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[5]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[6]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[7]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[8]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[9]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT2 #(
    .INIT(4'h6)) 
    B_V_data_1_sel_rd_i_1__1
       (.I0(B_V_data_1_sel0),
        .I1(B_V_data_1_sel_rd_reg_1),
        .O(B_V_data_1_sel_rd_reg_0));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__2
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(fcw_out_TREADY),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(B_V_data_1_sel_rd_i_1__2_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__2_n_0),
        .Q(B_V_data_1_sel_rd_reg_n_0),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1
       (.I0(B_V_data_1_sel0),
        .I1(ap_enable_reg_pp0_iter4),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'h88F0F8F0)) 
    \B_V_data_1_state[0]_i_1__3 
       (.I0(in_stream_TVALID_int_regslice),
        .I1(ap_enable_reg_pp0_iter4),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(fcw_out_TREADY),
        .O(\B_V_data_1_state[0]_i_1__3_n_0 ));
  LUT6 #(
    .INIT(64'hFFAAFFFFFF2AFFFF)) 
    \B_V_data_1_state[1]_i_1__2 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(ap_enable_reg_pp0_iter4),
        .I2(in_stream_TVALID_int_regslice),
        .I3(fcw_out_TREADY),
        .I4(\B_V_data_1_state_reg[0]_0 ),
        .I5(ap_enable_reg_pp0_iter5),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__3_n_0 ),
        .Q(\B_V_data_1_state_reg[0]_0 ),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[0]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[0]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[10]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[10]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[11]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[11]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[12]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[12]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[13]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[13]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[14]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[14]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[15]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[15]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[16]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[16]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[17]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[17]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[18]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[18]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[19]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[19]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[1]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[1]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[20]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[20]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[2]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[2]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[3]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[3]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[4]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[4]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[5]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[5]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[6]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[6]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[7]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[7]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[8]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[8]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \fcw_out_TDATA[9]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(fcw_out_TDATA[9]));
  LUT6 #(
    .INIT(64'hF700F70000005500)) 
    \input_pkt_user_V_reg_207[12]_i_1 
       (.I0(ap_enable_reg_pp0_iter5),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(fcw_out_TREADY),
        .I3(in_stream_TVALID_int_regslice),
        .I4(ap_enable_reg_pp0_iter4),
        .I5(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_sel0));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_0_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_37));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_10_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_17));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_11_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_15));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_12_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_13));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_13_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_11));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_14_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_9));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_15_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_7));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_16_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_5));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_17_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_3));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_18_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_1));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_19_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_1_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_35));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_2_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_33));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_3_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_31));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_4_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_29));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_5_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_27));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_6_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_25));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_7_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_23));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_8_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_21));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_0_9_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_19));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_0_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_36));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_10_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_16));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_11_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_14));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_12_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_12));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_13_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_10));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_14_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_8));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_15_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_6));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_16_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_4));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_17_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_2));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_18_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_0));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_19_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(RDEN));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_1_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_34));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_2_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_32));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_3_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_30));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_4_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_28));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_5_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_26));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_6_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_24));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_7_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_22));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_8_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_20));
  LUT2 #(
    .INIT(4'h8)) 
    q0_reg_1_9_i_1
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(B_V_data_1_sel0),
        .O(ap_enable_reg_pp0_iter3_reg_18));
endmodule
`ifndef GLBL
`define GLBL
`timescale  1 ps / 1 ps

module glbl ();

    parameter ROC_WIDTH = 100000;
    parameter TOC_WIDTH = 0;
    parameter GRES_WIDTH = 10000;
    parameter GRES_START = 10000;

//--------   STARTUP Globals --------------
    wire GSR;
    wire GTS;
    wire GWE;
    wire PRLD;
    wire GRESTORE;
    tri1 p_up_tmp;
    tri (weak1, strong0) PLL_LOCKG = p_up_tmp;

    wire PROGB_GLBL;
    wire CCLKO_GLBL;
    wire FCSBO_GLBL;
    wire [3:0] DO_GLBL;
    wire [3:0] DI_GLBL;
   
    reg GSR_int;
    reg GTS_int;
    reg PRLD_int;
    reg GRESTORE_int;

//--------   JTAG Globals --------------
    wire JTAG_TDO_GLBL;
    wire JTAG_TCK_GLBL;
    wire JTAG_TDI_GLBL;
    wire JTAG_TMS_GLBL;
    wire JTAG_TRST_GLBL;

    reg JTAG_CAPTURE_GLBL;
    reg JTAG_RESET_GLBL;
    reg JTAG_SHIFT_GLBL;
    reg JTAG_UPDATE_GLBL;
    reg JTAG_RUNTEST_GLBL;

    reg JTAG_SEL1_GLBL = 0;
    reg JTAG_SEL2_GLBL = 0 ;
    reg JTAG_SEL3_GLBL = 0;
    reg JTAG_SEL4_GLBL = 0;

    reg JTAG_USER_TDO1_GLBL = 1'bz;
    reg JTAG_USER_TDO2_GLBL = 1'bz;
    reg JTAG_USER_TDO3_GLBL = 1'bz;
    reg JTAG_USER_TDO4_GLBL = 1'bz;

    assign (strong1, weak0) GSR = GSR_int;
    assign (strong1, weak0) GTS = GTS_int;
    assign (weak1, weak0) PRLD = PRLD_int;
    assign (strong1, weak0) GRESTORE = GRESTORE_int;

    initial begin
	GSR_int = 1'b1;
	PRLD_int = 1'b1;
	#(ROC_WIDTH)
	GSR_int = 1'b0;
	PRLD_int = 1'b0;
    end

    initial begin
	GTS_int = 1'b1;
	#(TOC_WIDTH)
	GTS_int = 1'b0;
    end

    initial begin 
	GRESTORE_int = 1'b0;
	#(GRES_START);
	GRESTORE_int = 1'b1;
	#(GRES_WIDTH);
	GRESTORE_int = 1'b0;
    end

endmodule
`endif
