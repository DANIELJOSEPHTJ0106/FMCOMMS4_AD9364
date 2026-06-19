// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Tue Feb  3 16:25:29 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_fsk_phase_corrector_0_0_sim_netlist.v
// Design      : system_fsk_phase_corrector_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* ap_ST_fsm_pp0_stage0 = "1'b1" *) (* hls_module = "yes" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector
   (ap_clk,
    ap_rst_n,
    in_stream_TDATA,
    in_stream_TVALID,
    in_stream_TREADY,
    in_stream_TKEEP,
    in_stream_TSTRB,
    in_stream_TLAST,
    out_stream_TDATA,
    out_stream_TVALID,
    out_stream_TREADY,
    out_stream_TKEEP,
    out_stream_TSTRB,
    out_stream_TLAST);
  input ap_clk;
  input ap_rst_n;
  input [31:0]in_stream_TDATA;
  input in_stream_TVALID;
  output in_stream_TREADY;
  input [3:0]in_stream_TKEEP;
  input [3:0]in_stream_TSTRB;
  input [0:0]in_stream_TLAST;
  output [31:0]out_stream_TDATA;
  output out_stream_TVALID;
  input out_stream_TREADY;
  output [3:0]out_stream_TKEEP;
  output [3:0]out_stream_TSTRB;
  output [0:0]out_stream_TLAST;

  wire [31:0]add_ln32_fu_145_p2;
  wire add_ln32_reg_2830;
  wire [0:0]add_ln33_fu_161_p2;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter2;
  wire ap_enable_reg_pp0_iter3;
  wire ap_enable_reg_pp0_iter4;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [57:16]buff0_reg__1;
  wire calibrated;
  wire calibrated_load_reg_279;
  wire calibrated_load_reg_279_pp0_iter1_reg;
  wire calibrated_load_reg_279_pp0_iter2_reg;
  wire calibrated_load_reg_279_pp0_iter3_reg;
  wire [31:0]counter_reg;
  wire \counter_reg[0]_i_1_n_0 ;
  wire \counter_reg[0]_i_1_n_1 ;
  wire \counter_reg[0]_i_1_n_2 ;
  wire \counter_reg[0]_i_1_n_3 ;
  wire \counter_reg[0]_i_1_n_4 ;
  wire \counter_reg[0]_i_1_n_5 ;
  wire \counter_reg[0]_i_1_n_6 ;
  wire \counter_reg[0]_i_1_n_7 ;
  wire \counter_reg[12]_i_1_n_0 ;
  wire \counter_reg[12]_i_1_n_1 ;
  wire \counter_reg[12]_i_1_n_2 ;
  wire \counter_reg[12]_i_1_n_3 ;
  wire \counter_reg[12]_i_1_n_4 ;
  wire \counter_reg[12]_i_1_n_5 ;
  wire \counter_reg[12]_i_1_n_6 ;
  wire \counter_reg[12]_i_1_n_7 ;
  wire \counter_reg[16]_i_1_n_0 ;
  wire \counter_reg[16]_i_1_n_1 ;
  wire \counter_reg[16]_i_1_n_2 ;
  wire \counter_reg[16]_i_1_n_3 ;
  wire \counter_reg[16]_i_1_n_4 ;
  wire \counter_reg[16]_i_1_n_5 ;
  wire \counter_reg[16]_i_1_n_6 ;
  wire \counter_reg[16]_i_1_n_7 ;
  wire \counter_reg[20]_i_1_n_0 ;
  wire \counter_reg[20]_i_1_n_1 ;
  wire \counter_reg[20]_i_1_n_2 ;
  wire \counter_reg[20]_i_1_n_3 ;
  wire \counter_reg[20]_i_1_n_4 ;
  wire \counter_reg[20]_i_1_n_5 ;
  wire \counter_reg[20]_i_1_n_6 ;
  wire \counter_reg[20]_i_1_n_7 ;
  wire \counter_reg[24]_i_1_n_0 ;
  wire \counter_reg[24]_i_1_n_1 ;
  wire \counter_reg[24]_i_1_n_2 ;
  wire \counter_reg[24]_i_1_n_3 ;
  wire \counter_reg[24]_i_1_n_4 ;
  wire \counter_reg[24]_i_1_n_5 ;
  wire \counter_reg[24]_i_1_n_6 ;
  wire \counter_reg[24]_i_1_n_7 ;
  wire \counter_reg[28]_i_1_n_1 ;
  wire \counter_reg[28]_i_1_n_2 ;
  wire \counter_reg[28]_i_1_n_3 ;
  wire \counter_reg[28]_i_1_n_4 ;
  wire \counter_reg[28]_i_1_n_5 ;
  wire \counter_reg[28]_i_1_n_6 ;
  wire \counter_reg[28]_i_1_n_7 ;
  wire \counter_reg[4]_i_1_n_0 ;
  wire \counter_reg[4]_i_1_n_1 ;
  wire \counter_reg[4]_i_1_n_2 ;
  wire \counter_reg[4]_i_1_n_3 ;
  wire \counter_reg[4]_i_1_n_4 ;
  wire \counter_reg[4]_i_1_n_5 ;
  wire \counter_reg[4]_i_1_n_6 ;
  wire \counter_reg[4]_i_1_n_7 ;
  wire \counter_reg[8]_i_1_n_0 ;
  wire \counter_reg[8]_i_1_n_1 ;
  wire \counter_reg[8]_i_1_n_2 ;
  wire \counter_reg[8]_i_1_n_3 ;
  wire \counter_reg[8]_i_1_n_4 ;
  wire \counter_reg[8]_i_1_n_5 ;
  wire \counter_reg[8]_i_1_n_6 ;
  wire \counter_reg[8]_i_1_n_7 ;
  wire [15:0]dc_offset;
  wire dc_offset0;
  wire \dc_offset[0]_i_10_n_0 ;
  wire \dc_offset[0]_i_11_n_0 ;
  wire \dc_offset[0]_i_12_n_0 ;
  wire \dc_offset[0]_i_14_n_0 ;
  wire \dc_offset[0]_i_15_n_0 ;
  wire \dc_offset[0]_i_16_n_0 ;
  wire \dc_offset[0]_i_17_n_0 ;
  wire \dc_offset[0]_i_19_n_0 ;
  wire \dc_offset[0]_i_20_n_0 ;
  wire \dc_offset[0]_i_21_n_0 ;
  wire \dc_offset[0]_i_22_n_0 ;
  wire \dc_offset[0]_i_24_n_0 ;
  wire \dc_offset[0]_i_25_n_0 ;
  wire \dc_offset[0]_i_26_n_0 ;
  wire \dc_offset[0]_i_27_n_0 ;
  wire \dc_offset[0]_i_29_n_0 ;
  wire \dc_offset[0]_i_30_n_0 ;
  wire \dc_offset[0]_i_31_n_0 ;
  wire \dc_offset[0]_i_32_n_0 ;
  wire \dc_offset[0]_i_34_n_0 ;
  wire \dc_offset[0]_i_35_n_0 ;
  wire \dc_offset[0]_i_36_n_0 ;
  wire \dc_offset[0]_i_37_n_0 ;
  wire \dc_offset[0]_i_39_n_0 ;
  wire \dc_offset[0]_i_40_n_0 ;
  wire \dc_offset[0]_i_41_n_0 ;
  wire \dc_offset[0]_i_42_n_0 ;
  wire \dc_offset[0]_i_44_n_0 ;
  wire \dc_offset[0]_i_45_n_0 ;
  wire \dc_offset[0]_i_46_n_0 ;
  wire \dc_offset[0]_i_47_n_0 ;
  wire \dc_offset[0]_i_49_n_0 ;
  wire \dc_offset[0]_i_4_n_0 ;
  wire \dc_offset[0]_i_50_n_0 ;
  wire \dc_offset[0]_i_51_n_0 ;
  wire \dc_offset[0]_i_52_n_0 ;
  wire \dc_offset[0]_i_53_n_0 ;
  wire \dc_offset[0]_i_54_n_0 ;
  wire \dc_offset[0]_i_55_n_0 ;
  wire \dc_offset[0]_i_5_n_0 ;
  wire \dc_offset[0]_i_6_n_0 ;
  wire \dc_offset[0]_i_7_n_0 ;
  wire \dc_offset[0]_i_9_n_0 ;
  wire \dc_offset[12]_i_10_n_0 ;
  wire \dc_offset[12]_i_11_n_0 ;
  wire \dc_offset[12]_i_3_n_0 ;
  wire \dc_offset[12]_i_4_n_0 ;
  wire \dc_offset[12]_i_5_n_0 ;
  wire \dc_offset[12]_i_6_n_0 ;
  wire \dc_offset[12]_i_8_n_0 ;
  wire \dc_offset[12]_i_9_n_0 ;
  wire \dc_offset[15]_i_10_n_0 ;
  wire \dc_offset[15]_i_11_n_0 ;
  wire \dc_offset[15]_i_12_n_0 ;
  wire \dc_offset[15]_i_13_n_0 ;
  wire \dc_offset[15]_i_14_n_0 ;
  wire \dc_offset[15]_i_4_n_0 ;
  wire \dc_offset[15]_i_5_n_0 ;
  wire \dc_offset[15]_i_6_n_0 ;
  wire \dc_offset[15]_i_9_n_0 ;
  wire \dc_offset[4]_i_3_n_0 ;
  wire \dc_offset[4]_i_4_n_0 ;
  wire \dc_offset[4]_i_5_n_0 ;
  wire \dc_offset[4]_i_6_n_0 ;
  wire \dc_offset[4]_i_7_n_0 ;
  wire \dc_offset[8]_i_10_n_0 ;
  wire \dc_offset[8]_i_11_n_0 ;
  wire \dc_offset[8]_i_3_n_0 ;
  wire \dc_offset[8]_i_4_n_0 ;
  wire \dc_offset[8]_i_5_n_0 ;
  wire \dc_offset[8]_i_6_n_0 ;
  wire \dc_offset[8]_i_8_n_0 ;
  wire \dc_offset[8]_i_9_n_0 ;
  wire \dc_offset_reg[0]_i_13_n_0 ;
  wire \dc_offset_reg[0]_i_13_n_1 ;
  wire \dc_offset_reg[0]_i_13_n_2 ;
  wire \dc_offset_reg[0]_i_13_n_3 ;
  wire \dc_offset_reg[0]_i_18_n_0 ;
  wire \dc_offset_reg[0]_i_18_n_1 ;
  wire \dc_offset_reg[0]_i_18_n_2 ;
  wire \dc_offset_reg[0]_i_18_n_3 ;
  wire \dc_offset_reg[0]_i_23_n_0 ;
  wire \dc_offset_reg[0]_i_23_n_1 ;
  wire \dc_offset_reg[0]_i_23_n_2 ;
  wire \dc_offset_reg[0]_i_23_n_3 ;
  wire \dc_offset_reg[0]_i_28_n_0 ;
  wire \dc_offset_reg[0]_i_28_n_1 ;
  wire \dc_offset_reg[0]_i_28_n_2 ;
  wire \dc_offset_reg[0]_i_28_n_3 ;
  wire \dc_offset_reg[0]_i_2_n_0 ;
  wire \dc_offset_reg[0]_i_2_n_1 ;
  wire \dc_offset_reg[0]_i_2_n_2 ;
  wire \dc_offset_reg[0]_i_2_n_3 ;
  wire \dc_offset_reg[0]_i_33_n_0 ;
  wire \dc_offset_reg[0]_i_33_n_1 ;
  wire \dc_offset_reg[0]_i_33_n_2 ;
  wire \dc_offset_reg[0]_i_33_n_3 ;
  wire \dc_offset_reg[0]_i_38_n_0 ;
  wire \dc_offset_reg[0]_i_38_n_1 ;
  wire \dc_offset_reg[0]_i_38_n_2 ;
  wire \dc_offset_reg[0]_i_38_n_3 ;
  wire \dc_offset_reg[0]_i_3_n_0 ;
  wire \dc_offset_reg[0]_i_3_n_1 ;
  wire \dc_offset_reg[0]_i_3_n_2 ;
  wire \dc_offset_reg[0]_i_3_n_3 ;
  wire \dc_offset_reg[0]_i_43_n_0 ;
  wire \dc_offset_reg[0]_i_43_n_1 ;
  wire \dc_offset_reg[0]_i_43_n_2 ;
  wire \dc_offset_reg[0]_i_43_n_3 ;
  wire \dc_offset_reg[0]_i_48_n_0 ;
  wire \dc_offset_reg[0]_i_48_n_1 ;
  wire \dc_offset_reg[0]_i_48_n_2 ;
  wire \dc_offset_reg[0]_i_48_n_3 ;
  wire \dc_offset_reg[0]_i_8_n_0 ;
  wire \dc_offset_reg[0]_i_8_n_1 ;
  wire \dc_offset_reg[0]_i_8_n_2 ;
  wire \dc_offset_reg[0]_i_8_n_3 ;
  wire \dc_offset_reg[12]_i_2_n_0 ;
  wire \dc_offset_reg[12]_i_2_n_1 ;
  wire \dc_offset_reg[12]_i_2_n_2 ;
  wire \dc_offset_reg[12]_i_2_n_3 ;
  wire \dc_offset_reg[12]_i_7_n_0 ;
  wire \dc_offset_reg[12]_i_7_n_1 ;
  wire \dc_offset_reg[12]_i_7_n_2 ;
  wire \dc_offset_reg[12]_i_7_n_3 ;
  wire \dc_offset_reg[15]_i_3_n_2 ;
  wire \dc_offset_reg[15]_i_3_n_3 ;
  wire \dc_offset_reg[15]_i_7_n_3 ;
  wire \dc_offset_reg[15]_i_8_n_0 ;
  wire \dc_offset_reg[15]_i_8_n_1 ;
  wire \dc_offset_reg[15]_i_8_n_2 ;
  wire \dc_offset_reg[15]_i_8_n_3 ;
  wire \dc_offset_reg[4]_i_2_n_0 ;
  wire \dc_offset_reg[4]_i_2_n_1 ;
  wire \dc_offset_reg[4]_i_2_n_2 ;
  wire \dc_offset_reg[4]_i_2_n_3 ;
  wire \dc_offset_reg[8]_i_2_n_0 ;
  wire \dc_offset_reg[8]_i_2_n_1 ;
  wire \dc_offset_reg[8]_i_2_n_2 ;
  wire \dc_offset_reg[8]_i_2_n_3 ;
  wire \dc_offset_reg[8]_i_7_n_0 ;
  wire \dc_offset_reg[8]_i_7_n_1 ;
  wire \dc_offset_reg[8]_i_7_n_2 ;
  wire \dc_offset_reg[8]_i_7_n_3 ;
  wire icmp_ln35_fu_173_p2;
  wire icmp_ln35_reg_289;
  wire icmp_ln35_reg_289_pp0_iter1_reg;
  wire icmp_ln35_reg_289_pp0_iter2_reg;
  wire [15:0]in;
  wire [31:0]in_stream_TDATA;
  wire [3:0]in_stream_TKEEP;
  wire [3:0]in_stream_TKEEP_int_regslice;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TLAST_int_regslice;
  wire in_stream_TREADY;
  wire [3:0]in_stream_TSTRB;
  wire [3:0]in_stream_TSTRB_int_regslice;
  wire in_stream_TVALID;
  wire in_stream_TVALID_int_regslice;
  wire mul_32s_34ns_65_2_1_U1_n_0;
  wire mul_32s_34ns_65_2_1_U1_n_1;
  wire mul_32s_34ns_65_2_1_U1_n_10;
  wire mul_32s_34ns_65_2_1_U1_n_11;
  wire mul_32s_34ns_65_2_1_U1_n_12;
  wire mul_32s_34ns_65_2_1_U1_n_13;
  wire mul_32s_34ns_65_2_1_U1_n_14;
  wire mul_32s_34ns_65_2_1_U1_n_2;
  wire mul_32s_34ns_65_2_1_U1_n_3;
  wire mul_32s_34ns_65_2_1_U1_n_4;
  wire mul_32s_34ns_65_2_1_U1_n_41;
  wire mul_32s_34ns_65_2_1_U1_n_42;
  wire mul_32s_34ns_65_2_1_U1_n_43;
  wire mul_32s_34ns_65_2_1_U1_n_44;
  wire mul_32s_34ns_65_2_1_U1_n_45;
  wire mul_32s_34ns_65_2_1_U1_n_46;
  wire mul_32s_34ns_65_2_1_U1_n_47;
  wire mul_32s_34ns_65_2_1_U1_n_48;
  wire mul_32s_34ns_65_2_1_U1_n_49;
  wire mul_32s_34ns_65_2_1_U1_n_5;
  wire mul_32s_34ns_65_2_1_U1_n_50;
  wire mul_32s_34ns_65_2_1_U1_n_51;
  wire mul_32s_34ns_65_2_1_U1_n_52;
  wire mul_32s_34ns_65_2_1_U1_n_53;
  wire mul_32s_34ns_65_2_1_U1_n_54;
  wire mul_32s_34ns_65_2_1_U1_n_55;
  wire mul_32s_34ns_65_2_1_U1_n_56;
  wire mul_32s_34ns_65_2_1_U1_n_6;
  wire mul_32s_34ns_65_2_1_U1_n_7;
  wire mul_32s_34ns_65_2_1_U1_n_8;
  wire mul_32s_34ns_65_2_1_U1_n_9;
  wire [55:0]mul_ln39_reg_304;
  wire mul_ln39_reg_3040;
  wire [31:0]\^out_stream_TDATA ;
  wire [3:0]out_stream_TKEEP;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire [3:0]out_stream_TSTRB;
  wire out_stream_TVALID;
  wire out_stream_TVALID_int_regslice;
  wire p_0_in;
  wire \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_n_0 ;
  wire \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_n_0 ;
  wire \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_n_0 ;
  wire \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_n_0 ;
  wire [3:0]pkt_in_keep_V_reg_259_pp0_iter2_reg;
  wire \pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2_n_0 ;
  wire pkt_in_last_V_reg_269_pp0_iter2_reg;
  wire \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_n_0 ;
  wire \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_n_0 ;
  wire \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_n_0 ;
  wire \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_n_0 ;
  wire [3:0]pkt_in_strb_V_reg_264_pp0_iter2_reg;
  wire regslice_both_in_stream_V_data_V_U_n_19;
  wire regslice_both_in_stream_V_data_V_U_n_20;
  wire regslice_both_in_stream_V_data_V_U_n_21;
  wire regslice_both_in_stream_V_data_V_U_n_22;
  wire regslice_both_in_stream_V_data_V_U_n_23;
  wire regslice_both_in_stream_V_data_V_U_n_24;
  wire regslice_both_in_stream_V_data_V_U_n_25;
  wire regslice_both_in_stream_V_data_V_U_n_26;
  wire regslice_both_in_stream_V_data_V_U_n_27;
  wire regslice_both_in_stream_V_data_V_U_n_28;
  wire regslice_both_in_stream_V_data_V_U_n_29;
  wire regslice_both_in_stream_V_data_V_U_n_30;
  wire regslice_both_in_stream_V_data_V_U_n_31;
  wire regslice_both_in_stream_V_data_V_U_n_32;
  wire regslice_both_in_stream_V_data_V_U_n_33;
  wire regslice_both_in_stream_V_data_V_U_n_34;
  wire regslice_both_in_stream_V_data_V_U_n_35;
  wire regslice_both_in_stream_V_data_V_U_n_36;
  wire regslice_both_in_stream_V_data_V_U_n_37;
  wire regslice_both_in_stream_V_data_V_U_n_38;
  wire regslice_both_in_stream_V_data_V_U_n_39;
  wire regslice_both_in_stream_V_data_V_U_n_40;
  wire regslice_both_in_stream_V_data_V_U_n_41;
  wire regslice_both_in_stream_V_data_V_U_n_42;
  wire regslice_both_in_stream_V_data_V_U_n_43;
  wire regslice_both_in_stream_V_data_V_U_n_44;
  wire regslice_both_in_stream_V_data_V_U_n_45;
  wire regslice_both_in_stream_V_data_V_U_n_46;
  wire regslice_both_in_stream_V_data_V_U_n_47;
  wire regslice_both_in_stream_V_data_V_U_n_48;
  wire regslice_both_in_stream_V_data_V_U_n_49;
  wire regslice_both_in_stream_V_data_V_U_n_50;
  wire regslice_both_out_stream_V_data_V_U_n_12;
  wire regslice_both_out_stream_V_data_V_U_n_3;
  wire regslice_both_out_stream_V_data_V_U_n_5;
  wire regslice_both_out_stream_V_data_V_U_n_6;
  wire regslice_both_out_stream_V_data_V_U_n_7;
  wire regslice_both_out_stream_V_data_V_U_n_8;
  wire regslice_both_out_stream_V_data_V_U_n_9;
  wire [15:0]select_ln39_1_fu_233_p3;
  wire [15:1]sub_ln39_1_fu_227_p2;
  wire [57:42]sub_ln39_fu_206_p2;
  wire [31:0]sum_reg;
  wire [15:14]tmp_2_reg_309;
  wire tmp_reg_298;
  wire tmp_reg_298_pp0_iter2_reg;
  wire \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_n_0 ;
  wire \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_n_0 ;
  wire [15:0]val_in_reg_274_pp0_iter2_reg;
  wire [3:3]\NLW_counter_reg[28]_i_1_CO_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_13_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_18_O_UNCONNECTED ;
  wire [1:0]\NLW_dc_offset_reg[0]_i_2_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_23_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_28_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_3_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_33_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_38_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_43_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_48_O_UNCONNECTED ;
  wire [3:0]\NLW_dc_offset_reg[0]_i_8_O_UNCONNECTED ;
  wire [3:2]\NLW_dc_offset_reg[15]_i_3_CO_UNCONNECTED ;
  wire [3:3]\NLW_dc_offset_reg[15]_i_3_O_UNCONNECTED ;
  wire [3:1]\NLW_dc_offset_reg[15]_i_7_CO_UNCONNECTED ;
  wire [3:2]\NLW_dc_offset_reg[15]_i_7_O_UNCONNECTED ;

  assign out_stream_TDATA[31] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[30] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[29] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[28] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[27] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[26] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[25] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[24] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[23] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[22] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[21] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[20] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[19] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[18] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[17] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[16] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[15] = \^out_stream_TDATA [31];
  assign out_stream_TDATA[14:0] = \^out_stream_TDATA [14:0];
  FDRE \add_ln32_reg_283_reg[31] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(add_ln32_fu_145_p2[31]),
        .Q(p_0_in),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter1_reg
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(1'b1),
        .Q(ap_enable_reg_pp0_iter1),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter2_reg
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(ap_enable_reg_pp0_iter1),
        .Q(ap_enable_reg_pp0_iter2),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter3_reg
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(ap_enable_reg_pp0_iter2),
        .Q(ap_enable_reg_pp0_iter3),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter4_reg
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(ap_enable_reg_pp0_iter3),
        .Q(ap_enable_reg_pp0_iter4),
        .R(ap_rst_n_inv));
  FDRE \calibrated_load_reg_279_pp0_iter1_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(calibrated_load_reg_279),
        .Q(calibrated_load_reg_279_pp0_iter1_reg),
        .R(1'b0));
  FDRE \calibrated_load_reg_279_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(calibrated_load_reg_279_pp0_iter1_reg),
        .Q(calibrated_load_reg_279_pp0_iter2_reg),
        .R(1'b0));
  FDRE \calibrated_load_reg_279_pp0_iter3_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(calibrated_load_reg_279_pp0_iter2_reg),
        .Q(calibrated_load_reg_279_pp0_iter3_reg),
        .R(1'b0));
  FDRE \calibrated_load_reg_279_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(calibrated),
        .Q(calibrated_load_reg_279),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \calibrated_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_out_stream_V_data_V_U_n_5),
        .Q(calibrated),
        .R(1'b0));
  LUT1 #(
    .INIT(2'h1)) 
    \counter[0]_i_2 
       (.I0(counter_reg[0]),
        .O(add_ln33_fu_161_p2));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[0] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[0]_i_1_n_7 ),
        .Q(counter_reg[0]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[0]_i_1 
       (.CI(1'b0),
        .CO({\counter_reg[0]_i_1_n_0 ,\counter_reg[0]_i_1_n_1 ,\counter_reg[0]_i_1_n_2 ,\counter_reg[0]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O({\counter_reg[0]_i_1_n_4 ,\counter_reg[0]_i_1_n_5 ,\counter_reg[0]_i_1_n_6 ,\counter_reg[0]_i_1_n_7 }),
        .S({counter_reg[3:1],add_ln33_fu_161_p2}));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[10] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[8]_i_1_n_5 ),
        .Q(counter_reg[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[11] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[8]_i_1_n_4 ),
        .Q(counter_reg[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[12] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[12]_i_1_n_7 ),
        .Q(counter_reg[12]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[12]_i_1 
       (.CI(\counter_reg[8]_i_1_n_0 ),
        .CO({\counter_reg[12]_i_1_n_0 ,\counter_reg[12]_i_1_n_1 ,\counter_reg[12]_i_1_n_2 ,\counter_reg[12]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\counter_reg[12]_i_1_n_4 ,\counter_reg[12]_i_1_n_5 ,\counter_reg[12]_i_1_n_6 ,\counter_reg[12]_i_1_n_7 }),
        .S(counter_reg[15:12]));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[13] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[12]_i_1_n_6 ),
        .Q(counter_reg[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[14] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[12]_i_1_n_5 ),
        .Q(counter_reg[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[15] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[12]_i_1_n_4 ),
        .Q(counter_reg[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[16] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[16]_i_1_n_7 ),
        .Q(counter_reg[16]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[16]_i_1 
       (.CI(\counter_reg[12]_i_1_n_0 ),
        .CO({\counter_reg[16]_i_1_n_0 ,\counter_reg[16]_i_1_n_1 ,\counter_reg[16]_i_1_n_2 ,\counter_reg[16]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\counter_reg[16]_i_1_n_4 ,\counter_reg[16]_i_1_n_5 ,\counter_reg[16]_i_1_n_6 ,\counter_reg[16]_i_1_n_7 }),
        .S(counter_reg[19:16]));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[17] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[16]_i_1_n_6 ),
        .Q(counter_reg[17]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[18] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[16]_i_1_n_5 ),
        .Q(counter_reg[18]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[19] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[16]_i_1_n_4 ),
        .Q(counter_reg[19]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[1] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[0]_i_1_n_6 ),
        .Q(counter_reg[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[20] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[20]_i_1_n_7 ),
        .Q(counter_reg[20]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[20]_i_1 
       (.CI(\counter_reg[16]_i_1_n_0 ),
        .CO({\counter_reg[20]_i_1_n_0 ,\counter_reg[20]_i_1_n_1 ,\counter_reg[20]_i_1_n_2 ,\counter_reg[20]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\counter_reg[20]_i_1_n_4 ,\counter_reg[20]_i_1_n_5 ,\counter_reg[20]_i_1_n_6 ,\counter_reg[20]_i_1_n_7 }),
        .S(counter_reg[23:20]));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[21] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[20]_i_1_n_6 ),
        .Q(counter_reg[21]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[22] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[20]_i_1_n_5 ),
        .Q(counter_reg[22]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[23] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[20]_i_1_n_4 ),
        .Q(counter_reg[23]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[24] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[24]_i_1_n_7 ),
        .Q(counter_reg[24]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[24]_i_1 
       (.CI(\counter_reg[20]_i_1_n_0 ),
        .CO({\counter_reg[24]_i_1_n_0 ,\counter_reg[24]_i_1_n_1 ,\counter_reg[24]_i_1_n_2 ,\counter_reg[24]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\counter_reg[24]_i_1_n_4 ,\counter_reg[24]_i_1_n_5 ,\counter_reg[24]_i_1_n_6 ,\counter_reg[24]_i_1_n_7 }),
        .S(counter_reg[27:24]));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[25] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[24]_i_1_n_6 ),
        .Q(counter_reg[25]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[26] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[24]_i_1_n_5 ),
        .Q(counter_reg[26]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[27] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[24]_i_1_n_4 ),
        .Q(counter_reg[27]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[28] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[28]_i_1_n_7 ),
        .Q(counter_reg[28]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[28]_i_1 
       (.CI(\counter_reg[24]_i_1_n_0 ),
        .CO({\NLW_counter_reg[28]_i_1_CO_UNCONNECTED [3],\counter_reg[28]_i_1_n_1 ,\counter_reg[28]_i_1_n_2 ,\counter_reg[28]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\counter_reg[28]_i_1_n_4 ,\counter_reg[28]_i_1_n_5 ,\counter_reg[28]_i_1_n_6 ,\counter_reg[28]_i_1_n_7 }),
        .S(counter_reg[31:28]));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[29] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[28]_i_1_n_6 ),
        .Q(counter_reg[29]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[2] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[0]_i_1_n_5 ),
        .Q(counter_reg[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[30] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[28]_i_1_n_5 ),
        .Q(counter_reg[30]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[31] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[28]_i_1_n_4 ),
        .Q(counter_reg[31]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[3] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[0]_i_1_n_4 ),
        .Q(counter_reg[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[4] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[4]_i_1_n_7 ),
        .Q(counter_reg[4]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[4]_i_1 
       (.CI(\counter_reg[0]_i_1_n_0 ),
        .CO({\counter_reg[4]_i_1_n_0 ,\counter_reg[4]_i_1_n_1 ,\counter_reg[4]_i_1_n_2 ,\counter_reg[4]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\counter_reg[4]_i_1_n_4 ,\counter_reg[4]_i_1_n_5 ,\counter_reg[4]_i_1_n_6 ,\counter_reg[4]_i_1_n_7 }),
        .S(counter_reg[7:4]));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[5] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[4]_i_1_n_6 ),
        .Q(counter_reg[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[6] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[4]_i_1_n_5 ),
        .Q(counter_reg[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[7] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[4]_i_1_n_4 ),
        .Q(counter_reg[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[8] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[8]_i_1_n_7 ),
        .Q(counter_reg[8]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[8]_i_1 
       (.CI(\counter_reg[4]_i_1_n_0 ),
        .CO({\counter_reg[8]_i_1_n_0 ,\counter_reg[8]_i_1_n_1 ,\counter_reg[8]_i_1_n_2 ,\counter_reg[8]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\counter_reg[8]_i_1_n_4 ,\counter_reg[8]_i_1_n_5 ,\counter_reg[8]_i_1_n_6 ,\counter_reg[8]_i_1_n_7 }),
        .S(counter_reg[11:8]));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[9] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(\counter_reg[8]_i_1_n_6 ),
        .Q(counter_reg[9]),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair40" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[0]_i_1 
       (.I0(sub_ln39_fu_206_p2[42]),
        .I1(mul_ln39_reg_304[42]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[0]));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_10 
       (.I0(mul_ln39_reg_304[38]),
        .O(\dc_offset[0]_i_10_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_11 
       (.I0(mul_ln39_reg_304[37]),
        .O(\dc_offset[0]_i_11_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_12 
       (.I0(mul_ln39_reg_304[36]),
        .O(\dc_offset[0]_i_12_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_14 
       (.I0(mul_ln39_reg_304[35]),
        .O(\dc_offset[0]_i_14_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_15 
       (.I0(mul_ln39_reg_304[34]),
        .O(\dc_offset[0]_i_15_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_16 
       (.I0(mul_ln39_reg_304[33]),
        .O(\dc_offset[0]_i_16_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_17 
       (.I0(mul_ln39_reg_304[32]),
        .O(\dc_offset[0]_i_17_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_19 
       (.I0(mul_ln39_reg_304[31]),
        .O(\dc_offset[0]_i_19_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_20 
       (.I0(mul_ln39_reg_304[30]),
        .O(\dc_offset[0]_i_20_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_21 
       (.I0(mul_ln39_reg_304[29]),
        .O(\dc_offset[0]_i_21_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_22 
       (.I0(mul_ln39_reg_304[28]),
        .O(\dc_offset[0]_i_22_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_24 
       (.I0(mul_ln39_reg_304[27]),
        .O(\dc_offset[0]_i_24_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_25 
       (.I0(mul_ln39_reg_304[26]),
        .O(\dc_offset[0]_i_25_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_26 
       (.I0(mul_ln39_reg_304[25]),
        .O(\dc_offset[0]_i_26_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_27 
       (.I0(mul_ln39_reg_304[24]),
        .O(\dc_offset[0]_i_27_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_29 
       (.I0(mul_ln39_reg_304[23]),
        .O(\dc_offset[0]_i_29_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_30 
       (.I0(mul_ln39_reg_304[22]),
        .O(\dc_offset[0]_i_30_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_31 
       (.I0(mul_ln39_reg_304[21]),
        .O(\dc_offset[0]_i_31_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_32 
       (.I0(mul_ln39_reg_304[20]),
        .O(\dc_offset[0]_i_32_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_34 
       (.I0(mul_ln39_reg_304[19]),
        .O(\dc_offset[0]_i_34_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_35 
       (.I0(mul_ln39_reg_304[18]),
        .O(\dc_offset[0]_i_35_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_36 
       (.I0(mul_ln39_reg_304[17]),
        .O(\dc_offset[0]_i_36_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_37 
       (.I0(mul_ln39_reg_304[16]),
        .O(\dc_offset[0]_i_37_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_39 
       (.I0(mul_ln39_reg_304[15]),
        .O(\dc_offset[0]_i_39_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_4 
       (.I0(mul_ln39_reg_304[43]),
        .O(\dc_offset[0]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_40 
       (.I0(mul_ln39_reg_304[14]),
        .O(\dc_offset[0]_i_40_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_41 
       (.I0(mul_ln39_reg_304[13]),
        .O(\dc_offset[0]_i_41_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_42 
       (.I0(mul_ln39_reg_304[12]),
        .O(\dc_offset[0]_i_42_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_44 
       (.I0(mul_ln39_reg_304[11]),
        .O(\dc_offset[0]_i_44_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_45 
       (.I0(mul_ln39_reg_304[10]),
        .O(\dc_offset[0]_i_45_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_46 
       (.I0(mul_ln39_reg_304[9]),
        .O(\dc_offset[0]_i_46_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_47 
       (.I0(mul_ln39_reg_304[8]),
        .O(\dc_offset[0]_i_47_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_49 
       (.I0(mul_ln39_reg_304[7]),
        .O(\dc_offset[0]_i_49_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_5 
       (.I0(mul_ln39_reg_304[42]),
        .O(\dc_offset[0]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_50 
       (.I0(mul_ln39_reg_304[6]),
        .O(\dc_offset[0]_i_50_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_51 
       (.I0(mul_ln39_reg_304[5]),
        .O(\dc_offset[0]_i_51_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_52 
       (.I0(mul_ln39_reg_304[4]),
        .O(\dc_offset[0]_i_52_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_53 
       (.I0(mul_ln39_reg_304[3]),
        .O(\dc_offset[0]_i_53_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_54 
       (.I0(mul_ln39_reg_304[2]),
        .O(\dc_offset[0]_i_54_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_55 
       (.I0(mul_ln39_reg_304[1]),
        .O(\dc_offset[0]_i_55_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_6 
       (.I0(mul_ln39_reg_304[41]),
        .O(\dc_offset[0]_i_6_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_7 
       (.I0(mul_ln39_reg_304[40]),
        .O(\dc_offset[0]_i_7_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[0]_i_9 
       (.I0(mul_ln39_reg_304[39]),
        .O(\dc_offset[0]_i_9_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair34" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[10]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[10]),
        .I1(mul_ln39_reg_304[52]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[10]));
  (* SOFT_HLUTNM = "soft_lutpair35" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[11]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[11]),
        .I1(mul_ln39_reg_304[53]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[11]));
  (* SOFT_HLUTNM = "soft_lutpair35" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[12]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[12]),
        .I1(mul_ln39_reg_304[54]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[12]));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[12]_i_10 
       (.I0(mul_ln39_reg_304[49]),
        .O(\dc_offset[12]_i_10_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[12]_i_11 
       (.I0(mul_ln39_reg_304[48]),
        .O(\dc_offset[12]_i_11_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[12]_i_3 
       (.I0(sub_ln39_fu_206_p2[54]),
        .O(\dc_offset[12]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[12]_i_4 
       (.I0(sub_ln39_fu_206_p2[53]),
        .O(\dc_offset[12]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[12]_i_5 
       (.I0(sub_ln39_fu_206_p2[52]),
        .O(\dc_offset[12]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[12]_i_6 
       (.I0(sub_ln39_fu_206_p2[51]),
        .O(\dc_offset[12]_i_6_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[12]_i_8 
       (.I0(mul_ln39_reg_304[51]),
        .O(\dc_offset[12]_i_8_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[12]_i_9 
       (.I0(mul_ln39_reg_304[50]),
        .O(\dc_offset[12]_i_9_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair34" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[13]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[13]),
        .I1(mul_ln39_reg_304[55]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[13]));
  (* SOFT_HLUTNM = "soft_lutpair33" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[14]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[14]),
        .I1(tmp_2_reg_309[14]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[14]));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_10 
       (.I0(tmp_2_reg_309[14]),
        .O(\dc_offset[15]_i_10_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_11 
       (.I0(mul_ln39_reg_304[55]),
        .O(\dc_offset[15]_i_11_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_12 
       (.I0(mul_ln39_reg_304[54]),
        .O(\dc_offset[15]_i_12_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_13 
       (.I0(mul_ln39_reg_304[53]),
        .O(\dc_offset[15]_i_13_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_14 
       (.I0(mul_ln39_reg_304[52]),
        .O(\dc_offset[15]_i_14_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair33" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[15]_i_2 
       (.I0(sub_ln39_1_fu_227_p2[15]),
        .I1(tmp_2_reg_309[15]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[15]));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_4 
       (.I0(sub_ln39_fu_206_p2[57]),
        .O(\dc_offset[15]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_5 
       (.I0(sub_ln39_fu_206_p2[56]),
        .O(\dc_offset[15]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_6 
       (.I0(sub_ln39_fu_206_p2[55]),
        .O(\dc_offset[15]_i_6_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[15]_i_9 
       (.I0(tmp_2_reg_309[15]),
        .O(\dc_offset[15]_i_9_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair40" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[1]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[1]),
        .I1(mul_ln39_reg_304[43]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[1]));
  (* SOFT_HLUTNM = "soft_lutpair39" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[2]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[2]),
        .I1(mul_ln39_reg_304[44]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[2]));
  (* SOFT_HLUTNM = "soft_lutpair39" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[3]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[3]),
        .I1(mul_ln39_reg_304[45]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[3]));
  (* SOFT_HLUTNM = "soft_lutpair38" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[4]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[4]),
        .I1(mul_ln39_reg_304[46]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[4]));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[4]_i_3 
       (.I0(sub_ln39_fu_206_p2[42]),
        .O(\dc_offset[4]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[4]_i_4 
       (.I0(sub_ln39_fu_206_p2[46]),
        .O(\dc_offset[4]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[4]_i_5 
       (.I0(sub_ln39_fu_206_p2[45]),
        .O(\dc_offset[4]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[4]_i_6 
       (.I0(sub_ln39_fu_206_p2[44]),
        .O(\dc_offset[4]_i_6_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[4]_i_7 
       (.I0(sub_ln39_fu_206_p2[43]),
        .O(\dc_offset[4]_i_7_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair38" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[5]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[5]),
        .I1(mul_ln39_reg_304[47]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[5]));
  (* SOFT_HLUTNM = "soft_lutpair36" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[6]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[6]),
        .I1(mul_ln39_reg_304[48]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[6]));
  (* SOFT_HLUTNM = "soft_lutpair37" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[7]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[7]),
        .I1(mul_ln39_reg_304[49]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[7]));
  (* SOFT_HLUTNM = "soft_lutpair37" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[8]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[8]),
        .I1(mul_ln39_reg_304[50]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[8]));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[8]_i_10 
       (.I0(mul_ln39_reg_304[45]),
        .O(\dc_offset[8]_i_10_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[8]_i_11 
       (.I0(mul_ln39_reg_304[44]),
        .O(\dc_offset[8]_i_11_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[8]_i_3 
       (.I0(sub_ln39_fu_206_p2[50]),
        .O(\dc_offset[8]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[8]_i_4 
       (.I0(sub_ln39_fu_206_p2[49]),
        .O(\dc_offset[8]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[8]_i_5 
       (.I0(sub_ln39_fu_206_p2[48]),
        .O(\dc_offset[8]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[8]_i_6 
       (.I0(sub_ln39_fu_206_p2[47]),
        .O(\dc_offset[8]_i_6_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[8]_i_8 
       (.I0(mul_ln39_reg_304[47]),
        .O(\dc_offset[8]_i_8_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dc_offset[8]_i_9 
       (.I0(mul_ln39_reg_304[46]),
        .O(\dc_offset[8]_i_9_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair36" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dc_offset[9]_i_1 
       (.I0(sub_ln39_1_fu_227_p2[9]),
        .I1(mul_ln39_reg_304[51]),
        .I2(tmp_reg_298_pp0_iter2_reg),
        .O(select_ln39_1_fu_233_p3[9]));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[0] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[0]),
        .Q(dc_offset[0]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_13 
       (.CI(\dc_offset_reg[0]_i_18_n_0 ),
        .CO({\dc_offset_reg[0]_i_13_n_0 ,\dc_offset_reg[0]_i_13_n_1 ,\dc_offset_reg[0]_i_13_n_2 ,\dc_offset_reg[0]_i_13_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_13_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_19_n_0 ,\dc_offset[0]_i_20_n_0 ,\dc_offset[0]_i_21_n_0 ,\dc_offset[0]_i_22_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_18 
       (.CI(\dc_offset_reg[0]_i_23_n_0 ),
        .CO({\dc_offset_reg[0]_i_18_n_0 ,\dc_offset_reg[0]_i_18_n_1 ,\dc_offset_reg[0]_i_18_n_2 ,\dc_offset_reg[0]_i_18_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_18_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_24_n_0 ,\dc_offset[0]_i_25_n_0 ,\dc_offset[0]_i_26_n_0 ,\dc_offset[0]_i_27_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_2 
       (.CI(\dc_offset_reg[0]_i_3_n_0 ),
        .CO({\dc_offset_reg[0]_i_2_n_0 ,\dc_offset_reg[0]_i_2_n_1 ,\dc_offset_reg[0]_i_2_n_2 ,\dc_offset_reg[0]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({sub_ln39_fu_206_p2[43:42],\NLW_dc_offset_reg[0]_i_2_O_UNCONNECTED [1:0]}),
        .S({\dc_offset[0]_i_4_n_0 ,\dc_offset[0]_i_5_n_0 ,\dc_offset[0]_i_6_n_0 ,\dc_offset[0]_i_7_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_23 
       (.CI(\dc_offset_reg[0]_i_28_n_0 ),
        .CO({\dc_offset_reg[0]_i_23_n_0 ,\dc_offset_reg[0]_i_23_n_1 ,\dc_offset_reg[0]_i_23_n_2 ,\dc_offset_reg[0]_i_23_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_23_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_29_n_0 ,\dc_offset[0]_i_30_n_0 ,\dc_offset[0]_i_31_n_0 ,\dc_offset[0]_i_32_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_28 
       (.CI(\dc_offset_reg[0]_i_33_n_0 ),
        .CO({\dc_offset_reg[0]_i_28_n_0 ,\dc_offset_reg[0]_i_28_n_1 ,\dc_offset_reg[0]_i_28_n_2 ,\dc_offset_reg[0]_i_28_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_28_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_34_n_0 ,\dc_offset[0]_i_35_n_0 ,\dc_offset[0]_i_36_n_0 ,\dc_offset[0]_i_37_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_3 
       (.CI(\dc_offset_reg[0]_i_8_n_0 ),
        .CO({\dc_offset_reg[0]_i_3_n_0 ,\dc_offset_reg[0]_i_3_n_1 ,\dc_offset_reg[0]_i_3_n_2 ,\dc_offset_reg[0]_i_3_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_3_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_9_n_0 ,\dc_offset[0]_i_10_n_0 ,\dc_offset[0]_i_11_n_0 ,\dc_offset[0]_i_12_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_33 
       (.CI(\dc_offset_reg[0]_i_38_n_0 ),
        .CO({\dc_offset_reg[0]_i_33_n_0 ,\dc_offset_reg[0]_i_33_n_1 ,\dc_offset_reg[0]_i_33_n_2 ,\dc_offset_reg[0]_i_33_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_33_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_39_n_0 ,\dc_offset[0]_i_40_n_0 ,\dc_offset[0]_i_41_n_0 ,\dc_offset[0]_i_42_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_38 
       (.CI(\dc_offset_reg[0]_i_43_n_0 ),
        .CO({\dc_offset_reg[0]_i_38_n_0 ,\dc_offset_reg[0]_i_38_n_1 ,\dc_offset_reg[0]_i_38_n_2 ,\dc_offset_reg[0]_i_38_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_38_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_44_n_0 ,\dc_offset[0]_i_45_n_0 ,\dc_offset[0]_i_46_n_0 ,\dc_offset[0]_i_47_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_43 
       (.CI(\dc_offset_reg[0]_i_48_n_0 ),
        .CO({\dc_offset_reg[0]_i_43_n_0 ,\dc_offset_reg[0]_i_43_n_1 ,\dc_offset_reg[0]_i_43_n_2 ,\dc_offset_reg[0]_i_43_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_43_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_49_n_0 ,\dc_offset[0]_i_50_n_0 ,\dc_offset[0]_i_51_n_0 ,\dc_offset[0]_i_52_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_48 
       (.CI(1'b0),
        .CO({\dc_offset_reg[0]_i_48_n_0 ,\dc_offset_reg[0]_i_48_n_1 ,\dc_offset_reg[0]_i_48_n_2 ,\dc_offset_reg[0]_i_48_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O(\NLW_dc_offset_reg[0]_i_48_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_53_n_0 ,\dc_offset[0]_i_54_n_0 ,\dc_offset[0]_i_55_n_0 ,mul_ln39_reg_304[0]}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[0]_i_8 
       (.CI(\dc_offset_reg[0]_i_13_n_0 ),
        .CO({\dc_offset_reg[0]_i_8_n_0 ,\dc_offset_reg[0]_i_8_n_1 ,\dc_offset_reg[0]_i_8_n_2 ,\dc_offset_reg[0]_i_8_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(\NLW_dc_offset_reg[0]_i_8_O_UNCONNECTED [3:0]),
        .S({\dc_offset[0]_i_14_n_0 ,\dc_offset[0]_i_15_n_0 ,\dc_offset[0]_i_16_n_0 ,\dc_offset[0]_i_17_n_0 }));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[10] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[10]),
        .Q(dc_offset[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[11] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[11]),
        .Q(dc_offset[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[12] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[12]),
        .Q(dc_offset[12]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[12]_i_2 
       (.CI(\dc_offset_reg[8]_i_2_n_0 ),
        .CO({\dc_offset_reg[12]_i_2_n_0 ,\dc_offset_reg[12]_i_2_n_1 ,\dc_offset_reg[12]_i_2_n_2 ,\dc_offset_reg[12]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sub_ln39_1_fu_227_p2[12:9]),
        .S({\dc_offset[12]_i_3_n_0 ,\dc_offset[12]_i_4_n_0 ,\dc_offset[12]_i_5_n_0 ,\dc_offset[12]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[12]_i_7 
       (.CI(\dc_offset_reg[8]_i_7_n_0 ),
        .CO({\dc_offset_reg[12]_i_7_n_0 ,\dc_offset_reg[12]_i_7_n_1 ,\dc_offset_reg[12]_i_7_n_2 ,\dc_offset_reg[12]_i_7_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sub_ln39_fu_206_p2[51:48]),
        .S({\dc_offset[12]_i_8_n_0 ,\dc_offset[12]_i_9_n_0 ,\dc_offset[12]_i_10_n_0 ,\dc_offset[12]_i_11_n_0 }));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[13] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[13]),
        .Q(dc_offset[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[14] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[14]),
        .Q(dc_offset[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[15] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[15]),
        .Q(dc_offset[15]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[15]_i_3 
       (.CI(\dc_offset_reg[12]_i_2_n_0 ),
        .CO({\NLW_dc_offset_reg[15]_i_3_CO_UNCONNECTED [3:2],\dc_offset_reg[15]_i_3_n_2 ,\dc_offset_reg[15]_i_3_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_dc_offset_reg[15]_i_3_O_UNCONNECTED [3],sub_ln39_1_fu_227_p2[15:13]}),
        .S({1'b0,\dc_offset[15]_i_4_n_0 ,\dc_offset[15]_i_5_n_0 ,\dc_offset[15]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[15]_i_7 
       (.CI(\dc_offset_reg[15]_i_8_n_0 ),
        .CO({\NLW_dc_offset_reg[15]_i_7_CO_UNCONNECTED [3:1],\dc_offset_reg[15]_i_7_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_dc_offset_reg[15]_i_7_O_UNCONNECTED [3:2],sub_ln39_fu_206_p2[57:56]}),
        .S({1'b0,1'b0,\dc_offset[15]_i_9_n_0 ,\dc_offset[15]_i_10_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[15]_i_8 
       (.CI(\dc_offset_reg[12]_i_7_n_0 ),
        .CO({\dc_offset_reg[15]_i_8_n_0 ,\dc_offset_reg[15]_i_8_n_1 ,\dc_offset_reg[15]_i_8_n_2 ,\dc_offset_reg[15]_i_8_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sub_ln39_fu_206_p2[55:52]),
        .S({\dc_offset[15]_i_11_n_0 ,\dc_offset[15]_i_12_n_0 ,\dc_offset[15]_i_13_n_0 ,\dc_offset[15]_i_14_n_0 }));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[1] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[1]),
        .Q(dc_offset[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[2] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[2]),
        .Q(dc_offset[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[3] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[3]),
        .Q(dc_offset[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[4] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[4]),
        .Q(dc_offset[4]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[4]_i_2 
       (.CI(1'b0),
        .CO({\dc_offset_reg[4]_i_2_n_0 ,\dc_offset_reg[4]_i_2_n_1 ,\dc_offset_reg[4]_i_2_n_2 ,\dc_offset_reg[4]_i_2_n_3 }),
        .CYINIT(\dc_offset[4]_i_3_n_0 ),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sub_ln39_1_fu_227_p2[4:1]),
        .S({\dc_offset[4]_i_4_n_0 ,\dc_offset[4]_i_5_n_0 ,\dc_offset[4]_i_6_n_0 ,\dc_offset[4]_i_7_n_0 }));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[5] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[5]),
        .Q(dc_offset[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[6] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[6]),
        .Q(dc_offset[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[7] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[7]),
        .Q(dc_offset[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[8] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[8]),
        .Q(dc_offset[8]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[8]_i_2 
       (.CI(\dc_offset_reg[4]_i_2_n_0 ),
        .CO({\dc_offset_reg[8]_i_2_n_0 ,\dc_offset_reg[8]_i_2_n_1 ,\dc_offset_reg[8]_i_2_n_2 ,\dc_offset_reg[8]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sub_ln39_1_fu_227_p2[8:5]),
        .S({\dc_offset[8]_i_3_n_0 ,\dc_offset[8]_i_4_n_0 ,\dc_offset[8]_i_5_n_0 ,\dc_offset[8]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \dc_offset_reg[8]_i_7 
       (.CI(\dc_offset_reg[0]_i_2_n_0 ),
        .CO({\dc_offset_reg[8]_i_7_n_0 ,\dc_offset_reg[8]_i_7_n_1 ,\dc_offset_reg[8]_i_7_n_2 ,\dc_offset_reg[8]_i_7_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sub_ln39_fu_206_p2[47:44]),
        .S({\dc_offset[8]_i_8_n_0 ,\dc_offset[8]_i_9_n_0 ,\dc_offset[8]_i_10_n_0 ,\dc_offset[8]_i_11_n_0 }));
  FDRE #(
    .INIT(1'b0)) 
    \dc_offset_reg[9] 
       (.C(ap_clk),
        .CE(dc_offset0),
        .D(select_ln39_1_fu_233_p3[9]),
        .Q(dc_offset[9]),
        .R(1'b0));
  LUT4 #(
    .INIT(16'h8000)) 
    \icmp_ln35_reg_289[0]_i_1 
       (.I0(regslice_both_out_stream_V_data_V_U_n_6),
        .I1(regslice_both_out_stream_V_data_V_U_n_7),
        .I2(regslice_both_out_stream_V_data_V_U_n_8),
        .I3(regslice_both_out_stream_V_data_V_U_n_9),
        .O(icmp_ln35_fu_173_p2));
  FDRE \icmp_ln35_reg_289_pp0_iter1_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln35_reg_289),
        .Q(icmp_ln35_reg_289_pp0_iter1_reg),
        .R(1'b0));
  FDRE \icmp_ln35_reg_289_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln35_reg_289_pp0_iter1_reg),
        .Q(icmp_ln35_reg_289_pp0_iter2_reg),
        .R(1'b0));
  FDRE \icmp_ln35_reg_289_reg[0] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(icmp_ln35_fu_173_p2),
        .Q(icmp_ln35_reg_289),
        .R(1'b0));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_mul_32s_34ns_65_2_1 mul_32s_34ns_65_2_1_U1
       (.D({buff0_reg__1[41:16],mul_32s_34ns_65_2_1_U1_n_41,mul_32s_34ns_65_2_1_U1_n_42,mul_32s_34ns_65_2_1_U1_n_43,mul_32s_34ns_65_2_1_U1_n_44,mul_32s_34ns_65_2_1_U1_n_45,mul_32s_34ns_65_2_1_U1_n_46,mul_32s_34ns_65_2_1_U1_n_47,mul_32s_34ns_65_2_1_U1_n_48,mul_32s_34ns_65_2_1_U1_n_49,mul_32s_34ns_65_2_1_U1_n_50,mul_32s_34ns_65_2_1_U1_n_51,mul_32s_34ns_65_2_1_U1_n_52,mul_32s_34ns_65_2_1_U1_n_53,mul_32s_34ns_65_2_1_U1_n_54,mul_32s_34ns_65_2_1_U1_n_55,mul_32s_34ns_65_2_1_U1_n_56}),
        .S({mul_32s_34ns_65_2_1_U1_n_0,mul_32s_34ns_65_2_1_U1_n_1,mul_32s_34ns_65_2_1_U1_n_2,mul_32s_34ns_65_2_1_U1_n_3}),
        .add_ln32_fu_145_p2(add_ln32_fu_145_p2),
        .add_ln32_reg_2830(add_ln32_reg_2830),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .buff0_reg__0_0(buff0_reg__1[57:42]),
        .sum_reg(sum_reg[31:16]),
        .\sum_reg[18] ({mul_32s_34ns_65_2_1_U1_n_4,mul_32s_34ns_65_2_1_U1_n_5,mul_32s_34ns_65_2_1_U1_n_6}),
        .\sum_reg[22] ({mul_32s_34ns_65_2_1_U1_n_7,mul_32s_34ns_65_2_1_U1_n_8,mul_32s_34ns_65_2_1_U1_n_9,mul_32s_34ns_65_2_1_U1_n_10}),
        .\sum_reg[26] ({mul_32s_34ns_65_2_1_U1_n_11,mul_32s_34ns_65_2_1_U1_n_12,mul_32s_34ns_65_2_1_U1_n_13,mul_32s_34ns_65_2_1_U1_n_14}));
  FDRE \mul_ln39_reg_304_reg[0] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_56),
        .Q(mul_ln39_reg_304[0]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[10] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_46),
        .Q(mul_ln39_reg_304[10]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[11] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_45),
        .Q(mul_ln39_reg_304[11]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[12] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_44),
        .Q(mul_ln39_reg_304[12]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[13] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_43),
        .Q(mul_ln39_reg_304[13]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[14] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_42),
        .Q(mul_ln39_reg_304[14]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[15] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_41),
        .Q(mul_ln39_reg_304[15]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[16] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[16]),
        .Q(mul_ln39_reg_304[16]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[17] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[17]),
        .Q(mul_ln39_reg_304[17]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[18] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[18]),
        .Q(mul_ln39_reg_304[18]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[19] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[19]),
        .Q(mul_ln39_reg_304[19]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[1] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_55),
        .Q(mul_ln39_reg_304[1]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[20] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[20]),
        .Q(mul_ln39_reg_304[20]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[21] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[21]),
        .Q(mul_ln39_reg_304[21]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[22] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[22]),
        .Q(mul_ln39_reg_304[22]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[23] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[23]),
        .Q(mul_ln39_reg_304[23]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[24] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[24]),
        .Q(mul_ln39_reg_304[24]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[25] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[25]),
        .Q(mul_ln39_reg_304[25]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[26] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[26]),
        .Q(mul_ln39_reg_304[26]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[27] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[27]),
        .Q(mul_ln39_reg_304[27]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[28] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[28]),
        .Q(mul_ln39_reg_304[28]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[29] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[29]),
        .Q(mul_ln39_reg_304[29]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[2] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_54),
        .Q(mul_ln39_reg_304[2]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[30] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[30]),
        .Q(mul_ln39_reg_304[30]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[31] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[31]),
        .Q(mul_ln39_reg_304[31]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[32] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[32]),
        .Q(mul_ln39_reg_304[32]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[33] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[33]),
        .Q(mul_ln39_reg_304[33]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[34] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[34]),
        .Q(mul_ln39_reg_304[34]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[35] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[35]),
        .Q(mul_ln39_reg_304[35]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[36] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[36]),
        .Q(mul_ln39_reg_304[36]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[37] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[37]),
        .Q(mul_ln39_reg_304[37]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[38] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[38]),
        .Q(mul_ln39_reg_304[38]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[39] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[39]),
        .Q(mul_ln39_reg_304[39]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[3] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_53),
        .Q(mul_ln39_reg_304[3]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[40] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[40]),
        .Q(mul_ln39_reg_304[40]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[41] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[41]),
        .Q(mul_ln39_reg_304[41]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[42] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[42]),
        .Q(mul_ln39_reg_304[42]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[43] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[43]),
        .Q(mul_ln39_reg_304[43]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[44] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[44]),
        .Q(mul_ln39_reg_304[44]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[45] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[45]),
        .Q(mul_ln39_reg_304[45]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[46] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[46]),
        .Q(mul_ln39_reg_304[46]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[47] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[47]),
        .Q(mul_ln39_reg_304[47]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[48] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[48]),
        .Q(mul_ln39_reg_304[48]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[49] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[49]),
        .Q(mul_ln39_reg_304[49]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[4] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_52),
        .Q(mul_ln39_reg_304[4]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[50] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[50]),
        .Q(mul_ln39_reg_304[50]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[51] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[51]),
        .Q(mul_ln39_reg_304[51]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[52] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[52]),
        .Q(mul_ln39_reg_304[52]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[53] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[53]),
        .Q(mul_ln39_reg_304[53]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[54] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[54]),
        .Q(mul_ln39_reg_304[54]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[55] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[55]),
        .Q(mul_ln39_reg_304[55]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[5] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_51),
        .Q(mul_ln39_reg_304[5]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[6] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_50),
        .Q(mul_ln39_reg_304[6]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[7] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_49),
        .Q(mul_ln39_reg_304[7]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[8] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_48),
        .Q(mul_ln39_reg_304[8]),
        .R(1'b0));
  FDRE \mul_ln39_reg_304_reg[9] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(mul_32s_34ns_65_2_1_U1_n_47),
        .Q(mul_ln39_reg_304[9]),
        .R(1'b0));
  (* srl_bus_name = "inst/\\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2 " *) 
  SRL16E \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TKEEP_int_regslice[0]),
        .Q(\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2 " *) 
  SRL16E \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TKEEP_int_regslice[1]),
        .Q(\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2 " *) 
  SRL16E \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TKEEP_int_regslice[2]),
        .Q(\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2 " *) 
  SRL16E \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TKEEP_int_regslice[3]),
        .Q(\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_n_0 ));
  FDRE \pkt_in_keep_V_reg_259_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_n_0 ),
        .Q(pkt_in_keep_V_reg_259_pp0_iter2_reg[0]),
        .R(1'b0));
  FDRE \pkt_in_keep_V_reg_259_pp0_iter2_reg_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_n_0 ),
        .Q(pkt_in_keep_V_reg_259_pp0_iter2_reg[1]),
        .R(1'b0));
  FDRE \pkt_in_keep_V_reg_259_pp0_iter2_reg_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_n_0 ),
        .Q(pkt_in_keep_V_reg_259_pp0_iter2_reg[2]),
        .R(1'b0));
  FDRE \pkt_in_keep_V_reg_259_pp0_iter2_reg_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_n_0 ),
        .Q(pkt_in_keep_V_reg_259_pp0_iter2_reg[3]),
        .R(1'b0));
  (* srl_bus_name = "inst/\\pkt_in_last_V_reg_269_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2 " *) 
  SRL16E \pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TLAST_int_regslice),
        .Q(\pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2_n_0 ));
  FDRE \pkt_in_last_V_reg_269_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2_n_0 ),
        .Q(pkt_in_last_V_reg_269_pp0_iter2_reg),
        .R(1'b0));
  (* srl_bus_name = "inst/\\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2 " *) 
  SRL16E \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TSTRB_int_regslice[0]),
        .Q(\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2 " *) 
  SRL16E \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TSTRB_int_regslice[1]),
        .Q(\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2 " *) 
  SRL16E \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TSTRB_int_regslice[2]),
        .Q(\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2 " *) 
  SRL16E \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TSTRB_int_regslice[3]),
        .Q(\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_n_0 ));
  FDRE \pkt_in_strb_V_reg_264_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_n_0 ),
        .Q(pkt_in_strb_V_reg_264_pp0_iter2_reg[0]),
        .R(1'b0));
  FDRE \pkt_in_strb_V_reg_264_pp0_iter2_reg_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_n_0 ),
        .Q(pkt_in_strb_V_reg_264_pp0_iter2_reg[1]),
        .R(1'b0));
  FDRE \pkt_in_strb_V_reg_264_pp0_iter2_reg_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_n_0 ),
        .Q(pkt_in_strb_V_reg_264_pp0_iter2_reg[2]),
        .R(1'b0));
  FDRE \pkt_in_strb_V_reg_264_pp0_iter2_reg_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_n_0 ),
        .Q(pkt_in_strb_V_reg_264_pp0_iter2_reg[3]),
        .R(1'b0));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both regslice_both_in_stream_V_data_V_U
       (.\B_V_data_1_payload_B_reg[15]_0 ({regslice_both_in_stream_V_data_V_U_n_39,regslice_both_in_stream_V_data_V_U_n_40,regslice_both_in_stream_V_data_V_U_n_41,regslice_both_in_stream_V_data_V_U_n_42}),
        .\B_V_data_1_payload_B_reg[15]_1 ({regslice_both_in_stream_V_data_V_U_n_43,regslice_both_in_stream_V_data_V_U_n_44,regslice_both_in_stream_V_data_V_U_n_45,regslice_both_in_stream_V_data_V_U_n_46}),
        .\B_V_data_1_payload_B_reg[15]_2 ({regslice_both_in_stream_V_data_V_U_n_47,regslice_both_in_stream_V_data_V_U_n_48,regslice_both_in_stream_V_data_V_U_n_49,regslice_both_in_stream_V_data_V_U_n_50}),
        .\B_V_data_1_state_reg[1]_0 (in_stream_TREADY),
        .\B_V_data_1_state_reg[1]_1 (regslice_both_out_stream_V_data_V_U_n_3),
        .O({regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_20,regslice_both_in_stream_V_data_V_U_n_21,regslice_both_in_stream_V_data_V_U_n_22}),
        .S({mul_32s_34ns_65_2_1_U1_n_0,mul_32s_34ns_65_2_1_U1_n_1,mul_32s_34ns_65_2_1_U1_n_2,mul_32s_34ns_65_2_1_U1_n_3}),
        .add_ln32_fu_145_p2(add_ln32_fu_145_p2),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in(in),
        .in_stream_TDATA(in_stream_TDATA[15:0]),
        .in_stream_TVALID(in_stream_TVALID),
        .in_stream_TVALID_int_regslice(in_stream_TVALID_int_regslice),
        .sum_reg(sum_reg),
        .\sum_reg[11] ({regslice_both_in_stream_V_data_V_U_n_27,regslice_both_in_stream_V_data_V_U_n_28,regslice_both_in_stream_V_data_V_U_n_29,regslice_both_in_stream_V_data_V_U_n_30}),
        .\sum_reg[14] ({regslice_both_in_stream_V_data_V_U_n_31,regslice_both_in_stream_V_data_V_U_n_32,regslice_both_in_stream_V_data_V_U_n_33,regslice_both_in_stream_V_data_V_U_n_34}),
        .\sum_reg[14]_0 ({regslice_both_in_stream_V_data_V_U_n_35,regslice_both_in_stream_V_data_V_U_n_36,regslice_both_in_stream_V_data_V_U_n_37,regslice_both_in_stream_V_data_V_U_n_38}),
        .\sum_reg[7] ({regslice_both_in_stream_V_data_V_U_n_23,regslice_both_in_stream_V_data_V_U_n_24,regslice_both_in_stream_V_data_V_U_n_25,regslice_both_in_stream_V_data_V_U_n_26}),
        .tmp_product({mul_32s_34ns_65_2_1_U1_n_4,mul_32s_34ns_65_2_1_U1_n_5,mul_32s_34ns_65_2_1_U1_n_6}),
        .tmp_product_0({mul_32s_34ns_65_2_1_U1_n_7,mul_32s_34ns_65_2_1_U1_n_8,mul_32s_34ns_65_2_1_U1_n_9,mul_32s_34ns_65_2_1_U1_n_10}),
        .tmp_product_1({mul_32s_34ns_65_2_1_U1_n_11,mul_32s_34ns_65_2_1_U1_n_12,mul_32s_34ns_65_2_1_U1_n_13,mul_32s_34ns_65_2_1_U1_n_14}));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0 regslice_both_in_stream_V_keep_V_U
       (.\B_V_data_1_state_reg[0]_0 (regslice_both_out_stream_V_data_V_U_n_3),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_stream_TKEEP(in_stream_TKEEP),
        .in_stream_TKEEP_int_regslice(in_stream_TKEEP_int_regslice),
        .in_stream_TVALID(in_stream_TVALID));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1 regslice_both_in_stream_V_last_V_U
       (.\B_V_data_1_state_reg[0]_0 (regslice_both_out_stream_V_data_V_U_n_3),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_stream_TLAST(in_stream_TLAST),
        .in_stream_TLAST_int_regslice(in_stream_TLAST_int_regslice),
        .in_stream_TVALID(in_stream_TVALID));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_0 regslice_both_in_stream_V_strb_V_U
       (.\B_V_data_1_state_reg[0]_0 (regslice_both_out_stream_V_data_V_U_n_3),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_stream_TSTRB(in_stream_TSTRB),
        .in_stream_TSTRB_int_regslice(in_stream_TSTRB_int_regslice),
        .in_stream_TVALID(in_stream_TVALID));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both_1 regslice_both_out_stream_V_data_V_U
       (.\B_V_data_1_state_reg[0]_0 (out_stream_TVALID),
        .\B_V_data_1_state_reg[0]_1 (regslice_both_out_stream_V_data_V_U_n_3),
        .E(mul_ln39_reg_3040),
        .Q(dc_offset),
        .add_ln32_reg_2830(add_ln32_reg_2830),
        .\add_ln32_reg_283_reg[31] (regslice_both_out_stream_V_data_V_U_n_12),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter3(ap_enable_reg_pp0_iter3),
        .ap_enable_reg_pp0_iter4(ap_enable_reg_pp0_iter4),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .calibrated(calibrated),
        .calibrated_load_reg_279(calibrated_load_reg_279),
        .calibrated_load_reg_279_pp0_iter1_reg(calibrated_load_reg_279_pp0_iter1_reg),
        .calibrated_load_reg_279_pp0_iter2_reg(calibrated_load_reg_279_pp0_iter2_reg),
        .calibrated_load_reg_279_pp0_iter3_reg(calibrated_load_reg_279_pp0_iter3_reg),
        .\calibrated_reg[0] (regslice_both_out_stream_V_data_V_U_n_5),
        .\counter_reg[0] (regslice_both_out_stream_V_data_V_U_n_6),
        .\counter_reg[0]_0 (regslice_both_out_stream_V_data_V_U_n_7),
        .\counter_reg[20] (regslice_both_out_stream_V_data_V_U_n_9),
        .\counter_reg[28] (regslice_both_out_stream_V_data_V_U_n_8),
        .icmp_ln35_reg_289(icmp_ln35_reg_289),
        .icmp_ln35_reg_289_pp0_iter1_reg(icmp_ln35_reg_289_pp0_iter1_reg),
        .icmp_ln35_reg_289_pp0_iter2_reg(icmp_ln35_reg_289_pp0_iter2_reg),
        .\icmp_ln35_reg_289_pp0_iter2_reg_reg[0] (dc_offset0),
        .in_stream_TVALID_int_regslice(in_stream_TVALID_int_regslice),
        .out(counter_reg),
        .out_stream_TDATA({\^out_stream_TDATA [31],\^out_stream_TDATA [14:0]}),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TVALID_int_regslice(out_stream_TVALID_int_regslice),
        .p_0_in(p_0_in),
        .tmp_reg_298(tmp_reg_298),
        .val_in_reg_274_pp0_iter2_reg(val_in_reg_274_pp0_iter2_reg));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_2 regslice_both_out_stream_V_keep_V_U
       (.D(pkt_in_keep_V_reg_259_pp0_iter2_reg),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .out_stream_TKEEP(out_stream_TKEEP),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TVALID_int_regslice(out_stream_TVALID_int_regslice));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1_3 regslice_both_out_stream_V_last_V_U
       (.ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .out_stream_TLAST(out_stream_TLAST),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TVALID_int_regslice(out_stream_TVALID_int_regslice),
        .pkt_in_last_V_reg_269_pp0_iter2_reg(pkt_in_last_V_reg_269_pp0_iter2_reg));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_4 regslice_both_out_stream_V_strb_V_U
       (.D(pkt_in_strb_V_reg_264_pp0_iter2_reg),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TSTRB(out_stream_TSTRB),
        .out_stream_TVALID_int_regslice(out_stream_TVALID_int_regslice));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[0] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_22),
        .Q(sum_reg[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[10] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_28),
        .Q(sum_reg[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[11] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_27),
        .Q(sum_reg[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[12] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_34),
        .Q(sum_reg[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[13] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_33),
        .Q(sum_reg[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[14] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_32),
        .Q(sum_reg[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[15] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_31),
        .Q(sum_reg[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[16] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_38),
        .Q(sum_reg[16]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[17] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_37),
        .Q(sum_reg[17]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[18] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_36),
        .Q(sum_reg[18]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[19] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_35),
        .Q(sum_reg[19]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[1] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_21),
        .Q(sum_reg[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[20] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_42),
        .Q(sum_reg[20]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[21] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_41),
        .Q(sum_reg[21]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[22] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_40),
        .Q(sum_reg[22]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[23] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_39),
        .Q(sum_reg[23]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[24] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_46),
        .Q(sum_reg[24]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[25] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_45),
        .Q(sum_reg[25]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[26] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_44),
        .Q(sum_reg[26]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[27] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_43),
        .Q(sum_reg[27]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[28] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_50),
        .Q(sum_reg[28]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[29] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_49),
        .Q(sum_reg[29]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[2] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_20),
        .Q(sum_reg[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[30] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_48),
        .Q(sum_reg[30]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[31] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_47),
        .Q(sum_reg[31]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[3] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_19),
        .Q(sum_reg[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[4] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_26),
        .Q(sum_reg[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[5] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_25),
        .Q(sum_reg[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[6] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_24),
        .Q(sum_reg[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[7] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_23),
        .Q(sum_reg[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[8] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_30),
        .Q(sum_reg[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \sum_reg[9] 
       (.C(ap_clk),
        .CE(add_ln32_reg_2830),
        .D(regslice_both_in_stream_V_data_V_U_n_29),
        .Q(sum_reg[9]),
        .R(1'b0));
  FDRE \tmp_2_reg_309_reg[14] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[56]),
        .Q(tmp_2_reg_309[14]),
        .R(1'b0));
  FDRE \tmp_2_reg_309_reg[15] 
       (.C(ap_clk),
        .CE(mul_ln39_reg_3040),
        .D(buff0_reg__1[57]),
        .Q(tmp_2_reg_309[15]),
        .R(1'b0));
  FDRE \tmp_reg_298_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_reg_298),
        .Q(tmp_reg_298_pp0_iter2_reg),
        .R(1'b0));
  FDRE \tmp_reg_298_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_out_stream_V_data_V_U_n_12),
        .Q(tmp_reg_298),
        .R(1'b0));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[0]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[0]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[10]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[10]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[11]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[11]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[12]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[12]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[13]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[13]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[14]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[14]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[15]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[15]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[1]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[1]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[2]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[2]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[3]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[3]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[4]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[4]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[5]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[5]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[6]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[6]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[7]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[7]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[8]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[8]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\val_in_reg_274_pp0_iter1_reg_reg[9]_srl2 " *) 
  SRL16E \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in[9]),
        .Q(\val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_n_0 ));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[0]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[10]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[11]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[12]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[13]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[14]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[15]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[1]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[2]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[3]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[4]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[5]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[6]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[7]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[8]),
        .R(1'b0));
  FDRE \val_in_reg_274_pp0_iter2_reg_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_n_0 ),
        .Q(val_in_reg_274_pp0_iter2_reg[9]),
        .R(1'b0));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_mul_32s_34ns_65_2_1
   (S,
    \sum_reg[18] ,
    \sum_reg[22] ,
    \sum_reg[26] ,
    D,
    buff0_reg__0_0,
    add_ln32_reg_2830,
    ap_clk,
    add_ln32_fu_145_p2,
    ap_block_pp0_stage0_11001,
    sum_reg);
  output [3:0]S;
  output [2:0]\sum_reg[18] ;
  output [3:0]\sum_reg[22] ;
  output [3:0]\sum_reg[26] ;
  output [41:0]D;
  output [15:0]buff0_reg__0_0;
  input add_ln32_reg_2830;
  input ap_clk;
  input [31:0]add_ln32_fu_145_p2;
  input ap_block_pp0_stage0_11001;
  input [15:0]sum_reg;

  wire [41:0]D;
  wire [3:0]S;
  wire [31:0]add_ln32_fu_145_p2;
  wire add_ln32_reg_2830;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire \buff0_reg[16]__0_n_0 ;
  wire [15:0]buff0_reg__0_0;
  wire buff0_reg__0_n_100;
  wire buff0_reg__0_n_101;
  wire buff0_reg__0_n_102;
  wire buff0_reg__0_n_103;
  wire buff0_reg__0_n_104;
  wire buff0_reg__0_n_105;
  wire buff0_reg__0_n_58;
  wire buff0_reg__0_n_59;
  wire buff0_reg__0_n_60;
  wire buff0_reg__0_n_61;
  wire buff0_reg__0_n_62;
  wire buff0_reg__0_n_63;
  wire buff0_reg__0_n_64;
  wire buff0_reg__0_n_65;
  wire buff0_reg__0_n_66;
  wire buff0_reg__0_n_67;
  wire buff0_reg__0_n_68;
  wire buff0_reg__0_n_69;
  wire buff0_reg__0_n_70;
  wire buff0_reg__0_n_71;
  wire buff0_reg__0_n_72;
  wire buff0_reg__0_n_73;
  wire buff0_reg__0_n_74;
  wire buff0_reg__0_n_75;
  wire buff0_reg__0_n_76;
  wire buff0_reg__0_n_77;
  wire buff0_reg__0_n_78;
  wire buff0_reg__0_n_79;
  wire buff0_reg__0_n_80;
  wire buff0_reg__0_n_81;
  wire buff0_reg__0_n_82;
  wire buff0_reg__0_n_83;
  wire buff0_reg__0_n_84;
  wire buff0_reg__0_n_85;
  wire buff0_reg__0_n_86;
  wire buff0_reg__0_n_87;
  wire buff0_reg__0_n_88;
  wire buff0_reg__0_n_89;
  wire buff0_reg__0_n_90;
  wire buff0_reg__0_n_91;
  wire buff0_reg__0_n_92;
  wire buff0_reg__0_n_93;
  wire buff0_reg__0_n_94;
  wire buff0_reg__0_n_95;
  wire buff0_reg__0_n_96;
  wire buff0_reg__0_n_97;
  wire buff0_reg__0_n_98;
  wire buff0_reg__0_n_99;
  wire \buff0_reg_n_0_[0] ;
  wire \buff0_reg_n_0_[10] ;
  wire \buff0_reg_n_0_[11] ;
  wire \buff0_reg_n_0_[12] ;
  wire \buff0_reg_n_0_[13] ;
  wire \buff0_reg_n_0_[14] ;
  wire \buff0_reg_n_0_[15] ;
  wire \buff0_reg_n_0_[16] ;
  wire \buff0_reg_n_0_[1] ;
  wire \buff0_reg_n_0_[2] ;
  wire \buff0_reg_n_0_[3] ;
  wire \buff0_reg_n_0_[4] ;
  wire \buff0_reg_n_0_[5] ;
  wire \buff0_reg_n_0_[6] ;
  wire \buff0_reg_n_0_[7] ;
  wire \buff0_reg_n_0_[8] ;
  wire \buff0_reg_n_0_[9] ;
  wire buff0_reg_n_100;
  wire buff0_reg_n_101;
  wire buff0_reg_n_102;
  wire buff0_reg_n_103;
  wire buff0_reg_n_104;
  wire buff0_reg_n_105;
  wire buff0_reg_n_58;
  wire buff0_reg_n_59;
  wire buff0_reg_n_60;
  wire buff0_reg_n_61;
  wire buff0_reg_n_62;
  wire buff0_reg_n_63;
  wire buff0_reg_n_64;
  wire buff0_reg_n_65;
  wire buff0_reg_n_66;
  wire buff0_reg_n_67;
  wire buff0_reg_n_68;
  wire buff0_reg_n_69;
  wire buff0_reg_n_70;
  wire buff0_reg_n_71;
  wire buff0_reg_n_72;
  wire buff0_reg_n_73;
  wire buff0_reg_n_74;
  wire buff0_reg_n_75;
  wire buff0_reg_n_76;
  wire buff0_reg_n_77;
  wire buff0_reg_n_78;
  wire buff0_reg_n_79;
  wire buff0_reg_n_80;
  wire buff0_reg_n_81;
  wire buff0_reg_n_82;
  wire buff0_reg_n_83;
  wire buff0_reg_n_84;
  wire buff0_reg_n_85;
  wire buff0_reg_n_86;
  wire buff0_reg_n_87;
  wire buff0_reg_n_88;
  wire buff0_reg_n_89;
  wire buff0_reg_n_90;
  wire buff0_reg_n_91;
  wire buff0_reg_n_92;
  wire buff0_reg_n_93;
  wire buff0_reg_n_94;
  wire buff0_reg_n_95;
  wire buff0_reg_n_96;
  wire buff0_reg_n_97;
  wire buff0_reg_n_98;
  wire buff0_reg_n_99;
  wire \mul_ln39_reg_304[19]_i_2_n_0 ;
  wire \mul_ln39_reg_304[19]_i_3_n_0 ;
  wire \mul_ln39_reg_304[19]_i_4_n_0 ;
  wire \mul_ln39_reg_304[23]_i_2_n_0 ;
  wire \mul_ln39_reg_304[23]_i_3_n_0 ;
  wire \mul_ln39_reg_304[23]_i_4_n_0 ;
  wire \mul_ln39_reg_304[23]_i_5_n_0 ;
  wire \mul_ln39_reg_304[27]_i_2_n_0 ;
  wire \mul_ln39_reg_304[27]_i_3_n_0 ;
  wire \mul_ln39_reg_304[27]_i_4_n_0 ;
  wire \mul_ln39_reg_304[27]_i_5_n_0 ;
  wire \mul_ln39_reg_304[31]_i_2_n_0 ;
  wire \mul_ln39_reg_304[31]_i_3_n_0 ;
  wire \mul_ln39_reg_304[31]_i_4_n_0 ;
  wire \mul_ln39_reg_304[31]_i_5_n_0 ;
  wire \mul_ln39_reg_304[35]_i_2_n_0 ;
  wire \mul_ln39_reg_304[35]_i_3_n_0 ;
  wire \mul_ln39_reg_304[35]_i_4_n_0 ;
  wire \mul_ln39_reg_304[35]_i_5_n_0 ;
  wire \mul_ln39_reg_304[39]_i_2_n_0 ;
  wire \mul_ln39_reg_304[39]_i_3_n_0 ;
  wire \mul_ln39_reg_304[39]_i_4_n_0 ;
  wire \mul_ln39_reg_304[39]_i_5_n_0 ;
  wire \mul_ln39_reg_304[41]_i_3_n_0 ;
  wire \mul_ln39_reg_304[41]_i_4_n_0 ;
  wire \mul_ln39_reg_304[41]_i_5_n_0 ;
  wire \mul_ln39_reg_304[41]_i_6_n_0 ;
  wire \mul_ln39_reg_304[44]_i_2_n_0 ;
  wire \mul_ln39_reg_304[44]_i_3_n_0 ;
  wire \mul_ln39_reg_304[44]_i_4_n_0 ;
  wire \mul_ln39_reg_304[44]_i_5_n_0 ;
  wire \mul_ln39_reg_304[48]_i_2_n_0 ;
  wire \mul_ln39_reg_304[48]_i_3_n_0 ;
  wire \mul_ln39_reg_304[48]_i_4_n_0 ;
  wire \mul_ln39_reg_304[48]_i_5_n_0 ;
  wire \mul_ln39_reg_304[52]_i_2_n_0 ;
  wire \mul_ln39_reg_304[52]_i_3_n_0 ;
  wire \mul_ln39_reg_304[52]_i_4_n_0 ;
  wire \mul_ln39_reg_304[52]_i_5_n_0 ;
  wire \mul_ln39_reg_304_reg[19]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[19]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[19]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[19]_i_1_n_3 ;
  wire \mul_ln39_reg_304_reg[23]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[23]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[23]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[23]_i_1_n_3 ;
  wire \mul_ln39_reg_304_reg[27]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[27]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[27]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[27]_i_1_n_3 ;
  wire \mul_ln39_reg_304_reg[31]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[31]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[31]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[31]_i_1_n_3 ;
  wire \mul_ln39_reg_304_reg[35]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[35]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[35]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[35]_i_1_n_3 ;
  wire \mul_ln39_reg_304_reg[39]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[39]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[39]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[39]_i_1_n_3 ;
  wire \mul_ln39_reg_304_reg[41]_i_2_n_0 ;
  wire \mul_ln39_reg_304_reg[41]_i_2_n_1 ;
  wire \mul_ln39_reg_304_reg[41]_i_2_n_2 ;
  wire \mul_ln39_reg_304_reg[41]_i_2_n_3 ;
  wire \mul_ln39_reg_304_reg[44]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[44]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[44]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[44]_i_1_n_3 ;
  wire \mul_ln39_reg_304_reg[48]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[48]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[48]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[48]_i_1_n_3 ;
  wire \mul_ln39_reg_304_reg[52]_i_1_n_0 ;
  wire \mul_ln39_reg_304_reg[52]_i_1_n_1 ;
  wire \mul_ln39_reg_304_reg[52]_i_1_n_2 ;
  wire \mul_ln39_reg_304_reg[52]_i_1_n_3 ;
  wire [15:0]sum_reg;
  wire [2:0]\sum_reg[18] ;
  wire [3:0]\sum_reg[22] ;
  wire [3:0]\sum_reg[26] ;
  wire \tmp_2_reg_309[15]_i_2_n_0 ;
  wire \tmp_2_reg_309[15]_i_3_n_0 ;
  wire \tmp_2_reg_309_reg[15]_i_1_n_3 ;
  wire tmp_product__0_n_100;
  wire tmp_product__0_n_101;
  wire tmp_product__0_n_102;
  wire tmp_product__0_n_103;
  wire tmp_product__0_n_104;
  wire tmp_product__0_n_105;
  wire tmp_product__0_n_106;
  wire tmp_product__0_n_107;
  wire tmp_product__0_n_108;
  wire tmp_product__0_n_109;
  wire tmp_product__0_n_110;
  wire tmp_product__0_n_111;
  wire tmp_product__0_n_112;
  wire tmp_product__0_n_113;
  wire tmp_product__0_n_114;
  wire tmp_product__0_n_115;
  wire tmp_product__0_n_116;
  wire tmp_product__0_n_117;
  wire tmp_product__0_n_118;
  wire tmp_product__0_n_119;
  wire tmp_product__0_n_120;
  wire tmp_product__0_n_121;
  wire tmp_product__0_n_122;
  wire tmp_product__0_n_123;
  wire tmp_product__0_n_124;
  wire tmp_product__0_n_125;
  wire tmp_product__0_n_126;
  wire tmp_product__0_n_127;
  wire tmp_product__0_n_128;
  wire tmp_product__0_n_129;
  wire tmp_product__0_n_130;
  wire tmp_product__0_n_131;
  wire tmp_product__0_n_132;
  wire tmp_product__0_n_133;
  wire tmp_product__0_n_134;
  wire tmp_product__0_n_135;
  wire tmp_product__0_n_136;
  wire tmp_product__0_n_137;
  wire tmp_product__0_n_138;
  wire tmp_product__0_n_139;
  wire tmp_product__0_n_140;
  wire tmp_product__0_n_141;
  wire tmp_product__0_n_142;
  wire tmp_product__0_n_143;
  wire tmp_product__0_n_144;
  wire tmp_product__0_n_145;
  wire tmp_product__0_n_146;
  wire tmp_product__0_n_147;
  wire tmp_product__0_n_148;
  wire tmp_product__0_n_149;
  wire tmp_product__0_n_150;
  wire tmp_product__0_n_151;
  wire tmp_product__0_n_152;
  wire tmp_product__0_n_153;
  wire tmp_product__0_n_24;
  wire tmp_product__0_n_25;
  wire tmp_product__0_n_26;
  wire tmp_product__0_n_27;
  wire tmp_product__0_n_28;
  wire tmp_product__0_n_29;
  wire tmp_product__0_n_30;
  wire tmp_product__0_n_31;
  wire tmp_product__0_n_32;
  wire tmp_product__0_n_33;
  wire tmp_product__0_n_34;
  wire tmp_product__0_n_35;
  wire tmp_product__0_n_36;
  wire tmp_product__0_n_37;
  wire tmp_product__0_n_38;
  wire tmp_product__0_n_39;
  wire tmp_product__0_n_40;
  wire tmp_product__0_n_41;
  wire tmp_product__0_n_42;
  wire tmp_product__0_n_43;
  wire tmp_product__0_n_44;
  wire tmp_product__0_n_45;
  wire tmp_product__0_n_46;
  wire tmp_product__0_n_47;
  wire tmp_product__0_n_48;
  wire tmp_product__0_n_49;
  wire tmp_product__0_n_50;
  wire tmp_product__0_n_51;
  wire tmp_product__0_n_52;
  wire tmp_product__0_n_53;
  wire tmp_product__0_n_58;
  wire tmp_product__0_n_59;
  wire tmp_product__0_n_60;
  wire tmp_product__0_n_61;
  wire tmp_product__0_n_62;
  wire tmp_product__0_n_63;
  wire tmp_product__0_n_64;
  wire tmp_product__0_n_65;
  wire tmp_product__0_n_66;
  wire tmp_product__0_n_67;
  wire tmp_product__0_n_68;
  wire tmp_product__0_n_69;
  wire tmp_product__0_n_70;
  wire tmp_product__0_n_71;
  wire tmp_product__0_n_72;
  wire tmp_product__0_n_73;
  wire tmp_product__0_n_74;
  wire tmp_product__0_n_75;
  wire tmp_product__0_n_76;
  wire tmp_product__0_n_77;
  wire tmp_product__0_n_78;
  wire tmp_product__0_n_79;
  wire tmp_product__0_n_80;
  wire tmp_product__0_n_81;
  wire tmp_product__0_n_82;
  wire tmp_product__0_n_83;
  wire tmp_product__0_n_84;
  wire tmp_product__0_n_85;
  wire tmp_product__0_n_86;
  wire tmp_product__0_n_87;
  wire tmp_product__0_n_88;
  wire tmp_product__0_n_89;
  wire tmp_product__0_n_90;
  wire tmp_product__0_n_91;
  wire tmp_product__0_n_92;
  wire tmp_product__0_n_93;
  wire tmp_product__0_n_94;
  wire tmp_product__0_n_95;
  wire tmp_product__0_n_96;
  wire tmp_product__0_n_97;
  wire tmp_product__0_n_98;
  wire tmp_product__0_n_99;
  wire tmp_product_n_100;
  wire tmp_product_n_101;
  wire tmp_product_n_102;
  wire tmp_product_n_103;
  wire tmp_product_n_104;
  wire tmp_product_n_105;
  wire tmp_product_n_106;
  wire tmp_product_n_107;
  wire tmp_product_n_108;
  wire tmp_product_n_109;
  wire tmp_product_n_110;
  wire tmp_product_n_111;
  wire tmp_product_n_112;
  wire tmp_product_n_113;
  wire tmp_product_n_114;
  wire tmp_product_n_115;
  wire tmp_product_n_116;
  wire tmp_product_n_117;
  wire tmp_product_n_118;
  wire tmp_product_n_119;
  wire tmp_product_n_120;
  wire tmp_product_n_121;
  wire tmp_product_n_122;
  wire tmp_product_n_123;
  wire tmp_product_n_124;
  wire tmp_product_n_125;
  wire tmp_product_n_126;
  wire tmp_product_n_127;
  wire tmp_product_n_128;
  wire tmp_product_n_129;
  wire tmp_product_n_130;
  wire tmp_product_n_131;
  wire tmp_product_n_132;
  wire tmp_product_n_133;
  wire tmp_product_n_134;
  wire tmp_product_n_135;
  wire tmp_product_n_136;
  wire tmp_product_n_137;
  wire tmp_product_n_138;
  wire tmp_product_n_139;
  wire tmp_product_n_140;
  wire tmp_product_n_141;
  wire tmp_product_n_142;
  wire tmp_product_n_143;
  wire tmp_product_n_144;
  wire tmp_product_n_145;
  wire tmp_product_n_146;
  wire tmp_product_n_147;
  wire tmp_product_n_148;
  wire tmp_product_n_149;
  wire tmp_product_n_150;
  wire tmp_product_n_151;
  wire tmp_product_n_152;
  wire tmp_product_n_153;
  wire tmp_product_n_58;
  wire tmp_product_n_59;
  wire tmp_product_n_60;
  wire tmp_product_n_61;
  wire tmp_product_n_62;
  wire tmp_product_n_63;
  wire tmp_product_n_64;
  wire tmp_product_n_65;
  wire tmp_product_n_66;
  wire tmp_product_n_67;
  wire tmp_product_n_68;
  wire tmp_product_n_69;
  wire tmp_product_n_70;
  wire tmp_product_n_71;
  wire tmp_product_n_72;
  wire tmp_product_n_73;
  wire tmp_product_n_74;
  wire tmp_product_n_75;
  wire tmp_product_n_76;
  wire tmp_product_n_77;
  wire tmp_product_n_78;
  wire tmp_product_n_79;
  wire tmp_product_n_80;
  wire tmp_product_n_81;
  wire tmp_product_n_82;
  wire tmp_product_n_83;
  wire tmp_product_n_84;
  wire tmp_product_n_85;
  wire tmp_product_n_86;
  wire tmp_product_n_87;
  wire tmp_product_n_88;
  wire tmp_product_n_89;
  wire tmp_product_n_90;
  wire tmp_product_n_91;
  wire tmp_product_n_92;
  wire tmp_product_n_93;
  wire tmp_product_n_94;
  wire tmp_product_n_95;
  wire tmp_product_n_96;
  wire tmp_product_n_97;
  wire tmp_product_n_98;
  wire tmp_product_n_99;
  wire NLW_buff0_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_buff0_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_buff0_reg_OVERFLOW_UNCONNECTED;
  wire NLW_buff0_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_buff0_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_buff0_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_buff0_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_buff0_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_buff0_reg_CARRYOUT_UNCONNECTED;
  wire [47:0]NLW_buff0_reg_PCOUT_UNCONNECTED;
  wire NLW_buff0_reg__0_CARRYCASCOUT_UNCONNECTED;
  wire NLW_buff0_reg__0_MULTSIGNOUT_UNCONNECTED;
  wire NLW_buff0_reg__0_OVERFLOW_UNCONNECTED;
  wire NLW_buff0_reg__0_PATTERNBDETECT_UNCONNECTED;
  wire NLW_buff0_reg__0_PATTERNDETECT_UNCONNECTED;
  wire NLW_buff0_reg__0_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_buff0_reg__0_ACOUT_UNCONNECTED;
  wire [17:0]NLW_buff0_reg__0_BCOUT_UNCONNECTED;
  wire [3:0]NLW_buff0_reg__0_CARRYOUT_UNCONNECTED;
  wire [47:0]NLW_buff0_reg__0_PCOUT_UNCONNECTED;
  wire [3:1]\NLW_tmp_2_reg_309_reg[15]_i_1_CO_UNCONNECTED ;
  wire [3:2]\NLW_tmp_2_reg_309_reg[15]_i_1_O_UNCONNECTED ;
  wire NLW_tmp_product_CARRYCASCOUT_UNCONNECTED;
  wire NLW_tmp_product_MULTSIGNOUT_UNCONNECTED;
  wire NLW_tmp_product_OVERFLOW_UNCONNECTED;
  wire NLW_tmp_product_PATTERNBDETECT_UNCONNECTED;
  wire NLW_tmp_product_PATTERNDETECT_UNCONNECTED;
  wire NLW_tmp_product_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_tmp_product_ACOUT_UNCONNECTED;
  wire [17:0]NLW_tmp_product_BCOUT_UNCONNECTED;
  wire [3:0]NLW_tmp_product_CARRYOUT_UNCONNECTED;
  wire NLW_tmp_product__0_CARRYCASCOUT_UNCONNECTED;
  wire NLW_tmp_product__0_MULTSIGNOUT_UNCONNECTED;
  wire NLW_tmp_product__0_OVERFLOW_UNCONNECTED;
  wire NLW_tmp_product__0_PATTERNBDETECT_UNCONNECTED;
  wire NLW_tmp_product__0_PATTERNDETECT_UNCONNECTED;
  wire NLW_tmp_product__0_UNDERFLOW_UNCONNECTED;
  wire [17:0]NLW_tmp_product__0_BCOUT_UNCONNECTED;
  wire [3:0]NLW_tmp_product__0_CARRYOUT_UNCONNECTED;

  LUT2 #(
    .INIT(4'h9)) 
    \add_ln32_reg_283[31]_i_3 
       (.I0(sum_reg[14]),
        .I1(sum_reg[15]),
        .O(S[3]));
  LUT2 #(
    .INIT(4'h9)) 
    \add_ln32_reg_283[31]_i_4 
       (.I0(sum_reg[13]),
        .I1(sum_reg[14]),
        .O(S[2]));
  LUT2 #(
    .INIT(4'h9)) 
    \add_ln32_reg_283[31]_i_5 
       (.I0(sum_reg[12]),
        .I1(sum_reg[13]),
        .O(S[1]));
  LUT2 #(
    .INIT(4'h9)) 
    \add_ln32_reg_283[31]_i_6 
       (.I0(sum_reg[11]),
        .I1(sum_reg[12]),
        .O(S[0]));
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-10 {cell *THIS*} {string 15x17 4}}" *) 
  DSP48E1 #(
    .ACASCREG(0),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(0),
    .AUTORESET_PATDET("NO_RESET"),
    .A_INPUT("DIRECT"),
    .BCASCREG(1),
    .BREG(1),
    .B_INPUT("DIRECT"),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .CREG(1),
    .DREG(1),
    .INMODEREG(0),
    .MASK(48'h3FFFFFFFFFFF),
    .MREG(0),
    .OPMODEREG(0),
    .PATTERN(48'h000000000000),
    .PREG(1),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_DPORT("FALSE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    buff0_reg
       (.A({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b0,1'b0,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0,1'b1,1'b1,1'b1}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_buff0_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31:17]}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_buff0_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_buff0_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_buff0_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(1'b0),
        .CEA2(1'b0),
        .CEAD(1'b0),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(add_ln32_reg_2830),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(1'b0),
        .CEINMODE(1'b0),
        .CEM(1'b0),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .INMODE({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_buff0_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b1,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_buff0_reg_OVERFLOW_UNCONNECTED),
        .P({buff0_reg_n_58,buff0_reg_n_59,buff0_reg_n_60,buff0_reg_n_61,buff0_reg_n_62,buff0_reg_n_63,buff0_reg_n_64,buff0_reg_n_65,buff0_reg_n_66,buff0_reg_n_67,buff0_reg_n_68,buff0_reg_n_69,buff0_reg_n_70,buff0_reg_n_71,buff0_reg_n_72,buff0_reg_n_73,buff0_reg_n_74,buff0_reg_n_75,buff0_reg_n_76,buff0_reg_n_77,buff0_reg_n_78,buff0_reg_n_79,buff0_reg_n_80,buff0_reg_n_81,buff0_reg_n_82,buff0_reg_n_83,buff0_reg_n_84,buff0_reg_n_85,buff0_reg_n_86,buff0_reg_n_87,buff0_reg_n_88,buff0_reg_n_89,buff0_reg_n_90,buff0_reg_n_91,buff0_reg_n_92,buff0_reg_n_93,buff0_reg_n_94,buff0_reg_n_95,buff0_reg_n_96,buff0_reg_n_97,buff0_reg_n_98,buff0_reg_n_99,buff0_reg_n_100,buff0_reg_n_101,buff0_reg_n_102,buff0_reg_n_103,buff0_reg_n_104,buff0_reg_n_105}),
        .PATTERNBDETECT(NLW_buff0_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_buff0_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({tmp_product_n_106,tmp_product_n_107,tmp_product_n_108,tmp_product_n_109,tmp_product_n_110,tmp_product_n_111,tmp_product_n_112,tmp_product_n_113,tmp_product_n_114,tmp_product_n_115,tmp_product_n_116,tmp_product_n_117,tmp_product_n_118,tmp_product_n_119,tmp_product_n_120,tmp_product_n_121,tmp_product_n_122,tmp_product_n_123,tmp_product_n_124,tmp_product_n_125,tmp_product_n_126,tmp_product_n_127,tmp_product_n_128,tmp_product_n_129,tmp_product_n_130,tmp_product_n_131,tmp_product_n_132,tmp_product_n_133,tmp_product_n_134,tmp_product_n_135,tmp_product_n_136,tmp_product_n_137,tmp_product_n_138,tmp_product_n_139,tmp_product_n_140,tmp_product_n_141,tmp_product_n_142,tmp_product_n_143,tmp_product_n_144,tmp_product_n_145,tmp_product_n_146,tmp_product_n_147,tmp_product_n_148,tmp_product_n_149,tmp_product_n_150,tmp_product_n_151,tmp_product_n_152,tmp_product_n_153}),
        .PCOUT(NLW_buff0_reg_PCOUT_UNCONNECTED[47:0]),
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
        .UNDERFLOW(NLW_buff0_reg_UNDERFLOW_UNCONNECTED));
  FDRE \buff0_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_105),
        .Q(\buff0_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \buff0_reg[0]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_105),
        .Q(D[0]),
        .R(1'b0));
  FDRE \buff0_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_95),
        .Q(\buff0_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \buff0_reg[10]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_95),
        .Q(D[10]),
        .R(1'b0));
  FDRE \buff0_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_94),
        .Q(\buff0_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \buff0_reg[11]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_94),
        .Q(D[11]),
        .R(1'b0));
  FDRE \buff0_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_93),
        .Q(\buff0_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \buff0_reg[12]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_93),
        .Q(D[12]),
        .R(1'b0));
  FDRE \buff0_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_92),
        .Q(\buff0_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \buff0_reg[13]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_92),
        .Q(D[13]),
        .R(1'b0));
  FDRE \buff0_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_91),
        .Q(\buff0_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \buff0_reg[14]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_91),
        .Q(D[14]),
        .R(1'b0));
  FDRE \buff0_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_90),
        .Q(\buff0_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \buff0_reg[15]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_90),
        .Q(D[15]),
        .R(1'b0));
  FDRE \buff0_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_89),
        .Q(\buff0_reg_n_0_[16] ),
        .R(1'b0));
  FDRE \buff0_reg[16]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_89),
        .Q(\buff0_reg[16]__0_n_0 ),
        .R(1'b0));
  FDRE \buff0_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_104),
        .Q(\buff0_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \buff0_reg[1]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_104),
        .Q(D[1]),
        .R(1'b0));
  FDRE \buff0_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_103),
        .Q(\buff0_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \buff0_reg[2]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_103),
        .Q(D[2]),
        .R(1'b0));
  FDRE \buff0_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_102),
        .Q(\buff0_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \buff0_reg[3]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_102),
        .Q(D[3]),
        .R(1'b0));
  FDRE \buff0_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_101),
        .Q(\buff0_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \buff0_reg[4]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_101),
        .Q(D[4]),
        .R(1'b0));
  FDRE \buff0_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_100),
        .Q(\buff0_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \buff0_reg[5]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_100),
        .Q(D[5]),
        .R(1'b0));
  FDRE \buff0_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_99),
        .Q(\buff0_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \buff0_reg[6]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_99),
        .Q(D[6]),
        .R(1'b0));
  FDRE \buff0_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_98),
        .Q(\buff0_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \buff0_reg[7]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_98),
        .Q(D[7]),
        .R(1'b0));
  FDRE \buff0_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_97),
        .Q(\buff0_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \buff0_reg[8]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_97),
        .Q(D[8]),
        .R(1'b0));
  FDRE \buff0_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product_n_96),
        .Q(\buff0_reg_n_0_[9] ),
        .R(1'b0));
  FDRE \buff0_reg[9]__0 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_product__0_n_96),
        .Q(D[9]),
        .R(1'b0));
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-10 {cell *THIS*} {string 18x17 4}}" *) 
  DSP48E1 #(
    .ACASCREG(0),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(0),
    .AUTORESET_PATDET("NO_RESET"),
    .A_INPUT("CASCADE"),
    .BCASCREG(0),
    .BREG(0),
    .B_INPUT("DIRECT"),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .CREG(1),
    .DREG(1),
    .INMODEREG(0),
    .MASK(48'h3FFFFFFFFFFF),
    .MREG(0),
    .OPMODEREG(0),
    .PATTERN(48'h000000000000),
    .PREG(1),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_DPORT("FALSE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    buff0_reg__0
       (.A({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACIN({tmp_product__0_n_24,tmp_product__0_n_25,tmp_product__0_n_26,tmp_product__0_n_27,tmp_product__0_n_28,tmp_product__0_n_29,tmp_product__0_n_30,tmp_product__0_n_31,tmp_product__0_n_32,tmp_product__0_n_33,tmp_product__0_n_34,tmp_product__0_n_35,tmp_product__0_n_36,tmp_product__0_n_37,tmp_product__0_n_38,tmp_product__0_n_39,tmp_product__0_n_40,tmp_product__0_n_41,tmp_product__0_n_42,tmp_product__0_n_43,tmp_product__0_n_44,tmp_product__0_n_45,tmp_product__0_n_46,tmp_product__0_n_47,tmp_product__0_n_48,tmp_product__0_n_49,tmp_product__0_n_50,tmp_product__0_n_51,tmp_product__0_n_52,tmp_product__0_n_53}),
        .ACOUT(NLW_buff0_reg__0_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b0,1'b0,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0,1'b1,1'b1,1'b1}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_buff0_reg__0_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_buff0_reg__0_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_buff0_reg__0_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(1'b0),
        .CEA2(1'b0),
        .CEAD(1'b0),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(1'b0),
        .CEINMODE(1'b0),
        .CEM(1'b0),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .INMODE({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_buff0_reg__0_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b1,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_buff0_reg__0_OVERFLOW_UNCONNECTED),
        .P({buff0_reg__0_n_58,buff0_reg__0_n_59,buff0_reg__0_n_60,buff0_reg__0_n_61,buff0_reg__0_n_62,buff0_reg__0_n_63,buff0_reg__0_n_64,buff0_reg__0_n_65,buff0_reg__0_n_66,buff0_reg__0_n_67,buff0_reg__0_n_68,buff0_reg__0_n_69,buff0_reg__0_n_70,buff0_reg__0_n_71,buff0_reg__0_n_72,buff0_reg__0_n_73,buff0_reg__0_n_74,buff0_reg__0_n_75,buff0_reg__0_n_76,buff0_reg__0_n_77,buff0_reg__0_n_78,buff0_reg__0_n_79,buff0_reg__0_n_80,buff0_reg__0_n_81,buff0_reg__0_n_82,buff0_reg__0_n_83,buff0_reg__0_n_84,buff0_reg__0_n_85,buff0_reg__0_n_86,buff0_reg__0_n_87,buff0_reg__0_n_88,buff0_reg__0_n_89,buff0_reg__0_n_90,buff0_reg__0_n_91,buff0_reg__0_n_92,buff0_reg__0_n_93,buff0_reg__0_n_94,buff0_reg__0_n_95,buff0_reg__0_n_96,buff0_reg__0_n_97,buff0_reg__0_n_98,buff0_reg__0_n_99,buff0_reg__0_n_100,buff0_reg__0_n_101,buff0_reg__0_n_102,buff0_reg__0_n_103,buff0_reg__0_n_104,buff0_reg__0_n_105}),
        .PATTERNBDETECT(NLW_buff0_reg__0_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_buff0_reg__0_PATTERNDETECT_UNCONNECTED),
        .PCIN({tmp_product__0_n_106,tmp_product__0_n_107,tmp_product__0_n_108,tmp_product__0_n_109,tmp_product__0_n_110,tmp_product__0_n_111,tmp_product__0_n_112,tmp_product__0_n_113,tmp_product__0_n_114,tmp_product__0_n_115,tmp_product__0_n_116,tmp_product__0_n_117,tmp_product__0_n_118,tmp_product__0_n_119,tmp_product__0_n_120,tmp_product__0_n_121,tmp_product__0_n_122,tmp_product__0_n_123,tmp_product__0_n_124,tmp_product__0_n_125,tmp_product__0_n_126,tmp_product__0_n_127,tmp_product__0_n_128,tmp_product__0_n_129,tmp_product__0_n_130,tmp_product__0_n_131,tmp_product__0_n_132,tmp_product__0_n_133,tmp_product__0_n_134,tmp_product__0_n_135,tmp_product__0_n_136,tmp_product__0_n_137,tmp_product__0_n_138,tmp_product__0_n_139,tmp_product__0_n_140,tmp_product__0_n_141,tmp_product__0_n_142,tmp_product__0_n_143,tmp_product__0_n_144,tmp_product__0_n_145,tmp_product__0_n_146,tmp_product__0_n_147,tmp_product__0_n_148,tmp_product__0_n_149,tmp_product__0_n_150,tmp_product__0_n_151,tmp_product__0_n_152,tmp_product__0_n_153}),
        .PCOUT(NLW_buff0_reg__0_PCOUT_UNCONNECTED[47:0]),
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
        .UNDERFLOW(NLW_buff0_reg__0_UNDERFLOW_UNCONNECTED));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[19]_i_2 
       (.I0(buff0_reg__0_n_103),
        .I1(\buff0_reg_n_0_[2] ),
        .O(\mul_ln39_reg_304[19]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[19]_i_3 
       (.I0(buff0_reg__0_n_104),
        .I1(\buff0_reg_n_0_[1] ),
        .O(\mul_ln39_reg_304[19]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[19]_i_4 
       (.I0(buff0_reg__0_n_105),
        .I1(\buff0_reg_n_0_[0] ),
        .O(\mul_ln39_reg_304[19]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[23]_i_2 
       (.I0(buff0_reg__0_n_99),
        .I1(\buff0_reg_n_0_[6] ),
        .O(\mul_ln39_reg_304[23]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[23]_i_3 
       (.I0(buff0_reg__0_n_100),
        .I1(\buff0_reg_n_0_[5] ),
        .O(\mul_ln39_reg_304[23]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[23]_i_4 
       (.I0(buff0_reg__0_n_101),
        .I1(\buff0_reg_n_0_[4] ),
        .O(\mul_ln39_reg_304[23]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[23]_i_5 
       (.I0(buff0_reg__0_n_102),
        .I1(\buff0_reg_n_0_[3] ),
        .O(\mul_ln39_reg_304[23]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[27]_i_2 
       (.I0(buff0_reg__0_n_95),
        .I1(\buff0_reg_n_0_[10] ),
        .O(\mul_ln39_reg_304[27]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[27]_i_3 
       (.I0(buff0_reg__0_n_96),
        .I1(\buff0_reg_n_0_[9] ),
        .O(\mul_ln39_reg_304[27]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[27]_i_4 
       (.I0(buff0_reg__0_n_97),
        .I1(\buff0_reg_n_0_[8] ),
        .O(\mul_ln39_reg_304[27]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[27]_i_5 
       (.I0(buff0_reg__0_n_98),
        .I1(\buff0_reg_n_0_[7] ),
        .O(\mul_ln39_reg_304[27]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[31]_i_2 
       (.I0(buff0_reg__0_n_91),
        .I1(\buff0_reg_n_0_[14] ),
        .O(\mul_ln39_reg_304[31]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[31]_i_3 
       (.I0(buff0_reg__0_n_92),
        .I1(\buff0_reg_n_0_[13] ),
        .O(\mul_ln39_reg_304[31]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[31]_i_4 
       (.I0(buff0_reg__0_n_93),
        .I1(\buff0_reg_n_0_[12] ),
        .O(\mul_ln39_reg_304[31]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[31]_i_5 
       (.I0(buff0_reg__0_n_94),
        .I1(\buff0_reg_n_0_[11] ),
        .O(\mul_ln39_reg_304[31]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[35]_i_2 
       (.I0(buff0_reg__0_n_87),
        .I1(buff0_reg_n_104),
        .O(\mul_ln39_reg_304[35]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[35]_i_3 
       (.I0(buff0_reg__0_n_88),
        .I1(buff0_reg_n_105),
        .O(\mul_ln39_reg_304[35]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[35]_i_4 
       (.I0(buff0_reg__0_n_89),
        .I1(\buff0_reg_n_0_[16] ),
        .O(\mul_ln39_reg_304[35]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[35]_i_5 
       (.I0(buff0_reg__0_n_90),
        .I1(\buff0_reg_n_0_[15] ),
        .O(\mul_ln39_reg_304[35]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[39]_i_2 
       (.I0(buff0_reg__0_n_83),
        .I1(buff0_reg_n_100),
        .O(\mul_ln39_reg_304[39]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[39]_i_3 
       (.I0(buff0_reg__0_n_84),
        .I1(buff0_reg_n_101),
        .O(\mul_ln39_reg_304[39]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[39]_i_4 
       (.I0(buff0_reg__0_n_85),
        .I1(buff0_reg_n_102),
        .O(\mul_ln39_reg_304[39]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[39]_i_5 
       (.I0(buff0_reg__0_n_86),
        .I1(buff0_reg_n_103),
        .O(\mul_ln39_reg_304[39]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[41]_i_3 
       (.I0(buff0_reg__0_n_79),
        .I1(buff0_reg_n_96),
        .O(\mul_ln39_reg_304[41]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[41]_i_4 
       (.I0(buff0_reg__0_n_80),
        .I1(buff0_reg_n_97),
        .O(\mul_ln39_reg_304[41]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[41]_i_5 
       (.I0(buff0_reg__0_n_81),
        .I1(buff0_reg_n_98),
        .O(\mul_ln39_reg_304[41]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[41]_i_6 
       (.I0(buff0_reg__0_n_82),
        .I1(buff0_reg_n_99),
        .O(\mul_ln39_reg_304[41]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[44]_i_2 
       (.I0(buff0_reg__0_n_75),
        .I1(buff0_reg_n_92),
        .O(\mul_ln39_reg_304[44]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[44]_i_3 
       (.I0(buff0_reg__0_n_76),
        .I1(buff0_reg_n_93),
        .O(\mul_ln39_reg_304[44]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[44]_i_4 
       (.I0(buff0_reg__0_n_77),
        .I1(buff0_reg_n_94),
        .O(\mul_ln39_reg_304[44]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[44]_i_5 
       (.I0(buff0_reg__0_n_78),
        .I1(buff0_reg_n_95),
        .O(\mul_ln39_reg_304[44]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[48]_i_2 
       (.I0(buff0_reg__0_n_71),
        .I1(buff0_reg_n_88),
        .O(\mul_ln39_reg_304[48]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[48]_i_3 
       (.I0(buff0_reg__0_n_72),
        .I1(buff0_reg_n_89),
        .O(\mul_ln39_reg_304[48]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[48]_i_4 
       (.I0(buff0_reg__0_n_73),
        .I1(buff0_reg_n_90),
        .O(\mul_ln39_reg_304[48]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[48]_i_5 
       (.I0(buff0_reg__0_n_74),
        .I1(buff0_reg_n_91),
        .O(\mul_ln39_reg_304[48]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[52]_i_2 
       (.I0(buff0_reg__0_n_67),
        .I1(buff0_reg_n_84),
        .O(\mul_ln39_reg_304[52]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[52]_i_3 
       (.I0(buff0_reg__0_n_68),
        .I1(buff0_reg_n_85),
        .O(\mul_ln39_reg_304[52]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[52]_i_4 
       (.I0(buff0_reg__0_n_69),
        .I1(buff0_reg_n_86),
        .O(\mul_ln39_reg_304[52]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \mul_ln39_reg_304[52]_i_5 
       (.I0(buff0_reg__0_n_70),
        .I1(buff0_reg_n_87),
        .O(\mul_ln39_reg_304[52]_i_5_n_0 ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[19]_i_1 
       (.CI(1'b0),
        .CO({\mul_ln39_reg_304_reg[19]_i_1_n_0 ,\mul_ln39_reg_304_reg[19]_i_1_n_1 ,\mul_ln39_reg_304_reg[19]_i_1_n_2 ,\mul_ln39_reg_304_reg[19]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_103,buff0_reg__0_n_104,buff0_reg__0_n_105,1'b0}),
        .O(D[19:16]),
        .S({\mul_ln39_reg_304[19]_i_2_n_0 ,\mul_ln39_reg_304[19]_i_3_n_0 ,\mul_ln39_reg_304[19]_i_4_n_0 ,\buff0_reg[16]__0_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[23]_i_1 
       (.CI(\mul_ln39_reg_304_reg[19]_i_1_n_0 ),
        .CO({\mul_ln39_reg_304_reg[23]_i_1_n_0 ,\mul_ln39_reg_304_reg[23]_i_1_n_1 ,\mul_ln39_reg_304_reg[23]_i_1_n_2 ,\mul_ln39_reg_304_reg[23]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_99,buff0_reg__0_n_100,buff0_reg__0_n_101,buff0_reg__0_n_102}),
        .O(D[23:20]),
        .S({\mul_ln39_reg_304[23]_i_2_n_0 ,\mul_ln39_reg_304[23]_i_3_n_0 ,\mul_ln39_reg_304[23]_i_4_n_0 ,\mul_ln39_reg_304[23]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[27]_i_1 
       (.CI(\mul_ln39_reg_304_reg[23]_i_1_n_0 ),
        .CO({\mul_ln39_reg_304_reg[27]_i_1_n_0 ,\mul_ln39_reg_304_reg[27]_i_1_n_1 ,\mul_ln39_reg_304_reg[27]_i_1_n_2 ,\mul_ln39_reg_304_reg[27]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_95,buff0_reg__0_n_96,buff0_reg__0_n_97,buff0_reg__0_n_98}),
        .O(D[27:24]),
        .S({\mul_ln39_reg_304[27]_i_2_n_0 ,\mul_ln39_reg_304[27]_i_3_n_0 ,\mul_ln39_reg_304[27]_i_4_n_0 ,\mul_ln39_reg_304[27]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[31]_i_1 
       (.CI(\mul_ln39_reg_304_reg[27]_i_1_n_0 ),
        .CO({\mul_ln39_reg_304_reg[31]_i_1_n_0 ,\mul_ln39_reg_304_reg[31]_i_1_n_1 ,\mul_ln39_reg_304_reg[31]_i_1_n_2 ,\mul_ln39_reg_304_reg[31]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_91,buff0_reg__0_n_92,buff0_reg__0_n_93,buff0_reg__0_n_94}),
        .O(D[31:28]),
        .S({\mul_ln39_reg_304[31]_i_2_n_0 ,\mul_ln39_reg_304[31]_i_3_n_0 ,\mul_ln39_reg_304[31]_i_4_n_0 ,\mul_ln39_reg_304[31]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[35]_i_1 
       (.CI(\mul_ln39_reg_304_reg[31]_i_1_n_0 ),
        .CO({\mul_ln39_reg_304_reg[35]_i_1_n_0 ,\mul_ln39_reg_304_reg[35]_i_1_n_1 ,\mul_ln39_reg_304_reg[35]_i_1_n_2 ,\mul_ln39_reg_304_reg[35]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_87,buff0_reg__0_n_88,buff0_reg__0_n_89,buff0_reg__0_n_90}),
        .O(D[35:32]),
        .S({\mul_ln39_reg_304[35]_i_2_n_0 ,\mul_ln39_reg_304[35]_i_3_n_0 ,\mul_ln39_reg_304[35]_i_4_n_0 ,\mul_ln39_reg_304[35]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[39]_i_1 
       (.CI(\mul_ln39_reg_304_reg[35]_i_1_n_0 ),
        .CO({\mul_ln39_reg_304_reg[39]_i_1_n_0 ,\mul_ln39_reg_304_reg[39]_i_1_n_1 ,\mul_ln39_reg_304_reg[39]_i_1_n_2 ,\mul_ln39_reg_304_reg[39]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_83,buff0_reg__0_n_84,buff0_reg__0_n_85,buff0_reg__0_n_86}),
        .O(D[39:36]),
        .S({\mul_ln39_reg_304[39]_i_2_n_0 ,\mul_ln39_reg_304[39]_i_3_n_0 ,\mul_ln39_reg_304[39]_i_4_n_0 ,\mul_ln39_reg_304[39]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[41]_i_2 
       (.CI(\mul_ln39_reg_304_reg[39]_i_1_n_0 ),
        .CO({\mul_ln39_reg_304_reg[41]_i_2_n_0 ,\mul_ln39_reg_304_reg[41]_i_2_n_1 ,\mul_ln39_reg_304_reg[41]_i_2_n_2 ,\mul_ln39_reg_304_reg[41]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_79,buff0_reg__0_n_80,buff0_reg__0_n_81,buff0_reg__0_n_82}),
        .O({buff0_reg__0_0[1:0],D[41:40]}),
        .S({\mul_ln39_reg_304[41]_i_3_n_0 ,\mul_ln39_reg_304[41]_i_4_n_0 ,\mul_ln39_reg_304[41]_i_5_n_0 ,\mul_ln39_reg_304[41]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[44]_i_1 
       (.CI(\mul_ln39_reg_304_reg[41]_i_2_n_0 ),
        .CO({\mul_ln39_reg_304_reg[44]_i_1_n_0 ,\mul_ln39_reg_304_reg[44]_i_1_n_1 ,\mul_ln39_reg_304_reg[44]_i_1_n_2 ,\mul_ln39_reg_304_reg[44]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_75,buff0_reg__0_n_76,buff0_reg__0_n_77,buff0_reg__0_n_78}),
        .O(buff0_reg__0_0[5:2]),
        .S({\mul_ln39_reg_304[44]_i_2_n_0 ,\mul_ln39_reg_304[44]_i_3_n_0 ,\mul_ln39_reg_304[44]_i_4_n_0 ,\mul_ln39_reg_304[44]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[48]_i_1 
       (.CI(\mul_ln39_reg_304_reg[44]_i_1_n_0 ),
        .CO({\mul_ln39_reg_304_reg[48]_i_1_n_0 ,\mul_ln39_reg_304_reg[48]_i_1_n_1 ,\mul_ln39_reg_304_reg[48]_i_1_n_2 ,\mul_ln39_reg_304_reg[48]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_71,buff0_reg__0_n_72,buff0_reg__0_n_73,buff0_reg__0_n_74}),
        .O(buff0_reg__0_0[9:6]),
        .S({\mul_ln39_reg_304[48]_i_2_n_0 ,\mul_ln39_reg_304[48]_i_3_n_0 ,\mul_ln39_reg_304[48]_i_4_n_0 ,\mul_ln39_reg_304[48]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \mul_ln39_reg_304_reg[52]_i_1 
       (.CI(\mul_ln39_reg_304_reg[48]_i_1_n_0 ),
        .CO({\mul_ln39_reg_304_reg[52]_i_1_n_0 ,\mul_ln39_reg_304_reg[52]_i_1_n_1 ,\mul_ln39_reg_304_reg[52]_i_1_n_2 ,\mul_ln39_reg_304_reg[52]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({buff0_reg__0_n_67,buff0_reg__0_n_68,buff0_reg__0_n_69,buff0_reg__0_n_70}),
        .O(buff0_reg__0_0[13:10]),
        .S({\mul_ln39_reg_304[52]_i_2_n_0 ,\mul_ln39_reg_304[52]_i_3_n_0 ,\mul_ln39_reg_304[52]_i_4_n_0 ,\mul_ln39_reg_304[52]_i_5_n_0 }));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp_2_reg_309[15]_i_2 
       (.I0(buff0_reg__0_n_65),
        .I1(buff0_reg_n_82),
        .O(\tmp_2_reg_309[15]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp_2_reg_309[15]_i_3 
       (.I0(buff0_reg__0_n_66),
        .I1(buff0_reg_n_83),
        .O(\tmp_2_reg_309[15]_i_3_n_0 ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \tmp_2_reg_309_reg[15]_i_1 
       (.CI(\mul_ln39_reg_304_reg[52]_i_1_n_0 ),
        .CO({\NLW_tmp_2_reg_309_reg[15]_i_1_CO_UNCONNECTED [3:1],\tmp_2_reg_309_reg[15]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,buff0_reg__0_n_66}),
        .O({\NLW_tmp_2_reg_309_reg[15]_i_1_O_UNCONNECTED [3:2],buff0_reg__0_0[15:14]}),
        .S({1'b0,1'b0,\tmp_2_reg_309[15]_i_2_n_0 ,\tmp_2_reg_309[15]_i_3_n_0 }));
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-10 {cell *THIS*} {string 15x18 4}}" *) 
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
    .MREG(0),
    .OPMODEREG(0),
    .PATTERN(48'h000000000000),
    .PREG(0),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_DPORT("FALSE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    tmp_product
       (.A({add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31],add_ln32_fu_145_p2[31:17]}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_tmp_product_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b0,1'b0,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b1}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_tmp_product_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_tmp_product_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_tmp_product_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(1'b0),
        .CEA2(add_ln32_reg_2830),
        .CEAD(1'b0),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(1'b0),
        .CEINMODE(1'b0),
        .CEM(1'b0),
        .CEP(1'b0),
        .CLK(ap_clk),
        .D({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .INMODE({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_tmp_product_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_tmp_product_OVERFLOW_UNCONNECTED),
        .P({tmp_product_n_58,tmp_product_n_59,tmp_product_n_60,tmp_product_n_61,tmp_product_n_62,tmp_product_n_63,tmp_product_n_64,tmp_product_n_65,tmp_product_n_66,tmp_product_n_67,tmp_product_n_68,tmp_product_n_69,tmp_product_n_70,tmp_product_n_71,tmp_product_n_72,tmp_product_n_73,tmp_product_n_74,tmp_product_n_75,tmp_product_n_76,tmp_product_n_77,tmp_product_n_78,tmp_product_n_79,tmp_product_n_80,tmp_product_n_81,tmp_product_n_82,tmp_product_n_83,tmp_product_n_84,tmp_product_n_85,tmp_product_n_86,tmp_product_n_87,tmp_product_n_88,tmp_product_n_89,tmp_product_n_90,tmp_product_n_91,tmp_product_n_92,tmp_product_n_93,tmp_product_n_94,tmp_product_n_95,tmp_product_n_96,tmp_product_n_97,tmp_product_n_98,tmp_product_n_99,tmp_product_n_100,tmp_product_n_101,tmp_product_n_102,tmp_product_n_103,tmp_product_n_104,tmp_product_n_105}),
        .PATTERNBDETECT(NLW_tmp_product_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_tmp_product_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT({tmp_product_n_106,tmp_product_n_107,tmp_product_n_108,tmp_product_n_109,tmp_product_n_110,tmp_product_n_111,tmp_product_n_112,tmp_product_n_113,tmp_product_n_114,tmp_product_n_115,tmp_product_n_116,tmp_product_n_117,tmp_product_n_118,tmp_product_n_119,tmp_product_n_120,tmp_product_n_121,tmp_product_n_122,tmp_product_n_123,tmp_product_n_124,tmp_product_n_125,tmp_product_n_126,tmp_product_n_127,tmp_product_n_128,tmp_product_n_129,tmp_product_n_130,tmp_product_n_131,tmp_product_n_132,tmp_product_n_133,tmp_product_n_134,tmp_product_n_135,tmp_product_n_136,tmp_product_n_137,tmp_product_n_138,tmp_product_n_139,tmp_product_n_140,tmp_product_n_141,tmp_product_n_142,tmp_product_n_143,tmp_product_n_144,tmp_product_n_145,tmp_product_n_146,tmp_product_n_147,tmp_product_n_148,tmp_product_n_149,tmp_product_n_150,tmp_product_n_151,tmp_product_n_152,tmp_product_n_153}),
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
        .UNDERFLOW(NLW_tmp_product_UNDERFLOW_UNCONNECTED));
  (* METHODOLOGY_DRC_VIOS = "{SYNTH-10 {cell *THIS*} {string 18x18 4}}" *) 
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
    .MREG(0),
    .OPMODEREG(0),
    .PATTERN(48'h000000000000),
    .PREG(0),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_DPORT("FALSE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    tmp_product__0
       (.A({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,add_ln32_fu_145_p2[16:0]}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT({tmp_product__0_n_24,tmp_product__0_n_25,tmp_product__0_n_26,tmp_product__0_n_27,tmp_product__0_n_28,tmp_product__0_n_29,tmp_product__0_n_30,tmp_product__0_n_31,tmp_product__0_n_32,tmp_product__0_n_33,tmp_product__0_n_34,tmp_product__0_n_35,tmp_product__0_n_36,tmp_product__0_n_37,tmp_product__0_n_38,tmp_product__0_n_39,tmp_product__0_n_40,tmp_product__0_n_41,tmp_product__0_n_42,tmp_product__0_n_43,tmp_product__0_n_44,tmp_product__0_n_45,tmp_product__0_n_46,tmp_product__0_n_47,tmp_product__0_n_48,tmp_product__0_n_49,tmp_product__0_n_50,tmp_product__0_n_51,tmp_product__0_n_52,tmp_product__0_n_53}),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b0,1'b0,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b1}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_tmp_product__0_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_tmp_product__0_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_tmp_product__0_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(1'b0),
        .CEA2(add_ln32_reg_2830),
        .CEAD(1'b0),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(1'b0),
        .CEINMODE(1'b0),
        .CEM(1'b0),
        .CEP(1'b0),
        .CLK(ap_clk),
        .D({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .INMODE({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_tmp_product__0_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_tmp_product__0_OVERFLOW_UNCONNECTED),
        .P({tmp_product__0_n_58,tmp_product__0_n_59,tmp_product__0_n_60,tmp_product__0_n_61,tmp_product__0_n_62,tmp_product__0_n_63,tmp_product__0_n_64,tmp_product__0_n_65,tmp_product__0_n_66,tmp_product__0_n_67,tmp_product__0_n_68,tmp_product__0_n_69,tmp_product__0_n_70,tmp_product__0_n_71,tmp_product__0_n_72,tmp_product__0_n_73,tmp_product__0_n_74,tmp_product__0_n_75,tmp_product__0_n_76,tmp_product__0_n_77,tmp_product__0_n_78,tmp_product__0_n_79,tmp_product__0_n_80,tmp_product__0_n_81,tmp_product__0_n_82,tmp_product__0_n_83,tmp_product__0_n_84,tmp_product__0_n_85,tmp_product__0_n_86,tmp_product__0_n_87,tmp_product__0_n_88,tmp_product__0_n_89,tmp_product__0_n_90,tmp_product__0_n_91,tmp_product__0_n_92,tmp_product__0_n_93,tmp_product__0_n_94,tmp_product__0_n_95,tmp_product__0_n_96,tmp_product__0_n_97,tmp_product__0_n_98,tmp_product__0_n_99,tmp_product__0_n_100,tmp_product__0_n_101,tmp_product__0_n_102,tmp_product__0_n_103,tmp_product__0_n_104,tmp_product__0_n_105}),
        .PATTERNBDETECT(NLW_tmp_product__0_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_tmp_product__0_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT({tmp_product__0_n_106,tmp_product__0_n_107,tmp_product__0_n_108,tmp_product__0_n_109,tmp_product__0_n_110,tmp_product__0_n_111,tmp_product__0_n_112,tmp_product__0_n_113,tmp_product__0_n_114,tmp_product__0_n_115,tmp_product__0_n_116,tmp_product__0_n_117,tmp_product__0_n_118,tmp_product__0_n_119,tmp_product__0_n_120,tmp_product__0_n_121,tmp_product__0_n_122,tmp_product__0_n_123,tmp_product__0_n_124,tmp_product__0_n_125,tmp_product__0_n_126,tmp_product__0_n_127,tmp_product__0_n_128,tmp_product__0_n_129,tmp_product__0_n_130,tmp_product__0_n_131,tmp_product__0_n_132,tmp_product__0_n_133,tmp_product__0_n_134,tmp_product__0_n_135,tmp_product__0_n_136,tmp_product__0_n_137,tmp_product__0_n_138,tmp_product__0_n_139,tmp_product__0_n_140,tmp_product__0_n_141,tmp_product__0_n_142,tmp_product__0_n_143,tmp_product__0_n_144,tmp_product__0_n_145,tmp_product__0_n_146,tmp_product__0_n_147,tmp_product__0_n_148,tmp_product__0_n_149,tmp_product__0_n_150,tmp_product__0_n_151,tmp_product__0_n_152,tmp_product__0_n_153}),
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
        .UNDERFLOW(NLW_tmp_product__0_UNDERFLOW_UNCONNECTED));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_10
       (.I0(sum_reg[4]),
        .I1(sum_reg[5]),
        .O(\sum_reg[22] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_11
       (.I0(sum_reg[3]),
        .I1(sum_reg[4]),
        .O(\sum_reg[22] [0]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_13
       (.I0(sum_reg[2]),
        .I1(sum_reg[3]),
        .O(\sum_reg[18] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_14
       (.I0(sum_reg[1]),
        .I1(sum_reg[2]),
        .O(\sum_reg[18] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_15
       (.I0(sum_reg[0]),
        .I1(sum_reg[1]),
        .O(\sum_reg[18] [0]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_4
       (.I0(sum_reg[10]),
        .I1(sum_reg[11]),
        .O(\sum_reg[26] [3]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_5
       (.I0(sum_reg[9]),
        .I1(sum_reg[10]),
        .O(\sum_reg[26] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_6
       (.I0(sum_reg[8]),
        .I1(sum_reg[9]),
        .O(\sum_reg[26] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_7
       (.I0(sum_reg[7]),
        .I1(sum_reg[8]),
        .O(\sum_reg[26] [0]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_8
       (.I0(sum_reg[6]),
        .I1(sum_reg[7]),
        .O(\sum_reg[22] [3]));
  LUT2 #(
    .INIT(4'h9)) 
    tmp_product_i_9
       (.I0(sum_reg[5]),
        .I1(sum_reg[6]),
        .O(\sum_reg[22] [2]));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both
   (\B_V_data_1_state_reg[1]_0 ,
    ap_rst_n_inv,
    in_stream_TVALID_int_regslice,
    in,
    O,
    \sum_reg[7] ,
    \sum_reg[11] ,
    \sum_reg[14] ,
    \sum_reg[14]_0 ,
    \B_V_data_1_payload_B_reg[15]_0 ,
    \B_V_data_1_payload_B_reg[15]_1 ,
    \B_V_data_1_payload_B_reg[15]_2 ,
    add_ln32_fu_145_p2,
    ap_clk,
    \B_V_data_1_state_reg[1]_1 ,
    in_stream_TVALID,
    ap_rst_n,
    sum_reg,
    tmp_product,
    tmp_product_0,
    tmp_product_1,
    S,
    in_stream_TDATA);
  output \B_V_data_1_state_reg[1]_0 ;
  output ap_rst_n_inv;
  output in_stream_TVALID_int_regslice;
  output [15:0]in;
  output [3:0]O;
  output [3:0]\sum_reg[7] ;
  output [3:0]\sum_reg[11] ;
  output [3:0]\sum_reg[14] ;
  output [3:0]\sum_reg[14]_0 ;
  output [3:0]\B_V_data_1_payload_B_reg[15]_0 ;
  output [3:0]\B_V_data_1_payload_B_reg[15]_1 ;
  output [3:0]\B_V_data_1_payload_B_reg[15]_2 ;
  output [31:0]add_ln32_fu_145_p2;
  input ap_clk;
  input \B_V_data_1_state_reg[1]_1 ;
  input in_stream_TVALID;
  input ap_rst_n;
  input [31:0]sum_reg;
  input [2:0]tmp_product;
  input [3:0]tmp_product_0;
  input [3:0]tmp_product_1;
  input [3:0]S;
  input [15:0]in_stream_TDATA;

  wire B_V_data_1_load_A;
  wire B_V_data_1_load_B;
  wire \B_V_data_1_payload_A_reg_n_0_[0] ;
  wire \B_V_data_1_payload_A_reg_n_0_[10] ;
  wire \B_V_data_1_payload_A_reg_n_0_[11] ;
  wire \B_V_data_1_payload_A_reg_n_0_[12] ;
  wire \B_V_data_1_payload_A_reg_n_0_[13] ;
  wire \B_V_data_1_payload_A_reg_n_0_[14] ;
  wire \B_V_data_1_payload_A_reg_n_0_[15] ;
  wire \B_V_data_1_payload_A_reg_n_0_[1] ;
  wire \B_V_data_1_payload_A_reg_n_0_[2] ;
  wire \B_V_data_1_payload_A_reg_n_0_[3] ;
  wire \B_V_data_1_payload_A_reg_n_0_[4] ;
  wire \B_V_data_1_payload_A_reg_n_0_[5] ;
  wire \B_V_data_1_payload_A_reg_n_0_[6] ;
  wire \B_V_data_1_payload_A_reg_n_0_[7] ;
  wire \B_V_data_1_payload_A_reg_n_0_[8] ;
  wire \B_V_data_1_payload_A_reg_n_0_[9] ;
  wire [3:0]\B_V_data_1_payload_B_reg[15]_0 ;
  wire [3:0]\B_V_data_1_payload_B_reg[15]_1 ;
  wire [3:0]\B_V_data_1_payload_B_reg[15]_2 ;
  wire \B_V_data_1_payload_B_reg_n_0_[0] ;
  wire \B_V_data_1_payload_B_reg_n_0_[10] ;
  wire \B_V_data_1_payload_B_reg_n_0_[11] ;
  wire \B_V_data_1_payload_B_reg_n_0_[12] ;
  wire \B_V_data_1_payload_B_reg_n_0_[13] ;
  wire \B_V_data_1_payload_B_reg_n_0_[14] ;
  wire \B_V_data_1_payload_B_reg_n_0_[15] ;
  wire \B_V_data_1_payload_B_reg_n_0_[1] ;
  wire \B_V_data_1_payload_B_reg_n_0_[2] ;
  wire \B_V_data_1_payload_B_reg_n_0_[3] ;
  wire \B_V_data_1_payload_B_reg_n_0_[4] ;
  wire \B_V_data_1_payload_B_reg_n_0_[5] ;
  wire \B_V_data_1_payload_B_reg_n_0_[6] ;
  wire \B_V_data_1_payload_B_reg_n_0_[7] ;
  wire \B_V_data_1_payload_B_reg_n_0_[8] ;
  wire \B_V_data_1_payload_B_reg_n_0_[9] ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__2_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__3_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__3_n_0 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire \B_V_data_1_state_reg[1]_1 ;
  wire [3:0]O;
  wire [3:0]S;
  wire [31:0]add_ln32_fu_145_p2;
  wire \add_ln32_reg_283_reg[31]_i_2_n_1 ;
  wire \add_ln32_reg_283_reg[31]_i_2_n_2 ;
  wire \add_ln32_reg_283_reg[31]_i_2_n_3 ;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [15:0]in;
  wire [15:0]in_stream_TDATA;
  wire in_stream_TVALID;
  wire in_stream_TVALID_int_regslice;
  wire \sum[0]_i_2_n_0 ;
  wire \sum[0]_i_3_n_0 ;
  wire \sum[0]_i_4_n_0 ;
  wire \sum[0]_i_5_n_0 ;
  wire \sum[12]_i_2_n_0 ;
  wire \sum[12]_i_3_n_0 ;
  wire \sum[12]_i_4_n_0 ;
  wire \sum[12]_i_5_n_0 ;
  wire \sum[12]_i_6_n_0 ;
  wire \sum[16]_i_2_n_0 ;
  wire \sum[16]_i_3_n_0 ;
  wire \sum[16]_i_4_n_0 ;
  wire \sum[16]_i_5_n_0 ;
  wire \sum[16]_i_6_n_0 ;
  wire \sum[16]_i_7_n_0 ;
  wire \sum[16]_i_8_n_0 ;
  wire \sum[16]_i_9_n_0 ;
  wire \sum[20]_i_2_n_0 ;
  wire \sum[20]_i_3_n_0 ;
  wire \sum[20]_i_4_n_0 ;
  wire \sum[20]_i_5_n_0 ;
  wire \sum[20]_i_6_n_0 ;
  wire \sum[20]_i_7_n_0 ;
  wire \sum[20]_i_8_n_0 ;
  wire \sum[20]_i_9_n_0 ;
  wire \sum[24]_i_2_n_0 ;
  wire \sum[24]_i_3_n_0 ;
  wire \sum[24]_i_4_n_0 ;
  wire \sum[24]_i_5_n_0 ;
  wire \sum[24]_i_6_n_0 ;
  wire \sum[24]_i_7_n_0 ;
  wire \sum[24]_i_8_n_0 ;
  wire \sum[24]_i_9_n_0 ;
  wire \sum[28]_i_2_n_0 ;
  wire \sum[28]_i_3_n_0 ;
  wire \sum[28]_i_4_n_0 ;
  wire \sum[28]_i_5_n_0 ;
  wire \sum[28]_i_6_n_0 ;
  wire \sum[28]_i_7_n_0 ;
  wire \sum[28]_i_8_n_0 ;
  wire \sum[4]_i_2_n_0 ;
  wire \sum[4]_i_3_n_0 ;
  wire \sum[4]_i_4_n_0 ;
  wire \sum[4]_i_5_n_0 ;
  wire \sum[8]_i_2_n_0 ;
  wire \sum[8]_i_3_n_0 ;
  wire \sum[8]_i_4_n_0 ;
  wire \sum[8]_i_5_n_0 ;
  wire [31:0]sum_reg;
  wire \sum_reg[0]_i_1_n_0 ;
  wire \sum_reg[0]_i_1_n_1 ;
  wire \sum_reg[0]_i_1_n_2 ;
  wire \sum_reg[0]_i_1_n_3 ;
  wire [3:0]\sum_reg[11] ;
  wire \sum_reg[12]_i_1_n_0 ;
  wire \sum_reg[12]_i_1_n_1 ;
  wire \sum_reg[12]_i_1_n_2 ;
  wire \sum_reg[12]_i_1_n_3 ;
  wire [3:0]\sum_reg[14] ;
  wire [3:0]\sum_reg[14]_0 ;
  wire \sum_reg[16]_i_1_n_0 ;
  wire \sum_reg[16]_i_1_n_1 ;
  wire \sum_reg[16]_i_1_n_2 ;
  wire \sum_reg[16]_i_1_n_3 ;
  wire \sum_reg[20]_i_1_n_0 ;
  wire \sum_reg[20]_i_1_n_1 ;
  wire \sum_reg[20]_i_1_n_2 ;
  wire \sum_reg[20]_i_1_n_3 ;
  wire \sum_reg[24]_i_1_n_0 ;
  wire \sum_reg[24]_i_1_n_1 ;
  wire \sum_reg[24]_i_1_n_2 ;
  wire \sum_reg[24]_i_1_n_3 ;
  wire \sum_reg[28]_i_1_n_1 ;
  wire \sum_reg[28]_i_1_n_2 ;
  wire \sum_reg[28]_i_1_n_3 ;
  wire \sum_reg[4]_i_1_n_0 ;
  wire \sum_reg[4]_i_1_n_1 ;
  wire \sum_reg[4]_i_1_n_2 ;
  wire \sum_reg[4]_i_1_n_3 ;
  wire [3:0]\sum_reg[7] ;
  wire \sum_reg[8]_i_1_n_0 ;
  wire \sum_reg[8]_i_1_n_1 ;
  wire \sum_reg[8]_i_1_n_2 ;
  wire \sum_reg[8]_i_1_n_3 ;
  wire [2:0]tmp_product;
  wire [3:0]tmp_product_0;
  wire [3:0]tmp_product_1;
  wire tmp_product__0_i_10_n_0;
  wire tmp_product__0_i_11_n_0;
  wire tmp_product__0_i_12_n_0;
  wire tmp_product__0_i_13_n_0;
  wire tmp_product__0_i_14_n_0;
  wire tmp_product__0_i_15_n_0;
  wire tmp_product__0_i_16_n_0;
  wire tmp_product__0_i_17_n_0;
  wire tmp_product__0_i_18_n_0;
  wire tmp_product__0_i_19_n_0;
  wire tmp_product__0_i_1_n_0;
  wire tmp_product__0_i_1_n_1;
  wire tmp_product__0_i_1_n_2;
  wire tmp_product__0_i_1_n_3;
  wire tmp_product__0_i_20_n_0;
  wire tmp_product__0_i_21_n_0;
  wire tmp_product__0_i_2_n_0;
  wire tmp_product__0_i_2_n_1;
  wire tmp_product__0_i_2_n_2;
  wire tmp_product__0_i_2_n_3;
  wire tmp_product__0_i_3_n_0;
  wire tmp_product__0_i_3_n_1;
  wire tmp_product__0_i_3_n_2;
  wire tmp_product__0_i_3_n_3;
  wire tmp_product__0_i_4_n_0;
  wire tmp_product__0_i_4_n_1;
  wire tmp_product__0_i_4_n_2;
  wire tmp_product__0_i_4_n_3;
  wire tmp_product__0_i_5_n_0;
  wire tmp_product__0_i_6_n_0;
  wire tmp_product__0_i_7_n_0;
  wire tmp_product__0_i_8_n_0;
  wire tmp_product__0_i_9_n_0;
  wire tmp_product_i_12_n_0;
  wire tmp_product_i_16_n_0;
  wire tmp_product_i_1_n_0;
  wire tmp_product_i_1_n_1;
  wire tmp_product_i_1_n_2;
  wire tmp_product_i_1_n_3;
  wire tmp_product_i_2_n_0;
  wire tmp_product_i_2_n_1;
  wire tmp_product_i_2_n_2;
  wire tmp_product_i_2_n_3;
  wire tmp_product_i_3_n_0;
  wire tmp_product_i_3_n_1;
  wire tmp_product_i_3_n_2;
  wire tmp_product_i_3_n_3;
  wire [3:3]\NLW_add_ln32_reg_283_reg[31]_i_2_CO_UNCONNECTED ;
  wire [3:3]\NLW_sum_reg[28]_i_1_CO_UNCONNECTED ;

  LUT3 #(
    .INIT(8'h0D)) 
    \B_V_data_1_payload_A[15]_i_1 
       (.I0(in_stream_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[0]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[10]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[11]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[12]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[13]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[14]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[15] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[15]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[1]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[3]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[4]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[5]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[6]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[7]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[8]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TDATA[9]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .R(1'b0));
  LUT3 #(
    .INIT(8'hA2)) 
    \B_V_data_1_payload_B[15]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(in_stream_TVALID_int_regslice),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[0]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[10]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[11]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[12]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[13]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[14]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[15] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[15]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[1]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[3]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[4]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[5]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[6]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[7]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[8]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[9]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT3 #(
    .INIT(8'hB4)) 
    B_V_data_1_sel_rd_i_1__2
       (.I0(\B_V_data_1_state_reg[1]_1 ),
        .I1(in_stream_TVALID_int_regslice),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__2_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__2_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__3
       (.I0(in_stream_TVALID),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__3_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__3_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hAAA080A0)) 
    \B_V_data_1_state[0]_i_1__3 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg[1]_1 ),
        .I2(in_stream_TVALID_int_regslice),
        .I3(\B_V_data_1_state_reg[1]_0 ),
        .I4(in_stream_TVALID),
        .O(\B_V_data_1_state[0]_i_1__3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_state[1]_i_1__6 
       (.I0(ap_rst_n),
        .O(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT4 #(
    .INIT(16'h77F7)) 
    \B_V_data_1_state[1]_i_2 
       (.I0(\B_V_data_1_state_reg[1]_1 ),
        .I1(in_stream_TVALID_int_regslice),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(in_stream_TVALID),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__3_n_0 ),
        .Q(in_stream_TVALID_int_regslice),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg[1]_0 ),
        .R(ap_rst_n_inv));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln32_reg_283_reg[31]_i_2 
       (.CI(tmp_product_i_1_n_0),
        .CO({\NLW_add_ln32_reg_283_reg[31]_i_2_CO_UNCONNECTED [3],\add_ln32_reg_283_reg[31]_i_2_n_1 ,\add_ln32_reg_283_reg[31]_i_2_n_2 ,\add_ln32_reg_283_reg[31]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,sum_reg[29:27]}),
        .O(add_ln32_fu_145_p2[31:28]),
        .S(S));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[0]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I3(sum_reg[3]),
        .O(\sum[0]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[0]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I3(sum_reg[2]),
        .O(\sum[0]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[0]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I3(sum_reg[1]),
        .O(\sum[0]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[0]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I3(sum_reg[0]),
        .O(\sum[0]_i_5_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[12]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[12]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[12]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[15]),
        .O(\sum[12]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[12]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I3(sum_reg[14]),
        .O(\sum[12]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[12]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I3(sum_reg[13]),
        .O(\sum[12]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[12]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I3(sum_reg[12]),
        .O(\sum[12]_i_6_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[16]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[16]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[16]_i_3 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[16]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[16]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[16]_i_4_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[16]_i_5 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[16]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[16]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[19]),
        .O(\sum[16]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[16]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[18]),
        .O(\sum[16]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[16]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[17]),
        .O(\sum[16]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[16]_i_9 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[16]),
        .O(\sum[16]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[20]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[20]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[20]_i_3 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[20]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[20]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[20]_i_4_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[20]_i_5 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[20]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[20]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[23]),
        .O(\sum[20]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[20]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[22]),
        .O(\sum[20]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[20]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[21]),
        .O(\sum[20]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[20]_i_9 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[20]),
        .O(\sum[20]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[24]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[24]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[24]_i_3 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[24]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[24]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[24]_i_4_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[24]_i_5 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[24]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[24]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[27]),
        .O(\sum[24]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[24]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[26]),
        .O(\sum[24]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[24]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[25]),
        .O(\sum[24]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[24]_i_9 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[24]),
        .O(\sum[24]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[28]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[28]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[28]_i_3 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[28]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \sum[28]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\sum[28]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[28]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[31]),
        .O(\sum[28]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[28]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[30]),
        .O(\sum[28]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[28]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[29]),
        .O(\sum[28]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[28]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[28]),
        .O(\sum[28]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[4]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I3(sum_reg[7]),
        .O(\sum[4]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[4]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I3(sum_reg[6]),
        .O(\sum[4]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[4]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I3(sum_reg[5]),
        .O(\sum[4]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[4]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I3(sum_reg[4]),
        .O(\sum[4]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[8]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I3(sum_reg[11]),
        .O(\sum[8]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[8]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I3(sum_reg[10]),
        .O(\sum[8]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[8]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I3(sum_reg[9]),
        .O(\sum[8]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \sum[8]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I3(sum_reg[8]),
        .O(\sum[8]_i_5_n_0 ));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_reg[0]_i_1 
       (.CI(1'b0),
        .CO({\sum_reg[0]_i_1_n_0 ,\sum_reg[0]_i_1_n_1 ,\sum_reg[0]_i_1_n_2 ,\sum_reg[0]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_reg[3:0]),
        .O(O),
        .S({\sum[0]_i_2_n_0 ,\sum[0]_i_3_n_0 ,\sum[0]_i_4_n_0 ,\sum[0]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_reg[12]_i_1 
       (.CI(\sum_reg[8]_i_1_n_0 ),
        .CO({\sum_reg[12]_i_1_n_0 ,\sum_reg[12]_i_1_n_1 ,\sum_reg[12]_i_1_n_2 ,\sum_reg[12]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\sum[12]_i_2_n_0 ,sum_reg[14:12]}),
        .O(\sum_reg[14] ),
        .S({\sum[12]_i_3_n_0 ,\sum[12]_i_4_n_0 ,\sum[12]_i_5_n_0 ,\sum[12]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_reg[16]_i_1 
       (.CI(\sum_reg[12]_i_1_n_0 ),
        .CO({\sum_reg[16]_i_1_n_0 ,\sum_reg[16]_i_1_n_1 ,\sum_reg[16]_i_1_n_2 ,\sum_reg[16]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\sum[16]_i_2_n_0 ,\sum[16]_i_3_n_0 ,\sum[16]_i_4_n_0 ,\sum[16]_i_5_n_0 }),
        .O(\sum_reg[14]_0 ),
        .S({\sum[16]_i_6_n_0 ,\sum[16]_i_7_n_0 ,\sum[16]_i_8_n_0 ,\sum[16]_i_9_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_reg[20]_i_1 
       (.CI(\sum_reg[16]_i_1_n_0 ),
        .CO({\sum_reg[20]_i_1_n_0 ,\sum_reg[20]_i_1_n_1 ,\sum_reg[20]_i_1_n_2 ,\sum_reg[20]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\sum[20]_i_2_n_0 ,\sum[20]_i_3_n_0 ,\sum[20]_i_4_n_0 ,\sum[20]_i_5_n_0 }),
        .O(\B_V_data_1_payload_B_reg[15]_0 ),
        .S({\sum[20]_i_6_n_0 ,\sum[20]_i_7_n_0 ,\sum[20]_i_8_n_0 ,\sum[20]_i_9_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_reg[24]_i_1 
       (.CI(\sum_reg[20]_i_1_n_0 ),
        .CO({\sum_reg[24]_i_1_n_0 ,\sum_reg[24]_i_1_n_1 ,\sum_reg[24]_i_1_n_2 ,\sum_reg[24]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\sum[24]_i_2_n_0 ,\sum[24]_i_3_n_0 ,\sum[24]_i_4_n_0 ,\sum[24]_i_5_n_0 }),
        .O(\B_V_data_1_payload_B_reg[15]_1 ),
        .S({\sum[24]_i_6_n_0 ,\sum[24]_i_7_n_0 ,\sum[24]_i_8_n_0 ,\sum[24]_i_9_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_reg[28]_i_1 
       (.CI(\sum_reg[24]_i_1_n_0 ),
        .CO({\NLW_sum_reg[28]_i_1_CO_UNCONNECTED [3],\sum_reg[28]_i_1_n_1 ,\sum_reg[28]_i_1_n_2 ,\sum_reg[28]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,\sum[28]_i_2_n_0 ,\sum[28]_i_3_n_0 ,\sum[28]_i_4_n_0 }),
        .O(\B_V_data_1_payload_B_reg[15]_2 ),
        .S({\sum[28]_i_5_n_0 ,\sum[28]_i_6_n_0 ,\sum[28]_i_7_n_0 ,\sum[28]_i_8_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_reg[4]_i_1 
       (.CI(\sum_reg[0]_i_1_n_0 ),
        .CO({\sum_reg[4]_i_1_n_0 ,\sum_reg[4]_i_1_n_1 ,\sum_reg[4]_i_1_n_2 ,\sum_reg[4]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_reg[7:4]),
        .O(\sum_reg[7] ),
        .S({\sum[4]_i_2_n_0 ,\sum[4]_i_3_n_0 ,\sum[4]_i_4_n_0 ,\sum[4]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_reg[8]_i_1 
       (.CI(\sum_reg[4]_i_1_n_0 ),
        .CO({\sum_reg[8]_i_1_n_0 ,\sum_reg[8]_i_1_n_1 ,\sum_reg[8]_i_1_n_2 ,\sum_reg[8]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_reg[11:8]),
        .O(\sum_reg[11] ),
        .S({\sum[8]_i_2_n_0 ,\sum[8]_i_3_n_0 ,\sum[8]_i_4_n_0 ,\sum[8]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 tmp_product__0_i_1
       (.CI(tmp_product__0_i_2_n_0),
        .CO({tmp_product__0_i_1_n_0,tmp_product__0_i_1_n_1,tmp_product__0_i_1_n_2,tmp_product__0_i_1_n_3}),
        .CYINIT(1'b0),
        .DI({tmp_product__0_i_5_n_0,sum_reg[14:12]}),
        .O(add_ln32_fu_145_p2[15:12]),
        .S({tmp_product__0_i_6_n_0,tmp_product__0_i_7_n_0,tmp_product__0_i_8_n_0,tmp_product__0_i_9_n_0}));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_10
       (.I0(sum_reg[11]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .O(tmp_product__0_i_10_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_11
       (.I0(sum_reg[10]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .O(tmp_product__0_i_11_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_12
       (.I0(sum_reg[9]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .O(tmp_product__0_i_12_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_13
       (.I0(sum_reg[8]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .O(tmp_product__0_i_13_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_14
       (.I0(sum_reg[7]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .O(tmp_product__0_i_14_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_15
       (.I0(sum_reg[6]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .O(tmp_product__0_i_15_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_16
       (.I0(sum_reg[5]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .O(tmp_product__0_i_16_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_17
       (.I0(sum_reg[4]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .O(tmp_product__0_i_17_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_18
       (.I0(sum_reg[3]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .O(tmp_product__0_i_18_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_19
       (.I0(sum_reg[2]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .O(tmp_product__0_i_19_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 tmp_product__0_i_2
       (.CI(tmp_product__0_i_3_n_0),
        .CO({tmp_product__0_i_2_n_0,tmp_product__0_i_2_n_1,tmp_product__0_i_2_n_2,tmp_product__0_i_2_n_3}),
        .CYINIT(1'b0),
        .DI(sum_reg[11:8]),
        .O(add_ln32_fu_145_p2[11:8]),
        .S({tmp_product__0_i_10_n_0,tmp_product__0_i_11_n_0,tmp_product__0_i_12_n_0,tmp_product__0_i_13_n_0}));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_20
       (.I0(sum_reg[1]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .O(tmp_product__0_i_20_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_21
       (.I0(sum_reg[0]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .O(tmp_product__0_i_21_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 tmp_product__0_i_3
       (.CI(tmp_product__0_i_4_n_0),
        .CO({tmp_product__0_i_3_n_0,tmp_product__0_i_3_n_1,tmp_product__0_i_3_n_2,tmp_product__0_i_3_n_3}),
        .CYINIT(1'b0),
        .DI(sum_reg[7:4]),
        .O(add_ln32_fu_145_p2[7:4]),
        .S({tmp_product__0_i_14_n_0,tmp_product__0_i_15_n_0,tmp_product__0_i_16_n_0,tmp_product__0_i_17_n_0}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 tmp_product__0_i_4
       (.CI(1'b0),
        .CO({tmp_product__0_i_4_n_0,tmp_product__0_i_4_n_1,tmp_product__0_i_4_n_2,tmp_product__0_i_4_n_3}),
        .CYINIT(1'b0),
        .DI(sum_reg[3:0]),
        .O(add_ln32_fu_145_p2[3:0]),
        .S({tmp_product__0_i_18_n_0,tmp_product__0_i_19_n_0,tmp_product__0_i_20_n_0,tmp_product__0_i_21_n_0}));
  LUT3 #(
    .INIT(8'hB8)) 
    tmp_product__0_i_5
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(tmp_product__0_i_5_n_0));
  LUT4 #(
    .INIT(16'h1DE2)) 
    tmp_product__0_i_6
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[15]),
        .O(tmp_product__0_i_6_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_7
       (.I0(sum_reg[14]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .O(tmp_product__0_i_7_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_8
       (.I0(sum_reg[13]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .O(tmp_product__0_i_8_n_0));
  LUT4 #(
    .INIT(16'h56A6)) 
    tmp_product__0_i_9
       (.I0(sum_reg[12]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .O(tmp_product__0_i_9_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 tmp_product_i_1
       (.CI(tmp_product_i_2_n_0),
        .CO({tmp_product_i_1_n_0,tmp_product_i_1_n_1,tmp_product_i_1_n_2,tmp_product_i_1_n_3}),
        .CYINIT(1'b0),
        .DI(sum_reg[26:23]),
        .O(add_ln32_fu_145_p2[27:24]),
        .S(tmp_product_1));
  LUT3 #(
    .INIT(8'h1D)) 
    tmp_product_i_12
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .O(tmp_product_i_12_n_0));
  LUT4 #(
    .INIT(16'h1DE2)) 
    tmp_product_i_16
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_reg[16]),
        .O(tmp_product_i_16_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 tmp_product_i_2
       (.CI(tmp_product_i_3_n_0),
        .CO({tmp_product_i_2_n_0,tmp_product_i_2_n_1,tmp_product_i_2_n_2,tmp_product_i_2_n_3}),
        .CYINIT(1'b0),
        .DI(sum_reg[22:19]),
        .O(add_ln32_fu_145_p2[23:20]),
        .S(tmp_product_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 tmp_product_i_3
       (.CI(tmp_product__0_i_1_n_0),
        .CO({tmp_product_i_3_n_0,tmp_product_i_3_n_1,tmp_product_i_3_n_2,tmp_product_i_3_n_3}),
        .CYINIT(1'b0),
        .DI({sum_reg[18:16],tmp_product_i_12_n_0}),
        .O(add_ln32_fu_145_p2[19:16]),
        .S({tmp_product,tmp_product_i_16_n_0}));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .O(in[0]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .O(in[10]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .O(in[11]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .O(in[12]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .O(in[13]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .O(in[14]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(in[15]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .O(in[1]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .O(in[2]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .O(in[3]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .O(in[4]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .O(in[5]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .O(in[6]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .O(in[7]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .O(in[8]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .O(in[9]));
endmodule

(* ORIG_REF_NAME = "fsk_phase_corrector_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both_1
   (\B_V_data_1_state_reg[0]_0 ,
    out_stream_TVALID_int_regslice,
    E,
    \B_V_data_1_state_reg[0]_1 ,
    \icmp_ln35_reg_289_pp0_iter2_reg_reg[0] ,
    \calibrated_reg[0] ,
    \counter_reg[0] ,
    \counter_reg[0]_0 ,
    \counter_reg[28] ,
    \counter_reg[20] ,
    add_ln32_reg_2830,
    ap_block_pp0_stage0_11001,
    \add_ln32_reg_283_reg[31] ,
    out_stream_TDATA,
    ap_rst_n_inv,
    ap_clk,
    out_stream_TREADY,
    ap_rst_n,
    calibrated_load_reg_279_pp0_iter1_reg,
    icmp_ln35_reg_289_pp0_iter1_reg,
    icmp_ln35_reg_289_pp0_iter2_reg,
    ap_enable_reg_pp0_iter3,
    calibrated_load_reg_279_pp0_iter2_reg,
    calibrated,
    in_stream_TVALID_int_regslice,
    ap_enable_reg_pp0_iter4,
    calibrated_load_reg_279_pp0_iter3_reg,
    out,
    p_0_in,
    calibrated_load_reg_279,
    icmp_ln35_reg_289,
    tmp_reg_298,
    val_in_reg_274_pp0_iter2_reg,
    Q);
  output \B_V_data_1_state_reg[0]_0 ;
  output out_stream_TVALID_int_regslice;
  output [0:0]E;
  output \B_V_data_1_state_reg[0]_1 ;
  output [0:0]\icmp_ln35_reg_289_pp0_iter2_reg_reg[0] ;
  output \calibrated_reg[0] ;
  output \counter_reg[0] ;
  output \counter_reg[0]_0 ;
  output \counter_reg[28] ;
  output \counter_reg[20] ;
  output add_ln32_reg_2830;
  output ap_block_pp0_stage0_11001;
  output \add_ln32_reg_283_reg[31] ;
  output [15:0]out_stream_TDATA;
  input ap_rst_n_inv;
  input ap_clk;
  input out_stream_TREADY;
  input ap_rst_n;
  input calibrated_load_reg_279_pp0_iter1_reg;
  input icmp_ln35_reg_289_pp0_iter1_reg;
  input icmp_ln35_reg_289_pp0_iter2_reg;
  input ap_enable_reg_pp0_iter3;
  input calibrated_load_reg_279_pp0_iter2_reg;
  input calibrated;
  input in_stream_TVALID_int_regslice;
  input ap_enable_reg_pp0_iter4;
  input calibrated_load_reg_279_pp0_iter3_reg;
  input [31:0]out;
  input p_0_in;
  input calibrated_load_reg_279;
  input icmp_ln35_reg_289;
  input tmp_reg_298;
  input [15:0]val_in_reg_274_pp0_iter2_reg;
  input [15:0]Q;

  wire \B_V_data_1_payload_A[11]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[11]_i_3_n_0 ;
  wire \B_V_data_1_payload_A[11]_i_4_n_0 ;
  wire \B_V_data_1_payload_A[11]_i_5_n_0 ;
  wire \B_V_data_1_payload_A[15]_i_1__0_n_0 ;
  wire \B_V_data_1_payload_A[15]_i_3_n_0 ;
  wire \B_V_data_1_payload_A[15]_i_4_n_0 ;
  wire \B_V_data_1_payload_A[15]_i_5_n_0 ;
  wire \B_V_data_1_payload_A[15]_i_6_n_0 ;
  wire \B_V_data_1_payload_A[3]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[3]_i_3_n_0 ;
  wire \B_V_data_1_payload_A[3]_i_4_n_0 ;
  wire \B_V_data_1_payload_A[3]_i_5_n_0 ;
  wire \B_V_data_1_payload_A[7]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[7]_i_3_n_0 ;
  wire \B_V_data_1_payload_A[7]_i_4_n_0 ;
  wire \B_V_data_1_payload_A[7]_i_5_n_0 ;
  wire \B_V_data_1_payload_A_reg[11]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[11]_i_1_n_1 ;
  wire \B_V_data_1_payload_A_reg[11]_i_1_n_2 ;
  wire \B_V_data_1_payload_A_reg[11]_i_1_n_3 ;
  wire \B_V_data_1_payload_A_reg[15]_i_2_n_1 ;
  wire \B_V_data_1_payload_A_reg[15]_i_2_n_2 ;
  wire \B_V_data_1_payload_A_reg[15]_i_2_n_3 ;
  wire \B_V_data_1_payload_A_reg[3]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[3]_i_1_n_1 ;
  wire \B_V_data_1_payload_A_reg[3]_i_1_n_2 ;
  wire \B_V_data_1_payload_A_reg[3]_i_1_n_3 ;
  wire \B_V_data_1_payload_A_reg[7]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[7]_i_1_n_1 ;
  wire \B_V_data_1_payload_A_reg[7]_i_1_n_2 ;
  wire \B_V_data_1_payload_A_reg[7]_i_1_n_3 ;
  wire \B_V_data_1_payload_A_reg_n_0_[0] ;
  wire \B_V_data_1_payload_A_reg_n_0_[10] ;
  wire \B_V_data_1_payload_A_reg_n_0_[11] ;
  wire \B_V_data_1_payload_A_reg_n_0_[12] ;
  wire \B_V_data_1_payload_A_reg_n_0_[13] ;
  wire \B_V_data_1_payload_A_reg_n_0_[14] ;
  wire \B_V_data_1_payload_A_reg_n_0_[15] ;
  wire \B_V_data_1_payload_A_reg_n_0_[1] ;
  wire \B_V_data_1_payload_A_reg_n_0_[2] ;
  wire \B_V_data_1_payload_A_reg_n_0_[3] ;
  wire \B_V_data_1_payload_A_reg_n_0_[4] ;
  wire \B_V_data_1_payload_A_reg_n_0_[5] ;
  wire \B_V_data_1_payload_A_reg_n_0_[6] ;
  wire \B_V_data_1_payload_A_reg_n_0_[7] ;
  wire \B_V_data_1_payload_A_reg_n_0_[8] ;
  wire \B_V_data_1_payload_A_reg_n_0_[9] ;
  wire \B_V_data_1_payload_B[15]_i_1__0_n_0 ;
  wire \B_V_data_1_payload_B_reg_n_0_[0] ;
  wire \B_V_data_1_payload_B_reg_n_0_[10] ;
  wire \B_V_data_1_payload_B_reg_n_0_[11] ;
  wire \B_V_data_1_payload_B_reg_n_0_[12] ;
  wire \B_V_data_1_payload_B_reg_n_0_[13] ;
  wire \B_V_data_1_payload_B_reg_n_0_[14] ;
  wire \B_V_data_1_payload_B_reg_n_0_[15] ;
  wire \B_V_data_1_payload_B_reg_n_0_[1] ;
  wire \B_V_data_1_payload_B_reg_n_0_[2] ;
  wire \B_V_data_1_payload_B_reg_n_0_[3] ;
  wire \B_V_data_1_payload_B_reg_n_0_[4] ;
  wire \B_V_data_1_payload_B_reg_n_0_[5] ;
  wire \B_V_data_1_payload_B_reg_n_0_[6] ;
  wire \B_V_data_1_payload_B_reg_n_0_[7] ;
  wire \B_V_data_1_payload_B_reg_n_0_[8] ;
  wire \B_V_data_1_payload_B_reg_n_0_[9] ;
  wire B_V_data_1_sel_rd_i_1__3_n_0;
  wire B_V_data_1_sel_rd_reg_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__2_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1_n_0 ;
  wire \B_V_data_1_state[1]_i_4_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg[0]_1 ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire [0:0]E;
  wire [15:0]Q;
  wire add_ln32_reg_2830;
  wire \add_ln32_reg_283_reg[31] ;
  wire [31:1]add_ln33_fu_161_p2;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter3;
  wire ap_enable_reg_pp0_iter4;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire calibrated;
  wire calibrated_load_reg_279;
  wire calibrated_load_reg_279_pp0_iter1_reg;
  wire calibrated_load_reg_279_pp0_iter2_reg;
  wire calibrated_load_reg_279_pp0_iter3_reg;
  wire \calibrated_reg[0] ;
  wire \counter_reg[0] ;
  wire \counter_reg[0]_0 ;
  wire \counter_reg[20] ;
  wire \counter_reg[28] ;
  wire icmp_ln35_reg_289;
  wire \icmp_ln35_reg_289[0]_i_12_n_0 ;
  wire \icmp_ln35_reg_289[0]_i_14_n_0 ;
  wire \icmp_ln35_reg_289[0]_i_7_n_0 ;
  wire \icmp_ln35_reg_289[0]_i_9_n_0 ;
  wire icmp_ln35_reg_289_pp0_iter1_reg;
  wire icmp_ln35_reg_289_pp0_iter2_reg;
  wire [0:0]\icmp_ln35_reg_289_pp0_iter2_reg_reg[0] ;
  wire \icmp_ln35_reg_289_reg[0]_i_10_n_0 ;
  wire \icmp_ln35_reg_289_reg[0]_i_10_n_1 ;
  wire \icmp_ln35_reg_289_reg[0]_i_10_n_2 ;
  wire \icmp_ln35_reg_289_reg[0]_i_10_n_3 ;
  wire \icmp_ln35_reg_289_reg[0]_i_11_n_2 ;
  wire \icmp_ln35_reg_289_reg[0]_i_11_n_3 ;
  wire \icmp_ln35_reg_289_reg[0]_i_13_n_0 ;
  wire \icmp_ln35_reg_289_reg[0]_i_13_n_1 ;
  wire \icmp_ln35_reg_289_reg[0]_i_13_n_2 ;
  wire \icmp_ln35_reg_289_reg[0]_i_13_n_3 ;
  wire \icmp_ln35_reg_289_reg[0]_i_15_n_0 ;
  wire \icmp_ln35_reg_289_reg[0]_i_15_n_1 ;
  wire \icmp_ln35_reg_289_reg[0]_i_15_n_2 ;
  wire \icmp_ln35_reg_289_reg[0]_i_15_n_3 ;
  wire \icmp_ln35_reg_289_reg[0]_i_16_n_0 ;
  wire \icmp_ln35_reg_289_reg[0]_i_16_n_1 ;
  wire \icmp_ln35_reg_289_reg[0]_i_16_n_2 ;
  wire \icmp_ln35_reg_289_reg[0]_i_16_n_3 ;
  wire \icmp_ln35_reg_289_reg[0]_i_17_n_0 ;
  wire \icmp_ln35_reg_289_reg[0]_i_17_n_1 ;
  wire \icmp_ln35_reg_289_reg[0]_i_17_n_2 ;
  wire \icmp_ln35_reg_289_reg[0]_i_17_n_3 ;
  wire \icmp_ln35_reg_289_reg[0]_i_6_n_0 ;
  wire \icmp_ln35_reg_289_reg[0]_i_6_n_1 ;
  wire \icmp_ln35_reg_289_reg[0]_i_6_n_2 ;
  wire \icmp_ln35_reg_289_reg[0]_i_6_n_3 ;
  wire \icmp_ln35_reg_289_reg[0]_i_8_n_0 ;
  wire \icmp_ln35_reg_289_reg[0]_i_8_n_1 ;
  wire \icmp_ln35_reg_289_reg[0]_i_8_n_2 ;
  wire \icmp_ln35_reg_289_reg[0]_i_8_n_3 ;
  wire in_stream_TVALID_int_regslice;
  wire [31:0]out;
  wire [15:0]out_stream_TDATA;
  wire out_stream_TREADY;
  wire out_stream_TVALID_int_regslice;
  wire p_0_in;
  wire tmp_reg_298;
  wire [15:0]val_corrected_fu_249_p2;
  wire [15:0]val_in_reg_274_pp0_iter2_reg;
  wire [3:3]\NLW_B_V_data_1_payload_A_reg[15]_i_2_CO_UNCONNECTED ;
  wire [3:2]\NLW_icmp_ln35_reg_289_reg[0]_i_11_CO_UNCONNECTED ;
  wire [3:3]\NLW_icmp_ln35_reg_289_reg[0]_i_11_O_UNCONNECTED ;

  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[11]_i_2 
       (.I0(val_in_reg_274_pp0_iter2_reg[11]),
        .I1(Q[11]),
        .O(\B_V_data_1_payload_A[11]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[11]_i_3 
       (.I0(val_in_reg_274_pp0_iter2_reg[10]),
        .I1(Q[10]),
        .O(\B_V_data_1_payload_A[11]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[11]_i_4 
       (.I0(val_in_reg_274_pp0_iter2_reg[9]),
        .I1(Q[9]),
        .O(\B_V_data_1_payload_A[11]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[11]_i_5 
       (.I0(val_in_reg_274_pp0_iter2_reg[8]),
        .I1(Q[8]),
        .O(\B_V_data_1_payload_A[11]_i_5_n_0 ));
  LUT3 #(
    .INIT(8'h0B)) 
    \B_V_data_1_payload_A[15]_i_1__0 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(\B_V_data_1_payload_A[15]_i_1__0_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[15]_i_3 
       (.I0(val_in_reg_274_pp0_iter2_reg[15]),
        .I1(Q[15]),
        .O(\B_V_data_1_payload_A[15]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[15]_i_4 
       (.I0(val_in_reg_274_pp0_iter2_reg[14]),
        .I1(Q[14]),
        .O(\B_V_data_1_payload_A[15]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[15]_i_5 
       (.I0(val_in_reg_274_pp0_iter2_reg[13]),
        .I1(Q[13]),
        .O(\B_V_data_1_payload_A[15]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[15]_i_6 
       (.I0(val_in_reg_274_pp0_iter2_reg[12]),
        .I1(Q[12]),
        .O(\B_V_data_1_payload_A[15]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[3]_i_2 
       (.I0(val_in_reg_274_pp0_iter2_reg[3]),
        .I1(Q[3]),
        .O(\B_V_data_1_payload_A[3]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[3]_i_3 
       (.I0(val_in_reg_274_pp0_iter2_reg[2]),
        .I1(Q[2]),
        .O(\B_V_data_1_payload_A[3]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[3]_i_4 
       (.I0(val_in_reg_274_pp0_iter2_reg[1]),
        .I1(Q[1]),
        .O(\B_V_data_1_payload_A[3]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[3]_i_5 
       (.I0(val_in_reg_274_pp0_iter2_reg[0]),
        .I1(Q[0]),
        .O(\B_V_data_1_payload_A[3]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[7]_i_2 
       (.I0(val_in_reg_274_pp0_iter2_reg[7]),
        .I1(Q[7]),
        .O(\B_V_data_1_payload_A[7]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[7]_i_3 
       (.I0(val_in_reg_274_pp0_iter2_reg[6]),
        .I1(Q[6]),
        .O(\B_V_data_1_payload_A[7]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[7]_i_4 
       (.I0(val_in_reg_274_pp0_iter2_reg[5]),
        .I1(Q[5]),
        .O(\B_V_data_1_payload_A[7]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \B_V_data_1_payload_A[7]_i_5 
       (.I0(val_in_reg_274_pp0_iter2_reg[4]),
        .I1(Q[4]),
        .O(\B_V_data_1_payload_A[7]_i_5_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[0]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[10]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[11]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[11]_i_1 
       (.CI(\B_V_data_1_payload_A_reg[7]_i_1_n_0 ),
        .CO({\B_V_data_1_payload_A_reg[11]_i_1_n_0 ,\B_V_data_1_payload_A_reg[11]_i_1_n_1 ,\B_V_data_1_payload_A_reg[11]_i_1_n_2 ,\B_V_data_1_payload_A_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(val_in_reg_274_pp0_iter2_reg[11:8]),
        .O(val_corrected_fu_249_p2[11:8]),
        .S({\B_V_data_1_payload_A[11]_i_2_n_0 ,\B_V_data_1_payload_A[11]_i_3_n_0 ,\B_V_data_1_payload_A[11]_i_4_n_0 ,\B_V_data_1_payload_A[11]_i_5_n_0 }));
  FDRE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[12]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[13] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[13]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[14] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[14]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[15] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[15]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[15]_i_2 
       (.CI(\B_V_data_1_payload_A_reg[11]_i_1_n_0 ),
        .CO({\NLW_B_V_data_1_payload_A_reg[15]_i_2_CO_UNCONNECTED [3],\B_V_data_1_payload_A_reg[15]_i_2_n_1 ,\B_V_data_1_payload_A_reg[15]_i_2_n_2 ,\B_V_data_1_payload_A_reg[15]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,val_in_reg_274_pp0_iter2_reg[14:12]}),
        .O(val_corrected_fu_249_p2[15:12]),
        .S({\B_V_data_1_payload_A[15]_i_3_n_0 ,\B_V_data_1_payload_A[15]_i_4_n_0 ,\B_V_data_1_payload_A[15]_i_5_n_0 ,\B_V_data_1_payload_A[15]_i_6_n_0 }));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[1]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[3]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[3]_i_1 
       (.CI(1'b0),
        .CO({\B_V_data_1_payload_A_reg[3]_i_1_n_0 ,\B_V_data_1_payload_A_reg[3]_i_1_n_1 ,\B_V_data_1_payload_A_reg[3]_i_1_n_2 ,\B_V_data_1_payload_A_reg[3]_i_1_n_3 }),
        .CYINIT(1'b1),
        .DI(val_in_reg_274_pp0_iter2_reg[3:0]),
        .O(val_corrected_fu_249_p2[3:0]),
        .S({\B_V_data_1_payload_A[3]_i_2_n_0 ,\B_V_data_1_payload_A[3]_i_3_n_0 ,\B_V_data_1_payload_A[3]_i_4_n_0 ,\B_V_data_1_payload_A[3]_i_5_n_0 }));
  FDRE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[4]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[5]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[6]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[7]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \B_V_data_1_payload_A_reg[7]_i_1 
       (.CI(\B_V_data_1_payload_A_reg[3]_i_1_n_0 ),
        .CO({\B_V_data_1_payload_A_reg[7]_i_1_n_0 ,\B_V_data_1_payload_A_reg[7]_i_1_n_1 ,\B_V_data_1_payload_A_reg[7]_i_1_n_2 ,\B_V_data_1_payload_A_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(val_in_reg_274_pp0_iter2_reg[7:4]),
        .O(val_corrected_fu_249_p2[7:4]),
        .S({\B_V_data_1_payload_A[7]_i_2_n_0 ,\B_V_data_1_payload_A[7]_i_3_n_0 ,\B_V_data_1_payload_A[7]_i_4_n_0 ,\B_V_data_1_payload_A[7]_i_5_n_0 }));
  FDRE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[8]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[9]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .R(1'b0));
  LUT3 #(
    .INIT(8'h8A)) 
    \B_V_data_1_payload_B[15]_i_1__0 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .O(\B_V_data_1_payload_B[15]_i_1__0_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[0]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[10] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[10]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[11] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[11]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[12] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[12]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[13] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[13]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[14] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[14]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[15] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[15]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[1]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[3]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[4] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[4]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[5] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[5]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[6] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[6]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[7] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[7]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[8] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[8]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[9] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(val_corrected_fu_249_p2[9]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__3
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(B_V_data_1_sel_rd_i_1__3_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__3_n_0),
        .Q(B_V_data_1_sel_rd_reg_n_0),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__2
       (.I0(out_stream_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__2_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__2_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hA8A820A0)) 
    \B_V_data_1_state[0]_i_1 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(out_stream_TREADY),
        .I4(out_stream_TVALID_int_regslice),
        .O(\B_V_data_1_state[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'h08)) 
    \B_V_data_1_state[0]_i_2 
       (.I0(calibrated_load_reg_279_pp0_iter2_reg),
        .I1(ap_enable_reg_pp0_iter3),
        .I2(\B_V_data_1_state_reg[0]_1 ),
        .O(out_stream_TVALID_int_regslice));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT4 #(
    .INIT(16'hF3FB)) 
    \B_V_data_1_state[1]_i_1 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(out_stream_TREADY),
        .I3(out_stream_TVALID_int_regslice),
        .O(B_V_data_1_state));
  LUT5 #(
    .INIT(32'hFFFF7555)) 
    \B_V_data_1_state[1]_i_3 
       (.I0(in_stream_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(calibrated_load_reg_279_pp0_iter2_reg),
        .I3(ap_enable_reg_pp0_iter3),
        .I4(\B_V_data_1_state[1]_i_4_n_0 ),
        .O(\B_V_data_1_state_reg[0]_1 ));
  LUT5 #(
    .INIT(32'h22A220A0)) 
    \B_V_data_1_state[1]_i_4 
       (.I0(ap_enable_reg_pp0_iter4),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(out_stream_TREADY),
        .I4(calibrated_load_reg_279_pp0_iter3_reg),
        .O(\B_V_data_1_state[1]_i_4_n_0 ));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1_n_0 ),
        .Q(\B_V_data_1_state_reg[0]_0 ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  LUT2 #(
    .INIT(4'h1)) 
    \add_ln32_reg_283[31]_i_1 
       (.I0(\B_V_data_1_state_reg[0]_1 ),
        .I1(calibrated),
        .O(add_ln32_reg_2830));
  LUT6 #(
    .INIT(64'hFFFFFFFF40000000)) 
    \calibrated[0]_i_1 
       (.I0(\B_V_data_1_state_reg[0]_1 ),
        .I1(\counter_reg[0] ),
        .I2(\counter_reg[0]_0 ),
        .I3(\counter_reg[28] ),
        .I4(\counter_reg[20] ),
        .I5(calibrated),
        .O(\calibrated_reg[0] ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT4 #(
    .INIT(16'h0008)) 
    \dc_offset[15]_i_1 
       (.I0(icmp_ln35_reg_289_pp0_iter2_reg),
        .I1(ap_enable_reg_pp0_iter3),
        .I2(calibrated_load_reg_279_pp0_iter2_reg),
        .I3(\B_V_data_1_state_reg[0]_1 ),
        .O(\icmp_ln35_reg_289_pp0_iter2_reg_reg[0] ));
  LUT4 #(
    .INIT(16'h0001)) 
    \icmp_ln35_reg_289[0]_i_12 
       (.I0(add_ln33_fu_161_p2[27]),
        .I1(add_ln33_fu_161_p2[26]),
        .I2(add_ln33_fu_161_p2[25]),
        .I3(add_ln33_fu_161_p2[24]),
        .O(\icmp_ln35_reg_289[0]_i_12_n_0 ));
  LUT4 #(
    .INIT(16'h0001)) 
    \icmp_ln35_reg_289[0]_i_14 
       (.I0(add_ln33_fu_161_p2[23]),
        .I1(add_ln33_fu_161_p2[22]),
        .I2(add_ln33_fu_161_p2[21]),
        .I3(add_ln33_fu_161_p2[20]),
        .O(\icmp_ln35_reg_289[0]_i_14_n_0 ));
  LUT5 #(
    .INIT(32'h00100000)) 
    \icmp_ln35_reg_289[0]_i_2 
       (.I0(add_ln33_fu_161_p2[13]),
        .I1(add_ln33_fu_161_p2[14]),
        .I2(out[0]),
        .I3(add_ln33_fu_161_p2[15]),
        .I4(\icmp_ln35_reg_289[0]_i_7_n_0 ),
        .O(\counter_reg[0] ));
  LUT5 #(
    .INIT(32'h00010000)) 
    \icmp_ln35_reg_289[0]_i_3 
       (.I0(add_ln33_fu_161_p2[3]),
        .I1(add_ln33_fu_161_p2[4]),
        .I2(add_ln33_fu_161_p2[1]),
        .I3(add_ln33_fu_161_p2[2]),
        .I4(\icmp_ln35_reg_289[0]_i_9_n_0 ),
        .O(\counter_reg[0]_0 ));
  LUT5 #(
    .INIT(32'h00010000)) 
    \icmp_ln35_reg_289[0]_i_4 
       (.I0(add_ln33_fu_161_p2[28]),
        .I1(add_ln33_fu_161_p2[29]),
        .I2(add_ln33_fu_161_p2[30]),
        .I3(add_ln33_fu_161_p2[31]),
        .I4(\icmp_ln35_reg_289[0]_i_12_n_0 ),
        .O(\counter_reg[28] ));
  LUT5 #(
    .INIT(32'h00010000)) 
    \icmp_ln35_reg_289[0]_i_5 
       (.I0(add_ln33_fu_161_p2[18]),
        .I1(add_ln33_fu_161_p2[19]),
        .I2(add_ln33_fu_161_p2[16]),
        .I3(add_ln33_fu_161_p2[17]),
        .I4(\icmp_ln35_reg_289[0]_i_14_n_0 ),
        .O(\counter_reg[20] ));
  LUT4 #(
    .INIT(16'h0100)) 
    \icmp_ln35_reg_289[0]_i_7 
       (.I0(add_ln33_fu_161_p2[12]),
        .I1(add_ln33_fu_161_p2[11]),
        .I2(add_ln33_fu_161_p2[10]),
        .I3(add_ln33_fu_161_p2[9]),
        .O(\icmp_ln35_reg_289[0]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h0400)) 
    \icmp_ln35_reg_289[0]_i_9 
       (.I0(add_ln33_fu_161_p2[7]),
        .I1(add_ln33_fu_161_p2[8]),
        .I2(add_ln33_fu_161_p2[6]),
        .I3(add_ln33_fu_161_p2[5]),
        .O(\icmp_ln35_reg_289[0]_i_9_n_0 ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln35_reg_289_reg[0]_i_10 
       (.CI(\icmp_ln35_reg_289_reg[0]_i_17_n_0 ),
        .CO({\icmp_ln35_reg_289_reg[0]_i_10_n_0 ,\icmp_ln35_reg_289_reg[0]_i_10_n_1 ,\icmp_ln35_reg_289_reg[0]_i_10_n_2 ,\icmp_ln35_reg_289_reg[0]_i_10_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln33_fu_161_p2[28:25]),
        .S(out[28:25]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln35_reg_289_reg[0]_i_11 
       (.CI(\icmp_ln35_reg_289_reg[0]_i_10_n_0 ),
        .CO({\NLW_icmp_ln35_reg_289_reg[0]_i_11_CO_UNCONNECTED [3:2],\icmp_ln35_reg_289_reg[0]_i_11_n_2 ,\icmp_ln35_reg_289_reg[0]_i_11_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_icmp_ln35_reg_289_reg[0]_i_11_O_UNCONNECTED [3],add_ln33_fu_161_p2[31:29]}),
        .S({1'b0,out[31:29]}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln35_reg_289_reg[0]_i_13 
       (.CI(\icmp_ln35_reg_289_reg[0]_i_6_n_0 ),
        .CO({\icmp_ln35_reg_289_reg[0]_i_13_n_0 ,\icmp_ln35_reg_289_reg[0]_i_13_n_1 ,\icmp_ln35_reg_289_reg[0]_i_13_n_2 ,\icmp_ln35_reg_289_reg[0]_i_13_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln33_fu_161_p2[20:17]),
        .S(out[20:17]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln35_reg_289_reg[0]_i_15 
       (.CI(\icmp_ln35_reg_289_reg[0]_i_16_n_0 ),
        .CO({\icmp_ln35_reg_289_reg[0]_i_15_n_0 ,\icmp_ln35_reg_289_reg[0]_i_15_n_1 ,\icmp_ln35_reg_289_reg[0]_i_15_n_2 ,\icmp_ln35_reg_289_reg[0]_i_15_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln33_fu_161_p2[12:9]),
        .S(out[12:9]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln35_reg_289_reg[0]_i_16 
       (.CI(\icmp_ln35_reg_289_reg[0]_i_8_n_0 ),
        .CO({\icmp_ln35_reg_289_reg[0]_i_16_n_0 ,\icmp_ln35_reg_289_reg[0]_i_16_n_1 ,\icmp_ln35_reg_289_reg[0]_i_16_n_2 ,\icmp_ln35_reg_289_reg[0]_i_16_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln33_fu_161_p2[8:5]),
        .S(out[8:5]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln35_reg_289_reg[0]_i_17 
       (.CI(\icmp_ln35_reg_289_reg[0]_i_13_n_0 ),
        .CO({\icmp_ln35_reg_289_reg[0]_i_17_n_0 ,\icmp_ln35_reg_289_reg[0]_i_17_n_1 ,\icmp_ln35_reg_289_reg[0]_i_17_n_2 ,\icmp_ln35_reg_289_reg[0]_i_17_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln33_fu_161_p2[24:21]),
        .S(out[24:21]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln35_reg_289_reg[0]_i_6 
       (.CI(\icmp_ln35_reg_289_reg[0]_i_15_n_0 ),
        .CO({\icmp_ln35_reg_289_reg[0]_i_6_n_0 ,\icmp_ln35_reg_289_reg[0]_i_6_n_1 ,\icmp_ln35_reg_289_reg[0]_i_6_n_2 ,\icmp_ln35_reg_289_reg[0]_i_6_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln33_fu_161_p2[16:13]),
        .S(out[16:13]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln35_reg_289_reg[0]_i_8 
       (.CI(1'b0),
        .CO({\icmp_ln35_reg_289_reg[0]_i_8_n_0 ,\icmp_ln35_reg_289_reg[0]_i_8_n_1 ,\icmp_ln35_reg_289_reg[0]_i_8_n_2 ,\icmp_ln35_reg_289_reg[0]_i_8_n_3 }),
        .CYINIT(out[0]),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln33_fu_161_p2[4:1]),
        .S(out[4:1]));
  LUT3 #(
    .INIT(8'h04)) 
    \mul_ln39_reg_304[41]_i_1 
       (.I0(calibrated_load_reg_279_pp0_iter1_reg),
        .I1(icmp_ln35_reg_289_pp0_iter1_reg),
        .I2(\B_V_data_1_state_reg[0]_1 ),
        .O(E));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[0]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .O(out_stream_TDATA[0]));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[10]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .O(out_stream_TDATA[10]));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[11]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .O(out_stream_TDATA[11]));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[12]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .O(out_stream_TDATA[12]));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[13]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .O(out_stream_TDATA[13]));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[14]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .O(out_stream_TDATA[14]));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[15]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(out_stream_TDATA[15]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[1]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .O(out_stream_TDATA[1]));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[2]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .O(out_stream_TDATA[2]));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[3]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .O(out_stream_TDATA[3]));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[4]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .O(out_stream_TDATA[4]));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[5]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .O(out_stream_TDATA[5]));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[6]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .O(out_stream_TDATA[6]));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[7]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .O(out_stream_TDATA[7]));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[8]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .O(out_stream_TDATA[8]));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TDATA[9]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .O(out_stream_TDATA[9]));
  LUT5 #(
    .INIT(32'hFFEF0020)) 
    \tmp_reg_298[0]_i_1 
       (.I0(p_0_in),
        .I1(calibrated_load_reg_279),
        .I2(icmp_ln35_reg_289),
        .I3(\B_V_data_1_state_reg[0]_1 ),
        .I4(tmp_reg_298),
        .O(\add_ln32_reg_283_reg[31] ));
  LUT1 #(
    .INIT(2'h1)) 
    \tmp_reg_298_pp0_iter2_reg[0]_i_1 
       (.I0(\B_V_data_1_state_reg[0]_1 ),
        .O(ap_block_pp0_stage0_11001));
endmodule

(* ORIG_REF_NAME = "fsk_phase_corrector_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0
   (in_stream_TKEEP_int_regslice,
    ap_rst_n_inv,
    ap_clk,
    ap_rst_n,
    \B_V_data_1_state_reg[0]_0 ,
    in_stream_TVALID,
    in_stream_TKEEP);
  output [3:0]in_stream_TKEEP_int_regslice;
  input ap_rst_n_inv;
  input ap_clk;
  input ap_rst_n;
  input \B_V_data_1_state_reg[0]_0 ;
  input in_stream_TVALID;
  input [3:0]in_stream_TKEEP;

  wire B_V_data_1_load_A;
  wire B_V_data_1_load_B;
  wire [3:0]B_V_data_1_payload_A;
  wire [3:0]B_V_data_1_payload_B;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__1_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__4_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__2_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [3:0]in_stream_TKEEP;
  wire [3:0]in_stream_TKEEP_int_regslice;
  wire in_stream_TVALID;

  LUT3 #(
    .INIT(8'h0D)) 
    \B_V_data_1_payload_A[3]_i_1 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TKEEP[0]),
        .Q(B_V_data_1_payload_A[0]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TKEEP[1]),
        .Q(B_V_data_1_payload_A[1]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TKEEP[2]),
        .Q(B_V_data_1_payload_A[2]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TKEEP[3]),
        .Q(B_V_data_1_payload_A[3]),
        .R(1'b0));
  LUT3 #(
    .INIT(8'hA2)) 
    \B_V_data_1_payload_B[3]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TKEEP[0]),
        .Q(B_V_data_1_payload_B[0]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TKEEP[1]),
        .Q(B_V_data_1_payload_B[1]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TKEEP[2]),
        .Q(B_V_data_1_payload_B[2]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TKEEP[3]),
        .Q(B_V_data_1_payload_B[3]),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hB4)) 
    B_V_data_1_sel_rd_i_1__1
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__1_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__1_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__4
       (.I0(in_stream_TVALID),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__4_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__4_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hAAA080A0)) 
    \B_V_data_1_state[0]_i_1__2 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(in_stream_TVALID),
        .O(\B_V_data_1_state[0]_i_1__2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT4 #(
    .INIT(16'h77F7)) 
    \B_V_data_1_state[1]_i_1__2 
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(in_stream_TVALID),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__2_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_i_1 
       (.I0(B_V_data_1_payload_B[0]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[0]),
        .O(in_stream_TKEEP_int_regslice[0]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_i_1 
       (.I0(B_V_data_1_payload_B[1]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[1]),
        .O(in_stream_TKEEP_int_regslice[1]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_i_1 
       (.I0(B_V_data_1_payload_B[2]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[2]),
        .O(in_stream_TKEEP_int_regslice[2]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_i_1 
       (.I0(B_V_data_1_payload_B[3]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[3]),
        .O(in_stream_TKEEP_int_regslice[3]));
endmodule

(* ORIG_REF_NAME = "fsk_phase_corrector_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_0
   (in_stream_TSTRB_int_regslice,
    ap_rst_n_inv,
    ap_clk,
    ap_rst_n,
    \B_V_data_1_state_reg[0]_0 ,
    in_stream_TVALID,
    in_stream_TSTRB);
  output [3:0]in_stream_TSTRB_int_regslice;
  input ap_rst_n_inv;
  input ap_clk;
  input ap_rst_n;
  input \B_V_data_1_state_reg[0]_0 ;
  input in_stream_TVALID;
  input [3:0]in_stream_TSTRB;

  wire B_V_data_1_load_A;
  wire B_V_data_1_load_B;
  wire [3:0]B_V_data_1_payload_A;
  wire [3:0]B_V_data_1_payload_B;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__0_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__5_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__1_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [3:0]in_stream_TSTRB;
  wire [3:0]in_stream_TSTRB_int_regslice;
  wire in_stream_TVALID;

  LUT3 #(
    .INIT(8'h0D)) 
    \B_V_data_1_payload_A[3]_i_1__0 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TSTRB[0]),
        .Q(B_V_data_1_payload_A[0]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TSTRB[1]),
        .Q(B_V_data_1_payload_A[1]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TSTRB[2]),
        .Q(B_V_data_1_payload_A[2]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_stream_TSTRB[3]),
        .Q(B_V_data_1_payload_A[3]),
        .R(1'b0));
  LUT3 #(
    .INIT(8'hA2)) 
    \B_V_data_1_payload_B[3]_i_1__0 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TSTRB[0]),
        .Q(B_V_data_1_payload_B[0]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TSTRB[1]),
        .Q(B_V_data_1_payload_B[1]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TSTRB[2]),
        .Q(B_V_data_1_payload_B[2]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TSTRB[3]),
        .Q(B_V_data_1_payload_B[3]),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hB4)) 
    B_V_data_1_sel_rd_i_1__0
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__0_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__0_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__5
       (.I0(in_stream_TVALID),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__5_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__5_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hAAA080A0)) 
    \B_V_data_1_state[0]_i_1__1 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(in_stream_TVALID),
        .O(\B_V_data_1_state[0]_i_1__1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT4 #(
    .INIT(16'h77F7)) 
    \B_V_data_1_state[1]_i_1__1 
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(in_stream_TVALID),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__1_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_i_1 
       (.I0(B_V_data_1_payload_B[0]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[0]),
        .O(in_stream_TSTRB_int_regslice[0]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_i_1 
       (.I0(B_V_data_1_payload_B[1]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[1]),
        .O(in_stream_TSTRB_int_regslice[1]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_i_1 
       (.I0(B_V_data_1_payload_B[2]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[2]),
        .O(in_stream_TSTRB_int_regslice[2]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_i_1 
       (.I0(B_V_data_1_payload_B[3]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[3]),
        .O(in_stream_TSTRB_int_regslice[3]));
endmodule

(* ORIG_REF_NAME = "fsk_phase_corrector_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_2
   (out_stream_TKEEP,
    ap_rst_n_inv,
    ap_clk,
    out_stream_TREADY,
    out_stream_TVALID_int_regslice,
    ap_rst_n,
    D);
  output [3:0]out_stream_TKEEP;
  input ap_rst_n_inv;
  input ap_clk;
  input out_stream_TREADY;
  input out_stream_TVALID_int_regslice;
  input ap_rst_n;
  input [3:0]D;

  wire B_V_data_1_load_A;
  wire B_V_data_1_load_B;
  wire [3:0]B_V_data_1_payload_A;
  wire [3:0]B_V_data_1_payload_B;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__4_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__6_n_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire [3:0]D;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [3:0]out_stream_TKEEP;
  wire out_stream_TREADY;
  wire out_stream_TVALID_int_regslice;

  LUT3 #(
    .INIT(8'h0D)) 
    \B_V_data_1_payload_A[3]_i_1__1 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[0]),
        .Q(B_V_data_1_payload_A[0]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[1]),
        .Q(B_V_data_1_payload_A[1]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[2]),
        .Q(B_V_data_1_payload_A[2]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[3]),
        .Q(B_V_data_1_payload_A[3]),
        .R(1'b0));
  LUT3 #(
    .INIT(8'hA2)) 
    \B_V_data_1_payload_B[3]_i_1__1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[0]),
        .Q(B_V_data_1_payload_B[0]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[1]),
        .Q(B_V_data_1_payload_B[1]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[2]),
        .Q(B_V_data_1_payload_B[2]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[3]),
        .Q(B_V_data_1_payload_B[3]),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__4
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__4_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__4_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__1
       (.I0(out_stream_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hA8A80888)) 
    \B_V_data_1_state[0]_i_1__6 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(out_stream_TREADY),
        .I4(out_stream_TVALID_int_regslice),
        .O(\B_V_data_1_state[0]_i_1__6_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT4 #(
    .INIT(16'hF5FD)) 
    \B_V_data_1_state[1]_i_1__5 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(out_stream_TREADY),
        .I3(out_stream_TVALID_int_regslice),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__6_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TKEEP[0]_INST_0 
       (.I0(B_V_data_1_payload_B[0]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[0]),
        .O(out_stream_TKEEP[0]));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TKEEP[1]_INST_0 
       (.I0(B_V_data_1_payload_B[1]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[1]),
        .O(out_stream_TKEEP[1]));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TKEEP[2]_INST_0 
       (.I0(B_V_data_1_payload_B[2]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[2]),
        .O(out_stream_TKEEP[2]));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TKEEP[3]_INST_0 
       (.I0(B_V_data_1_payload_B[3]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[3]),
        .O(out_stream_TKEEP[3]));
endmodule

(* ORIG_REF_NAME = "fsk_phase_corrector_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_4
   (out_stream_TSTRB,
    ap_rst_n_inv,
    ap_clk,
    out_stream_TREADY,
    out_stream_TVALID_int_regslice,
    ap_rst_n,
    D);
  output [3:0]out_stream_TSTRB;
  input ap_rst_n_inv;
  input ap_clk;
  input out_stream_TREADY;
  input out_stream_TVALID_int_regslice;
  input ap_rst_n;
  input [3:0]D;

  wire B_V_data_1_load_A;
  wire B_V_data_1_load_B;
  wire [3:0]B_V_data_1_payload_A;
  wire [3:0]B_V_data_1_payload_B;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__5_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__0_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__5_n_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire [3:0]D;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire out_stream_TREADY;
  wire [3:0]out_stream_TSTRB;
  wire out_stream_TVALID_int_regslice;

  LUT3 #(
    .INIT(8'h0D)) 
    \B_V_data_1_payload_A[3]_i_1__2 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[0]),
        .Q(B_V_data_1_payload_A[0]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[1]),
        .Q(B_V_data_1_payload_A[1]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[2]),
        .Q(B_V_data_1_payload_A[2]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[3]),
        .Q(B_V_data_1_payload_A[3]),
        .R(1'b0));
  LUT3 #(
    .INIT(8'hA2)) 
    \B_V_data_1_payload_B[3]_i_1__2 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[0]),
        .Q(B_V_data_1_payload_B[0]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[1]),
        .Q(B_V_data_1_payload_B[1]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[2]),
        .Q(B_V_data_1_payload_B[2]),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[3]),
        .Q(B_V_data_1_payload_B[3]),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__5
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__5_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__5_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__0
       (.I0(out_stream_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__0_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__0_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hA8A80888)) 
    \B_V_data_1_state[0]_i_1__5 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(out_stream_TREADY),
        .I4(out_stream_TVALID_int_regslice),
        .O(\B_V_data_1_state[0]_i_1__5_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT4 #(
    .INIT(16'hF5FD)) 
    \B_V_data_1_state[1]_i_1__4 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(out_stream_TREADY),
        .I3(out_stream_TVALID_int_regslice),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__5_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair31" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TSTRB[0]_INST_0 
       (.I0(B_V_data_1_payload_B[0]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[0]),
        .O(out_stream_TSTRB[0]));
  (* SOFT_HLUTNM = "soft_lutpair31" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TSTRB[1]_INST_0 
       (.I0(B_V_data_1_payload_B[1]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[1]),
        .O(out_stream_TSTRB[1]));
  (* SOFT_HLUTNM = "soft_lutpair32" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TSTRB[2]_INST_0 
       (.I0(B_V_data_1_payload_B[2]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[2]),
        .O(out_stream_TSTRB[2]));
  (* SOFT_HLUTNM = "soft_lutpair32" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TSTRB[3]_INST_0 
       (.I0(B_V_data_1_payload_B[3]),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A[3]),
        .O(out_stream_TSTRB[3]));
endmodule

(* ORIG_REF_NAME = "fsk_phase_corrector_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1
   (in_stream_TLAST_int_regslice,
    ap_rst_n_inv,
    ap_clk,
    ap_rst_n,
    \B_V_data_1_state_reg[0]_0 ,
    in_stream_TVALID,
    in_stream_TLAST);
  output in_stream_TLAST_int_regslice;
  input ap_rst_n_inv;
  input ap_clk;
  input ap_rst_n;
  input \B_V_data_1_state_reg[0]_0 ;
  input in_stream_TVALID;
  input [0:0]in_stream_TLAST;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__6_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__0_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TLAST_int_regslice;
  wire in_stream_TVALID;

  LUT5 #(
    .INIT(32'hFFAE00A2)) 
    \B_V_data_1_payload_A[0]_i_1 
       (.I0(in_stream_TLAST),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(B_V_data_1_sel_wr),
        .I4(B_V_data_1_payload_A),
        .O(\B_V_data_1_payload_A[0]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[0]_i_1_n_0 ),
        .Q(B_V_data_1_payload_A),
        .R(1'b0));
  LUT5 #(
    .INIT(32'hBBFB8808)) 
    \B_V_data_1_payload_B[0]_i_1 
       (.I0(in_stream_TLAST),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(B_V_data_1_payload_B),
        .O(\B_V_data_1_payload_B[0]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[0]_i_1_n_0 ),
        .Q(B_V_data_1_payload_B),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hB4)) 
    B_V_data_1_sel_rd_i_1
       (.I0(\B_V_data_1_state_reg[0]_0 ),
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
    B_V_data_1_sel_wr_i_1__6
       (.I0(in_stream_TVALID),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__6_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__6_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hAAA080A0)) 
    \B_V_data_1_state[0]_i_1__0 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(in_stream_TVALID),
        .O(\B_V_data_1_state[0]_i_1__0_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT4 #(
    .INIT(16'h77F7)) 
    \B_V_data_1_state[1]_i_1__0 
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(in_stream_TVALID),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__0_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2_i_1 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(in_stream_TLAST_int_regslice));
endmodule

(* ORIG_REF_NAME = "fsk_phase_corrector_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1_3
   (out_stream_TLAST,
    ap_rst_n_inv,
    ap_clk,
    out_stream_TREADY,
    out_stream_TVALID_int_regslice,
    ap_rst_n,
    pkt_in_last_V_reg_269_pp0_iter2_reg);
  output [0:0]out_stream_TLAST;
  input ap_rst_n_inv;
  input ap_clk;
  input out_stream_TREADY;
  input out_stream_TVALID_int_regslice;
  input ap_rst_n;
  input pkt_in_last_V_reg_269_pp0_iter2_reg;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__0_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1__0_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__6_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__4_n_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire out_stream_TVALID_int_regslice;
  wire pkt_in_last_V_reg_269_pp0_iter2_reg;

  LUT5 #(
    .INIT(32'hFFAE00A2)) 
    \B_V_data_1_payload_A[0]_i_1__0 
       (.I0(pkt_in_last_V_reg_269_pp0_iter2_reg),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(B_V_data_1_sel_wr),
        .I4(B_V_data_1_payload_A),
        .O(\B_V_data_1_payload_A[0]_i_1__0_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[0]_i_1__0_n_0 ),
        .Q(B_V_data_1_payload_A),
        .R(1'b0));
  LUT5 #(
    .INIT(32'hBBFB8808)) 
    \B_V_data_1_payload_B[0]_i_1__0 
       (.I0(pkt_in_last_V_reg_269_pp0_iter2_reg),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(B_V_data_1_payload_B),
        .O(\B_V_data_1_payload_B[0]_i_1__0_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[0]_i_1__0_n_0 ),
        .Q(B_V_data_1_payload_B),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__6
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__6_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__6_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1
       (.I0(out_stream_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hA8A80888)) 
    \B_V_data_1_state[0]_i_1__4 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(out_stream_TREADY),
        .I4(out_stream_TVALID_int_regslice),
        .O(\B_V_data_1_state[0]_i_1__4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT4 #(
    .INIT(16'hF5FD)) 
    \B_V_data_1_state[1]_i_1__3 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(out_stream_TREADY),
        .I3(out_stream_TVALID_int_regslice),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__4_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TLAST[0]_INST_0 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(out_stream_TLAST));
endmodule

(* CHECK_LICENSE_TYPE = "system_fsk_phase_corrector_0_0,fsk_phase_corrector,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "HLS" *) 
(* X_CORE_INFO = "fsk_phase_corrector,Vivado 2023.1" *) (* hls_module = "yes" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (ap_clk,
    ap_rst_n,
    in_stream_TVALID,
    in_stream_TREADY,
    in_stream_TDATA,
    in_stream_TLAST,
    in_stream_TKEEP,
    in_stream_TSTRB,
    out_stream_TVALID,
    out_stream_TREADY,
    out_stream_TDATA,
    out_stream_TLAST,
    out_stream_TKEEP,
    out_stream_TSTRB);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ap_clk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ap_clk, ASSOCIATED_BUSIF in_stream:out_stream, ASSOCIATED_RESET ap_rst_n, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input ap_clk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 ap_rst_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ap_rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input ap_rst_n;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TVALID" *) input in_stream_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TREADY" *) output in_stream_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TDATA" *) input [31:0]in_stream_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TLAST" *) input [0:0]in_stream_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TKEEP" *) input [3:0]in_stream_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME in_stream, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input [3:0]in_stream_TSTRB;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TVALID" *) output out_stream_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TREADY" *) input out_stream_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TDATA" *) output [31:0]out_stream_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TLAST" *) output [0:0]out_stream_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TKEEP" *) output [3:0]out_stream_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME out_stream, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) output [3:0]out_stream_TSTRB;

  wire ap_clk;
  wire ap_rst_n;
  wire [31:0]in_stream_TDATA;
  wire [3:0]in_stream_TKEEP;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TREADY;
  wire [3:0]in_stream_TSTRB;
  wire in_stream_TVALID;
  wire [31:0]out_stream_TDATA;
  wire [3:0]out_stream_TKEEP;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire [3:0]out_stream_TSTRB;
  wire out_stream_TVALID;

  (* SDX_KERNEL = "true" *) 
  (* SDX_KERNEL_SYNTH_INST = "inst" *) 
  (* SDX_KERNEL_TYPE = "hls" *) 
  (* ap_ST_fsm_pp0_stage0 = "1'b1" *) 
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector inst
       (.ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .in_stream_TDATA({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,in_stream_TDATA[15:0]}),
        .in_stream_TKEEP(in_stream_TKEEP),
        .in_stream_TLAST(in_stream_TLAST),
        .in_stream_TREADY(in_stream_TREADY),
        .in_stream_TSTRB(in_stream_TSTRB),
        .in_stream_TVALID(in_stream_TVALID),
        .out_stream_TDATA(out_stream_TDATA),
        .out_stream_TKEEP(out_stream_TKEEP),
        .out_stream_TLAST(out_stream_TLAST),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TSTRB(out_stream_TSTRB),
        .out_stream_TVALID(out_stream_TVALID));
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
