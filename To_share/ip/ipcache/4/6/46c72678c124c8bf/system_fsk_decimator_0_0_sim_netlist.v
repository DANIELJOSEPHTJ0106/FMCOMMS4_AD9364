// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Tue Feb  3 16:25:28 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_fsk_decimator_0_0_sim_netlist.v
// Design      : system_fsk_decimator_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* ap_ST_fsm_pp0_stage0 = "1'b1" *) (* hls_module = "yes" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator
   (ap_clk,
    ap_rst_n,
    rx_in_TDATA,
    rx_in_TVALID,
    rx_in_TREADY,
    rx_in_TKEEP,
    rx_in_TSTRB,
    rx_in_TLAST,
    dec_out_TDATA,
    dec_out_TVALID,
    dec_out_TREADY,
    dec_out_TKEEP,
    dec_out_TSTRB,
    dec_out_TLAST);
  input ap_clk;
  input ap_rst_n;
  input [31:0]rx_in_TDATA;
  input rx_in_TVALID;
  output rx_in_TREADY;
  input [3:0]rx_in_TKEEP;
  input [3:0]rx_in_TSTRB;
  input [0:0]rx_in_TLAST;
  output [31:0]dec_out_TDATA;
  output dec_out_TVALID;
  input dec_out_TREADY;
  output [3:0]dec_out_TKEEP;
  output [3:0]dec_out_TSTRB;
  output [0:0]dec_out_TLAST;

  wire \<const0> ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_wr;
  wire [23:0]acc_i_reg;
  wire [23:0]acc_q_reg;
  wire [23:8]add_ln40_fu_216_p2;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter2;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [31:0]data_in;
  wire \dec_counter[1]_i_1_n_0 ;
  wire [7:0]dec_counter_reg;
  wire [31:0]dec_out_TDATA;
  wire [0:0]dec_out_TLAST;
  wire dec_out_TREADY;
  wire dec_out_TVALID;
  wire icmp_ln40_fu_226_p2;
  wire icmp_ln40_reg_289;
  wire icmp_ln40_reg_289_pp0_iter1_reg;
  wire [15:0]p_0_in;
  wire pkt_rx_last_V_reg_284;
  wire regslice_both_dec_out_V_data_V_U_n_0;
  wire regslice_both_dec_out_V_data_V_U_n_10;
  wire regslice_both_dec_out_V_data_V_U_n_11;
  wire regslice_both_dec_out_V_data_V_U_n_12;
  wire regslice_both_dec_out_V_data_V_U_n_14;
  wire regslice_both_dec_out_V_data_V_U_n_15;
  wire regslice_both_dec_out_V_data_V_U_n_16;
  wire regslice_both_dec_out_V_data_V_U_n_17;
  wire regslice_both_dec_out_V_data_V_U_n_2;
  wire regslice_both_dec_out_V_data_V_U_n_4;
  wire regslice_both_dec_out_V_data_V_U_n_5;
  wire regslice_both_dec_out_V_data_V_U_n_6;
  wire regslice_both_dec_out_V_data_V_U_n_7;
  wire regslice_both_dec_out_V_data_V_U_n_8;
  wire regslice_both_dec_out_V_data_V_U_n_9;
  wire regslice_both_dec_out_V_last_V_U_n_0;
  wire regslice_both_dec_out_V_last_V_U_n_3;
  wire regslice_both_dec_out_V_last_V_U_n_4;
  wire regslice_both_rx_in_V_data_V_U_n_10;
  wire regslice_both_rx_in_V_data_V_U_n_11;
  wire regslice_both_rx_in_V_data_V_U_n_12;
  wire regslice_both_rx_in_V_data_V_U_n_13;
  wire regslice_both_rx_in_V_data_V_U_n_14;
  wire regslice_both_rx_in_V_data_V_U_n_15;
  wire regslice_both_rx_in_V_data_V_U_n_16;
  wire regslice_both_rx_in_V_data_V_U_n_17;
  wire regslice_both_rx_in_V_data_V_U_n_18;
  wire regslice_both_rx_in_V_data_V_U_n_19;
  wire regslice_both_rx_in_V_data_V_U_n_20;
  wire regslice_both_rx_in_V_data_V_U_n_21;
  wire regslice_both_rx_in_V_data_V_U_n_22;
  wire regslice_both_rx_in_V_data_V_U_n_23;
  wire regslice_both_rx_in_V_data_V_U_n_24;
  wire regslice_both_rx_in_V_data_V_U_n_25;
  wire regslice_both_rx_in_V_data_V_U_n_26;
  wire regslice_both_rx_in_V_data_V_U_n_3;
  wire regslice_both_rx_in_V_data_V_U_n_4;
  wire regslice_both_rx_in_V_data_V_U_n_43;
  wire regslice_both_rx_in_V_data_V_U_n_44;
  wire regslice_both_rx_in_V_data_V_U_n_45;
  wire regslice_both_rx_in_V_data_V_U_n_46;
  wire regslice_both_rx_in_V_data_V_U_n_47;
  wire regslice_both_rx_in_V_data_V_U_n_48;
  wire regslice_both_rx_in_V_data_V_U_n_49;
  wire regslice_both_rx_in_V_data_V_U_n_5;
  wire regslice_both_rx_in_V_data_V_U_n_50;
  wire regslice_both_rx_in_V_data_V_U_n_51;
  wire regslice_both_rx_in_V_data_V_U_n_52;
  wire regslice_both_rx_in_V_data_V_U_n_53;
  wire regslice_both_rx_in_V_data_V_U_n_54;
  wire regslice_both_rx_in_V_data_V_U_n_55;
  wire regslice_both_rx_in_V_data_V_U_n_56;
  wire regslice_both_rx_in_V_data_V_U_n_57;
  wire regslice_both_rx_in_V_data_V_U_n_58;
  wire regslice_both_rx_in_V_data_V_U_n_59;
  wire regslice_both_rx_in_V_data_V_U_n_6;
  wire regslice_both_rx_in_V_data_V_U_n_60;
  wire regslice_both_rx_in_V_data_V_U_n_61;
  wire regslice_both_rx_in_V_data_V_U_n_62;
  wire regslice_both_rx_in_V_data_V_U_n_63;
  wire regslice_both_rx_in_V_data_V_U_n_64;
  wire regslice_both_rx_in_V_data_V_U_n_65;
  wire regslice_both_rx_in_V_data_V_U_n_66;
  wire regslice_both_rx_in_V_data_V_U_n_7;
  wire regslice_both_rx_in_V_data_V_U_n_8;
  wire regslice_both_rx_in_V_data_V_U_n_83;
  wire regslice_both_rx_in_V_data_V_U_n_9;
  wire [31:0]rx_in_TDATA;
  wire [0:0]rx_in_TLAST;
  wire rx_in_TLAST_int_regslice;
  wire rx_in_TREADY;
  wire rx_in_TVALID;
  wire rx_in_TVALID_int_regslice;
  wire trunc_ln2_reg_2930;

  assign dec_out_TKEEP[3] = \<const0> ;
  assign dec_out_TKEEP[2] = \<const0> ;
  assign dec_out_TKEEP[1] = \<const0> ;
  assign dec_out_TKEEP[0] = \<const0> ;
  assign dec_out_TSTRB[3] = \<const0> ;
  assign dec_out_TSTRB[2] = \<const0> ;
  assign dec_out_TSTRB[1] = \<const0> ;
  assign dec_out_TSTRB[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_6),
        .Q(acc_i_reg[0]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_12),
        .Q(acc_i_reg[10]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_11),
        .Q(acc_i_reg[11]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_18),
        .Q(acc_i_reg[12]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_17),
        .Q(acc_i_reg[13]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_16),
        .Q(acc_i_reg[14]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_15),
        .Q(acc_i_reg[15]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_22),
        .Q(acc_i_reg[16]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_21),
        .Q(acc_i_reg[17]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_20),
        .Q(acc_i_reg[18]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_19),
        .Q(acc_i_reg[19]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_5),
        .Q(acc_i_reg[1]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_26),
        .Q(acc_i_reg[20]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_25),
        .Q(acc_i_reg[21]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_24),
        .Q(acc_i_reg[22]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_23),
        .Q(acc_i_reg[23]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_4),
        .Q(acc_i_reg[2]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_3),
        .Q(acc_i_reg[3]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_10),
        .Q(acc_i_reg[4]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_9),
        .Q(acc_i_reg[5]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_8),
        .Q(acc_i_reg[6]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_7),
        .Q(acc_i_reg[7]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_14),
        .Q(acc_i_reg[8]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_i_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_13),
        .Q(acc_i_reg[9]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_46),
        .Q(acc_q_reg[0]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_52),
        .Q(acc_q_reg[10]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_51),
        .Q(acc_q_reg[11]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_58),
        .Q(acc_q_reg[12]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_57),
        .Q(acc_q_reg[13]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_56),
        .Q(acc_q_reg[14]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_55),
        .Q(acc_q_reg[15]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_62),
        .Q(acc_q_reg[16]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_61),
        .Q(acc_q_reg[17]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_60),
        .Q(acc_q_reg[18]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_59),
        .Q(acc_q_reg[19]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_45),
        .Q(acc_q_reg[1]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_66),
        .Q(acc_q_reg[20]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_65),
        .Q(acc_q_reg[21]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_64),
        .Q(acc_q_reg[22]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_63),
        .Q(acc_q_reg[23]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_44),
        .Q(acc_q_reg[2]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_43),
        .Q(acc_q_reg[3]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_50),
        .Q(acc_q_reg[4]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_49),
        .Q(acc_q_reg[5]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_48),
        .Q(acc_q_reg[6]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_47),
        .Q(acc_q_reg[7]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_54),
        .Q(acc_q_reg[8]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    \acc_q_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_rx_in_V_data_V_U_n_53),
        .Q(acc_q_reg[9]),
        .R(trunc_ln2_reg_2930));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter1_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_rx_in_V_data_V_U_n_83),
        .Q(ap_enable_reg_pp0_iter1),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter2_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_dec_out_V_data_V_U_n_16),
        .Q(ap_enable_reg_pp0_iter2),
        .R(ap_rst_n_inv));
  LUT2 #(
    .INIT(4'h6)) 
    \dec_counter[1]_i_1 
       (.I0(dec_counter_reg[1]),
        .I1(dec_counter_reg[0]),
        .O(\dec_counter[1]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \dec_counter_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_dec_out_V_data_V_U_n_12),
        .Q(dec_counter_reg[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dec_counter_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\dec_counter[1]_i_1_n_0 ),
        .Q(dec_counter_reg[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dec_counter_reg[2] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_dec_out_V_data_V_U_n_10),
        .Q(dec_counter_reg[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dec_counter_reg[3] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_dec_out_V_data_V_U_n_11),
        .Q(dec_counter_reg[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dec_counter_reg[4] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_dec_out_V_data_V_U_n_9),
        .Q(dec_counter_reg[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dec_counter_reg[5] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_dec_out_V_data_V_U_n_8),
        .Q(dec_counter_reg[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dec_counter_reg[6] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_dec_out_V_data_V_U_n_7),
        .Q(dec_counter_reg[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \dec_counter_reg[7] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_dec_out_V_data_V_U_n_6),
        .Q(dec_counter_reg[7]),
        .R(1'b0));
  LUT3 #(
    .INIT(8'h80)) 
    \icmp_ln40_reg_289[0]_i_1 
       (.I0(dec_counter_reg[7]),
        .I1(regslice_both_dec_out_V_data_V_U_n_5),
        .I2(dec_counter_reg[6]),
        .O(icmp_ln40_fu_226_p2));
  FDRE \icmp_ln40_reg_289_pp0_iter1_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln40_reg_289),
        .Q(icmp_ln40_reg_289_pp0_iter1_reg),
        .R(1'b0));
  FDRE \icmp_ln40_reg_289_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln40_fu_226_p2),
        .Q(icmp_ln40_reg_289),
        .R(1'b0));
  FDRE \pkt_rx_last_V_reg_284_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(rx_in_TLAST_int_regslice),
        .Q(pkt_rx_last_V_reg_284),
        .R(1'b0));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both regslice_both_dec_out_V_data_V_U
       (.B_V_data_1_sel(B_V_data_1_sel),
        .B_V_data_1_sel_wr(B_V_data_1_sel_wr),
        .\B_V_data_1_state_reg[0]_0 (dec_out_TVALID),
        .\B_V_data_1_state_reg[0]_1 (regslice_both_dec_out_V_data_V_U_n_16),
        .\B_V_data_1_state_reg[0]_2 (regslice_both_dec_out_V_last_V_U_n_3),
        .\B_V_data_1_state_reg[0]_3 (regslice_both_dec_out_V_last_V_U_n_0),
        .\B_V_data_1_state_reg[0]_4 (regslice_both_dec_out_V_last_V_U_n_4),
        .\B_V_data_1_state_reg[1]_0 (regslice_both_dec_out_V_data_V_U_n_0),
        .\B_V_data_1_state_reg[1]_1 (regslice_both_dec_out_V_data_V_U_n_14),
        .\B_V_data_1_state_reg[1]_2 (regslice_both_dec_out_V_data_V_U_n_15),
        .D(data_in),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter1(ap_enable_reg_pp0_iter1),
        .ap_enable_reg_pp0_iter1_reg(regslice_both_dec_out_V_data_V_U_n_4),
        .ap_enable_reg_pp0_iter1_reg_0(regslice_both_dec_out_V_data_V_U_n_17),
        .ap_enable_reg_pp0_iter2(ap_enable_reg_pp0_iter2),
        .ap_rst_n_inv(ap_rst_n_inv),
        .dec_counter_reg(dec_counter_reg),
        .\dec_counter_reg[5]_0 (regslice_both_dec_out_V_data_V_U_n_8),
        .dec_counter_reg_0_sp_1(regslice_both_dec_out_V_data_V_U_n_12),
        .dec_counter_reg_2_sp_1(regslice_both_dec_out_V_data_V_U_n_10),
        .dec_counter_reg_3_sp_1(regslice_both_dec_out_V_data_V_U_n_11),
        .dec_counter_reg_4_sp_1(regslice_both_dec_out_V_data_V_U_n_9),
        .dec_counter_reg_5_sp_1(regslice_both_dec_out_V_data_V_U_n_5),
        .dec_counter_reg_6_sp_1(regslice_both_dec_out_V_data_V_U_n_7),
        .dec_counter_reg_7_sp_1(regslice_both_dec_out_V_data_V_U_n_6),
        .dec_out_TDATA(dec_out_TDATA),
        .dec_out_TREADY(dec_out_TREADY),
        .icmp_ln40_reg_289(icmp_ln40_reg_289),
        .icmp_ln40_reg_289_pp0_iter1_reg(icmp_ln40_reg_289_pp0_iter1_reg),
        .\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] (regslice_both_dec_out_V_data_V_U_n_2),
        .rx_in_TVALID_int_regslice(rx_in_TVALID_int_regslice),
        .trunc_ln2_reg_2930(trunc_ln2_reg_2930));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1 regslice_both_dec_out_V_last_V_U
       (.B_V_data_1_sel_wr(B_V_data_1_sel_wr),
        .B_V_data_1_sel_wr_reg_0(regslice_both_dec_out_V_data_V_U_n_15),
        .\B_V_data_1_state_reg[0]_0 (regslice_both_dec_out_V_data_V_U_n_14),
        .\B_V_data_1_state_reg[1]_0 (regslice_both_dec_out_V_last_V_U_n_0),
        .\B_V_data_1_state_reg[1]_1 (regslice_both_dec_out_V_last_V_U_n_4),
        .\B_V_data_1_state_reg[1]_2 (regslice_both_dec_out_V_data_V_U_n_0),
        .\B_V_data_1_state_reg[1]_3 (regslice_both_dec_out_V_data_V_U_n_2),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter1(ap_enable_reg_pp0_iter1),
        .ap_enable_reg_pp0_iter1_reg(regslice_both_dec_out_V_last_V_U_n_3),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .dec_out_TLAST(dec_out_TLAST),
        .dec_out_TREADY(dec_out_TREADY),
        .icmp_ln40_reg_289(icmp_ln40_reg_289),
        .pkt_rx_last_V_reg_284(pkt_rx_last_V_reg_284),
        .rx_in_TVALID_int_regslice(rx_in_TVALID_int_regslice));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both_0 regslice_both_rx_in_V_data_V_U
       (.\B_V_data_1_payload_B_reg[15]_0 ({regslice_both_rx_in_V_data_V_U_n_23,regslice_both_rx_in_V_data_V_U_n_24,regslice_both_rx_in_V_data_V_U_n_25,regslice_both_rx_in_V_data_V_U_n_26}),
        .\B_V_data_1_payload_B_reg[31]_0 ({regslice_both_rx_in_V_data_V_U_n_63,regslice_both_rx_in_V_data_V_U_n_64,regslice_both_rx_in_V_data_V_U_n_65,regslice_both_rx_in_V_data_V_U_n_66}),
        .B_V_data_1_sel(B_V_data_1_sel),
        .B_V_data_1_sel_rd_reg_0(regslice_both_dec_out_V_data_V_U_n_17),
        .\B_V_data_1_state_reg[0]_0 (regslice_both_rx_in_V_data_V_U_n_83),
        .\B_V_data_1_state_reg[0]_1 (regslice_both_dec_out_V_data_V_U_n_2),
        .\B_V_data_1_state_reg[0]_2 (regslice_both_dec_out_V_data_V_U_n_0),
        .\B_V_data_1_state_reg[0]_3 (regslice_both_dec_out_V_last_V_U_n_3),
        .\B_V_data_1_state_reg[1]_0 (rx_in_TREADY),
        .D(p_0_in),
        .O({regslice_both_rx_in_V_data_V_U_n_3,regslice_both_rx_in_V_data_V_U_n_4,regslice_both_rx_in_V_data_V_U_n_5,regslice_both_rx_in_V_data_V_U_n_6}),
        .acc_i_reg(acc_i_reg),
        .\acc_i_reg[11] ({regslice_both_rx_in_V_data_V_U_n_11,regslice_both_rx_in_V_data_V_U_n_12,regslice_both_rx_in_V_data_V_U_n_13,regslice_both_rx_in_V_data_V_U_n_14}),
        .\acc_i_reg[14] ({regslice_both_rx_in_V_data_V_U_n_15,regslice_both_rx_in_V_data_V_U_n_16,regslice_both_rx_in_V_data_V_U_n_17,regslice_both_rx_in_V_data_V_U_n_18}),
        .\acc_i_reg[14]_0 ({regslice_both_rx_in_V_data_V_U_n_19,regslice_both_rx_in_V_data_V_U_n_20,regslice_both_rx_in_V_data_V_U_n_21,regslice_both_rx_in_V_data_V_U_n_22}),
        .\acc_i_reg[7] ({regslice_both_rx_in_V_data_V_U_n_7,regslice_both_rx_in_V_data_V_U_n_8,regslice_both_rx_in_V_data_V_U_n_9,regslice_both_rx_in_V_data_V_U_n_10}),
        .acc_q_reg(acc_q_reg),
        .\acc_q_reg[11] ({regslice_both_rx_in_V_data_V_U_n_51,regslice_both_rx_in_V_data_V_U_n_52,regslice_both_rx_in_V_data_V_U_n_53,regslice_both_rx_in_V_data_V_U_n_54}),
        .\acc_q_reg[14] ({regslice_both_rx_in_V_data_V_U_n_55,regslice_both_rx_in_V_data_V_U_n_56,regslice_both_rx_in_V_data_V_U_n_57,regslice_both_rx_in_V_data_V_U_n_58}),
        .\acc_q_reg[14]_0 ({regslice_both_rx_in_V_data_V_U_n_59,regslice_both_rx_in_V_data_V_U_n_60,regslice_both_rx_in_V_data_V_U_n_61,regslice_both_rx_in_V_data_V_U_n_62}),
        .\acc_q_reg[21] (add_ln40_fu_216_p2),
        .\acc_q_reg[3] ({regslice_both_rx_in_V_data_V_U_n_43,regslice_both_rx_in_V_data_V_U_n_44,regslice_both_rx_in_V_data_V_U_n_45,regslice_both_rx_in_V_data_V_U_n_46}),
        .\acc_q_reg[7] ({regslice_both_rx_in_V_data_V_U_n_47,regslice_both_rx_in_V_data_V_U_n_48,regslice_both_rx_in_V_data_V_U_n_49,regslice_both_rx_in_V_data_V_U_n_50}),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter1(ap_enable_reg_pp0_iter1),
        .ap_rst_n_inv(ap_rst_n_inv),
        .rx_in_TDATA(rx_in_TDATA),
        .rx_in_TVALID(rx_in_TVALID),
        .rx_in_TVALID_int_regslice(rx_in_TVALID_int_regslice));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1_1 regslice_both_rx_in_V_last_V_U
       (.B_V_data_1_sel_rd_reg_0(regslice_both_dec_out_V_data_V_U_n_2),
        .B_V_data_1_sel_rd_reg_1(regslice_both_dec_out_V_data_V_U_n_0),
        .B_V_data_1_sel_rd_reg_2(regslice_both_dec_out_V_last_V_U_n_3),
        .\B_V_data_1_state_reg[0]_0 (regslice_both_dec_out_V_data_V_U_n_4),
        .ap_clk(ap_clk),
        .ap_rst_n_inv(ap_rst_n_inv),
        .rx_in_TLAST(rx_in_TLAST),
        .rx_in_TLAST_int_regslice(rx_in_TLAST_int_regslice),
        .rx_in_TVALID(rx_in_TVALID),
        .rx_in_TVALID_int_regslice(rx_in_TVALID_int_regslice));
  FDRE \trunc_ln2_reg_293_reg[0] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[0]),
        .Q(data_in[0]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[10] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[10]),
        .Q(data_in[10]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[11] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[11]),
        .Q(data_in[11]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[12] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[12]),
        .Q(data_in[12]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[13] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[13]),
        .Q(data_in[13]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[14] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[14]),
        .Q(data_in[14]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[15] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[15]),
        .Q(data_in[15]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[1] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[1]),
        .Q(data_in[1]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[2] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[2]),
        .Q(data_in[2]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[3] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[3]),
        .Q(data_in[3]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[4] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[4]),
        .Q(data_in[4]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[5] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[5]),
        .Q(data_in[5]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[6] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[6]),
        .Q(data_in[6]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[7] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[7]),
        .Q(data_in[7]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[8] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[8]),
        .Q(data_in[8]),
        .R(1'b0));
  FDRE \trunc_ln2_reg_293_reg[9] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(p_0_in[9]),
        .Q(data_in[9]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[0] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[8]),
        .Q(data_in[16]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[10] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[18]),
        .Q(data_in[26]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[11] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[19]),
        .Q(data_in[27]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[12] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[20]),
        .Q(data_in[28]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[13] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[21]),
        .Q(data_in[29]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[14] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[22]),
        .Q(data_in[30]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[15] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[23]),
        .Q(data_in[31]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[1] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[9]),
        .Q(data_in[17]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[2] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[10]),
        .Q(data_in[18]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[3] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[11]),
        .Q(data_in[19]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[4] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[12]),
        .Q(data_in[20]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[5] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[13]),
        .Q(data_in[21]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[6] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[14]),
        .Q(data_in[22]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[7] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[15]),
        .Q(data_in[23]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[8] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[16]),
        .Q(data_in[24]),
        .R(1'b0));
  FDRE \trunc_ln3_reg_298_reg[9] 
       (.C(ap_clk),
        .CE(trunc_ln2_reg_2930),
        .D(add_ln40_fu_216_p2[17]),
        .Q(data_in[25]),
        .R(1'b0));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both
   (\B_V_data_1_state_reg[1]_0 ,
    \B_V_data_1_state_reg[0]_0 ,
    \icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ,
    trunc_ln2_reg_2930,
    ap_enable_reg_pp0_iter1_reg,
    dec_counter_reg_5_sp_1,
    dec_counter_reg_7_sp_1,
    dec_counter_reg_6_sp_1,
    \dec_counter_reg[5]_0 ,
    dec_counter_reg_4_sp_1,
    dec_counter_reg_2_sp_1,
    dec_counter_reg_3_sp_1,
    dec_counter_reg_0_sp_1,
    ap_block_pp0_stage0_11001,
    \B_V_data_1_state_reg[1]_1 ,
    \B_V_data_1_state_reg[1]_2 ,
    \B_V_data_1_state_reg[0]_1 ,
    ap_enable_reg_pp0_iter1_reg_0,
    dec_out_TDATA,
    ap_rst_n_inv,
    ap_clk,
    dec_out_TREADY,
    rx_in_TVALID_int_regslice,
    \B_V_data_1_state_reg[0]_2 ,
    dec_counter_reg,
    ap_enable_reg_pp0_iter1,
    icmp_ln40_reg_289,
    icmp_ln40_reg_289_pp0_iter1_reg,
    ap_enable_reg_pp0_iter2,
    \B_V_data_1_state_reg[0]_3 ,
    \B_V_data_1_state_reg[0]_4 ,
    B_V_data_1_sel_wr,
    B_V_data_1_sel,
    D);
  output \B_V_data_1_state_reg[1]_0 ;
  output \B_V_data_1_state_reg[0]_0 ;
  output \icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ;
  output trunc_ln2_reg_2930;
  output ap_enable_reg_pp0_iter1_reg;
  output dec_counter_reg_5_sp_1;
  output dec_counter_reg_7_sp_1;
  output dec_counter_reg_6_sp_1;
  output \dec_counter_reg[5]_0 ;
  output dec_counter_reg_4_sp_1;
  output dec_counter_reg_2_sp_1;
  output dec_counter_reg_3_sp_1;
  output dec_counter_reg_0_sp_1;
  output ap_block_pp0_stage0_11001;
  output \B_V_data_1_state_reg[1]_1 ;
  output \B_V_data_1_state_reg[1]_2 ;
  output \B_V_data_1_state_reg[0]_1 ;
  output ap_enable_reg_pp0_iter1_reg_0;
  output [31:0]dec_out_TDATA;
  input ap_rst_n_inv;
  input ap_clk;
  input dec_out_TREADY;
  input rx_in_TVALID_int_regslice;
  input \B_V_data_1_state_reg[0]_2 ;
  input [7:0]dec_counter_reg;
  input ap_enable_reg_pp0_iter1;
  input icmp_ln40_reg_289;
  input icmp_ln40_reg_289_pp0_iter1_reg;
  input ap_enable_reg_pp0_iter2;
  input \B_V_data_1_state_reg[0]_3 ;
  input \B_V_data_1_state_reg[0]_4 ;
  input B_V_data_1_sel_wr;
  input B_V_data_1_sel;
  input [31:0]D;

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
  wire \B_V_data_1_payload_A_reg_n_0_[21] ;
  wire \B_V_data_1_payload_A_reg_n_0_[22] ;
  wire \B_V_data_1_payload_A_reg_n_0_[23] ;
  wire \B_V_data_1_payload_A_reg_n_0_[24] ;
  wire \B_V_data_1_payload_A_reg_n_0_[25] ;
  wire \B_V_data_1_payload_A_reg_n_0_[26] ;
  wire \B_V_data_1_payload_A_reg_n_0_[27] ;
  wire \B_V_data_1_payload_A_reg_n_0_[28] ;
  wire \B_V_data_1_payload_A_reg_n_0_[29] ;
  wire \B_V_data_1_payload_A_reg_n_0_[2] ;
  wire \B_V_data_1_payload_A_reg_n_0_[30] ;
  wire \B_V_data_1_payload_A_reg_n_0_[31] ;
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
  wire \B_V_data_1_payload_B_reg_n_0_[21] ;
  wire \B_V_data_1_payload_B_reg_n_0_[22] ;
  wire \B_V_data_1_payload_B_reg_n_0_[23] ;
  wire \B_V_data_1_payload_B_reg_n_0_[24] ;
  wire \B_V_data_1_payload_B_reg_n_0_[25] ;
  wire \B_V_data_1_payload_B_reg_n_0_[26] ;
  wire \B_V_data_1_payload_B_reg_n_0_[27] ;
  wire \B_V_data_1_payload_B_reg_n_0_[28] ;
  wire \B_V_data_1_payload_B_reg_n_0_[29] ;
  wire \B_V_data_1_payload_B_reg_n_0_[2] ;
  wire \B_V_data_1_payload_B_reg_n_0_[30] ;
  wire \B_V_data_1_payload_B_reg_n_0_[31] ;
  wire \B_V_data_1_payload_B_reg_n_0_[3] ;
  wire \B_V_data_1_payload_B_reg_n_0_[4] ;
  wire \B_V_data_1_payload_B_reg_n_0_[5] ;
  wire \B_V_data_1_payload_B_reg_n_0_[6] ;
  wire \B_V_data_1_payload_B_reg_n_0_[7] ;
  wire \B_V_data_1_payload_B_reg_n_0_[8] ;
  wire \B_V_data_1_payload_B_reg_n_0_[9] ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__1_n_0;
  wire B_V_data_1_sel_rd_reg_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_0;
  wire B_V_data_1_sel_wr_i_1__0_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__2_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg[0]_1 ;
  wire \B_V_data_1_state_reg[0]_2 ;
  wire \B_V_data_1_state_reg[0]_3 ;
  wire \B_V_data_1_state_reg[0]_4 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire \B_V_data_1_state_reg[1]_1 ;
  wire \B_V_data_1_state_reg[1]_2 ;
  wire [31:0]D;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter1_reg;
  wire ap_enable_reg_pp0_iter1_reg_0;
  wire ap_enable_reg_pp0_iter2;
  wire ap_rst_n_inv;
  wire \dec_counter[4]_i_2_n_0 ;
  wire \dec_counter[5]_i_2_n_0 ;
  wire [7:0]dec_counter_reg;
  wire \dec_counter_reg[5]_0 ;
  wire dec_counter_reg_0_sn_1;
  wire dec_counter_reg_2_sn_1;
  wire dec_counter_reg_3_sn_1;
  wire dec_counter_reg_4_sn_1;
  wire dec_counter_reg_5_sn_1;
  wire dec_counter_reg_6_sn_1;
  wire dec_counter_reg_7_sn_1;
  wire [31:0]dec_out_TDATA;
  wire dec_out_TREADY;
  wire icmp_ln40_reg_289;
  wire icmp_ln40_reg_289_pp0_iter1_reg;
  wire \icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ;
  wire rx_in_TVALID_int_regslice;
  wire trunc_ln2_reg_2930;

  assign dec_counter_reg_0_sp_1 = dec_counter_reg_0_sn_1;
  assign dec_counter_reg_2_sp_1 = dec_counter_reg_2_sn_1;
  assign dec_counter_reg_3_sp_1 = dec_counter_reg_3_sn_1;
  assign dec_counter_reg_4_sp_1 = dec_counter_reg_4_sn_1;
  assign dec_counter_reg_5_sp_1 = dec_counter_reg_5_sn_1;
  assign dec_counter_reg_6_sp_1 = dec_counter_reg_6_sn_1;
  assign dec_counter_reg_7_sp_1 = dec_counter_reg_7_sn_1;
  LUT3 #(
    .INIT(8'h45)) 
    \B_V_data_1_payload_A[31]_i_1 
       (.I0(B_V_data_1_sel_wr_0),
        .I1(\B_V_data_1_state_reg[1]_0 ),
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
  FDRE \B_V_data_1_payload_A_reg[21] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[21]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[21] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[22] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[22]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[22] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[23] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[23]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[23] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[24] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[24]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[24] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[25] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[25]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[25] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[26] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[26]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[26] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[27] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[27]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[27] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[28] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[28]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[28] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[29] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[29]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[29] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[30] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[30]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[30] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[31] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(D[31]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[31] ),
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
    \B_V_data_1_payload_B[31]_i_1 
       (.I0(B_V_data_1_sel_wr_0),
        .I1(\B_V_data_1_state_reg[1]_0 ),
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
  FDRE \B_V_data_1_payload_B_reg[21] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[21]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[21] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[22] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[22]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[22] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[23] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[23]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[23] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[24] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[24]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[24] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[25] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[25]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[25] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[26] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[26]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[26] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[27] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[27]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[27] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[28] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[28]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[28] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[29] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[29]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[29] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[30] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[30]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[30] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[31] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(D[31]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[31] ),
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
  LUT6 #(
    .INIT(64'h08FFFFFFF7000000)) 
    B_V_data_1_sel_rd_i_1__0
       (.I0(ap_enable_reg_pp0_iter1),
        .I1(icmp_ln40_reg_289),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(rx_in_TVALID_int_regslice),
        .I4(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I5(B_V_data_1_sel),
        .O(ap_enable_reg_pp0_iter1_reg_0));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__1
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(dec_out_TREADY),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(B_V_data_1_sel_rd_i_1__1_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__1_n_0),
        .Q(B_V_data_1_sel_rd_reg_n_0),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'hFF7FFFFF00800000)) 
    B_V_data_1_sel_wr_i_1
       (.I0(\B_V_data_1_state_reg[1]_0 ),
        .I1(rx_in_TVALID_int_regslice),
        .I2(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I3(\B_V_data_1_state_reg[0]_2 ),
        .I4(\B_V_data_1_state_reg[0]_3 ),
        .I5(B_V_data_1_sel_wr),
        .O(\B_V_data_1_state_reg[1]_2 ));
  LUT6 #(
    .INIT(64'h7FFFFFFF80000000)) 
    B_V_data_1_sel_wr_i_1__0
       (.I0(ap_enable_reg_pp0_iter1),
        .I1(icmp_ln40_reg_289),
        .I2(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I3(rx_in_TVALID_int_regslice),
        .I4(\B_V_data_1_state_reg[1]_0 ),
        .I5(B_V_data_1_sel_wr_0),
        .O(B_V_data_1_sel_wr_i_1__0_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__0_n_0),
        .Q(B_V_data_1_sel_wr_0),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'h00800000FFFFFFFF)) 
    \B_V_data_1_state[0]_i_1 
       (.I0(\B_V_data_1_state_reg[1]_0 ),
        .I1(rx_in_TVALID_int_regslice),
        .I2(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I3(\B_V_data_1_state_reg[0]_2 ),
        .I4(\B_V_data_1_state_reg[0]_3 ),
        .I5(\B_V_data_1_state_reg[0]_4 ),
        .O(\B_V_data_1_state_reg[1]_1 ));
  LUT5 #(
    .INIT(32'h70FC7070)) 
    \B_V_data_1_state[0]_i_1__2 
       (.I0(dec_out_TREADY),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(\B_V_data_1_state_reg[0]_2 ),
        .I4(rx_in_TVALID_int_regslice),
        .O(\B_V_data_1_state[0]_i_1__2_n_0 ));
  LUT6 #(
    .INIT(64'hFDFDFDFDDDFDFDFD)) 
    \B_V_data_1_state[1]_i_1 
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(dec_out_TREADY),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(rx_in_TVALID_int_regslice),
        .I4(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I5(\B_V_data_1_state_reg[0]_2 ),
        .O(B_V_data_1_state));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'h08FF)) 
    \B_V_data_1_state[1]_i_2__1 
       (.I0(ap_enable_reg_pp0_iter1),
        .I1(icmp_ln40_reg_289),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(rx_in_TVALID_int_regslice),
        .O(ap_enable_reg_pp0_iter1_reg));
  LUT5 #(
    .INIT(32'hC0DDFFFF)) 
    \B_V_data_1_state[1]_i_3 
       (.I0(icmp_ln40_reg_289_pp0_iter1_reg),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(dec_out_TREADY),
        .I3(\B_V_data_1_state_reg[0]_0 ),
        .I4(ap_enable_reg_pp0_iter2),
        .O(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__2_n_0 ),
        .Q(\B_V_data_1_state_reg[0]_0 ),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg[1]_0 ),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'hFFFF777780880000)) 
    ap_enable_reg_pp0_iter2_i_1
       (.I0(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I1(rx_in_TVALID_int_regslice),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(icmp_ln40_reg_289),
        .I4(ap_enable_reg_pp0_iter1),
        .I5(ap_enable_reg_pp0_iter2),
        .O(\B_V_data_1_state_reg[0]_1 ));
  LUT6 #(
    .INIT(64'h5595AAAAAAAAAAAA)) 
    \dec_counter[0]_i_1 
       (.I0(dec_counter_reg[0]),
        .I1(ap_enable_reg_pp0_iter1),
        .I2(icmp_ln40_reg_289),
        .I3(\B_V_data_1_state_reg[1]_0 ),
        .I4(rx_in_TVALID_int_regslice),
        .I5(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .O(dec_counter_reg_0_sn_1));
  LUT5 #(
    .INIT(32'hAAAA6AAA)) 
    \dec_counter[2]_i_1 
       (.I0(dec_counter_reg[2]),
        .I1(dec_counter_reg[1]),
        .I2(dec_counter_reg[0]),
        .I3(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I4(ap_enable_reg_pp0_iter1_reg),
        .O(dec_counter_reg_2_sn_1));
  LUT6 #(
    .INIT(64'h9AAAAAAAAAAAAAAA)) 
    \dec_counter[3]_i_1 
       (.I0(dec_counter_reg[3]),
        .I1(ap_enable_reg_pp0_iter1_reg),
        .I2(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I3(dec_counter_reg[0]),
        .I4(dec_counter_reg[1]),
        .I5(dec_counter_reg[2]),
        .O(dec_counter_reg_3_sn_1));
  LUT6 #(
    .INIT(64'h6AAA6AAA6AAAAAAA)) 
    \dec_counter[4]_i_1 
       (.I0(dec_counter_reg[4]),
        .I1(\dec_counter[4]_i_2_n_0 ),
        .I2(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I3(rx_in_TVALID_int_regslice),
        .I4(\B_V_data_1_state_reg[1]_0 ),
        .I5(\B_V_data_1_state_reg[0]_2 ),
        .O(dec_counter_reg_4_sn_1));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT4 #(
    .INIT(16'h8000)) 
    \dec_counter[4]_i_2 
       (.I0(dec_counter_reg[0]),
        .I1(dec_counter_reg[1]),
        .I2(dec_counter_reg[3]),
        .I3(dec_counter_reg[2]),
        .O(\dec_counter[4]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h9AAA9AAA9AAAAAAA)) 
    \dec_counter[5]_i_1 
       (.I0(dec_counter_reg[5]),
        .I1(\dec_counter[5]_i_2_n_0 ),
        .I2(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I3(rx_in_TVALID_int_regslice),
        .I4(\B_V_data_1_state_reg[1]_0 ),
        .I5(\B_V_data_1_state_reg[0]_2 ),
        .O(\dec_counter_reg[5]_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h7FFFFFFF)) 
    \dec_counter[5]_i_2 
       (.I0(dec_counter_reg[4]),
        .I1(dec_counter_reg[2]),
        .I2(dec_counter_reg[3]),
        .I3(dec_counter_reg[1]),
        .I4(dec_counter_reg[0]),
        .O(\dec_counter[5]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h6AAA6AAA6AAAAAAA)) 
    \dec_counter[6]_i_1 
       (.I0(dec_counter_reg[6]),
        .I1(dec_counter_reg_5_sn_1),
        .I2(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I3(rx_in_TVALID_int_regslice),
        .I4(\B_V_data_1_state_reg[1]_0 ),
        .I5(\B_V_data_1_state_reg[0]_2 ),
        .O(dec_counter_reg_6_sn_1));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'hAAAA6AAA)) 
    \dec_counter[7]_i_1 
       (.I0(dec_counter_reg[7]),
        .I1(dec_counter_reg[6]),
        .I2(dec_counter_reg_5_sn_1),
        .I3(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I4(ap_enable_reg_pp0_iter1_reg),
        .O(dec_counter_reg_7_sn_1));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[0]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[0]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[10]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[10]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[11]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[11]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[12]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[12]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[13]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[13]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[14]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[14]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[15]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[15]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[16]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[16]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[17]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[17]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[18]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[18]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[19]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[19]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[1]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[1]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[20]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[20]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[21]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[21] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[21] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[21]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[22]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[22] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[22] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[22]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[23]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[23] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[23] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[23]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[24]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[24] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[24] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[24]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[25]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[25] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[25] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[25]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[26]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[26] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[26] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[26]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[27]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[27] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[27] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[27]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[28]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[28] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[28] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[28]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[29]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[29] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[29] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[29]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[2]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[2]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[30]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[30] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[30] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[30]));
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[31]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[31]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[3]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[3]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[4]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[4]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[5]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[5]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[6]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[6]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[7]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[7]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[8]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[8]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \dec_out_TDATA[9]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(dec_out_TDATA[9]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT5 #(
    .INIT(32'h80888888)) 
    \pkt_rx_last_V_reg_284[0]_i_1 
       (.I0(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I1(rx_in_TVALID_int_regslice),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(icmp_ln40_reg_289),
        .I4(ap_enable_reg_pp0_iter1),
        .O(ap_block_pp0_stage0_11001));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h40000000)) 
    \trunc_ln2_reg_293[15]_i_1 
       (.I0(ap_enable_reg_pp0_iter1_reg),
        .I1(\icmp_ln40_reg_289_pp0_iter1_reg_reg[0] ),
        .I2(dec_counter_reg[6]),
        .I3(dec_counter_reg_5_sn_1),
        .I4(dec_counter_reg[7]),
        .O(trunc_ln2_reg_2930));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    \trunc_ln2_reg_293[15]_i_3 
       (.I0(dec_counter_reg[5]),
        .I1(dec_counter_reg[0]),
        .I2(dec_counter_reg[1]),
        .I3(dec_counter_reg[3]),
        .I4(dec_counter_reg[2]),
        .I5(dec_counter_reg[4]),
        .O(dec_counter_reg_5_sn_1));
endmodule

(* ORIG_REF_NAME = "fsk_decimator_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both_0
   (\B_V_data_1_state_reg[1]_0 ,
    rx_in_TVALID_int_regslice,
    B_V_data_1_sel,
    O,
    \acc_i_reg[7] ,
    \acc_i_reg[11] ,
    \acc_i_reg[14] ,
    \acc_i_reg[14]_0 ,
    \B_V_data_1_payload_B_reg[15]_0 ,
    D,
    \acc_q_reg[3] ,
    \acc_q_reg[7] ,
    \acc_q_reg[11] ,
    \acc_q_reg[14] ,
    \acc_q_reg[14]_0 ,
    \B_V_data_1_payload_B_reg[31]_0 ,
    \acc_q_reg[21] ,
    \B_V_data_1_state_reg[0]_0 ,
    ap_rst_n_inv,
    ap_clk,
    B_V_data_1_sel_rd_reg_0,
    \B_V_data_1_state_reg[0]_1 ,
    \B_V_data_1_state_reg[0]_2 ,
    \B_V_data_1_state_reg[0]_3 ,
    rx_in_TVALID,
    acc_i_reg,
    acc_q_reg,
    ap_enable_reg_pp0_iter1,
    rx_in_TDATA);
  output \B_V_data_1_state_reg[1]_0 ;
  output rx_in_TVALID_int_regslice;
  output B_V_data_1_sel;
  output [3:0]O;
  output [3:0]\acc_i_reg[7] ;
  output [3:0]\acc_i_reg[11] ;
  output [3:0]\acc_i_reg[14] ;
  output [3:0]\acc_i_reg[14]_0 ;
  output [3:0]\B_V_data_1_payload_B_reg[15]_0 ;
  output [15:0]D;
  output [3:0]\acc_q_reg[3] ;
  output [3:0]\acc_q_reg[7] ;
  output [3:0]\acc_q_reg[11] ;
  output [3:0]\acc_q_reg[14] ;
  output [3:0]\acc_q_reg[14]_0 ;
  output [3:0]\B_V_data_1_payload_B_reg[31]_0 ;
  output [15:0]\acc_q_reg[21] ;
  output \B_V_data_1_state_reg[0]_0 ;
  input ap_rst_n_inv;
  input ap_clk;
  input B_V_data_1_sel_rd_reg_0;
  input \B_V_data_1_state_reg[0]_1 ;
  input \B_V_data_1_state_reg[0]_2 ;
  input \B_V_data_1_state_reg[0]_3 ;
  input rx_in_TVALID;
  input [23:0]acc_i_reg;
  input [23:0]acc_q_reg;
  input ap_enable_reg_pp0_iter1;
  input [31:0]rx_in_TDATA;

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
  wire \B_V_data_1_payload_A_reg_n_0_[21] ;
  wire \B_V_data_1_payload_A_reg_n_0_[22] ;
  wire \B_V_data_1_payload_A_reg_n_0_[23] ;
  wire \B_V_data_1_payload_A_reg_n_0_[24] ;
  wire \B_V_data_1_payload_A_reg_n_0_[25] ;
  wire \B_V_data_1_payload_A_reg_n_0_[26] ;
  wire \B_V_data_1_payload_A_reg_n_0_[27] ;
  wire \B_V_data_1_payload_A_reg_n_0_[28] ;
  wire \B_V_data_1_payload_A_reg_n_0_[29] ;
  wire \B_V_data_1_payload_A_reg_n_0_[2] ;
  wire \B_V_data_1_payload_A_reg_n_0_[30] ;
  wire \B_V_data_1_payload_A_reg_n_0_[31] ;
  wire \B_V_data_1_payload_A_reg_n_0_[3] ;
  wire \B_V_data_1_payload_A_reg_n_0_[4] ;
  wire \B_V_data_1_payload_A_reg_n_0_[5] ;
  wire \B_V_data_1_payload_A_reg_n_0_[6] ;
  wire \B_V_data_1_payload_A_reg_n_0_[7] ;
  wire \B_V_data_1_payload_A_reg_n_0_[8] ;
  wire \B_V_data_1_payload_A_reg_n_0_[9] ;
  wire [3:0]\B_V_data_1_payload_B_reg[15]_0 ;
  wire [3:0]\B_V_data_1_payload_B_reg[31]_0 ;
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
  wire \B_V_data_1_payload_B_reg_n_0_[21] ;
  wire \B_V_data_1_payload_B_reg_n_0_[22] ;
  wire \B_V_data_1_payload_B_reg_n_0_[23] ;
  wire \B_V_data_1_payload_B_reg_n_0_[24] ;
  wire \B_V_data_1_payload_B_reg_n_0_[25] ;
  wire \B_V_data_1_payload_B_reg_n_0_[26] ;
  wire \B_V_data_1_payload_B_reg_n_0_[27] ;
  wire \B_V_data_1_payload_B_reg_n_0_[28] ;
  wire \B_V_data_1_payload_B_reg_n_0_[29] ;
  wire \B_V_data_1_payload_B_reg_n_0_[2] ;
  wire \B_V_data_1_payload_B_reg_n_0_[30] ;
  wire \B_V_data_1_payload_B_reg_n_0_[31] ;
  wire \B_V_data_1_payload_B_reg_n_0_[3] ;
  wire \B_V_data_1_payload_B_reg_n_0_[4] ;
  wire \B_V_data_1_payload_B_reg_n_0_[5] ;
  wire \B_V_data_1_payload_B_reg_n_0_[6] ;
  wire \B_V_data_1_payload_B_reg_n_0_[7] ;
  wire \B_V_data_1_payload_B_reg_n_0_[8] ;
  wire \B_V_data_1_payload_B_reg_n_0_[9] ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_reg_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__1_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg[0]_1 ;
  wire \B_V_data_1_state_reg[0]_2 ;
  wire \B_V_data_1_state_reg[0]_3 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire [15:0]D;
  wire [3:0]O;
  wire \acc_i[0]_i_2_n_0 ;
  wire \acc_i[0]_i_3_n_0 ;
  wire \acc_i[0]_i_4_n_0 ;
  wire \acc_i[0]_i_5_n_0 ;
  wire \acc_i[12]_i_2_n_0 ;
  wire \acc_i[12]_i_3_n_0 ;
  wire \acc_i[12]_i_4_n_0 ;
  wire \acc_i[12]_i_5_n_0 ;
  wire \acc_i[12]_i_6_n_0 ;
  wire \acc_i[16]_i_2_n_0 ;
  wire \acc_i[16]_i_3_n_0 ;
  wire \acc_i[16]_i_4_n_0 ;
  wire \acc_i[16]_i_5_n_0 ;
  wire \acc_i[16]_i_6_n_0 ;
  wire \acc_i[16]_i_7_n_0 ;
  wire \acc_i[16]_i_8_n_0 ;
  wire \acc_i[16]_i_9_n_0 ;
  wire \acc_i[20]_i_2_n_0 ;
  wire \acc_i[20]_i_3_n_0 ;
  wire \acc_i[20]_i_4_n_0 ;
  wire \acc_i[20]_i_5_n_0 ;
  wire \acc_i[20]_i_6_n_0 ;
  wire \acc_i[20]_i_7_n_0 ;
  wire \acc_i[20]_i_8_n_0 ;
  wire \acc_i[4]_i_2_n_0 ;
  wire \acc_i[4]_i_3_n_0 ;
  wire \acc_i[4]_i_4_n_0 ;
  wire \acc_i[4]_i_5_n_0 ;
  wire \acc_i[8]_i_2_n_0 ;
  wire \acc_i[8]_i_3_n_0 ;
  wire \acc_i[8]_i_4_n_0 ;
  wire \acc_i[8]_i_5_n_0 ;
  wire [23:0]acc_i_reg;
  wire \acc_i_reg[0]_i_1_n_0 ;
  wire \acc_i_reg[0]_i_1_n_1 ;
  wire \acc_i_reg[0]_i_1_n_2 ;
  wire \acc_i_reg[0]_i_1_n_3 ;
  wire [3:0]\acc_i_reg[11] ;
  wire \acc_i_reg[12]_i_1_n_0 ;
  wire \acc_i_reg[12]_i_1_n_1 ;
  wire \acc_i_reg[12]_i_1_n_2 ;
  wire \acc_i_reg[12]_i_1_n_3 ;
  wire [3:0]\acc_i_reg[14] ;
  wire [3:0]\acc_i_reg[14]_0 ;
  wire \acc_i_reg[16]_i_1_n_0 ;
  wire \acc_i_reg[16]_i_1_n_1 ;
  wire \acc_i_reg[16]_i_1_n_2 ;
  wire \acc_i_reg[16]_i_1_n_3 ;
  wire \acc_i_reg[20]_i_1_n_1 ;
  wire \acc_i_reg[20]_i_1_n_2 ;
  wire \acc_i_reg[20]_i_1_n_3 ;
  wire \acc_i_reg[4]_i_1_n_0 ;
  wire \acc_i_reg[4]_i_1_n_1 ;
  wire \acc_i_reg[4]_i_1_n_2 ;
  wire \acc_i_reg[4]_i_1_n_3 ;
  wire [3:0]\acc_i_reg[7] ;
  wire \acc_i_reg[8]_i_1_n_0 ;
  wire \acc_i_reg[8]_i_1_n_1 ;
  wire \acc_i_reg[8]_i_1_n_2 ;
  wire \acc_i_reg[8]_i_1_n_3 ;
  wire \acc_q[0]_i_2_n_0 ;
  wire \acc_q[0]_i_3_n_0 ;
  wire \acc_q[0]_i_4_n_0 ;
  wire \acc_q[0]_i_5_n_0 ;
  wire \acc_q[12]_i_2_n_0 ;
  wire \acc_q[12]_i_3_n_0 ;
  wire \acc_q[12]_i_4_n_0 ;
  wire \acc_q[12]_i_5_n_0 ;
  wire \acc_q[12]_i_6_n_0 ;
  wire \acc_q[16]_i_2_n_0 ;
  wire \acc_q[16]_i_3_n_0 ;
  wire \acc_q[16]_i_4_n_0 ;
  wire \acc_q[16]_i_5_n_0 ;
  wire \acc_q[16]_i_6_n_0 ;
  wire \acc_q[16]_i_7_n_0 ;
  wire \acc_q[16]_i_8_n_0 ;
  wire \acc_q[16]_i_9_n_0 ;
  wire \acc_q[20]_i_2_n_0 ;
  wire \acc_q[20]_i_3_n_0 ;
  wire \acc_q[20]_i_4_n_0 ;
  wire \acc_q[20]_i_5_n_0 ;
  wire \acc_q[20]_i_6_n_0 ;
  wire \acc_q[20]_i_7_n_0 ;
  wire \acc_q[20]_i_8_n_0 ;
  wire \acc_q[4]_i_2_n_0 ;
  wire \acc_q[4]_i_3_n_0 ;
  wire \acc_q[4]_i_4_n_0 ;
  wire \acc_q[4]_i_5_n_0 ;
  wire \acc_q[8]_i_2_n_0 ;
  wire \acc_q[8]_i_3_n_0 ;
  wire \acc_q[8]_i_4_n_0 ;
  wire \acc_q[8]_i_5_n_0 ;
  wire [23:0]acc_q_reg;
  wire \acc_q_reg[0]_i_1_n_0 ;
  wire \acc_q_reg[0]_i_1_n_1 ;
  wire \acc_q_reg[0]_i_1_n_2 ;
  wire \acc_q_reg[0]_i_1_n_3 ;
  wire [3:0]\acc_q_reg[11] ;
  wire \acc_q_reg[12]_i_1_n_0 ;
  wire \acc_q_reg[12]_i_1_n_1 ;
  wire \acc_q_reg[12]_i_1_n_2 ;
  wire \acc_q_reg[12]_i_1_n_3 ;
  wire [3:0]\acc_q_reg[14] ;
  wire [3:0]\acc_q_reg[14]_0 ;
  wire \acc_q_reg[16]_i_1_n_0 ;
  wire \acc_q_reg[16]_i_1_n_1 ;
  wire \acc_q_reg[16]_i_1_n_2 ;
  wire \acc_q_reg[16]_i_1_n_3 ;
  wire \acc_q_reg[20]_i_1_n_1 ;
  wire \acc_q_reg[20]_i_1_n_2 ;
  wire \acc_q_reg[20]_i_1_n_3 ;
  wire [15:0]\acc_q_reg[21] ;
  wire [3:0]\acc_q_reg[3] ;
  wire \acc_q_reg[4]_i_1_n_0 ;
  wire \acc_q_reg[4]_i_1_n_1 ;
  wire \acc_q_reg[4]_i_1_n_2 ;
  wire \acc_q_reg[4]_i_1_n_3 ;
  wire [3:0]\acc_q_reg[7] ;
  wire \acc_q_reg[8]_i_1_n_0 ;
  wire \acc_q_reg[8]_i_1_n_1 ;
  wire \acc_q_reg[8]_i_1_n_2 ;
  wire \acc_q_reg[8]_i_1_n_3 ;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_rst_n_inv;
  wire [31:0]rx_in_TDATA;
  wire rx_in_TVALID;
  wire rx_in_TVALID_int_regslice;
  wire \trunc_ln2_reg_293[11]_i_2_n_0 ;
  wire \trunc_ln2_reg_293[11]_i_3_n_0 ;
  wire \trunc_ln2_reg_293[11]_i_4_n_0 ;
  wire \trunc_ln2_reg_293[11]_i_5_n_0 ;
  wire \trunc_ln2_reg_293[11]_i_6_n_0 ;
  wire \trunc_ln2_reg_293[15]_i_4_n_0 ;
  wire \trunc_ln2_reg_293[15]_i_5_n_0 ;
  wire \trunc_ln2_reg_293[15]_i_6_n_0 ;
  wire \trunc_ln2_reg_293[15]_i_7_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_10_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_11_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_12_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_13_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_14_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_15_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_3_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_4_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_5_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_6_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_8_n_0 ;
  wire \trunc_ln2_reg_293[3]_i_9_n_0 ;
  wire \trunc_ln2_reg_293[7]_i_2_n_0 ;
  wire \trunc_ln2_reg_293[7]_i_3_n_0 ;
  wire \trunc_ln2_reg_293[7]_i_4_n_0 ;
  wire \trunc_ln2_reg_293[7]_i_5_n_0 ;
  wire \trunc_ln2_reg_293[7]_i_6_n_0 ;
  wire \trunc_ln2_reg_293_reg[11]_i_1_n_0 ;
  wire \trunc_ln2_reg_293_reg[11]_i_1_n_1 ;
  wire \trunc_ln2_reg_293_reg[11]_i_1_n_2 ;
  wire \trunc_ln2_reg_293_reg[11]_i_1_n_3 ;
  wire \trunc_ln2_reg_293_reg[15]_i_2_n_1 ;
  wire \trunc_ln2_reg_293_reg[15]_i_2_n_2 ;
  wire \trunc_ln2_reg_293_reg[15]_i_2_n_3 ;
  wire \trunc_ln2_reg_293_reg[3]_i_1_n_0 ;
  wire \trunc_ln2_reg_293_reg[3]_i_1_n_1 ;
  wire \trunc_ln2_reg_293_reg[3]_i_1_n_2 ;
  wire \trunc_ln2_reg_293_reg[3]_i_1_n_3 ;
  wire \trunc_ln2_reg_293_reg[3]_i_2_n_0 ;
  wire \trunc_ln2_reg_293_reg[3]_i_2_n_1 ;
  wire \trunc_ln2_reg_293_reg[3]_i_2_n_2 ;
  wire \trunc_ln2_reg_293_reg[3]_i_2_n_3 ;
  wire \trunc_ln2_reg_293_reg[3]_i_7_n_0 ;
  wire \trunc_ln2_reg_293_reg[3]_i_7_n_1 ;
  wire \trunc_ln2_reg_293_reg[3]_i_7_n_2 ;
  wire \trunc_ln2_reg_293_reg[3]_i_7_n_3 ;
  wire \trunc_ln2_reg_293_reg[7]_i_1_n_0 ;
  wire \trunc_ln2_reg_293_reg[7]_i_1_n_1 ;
  wire \trunc_ln2_reg_293_reg[7]_i_1_n_2 ;
  wire \trunc_ln2_reg_293_reg[7]_i_1_n_3 ;
  wire \trunc_ln3_reg_298[11]_i_2_n_0 ;
  wire \trunc_ln3_reg_298[11]_i_3_n_0 ;
  wire \trunc_ln3_reg_298[11]_i_4_n_0 ;
  wire \trunc_ln3_reg_298[11]_i_5_n_0 ;
  wire \trunc_ln3_reg_298[11]_i_6_n_0 ;
  wire \trunc_ln3_reg_298[15]_i_2_n_0 ;
  wire \trunc_ln3_reg_298[15]_i_3_n_0 ;
  wire \trunc_ln3_reg_298[15]_i_4_n_0 ;
  wire \trunc_ln3_reg_298[15]_i_5_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_10_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_11_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_12_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_13_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_14_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_15_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_3_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_4_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_5_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_6_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_8_n_0 ;
  wire \trunc_ln3_reg_298[3]_i_9_n_0 ;
  wire \trunc_ln3_reg_298[7]_i_2_n_0 ;
  wire \trunc_ln3_reg_298[7]_i_3_n_0 ;
  wire \trunc_ln3_reg_298[7]_i_4_n_0 ;
  wire \trunc_ln3_reg_298[7]_i_5_n_0 ;
  wire \trunc_ln3_reg_298[7]_i_6_n_0 ;
  wire \trunc_ln3_reg_298_reg[11]_i_1_n_0 ;
  wire \trunc_ln3_reg_298_reg[11]_i_1_n_1 ;
  wire \trunc_ln3_reg_298_reg[11]_i_1_n_2 ;
  wire \trunc_ln3_reg_298_reg[11]_i_1_n_3 ;
  wire \trunc_ln3_reg_298_reg[15]_i_1_n_1 ;
  wire \trunc_ln3_reg_298_reg[15]_i_1_n_2 ;
  wire \trunc_ln3_reg_298_reg[15]_i_1_n_3 ;
  wire \trunc_ln3_reg_298_reg[3]_i_1_n_0 ;
  wire \trunc_ln3_reg_298_reg[3]_i_1_n_1 ;
  wire \trunc_ln3_reg_298_reg[3]_i_1_n_2 ;
  wire \trunc_ln3_reg_298_reg[3]_i_1_n_3 ;
  wire \trunc_ln3_reg_298_reg[3]_i_2_n_0 ;
  wire \trunc_ln3_reg_298_reg[3]_i_2_n_1 ;
  wire \trunc_ln3_reg_298_reg[3]_i_2_n_2 ;
  wire \trunc_ln3_reg_298_reg[3]_i_2_n_3 ;
  wire \trunc_ln3_reg_298_reg[3]_i_7_n_0 ;
  wire \trunc_ln3_reg_298_reg[3]_i_7_n_1 ;
  wire \trunc_ln3_reg_298_reg[3]_i_7_n_2 ;
  wire \trunc_ln3_reg_298_reg[3]_i_7_n_3 ;
  wire \trunc_ln3_reg_298_reg[7]_i_1_n_0 ;
  wire \trunc_ln3_reg_298_reg[7]_i_1_n_1 ;
  wire \trunc_ln3_reg_298_reg[7]_i_1_n_2 ;
  wire \trunc_ln3_reg_298_reg[7]_i_1_n_3 ;
  wire [3:3]\NLW_acc_i_reg[20]_i_1_CO_UNCONNECTED ;
  wire [3:3]\NLW_acc_q_reg[20]_i_1_CO_UNCONNECTED ;
  wire [3:3]\NLW_trunc_ln2_reg_293_reg[15]_i_2_CO_UNCONNECTED ;
  wire [3:0]\NLW_trunc_ln2_reg_293_reg[3]_i_2_O_UNCONNECTED ;
  wire [3:0]\NLW_trunc_ln2_reg_293_reg[3]_i_7_O_UNCONNECTED ;
  wire [3:3]\NLW_trunc_ln3_reg_298_reg[15]_i_1_CO_UNCONNECTED ;
  wire [3:0]\NLW_trunc_ln3_reg_298_reg[3]_i_2_O_UNCONNECTED ;
  wire [3:0]\NLW_trunc_ln3_reg_298_reg[3]_i_7_O_UNCONNECTED ;

  LUT3 #(
    .INIT(8'h45)) 
    \B_V_data_1_payload_A[31]_i_1__0 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(rx_in_TVALID_int_regslice),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[0]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[10]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[11]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[12]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[13]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[14]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[15] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[15]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[16] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[16]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[17] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[17]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[18] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[18]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[19] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[19]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[1]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[20] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[20]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[21] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[21]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[21] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[22] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[22]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[22] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[23] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[23]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[23] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[24] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[24]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[24] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[25] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[25]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[25] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[26] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[26]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[26] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[27] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[27]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[27] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[28] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[28]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[28] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[29] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[29]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[29] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[30] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[30]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[30] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[31] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[31]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[3]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[4]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[5]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[6]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[7]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[8]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(rx_in_TDATA[9]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .R(1'b0));
  LUT3 #(
    .INIT(8'h8A)) 
    \B_V_data_1_payload_B[31]_i_1__0 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(rx_in_TVALID_int_regslice),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[0]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[10]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[11]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[12]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[13]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[14]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[15] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[15]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[16] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[16]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[17] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[17]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[18] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[18]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[19] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[19]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[1]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[20] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[20]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[21] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[21]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[21] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[22] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[22]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[22] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[23] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[23]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[23] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[24] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[24]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[24] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[25] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[25]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[25] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[26] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[26]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[26] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[27] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[27]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[27] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[28] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[28]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[28] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[29] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[29]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[29] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[30] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[30]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[30] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[31] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[31]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[3]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[4]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[5]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[6]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[7]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[8]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(rx_in_TDATA[9]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .R(1'b0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_reg_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__1
       (.I0(rx_in_TVALID),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'hD8F8D8F8D8F8F8F8)) 
    \B_V_data_1_state[0]_i_1__1 
       (.I0(\B_V_data_1_state_reg[1]_0 ),
        .I1(rx_in_TVALID),
        .I2(rx_in_TVALID_int_regslice),
        .I3(\B_V_data_1_state_reg[0]_1 ),
        .I4(\B_V_data_1_state_reg[0]_2 ),
        .I5(\B_V_data_1_state_reg[0]_3 ),
        .O(\B_V_data_1_state[0]_i_1__1_n_0 ));
  LUT6 #(
    .INIT(64'hDDD5FFFFDDD5DDD5)) 
    \B_V_data_1_state[1]_i_2 
       (.I0(rx_in_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .I2(\B_V_data_1_state_reg[0]_2 ),
        .I3(\B_V_data_1_state_reg[0]_3 ),
        .I4(rx_in_TVALID),
        .I5(\B_V_data_1_state_reg[1]_0 ),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__1_n_0 ),
        .Q(rx_in_TVALID_int_regslice),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg[1]_0 ),
        .R(ap_rst_n_inv));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[0]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I3(acc_i_reg[3]),
        .O(\acc_i[0]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[0]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I3(acc_i_reg[2]),
        .O(\acc_i[0]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[0]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I3(acc_i_reg[1]),
        .O(\acc_i[0]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[0]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I3(acc_i_reg[0]),
        .O(\acc_i[0]_i_5_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_i[12]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\acc_i[12]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[12]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[15]),
        .O(\acc_i[12]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[12]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I3(acc_i_reg[14]),
        .O(\acc_i[12]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[12]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I3(acc_i_reg[13]),
        .O(\acc_i[12]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[12]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I3(acc_i_reg[12]),
        .O(\acc_i[12]_i_6_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_i[16]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\acc_i[16]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_i[16]_i_3 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\acc_i[16]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_i[16]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\acc_i[16]_i_4_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_i[16]_i_5 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\acc_i[16]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[16]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[19]),
        .O(\acc_i[16]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[16]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[18]),
        .O(\acc_i[16]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[16]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[17]),
        .O(\acc_i[16]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[16]_i_9 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[16]),
        .O(\acc_i[16]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_i[20]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\acc_i[20]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_i[20]_i_3 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\acc_i[20]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_i[20]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\acc_i[20]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \acc_i[20]_i_5 
       (.I0(acc_i_reg[23]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .O(\acc_i[20]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[20]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[22]),
        .O(\acc_i[20]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[20]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[21]),
        .O(\acc_i[20]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[20]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[20]),
        .O(\acc_i[20]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[4]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I3(acc_i_reg[7]),
        .O(\acc_i[4]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[4]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I3(acc_i_reg[6]),
        .O(\acc_i[4]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[4]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I3(acc_i_reg[5]),
        .O(\acc_i[4]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[4]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I3(acc_i_reg[4]),
        .O(\acc_i[4]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[8]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I3(acc_i_reg[11]),
        .O(\acc_i[8]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[8]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I3(acc_i_reg[10]),
        .O(\acc_i[8]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[8]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I3(acc_i_reg[9]),
        .O(\acc_i[8]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_i[8]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I3(acc_i_reg[8]),
        .O(\acc_i[8]_i_5_n_0 ));
  CARRY4 \acc_i_reg[0]_i_1 
       (.CI(1'b0),
        .CO({\acc_i_reg[0]_i_1_n_0 ,\acc_i_reg[0]_i_1_n_1 ,\acc_i_reg[0]_i_1_n_2 ,\acc_i_reg[0]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_i_reg[3:0]),
        .O(O),
        .S({\acc_i[0]_i_2_n_0 ,\acc_i[0]_i_3_n_0 ,\acc_i[0]_i_4_n_0 ,\acc_i[0]_i_5_n_0 }));
  CARRY4 \acc_i_reg[12]_i_1 
       (.CI(\acc_i_reg[8]_i_1_n_0 ),
        .CO({\acc_i_reg[12]_i_1_n_0 ,\acc_i_reg[12]_i_1_n_1 ,\acc_i_reg[12]_i_1_n_2 ,\acc_i_reg[12]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_i[12]_i_2_n_0 ,acc_i_reg[14:12]}),
        .O(\acc_i_reg[14] ),
        .S({\acc_i[12]_i_3_n_0 ,\acc_i[12]_i_4_n_0 ,\acc_i[12]_i_5_n_0 ,\acc_i[12]_i_6_n_0 }));
  CARRY4 \acc_i_reg[16]_i_1 
       (.CI(\acc_i_reg[12]_i_1_n_0 ),
        .CO({\acc_i_reg[16]_i_1_n_0 ,\acc_i_reg[16]_i_1_n_1 ,\acc_i_reg[16]_i_1_n_2 ,\acc_i_reg[16]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_i[16]_i_2_n_0 ,\acc_i[16]_i_3_n_0 ,\acc_i[16]_i_4_n_0 ,\acc_i[16]_i_5_n_0 }),
        .O(\acc_i_reg[14]_0 ),
        .S({\acc_i[16]_i_6_n_0 ,\acc_i[16]_i_7_n_0 ,\acc_i[16]_i_8_n_0 ,\acc_i[16]_i_9_n_0 }));
  CARRY4 \acc_i_reg[20]_i_1 
       (.CI(\acc_i_reg[16]_i_1_n_0 ),
        .CO({\NLW_acc_i_reg[20]_i_1_CO_UNCONNECTED [3],\acc_i_reg[20]_i_1_n_1 ,\acc_i_reg[20]_i_1_n_2 ,\acc_i_reg[20]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,\acc_i[20]_i_2_n_0 ,\acc_i[20]_i_3_n_0 ,\acc_i[20]_i_4_n_0 }),
        .O(\B_V_data_1_payload_B_reg[15]_0 ),
        .S({\acc_i[20]_i_5_n_0 ,\acc_i[20]_i_6_n_0 ,\acc_i[20]_i_7_n_0 ,\acc_i[20]_i_8_n_0 }));
  CARRY4 \acc_i_reg[4]_i_1 
       (.CI(\acc_i_reg[0]_i_1_n_0 ),
        .CO({\acc_i_reg[4]_i_1_n_0 ,\acc_i_reg[4]_i_1_n_1 ,\acc_i_reg[4]_i_1_n_2 ,\acc_i_reg[4]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_i_reg[7:4]),
        .O(\acc_i_reg[7] ),
        .S({\acc_i[4]_i_2_n_0 ,\acc_i[4]_i_3_n_0 ,\acc_i[4]_i_4_n_0 ,\acc_i[4]_i_5_n_0 }));
  CARRY4 \acc_i_reg[8]_i_1 
       (.CI(\acc_i_reg[4]_i_1_n_0 ),
        .CO({\acc_i_reg[8]_i_1_n_0 ,\acc_i_reg[8]_i_1_n_1 ,\acc_i_reg[8]_i_1_n_2 ,\acc_i_reg[8]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_i_reg[11:8]),
        .O(\acc_i_reg[11] ),
        .S({\acc_i[8]_i_2_n_0 ,\acc_i[8]_i_3_n_0 ,\acc_i[8]_i_4_n_0 ,\acc_i[8]_i_5_n_0 }));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[0]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .I3(acc_q_reg[3]),
        .O(\acc_q[0]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[0]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .I3(acc_q_reg[2]),
        .O(\acc_q[0]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[0]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .I3(acc_q_reg[1]),
        .O(\acc_q[0]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[0]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .I3(acc_q_reg[0]),
        .O(\acc_q[0]_i_5_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_q[12]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\acc_q[12]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[12]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[15]),
        .O(\acc_q[12]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[12]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[30] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[30] ),
        .I3(acc_q_reg[14]),
        .O(\acc_q[12]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[12]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[29] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[29] ),
        .I3(acc_q_reg[13]),
        .O(\acc_q[12]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[12]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[28] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[28] ),
        .I3(acc_q_reg[12]),
        .O(\acc_q[12]_i_6_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_q[16]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\acc_q[16]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_q[16]_i_3 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\acc_q[16]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_q[16]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\acc_q[16]_i_4_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_q[16]_i_5 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\acc_q[16]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[16]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[19]),
        .O(\acc_q[16]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[16]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[18]),
        .O(\acc_q[16]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[16]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[17]),
        .O(\acc_q[16]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[16]_i_9 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[16]),
        .O(\acc_q[16]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_q[20]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\acc_q[20]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_q[20]_i_3 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\acc_q[20]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \acc_q[20]_i_4 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\acc_q[20]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \acc_q[20]_i_5 
       (.I0(acc_q_reg[23]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .O(\acc_q[20]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[20]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[22]),
        .O(\acc_q[20]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[20]_i_7 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[21]),
        .O(\acc_q[20]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[20]_i_8 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[20]),
        .O(\acc_q[20]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[4]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[23] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[23] ),
        .I3(acc_q_reg[7]),
        .O(\acc_q[4]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[4]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[22] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[22] ),
        .I3(acc_q_reg[6]),
        .O(\acc_q[4]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[4]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[21] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[21] ),
        .I3(acc_q_reg[5]),
        .O(\acc_q[4]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[4]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .I3(acc_q_reg[4]),
        .O(\acc_q[4]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[8]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[27] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[27] ),
        .I3(acc_q_reg[11]),
        .O(\acc_q[8]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[8]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[26] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[26] ),
        .I3(acc_q_reg[10]),
        .O(\acc_q[8]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[8]_i_4 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[25] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[25] ),
        .I3(acc_q_reg[9]),
        .O(\acc_q[8]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \acc_q[8]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[24] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[24] ),
        .I3(acc_q_reg[8]),
        .O(\acc_q[8]_i_5_n_0 ));
  CARRY4 \acc_q_reg[0]_i_1 
       (.CI(1'b0),
        .CO({\acc_q_reg[0]_i_1_n_0 ,\acc_q_reg[0]_i_1_n_1 ,\acc_q_reg[0]_i_1_n_2 ,\acc_q_reg[0]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_q_reg[3:0]),
        .O(\acc_q_reg[3] ),
        .S({\acc_q[0]_i_2_n_0 ,\acc_q[0]_i_3_n_0 ,\acc_q[0]_i_4_n_0 ,\acc_q[0]_i_5_n_0 }));
  CARRY4 \acc_q_reg[12]_i_1 
       (.CI(\acc_q_reg[8]_i_1_n_0 ),
        .CO({\acc_q_reg[12]_i_1_n_0 ,\acc_q_reg[12]_i_1_n_1 ,\acc_q_reg[12]_i_1_n_2 ,\acc_q_reg[12]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_q[12]_i_2_n_0 ,acc_q_reg[14:12]}),
        .O(\acc_q_reg[14] ),
        .S({\acc_q[12]_i_3_n_0 ,\acc_q[12]_i_4_n_0 ,\acc_q[12]_i_5_n_0 ,\acc_q[12]_i_6_n_0 }));
  CARRY4 \acc_q_reg[16]_i_1 
       (.CI(\acc_q_reg[12]_i_1_n_0 ),
        .CO({\acc_q_reg[16]_i_1_n_0 ,\acc_q_reg[16]_i_1_n_1 ,\acc_q_reg[16]_i_1_n_2 ,\acc_q_reg[16]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_q[16]_i_2_n_0 ,\acc_q[16]_i_3_n_0 ,\acc_q[16]_i_4_n_0 ,\acc_q[16]_i_5_n_0 }),
        .O(\acc_q_reg[14]_0 ),
        .S({\acc_q[16]_i_6_n_0 ,\acc_q[16]_i_7_n_0 ,\acc_q[16]_i_8_n_0 ,\acc_q[16]_i_9_n_0 }));
  CARRY4 \acc_q_reg[20]_i_1 
       (.CI(\acc_q_reg[16]_i_1_n_0 ),
        .CO({\NLW_acc_q_reg[20]_i_1_CO_UNCONNECTED [3],\acc_q_reg[20]_i_1_n_1 ,\acc_q_reg[20]_i_1_n_2 ,\acc_q_reg[20]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,\acc_q[20]_i_2_n_0 ,\acc_q[20]_i_3_n_0 ,\acc_q[20]_i_4_n_0 }),
        .O(\B_V_data_1_payload_B_reg[31]_0 ),
        .S({\acc_q[20]_i_5_n_0 ,\acc_q[20]_i_6_n_0 ,\acc_q[20]_i_7_n_0 ,\acc_q[20]_i_8_n_0 }));
  CARRY4 \acc_q_reg[4]_i_1 
       (.CI(\acc_q_reg[0]_i_1_n_0 ),
        .CO({\acc_q_reg[4]_i_1_n_0 ,\acc_q_reg[4]_i_1_n_1 ,\acc_q_reg[4]_i_1_n_2 ,\acc_q_reg[4]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_q_reg[7:4]),
        .O(\acc_q_reg[7] ),
        .S({\acc_q[4]_i_2_n_0 ,\acc_q[4]_i_3_n_0 ,\acc_q[4]_i_4_n_0 ,\acc_q[4]_i_5_n_0 }));
  CARRY4 \acc_q_reg[8]_i_1 
       (.CI(\acc_q_reg[4]_i_1_n_0 ),
        .CO({\acc_q_reg[8]_i_1_n_0 ,\acc_q_reg[8]_i_1_n_1 ,\acc_q_reg[8]_i_1_n_2 ,\acc_q_reg[8]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_q_reg[11:8]),
        .O(\acc_q_reg[11] ),
        .S({\acc_q[8]_i_2_n_0 ,\acc_q[8]_i_3_n_0 ,\acc_q[8]_i_4_n_0 ,\acc_q[8]_i_5_n_0 }));
  LUT3 #(
    .INIT(8'hF8)) 
    ap_enable_reg_pp0_iter1_i_1
       (.I0(rx_in_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .I2(ap_enable_reg_pp0_iter1),
        .O(\B_V_data_1_state_reg[0]_0 ));
  LUT3 #(
    .INIT(8'h1D)) 
    \trunc_ln2_reg_293[11]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .O(\trunc_ln2_reg_293[11]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln2_reg_293[11]_i_3 
       (.I0(acc_i_reg[18]),
        .I1(acc_i_reg[19]),
        .O(\trunc_ln2_reg_293[11]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln2_reg_293[11]_i_4 
       (.I0(acc_i_reg[17]),
        .I1(acc_i_reg[18]),
        .O(\trunc_ln2_reg_293[11]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln2_reg_293[11]_i_5 
       (.I0(acc_i_reg[16]),
        .I1(acc_i_reg[17]),
        .O(\trunc_ln2_reg_293[11]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \trunc_ln2_reg_293[11]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[16]),
        .O(\trunc_ln2_reg_293[11]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln2_reg_293[15]_i_4 
       (.I0(acc_i_reg[23]),
        .I1(acc_i_reg[22]),
        .O(\trunc_ln2_reg_293[15]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln2_reg_293[15]_i_5 
       (.I0(acc_i_reg[21]),
        .I1(acc_i_reg[22]),
        .O(\trunc_ln2_reg_293[15]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln2_reg_293[15]_i_6 
       (.I0(acc_i_reg[20]),
        .I1(acc_i_reg[21]),
        .O(\trunc_ln2_reg_293[15]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln2_reg_293[15]_i_7 
       (.I0(acc_i_reg[19]),
        .I1(acc_i_reg[20]),
        .O(\trunc_ln2_reg_293[15]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_10 
       (.I0(acc_i_reg[5]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .O(\trunc_ln2_reg_293[3]_i_10_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_11 
       (.I0(acc_i_reg[4]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .O(\trunc_ln2_reg_293[3]_i_11_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_12 
       (.I0(acc_i_reg[3]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .O(\trunc_ln2_reg_293[3]_i_12_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_13 
       (.I0(acc_i_reg[2]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .O(\trunc_ln2_reg_293[3]_i_13_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_14 
       (.I0(acc_i_reg[1]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .O(\trunc_ln2_reg_293[3]_i_14_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_15 
       (.I0(acc_i_reg[0]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .O(\trunc_ln2_reg_293[3]_i_15_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_3 
       (.I0(acc_i_reg[11]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .O(\trunc_ln2_reg_293[3]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_4 
       (.I0(acc_i_reg[10]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .O(\trunc_ln2_reg_293[3]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_5 
       (.I0(acc_i_reg[9]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .O(\trunc_ln2_reg_293[3]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_6 
       (.I0(acc_i_reg[8]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .O(\trunc_ln2_reg_293[3]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_8 
       (.I0(acc_i_reg[7]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .O(\trunc_ln2_reg_293[3]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[3]_i_9 
       (.I0(acc_i_reg[6]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .O(\trunc_ln2_reg_293[3]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \trunc_ln2_reg_293[7]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\trunc_ln2_reg_293[7]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \trunc_ln2_reg_293[7]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(acc_i_reg[15]),
        .O(\trunc_ln2_reg_293[7]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[7]_i_4 
       (.I0(acc_i_reg[14]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .O(\trunc_ln2_reg_293[7]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[7]_i_5 
       (.I0(acc_i_reg[13]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .O(\trunc_ln2_reg_293[7]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln2_reg_293[7]_i_6 
       (.I0(acc_i_reg[12]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .O(\trunc_ln2_reg_293[7]_i_6_n_0 ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln2_reg_293_reg[11]_i_1 
       (.CI(\trunc_ln2_reg_293_reg[7]_i_1_n_0 ),
        .CO({\trunc_ln2_reg_293_reg[11]_i_1_n_0 ,\trunc_ln2_reg_293_reg[11]_i_1_n_1 ,\trunc_ln2_reg_293_reg[11]_i_1_n_2 ,\trunc_ln2_reg_293_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({acc_i_reg[18:16],\trunc_ln2_reg_293[11]_i_2_n_0 }),
        .O(D[11:8]),
        .S({\trunc_ln2_reg_293[11]_i_3_n_0 ,\trunc_ln2_reg_293[11]_i_4_n_0 ,\trunc_ln2_reg_293[11]_i_5_n_0 ,\trunc_ln2_reg_293[11]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln2_reg_293_reg[15]_i_2 
       (.CI(\trunc_ln2_reg_293_reg[11]_i_1_n_0 ),
        .CO({\NLW_trunc_ln2_reg_293_reg[15]_i_2_CO_UNCONNECTED [3],\trunc_ln2_reg_293_reg[15]_i_2_n_1 ,\trunc_ln2_reg_293_reg[15]_i_2_n_2 ,\trunc_ln2_reg_293_reg[15]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,acc_i_reg[21:19]}),
        .O(D[15:12]),
        .S({\trunc_ln2_reg_293[15]_i_4_n_0 ,\trunc_ln2_reg_293[15]_i_5_n_0 ,\trunc_ln2_reg_293[15]_i_6_n_0 ,\trunc_ln2_reg_293[15]_i_7_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln2_reg_293_reg[3]_i_1 
       (.CI(\trunc_ln2_reg_293_reg[3]_i_2_n_0 ),
        .CO({\trunc_ln2_reg_293_reg[3]_i_1_n_0 ,\trunc_ln2_reg_293_reg[3]_i_1_n_1 ,\trunc_ln2_reg_293_reg[3]_i_1_n_2 ,\trunc_ln2_reg_293_reg[3]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_i_reg[11:8]),
        .O(D[3:0]),
        .S({\trunc_ln2_reg_293[3]_i_3_n_0 ,\trunc_ln2_reg_293[3]_i_4_n_0 ,\trunc_ln2_reg_293[3]_i_5_n_0 ,\trunc_ln2_reg_293[3]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln2_reg_293_reg[3]_i_2 
       (.CI(\trunc_ln2_reg_293_reg[3]_i_7_n_0 ),
        .CO({\trunc_ln2_reg_293_reg[3]_i_2_n_0 ,\trunc_ln2_reg_293_reg[3]_i_2_n_1 ,\trunc_ln2_reg_293_reg[3]_i_2_n_2 ,\trunc_ln2_reg_293_reg[3]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_i_reg[7:4]),
        .O(\NLW_trunc_ln2_reg_293_reg[3]_i_2_O_UNCONNECTED [3:0]),
        .S({\trunc_ln2_reg_293[3]_i_8_n_0 ,\trunc_ln2_reg_293[3]_i_9_n_0 ,\trunc_ln2_reg_293[3]_i_10_n_0 ,\trunc_ln2_reg_293[3]_i_11_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln2_reg_293_reg[3]_i_7 
       (.CI(1'b0),
        .CO({\trunc_ln2_reg_293_reg[3]_i_7_n_0 ,\trunc_ln2_reg_293_reg[3]_i_7_n_1 ,\trunc_ln2_reg_293_reg[3]_i_7_n_2 ,\trunc_ln2_reg_293_reg[3]_i_7_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_i_reg[3:0]),
        .O(\NLW_trunc_ln2_reg_293_reg[3]_i_7_O_UNCONNECTED [3:0]),
        .S({\trunc_ln2_reg_293[3]_i_12_n_0 ,\trunc_ln2_reg_293[3]_i_13_n_0 ,\trunc_ln2_reg_293[3]_i_14_n_0 ,\trunc_ln2_reg_293[3]_i_15_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln2_reg_293_reg[7]_i_1 
       (.CI(\trunc_ln2_reg_293_reg[3]_i_1_n_0 ),
        .CO({\trunc_ln2_reg_293_reg[7]_i_1_n_0 ,\trunc_ln2_reg_293_reg[7]_i_1_n_1 ,\trunc_ln2_reg_293_reg[7]_i_1_n_2 ,\trunc_ln2_reg_293_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\trunc_ln2_reg_293[7]_i_2_n_0 ,acc_i_reg[14:12]}),
        .O(D[7:4]),
        .S({\trunc_ln2_reg_293[7]_i_3_n_0 ,\trunc_ln2_reg_293[7]_i_4_n_0 ,\trunc_ln2_reg_293[7]_i_5_n_0 ,\trunc_ln2_reg_293[7]_i_6_n_0 }));
  LUT3 #(
    .INIT(8'h1D)) 
    \trunc_ln3_reg_298[11]_i_2 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .O(\trunc_ln3_reg_298[11]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln3_reg_298[11]_i_3 
       (.I0(acc_q_reg[18]),
        .I1(acc_q_reg[19]),
        .O(\trunc_ln3_reg_298[11]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln3_reg_298[11]_i_4 
       (.I0(acc_q_reg[17]),
        .I1(acc_q_reg[18]),
        .O(\trunc_ln3_reg_298[11]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln3_reg_298[11]_i_5 
       (.I0(acc_q_reg[16]),
        .I1(acc_q_reg[17]),
        .O(\trunc_ln3_reg_298[11]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \trunc_ln3_reg_298[11]_i_6 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[16]),
        .O(\trunc_ln3_reg_298[11]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln3_reg_298[15]_i_2 
       (.I0(acc_q_reg[23]),
        .I1(acc_q_reg[22]),
        .O(\trunc_ln3_reg_298[15]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln3_reg_298[15]_i_3 
       (.I0(acc_q_reg[21]),
        .I1(acc_q_reg[22]),
        .O(\trunc_ln3_reg_298[15]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln3_reg_298[15]_i_4 
       (.I0(acc_q_reg[20]),
        .I1(acc_q_reg[21]),
        .O(\trunc_ln3_reg_298[15]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h9)) 
    \trunc_ln3_reg_298[15]_i_5 
       (.I0(acc_q_reg[19]),
        .I1(acc_q_reg[20]),
        .O(\trunc_ln3_reg_298[15]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_10 
       (.I0(acc_q_reg[5]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[21] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[21] ),
        .O(\trunc_ln3_reg_298[3]_i_10_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_11 
       (.I0(acc_q_reg[4]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .O(\trunc_ln3_reg_298[3]_i_11_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_12 
       (.I0(acc_q_reg[3]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .O(\trunc_ln3_reg_298[3]_i_12_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_13 
       (.I0(acc_q_reg[2]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .O(\trunc_ln3_reg_298[3]_i_13_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_14 
       (.I0(acc_q_reg[1]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .O(\trunc_ln3_reg_298[3]_i_14_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_15 
       (.I0(acc_q_reg[0]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .O(\trunc_ln3_reg_298[3]_i_15_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_3 
       (.I0(acc_q_reg[11]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[27] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[27] ),
        .O(\trunc_ln3_reg_298[3]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_4 
       (.I0(acc_q_reg[10]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[26] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[26] ),
        .O(\trunc_ln3_reg_298[3]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_5 
       (.I0(acc_q_reg[9]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[25] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[25] ),
        .O(\trunc_ln3_reg_298[3]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_6 
       (.I0(acc_q_reg[8]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[24] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[24] ),
        .O(\trunc_ln3_reg_298[3]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_8 
       (.I0(acc_q_reg[7]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[23] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[23] ),
        .O(\trunc_ln3_reg_298[3]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[3]_i_9 
       (.I0(acc_q_reg[6]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[22] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[22] ),
        .O(\trunc_ln3_reg_298[3]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'hB8)) 
    \trunc_ln3_reg_298[7]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .O(\trunc_ln3_reg_298[7]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1DE2)) 
    \trunc_ln3_reg_298[7]_i_3 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I1(B_V_data_1_sel),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(acc_q_reg[15]),
        .O(\trunc_ln3_reg_298[7]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[7]_i_4 
       (.I0(acc_q_reg[14]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[30] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[30] ),
        .O(\trunc_ln3_reg_298[7]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[7]_i_5 
       (.I0(acc_q_reg[13]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[29] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[29] ),
        .O(\trunc_ln3_reg_298[7]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h56A6)) 
    \trunc_ln3_reg_298[7]_i_6 
       (.I0(acc_q_reg[12]),
        .I1(\B_V_data_1_payload_A_reg_n_0_[28] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_B_reg_n_0_[28] ),
        .O(\trunc_ln3_reg_298[7]_i_6_n_0 ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln3_reg_298_reg[11]_i_1 
       (.CI(\trunc_ln3_reg_298_reg[7]_i_1_n_0 ),
        .CO({\trunc_ln3_reg_298_reg[11]_i_1_n_0 ,\trunc_ln3_reg_298_reg[11]_i_1_n_1 ,\trunc_ln3_reg_298_reg[11]_i_1_n_2 ,\trunc_ln3_reg_298_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({acc_q_reg[18:16],\trunc_ln3_reg_298[11]_i_2_n_0 }),
        .O(\acc_q_reg[21] [11:8]),
        .S({\trunc_ln3_reg_298[11]_i_3_n_0 ,\trunc_ln3_reg_298[11]_i_4_n_0 ,\trunc_ln3_reg_298[11]_i_5_n_0 ,\trunc_ln3_reg_298[11]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln3_reg_298_reg[15]_i_1 
       (.CI(\trunc_ln3_reg_298_reg[11]_i_1_n_0 ),
        .CO({\NLW_trunc_ln3_reg_298_reg[15]_i_1_CO_UNCONNECTED [3],\trunc_ln3_reg_298_reg[15]_i_1_n_1 ,\trunc_ln3_reg_298_reg[15]_i_1_n_2 ,\trunc_ln3_reg_298_reg[15]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,acc_q_reg[21:19]}),
        .O(\acc_q_reg[21] [15:12]),
        .S({\trunc_ln3_reg_298[15]_i_2_n_0 ,\trunc_ln3_reg_298[15]_i_3_n_0 ,\trunc_ln3_reg_298[15]_i_4_n_0 ,\trunc_ln3_reg_298[15]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln3_reg_298_reg[3]_i_1 
       (.CI(\trunc_ln3_reg_298_reg[3]_i_2_n_0 ),
        .CO({\trunc_ln3_reg_298_reg[3]_i_1_n_0 ,\trunc_ln3_reg_298_reg[3]_i_1_n_1 ,\trunc_ln3_reg_298_reg[3]_i_1_n_2 ,\trunc_ln3_reg_298_reg[3]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_q_reg[11:8]),
        .O(\acc_q_reg[21] [3:0]),
        .S({\trunc_ln3_reg_298[3]_i_3_n_0 ,\trunc_ln3_reg_298[3]_i_4_n_0 ,\trunc_ln3_reg_298[3]_i_5_n_0 ,\trunc_ln3_reg_298[3]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln3_reg_298_reg[3]_i_2 
       (.CI(\trunc_ln3_reg_298_reg[3]_i_7_n_0 ),
        .CO({\trunc_ln3_reg_298_reg[3]_i_2_n_0 ,\trunc_ln3_reg_298_reg[3]_i_2_n_1 ,\trunc_ln3_reg_298_reg[3]_i_2_n_2 ,\trunc_ln3_reg_298_reg[3]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_q_reg[7:4]),
        .O(\NLW_trunc_ln3_reg_298_reg[3]_i_2_O_UNCONNECTED [3:0]),
        .S({\trunc_ln3_reg_298[3]_i_8_n_0 ,\trunc_ln3_reg_298[3]_i_9_n_0 ,\trunc_ln3_reg_298[3]_i_10_n_0 ,\trunc_ln3_reg_298[3]_i_11_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln3_reg_298_reg[3]_i_7 
       (.CI(1'b0),
        .CO({\trunc_ln3_reg_298_reg[3]_i_7_n_0 ,\trunc_ln3_reg_298_reg[3]_i_7_n_1 ,\trunc_ln3_reg_298_reg[3]_i_7_n_2 ,\trunc_ln3_reg_298_reg[3]_i_7_n_3 }),
        .CYINIT(1'b0),
        .DI(acc_q_reg[3:0]),
        .O(\NLW_trunc_ln3_reg_298_reg[3]_i_7_O_UNCONNECTED [3:0]),
        .S({\trunc_ln3_reg_298[3]_i_12_n_0 ,\trunc_ln3_reg_298[3]_i_13_n_0 ,\trunc_ln3_reg_298[3]_i_14_n_0 ,\trunc_ln3_reg_298[3]_i_15_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \trunc_ln3_reg_298_reg[7]_i_1 
       (.CI(\trunc_ln3_reg_298_reg[3]_i_1_n_0 ),
        .CO({\trunc_ln3_reg_298_reg[7]_i_1_n_0 ,\trunc_ln3_reg_298_reg[7]_i_1_n_1 ,\trunc_ln3_reg_298_reg[7]_i_1_n_2 ,\trunc_ln3_reg_298_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\trunc_ln3_reg_298[7]_i_2_n_0 ,acc_q_reg[14:12]}),
        .O(\acc_q_reg[21] [7:4]),
        .S({\trunc_ln3_reg_298[7]_i_3_n_0 ,\trunc_ln3_reg_298[7]_i_4_n_0 ,\trunc_ln3_reg_298[7]_i_5_n_0 ,\trunc_ln3_reg_298[7]_i_6_n_0 }));
endmodule

(* ORIG_REF_NAME = "fsk_decimator_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1
   (\B_V_data_1_state_reg[1]_0 ,
    ap_rst_n_inv,
    B_V_data_1_sel_wr,
    ap_enable_reg_pp0_iter1_reg,
    \B_V_data_1_state_reg[1]_1 ,
    dec_out_TLAST,
    ap_clk,
    \B_V_data_1_state_reg[0]_0 ,
    B_V_data_1_sel_wr_reg_0,
    \B_V_data_1_state_reg[1]_2 ,
    rx_in_TVALID_int_regslice,
    \B_V_data_1_state_reg[1]_3 ,
    ap_enable_reg_pp0_iter1,
    icmp_ln40_reg_289,
    ap_rst_n,
    dec_out_TREADY,
    pkt_rx_last_V_reg_284);
  output \B_V_data_1_state_reg[1]_0 ;
  output ap_rst_n_inv;
  output B_V_data_1_sel_wr;
  output ap_enable_reg_pp0_iter1_reg;
  output \B_V_data_1_state_reg[1]_1 ;
  output [0:0]dec_out_TLAST;
  input ap_clk;
  input \B_V_data_1_state_reg[0]_0 ;
  input B_V_data_1_sel_wr_reg_0;
  input \B_V_data_1_state_reg[1]_2 ;
  input rx_in_TVALID_int_regslice;
  input \B_V_data_1_state_reg[1]_3 ;
  input ap_enable_reg_pp0_iter1;
  input icmp_ln40_reg_289;
  input ap_rst_n;
  input dec_out_TREADY;
  input pkt_rx_last_V_reg_284;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__0_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1__0_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__2_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_reg_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[1]_i_2__0_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire \B_V_data_1_state_reg[1]_1 ;
  wire \B_V_data_1_state_reg[1]_2 ;
  wire \B_V_data_1_state_reg[1]_3 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter1_reg;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [0:0]dec_out_TLAST;
  wire dec_out_TREADY;
  wire icmp_ln40_reg_289;
  wire pkt_rx_last_V_reg_284;
  wire rx_in_TVALID_int_regslice;

  LUT5 #(
    .INIT(32'hEFEE2022)) 
    \B_V_data_1_payload_A[0]_i_1__0 
       (.I0(pkt_rx_last_V_reg_284),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg[1]_0 ),
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
       (.I0(pkt_rx_last_V_reg_284),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(\B_V_data_1_state_reg_n_0_[0] ),
        .I4(B_V_data_1_payload_B),
        .O(\B_V_data_1_payload_B[0]_i_1__0_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[0]_i_1__0_n_0 ),
        .Q(B_V_data_1_payload_B),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__2
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(dec_out_TREADY),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__2_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__2_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_reg_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'h8F)) 
    \B_V_data_1_state[0]_i_2 
       (.I0(\B_V_data_1_state_reg[1]_0 ),
        .I1(dec_out_TREADY),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .O(\B_V_data_1_state_reg[1]_1 ));
  LUT6 #(
    .INIT(64'hDDDDDDDD5DDDDDDD)) 
    \B_V_data_1_state[1]_i_1__0 
       (.I0(\B_V_data_1_state[1]_i_2__0_n_0 ),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(\B_V_data_1_state_reg[1]_2 ),
        .I3(rx_in_TVALID_int_regslice),
        .I4(\B_V_data_1_state_reg[1]_3 ),
        .I5(ap_enable_reg_pp0_iter1_reg),
        .O(B_V_data_1_state));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_state[1]_i_1__2 
       (.I0(ap_rst_n),
        .O(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_state[1]_i_2__0 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(dec_out_TREADY),
        .O(\B_V_data_1_state[1]_i_2__0_n_0 ));
  LUT2 #(
    .INIT(4'h7)) 
    \B_V_data_1_state[1]_i_4 
       (.I0(ap_enable_reg_pp0_iter1),
        .I1(icmp_ln40_reg_289),
        .O(ap_enable_reg_pp0_iter1_reg));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state_reg[0]_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg[1]_0 ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \dec_out_TLAST[0]_INST_0 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(dec_out_TLAST));
endmodule

(* ORIG_REF_NAME = "fsk_decimator_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1_1
   (rx_in_TLAST_int_regslice,
    ap_rst_n_inv,
    ap_clk,
    B_V_data_1_sel_rd_reg_0,
    \B_V_data_1_state_reg[0]_0 ,
    rx_in_TVALID,
    rx_in_TLAST,
    rx_in_TVALID_int_regslice,
    B_V_data_1_sel_rd_reg_1,
    B_V_data_1_sel_rd_reg_2);
  output rx_in_TLAST_int_regslice;
  input ap_rst_n_inv;
  input ap_clk;
  input B_V_data_1_sel_rd_reg_0;
  input \B_V_data_1_state_reg[0]_0 ;
  input rx_in_TVALID;
  input [0:0]rx_in_TLAST;
  input rx_in_TVALID_int_regslice;
  input B_V_data_1_sel_rd_reg_1;
  input B_V_data_1_sel_rd_reg_2;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1_n_0;
  wire B_V_data_1_sel_rd_reg_0;
  wire B_V_data_1_sel_rd_reg_1;
  wire B_V_data_1_sel_rd_reg_2;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__2_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__0_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n_inv;
  wire [0:0]rx_in_TLAST;
  wire rx_in_TLAST_int_regslice;
  wire rx_in_TVALID;
  wire rx_in_TVALID_int_regslice;

  LUT5 #(
    .INIT(32'hEFEE2022)) 
    \B_V_data_1_payload_A[0]_i_1 
       (.I0(rx_in_TLAST),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg_n_0_[0] ),
        .I4(B_V_data_1_payload_A),
        .O(\B_V_data_1_payload_A[0]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[0]_i_1_n_0 ),
        .Q(B_V_data_1_payload_A),
        .R(1'b0));
  LUT5 #(
    .INIT(32'hBFBB8088)) 
    \B_V_data_1_payload_B[0]_i_1 
       (.I0(rx_in_TLAST),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg_n_0_[0] ),
        .I4(B_V_data_1_payload_B),
        .O(\B_V_data_1_payload_B[0]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[0]_i_1_n_0 ),
        .Q(B_V_data_1_payload_B),
        .R(1'b0));
  LUT6 #(
    .INIT(64'h7F7F7FFF80808000)) 
    B_V_data_1_sel_rd_i_1
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(B_V_data_1_sel_rd_reg_0),
        .I2(rx_in_TVALID_int_regslice),
        .I3(B_V_data_1_sel_rd_reg_1),
        .I4(B_V_data_1_sel_rd_reg_2),
        .I5(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_wr_i_1__2
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(rx_in_TVALID),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__2_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__2_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT5 #(
    .INIT(32'hF8F8D8F8)) 
    \B_V_data_1_state[0]_i_1__0 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(rx_in_TVALID),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(B_V_data_1_sel_rd_reg_0),
        .I4(\B_V_data_1_state_reg[0]_0 ),
        .O(\B_V_data_1_state[0]_i_1__0_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT5 #(
    .INIT(32'h5DFF5D5D)) 
    \B_V_data_1_state[1]_i_1__1 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(B_V_data_1_sel_rd_reg_0),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(rx_in_TVALID),
        .I4(\B_V_data_1_state_reg_n_0_[1] ),
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
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_rx_last_V_reg_284[0]_i_2 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(rx_in_TLAST_int_regslice));
endmodule

(* CHECK_LICENSE_TYPE = "system_fsk_decimator_0_0,fsk_decimator,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "HLS" *) 
(* X_CORE_INFO = "fsk_decimator,Vivado 2023.1" *) (* hls_module = "yes" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (ap_clk,
    ap_rst_n,
    rx_in_TVALID,
    rx_in_TREADY,
    rx_in_TDATA,
    rx_in_TLAST,
    rx_in_TKEEP,
    rx_in_TSTRB,
    dec_out_TVALID,
    dec_out_TREADY,
    dec_out_TDATA,
    dec_out_TLAST,
    dec_out_TKEEP,
    dec_out_TSTRB);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ap_clk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ap_clk, ASSOCIATED_BUSIF rx_in:dec_out, ASSOCIATED_RESET ap_rst_n, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input ap_clk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 ap_rst_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ap_rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input ap_rst_n;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 rx_in TVALID" *) input rx_in_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 rx_in TREADY" *) output rx_in_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 rx_in TDATA" *) input [31:0]rx_in_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 rx_in TLAST" *) input [0:0]rx_in_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 rx_in TKEEP" *) input [3:0]rx_in_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 rx_in TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME rx_in, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input [3:0]rx_in_TSTRB;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 dec_out TVALID" *) output dec_out_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 dec_out TREADY" *) input dec_out_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 dec_out TDATA" *) output [31:0]dec_out_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 dec_out TLAST" *) output [0:0]dec_out_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 dec_out TKEEP" *) output [3:0]dec_out_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 dec_out TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME dec_out, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) output [3:0]dec_out_TSTRB;

  wire \<const0> ;
  wire \<const1> ;
  wire ap_clk;
  wire ap_rst_n;
  wire [31:0]dec_out_TDATA;
  wire [0:0]dec_out_TLAST;
  wire dec_out_TREADY;
  wire dec_out_TVALID;
  wire [31:0]rx_in_TDATA;
  wire [0:0]rx_in_TLAST;
  wire rx_in_TREADY;
  wire rx_in_TVALID;
  wire [3:0]NLW_inst_dec_out_TKEEP_UNCONNECTED;
  wire [3:0]NLW_inst_dec_out_TSTRB_UNCONNECTED;

  assign dec_out_TKEEP[3] = \<const1> ;
  assign dec_out_TKEEP[2] = \<const1> ;
  assign dec_out_TKEEP[1] = \<const1> ;
  assign dec_out_TKEEP[0] = \<const1> ;
  assign dec_out_TSTRB[3] = \<const0> ;
  assign dec_out_TSTRB[2] = \<const0> ;
  assign dec_out_TSTRB[1] = \<const0> ;
  assign dec_out_TSTRB[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  VCC VCC
       (.P(\<const1> ));
  (* SDX_KERNEL = "true" *) 
  (* SDX_KERNEL_SYNTH_INST = "inst" *) 
  (* SDX_KERNEL_TYPE = "hls" *) 
  (* ap_ST_fsm_pp0_stage0 = "1'b1" *) 
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator inst
       (.ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .dec_out_TDATA(dec_out_TDATA),
        .dec_out_TKEEP(NLW_inst_dec_out_TKEEP_UNCONNECTED[3:0]),
        .dec_out_TLAST(dec_out_TLAST),
        .dec_out_TREADY(dec_out_TREADY),
        .dec_out_TSTRB(NLW_inst_dec_out_TSTRB_UNCONNECTED[3:0]),
        .dec_out_TVALID(dec_out_TVALID),
        .rx_in_TDATA(rx_in_TDATA),
        .rx_in_TKEEP({1'b0,1'b0,1'b0,1'b0}),
        .rx_in_TLAST(rx_in_TLAST),
        .rx_in_TREADY(rx_in_TREADY),
        .rx_in_TSTRB({1'b0,1'b0,1'b0,1'b0}),
        .rx_in_TVALID(rx_in_TVALID));
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
