// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Wed Jan  7 18:03:55 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_Data_pack_preamble_2_0_0_sim_netlist.v
// Design      : system_Data_pack_preamble_2_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_Data_pack_preamble_2FSK_upd
   (out_bit_valid,
    out_bit_prmbl_strt,
    out_bit_prmbl_end,
    out_bit_to_fsk,
    clock,
    reset_n);
  output out_bit_valid;
  output out_bit_prmbl_strt;
  output out_bit_prmbl_end;
  output out_bit_to_fsk;
  input clock;
  input reset_n;

  wire \bit_cnt[5]_i_2_n_0 ;
  wire \bit_cnt[5]_i_3_n_0 ;
  wire \bit_cnt[5]_i_4_n_0 ;
  wire \bit_cnt[6]_i_2_n_0 ;
  wire \bit_cnt[6]_i_3_n_0 ;
  wire \bit_cnt[7]_i_1_n_0 ;
  wire \bit_cnt[7]_i_3_n_0 ;
  wire \bit_cnt[7]_i_4_n_0 ;
  wire \bit_cnt[7]_i_5_n_0 ;
  wire \bit_cnt_reg_n_0_[0] ;
  wire \bit_cnt_reg_n_0_[1] ;
  wire \bit_cnt_reg_n_0_[2] ;
  wire \bit_cnt_reg_n_0_[3] ;
  wire \bit_cnt_reg_n_0_[4] ;
  wire \bit_cnt_reg_n_0_[5] ;
  wire \bit_cnt_reg_n_0_[6] ;
  wire \bit_cnt_reg_n_0_[7] ;
  wire [15:0]bit_dur_cnt;
  wire [15:1]bit_dur_cnt0;
  wire bit_dur_cnt0_carry__0_n_0;
  wire bit_dur_cnt0_carry__0_n_1;
  wire bit_dur_cnt0_carry__0_n_2;
  wire bit_dur_cnt0_carry__0_n_3;
  wire bit_dur_cnt0_carry__1_n_0;
  wire bit_dur_cnt0_carry__1_n_1;
  wire bit_dur_cnt0_carry__1_n_2;
  wire bit_dur_cnt0_carry__1_n_3;
  wire bit_dur_cnt0_carry__2_n_2;
  wire bit_dur_cnt0_carry__2_n_3;
  wire bit_dur_cnt0_carry_n_0;
  wire bit_dur_cnt0_carry_n_1;
  wire bit_dur_cnt0_carry_n_2;
  wire bit_dur_cnt0_carry_n_3;
  wire \bit_dur_cnt[0]_i_1_n_0 ;
  wire \bit_dur_cnt[10]_i_1_n_0 ;
  wire \bit_dur_cnt[11]_i_1_n_0 ;
  wire \bit_dur_cnt[12]_i_1_n_0 ;
  wire \bit_dur_cnt[13]_i_1_n_0 ;
  wire \bit_dur_cnt[14]_i_1_n_0 ;
  wire \bit_dur_cnt[15]_i_1_n_0 ;
  wire \bit_dur_cnt[15]_i_2_n_0 ;
  wire \bit_dur_cnt[15]_i_3_n_0 ;
  wire \bit_dur_cnt[1]_i_1_n_0 ;
  wire \bit_dur_cnt[2]_i_1_n_0 ;
  wire \bit_dur_cnt[3]_i_1_n_0 ;
  wire \bit_dur_cnt[4]_i_1_n_0 ;
  wire \bit_dur_cnt[5]_i_1_n_0 ;
  wire \bit_dur_cnt[6]_i_1_n_0 ;
  wire \bit_dur_cnt[7]_i_1_n_0 ;
  wire \bit_dur_cnt[8]_i_1_n_0 ;
  wire \bit_dur_cnt[9]_i_1_n_0 ;
  wire clock;
  wire out_bit_prmbl_end;
  wire out_bit_prmbl_end_i_1_n_0;
  wire out_bit_prmbl_strt;
  wire out_bit_prmbl_strt_i_1_n_0;
  wire out_bit_prmbl_strt_i_2_n_0;
  wire out_bit_to_fsk;
  wire out_bit_to_fsk_i_4_n_0;
  wire out_bit_to_fsk_i_5_n_0;
  wire out_bit_to_fsk_reg_i_3_n_0;
  wire out_bit_valid;
  wire out_bit_valid_i_1_n_0;
  wire [7:0]p_1_in;
  wire preamble_en;
  wire preamble_en_i_1_n_0;
  wire preamble_en_i_2_n_0;
  wire preamble_en_i_3_n_0;
  wire preamble_mod_n_0;
  wire preamble_mod_n_1;
  wire reset_n;
  wire \state_udp[0]_i_1_n_0 ;
  wire \state_udp[0]_i_2_n_0 ;
  wire \state_udp[0]_i_3_n_0 ;
  wire \state_udp[0]_i_4_n_0 ;
  wire \state_udp[0]_i_5_n_0 ;
  wire \state_udp[0]_i_6_n_0 ;
  wire \state_udp[1]_i_1_n_0 ;
  wire \state_udp[1]_i_2_n_0 ;
  wire \state_udp[1]_i_3_n_0 ;
  wire \state_udp[2]_i_1_n_0 ;
  wire \state_udp[2]_i_2_n_0 ;
  wire \state_udp[2]_i_3_n_0 ;
  wire \state_udp[2]_i_4_n_0 ;
  wire \state_udp[2]_i_5_n_0 ;
  wire \state_udp[2]_i_6_n_0 ;
  wire \state_udp[2]_i_7_n_0 ;
  wire \state_udp[2]_i_8_n_0 ;
  wire \state_udp[2]_i_9_n_0 ;
  wire \state_udp_nxt[0]_i_1_n_0 ;
  wire \state_udp_nxt[0]_i_2_n_0 ;
  wire \state_udp_nxt[1]_i_1_n_0 ;
  wire \state_udp_nxt[1]_i_2_n_0 ;
  wire \state_udp_nxt[1]_i_3_n_0 ;
  wire \state_udp_nxt[1]_i_4_n_0 ;
  wire \state_udp_nxt[2]_i_1_n_0 ;
  wire \state_udp_nxt_reg_n_0_[0] ;
  wire \state_udp_nxt_reg_n_0_[1] ;
  wire \state_udp_nxt_reg_n_0_[2] ;
  wire \state_udp_reg_n_0_[0] ;
  wire \state_udp_reg_n_0_[1] ;
  wire \state_udp_reg_n_0_[2] ;
  wire [7:0]temp_Data;
  wire \temp_Data[0]_i_1_n_0 ;
  wire \temp_Data[1]_i_1_n_0 ;
  wire \temp_Data[2]_i_1_n_0 ;
  wire \temp_Data[3]_i_1_n_0 ;
  wire \temp_Data[4]_i_1_n_0 ;
  wire \temp_Data[4]_i_2_n_0 ;
  wire \temp_Data[5]_i_1_n_0 ;
  wire \temp_Data[5]_i_2_n_0 ;
  wire \temp_Data[6]_i_1_n_0 ;
  wire \temp_Data[6]_i_2_n_0 ;
  wire \temp_Data[7]_i_1_n_0 ;
  wire \temp_Data[7]_i_2_n_0 ;
  wire \temp_Data[7]_i_3_n_0 ;
  wire \temp_Data[7]_i_4_n_0 ;
  wire [7:0]word_count;
  wire \word_count[0]_i_1_n_0 ;
  wire \word_count[1]_i_1_n_0 ;
  wire \word_count[2]_i_1_n_0 ;
  wire \word_count[3]_i_1_n_0 ;
  wire \word_count[4]_i_1_n_0 ;
  wire \word_count[5]_i_1_n_0 ;
  wire \word_count[5]_i_2_n_0 ;
  wire \word_count[6]_i_1_n_0 ;
  wire \word_count[6]_i_2_n_0 ;
  wire \word_count[6]_i_3_n_0 ;
  wire \word_count[7]_i_1_n_0 ;
  wire \word_count[7]_i_2_n_0 ;
  wire \word_count[7]_i_3_n_0 ;
  wire \word_count[7]_i_4_n_0 ;
  wire \word_count_reg_rep_n_0_[0] ;
  wire \word_count_reg_rep_n_0_[1] ;
  wire \word_count_reg_rep_n_0_[2] ;
  wire \word_count_reg_rep_n_0_[3] ;
  wire \word_count_reg_rep_n_0_[4] ;
  wire \word_count_reg_rep_n_0_[5] ;
  wire \word_count_reg_rep_n_0_[6] ;
  wire \word_count_reg_rep_n_0_[7] ;
  wire [3:2]NLW_bit_dur_cnt0_carry__2_CO_UNCONNECTED;
  wire [3:3]NLW_bit_dur_cnt0_carry__2_O_UNCONNECTED;

  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT4 #(
    .INIT(16'h3354)) 
    \bit_cnt[0]_i_1 
       (.I0(\bit_cnt_reg_n_0_[0] ),
        .I1(\state_udp_reg_n_0_[2] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\state_udp_reg_n_0_[1] ),
        .O(p_1_in[0]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT5 #(
    .INIT(32'h00F9FF60)) 
    \bit_cnt[1]_i_1 
       (.I0(\bit_cnt_reg_n_0_[1] ),
        .I1(\bit_cnt_reg_n_0_[0] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\state_udp_reg_n_0_[1] ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(p_1_in[1]));
  LUT6 #(
    .INIT(64'h3C3E3E2E2E0C0C3C)) 
    \bit_cnt[2]_i_1 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[2] ),
        .I3(\bit_cnt_reg_n_0_[0] ),
        .I4(\bit_cnt_reg_n_0_[1] ),
        .I5(\bit_cnt_reg_n_0_[2] ),
        .O(p_1_in[2]));
  LUT6 #(
    .INIT(64'hBCCCCCCE88888882)) 
    \bit_cnt[3]_i_1 
       (.I0(\bit_cnt[6]_i_3_n_0 ),
        .I1(\bit_cnt_reg_n_0_[3] ),
        .I2(\bit_cnt_reg_n_0_[2] ),
        .I3(\bit_cnt_reg_n_0_[1] ),
        .I4(\bit_cnt_reg_n_0_[0] ),
        .I5(\bit_cnt[5]_i_4_n_0 ),
        .O(p_1_in[3]));
  LUT6 #(
    .INIT(64'h884488448FF48844)) 
    \bit_cnt[4]_i_1 
       (.I0(\bit_cnt[5]_i_2_n_0 ),
        .I1(\bit_cnt[6]_i_3_n_0 ),
        .I2(\bit_cnt[5]_i_3_n_0 ),
        .I3(\bit_cnt_reg_n_0_[4] ),
        .I4(\state_udp_reg_n_0_[0] ),
        .I5(\state_udp_reg_n_0_[2] ),
        .O(p_1_in[4]));
  LUT6 #(
    .INIT(64'hCFF4FF04C804C804)) 
    \bit_cnt[5]_i_1 
       (.I0(\bit_cnt[5]_i_2_n_0 ),
        .I1(\bit_cnt[6]_i_3_n_0 ),
        .I2(\bit_cnt_reg_n_0_[4] ),
        .I3(\bit_cnt_reg_n_0_[5] ),
        .I4(\bit_cnt[5]_i_3_n_0 ),
        .I5(\bit_cnt[5]_i_4_n_0 ),
        .O(p_1_in[5]));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT4 #(
    .INIT(16'hFFFE)) 
    \bit_cnt[5]_i_2 
       (.I0(\bit_cnt_reg_n_0_[3] ),
        .I1(\bit_cnt_reg_n_0_[0] ),
        .I2(\bit_cnt_reg_n_0_[1] ),
        .I3(\bit_cnt_reg_n_0_[2] ),
        .O(\bit_cnt[5]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT4 #(
    .INIT(16'h8000)) 
    \bit_cnt[5]_i_3 
       (.I0(\bit_cnt_reg_n_0_[0] ),
        .I1(\bit_cnt_reg_n_0_[1] ),
        .I2(\bit_cnt_reg_n_0_[2] ),
        .I3(\bit_cnt_reg_n_0_[3] ),
        .O(\bit_cnt[5]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \bit_cnt[5]_i_4 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp_reg_n_0_[2] ),
        .O(\bit_cnt[5]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'h448844884FF84488)) 
    \bit_cnt[6]_i_1 
       (.I0(\bit_cnt[6]_i_2_n_0 ),
        .I1(\bit_cnt[6]_i_3_n_0 ),
        .I2(\bit_cnt[7]_i_3_n_0 ),
        .I3(\bit_cnt_reg_n_0_[6] ),
        .I4(\state_udp_reg_n_0_[0] ),
        .I5(\state_udp_reg_n_0_[2] ),
        .O(p_1_in[6]));
  LUT6 #(
    .INIT(64'h0000000000000001)) 
    \bit_cnt[6]_i_2 
       (.I0(\bit_cnt_reg_n_0_[5] ),
        .I1(\bit_cnt_reg_n_0_[3] ),
        .I2(\bit_cnt_reg_n_0_[0] ),
        .I3(\bit_cnt_reg_n_0_[1] ),
        .I4(\bit_cnt_reg_n_0_[2] ),
        .I5(\bit_cnt_reg_n_0_[4] ),
        .O(\bit_cnt[6]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAAAAAAAAAAAA8)) 
    \bit_cnt[6]_i_3 
       (.I0(\bit_cnt[7]_i_5_n_0 ),
        .I1(\bit_cnt_reg_n_0_[6] ),
        .I2(\bit_cnt_reg_n_0_[7] ),
        .I3(\bit_cnt_reg_n_0_[5] ),
        .I4(\bit_cnt_reg_n_0_[4] ),
        .I5(\bit_cnt[5]_i_2_n_0 ),
        .O(\bit_cnt[6]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hFF0000EF00FFFFFF)) 
    \bit_cnt[7]_i_1 
       (.I0(\bit_cnt_reg_n_0_[7] ),
        .I1(\bit_cnt_reg_n_0_[6] ),
        .I2(\bit_cnt[7]_i_3_n_0 ),
        .I3(\state_udp_reg_n_0_[1] ),
        .I4(\state_udp_reg_n_0_[2] ),
        .I5(\state_udp_reg_n_0_[0] ),
        .O(\bit_cnt[7]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAAAAABFEAAAAA)) 
    \bit_cnt[7]_i_2 
       (.I0(\bit_cnt[7]_i_4_n_0 ),
        .I1(\bit_cnt_reg_n_0_[6] ),
        .I2(\bit_cnt[7]_i_3_n_0 ),
        .I3(\bit_cnt_reg_n_0_[7] ),
        .I4(\state_udp_reg_n_0_[0] ),
        .I5(\state_udp_reg_n_0_[2] ),
        .O(p_1_in[7]));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    \bit_cnt[7]_i_3 
       (.I0(\bit_cnt_reg_n_0_[3] ),
        .I1(\bit_cnt_reg_n_0_[2] ),
        .I2(\bit_cnt_reg_n_0_[1] ),
        .I3(\bit_cnt_reg_n_0_[0] ),
        .I4(\bit_cnt_reg_n_0_[5] ),
        .I5(\bit_cnt_reg_n_0_[4] ),
        .O(\bit_cnt[7]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hFFFE000000000000)) 
    \bit_cnt[7]_i_4 
       (.I0(\bit_cnt_reg_n_0_[5] ),
        .I1(\bit_cnt[5]_i_2_n_0 ),
        .I2(\bit_cnt_reg_n_0_[4] ),
        .I3(\bit_cnt_reg_n_0_[6] ),
        .I4(\bit_cnt[7]_i_5_n_0 ),
        .I5(\bit_cnt_reg_n_0_[7] ),
        .O(\bit_cnt[7]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \bit_cnt[7]_i_5 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\state_udp_reg_n_0_[1] ),
        .O(\bit_cnt[7]_i_5_n_0 ));
  FDCE \bit_cnt_reg[0] 
       (.C(clock),
        .CE(\bit_cnt[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(p_1_in[0]),
        .Q(\bit_cnt_reg_n_0_[0] ));
  FDCE \bit_cnt_reg[1] 
       (.C(clock),
        .CE(\bit_cnt[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(p_1_in[1]),
        .Q(\bit_cnt_reg_n_0_[1] ));
  FDCE \bit_cnt_reg[2] 
       (.C(clock),
        .CE(\bit_cnt[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(p_1_in[2]),
        .Q(\bit_cnt_reg_n_0_[2] ));
  FDCE \bit_cnt_reg[3] 
       (.C(clock),
        .CE(\bit_cnt[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(p_1_in[3]),
        .Q(\bit_cnt_reg_n_0_[3] ));
  FDCE \bit_cnt_reg[4] 
       (.C(clock),
        .CE(\bit_cnt[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(p_1_in[4]),
        .Q(\bit_cnt_reg_n_0_[4] ));
  FDCE \bit_cnt_reg[5] 
       (.C(clock),
        .CE(\bit_cnt[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(p_1_in[5]),
        .Q(\bit_cnt_reg_n_0_[5] ));
  FDCE \bit_cnt_reg[6] 
       (.C(clock),
        .CE(\bit_cnt[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(p_1_in[6]),
        .Q(\bit_cnt_reg_n_0_[6] ));
  FDCE \bit_cnt_reg[7] 
       (.C(clock),
        .CE(\bit_cnt[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(p_1_in[7]),
        .Q(\bit_cnt_reg_n_0_[7] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 bit_dur_cnt0_carry
       (.CI(1'b0),
        .CO({bit_dur_cnt0_carry_n_0,bit_dur_cnt0_carry_n_1,bit_dur_cnt0_carry_n_2,bit_dur_cnt0_carry_n_3}),
        .CYINIT(bit_dur_cnt[0]),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(bit_dur_cnt0[4:1]),
        .S(bit_dur_cnt[4:1]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 bit_dur_cnt0_carry__0
       (.CI(bit_dur_cnt0_carry_n_0),
        .CO({bit_dur_cnt0_carry__0_n_0,bit_dur_cnt0_carry__0_n_1,bit_dur_cnt0_carry__0_n_2,bit_dur_cnt0_carry__0_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(bit_dur_cnt0[8:5]),
        .S(bit_dur_cnt[8:5]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 bit_dur_cnt0_carry__1
       (.CI(bit_dur_cnt0_carry__0_n_0),
        .CO({bit_dur_cnt0_carry__1_n_0,bit_dur_cnt0_carry__1_n_1,bit_dur_cnt0_carry__1_n_2,bit_dur_cnt0_carry__1_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(bit_dur_cnt0[12:9]),
        .S(bit_dur_cnt[12:9]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 bit_dur_cnt0_carry__2
       (.CI(bit_dur_cnt0_carry__1_n_0),
        .CO({NLW_bit_dur_cnt0_carry__2_CO_UNCONNECTED[3:2],bit_dur_cnt0_carry__2_n_2,bit_dur_cnt0_carry__2_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({NLW_bit_dur_cnt0_carry__2_O_UNCONNECTED[3],bit_dur_cnt0[15:13]}),
        .S({1'b0,bit_dur_cnt[15:13]}));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT5 #(
    .INIT(32'h15111010)) 
    \bit_dur_cnt[0]_i_1 
       (.I0(bit_dur_cnt[0]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[0]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[10]_i_1 
       (.I0(bit_dur_cnt0[10]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[10]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[11]_i_1 
       (.I0(bit_dur_cnt0[11]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[11]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[12]_i_1 
       (.I0(bit_dur_cnt0[12]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[12]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[13]_i_1 
       (.I0(bit_dur_cnt0[13]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[13]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[14]_i_1 
       (.I0(bit_dur_cnt0[14]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[14]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'hB)) 
    \bit_dur_cnt[15]_i_1 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\state_udp_reg_n_0_[1] ),
        .O(\bit_dur_cnt[15]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[15]_i_2 
       (.I0(bit_dur_cnt0[15]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[15]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT4 #(
    .INIT(16'hFFFE)) 
    \bit_dur_cnt[15]_i_3 
       (.I0(\state_udp[2]_i_7_n_0 ),
        .I1(\state_udp[2]_i_6_n_0 ),
        .I2(\state_udp[2]_i_5_n_0 ),
        .I3(\state_udp[2]_i_4_n_0 ),
        .O(\bit_dur_cnt[15]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[1]_i_1 
       (.I0(bit_dur_cnt0[1]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[1]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[2]_i_1 
       (.I0(bit_dur_cnt0[2]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[2]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[3]_i_1 
       (.I0(bit_dur_cnt0[3]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[3]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[4]_i_1 
       (.I0(bit_dur_cnt0[4]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[4]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[5]_i_1 
       (.I0(bit_dur_cnt0[5]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[5]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[6]_i_1 
       (.I0(bit_dur_cnt0[6]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[6]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[7]_i_1 
       (.I0(bit_dur_cnt0[7]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[8]_i_1 
       (.I0(bit_dur_cnt0[8]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[8]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h2A222020)) 
    \bit_dur_cnt[9]_i_1 
       (.I0(bit_dur_cnt0[9]),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\bit_dur_cnt[15]_i_3_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\bit_dur_cnt[9]_i_1_n_0 ));
  FDCE \bit_dur_cnt_reg[0] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[0]_i_1_n_0 ),
        .Q(bit_dur_cnt[0]));
  FDCE \bit_dur_cnt_reg[10] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[10]_i_1_n_0 ),
        .Q(bit_dur_cnt[10]));
  FDCE \bit_dur_cnt_reg[11] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[11]_i_1_n_0 ),
        .Q(bit_dur_cnt[11]));
  FDCE \bit_dur_cnt_reg[12] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[12]_i_1_n_0 ),
        .Q(bit_dur_cnt[12]));
  FDCE \bit_dur_cnt_reg[13] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[13]_i_1_n_0 ),
        .Q(bit_dur_cnt[13]));
  FDCE \bit_dur_cnt_reg[14] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[14]_i_1_n_0 ),
        .Q(bit_dur_cnt[14]));
  FDCE \bit_dur_cnt_reg[15] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[15]_i_2_n_0 ),
        .Q(bit_dur_cnt[15]));
  FDCE \bit_dur_cnt_reg[1] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[1]_i_1_n_0 ),
        .Q(bit_dur_cnt[1]));
  FDCE \bit_dur_cnt_reg[2] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[2]_i_1_n_0 ),
        .Q(bit_dur_cnt[2]));
  FDCE \bit_dur_cnt_reg[3] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[3]_i_1_n_0 ),
        .Q(bit_dur_cnt[3]));
  FDCE \bit_dur_cnt_reg[4] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[4]_i_1_n_0 ),
        .Q(bit_dur_cnt[4]));
  FDCE \bit_dur_cnt_reg[5] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[5]_i_1_n_0 ),
        .Q(bit_dur_cnt[5]));
  FDCE \bit_dur_cnt_reg[6] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[6]_i_1_n_0 ),
        .Q(bit_dur_cnt[6]));
  FDCE \bit_dur_cnt_reg[7] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[7]_i_1_n_0 ),
        .Q(bit_dur_cnt[7]));
  FDCE \bit_dur_cnt_reg[8] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[8]_i_1_n_0 ),
        .Q(bit_dur_cnt[8]));
  FDCE \bit_dur_cnt_reg[9] 
       (.C(clock),
        .CE(\bit_dur_cnt[15]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\bit_dur_cnt[9]_i_1_n_0 ),
        .Q(bit_dur_cnt[9]));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'h04)) 
    out_bit_prmbl_end_i_1
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[2] ),
        .O(out_bit_prmbl_end_i_1_n_0));
  FDCE out_bit_prmbl_end_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(out_bit_prmbl_end_i_1_n_0),
        .Q(out_bit_prmbl_end));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT4 #(
    .INIT(16'h0020)) 
    out_bit_prmbl_strt_i_1
       (.I0(out_bit_prmbl_strt_i_2_n_0),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\state_udp_reg_n_0_[2] ),
        .O(out_bit_prmbl_strt_i_1_n_0));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT5 #(
    .INIT(32'h00000001)) 
    out_bit_prmbl_strt_i_2
       (.I0(\bit_cnt[5]_i_2_n_0 ),
        .I1(\bit_cnt_reg_n_0_[4] ),
        .I2(\bit_cnt_reg_n_0_[5] ),
        .I3(\bit_cnt_reg_n_0_[7] ),
        .I4(\bit_cnt_reg_n_0_[6] ),
        .O(out_bit_prmbl_strt_i_2_n_0));
  FDCE out_bit_prmbl_strt_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(out_bit_prmbl_strt_i_1_n_0),
        .Q(out_bit_prmbl_strt));
  LUT6 #(
    .INIT(64'hAFA0CFCFAFA0C0C0)) 
    out_bit_to_fsk_i_4
       (.I0(temp_Data[3]),
        .I1(temp_Data[2]),
        .I2(\bit_cnt_reg_n_0_[1] ),
        .I3(temp_Data[1]),
        .I4(\bit_cnt_reg_n_0_[0] ),
        .I5(temp_Data[0]),
        .O(out_bit_to_fsk_i_4_n_0));
  LUT6 #(
    .INIT(64'hAFA0CFCFAFA0C0C0)) 
    out_bit_to_fsk_i_5
       (.I0(temp_Data[7]),
        .I1(temp_Data[6]),
        .I2(\bit_cnt_reg_n_0_[1] ),
        .I3(temp_Data[5]),
        .I4(\bit_cnt_reg_n_0_[0] ),
        .I5(temp_Data[4]),
        .O(out_bit_to_fsk_i_5_n_0));
  FDCE out_bit_to_fsk_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_1),
        .Q(out_bit_to_fsk));
  MUXF7 out_bit_to_fsk_reg_i_3
       (.I0(out_bit_to_fsk_i_4_n_0),
        .I1(out_bit_to_fsk_i_5_n_0),
        .O(out_bit_to_fsk_reg_i_3_n_0),
        .S(\bit_cnt_reg_n_0_[2] ));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'h16)) 
    out_bit_valid_i_1
       (.I0(\state_udp_reg_n_0_[1] ),
        .I1(\state_udp_reg_n_0_[0] ),
        .I2(\state_udp_reg_n_0_[2] ),
        .O(out_bit_valid_i_1_n_0));
  FDCE out_bit_valid_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(out_bit_valid_i_1_n_0),
        .Q(out_bit_valid));
  LUT6 #(
    .INIT(64'hFF44FF33EF000303)) 
    preamble_en_i_1
       (.I0(preamble_en_i_2_n_0),
        .I1(\state_udp_reg_n_0_[0] ),
        .I2(\word_count[5]_i_2_n_0 ),
        .I3(\state_udp_reg_n_0_[2] ),
        .I4(\state_udp_reg_n_0_[1] ),
        .I5(preamble_en),
        .O(preamble_en_i_1_n_0));
  LUT6 #(
    .INIT(64'h0000000000000002)) 
    preamble_en_i_2
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(preamble_en_i_3_n_0),
        .I2(\state_udp[0]_i_3_n_0 ),
        .I3(\state_udp[0]_i_4_n_0 ),
        .I4(\state_udp[0]_i_5_n_0 ),
        .I5(\state_udp[0]_i_6_n_0 ),
        .O(preamble_en_i_2_n_0));
  LUT3 #(
    .INIT(8'hFB)) 
    preamble_en_i_3
       (.I0(\state_udp_nxt_reg_n_0_[2] ),
        .I1(\state_udp_nxt_reg_n_0_[0] ),
        .I2(\state_udp_nxt_reg_n_0_[1] ),
        .O(preamble_en_i_3_n_0));
  FDCE preamble_en_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(preamble_en_i_1_n_0),
        .Q(preamble_en));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr preamble_mod
       (.E(preamble_en),
        .clock(clock),
        .out_bit_to_fsk(out_bit_to_fsk),
        .out_bit_to_fsk_reg(out_bit_to_fsk_reg_i_3_n_0),
        .out_bit_to_fsk_reg_0(\state_udp_reg_n_0_[0] ),
        .out_bit_to_fsk_reg_1(\state_udp_reg_n_0_[2] ),
        .out_bit_to_fsk_reg_2(\state_udp_reg_n_0_[1] ),
        .pn_bit_reg_0(preamble_mod_n_1),
        .reset_n(reset_n),
        .reset_n_0(preamble_mod_n_0));
  LUT6 #(
    .INIT(64'h4400FFFF74330000)) 
    \state_udp[0]_i_1 
       (.I0(\state_udp[0]_i_2_n_0 ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(out_bit_prmbl_strt_i_2_n_0),
        .I3(\state_udp_reg_n_0_[2] ),
        .I4(\state_udp[2]_i_3_n_0 ),
        .I5(\state_udp_reg_n_0_[0] ),
        .O(\state_udp[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h5555555455555555)) 
    \state_udp[0]_i_2 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp[0]_i_3_n_0 ),
        .I2(\state_udp[0]_i_4_n_0 ),
        .I3(\state_udp[0]_i_5_n_0 ),
        .I4(\state_udp[0]_i_6_n_0 ),
        .I5(\state_udp_nxt_reg_n_0_[0] ),
        .O(\state_udp[0]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h7FFF)) 
    \state_udp[0]_i_3 
       (.I0(bit_dur_cnt[2]),
        .I1(bit_dur_cnt[3]),
        .I2(bit_dur_cnt[1]),
        .I3(bit_dur_cnt[0]),
        .O(\state_udp[0]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h7FFF)) 
    \state_udp[0]_i_4 
       (.I0(bit_dur_cnt[6]),
        .I1(bit_dur_cnt[7]),
        .I2(bit_dur_cnt[4]),
        .I3(bit_dur_cnt[5]),
        .O(\state_udp[0]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \state_udp[0]_i_5 
       (.I0(bit_dur_cnt[10]),
        .I1(bit_dur_cnt[13]),
        .I2(bit_dur_cnt[15]),
        .I3(bit_dur_cnt[14]),
        .O(\state_udp[0]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'hEFFF)) 
    \state_udp[0]_i_6 
       (.I0(bit_dur_cnt[8]),
        .I1(bit_dur_cnt[9]),
        .I2(bit_dur_cnt[11]),
        .I3(bit_dur_cnt[12]),
        .O(\state_udp[0]_i_6_n_0 ));
  LUT6 #(
    .INIT(64'h00FCFFFF55550000)) 
    \state_udp[1]_i_1 
       (.I0(\state_udp[1]_i_2_n_0 ),
        .I1(\state_udp_nxt_reg_n_0_[1] ),
        .I2(\state_udp[2]_i_2_n_0 ),
        .I3(\state_udp_reg_n_0_[0] ),
        .I4(\state_udp[2]_i_3_n_0 ),
        .I5(\state_udp_reg_n_0_[1] ),
        .O(\state_udp[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0000000155555555)) 
    \state_udp[1]_i_2 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\bit_cnt[5]_i_2_n_0 ),
        .I2(\bit_cnt_reg_n_0_[4] ),
        .I3(\bit_cnt_reg_n_0_[5] ),
        .I4(\state_udp[1]_i_3_n_0 ),
        .I5(\state_udp_reg_n_0_[2] ),
        .O(\state_udp[1]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \state_udp[1]_i_3 
       (.I0(\bit_cnt_reg_n_0_[7] ),
        .I1(\bit_cnt_reg_n_0_[6] ),
        .O(\state_udp[1]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0EFFFFFF0EE00000)) 
    \state_udp[2]_i_1 
       (.I0(\state_udp_nxt_reg_n_0_[2] ),
        .I1(\state_udp[2]_i_2_n_0 ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\state_udp_reg_n_0_[1] ),
        .I4(\state_udp[2]_i_3_n_0 ),
        .I5(\state_udp_reg_n_0_[2] ),
        .O(\state_udp[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT5 #(
    .INIT(32'hFFFEFFFF)) 
    \state_udp[2]_i_2 
       (.I0(\state_udp[2]_i_4_n_0 ),
        .I1(\state_udp[2]_i_5_n_0 ),
        .I2(\state_udp[2]_i_6_n_0 ),
        .I3(\state_udp[2]_i_7_n_0 ),
        .I4(\state_udp_reg_n_0_[2] ),
        .O(\state_udp[2]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT5 #(
    .INIT(32'hFEFEFEFF)) 
    \state_udp[2]_i_3 
       (.I0(\state_udp_reg_n_0_[1] ),
        .I1(\state_udp_reg_n_0_[0] ),
        .I2(\state_udp_reg_n_0_[2] ),
        .I3(\state_udp[2]_i_8_n_0 ),
        .I4(\state_udp[2]_i_9_n_0 ),
        .O(\state_udp[2]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'hDFFF)) 
    \state_udp[2]_i_4 
       (.I0(bit_dur_cnt[7]),
        .I1(bit_dur_cnt[13]),
        .I2(bit_dur_cnt[12]),
        .I3(bit_dur_cnt[3]),
        .O(\state_udp[2]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'hFF7F)) 
    \state_udp[2]_i_5 
       (.I0(bit_dur_cnt[11]),
        .I1(bit_dur_cnt[2]),
        .I2(bit_dur_cnt[0]),
        .I3(bit_dur_cnt[10]),
        .O(\state_udp[2]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'hFFDF)) 
    \state_udp[2]_i_6 
       (.I0(bit_dur_cnt[1]),
        .I1(bit_dur_cnt[9]),
        .I2(bit_dur_cnt[5]),
        .I3(bit_dur_cnt[14]),
        .O(\state_udp[2]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'hFFDF)) 
    \state_udp[2]_i_7 
       (.I0(bit_dur_cnt[4]),
        .I1(bit_dur_cnt[15]),
        .I2(bit_dur_cnt[6]),
        .I3(bit_dur_cnt[8]),
        .O(\state_udp[2]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'hFFF7)) 
    \state_udp[2]_i_8 
       (.I0(word_count[1]),
        .I1(word_count[4]),
        .I2(word_count[3]),
        .I3(word_count[7]),
        .O(\state_udp[2]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'hFFEF)) 
    \state_udp[2]_i_9 
       (.I0(word_count[2]),
        .I1(word_count[6]),
        .I2(word_count[5]),
        .I3(word_count[0]),
        .O(\state_udp[2]_i_9_n_0 ));
  LUT5 #(
    .INIT(32'h8AFF8A00)) 
    \state_udp_nxt[0]_i_1 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp_nxt[0]_i_2_n_0 ),
        .I2(\word_count[7]_i_3_n_0 ),
        .I3(\state_udp_nxt[1]_i_3_n_0 ),
        .I4(\state_udp_nxt_reg_n_0_[0] ),
        .O(\state_udp_nxt[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT4 #(
    .INIT(16'h5455)) 
    \state_udp_nxt[0]_i_2 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\bit_cnt_reg_n_0_[7] ),
        .I2(\bit_cnt_reg_n_0_[6] ),
        .I3(\bit_cnt[7]_i_3_n_0 ),
        .O(\state_udp_nxt[0]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h4C44FFFF4C440000)) 
    \state_udp_nxt[1]_i_1 
       (.I0(\word_count[7]_i_3_n_0 ),
        .I1(\state_udp_reg_n_0_[0] ),
        .I2(\state_udp_reg_n_0_[2] ),
        .I3(\state_udp_nxt[1]_i_2_n_0 ),
        .I4(\state_udp_nxt[1]_i_3_n_0 ),
        .I5(\state_udp_nxt_reg_n_0_[1] ),
        .O(\state_udp_nxt[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'h02)) 
    \state_udp_nxt[1]_i_2 
       (.I0(\bit_cnt[7]_i_3_n_0 ),
        .I1(\bit_cnt_reg_n_0_[6] ),
        .I2(\bit_cnt_reg_n_0_[7] ),
        .O(\state_udp_nxt[1]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT5 #(
    .INIT(32'h0055FEAA)) 
    \state_udp_nxt[1]_i_3 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\bit_cnt[5]_i_2_n_0 ),
        .I2(\state_udp_nxt[1]_i_4_n_0 ),
        .I3(\state_udp_reg_n_0_[2] ),
        .I4(\state_udp_reg_n_0_[1] ),
        .O(\state_udp_nxt[1]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT4 #(
    .INIT(16'hFFFE)) 
    \state_udp_nxt[1]_i_4 
       (.I0(\bit_cnt_reg_n_0_[6] ),
        .I1(\bit_cnt_reg_n_0_[7] ),
        .I2(\bit_cnt_reg_n_0_[5] ),
        .I3(\bit_cnt_reg_n_0_[4] ),
        .O(\state_udp_nxt[1]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT5 #(
    .INIT(32'hFFF505B0)) 
    \state_udp_nxt[2]_i_1 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(out_bit_prmbl_strt_i_2_n_0),
        .I2(\state_udp_reg_n_0_[2] ),
        .I3(\state_udp_reg_n_0_[1] ),
        .I4(\state_udp_nxt_reg_n_0_[2] ),
        .O(\state_udp_nxt[2]_i_1_n_0 ));
  FDCE \state_udp_nxt_reg[0] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\state_udp_nxt[0]_i_1_n_0 ),
        .Q(\state_udp_nxt_reg_n_0_[0] ));
  FDCE \state_udp_nxt_reg[1] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\state_udp_nxt[1]_i_1_n_0 ),
        .Q(\state_udp_nxt_reg_n_0_[1] ));
  FDCE \state_udp_nxt_reg[2] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\state_udp_nxt[2]_i_1_n_0 ),
        .Q(\state_udp_nxt_reg_n_0_[2] ));
  FDCE \state_udp_reg[0] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\state_udp[0]_i_1_n_0 ),
        .Q(\state_udp_reg_n_0_[0] ));
  FDCE \state_udp_reg[1] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\state_udp[1]_i_1_n_0 ),
        .Q(\state_udp_reg_n_0_[1] ));
  FDCE \state_udp_reg[2] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\state_udp[2]_i_1_n_0 ),
        .Q(\state_udp_reg_n_0_[2] ));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT4 #(
    .INIT(16'hFFDF)) 
    \temp_Data[0]_i_1 
       (.I0(\word_count_reg_rep_n_0_[3] ),
        .I1(\temp_Data[4]_i_2_n_0 ),
        .I2(\state_udp_reg_n_0_[2] ),
        .I3(\temp_Data[7]_i_3_n_0 ),
        .O(\temp_Data[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT5 #(
    .INIT(32'hA8AA8888)) 
    \temp_Data[1]_i_1 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\temp_Data[7]_i_3_n_0 ),
        .I2(\temp_Data[4]_i_2_n_0 ),
        .I3(\word_count_reg_rep_n_0_[3] ),
        .I4(\word_count_reg_rep_n_0_[0] ),
        .O(\temp_Data[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT5 #(
    .INIT(32'hA8AA8888)) 
    \temp_Data[2]_i_1 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\temp_Data[7]_i_3_n_0 ),
        .I2(\temp_Data[4]_i_2_n_0 ),
        .I3(\word_count_reg_rep_n_0_[3] ),
        .I4(\word_count_reg_rep_n_0_[1] ),
        .O(\temp_Data[2]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hA8AA8888)) 
    \temp_Data[3]_i_1 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\temp_Data[7]_i_3_n_0 ),
        .I2(\temp_Data[4]_i_2_n_0 ),
        .I3(\word_count_reg_rep_n_0_[3] ),
        .I4(\word_count_reg_rep_n_0_[2] ),
        .O(\temp_Data[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT4 #(
    .INIT(16'h0080)) 
    \temp_Data[4]_i_1 
       (.I0(\word_count_reg_rep_n_0_[3] ),
        .I1(\temp_Data[4]_i_2_n_0 ),
        .I2(\state_udp_reg_n_0_[2] ),
        .I3(\temp_Data[7]_i_3_n_0 ),
        .O(\temp_Data[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT4 #(
    .INIT(16'h7FFF)) 
    \temp_Data[4]_i_2 
       (.I0(\word_count_reg_rep_n_0_[6] ),
        .I1(\word_count_reg_rep_n_0_[5] ),
        .I2(\word_count_reg_rep_n_0_[4] ),
        .I3(\word_count_reg_rep_n_0_[7] ),
        .O(\temp_Data[4]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hA8)) 
    \temp_Data[5]_i_1 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\temp_Data[7]_i_3_n_0 ),
        .I2(\temp_Data[5]_i_2_n_0 ),
        .O(\temp_Data[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT5 #(
    .INIT(32'h7FFF0000)) 
    \temp_Data[5]_i_2 
       (.I0(\word_count_reg_rep_n_0_[5] ),
        .I1(\word_count_reg_rep_n_0_[6] ),
        .I2(\word_count_reg_rep_n_0_[7] ),
        .I3(\word_count_reg_rep_n_0_[3] ),
        .I4(\word_count_reg_rep_n_0_[4] ),
        .O(\temp_Data[5]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'hA8)) 
    \temp_Data[6]_i_1 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\temp_Data[7]_i_3_n_0 ),
        .I2(\temp_Data[6]_i_2_n_0 ),
        .O(\temp_Data[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT5 #(
    .INIT(32'h7FFF0000)) 
    \temp_Data[6]_i_2 
       (.I0(\word_count_reg_rep_n_0_[4] ),
        .I1(\word_count_reg_rep_n_0_[6] ),
        .I2(\word_count_reg_rep_n_0_[7] ),
        .I3(\word_count_reg_rep_n_0_[3] ),
        .I4(\word_count_reg_rep_n_0_[5] ),
        .O(\temp_Data[6]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1044)) 
    \temp_Data[7]_i_1 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(out_bit_prmbl_strt_i_2_n_0),
        .I3(\state_udp_reg_n_0_[2] ),
        .O(\temp_Data[7]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hA8AAAAAA88888888)) 
    \temp_Data[7]_i_2 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\temp_Data[7]_i_3_n_0 ),
        .I2(\temp_Data[7]_i_4_n_0 ),
        .I3(\word_count_reg_rep_n_0_[7] ),
        .I4(\word_count_reg_rep_n_0_[3] ),
        .I5(\word_count_reg_rep_n_0_[6] ),
        .O(\temp_Data[7]_i_2_n_0 ));
  LUT5 #(
    .INIT(32'h80000000)) 
    \temp_Data[7]_i_3 
       (.I0(word_count[3]),
        .I1(word_count[4]),
        .I2(word_count[6]),
        .I3(word_count[5]),
        .I4(word_count[7]),
        .O(\temp_Data[7]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT2 #(
    .INIT(4'h7)) 
    \temp_Data[7]_i_4 
       (.I0(\word_count_reg_rep_n_0_[5] ),
        .I1(\word_count_reg_rep_n_0_[4] ),
        .O(\temp_Data[7]_i_4_n_0 ));
  FDCE \temp_Data_reg[0] 
       (.C(clock),
        .CE(\temp_Data[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\temp_Data[0]_i_1_n_0 ),
        .Q(temp_Data[0]));
  FDCE \temp_Data_reg[1] 
       (.C(clock),
        .CE(\temp_Data[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\temp_Data[1]_i_1_n_0 ),
        .Q(temp_Data[1]));
  FDCE \temp_Data_reg[2] 
       (.C(clock),
        .CE(\temp_Data[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\temp_Data[2]_i_1_n_0 ),
        .Q(temp_Data[2]));
  FDCE \temp_Data_reg[3] 
       (.C(clock),
        .CE(\temp_Data[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\temp_Data[3]_i_1_n_0 ),
        .Q(temp_Data[3]));
  FDCE \temp_Data_reg[4] 
       (.C(clock),
        .CE(\temp_Data[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\temp_Data[4]_i_1_n_0 ),
        .Q(temp_Data[4]));
  FDCE \temp_Data_reg[5] 
       (.C(clock),
        .CE(\temp_Data[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\temp_Data[5]_i_1_n_0 ),
        .Q(temp_Data[5]));
  FDCE \temp_Data_reg[6] 
       (.C(clock),
        .CE(\temp_Data[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\temp_Data[6]_i_1_n_0 ),
        .Q(temp_Data[6]));
  FDCE \temp_Data_reg[7] 
       (.C(clock),
        .CE(\temp_Data[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\temp_Data[7]_i_2_n_0 ),
        .Q(temp_Data[7]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT4 #(
    .INIT(16'h2322)) 
    \word_count[0]_i_1 
       (.I0(\state_udp_reg_n_0_[1] ),
        .I1(\state_udp_reg_n_0_[0] ),
        .I2(word_count[0]),
        .I3(\word_count[5]_i_2_n_0 ),
        .O(\word_count[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT5 #(
    .INIT(32'h00020200)) 
    \word_count[1]_i_1 
       (.I0(\word_count[5]_i_2_n_0 ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(word_count[0]),
        .I4(word_count[1]),
        .O(\word_count[1]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h01111000)) 
    \word_count[2]_i_1 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(word_count[1]),
        .I3(word_count[0]),
        .I4(word_count[2]),
        .O(\word_count[2]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0111111110000000)) 
    \word_count[3]_i_1 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(word_count[0]),
        .I3(word_count[1]),
        .I4(word_count[2]),
        .I5(word_count[3]),
        .O(\word_count[3]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0200020200020000)) 
    \word_count[4]_i_1 
       (.I0(\word_count[5]_i_2_n_0 ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\state_udp_reg_n_0_[0] ),
        .I3(\word_count[6]_i_3_n_0 ),
        .I4(word_count[3]),
        .I5(word_count[4]),
        .O(\word_count[4]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h8088888808000000)) 
    \word_count[5]_i_1 
       (.I0(\word_count[5]_i_2_n_0 ),
        .I1(\word_count[6]_i_2_n_0 ),
        .I2(\word_count[6]_i_3_n_0 ),
        .I3(word_count[3]),
        .I4(word_count[4]),
        .I5(word_count[5]),
        .O(\word_count[5]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFEFFFF)) 
    \word_count[5]_i_2 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(\state_udp[2]_i_8_n_0 ),
        .I2(word_count[2]),
        .I3(word_count[6]),
        .I4(word_count[5]),
        .I5(word_count[0]),
        .O(\word_count[5]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hAAAA2AAA00008000)) 
    \word_count[6]_i_1 
       (.I0(\word_count[6]_i_2_n_0 ),
        .I1(word_count[5]),
        .I2(word_count[4]),
        .I3(word_count[3]),
        .I4(\word_count[6]_i_3_n_0 ),
        .I5(word_count[6]),
        .O(\word_count[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT2 #(
    .INIT(4'h1)) 
    \word_count[6]_i_2 
       (.I0(\state_udp_reg_n_0_[1] ),
        .I1(\state_udp_reg_n_0_[0] ),
        .O(\word_count[6]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'h7F)) 
    \word_count[6]_i_3 
       (.I0(word_count[0]),
        .I1(word_count[1]),
        .I2(word_count[2]),
        .O(\word_count[6]_i_3_n_0 ));
  LUT5 #(
    .INIT(32'h005D0F5D)) 
    \word_count[7]_i_1 
       (.I0(\state_udp_reg_n_0_[2] ),
        .I1(out_bit_prmbl_strt_i_2_n_0),
        .I2(\state_udp_reg_n_0_[1] ),
        .I3(\state_udp_reg_n_0_[0] ),
        .I4(\word_count[7]_i_3_n_0 ),
        .O(\word_count[7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT5 #(
    .INIT(32'h01111000)) 
    \word_count[7]_i_2 
       (.I0(\state_udp_reg_n_0_[0] ),
        .I1(\state_udp_reg_n_0_[1] ),
        .I2(\word_count[7]_i_4_n_0 ),
        .I3(word_count[6]),
        .I4(word_count[7]),
        .O(\word_count[7]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT5 #(
    .INIT(32'hFBFFFFFF)) 
    \word_count[7]_i_3 
       (.I0(word_count[2]),
        .I1(\state_udp_reg_n_0_[2] ),
        .I2(word_count[1]),
        .I3(word_count[0]),
        .I4(\temp_Data[7]_i_3_n_0 ),
        .O(\word_count[7]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    \word_count[7]_i_4 
       (.I0(word_count[5]),
        .I1(word_count[4]),
        .I2(word_count[3]),
        .I3(word_count[0]),
        .I4(word_count[1]),
        .I5(word_count[2]),
        .O(\word_count[7]_i_4_n_0 ));
  FDCE \word_count_reg[0] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[0]_i_1_n_0 ),
        .Q(word_count[0]));
  FDCE \word_count_reg[1] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[1]_i_1_n_0 ),
        .Q(word_count[1]));
  FDCE \word_count_reg[2] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[2]_i_1_n_0 ),
        .Q(word_count[2]));
  FDCE \word_count_reg[3] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[3]_i_1_n_0 ),
        .Q(word_count[3]));
  FDCE \word_count_reg[4] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[4]_i_1_n_0 ),
        .Q(word_count[4]));
  FDCE \word_count_reg[5] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[5]_i_1_n_0 ),
        .Q(word_count[5]));
  FDCE \word_count_reg[6] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[6]_i_1_n_0 ),
        .Q(word_count[6]));
  FDCE \word_count_reg[7] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[7]_i_2_n_0 ),
        .Q(word_count[7]));
  (* equivalent_register_removal = "no" *) 
  FDCE \word_count_reg_rep[0] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[0]_i_1_n_0 ),
        .Q(\word_count_reg_rep_n_0_[0] ));
  (* equivalent_register_removal = "no" *) 
  FDCE \word_count_reg_rep[1] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[1]_i_1_n_0 ),
        .Q(\word_count_reg_rep_n_0_[1] ));
  (* equivalent_register_removal = "no" *) 
  FDCE \word_count_reg_rep[2] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[2]_i_1_n_0 ),
        .Q(\word_count_reg_rep_n_0_[2] ));
  (* equivalent_register_removal = "no" *) 
  FDCE \word_count_reg_rep[3] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[3]_i_1_n_0 ),
        .Q(\word_count_reg_rep_n_0_[3] ));
  (* equivalent_register_removal = "no" *) 
  FDCE \word_count_reg_rep[4] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[4]_i_1_n_0 ),
        .Q(\word_count_reg_rep_n_0_[4] ));
  (* equivalent_register_removal = "no" *) 
  FDCE \word_count_reg_rep[5] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[5]_i_1_n_0 ),
        .Q(\word_count_reg_rep_n_0_[5] ));
  (* equivalent_register_removal = "no" *) 
  FDCE \word_count_reg_rep[6] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[6]_i_1_n_0 ),
        .Q(\word_count_reg_rep_n_0_[6] ));
  (* equivalent_register_removal = "no" *) 
  FDCE \word_count_reg_rep[7] 
       (.C(clock),
        .CE(\word_count[7]_i_1_n_0 ),
        .CLR(preamble_mod_n_0),
        .D(\word_count[7]_i_2_n_0 ),
        .Q(\word_count_reg_rep_n_0_[7] ));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr
   (reset_n_0,
    pn_bit_reg_0,
    E,
    clock,
    reset_n,
    out_bit_to_fsk_reg,
    out_bit_to_fsk_reg_0,
    out_bit_to_fsk_reg_1,
    out_bit_to_fsk_reg_2,
    out_bit_to_fsk);
  output reset_n_0;
  output pn_bit_reg_0;
  input [0:0]E;
  input clock;
  input reset_n;
  input out_bit_to_fsk_reg;
  input out_bit_to_fsk_reg_0;
  input out_bit_to_fsk_reg_1;
  input out_bit_to_fsk_reg_2;
  input out_bit_to_fsk;

  wire [0:0]E;
  wire clock;
  wire [7:0]delay_cntr;
  wire \delay_cntr[7]_i_2_n_0 ;
  wire [7:0]delay_cntr_0;
  wire out_bit_to_fsk;
  wire out_bit_to_fsk_reg;
  wire out_bit_to_fsk_reg_0;
  wire out_bit_to_fsk_reg_1;
  wire out_bit_to_fsk_reg_2;
  wire p_0_in;
  wire pn_bit_reg_0;
  wire preamble_bit;
  wire reset_n;
  wire reset_n_0;
  wire [7:0]shift_reg;
  wire \shift_reg[7]_i_2_n_0 ;
  wire \shift_reg_reg_n_0_[0] ;
  wire \shift_reg_reg_n_0_[1] ;
  wire \shift_reg_reg_n_0_[2] ;
  wire \shift_reg_reg_n_0_[3] ;
  wire \shift_reg_reg_n_0_[4] ;
  wire \shift_reg_reg_n_0_[5] ;
  wire \shift_reg_reg_n_0_[6] ;

  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \delay_cntr[0]_i_1 
       (.I0(delay_cntr[0]),
        .O(delay_cntr_0[0]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \delay_cntr[1]_i_1 
       (.I0(delay_cntr[0]),
        .I1(delay_cntr[1]),
        .O(delay_cntr_0[1]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    \delay_cntr[2]_i_1 
       (.I0(delay_cntr[2]),
        .I1(delay_cntr[1]),
        .I2(delay_cntr[0]),
        .O(delay_cntr_0[2]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    \delay_cntr[3]_i_1 
       (.I0(delay_cntr[3]),
        .I1(delay_cntr[0]),
        .I2(delay_cntr[1]),
        .I3(delay_cntr[2]),
        .O(delay_cntr_0[3]));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h6AAAAAAA)) 
    \delay_cntr[4]_i_1 
       (.I0(delay_cntr[4]),
        .I1(delay_cntr[2]),
        .I2(delay_cntr[3]),
        .I3(delay_cntr[0]),
        .I4(delay_cntr[1]),
        .O(delay_cntr_0[4]));
  LUT6 #(
    .INIT(64'h6AAAAAAAAAAAAAAA)) 
    \delay_cntr[5]_i_1 
       (.I0(delay_cntr[5]),
        .I1(delay_cntr[1]),
        .I2(delay_cntr[0]),
        .I3(delay_cntr[3]),
        .I4(delay_cntr[2]),
        .I5(delay_cntr[4]),
        .O(delay_cntr_0[5]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h3FFF8000)) 
    \delay_cntr[6]_i_1 
       (.I0(delay_cntr[7]),
        .I1(delay_cntr[5]),
        .I2(\delay_cntr[7]_i_2_n_0 ),
        .I3(delay_cntr[4]),
        .I4(delay_cntr[6]),
        .O(delay_cntr_0[6]));
  LUT5 #(
    .INIT(32'h6AAAAAAA)) 
    \delay_cntr[7]_i_1 
       (.I0(delay_cntr[7]),
        .I1(delay_cntr[6]),
        .I2(delay_cntr[5]),
        .I3(\delay_cntr[7]_i_2_n_0 ),
        .I4(delay_cntr[4]),
        .O(delay_cntr_0[7]));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT4 #(
    .INIT(16'h8000)) 
    \delay_cntr[7]_i_2 
       (.I0(delay_cntr[1]),
        .I1(delay_cntr[0]),
        .I2(delay_cntr[3]),
        .I3(delay_cntr[2]),
        .O(\delay_cntr[7]_i_2_n_0 ));
  FDCE \delay_cntr_reg[0] 
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(delay_cntr_0[0]),
        .Q(delay_cntr[0]));
  FDCE \delay_cntr_reg[1] 
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(delay_cntr_0[1]),
        .Q(delay_cntr[1]));
  FDCE \delay_cntr_reg[2] 
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(delay_cntr_0[2]),
        .Q(delay_cntr[2]));
  FDCE \delay_cntr_reg[3] 
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(delay_cntr_0[3]),
        .Q(delay_cntr[3]));
  FDCE \delay_cntr_reg[4] 
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(delay_cntr_0[4]),
        .Q(delay_cntr[4]));
  FDCE \delay_cntr_reg[5] 
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(delay_cntr_0[5]),
        .Q(delay_cntr[5]));
  FDCE \delay_cntr_reg[6] 
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(delay_cntr_0[6]),
        .Q(delay_cntr[6]));
  FDCE \delay_cntr_reg[7] 
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(delay_cntr_0[7]),
        .Q(delay_cntr[7]));
  LUT6 #(
    .INIT(64'h0FFCFACF000C0AC0)) 
    out_bit_to_fsk_i_1
       (.I0(out_bit_to_fsk_reg),
        .I1(preamble_bit),
        .I2(out_bit_to_fsk_reg_0),
        .I3(out_bit_to_fsk_reg_1),
        .I4(out_bit_to_fsk_reg_2),
        .I5(out_bit_to_fsk),
        .O(pn_bit_reg_0));
  LUT1 #(
    .INIT(2'h1)) 
    out_bit_to_fsk_i_2
       (.I0(reset_n),
        .O(reset_n_0));
  FDCE pn_bit_reg
       (.C(clock),
        .CE(E),
        .CLR(reset_n_0),
        .D(p_0_in),
        .Q(preamble_bit));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT5 #(
    .INIT(32'hBEEBEBBE)) 
    \shift_reg[0]_i_1 
       (.I0(\shift_reg[7]_i_2_n_0 ),
        .I1(p_0_in),
        .I2(\shift_reg_reg_n_0_[0] ),
        .I3(\shift_reg_reg_n_0_[1] ),
        .I4(\shift_reg_reg_n_0_[6] ),
        .O(shift_reg[0]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \shift_reg[1]_i_1 
       (.I0(\shift_reg_reg_n_0_[0] ),
        .I1(\shift_reg[7]_i_2_n_0 ),
        .O(shift_reg[1]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \shift_reg[2]_i_1 
       (.I0(\shift_reg_reg_n_0_[1] ),
        .I1(\shift_reg[7]_i_2_n_0 ),
        .O(shift_reg[2]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \shift_reg[3]_i_1 
       (.I0(\shift_reg_reg_n_0_[2] ),
        .I1(\shift_reg[7]_i_2_n_0 ),
        .O(shift_reg[3]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \shift_reg[4]_i_1 
       (.I0(\shift_reg_reg_n_0_[3] ),
        .I1(\shift_reg[7]_i_2_n_0 ),
        .O(shift_reg[4]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \shift_reg[5]_i_1 
       (.I0(\shift_reg_reg_n_0_[4] ),
        .I1(\shift_reg[7]_i_2_n_0 ),
        .O(shift_reg[5]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \shift_reg[6]_i_1 
       (.I0(\shift_reg_reg_n_0_[5] ),
        .I1(\shift_reg[7]_i_2_n_0 ),
        .O(shift_reg[6]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \shift_reg[7]_i_1 
       (.I0(\shift_reg_reg_n_0_[6] ),
        .I1(\shift_reg[7]_i_2_n_0 ),
        .O(shift_reg[7]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h00000080)) 
    \shift_reg[7]_i_2 
       (.I0(\delay_cntr[7]_i_2_n_0 ),
        .I1(delay_cntr[4]),
        .I2(delay_cntr[5]),
        .I3(delay_cntr[7]),
        .I4(delay_cntr[6]),
        .O(\shift_reg[7]_i_2_n_0 ));
  FDPE \shift_reg_reg[0] 
       (.C(clock),
        .CE(E),
        .D(shift_reg[0]),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[0] ));
  FDPE \shift_reg_reg[1] 
       (.C(clock),
        .CE(E),
        .D(shift_reg[1]),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[1] ));
  FDPE \shift_reg_reg[2] 
       (.C(clock),
        .CE(E),
        .D(shift_reg[2]),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[2] ));
  FDPE \shift_reg_reg[3] 
       (.C(clock),
        .CE(E),
        .D(shift_reg[3]),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[3] ));
  FDPE \shift_reg_reg[4] 
       (.C(clock),
        .CE(E),
        .D(shift_reg[4]),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[4] ));
  FDPE \shift_reg_reg[5] 
       (.C(clock),
        .CE(E),
        .D(shift_reg[5]),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[5] ));
  FDPE \shift_reg_reg[6] 
       (.C(clock),
        .CE(E),
        .D(shift_reg[6]),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[6] ));
  FDPE \shift_reg_reg[7] 
       (.C(clock),
        .CE(E),
        .D(shift_reg[7]),
        .PRE(reset_n_0),
        .Q(p_0_in));
endmodule

(* CHECK_LICENSE_TYPE = "system_Data_pack_preamble_2_0_0,Data_pack_preamble_2FSK_upd,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "Data_pack_preamble_2FSK_upd,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (clock,
    reset_n,
    out_bit_to_fsk,
    out_bit_valid,
    out_bit_prmbl_strt,
    out_bit_prmbl_end);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clock CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clock, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input clock;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 reset_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input reset_n;
  output out_bit_to_fsk;
  (* X_INTERFACE_INFO = "analog.com:interface:fifo_rd:1.0 out_bit VALID" *) output out_bit_valid;
  output out_bit_prmbl_strt;
  output out_bit_prmbl_end;

  wire clock;
  wire out_bit_prmbl_end;
  wire out_bit_prmbl_strt;
  wire out_bit_to_fsk;
  wire out_bit_valid;
  wire reset_n;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_Data_pack_preamble_2FSK_upd inst
       (.clock(clock),
        .out_bit_prmbl_end(out_bit_prmbl_end),
        .out_bit_prmbl_strt(out_bit_prmbl_strt),
        .out_bit_to_fsk(out_bit_to_fsk),
        .out_bit_valid(out_bit_valid),
        .reset_n(reset_n));
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
