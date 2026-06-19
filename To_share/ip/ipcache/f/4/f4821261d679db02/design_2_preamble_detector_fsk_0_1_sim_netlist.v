// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Mon Jan 12 12:01:51 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_2_preamble_detector_fsk_0_1_sim_netlist.v
// Design      : design_2_preamble_detector_fsk_0_1
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr
   (reset_n_0,
    D,
    pn_bit_reg_0,
    \preamble_det_bit_count_reg[7] ,
    clock,
    reset_n,
    reset_prng,
    Q,
    bsync_val_in,
    \FSM_onehot_preamble_Stm_reg[0] ,
    delay_3_indata,
    \preamble_det_bit_count_reg[7]_0 ,
    \preamble_det_bit_count_reg[5] ,
    \preamble_det_bit_count_reg[7]_1 ,
    preamble_en);
  output reset_n_0;
  output [1:0]D;
  output pn_bit_reg_0;
  output [7:0]\preamble_det_bit_count_reg[7] ;
  input clock;
  input reset_n;
  input reset_prng;
  input [3:0]Q;
  input bsync_val_in;
  input \FSM_onehot_preamble_Stm_reg[0] ;
  input delay_3_indata;
  input [7:0]\preamble_det_bit_count_reg[7]_0 ;
  input \preamble_det_bit_count_reg[5] ;
  input \preamble_det_bit_count_reg[7]_1 ;
  input preamble_en;

  wire [1:0]D;
  wire \FSM_onehot_preamble_Stm[3]_i_3_n_0 ;
  wire \FSM_onehot_preamble_Stm_reg[0] ;
  wire [3:0]Q;
  wire bsync_val_in;
  wire clock;
  wire delay_3_indata;
  wire [5:0]delay_cntr;
  wire \delay_cntr[3]_i_1_n_0 ;
  wire \delay_cntr[4]_i_2_n_0 ;
  wire \delay_cntr[5]_i_1_n_0 ;
  wire \delay_cntr[5]_i_2_n_0 ;
  wire \delay_cntr[5]_i_3_n_0 ;
  wire [4:0]p_0_in;
  wire p_0_in_0;
  wire pn_bit;
  wire pn_bit_i_1_n_0;
  wire pn_bit_reg_0;
  wire \preamble_det_bit_count_reg[5] ;
  wire [7:0]\preamble_det_bit_count_reg[7] ;
  wire [7:0]\preamble_det_bit_count_reg[7]_0 ;
  wire \preamble_det_bit_count_reg[7]_1 ;
  wire preamble_en;
  wire reset_n;
  wire reset_n_0;
  wire reset_prng;
  wire \shift_reg[0]_i_1_n_0 ;
  wire \shift_reg[1]_i_1_n_0 ;
  wire \shift_reg[2]_i_1_n_0 ;
  wire \shift_reg[3]_i_1_n_0 ;
  wire \shift_reg[4]_i_1_n_0 ;
  wire \shift_reg[5]_i_1_n_0 ;
  wire \shift_reg[6]_i_1_n_0 ;
  wire \shift_reg[7]_i_1_n_0 ;
  wire \shift_reg[7]_i_2_n_0 ;
  wire \shift_reg_reg_n_0_[0] ;
  wire \shift_reg_reg_n_0_[1] ;
  wire \shift_reg_reg_n_0_[2] ;
  wire \shift_reg_reg_n_0_[3] ;
  wire \shift_reg_reg_n_0_[4] ;
  wire \shift_reg_reg_n_0_[5] ;
  wire \shift_reg_reg_n_0_[6] ;

  LUT6 #(
    .INIT(64'hFFFFEFEAEFEAEFEA)) 
    \FSM_onehot_preamble_Stm[0]_i_1 
       (.I0(pn_bit_reg_0),
        .I1(Q[1]),
        .I2(bsync_val_in),
        .I3(Q[0]),
        .I4(Q[3]),
        .I5(\FSM_onehot_preamble_Stm_reg[0] ),
        .O(D[0]));
  LUT6 #(
    .INIT(64'h0100FFFF01000100)) 
    \FSM_onehot_preamble_Stm[3]_i_1 
       (.I0(\preamble_det_bit_count_reg[7]_0 [7]),
        .I1(\preamble_det_bit_count_reg[7]_1 ),
        .I2(\preamble_det_bit_count_reg[7]_0 [6]),
        .I3(\FSM_onehot_preamble_Stm[3]_i_3_n_0 ),
        .I4(bsync_val_in),
        .I5(Q[2]),
        .O(D[1]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'h82)) 
    \FSM_onehot_preamble_Stm[3]_i_3 
       (.I0(Q[3]),
        .I1(pn_bit),
        .I2(delay_3_indata),
        .O(\FSM_onehot_preamble_Stm[3]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    delay_1_indata_i_1
       (.I0(reset_n),
        .O(reset_n_0));
  LUT3 #(
    .INIT(8'h15)) 
    \delay_cntr[0]_i_1 
       (.I0(delay_cntr[0]),
        .I1(reset_n),
        .I2(reset_prng),
        .O(p_0_in[0]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT4 #(
    .INIT(16'h0666)) 
    \delay_cntr[1]_i_1 
       (.I0(delay_cntr[1]),
        .I1(delay_cntr[0]),
        .I2(reset_n),
        .I3(reset_prng),
        .O(p_0_in[1]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT5 #(
    .INIT(32'h00787878)) 
    \delay_cntr[2]_i_1 
       (.I0(delay_cntr[0]),
        .I1(delay_cntr[1]),
        .I2(delay_cntr[2]),
        .I3(reset_n),
        .I4(reset_prng),
        .O(p_0_in[2]));
  LUT6 #(
    .INIT(64'h0777777770000000)) 
    \delay_cntr[3]_i_1 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(delay_cntr[2]),
        .I3(delay_cntr[1]),
        .I4(delay_cntr[0]),
        .I5(delay_cntr[3]),
        .O(\delay_cntr[3]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h000000007FFF8000)) 
    \delay_cntr[4]_i_1 
       (.I0(delay_cntr[1]),
        .I1(delay_cntr[0]),
        .I2(delay_cntr[3]),
        .I3(delay_cntr[2]),
        .I4(delay_cntr[4]),
        .I5(\delay_cntr[4]_i_2_n_0 ),
        .O(p_0_in[4]));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \delay_cntr[4]_i_2 
       (.I0(reset_prng),
        .I1(reset_n),
        .O(\delay_cntr[4]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'hEA)) 
    \delay_cntr[5]_i_1 
       (.I0(preamble_en),
        .I1(reset_n),
        .I2(reset_prng),
        .O(\delay_cntr[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h07777000)) 
    \delay_cntr[5]_i_2 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(delay_cntr[4]),
        .I3(\delay_cntr[5]_i_3_n_0 ),
        .I4(delay_cntr[5]),
        .O(\delay_cntr[5]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h8000)) 
    \delay_cntr[5]_i_3 
       (.I0(delay_cntr[1]),
        .I1(delay_cntr[0]),
        .I2(delay_cntr[3]),
        .I3(delay_cntr[2]),
        .O(\delay_cntr[5]_i_3_n_0 ));
  FDCE \delay_cntr_reg[0] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .CLR(reset_n_0),
        .D(p_0_in[0]),
        .Q(delay_cntr[0]));
  FDCE \delay_cntr_reg[1] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .CLR(reset_n_0),
        .D(p_0_in[1]),
        .Q(delay_cntr[1]));
  FDCE \delay_cntr_reg[2] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .CLR(reset_n_0),
        .D(p_0_in[2]),
        .Q(delay_cntr[2]));
  FDCE \delay_cntr_reg[3] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .CLR(reset_n_0),
        .D(\delay_cntr[3]_i_1_n_0 ),
        .Q(delay_cntr[3]));
  FDCE \delay_cntr_reg[4] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .CLR(reset_n_0),
        .D(p_0_in[4]),
        .Q(delay_cntr[4]));
  FDCE \delay_cntr_reg[5] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .CLR(reset_n_0),
        .D(\delay_cntr[5]_i_2_n_0 ),
        .Q(delay_cntr[5]));
  LUT3 #(
    .INIT(8'hB8)) 
    pn_bit_i_1
       (.I0(p_0_in_0),
        .I1(preamble_en),
        .I2(pn_bit),
        .O(pn_bit_i_1_n_0));
  FDCE pn_bit_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(reset_n_0),
        .D(pn_bit_i_1_n_0),
        .Q(pn_bit));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'h0090)) 
    \preamble_det_bit_count[0]_i_1 
       (.I0(delay_3_indata),
        .I1(pn_bit),
        .I2(Q[3]),
        .I3(\preamble_det_bit_count_reg[7]_0 [0]),
        .O(\preamble_det_bit_count_reg[7] [0]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT5 #(
    .INIT(32'h00828200)) 
    \preamble_det_bit_count[1]_i_1 
       (.I0(Q[3]),
        .I1(pn_bit),
        .I2(delay_3_indata),
        .I3(\preamble_det_bit_count_reg[7]_0 [0]),
        .I4(\preamble_det_bit_count_reg[7]_0 [1]),
        .O(\preamble_det_bit_count_reg[7] [1]));
  LUT6 #(
    .INIT(64'h0082828282000000)) 
    \preamble_det_bit_count[2]_i_1 
       (.I0(Q[3]),
        .I1(pn_bit),
        .I2(delay_3_indata),
        .I3(\preamble_det_bit_count_reg[7]_0 [1]),
        .I4(\preamble_det_bit_count_reg[7]_0 [0]),
        .I5(\preamble_det_bit_count_reg[7]_0 [2]),
        .O(\preamble_det_bit_count_reg[7] [2]));
  LUT5 #(
    .INIT(32'h7F008000)) 
    \preamble_det_bit_count[3]_i_1 
       (.I0(\preamble_det_bit_count_reg[7]_0 [1]),
        .I1(\preamble_det_bit_count_reg[7]_0 [0]),
        .I2(\preamble_det_bit_count_reg[7]_0 [2]),
        .I3(\FSM_onehot_preamble_Stm[3]_i_3_n_0 ),
        .I4(\preamble_det_bit_count_reg[7]_0 [3]),
        .O(\preamble_det_bit_count_reg[7] [3]));
  LUT6 #(
    .INIT(64'h7FFF000080000000)) 
    \preamble_det_bit_count[4]_i_1 
       (.I0(\preamble_det_bit_count_reg[7]_0 [2]),
        .I1(\preamble_det_bit_count_reg[7]_0 [0]),
        .I2(\preamble_det_bit_count_reg[7]_0 [1]),
        .I3(\preamble_det_bit_count_reg[7]_0 [3]),
        .I4(\FSM_onehot_preamble_Stm[3]_i_3_n_0 ),
        .I5(\preamble_det_bit_count_reg[7]_0 [4]),
        .O(\preamble_det_bit_count_reg[7] [4]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h82004100)) 
    \preamble_det_bit_count[5]_i_1 
       (.I0(\preamble_det_bit_count_reg[5] ),
        .I1(delay_3_indata),
        .I2(pn_bit),
        .I3(Q[3]),
        .I4(\preamble_det_bit_count_reg[7]_0 [5]),
        .O(\preamble_det_bit_count_reg[7] [5]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT5 #(
    .INIT(32'h82004100)) 
    \preamble_det_bit_count[6]_i_1 
       (.I0(\preamble_det_bit_count_reg[7]_1 ),
        .I1(delay_3_indata),
        .I2(pn_bit),
        .I3(Q[3]),
        .I4(\preamble_det_bit_count_reg[7]_0 [6]),
        .O(\preamble_det_bit_count_reg[7] [6]));
  LUT6 #(
    .INIT(64'h90090000C00C0000)) 
    \preamble_det_bit_count[7]_i_2 
       (.I0(\preamble_det_bit_count_reg[7]_1 ),
        .I1(\preamble_det_bit_count_reg[7]_0 [7]),
        .I2(delay_3_indata),
        .I3(pn_bit),
        .I4(Q[3]),
        .I5(\preamble_det_bit_count_reg[7]_0 [6]),
        .O(\preamble_det_bit_count_reg[7] [7]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'h60)) 
    reset_prng_reg_i_1
       (.I0(pn_bit),
        .I1(delay_3_indata),
        .I2(Q[3]),
        .O(pn_bit_reg_0));
  LUT6 #(
    .INIT(64'hFFFFFFFFBEEBEBBE)) 
    \shift_reg[0]_i_1 
       (.I0(\delay_cntr[4]_i_2_n_0 ),
        .I1(\shift_reg_reg_n_0_[0] ),
        .I2(p_0_in_0),
        .I3(\shift_reg_reg_n_0_[6] ),
        .I4(\shift_reg_reg_n_0_[1] ),
        .I5(\shift_reg[7]_i_2_n_0 ),
        .O(\shift_reg[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT4 #(
    .INIT(16'hFFF8)) 
    \shift_reg[1]_i_1 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(\shift_reg[7]_i_2_n_0 ),
        .I3(\shift_reg_reg_n_0_[0] ),
        .O(\shift_reg[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT4 #(
    .INIT(16'hFFF8)) 
    \shift_reg[2]_i_1 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(\shift_reg[7]_i_2_n_0 ),
        .I3(\shift_reg_reg_n_0_[1] ),
        .O(\shift_reg[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT4 #(
    .INIT(16'hFFF8)) 
    \shift_reg[3]_i_1 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(\shift_reg[7]_i_2_n_0 ),
        .I3(\shift_reg_reg_n_0_[2] ),
        .O(\shift_reg[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT4 #(
    .INIT(16'hFFF8)) 
    \shift_reg[4]_i_1 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(\shift_reg[7]_i_2_n_0 ),
        .I3(\shift_reg_reg_n_0_[3] ),
        .O(\shift_reg[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT4 #(
    .INIT(16'hFFF8)) 
    \shift_reg[5]_i_1 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(\shift_reg[7]_i_2_n_0 ),
        .I3(\shift_reg_reg_n_0_[4] ),
        .O(\shift_reg[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT4 #(
    .INIT(16'hFFF8)) 
    \shift_reg[6]_i_1 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(\shift_reg[7]_i_2_n_0 ),
        .I3(\shift_reg_reg_n_0_[5] ),
        .O(\shift_reg[6]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hFFF8)) 
    \shift_reg[7]_i_1 
       (.I0(reset_n),
        .I1(reset_prng),
        .I2(\shift_reg[7]_i_2_n_0 ),
        .I3(\shift_reg_reg_n_0_[6] ),
        .O(\shift_reg[7]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    \shift_reg[7]_i_2 
       (.I0(delay_cntr[2]),
        .I1(delay_cntr[3]),
        .I2(delay_cntr[0]),
        .I3(delay_cntr[1]),
        .I4(delay_cntr[5]),
        .I5(delay_cntr[4]),
        .O(\shift_reg[7]_i_2_n_0 ));
  FDPE \shift_reg_reg[0] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .D(\shift_reg[0]_i_1_n_0 ),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[0] ));
  FDPE \shift_reg_reg[1] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .D(\shift_reg[1]_i_1_n_0 ),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[1] ));
  FDPE \shift_reg_reg[2] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .D(\shift_reg[2]_i_1_n_0 ),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[2] ));
  FDPE \shift_reg_reg[3] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .D(\shift_reg[3]_i_1_n_0 ),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[3] ));
  FDPE \shift_reg_reg[4] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .D(\shift_reg[4]_i_1_n_0 ),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[4] ));
  FDPE \shift_reg_reg[5] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .D(\shift_reg[5]_i_1_n_0 ),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[5] ));
  FDPE \shift_reg_reg[6] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .D(\shift_reg[6]_i_1_n_0 ),
        .PRE(reset_n_0),
        .Q(\shift_reg_reg_n_0_[6] ));
  FDPE \shift_reg_reg[7] 
       (.C(clock),
        .CE(\delay_cntr[5]_i_1_n_0 ),
        .D(\shift_reg[7]_i_1_n_0 ),
        .PRE(reset_n_0),
        .Q(p_0_in_0));
endmodule

(* CHECK_LICENSE_TYPE = "design_2_preamble_detector_fsk_0_1,preamble_detector_fsk,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "preamble_detector_fsk,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (clock,
    reset_n,
    bsync_dat_in,
    bsync_val_in,
    data_det_out,
    data_det_val_out,
    data_det_start,
    data_det_end);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clock CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clock, FREQ_HZ 61440000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, INSERT_VIP 0" *) input clock;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 reset_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input reset_n;
  input [7:0]bsync_dat_in;
  input bsync_val_in;
  output data_det_out;
  output data_det_val_out;
  output data_det_start;
  output data_det_end;

  wire [7:0]bsync_dat_in;
  wire bsync_val_in;
  wire clock;
  wire data_det_end;
  wire data_det_out;
  wire data_det_start;
  wire data_det_val_out;
  wire reset_n;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_preamble_detector_fsk inst
       (.bsync_dat_in(bsync_dat_in[0]),
        .bsync_val_in(bsync_val_in),
        .clock(clock),
        .data_det_end(data_det_end),
        .data_det_out(data_det_out),
        .data_det_start(data_det_start),
        .data_det_val_out(data_det_val_out),
        .reset_n(reset_n));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_preamble_detector_fsk
   (data_det_out,
    data_det_val_out,
    data_det_start,
    data_det_end,
    reset_n,
    clock,
    bsync_dat_in,
    bsync_val_in);
  output data_det_out;
  output data_det_val_out;
  output data_det_start;
  output data_det_end;
  input reset_n;
  input clock;
  input [0:0]bsync_dat_in;
  input bsync_val_in;

  wire \FSM_onehot_preamble_Stm[0]_i_2_n_0 ;
  wire \FSM_onehot_preamble_Stm[1]_i_1_n_0 ;
  wire \FSM_onehot_preamble_Stm[2]_i_1_n_0 ;
  wire \FSM_onehot_preamble_Stm[2]_i_2_n_0 ;
  wire \FSM_onehot_preamble_Stm[2]_i_3_n_0 ;
  wire \FSM_onehot_preamble_Stm[2]_i_4_n_0 ;
  wire \FSM_onehot_preamble_Stm[2]_i_5_n_0 ;
  wire \FSM_onehot_preamble_Stm[3]_i_2_n_0 ;
  wire \FSM_onehot_preamble_Stm[6]_i_1_n_0 ;
  wire \FSM_onehot_preamble_Stm_reg_n_0_[0] ;
  wire \FSM_onehot_preamble_Stm_reg_n_0_[1] ;
  wire \FSM_onehot_preamble_Stm_reg_n_0_[2] ;
  wire \FSM_onehot_preamble_Stm_reg_n_0_[4] ;
  wire \FSM_onehot_preamble_Stm_reg_n_0_[5] ;
  wire [0:0]bsync_dat_in;
  wire bsync_val_in;
  wire clock;
  wire [15:0]data_bit_count;
  wire data_bit_count0_carry__0_n_0;
  wire data_bit_count0_carry__0_n_1;
  wire data_bit_count0_carry__0_n_2;
  wire data_bit_count0_carry__0_n_3;
  wire data_bit_count0_carry__1_n_0;
  wire data_bit_count0_carry__1_n_1;
  wire data_bit_count0_carry__1_n_2;
  wire data_bit_count0_carry__1_n_3;
  wire data_bit_count0_carry__2_n_2;
  wire data_bit_count0_carry__2_n_3;
  wire data_bit_count0_carry_n_0;
  wire data_bit_count0_carry_n_1;
  wire data_bit_count0_carry_n_2;
  wire data_bit_count0_carry_n_3;
  wire \data_bit_count[0]_i_1_n_0 ;
  wire \data_bit_count[10]_i_1_n_0 ;
  wire \data_bit_count[11]_i_1_n_0 ;
  wire \data_bit_count[12]_i_1_n_0 ;
  wire \data_bit_count[13]_i_1_n_0 ;
  wire \data_bit_count[14]_i_1_n_0 ;
  wire \data_bit_count[15]_i_2_n_0 ;
  wire \data_bit_count[1]_i_1_n_0 ;
  wire \data_bit_count[2]_i_1_n_0 ;
  wire \data_bit_count[3]_i_1_n_0 ;
  wire \data_bit_count[4]_i_1_n_0 ;
  wire \data_bit_count[5]_i_1_n_0 ;
  wire \data_bit_count[6]_i_1_n_0 ;
  wire \data_bit_count[7]_i_1_n_0 ;
  wire \data_bit_count[8]_i_1_n_0 ;
  wire \data_bit_count[9]_i_1_n_0 ;
  wire data_bit_count_1;
  wire data_det_end;
  wire data_det_out;
  wire data_det_start;
  wire data_det_val_out;
  wire delay_2_indata;
  wire delay_3_indata;
  wire en_preamble;
  wire end_flag;
  wire [15:1]in7;
  wire p_0_in0_in;
  wire p_0_in_0;
  wire preamble_det_bit_count;
  wire \preamble_det_bit_count[5]_i_2_n_0 ;
  wire \preamble_det_bit_count_reg_n_0_[0] ;
  wire \preamble_det_bit_count_reg_n_0_[1] ;
  wire \preamble_det_bit_count_reg_n_0_[2] ;
  wire \preamble_det_bit_count_reg_n_0_[3] ;
  wire \preamble_det_bit_count_reg_n_0_[4] ;
  wire \preamble_det_bit_count_reg_n_0_[5] ;
  wire \preamble_det_bit_count_reg_n_0_[6] ;
  wire \preamble_det_bit_count_reg_n_0_[7] ;
  wire preamble_en;
  wire preamble_en_i_1_n_0;
  wire preamble_mod_n_0;
  wire preamble_mod_n_1;
  wire preamble_mod_n_10;
  wire preamble_mod_n_11;
  wire preamble_mod_n_2;
  wire preamble_mod_n_3;
  wire preamble_mod_n_4;
  wire preamble_mod_n_5;
  wire preamble_mod_n_6;
  wire preamble_mod_n_7;
  wire preamble_mod_n_8;
  wire preamble_mod_n_9;
  wire reset_n;
  wire reset_prng;
  wire start_flag;
  wire [3:2]NLW_data_bit_count0_carry__2_CO_UNCONNECTED;
  wire [3:3]NLW_data_bit_count0_carry__2_O_UNCONNECTED;

  LUT3 #(
    .INIT(8'hFE)) 
    \FSM_onehot_preamble_Stm[0]_i_2 
       (.I0(\preamble_det_bit_count_reg_n_0_[7] ),
        .I1(\FSM_onehot_preamble_Stm[3]_i_2_n_0 ),
        .I2(\preamble_det_bit_count_reg_n_0_[6] ),
        .O(\FSM_onehot_preamble_Stm[0]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT5 #(
    .INIT(32'h10FF1010)) 
    \FSM_onehot_preamble_Stm[1]_i_1 
       (.I0(\FSM_onehot_preamble_Stm[2]_i_2_n_0 ),
        .I1(\FSM_onehot_preamble_Stm[2]_i_3_n_0 ),
        .I2(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I3(bsync_val_in),
        .I4(\FSM_onehot_preamble_Stm_reg_n_0_[1] ),
        .O(\FSM_onehot_preamble_Stm[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT5 #(
    .INIT(32'hFFA8A8A8)) 
    \FSM_onehot_preamble_Stm[2]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(\FSM_onehot_preamble_Stm[2]_i_2_n_0 ),
        .I2(\FSM_onehot_preamble_Stm[2]_i_3_n_0 ),
        .I3(bsync_val_in),
        .I4(p_0_in0_in),
        .O(\FSM_onehot_preamble_Stm[2]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hEFFFFFFF)) 
    \FSM_onehot_preamble_Stm[2]_i_2 
       (.I0(\FSM_onehot_preamble_Stm[2]_i_4_n_0 ),
        .I1(data_bit_count[6]),
        .I2(data_bit_count[5]),
        .I3(data_bit_count[3]),
        .I4(data_bit_count[4]),
        .O(\FSM_onehot_preamble_Stm[2]_i_2_n_0 ));
  LUT5 #(
    .INIT(32'hFFFF7FFF)) 
    \FSM_onehot_preamble_Stm[2]_i_3 
       (.I0(data_bit_count[9]),
        .I1(data_bit_count[10]),
        .I2(data_bit_count[7]),
        .I3(data_bit_count[8]),
        .I4(\FSM_onehot_preamble_Stm[2]_i_5_n_0 ),
        .O(\FSM_onehot_preamble_Stm[2]_i_3_n_0 ));
  LUT5 #(
    .INIT(32'hEFFFFFFF)) 
    \FSM_onehot_preamble_Stm[2]_i_4 
       (.I0(data_bit_count[0]),
        .I1(data_bit_count[15]),
        .I2(bsync_val_in),
        .I3(data_bit_count[2]),
        .I4(data_bit_count[1]),
        .O(\FSM_onehot_preamble_Stm[2]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \FSM_onehot_preamble_Stm[2]_i_5 
       (.I0(data_bit_count[12]),
        .I1(data_bit_count[11]),
        .I2(data_bit_count[14]),
        .I3(data_bit_count[13]),
        .O(\FSM_onehot_preamble_Stm[2]_i_5_n_0 ));
  LUT6 #(
    .INIT(64'h7FFFFFFFFFFFFFFF)) 
    \FSM_onehot_preamble_Stm[3]_i_2 
       (.I0(\preamble_det_bit_count_reg_n_0_[4] ),
        .I1(\preamble_det_bit_count_reg_n_0_[2] ),
        .I2(\preamble_det_bit_count_reg_n_0_[0] ),
        .I3(\preamble_det_bit_count_reg_n_0_[1] ),
        .I4(\preamble_det_bit_count_reg_n_0_[3] ),
        .I5(\preamble_det_bit_count_reg_n_0_[5] ),
        .O(\FSM_onehot_preamble_Stm[3]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \FSM_onehot_preamble_Stm[6]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[0] ),
        .I1(bsync_val_in),
        .O(\FSM_onehot_preamble_Stm[6]_i_1_n_0 ));
  (* FSM_ENCODED_STATES = "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001" *) 
  FDPE #(
    .INIT(1'b1)) 
    \FSM_onehot_preamble_Stm_reg[0] 
       (.C(clock),
        .CE(1'b1),
        .D(preamble_mod_n_2),
        .PRE(preamble_mod_n_0),
        .Q(\FSM_onehot_preamble_Stm_reg_n_0_[0] ));
  (* FSM_ENCODED_STATES = "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_preamble_Stm_reg[1] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\FSM_onehot_preamble_Stm[1]_i_1_n_0 ),
        .Q(\FSM_onehot_preamble_Stm_reg_n_0_[1] ));
  (* FSM_ENCODED_STATES = "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_preamble_Stm_reg[2] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\FSM_onehot_preamble_Stm[2]_i_1_n_0 ),
        .Q(\FSM_onehot_preamble_Stm_reg_n_0_[2] ));
  (* FSM_ENCODED_STATES = "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_preamble_Stm_reg[3] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_1),
        .Q(p_0_in0_in));
  (* FSM_ENCODED_STATES = "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_preamble_Stm_reg[4] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\FSM_onehot_preamble_Stm_reg_n_0_[5] ),
        .Q(\FSM_onehot_preamble_Stm_reg_n_0_[4] ));
  (* FSM_ENCODED_STATES = "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_preamble_Stm_reg[5] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(p_0_in_0),
        .Q(\FSM_onehot_preamble_Stm_reg_n_0_[5] ));
  (* FSM_ENCODED_STATES = "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_preamble_Stm_reg[6] 
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(\FSM_onehot_preamble_Stm[6]_i_1_n_0 ),
        .Q(p_0_in_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 data_bit_count0_carry
       (.CI(1'b0),
        .CO({data_bit_count0_carry_n_0,data_bit_count0_carry_n_1,data_bit_count0_carry_n_2,data_bit_count0_carry_n_3}),
        .CYINIT(data_bit_count[0]),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(in7[4:1]),
        .S(data_bit_count[4:1]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 data_bit_count0_carry__0
       (.CI(data_bit_count0_carry_n_0),
        .CO({data_bit_count0_carry__0_n_0,data_bit_count0_carry__0_n_1,data_bit_count0_carry__0_n_2,data_bit_count0_carry__0_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(in7[8:5]),
        .S(data_bit_count[8:5]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 data_bit_count0_carry__1
       (.CI(data_bit_count0_carry__0_n_0),
        .CO({data_bit_count0_carry__1_n_0,data_bit_count0_carry__1_n_1,data_bit_count0_carry__1_n_2,data_bit_count0_carry__1_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(in7[12:9]),
        .S(data_bit_count[12:9]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 data_bit_count0_carry__2
       (.CI(data_bit_count0_carry__1_n_0),
        .CO({NLW_data_bit_count0_carry__2_CO_UNCONNECTED[3:2],data_bit_count0_carry__2_n_2,data_bit_count0_carry__2_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({NLW_data_bit_count0_carry__2_O_UNCONNECTED[3],in7[15:13]}),
        .S({1'b0,data_bit_count[15:13]}));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hF4)) 
    \data_bit_count[0]_i_1 
       (.I0(data_bit_count[0]),
        .I1(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I2(p_0_in0_in),
        .O(\data_bit_count[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[10]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[10]),
        .O(\data_bit_count[10]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[11]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[11]),
        .O(\data_bit_count[11]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[12]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[12]),
        .O(\data_bit_count[12]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[13]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[13]),
        .O(\data_bit_count[13]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[14]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[14]),
        .O(\data_bit_count[14]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hFEEE)) 
    \data_bit_count[15]_i_1 
       (.I0(p_0_in0_in),
        .I1(\FSM_onehot_preamble_Stm_reg_n_0_[1] ),
        .I2(bsync_val_in),
        .I3(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .O(data_bit_count_1));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[15]_i_2 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[15]),
        .O(\data_bit_count[15]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[1]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[1]),
        .O(\data_bit_count[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[2]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[2]),
        .O(\data_bit_count[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[3]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[3]),
        .O(\data_bit_count[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[4]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[4]),
        .O(\data_bit_count[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[5]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[5]),
        .O(\data_bit_count[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[6]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[6]),
        .O(\data_bit_count[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[7]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[7]),
        .O(\data_bit_count[7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[8]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[8]),
        .O(\data_bit_count[8]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \data_bit_count[9]_i_1 
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[2] ),
        .I1(in7[9]),
        .O(\data_bit_count[9]_i_1_n_0 ));
  FDCE \data_bit_count_reg[0] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[0]_i_1_n_0 ),
        .Q(data_bit_count[0]));
  FDCE \data_bit_count_reg[10] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[10]_i_1_n_0 ),
        .Q(data_bit_count[10]));
  FDCE \data_bit_count_reg[11] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[11]_i_1_n_0 ),
        .Q(data_bit_count[11]));
  FDCE \data_bit_count_reg[12] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[12]_i_1_n_0 ),
        .Q(data_bit_count[12]));
  FDCE \data_bit_count_reg[13] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[13]_i_1_n_0 ),
        .Q(data_bit_count[13]));
  FDCE \data_bit_count_reg[14] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[14]_i_1_n_0 ),
        .Q(data_bit_count[14]));
  FDCE \data_bit_count_reg[15] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[15]_i_2_n_0 ),
        .Q(data_bit_count[15]));
  FDCE \data_bit_count_reg[1] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[1]_i_1_n_0 ),
        .Q(data_bit_count[1]));
  FDCE \data_bit_count_reg[2] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[2]_i_1_n_0 ),
        .Q(data_bit_count[2]));
  FDCE \data_bit_count_reg[3] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[3]_i_1_n_0 ),
        .Q(data_bit_count[3]));
  FDCE \data_bit_count_reg[4] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[4]_i_1_n_0 ),
        .Q(data_bit_count[4]));
  FDCE \data_bit_count_reg[5] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[5]_i_1_n_0 ),
        .Q(data_bit_count[5]));
  FDCE \data_bit_count_reg[6] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[6]_i_1_n_0 ),
        .Q(data_bit_count[6]));
  FDCE \data_bit_count_reg[7] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[7]_i_1_n_0 ),
        .Q(data_bit_count[7]));
  FDCE \data_bit_count_reg[8] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[8]_i_1_n_0 ),
        .Q(data_bit_count[8]));
  FDCE \data_bit_count_reg[9] 
       (.C(clock),
        .CE(data_bit_count_1),
        .CLR(preamble_mod_n_0),
        .D(\data_bit_count[9]_i_1_n_0 ),
        .Q(data_bit_count[9]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT2 #(
    .INIT(4'h8)) 
    data_det_end_i_1
       (.I0(bsync_val_in),
        .I1(\FSM_onehot_preamble_Stm_reg_n_0_[1] ),
        .O(end_flag));
  FDCE data_det_end_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(end_flag),
        .Q(data_det_end));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT2 #(
    .INIT(4'h8)) 
    data_det_start_i_1
       (.I0(bsync_val_in),
        .I1(p_0_in0_in),
        .O(start_flag));
  FDCE data_det_start_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(start_flag),
        .Q(data_det_start));
  FDCE data_det_val_out_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(bsync_val_in),
        .Q(data_det_val_out));
  FDCE delay_1_indata_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(bsync_dat_in),
        .Q(data_det_out));
  FDCE delay_2_indata_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(data_det_out),
        .Q(delay_2_indata));
  FDCE delay_3_indata_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(delay_2_indata),
        .Q(delay_3_indata));
  LUT5 #(
    .INIT(32'h7FFFFFFF)) 
    \preamble_det_bit_count[5]_i_2 
       (.I0(\preamble_det_bit_count_reg_n_0_[3] ),
        .I1(\preamble_det_bit_count_reg_n_0_[1] ),
        .I2(\preamble_det_bit_count_reg_n_0_[0] ),
        .I3(\preamble_det_bit_count_reg_n_0_[2] ),
        .I4(\preamble_det_bit_count_reg_n_0_[4] ),
        .O(\preamble_det_bit_count[5]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \preamble_det_bit_count[7]_i_1 
       (.I0(p_0_in0_in),
        .I1(\FSM_onehot_preamble_Stm_reg_n_0_[4] ),
        .O(preamble_det_bit_count));
  FDCE \preamble_det_bit_count_reg[0] 
       (.C(clock),
        .CE(preamble_det_bit_count),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_11),
        .Q(\preamble_det_bit_count_reg_n_0_[0] ));
  FDCE \preamble_det_bit_count_reg[1] 
       (.C(clock),
        .CE(preamble_det_bit_count),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_10),
        .Q(\preamble_det_bit_count_reg_n_0_[1] ));
  FDCE \preamble_det_bit_count_reg[2] 
       (.C(clock),
        .CE(preamble_det_bit_count),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_9),
        .Q(\preamble_det_bit_count_reg_n_0_[2] ));
  FDCE \preamble_det_bit_count_reg[3] 
       (.C(clock),
        .CE(preamble_det_bit_count),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_8),
        .Q(\preamble_det_bit_count_reg_n_0_[3] ));
  FDCE \preamble_det_bit_count_reg[4] 
       (.C(clock),
        .CE(preamble_det_bit_count),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_7),
        .Q(\preamble_det_bit_count_reg_n_0_[4] ));
  FDCE \preamble_det_bit_count_reg[5] 
       (.C(clock),
        .CE(preamble_det_bit_count),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_6),
        .Q(\preamble_det_bit_count_reg_n_0_[5] ));
  FDCE \preamble_det_bit_count_reg[6] 
       (.C(clock),
        .CE(preamble_det_bit_count),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_5),
        .Q(\preamble_det_bit_count_reg_n_0_[6] ));
  FDCE \preamble_det_bit_count_reg[7] 
       (.C(clock),
        .CE(preamble_det_bit_count),
        .CLR(preamble_mod_n_0),
        .D(preamble_mod_n_4),
        .Q(\preamble_det_bit_count_reg_n_0_[7] ));
  LUT6 #(
    .INIT(64'hAAAAAAABAAAAAAA8)) 
    preamble_en_i_1
       (.I0(bsync_val_in),
        .I1(\FSM_onehot_preamble_Stm_reg_n_0_[4] ),
        .I2(\FSM_onehot_preamble_Stm_reg_n_0_[0] ),
        .I3(p_0_in_0),
        .I4(\FSM_onehot_preamble_Stm_reg_n_0_[5] ),
        .I5(preamble_en),
        .O(preamble_en_i_1_n_0));
  FDCE preamble_en_reg
       (.C(clock),
        .CE(1'b1),
        .CLR(preamble_mod_n_0),
        .D(preamble_en_i_1_n_0),
        .Q(preamble_en));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr preamble_mod
       (.D({preamble_mod_n_1,preamble_mod_n_2}),
        .\FSM_onehot_preamble_Stm_reg[0] (\FSM_onehot_preamble_Stm[0]_i_2_n_0 ),
        .Q({\FSM_onehot_preamble_Stm_reg_n_0_[4] ,p_0_in0_in,\FSM_onehot_preamble_Stm_reg_n_0_[1] ,\FSM_onehot_preamble_Stm_reg_n_0_[0] }),
        .bsync_val_in(bsync_val_in),
        .clock(clock),
        .delay_3_indata(delay_3_indata),
        .pn_bit_reg_0(preamble_mod_n_3),
        .\preamble_det_bit_count_reg[5] (\preamble_det_bit_count[5]_i_2_n_0 ),
        .\preamble_det_bit_count_reg[7] ({preamble_mod_n_4,preamble_mod_n_5,preamble_mod_n_6,preamble_mod_n_7,preamble_mod_n_8,preamble_mod_n_9,preamble_mod_n_10,preamble_mod_n_11}),
        .\preamble_det_bit_count_reg[7]_0 ({\preamble_det_bit_count_reg_n_0_[7] ,\preamble_det_bit_count_reg_n_0_[6] ,\preamble_det_bit_count_reg_n_0_[5] ,\preamble_det_bit_count_reg_n_0_[4] ,\preamble_det_bit_count_reg_n_0_[3] ,\preamble_det_bit_count_reg_n_0_[2] ,\preamble_det_bit_count_reg_n_0_[1] ,\preamble_det_bit_count_reg_n_0_[0] }),
        .\preamble_det_bit_count_reg[7]_1 (\FSM_onehot_preamble_Stm[3]_i_2_n_0 ),
        .preamble_en(preamble_en),
        .reset_n(reset_n),
        .reset_n_0(preamble_mod_n_0),
        .reset_prng(reset_prng));
  (* XILINX_LEGACY_PRIM = "LD" *) 
  (* XILINX_TRANSFORM_PINMAP = "VCC:GE GND:CLR" *) 
  LDCE #(
    .INIT(1'b0)) 
    reset_prng_reg
       (.CLR(1'b0),
        .D(preamble_mod_n_3),
        .G(en_preamble),
        .GE(1'b1),
        .Q(reset_prng));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT4 #(
    .INIT(16'hFFFE)) 
    reset_prng_reg_i_2
       (.I0(\FSM_onehot_preamble_Stm_reg_n_0_[4] ),
        .I1(\FSM_onehot_preamble_Stm_reg_n_0_[0] ),
        .I2(p_0_in_0),
        .I3(\FSM_onehot_preamble_Stm_reg_n_0_[5] ),
        .O(en_preamble));
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
