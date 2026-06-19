// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Wed Jan  7 13:01:36 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_2_fsk_lpf_0_0_sim_netlist.v
// Design      : design_2_fsk_lpf_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "design_2_fsk_lpf_0_0,fsk_lpf,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "HLS" *) 
(* X_CORE_INFO = "fsk_lpf,Vivado 2023.1" *) (* hls_module = "yes" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (ap_clk,
    ap_rst_n,
    in_r_TVALID,
    in_r_TREADY,
    in_r_TDATA,
    in_r_TLAST,
    in_r_TKEEP,
    in_r_TSTRB,
    out_r_TVALID,
    out_r_TREADY,
    out_r_TDATA,
    out_r_TLAST,
    out_r_TKEEP,
    out_r_TSTRB);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ap_clk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ap_clk, ASSOCIATED_BUSIF in_r:out_r, ASSOCIATED_RESET ap_rst_n, FREQ_HZ 61440000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, INSERT_VIP 0" *) input ap_clk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 ap_rst_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ap_rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input ap_rst_n;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_r TVALID" *) input in_r_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_r TREADY" *) output in_r_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_r TDATA" *) input [15:0]in_r_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_r TLAST" *) input [0:0]in_r_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_r TKEEP" *) input [1:0]in_r_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_r TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME in_r, TDATA_NUM_BYTES 2, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 61440000, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, INSERT_VIP 0" *) input [1:0]in_r_TSTRB;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_r TVALID" *) output out_r_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_r TREADY" *) input out_r_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_r TDATA" *) output [15:0]out_r_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_r TLAST" *) output [0:0]out_r_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_r TKEEP" *) output [1:0]out_r_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_r TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME out_r, TDATA_NUM_BYTES 2, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 61440000, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, INSERT_VIP 0" *) output [1:0]out_r_TSTRB;

  wire \<const1> ;
  wire ap_clk;
  wire ap_rst_n;
  wire [15:0]in_r_TDATA;
  wire [0:0]in_r_TLAST;
  wire in_r_TREADY;
  wire in_r_TVALID;
  wire [15:0]out_r_TDATA;
  wire [0:0]out_r_TLAST;
  wire out_r_TREADY;
  wire out_r_TVALID;
  wire [1:0]NLW_inst_out_r_TKEEP_UNCONNECTED;
  wire [1:0]NLW_inst_out_r_TSTRB_UNCONNECTED;

  assign out_r_TKEEP[1] = \<const1> ;
  assign out_r_TKEEP[0] = \<const1> ;
  assign out_r_TSTRB[1] = \<const1> ;
  assign out_r_TSTRB[0] = \<const1> ;
  VCC VCC
       (.P(\<const1> ));
  (* SDX_KERNEL = "true" *) 
  (* SDX_KERNEL_SYNTH_INST = "inst" *) 
  (* SDX_KERNEL_TYPE = "hls" *) 
  (* ap_ST_fsm_pp0_stage0 = "1'b1" *) 
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf inst
       (.ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .in_r_TDATA(in_r_TDATA),
        .in_r_TKEEP({1'b0,1'b0}),
        .in_r_TLAST(in_r_TLAST),
        .in_r_TREADY(in_r_TREADY),
        .in_r_TSTRB({1'b0,1'b0}),
        .in_r_TVALID(in_r_TVALID),
        .out_r_TDATA(out_r_TDATA),
        .out_r_TKEEP(NLW_inst_out_r_TKEEP_UNCONNECTED[1:0]),
        .out_r_TLAST(out_r_TLAST),
        .out_r_TREADY(out_r_TREADY),
        .out_r_TSTRB(NLW_inst_out_r_TSTRB_UNCONNECTED[1:0]),
        .out_r_TVALID(out_r_TVALID));
endmodule

(* ap_ST_fsm_pp0_stage0 = "1'b1" *) (* hls_module = "yes" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf
   (ap_clk,
    ap_rst_n,
    in_r_TDATA,
    in_r_TVALID,
    in_r_TREADY,
    in_r_TKEEP,
    in_r_TSTRB,
    in_r_TLAST,
    out_r_TDATA,
    out_r_TVALID,
    out_r_TREADY,
    out_r_TKEEP,
    out_r_TSTRB,
    out_r_TLAST);
  input ap_clk;
  input ap_rst_n;
  input [15:0]in_r_TDATA;
  input in_r_TVALID;
  output in_r_TREADY;
  input [1:0]in_r_TKEEP;
  input [1:0]in_r_TSTRB;
  input [0:0]in_r_TLAST;
  output [15:0]out_r_TDATA;
  output out_r_TVALID;
  input out_r_TREADY;
  output [1:0]out_r_TKEEP;
  output [1:0]out_r_TSTRB;
  output [0:0]out_r_TLAST;

  wire \<const0> ;
  wire [31:16]acc_reg_1050;
  wire \acc_reg_1050[19]_i_10_n_0 ;
  wire \acc_reg_1050[19]_i_12_n_0 ;
  wire \acc_reg_1050[19]_i_13_n_0 ;
  wire \acc_reg_1050[19]_i_14_n_0 ;
  wire \acc_reg_1050[19]_i_15_n_0 ;
  wire \acc_reg_1050[19]_i_16_n_0 ;
  wire \acc_reg_1050[19]_i_17_n_0 ;
  wire \acc_reg_1050[19]_i_18_n_0 ;
  wire \acc_reg_1050[19]_i_19_n_0 ;
  wire \acc_reg_1050[19]_i_21_n_0 ;
  wire \acc_reg_1050[19]_i_22_n_0 ;
  wire \acc_reg_1050[19]_i_23_n_0 ;
  wire \acc_reg_1050[19]_i_24_n_0 ;
  wire \acc_reg_1050[19]_i_25_n_0 ;
  wire \acc_reg_1050[19]_i_26_n_0 ;
  wire \acc_reg_1050[19]_i_27_n_0 ;
  wire \acc_reg_1050[19]_i_28_n_0 ;
  wire \acc_reg_1050[19]_i_30_n_0 ;
  wire \acc_reg_1050[19]_i_31_n_0 ;
  wire \acc_reg_1050[19]_i_32_n_0 ;
  wire \acc_reg_1050[19]_i_33_n_0 ;
  wire \acc_reg_1050[19]_i_34_n_0 ;
  wire \acc_reg_1050[19]_i_35_n_0 ;
  wire \acc_reg_1050[19]_i_36_n_0 ;
  wire \acc_reg_1050[19]_i_37_n_0 ;
  wire \acc_reg_1050[19]_i_38_n_0 ;
  wire \acc_reg_1050[19]_i_39_n_0 ;
  wire \acc_reg_1050[19]_i_3_n_0 ;
  wire \acc_reg_1050[19]_i_40_n_0 ;
  wire \acc_reg_1050[19]_i_41_n_0 ;
  wire \acc_reg_1050[19]_i_42_n_0 ;
  wire \acc_reg_1050[19]_i_43_n_0 ;
  wire \acc_reg_1050[19]_i_44_n_0 ;
  wire \acc_reg_1050[19]_i_4_n_0 ;
  wire \acc_reg_1050[19]_i_5_n_0 ;
  wire \acc_reg_1050[19]_i_6_n_0 ;
  wire \acc_reg_1050[19]_i_7_n_0 ;
  wire \acc_reg_1050[19]_i_8_n_0 ;
  wire \acc_reg_1050[19]_i_9_n_0 ;
  wire \acc_reg_1050[23]_i_2_n_0 ;
  wire \acc_reg_1050[23]_i_3_n_0 ;
  wire \acc_reg_1050[23]_i_4_n_0 ;
  wire \acc_reg_1050[23]_i_5_n_0 ;
  wire \acc_reg_1050[23]_i_6_n_0 ;
  wire \acc_reg_1050[23]_i_7_n_0 ;
  wire \acc_reg_1050[23]_i_8_n_0 ;
  wire \acc_reg_1050[23]_i_9_n_0 ;
  wire \acc_reg_1050[27]_i_2_n_0 ;
  wire \acc_reg_1050[27]_i_3_n_0 ;
  wire \acc_reg_1050[27]_i_4_n_0 ;
  wire \acc_reg_1050[27]_i_5_n_0 ;
  wire \acc_reg_1050[27]_i_6_n_0 ;
  wire \acc_reg_1050[27]_i_7_n_0 ;
  wire \acc_reg_1050[27]_i_8_n_0 ;
  wire \acc_reg_1050[27]_i_9_n_0 ;
  wire \acc_reg_1050[31]_i_2_n_0 ;
  wire \acc_reg_1050[31]_i_3_n_0 ;
  wire \acc_reg_1050[31]_i_4_n_0 ;
  wire \acc_reg_1050[31]_i_5_n_0 ;
  wire \acc_reg_1050[31]_i_6_n_0 ;
  wire \acc_reg_1050[31]_i_7_n_0 ;
  wire \acc_reg_1050[31]_i_8_n_0 ;
  wire \acc_reg_1050[31]_i_9_n_0 ;
  wire \acc_reg_1050_reg[19]_i_11_n_0 ;
  wire \acc_reg_1050_reg[19]_i_11_n_1 ;
  wire \acc_reg_1050_reg[19]_i_11_n_2 ;
  wire \acc_reg_1050_reg[19]_i_11_n_3 ;
  wire \acc_reg_1050_reg[19]_i_1_n_0 ;
  wire \acc_reg_1050_reg[19]_i_1_n_1 ;
  wire \acc_reg_1050_reg[19]_i_1_n_2 ;
  wire \acc_reg_1050_reg[19]_i_1_n_3 ;
  wire \acc_reg_1050_reg[19]_i_1_n_4 ;
  wire \acc_reg_1050_reg[19]_i_1_n_5 ;
  wire \acc_reg_1050_reg[19]_i_1_n_6 ;
  wire \acc_reg_1050_reg[19]_i_1_n_7 ;
  wire \acc_reg_1050_reg[19]_i_20_n_0 ;
  wire \acc_reg_1050_reg[19]_i_20_n_1 ;
  wire \acc_reg_1050_reg[19]_i_20_n_2 ;
  wire \acc_reg_1050_reg[19]_i_20_n_3 ;
  wire \acc_reg_1050_reg[19]_i_29_n_0 ;
  wire \acc_reg_1050_reg[19]_i_29_n_1 ;
  wire \acc_reg_1050_reg[19]_i_29_n_2 ;
  wire \acc_reg_1050_reg[19]_i_29_n_3 ;
  wire \acc_reg_1050_reg[19]_i_2_n_0 ;
  wire \acc_reg_1050_reg[19]_i_2_n_1 ;
  wire \acc_reg_1050_reg[19]_i_2_n_2 ;
  wire \acc_reg_1050_reg[19]_i_2_n_3 ;
  wire \acc_reg_1050_reg[23]_i_1_n_0 ;
  wire \acc_reg_1050_reg[23]_i_1_n_1 ;
  wire \acc_reg_1050_reg[23]_i_1_n_2 ;
  wire \acc_reg_1050_reg[23]_i_1_n_3 ;
  wire \acc_reg_1050_reg[23]_i_1_n_4 ;
  wire \acc_reg_1050_reg[23]_i_1_n_5 ;
  wire \acc_reg_1050_reg[23]_i_1_n_6 ;
  wire \acc_reg_1050_reg[23]_i_1_n_7 ;
  wire \acc_reg_1050_reg[27]_i_1_n_0 ;
  wire \acc_reg_1050_reg[27]_i_1_n_1 ;
  wire \acc_reg_1050_reg[27]_i_1_n_2 ;
  wire \acc_reg_1050_reg[27]_i_1_n_3 ;
  wire \acc_reg_1050_reg[27]_i_1_n_4 ;
  wire \acc_reg_1050_reg[27]_i_1_n_5 ;
  wire \acc_reg_1050_reg[27]_i_1_n_6 ;
  wire \acc_reg_1050_reg[27]_i_1_n_7 ;
  wire \acc_reg_1050_reg[31]_i_1_n_0 ;
  wire \acc_reg_1050_reg[31]_i_1_n_1 ;
  wire \acc_reg_1050_reg[31]_i_1_n_2 ;
  wire \acc_reg_1050_reg[31]_i_1_n_3 ;
  wire \acc_reg_1050_reg[31]_i_1_n_5 ;
  wire \acc_reg_1050_reg[31]_i_1_n_6 ;
  wire \acc_reg_1050_reg[31]_i_1_n_7 ;
  wire [28:0]add_ln131_11_fu_652_p2;
  wire [28:0]add_ln131_11_reg_1040;
  wire [29:0]add_ln131_12_fu_672_p2;
  wire [29:0]add_ln131_12_reg_1045;
  wire \add_ln131_12_reg_1045[11]_i_2_n_0 ;
  wire \add_ln131_12_reg_1045[11]_i_3_n_0 ;
  wire \add_ln131_12_reg_1045[11]_i_4_n_0 ;
  wire \add_ln131_12_reg_1045[11]_i_5_n_0 ;
  wire \add_ln131_12_reg_1045[11]_i_6_n_0 ;
  wire \add_ln131_12_reg_1045[11]_i_7_n_0 ;
  wire \add_ln131_12_reg_1045[11]_i_8_n_0 ;
  wire \add_ln131_12_reg_1045[11]_i_9_n_0 ;
  wire \add_ln131_12_reg_1045[15]_i_2_n_0 ;
  wire \add_ln131_12_reg_1045[15]_i_3_n_0 ;
  wire \add_ln131_12_reg_1045[15]_i_4_n_0 ;
  wire \add_ln131_12_reg_1045[15]_i_5_n_0 ;
  wire \add_ln131_12_reg_1045[15]_i_6_n_0 ;
  wire \add_ln131_12_reg_1045[15]_i_7_n_0 ;
  wire \add_ln131_12_reg_1045[15]_i_8_n_0 ;
  wire \add_ln131_12_reg_1045[15]_i_9_n_0 ;
  wire \add_ln131_12_reg_1045[19]_i_2_n_0 ;
  wire \add_ln131_12_reg_1045[19]_i_3_n_0 ;
  wire \add_ln131_12_reg_1045[19]_i_4_n_0 ;
  wire \add_ln131_12_reg_1045[19]_i_5_n_0 ;
  wire \add_ln131_12_reg_1045[19]_i_6_n_0 ;
  wire \add_ln131_12_reg_1045[19]_i_7_n_0 ;
  wire \add_ln131_12_reg_1045[19]_i_8_n_0 ;
  wire \add_ln131_12_reg_1045[19]_i_9_n_0 ;
  wire \add_ln131_12_reg_1045[23]_i_2_n_0 ;
  wire \add_ln131_12_reg_1045[23]_i_3_n_0 ;
  wire \add_ln131_12_reg_1045[23]_i_4_n_0 ;
  wire \add_ln131_12_reg_1045[23]_i_5_n_0 ;
  wire \add_ln131_12_reg_1045[23]_i_6_n_0 ;
  wire \add_ln131_12_reg_1045[23]_i_7_n_0 ;
  wire \add_ln131_12_reg_1045[23]_i_8_n_0 ;
  wire \add_ln131_12_reg_1045[23]_i_9_n_0 ;
  wire \add_ln131_12_reg_1045[27]_i_2_n_0 ;
  wire \add_ln131_12_reg_1045[27]_i_3_n_0 ;
  wire \add_ln131_12_reg_1045[27]_i_4_n_0 ;
  wire \add_ln131_12_reg_1045[27]_i_5_n_0 ;
  wire \add_ln131_12_reg_1045[27]_i_6_n_0 ;
  wire \add_ln131_12_reg_1045[27]_i_7_n_0 ;
  wire \add_ln131_12_reg_1045[27]_i_8_n_0 ;
  wire \add_ln131_12_reg_1045[27]_i_9_n_0 ;
  wire \add_ln131_12_reg_1045[29]_i_2_n_0 ;
  wire \add_ln131_12_reg_1045[29]_i_3_n_0 ;
  wire \add_ln131_12_reg_1045[29]_i_4_n_0 ;
  wire \add_ln131_12_reg_1045[3]_i_2_n_0 ;
  wire \add_ln131_12_reg_1045[3]_i_3_n_0 ;
  wire \add_ln131_12_reg_1045[3]_i_4_n_0 ;
  wire \add_ln131_12_reg_1045[3]_i_5_n_0 ;
  wire \add_ln131_12_reg_1045[3]_i_6_n_0 ;
  wire \add_ln131_12_reg_1045[3]_i_7_n_0 ;
  wire \add_ln131_12_reg_1045[3]_i_8_n_0 ;
  wire \add_ln131_12_reg_1045[7]_i_2_n_0 ;
  wire \add_ln131_12_reg_1045[7]_i_3_n_0 ;
  wire \add_ln131_12_reg_1045[7]_i_4_n_0 ;
  wire \add_ln131_12_reg_1045[7]_i_5_n_0 ;
  wire \add_ln131_12_reg_1045[7]_i_6_n_0 ;
  wire \add_ln131_12_reg_1045[7]_i_7_n_0 ;
  wire \add_ln131_12_reg_1045[7]_i_8_n_0 ;
  wire \add_ln131_12_reg_1045[7]_i_9_n_0 ;
  wire \add_ln131_12_reg_1045_reg[11]_i_1_n_0 ;
  wire \add_ln131_12_reg_1045_reg[11]_i_1_n_1 ;
  wire \add_ln131_12_reg_1045_reg[11]_i_1_n_2 ;
  wire \add_ln131_12_reg_1045_reg[11]_i_1_n_3 ;
  wire \add_ln131_12_reg_1045_reg[15]_i_1_n_0 ;
  wire \add_ln131_12_reg_1045_reg[15]_i_1_n_1 ;
  wire \add_ln131_12_reg_1045_reg[15]_i_1_n_2 ;
  wire \add_ln131_12_reg_1045_reg[15]_i_1_n_3 ;
  wire \add_ln131_12_reg_1045_reg[19]_i_1_n_0 ;
  wire \add_ln131_12_reg_1045_reg[19]_i_1_n_1 ;
  wire \add_ln131_12_reg_1045_reg[19]_i_1_n_2 ;
  wire \add_ln131_12_reg_1045_reg[19]_i_1_n_3 ;
  wire \add_ln131_12_reg_1045_reg[23]_i_1_n_0 ;
  wire \add_ln131_12_reg_1045_reg[23]_i_1_n_1 ;
  wire \add_ln131_12_reg_1045_reg[23]_i_1_n_2 ;
  wire \add_ln131_12_reg_1045_reg[23]_i_1_n_3 ;
  wire \add_ln131_12_reg_1045_reg[27]_i_1_n_0 ;
  wire \add_ln131_12_reg_1045_reg[27]_i_1_n_1 ;
  wire \add_ln131_12_reg_1045_reg[27]_i_1_n_2 ;
  wire \add_ln131_12_reg_1045_reg[27]_i_1_n_3 ;
  wire \add_ln131_12_reg_1045_reg[29]_i_1_n_3 ;
  wire \add_ln131_12_reg_1045_reg[3]_i_1_n_0 ;
  wire \add_ln131_12_reg_1045_reg[3]_i_1_n_1 ;
  wire \add_ln131_12_reg_1045_reg[3]_i_1_n_2 ;
  wire \add_ln131_12_reg_1045_reg[3]_i_1_n_3 ;
  wire \add_ln131_12_reg_1045_reg[7]_i_1_n_0 ;
  wire \add_ln131_12_reg_1045_reg[7]_i_1_n_1 ;
  wire \add_ln131_12_reg_1045_reg[7]_i_1_n_2 ;
  wire \add_ln131_12_reg_1045_reg[7]_i_1_n_3 ;
  wire [32:0]add_ln131_1_reg_1020;
  wire add_ln131_1_reg_10200;
  wire [32:0]add_ln131_1_reg_1020_pp0_iter5_reg;
  wire [32:0]add_ln131_4_fu_644_p2;
  wire [32:0]add_ln131_4_reg_1025;
  wire [32:0]add_ln131_4_reg_1025_pp0_iter5_reg;
  wire [27:0]add_ln131_6_reg_1030;
  wire [28:0]add_ln131_7_reg_1035;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_0;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_1;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_10;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_11;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_12;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_13;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_14;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_15;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_16;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_17;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_18;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_19;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_2;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_20;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_21;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_22;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_23;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_24;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_25;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_26;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_27;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_28;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_29;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_3;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_30;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_31;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_32;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_33;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_34;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_35;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_36;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_37;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_38;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_39;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_4;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_40;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_41;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_42;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_43;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_44;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_45;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_46;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_47;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_5;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_6;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_7;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_8;
  wire am_addmul_16s_16s_11ns_28_4_1_U4_n_9;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_0;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_1;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_10;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_11;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_12;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_13;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_14;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_15;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_16;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_17;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_18;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_19;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_2;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_20;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_21;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_22;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_23;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_24;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_25;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_26;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_27;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_28;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_29;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_3;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_30;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_31;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_32;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_33;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_34;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_35;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_36;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_37;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_38;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_39;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_4;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_40;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_41;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_42;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_43;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_44;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_45;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_46;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_47;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_5;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_6;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_7;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_8;
  wire am_addmul_16s_16s_11s_28_4_1_U7_n_9;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_0;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_1;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_10;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_11;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_12;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_13;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_14;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_15;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_16;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_17;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_18;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_19;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_2;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_20;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_21;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_22;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_23;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_24;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_25;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_26;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_27;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_28;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_29;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_3;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_30;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_31;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_32;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_33;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_34;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_35;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_36;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_37;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_38;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_39;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_4;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_40;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_41;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_42;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_43;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_44;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_45;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_46;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_47;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_5;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_6;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_7;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_8;
  wire am_addmul_16s_16s_12s_29_4_1_U5_n_9;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_0;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_1;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_10;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_11;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_12;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_13;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_14;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_15;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_16;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_17;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_18;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_19;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_2;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_20;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_21;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_22;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_23;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_24;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_25;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_26;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_27;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_28;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_29;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_3;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_30;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_31;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_32;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_33;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_34;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_35;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_36;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_37;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_38;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_39;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_4;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_40;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_41;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_42;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_43;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_44;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_45;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_46;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_47;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_5;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_6;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_7;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_8;
  wire am_addmul_16s_16s_12s_29_4_1_U6_n_9;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_0;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_1;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_10;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_11;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_12;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_13;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_14;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_15;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_16;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_17;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_18;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_19;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_2;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_20;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_21;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_22;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_23;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_24;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_25;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_26;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_27;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_28;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_29;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_3;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_30;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_31;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_32;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_33;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_34;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_35;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_36;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_37;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_38;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_39;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_4;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_40;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_41;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_42;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_43;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_44;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_45;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_46;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_47;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_5;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_6;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_7;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_8;
  wire am_addmul_16s_16s_14ns_31_4_1_U3_n_9;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_0;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_1;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_10;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_11;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_12;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_13;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_14;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_15;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_16;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_17;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_18;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_19;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_2;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_20;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_21;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_22;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_23;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_24;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_25;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_26;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_27;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_28;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_29;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_3;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_30;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_31;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_32;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_33;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_34;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_35;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_36;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_37;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_38;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_39;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_4;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_40;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_41;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_42;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_43;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_44;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_45;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_46;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_47;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_48;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_49;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_5;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_50;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_51;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_52;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_53;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_54;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_55;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_56;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_57;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_58;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_59;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_6;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_60;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_61;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_62;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_63;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_64;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_65;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_66;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_67;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_68;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_69;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_7;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_70;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_71;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_72;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_73;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_74;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_75;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_76;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_77;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_8;
  wire am_addmul_16s_16s_15ns_33_4_1_U2_n_9;
  wire ama_addmuladd_16s_16s_10s_28s_28_4_1_U14_n_0;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_0;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_1;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_10;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_11;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_12;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_13;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_14;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_15;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_16;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_17;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_18;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_19;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_2;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_20;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_21;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_22;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_23;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_24;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_25;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_26;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_27;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_28;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_3;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_4;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_5;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_6;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_7;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_8;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_9;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_0;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_1;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_10;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_11;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_12;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_13;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_14;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_15;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_16;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_17;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_18;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_19;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_2;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_20;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_21;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_22;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_23;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_24;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_25;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_26;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_27;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_28;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_29;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_3;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_4;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_5;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_6;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_7;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_8;
  wire ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_9;
  wire ama_addmuladd_16s_16s_13ns_31s_31_4_1_U10_n_0;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_0;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_1;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_10;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_11;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_12;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_13;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_14;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_15;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_16;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_17;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_18;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_19;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_2;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_20;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_21;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_22;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_23;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_24;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_25;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_26;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_27;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_28;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_29;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_3;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_30;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_31;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_32;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_33;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_34;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_4;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_5;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_6;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_7;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_8;
  wire ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_9;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_0;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_1;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_10;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_11;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_12;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_13;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_14;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_15;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_16;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_17;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_18;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_19;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_2;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_20;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_21;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_22;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_23;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_24;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_25;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_26;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_27;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_28;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_29;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_3;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_30;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_31;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_32;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_33;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_34;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_35;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_36;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_37;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_38;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_39;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_4;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_40;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_41;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_42;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_43;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_44;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_45;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_46;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_47;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_48;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_49;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_5;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_50;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_51;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_52;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_53;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_54;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_55;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_56;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_57;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_58;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_59;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_6;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_60;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_61;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_7;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_8;
  wire ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_9;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_0;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_1;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_10;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_11;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_12;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_13;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_14;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_15;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_16;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_17;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_18;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_19;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_2;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_20;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_21;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_22;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_23;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_24;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_25;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_26;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_27;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_28;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_29;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_3;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_30;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_31;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_32;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_4;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_5;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_6;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_7;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_8;
  wire ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_9;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_0;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_1;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_10;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_11;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_12;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_13;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_14;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_15;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_16;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_17;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_18;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_19;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_2;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_20;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_21;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_22;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_23;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_24;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_25;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_26;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_27;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_3;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_4;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_5;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_6;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_7;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_8;
  wire ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_9;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter2;
  wire ap_enable_reg_pp0_iter3;
  wire ap_enable_reg_pp0_iter4;
  wire ap_enable_reg_pp0_iter5;
  wire ap_enable_reg_pp0_iter6;
  wire ap_enable_reg_pp0_iter7;
  wire ap_enable_reg_pp0_iter8;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [15:0]din_data_reg_893;
  wire \din_last_reg_898_pp0_iter5_reg_reg[0]_srl6_n_0 ;
  wire din_last_reg_898_pp0_iter6_reg;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[0]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[10]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[11]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[12]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[13]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[14]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[15]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[1]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[2]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[3]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[4]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[5]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[6]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[7]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[8]_srl3_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[9]_srl3_n_0 ;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[0]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[10]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[11]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[12]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[13]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[14]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[15]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[1]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[2]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[3]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[4]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[5]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[6]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[7]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[8]_srl2_n_0 ;
  wire \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[9]_srl2_n_0 ;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7;
  wire fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70;
  wire [15:0]fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8;
  wire [15:0]fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9;
  wire icmp_ln139_reg_1055;
  wire \icmp_ln139_reg_1055[0]_i_1_n_0 ;
  wire \icmp_ln139_reg_1055[0]_i_3_n_0 ;
  wire \icmp_ln139_reg_1055[0]_i_4_n_0 ;
  wire \icmp_ln139_reg_1055[0]_i_5_n_0 ;
  wire \icmp_ln139_reg_1055_reg[0]_i_2_n_3 ;
  wire icmp_ln142_fu_725_p2;
  wire icmp_ln142_reg_1061;
  wire \icmp_ln142_reg_1061[0]_i_2_n_0 ;
  wire \icmp_ln142_reg_1061[0]_i_3_n_0 ;
  wire \icmp_ln142_reg_1061[0]_i_4_n_0 ;
  wire \icmp_ln142_reg_1061[0]_i_5_n_0 ;
  wire \icmp_ln142_reg_1061_reg[0]_i_1_n_3 ;
  wire [15:0]in_r_TDATA;
  wire [15:0]in_r_TDATA_int_regslice;
  wire [0:0]in_r_TLAST;
  wire in_r_TLAST_int_regslice;
  wire in_r_TREADY;
  wire in_r_TVALID;
  wire in_r_TVALID_int_regslice;
  wire [15:0]out_r_TDATA;
  wire [0:0]out_r_TLAST;
  wire out_r_TREADY;
  wire out_r_TVALID;
  wire [18:2]p_shl1_cast_fu_624_p1;
  wire regslice_both_out_r_V_data_V_U_n_1;
  wire [16:0]tmp29_fu_600_p2;
  wire \tmp29_reg_974[11]_i_2_n_0 ;
  wire \tmp29_reg_974[11]_i_3_n_0 ;
  wire \tmp29_reg_974[11]_i_4_n_0 ;
  wire \tmp29_reg_974[11]_i_5_n_0 ;
  wire \tmp29_reg_974[15]_i_2_n_0 ;
  wire \tmp29_reg_974[15]_i_3_n_0 ;
  wire \tmp29_reg_974[15]_i_4_n_0 ;
  wire \tmp29_reg_974[15]_i_5_n_0 ;
  wire \tmp29_reg_974[15]_i_6_n_0 ;
  wire \tmp29_reg_974[3]_i_2_n_0 ;
  wire \tmp29_reg_974[3]_i_3_n_0 ;
  wire \tmp29_reg_974[3]_i_4_n_0 ;
  wire \tmp29_reg_974[3]_i_5_n_0 ;
  wire \tmp29_reg_974[7]_i_2_n_0 ;
  wire \tmp29_reg_974[7]_i_3_n_0 ;
  wire \tmp29_reg_974[7]_i_4_n_0 ;
  wire \tmp29_reg_974[7]_i_5_n_0 ;
  wire \tmp29_reg_974_reg[11]_i_1_n_0 ;
  wire \tmp29_reg_974_reg[11]_i_1_n_1 ;
  wire \tmp29_reg_974_reg[11]_i_1_n_2 ;
  wire \tmp29_reg_974_reg[11]_i_1_n_3 ;
  wire \tmp29_reg_974_reg[15]_i_1_n_0 ;
  wire \tmp29_reg_974_reg[15]_i_1_n_1 ;
  wire \tmp29_reg_974_reg[15]_i_1_n_2 ;
  wire \tmp29_reg_974_reg[15]_i_1_n_3 ;
  wire \tmp29_reg_974_reg[3]_i_1_n_0 ;
  wire \tmp29_reg_974_reg[3]_i_1_n_1 ;
  wire \tmp29_reg_974_reg[3]_i_1_n_2 ;
  wire \tmp29_reg_974_reg[3]_i_1_n_3 ;
  wire \tmp29_reg_974_reg[7]_i_1_n_0 ;
  wire \tmp29_reg_974_reg[7]_i_1_n_1 ;
  wire \tmp29_reg_974_reg[7]_i_1_n_2 ;
  wire \tmp29_reg_974_reg[7]_i_1_n_3 ;
  wire [0:0]tmp_1_fu_709_p4;
  wire [2:1]tmp_1_fu_709_p4__0;
  wire [3:0]\NLW_acc_reg_1050_reg[19]_i_11_O_UNCONNECTED ;
  wire [3:0]\NLW_acc_reg_1050_reg[19]_i_2_O_UNCONNECTED ;
  wire [3:0]\NLW_acc_reg_1050_reg[19]_i_20_O_UNCONNECTED ;
  wire [3:0]\NLW_acc_reg_1050_reg[19]_i_29_O_UNCONNECTED ;
  wire [3:1]\NLW_add_ln131_12_reg_1045_reg[29]_i_1_CO_UNCONNECTED ;
  wire [3:2]\NLW_add_ln131_12_reg_1045_reg[29]_i_1_O_UNCONNECTED ;
  wire [3:1]\NLW_icmp_ln139_reg_1055_reg[0]_i_2_CO_UNCONNECTED ;
  wire [3:2]\NLW_icmp_ln139_reg_1055_reg[0]_i_2_O_UNCONNECTED ;
  wire [3:2]\NLW_icmp_ln142_reg_1061_reg[0]_i_1_CO_UNCONNECTED ;
  wire [3:0]\NLW_icmp_ln142_reg_1061_reg[0]_i_1_O_UNCONNECTED ;
  wire [3:0]\NLW_tmp29_reg_974_reg[16]_i_1_CO_UNCONNECTED ;
  wire [3:1]\NLW_tmp29_reg_974_reg[16]_i_1_O_UNCONNECTED ;

  assign out_r_TKEEP[1] = \<const0> ;
  assign out_r_TKEEP[0] = \<const0> ;
  assign out_r_TSTRB[1] = \<const0> ;
  assign out_r_TSTRB[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  (* HLUTNM = "lutpair40" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_10 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[16]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[16]),
        .I2(add_ln131_12_reg_1045[16]),
        .I3(\acc_reg_1050[19]_i_6_n_0 ),
        .O(\acc_reg_1050[19]_i_10_n_0 ));
  (* HLUTNM = "lutpair38" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_12 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[14]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[14]),
        .I2(add_ln131_12_reg_1045[14]),
        .O(\acc_reg_1050[19]_i_12_n_0 ));
  (* HLUTNM = "lutpair37" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_13 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[13]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[13]),
        .I2(add_ln131_12_reg_1045[13]),
        .O(\acc_reg_1050[19]_i_13_n_0 ));
  (* HLUTNM = "lutpair36" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_14 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[12]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[12]),
        .I2(add_ln131_12_reg_1045[12]),
        .O(\acc_reg_1050[19]_i_14_n_0 ));
  (* HLUTNM = "lutpair35" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_15 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[11]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[11]),
        .I2(add_ln131_12_reg_1045[11]),
        .O(\acc_reg_1050[19]_i_15_n_0 ));
  (* HLUTNM = "lutpair39" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_16 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[15]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[15]),
        .I2(add_ln131_12_reg_1045[15]),
        .I3(\acc_reg_1050[19]_i_12_n_0 ),
        .O(\acc_reg_1050[19]_i_16_n_0 ));
  (* HLUTNM = "lutpair38" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_17 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[14]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[14]),
        .I2(add_ln131_12_reg_1045[14]),
        .I3(\acc_reg_1050[19]_i_13_n_0 ),
        .O(\acc_reg_1050[19]_i_17_n_0 ));
  (* HLUTNM = "lutpair37" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_18 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[13]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[13]),
        .I2(add_ln131_12_reg_1045[13]),
        .I3(\acc_reg_1050[19]_i_14_n_0 ),
        .O(\acc_reg_1050[19]_i_18_n_0 ));
  (* HLUTNM = "lutpair36" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_19 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[12]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[12]),
        .I2(add_ln131_12_reg_1045[12]),
        .I3(\acc_reg_1050[19]_i_15_n_0 ),
        .O(\acc_reg_1050[19]_i_19_n_0 ));
  (* HLUTNM = "lutpair34" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_21 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[10]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[10]),
        .I2(add_ln131_12_reg_1045[10]),
        .O(\acc_reg_1050[19]_i_21_n_0 ));
  (* HLUTNM = "lutpair33" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_22 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[9]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[9]),
        .I2(add_ln131_12_reg_1045[9]),
        .O(\acc_reg_1050[19]_i_22_n_0 ));
  (* HLUTNM = "lutpair32" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_23 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[8]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[8]),
        .I2(add_ln131_12_reg_1045[8]),
        .O(\acc_reg_1050[19]_i_23_n_0 ));
  (* HLUTNM = "lutpair31" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_24 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[7]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[7]),
        .I2(add_ln131_12_reg_1045[7]),
        .O(\acc_reg_1050[19]_i_24_n_0 ));
  (* HLUTNM = "lutpair35" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_25 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[11]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[11]),
        .I2(add_ln131_12_reg_1045[11]),
        .I3(\acc_reg_1050[19]_i_21_n_0 ),
        .O(\acc_reg_1050[19]_i_25_n_0 ));
  (* HLUTNM = "lutpair34" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_26 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[10]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[10]),
        .I2(add_ln131_12_reg_1045[10]),
        .I3(\acc_reg_1050[19]_i_22_n_0 ),
        .O(\acc_reg_1050[19]_i_26_n_0 ));
  (* HLUTNM = "lutpair33" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_27 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[9]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[9]),
        .I2(add_ln131_12_reg_1045[9]),
        .I3(\acc_reg_1050[19]_i_23_n_0 ),
        .O(\acc_reg_1050[19]_i_27_n_0 ));
  (* HLUTNM = "lutpair32" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_28 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[8]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[8]),
        .I2(add_ln131_12_reg_1045[8]),
        .I3(\acc_reg_1050[19]_i_24_n_0 ),
        .O(\acc_reg_1050[19]_i_28_n_0 ));
  (* HLUTNM = "lutpair42" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_3 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[18]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[18]),
        .I2(add_ln131_12_reg_1045[18]),
        .O(\acc_reg_1050[19]_i_3_n_0 ));
  (* HLUTNM = "lutpair30" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_30 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[6]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[6]),
        .I2(add_ln131_12_reg_1045[6]),
        .O(\acc_reg_1050[19]_i_30_n_0 ));
  (* HLUTNM = "lutpair29" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_31 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[5]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[5]),
        .I2(add_ln131_12_reg_1045[5]),
        .O(\acc_reg_1050[19]_i_31_n_0 ));
  (* HLUTNM = "lutpair28" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_32 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[4]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[4]),
        .I2(add_ln131_12_reg_1045[4]),
        .O(\acc_reg_1050[19]_i_32_n_0 ));
  (* HLUTNM = "lutpair27" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_33 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[3]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[3]),
        .I2(add_ln131_12_reg_1045[3]),
        .O(\acc_reg_1050[19]_i_33_n_0 ));
  (* HLUTNM = "lutpair31" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_34 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[7]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[7]),
        .I2(add_ln131_12_reg_1045[7]),
        .I3(\acc_reg_1050[19]_i_30_n_0 ),
        .O(\acc_reg_1050[19]_i_34_n_0 ));
  (* HLUTNM = "lutpair30" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_35 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[6]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[6]),
        .I2(add_ln131_12_reg_1045[6]),
        .I3(\acc_reg_1050[19]_i_31_n_0 ),
        .O(\acc_reg_1050[19]_i_35_n_0 ));
  (* HLUTNM = "lutpair29" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_36 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[5]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[5]),
        .I2(add_ln131_12_reg_1045[5]),
        .I3(\acc_reg_1050[19]_i_32_n_0 ),
        .O(\acc_reg_1050[19]_i_36_n_0 ));
  (* HLUTNM = "lutpair28" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_37 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[4]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[4]),
        .I2(add_ln131_12_reg_1045[4]),
        .I3(\acc_reg_1050[19]_i_33_n_0 ),
        .O(\acc_reg_1050[19]_i_37_n_0 ));
  (* HLUTNM = "lutpair26" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_38 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[2]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[2]),
        .I2(add_ln131_12_reg_1045[2]),
        .O(\acc_reg_1050[19]_i_38_n_0 ));
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_39 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[1]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[1]),
        .I2(add_ln131_12_reg_1045[1]),
        .O(\acc_reg_1050[19]_i_39_n_0 ));
  (* HLUTNM = "lutpair41" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_4 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[17]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[17]),
        .I2(add_ln131_12_reg_1045[17]),
        .O(\acc_reg_1050[19]_i_4_n_0 ));
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_40 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[0]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[0]),
        .I2(add_ln131_12_reg_1045[0]),
        .O(\acc_reg_1050[19]_i_40_n_0 ));
  (* HLUTNM = "lutpair27" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_41 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[3]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[3]),
        .I2(add_ln131_12_reg_1045[3]),
        .I3(\acc_reg_1050[19]_i_38_n_0 ),
        .O(\acc_reg_1050[19]_i_41_n_0 ));
  (* HLUTNM = "lutpair26" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_42 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[2]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[2]),
        .I2(add_ln131_12_reg_1045[2]),
        .I3(\acc_reg_1050[19]_i_39_n_0 ),
        .O(\acc_reg_1050[19]_i_42_n_0 ));
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_43 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[1]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[1]),
        .I2(add_ln131_12_reg_1045[1]),
        .I3(\acc_reg_1050[19]_i_40_n_0 ),
        .O(\acc_reg_1050[19]_i_43_n_0 ));
  LUT3 #(
    .INIT(8'h96)) 
    \acc_reg_1050[19]_i_44 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[0]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[0]),
        .I2(add_ln131_12_reg_1045[0]),
        .O(\acc_reg_1050[19]_i_44_n_0 ));
  (* HLUTNM = "lutpair40" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_5 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[16]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[16]),
        .I2(add_ln131_12_reg_1045[16]),
        .O(\acc_reg_1050[19]_i_5_n_0 ));
  (* HLUTNM = "lutpair39" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[19]_i_6 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[15]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[15]),
        .I2(add_ln131_12_reg_1045[15]),
        .O(\acc_reg_1050[19]_i_6_n_0 ));
  (* HLUTNM = "lutpair43" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_7 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[19]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[19]),
        .I2(add_ln131_12_reg_1045[19]),
        .I3(\acc_reg_1050[19]_i_3_n_0 ),
        .O(\acc_reg_1050[19]_i_7_n_0 ));
  (* HLUTNM = "lutpair42" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_8 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[18]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[18]),
        .I2(add_ln131_12_reg_1045[18]),
        .I3(\acc_reg_1050[19]_i_4_n_0 ),
        .O(\acc_reg_1050[19]_i_8_n_0 ));
  (* HLUTNM = "lutpair41" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[19]_i_9 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[17]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[17]),
        .I2(add_ln131_12_reg_1045[17]),
        .I3(\acc_reg_1050[19]_i_5_n_0 ),
        .O(\acc_reg_1050[19]_i_9_n_0 ));
  (* HLUTNM = "lutpair46" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[23]_i_2 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[22]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[22]),
        .I2(add_ln131_12_reg_1045[22]),
        .O(\acc_reg_1050[23]_i_2_n_0 ));
  (* HLUTNM = "lutpair45" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[23]_i_3 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[21]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[21]),
        .I2(add_ln131_12_reg_1045[21]),
        .O(\acc_reg_1050[23]_i_3_n_0 ));
  (* HLUTNM = "lutpair44" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[23]_i_4 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[20]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[20]),
        .I2(add_ln131_12_reg_1045[20]),
        .O(\acc_reg_1050[23]_i_4_n_0 ));
  (* HLUTNM = "lutpair43" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[23]_i_5 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[19]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[19]),
        .I2(add_ln131_12_reg_1045[19]),
        .O(\acc_reg_1050[23]_i_5_n_0 ));
  (* HLUTNM = "lutpair47" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[23]_i_6 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[23]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[23]),
        .I2(add_ln131_12_reg_1045[23]),
        .I3(\acc_reg_1050[23]_i_2_n_0 ),
        .O(\acc_reg_1050[23]_i_6_n_0 ));
  (* HLUTNM = "lutpair46" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[23]_i_7 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[22]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[22]),
        .I2(add_ln131_12_reg_1045[22]),
        .I3(\acc_reg_1050[23]_i_3_n_0 ),
        .O(\acc_reg_1050[23]_i_7_n_0 ));
  (* HLUTNM = "lutpair45" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[23]_i_8 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[21]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[21]),
        .I2(add_ln131_12_reg_1045[21]),
        .I3(\acc_reg_1050[23]_i_4_n_0 ),
        .O(\acc_reg_1050[23]_i_8_n_0 ));
  (* HLUTNM = "lutpair44" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[23]_i_9 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[20]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[20]),
        .I2(add_ln131_12_reg_1045[20]),
        .I3(\acc_reg_1050[23]_i_5_n_0 ),
        .O(\acc_reg_1050[23]_i_9_n_0 ));
  (* HLUTNM = "lutpair50" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[27]_i_2 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[26]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[26]),
        .I2(add_ln131_12_reg_1045[26]),
        .O(\acc_reg_1050[27]_i_2_n_0 ));
  (* HLUTNM = "lutpair49" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[27]_i_3 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[25]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[25]),
        .I2(add_ln131_12_reg_1045[25]),
        .O(\acc_reg_1050[27]_i_3_n_0 ));
  (* HLUTNM = "lutpair48" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[27]_i_4 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[24]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[24]),
        .I2(add_ln131_12_reg_1045[24]),
        .O(\acc_reg_1050[27]_i_4_n_0 ));
  (* HLUTNM = "lutpair47" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[27]_i_5 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[23]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[23]),
        .I2(add_ln131_12_reg_1045[23]),
        .O(\acc_reg_1050[27]_i_5_n_0 ));
  (* HLUTNM = "lutpair51" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[27]_i_6 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[27]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[27]),
        .I2(add_ln131_12_reg_1045[27]),
        .I3(\acc_reg_1050[27]_i_2_n_0 ),
        .O(\acc_reg_1050[27]_i_6_n_0 ));
  (* HLUTNM = "lutpair50" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[27]_i_7 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[26]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[26]),
        .I2(add_ln131_12_reg_1045[26]),
        .I3(\acc_reg_1050[27]_i_3_n_0 ),
        .O(\acc_reg_1050[27]_i_7_n_0 ));
  (* HLUTNM = "lutpair49" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[27]_i_8 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[25]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[25]),
        .I2(add_ln131_12_reg_1045[25]),
        .I3(\acc_reg_1050[27]_i_4_n_0 ),
        .O(\acc_reg_1050[27]_i_8_n_0 ));
  (* HLUTNM = "lutpair48" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[27]_i_9 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[24]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[24]),
        .I2(add_ln131_12_reg_1045[24]),
        .I3(\acc_reg_1050[27]_i_5_n_0 ),
        .O(\acc_reg_1050[27]_i_9_n_0 ));
  LUT4 #(
    .INIT(16'hE00E)) 
    \acc_reg_1050[31]_i_2 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[29]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[29]),
        .I2(add_ln131_1_reg_1020_pp0_iter5_reg[30]),
        .I3(add_ln131_4_reg_1025_pp0_iter5_reg[30]),
        .O(\acc_reg_1050[31]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'h09)) 
    \acc_reg_1050[31]_i_3 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[29]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[29]),
        .I2(add_ln131_12_reg_1045[29]),
        .O(\acc_reg_1050[31]_i_3_n_0 ));
  LUT3 #(
    .INIT(8'h96)) 
    \acc_reg_1050[31]_i_4 
       (.I0(add_ln131_12_reg_1045[29]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[29]),
        .I2(add_ln131_1_reg_1020_pp0_iter5_reg[29]),
        .O(\acc_reg_1050[31]_i_4_n_0 ));
  (* HLUTNM = "lutpair51" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \acc_reg_1050[31]_i_5 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[27]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[27]),
        .I2(add_ln131_12_reg_1045[27]),
        .O(\acc_reg_1050[31]_i_5_n_0 ));
  LUT6 #(
    .INIT(64'hE11E0FF00FF01EE1)) 
    \acc_reg_1050[31]_i_6 
       (.I0(add_ln131_4_reg_1025_pp0_iter5_reg[29]),
        .I1(add_ln131_1_reg_1020_pp0_iter5_reg[29]),
        .I2(add_ln131_4_reg_1025_pp0_iter5_reg[31]),
        .I3(add_ln131_1_reg_1020_pp0_iter5_reg[31]),
        .I4(add_ln131_4_reg_1025_pp0_iter5_reg[30]),
        .I5(add_ln131_1_reg_1020_pp0_iter5_reg[30]),
        .O(\acc_reg_1050[31]_i_6_n_0 ));
  LUT5 #(
    .INIT(32'h693C3C96)) 
    \acc_reg_1050[31]_i_7 
       (.I0(add_ln131_12_reg_1045[29]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[30]),
        .I2(add_ln131_1_reg_1020_pp0_iter5_reg[30]),
        .I3(add_ln131_4_reg_1025_pp0_iter5_reg[29]),
        .I4(add_ln131_1_reg_1020_pp0_iter5_reg[29]),
        .O(\acc_reg_1050[31]_i_7_n_0 ));
  LUT6 #(
    .INIT(64'h6969699669969696)) 
    \acc_reg_1050[31]_i_8 
       (.I0(add_ln131_12_reg_1045[29]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[29]),
        .I2(add_ln131_1_reg_1020_pp0_iter5_reg[29]),
        .I3(add_ln131_12_reg_1045[28]),
        .I4(add_ln131_4_reg_1025_pp0_iter5_reg[28]),
        .I5(add_ln131_1_reg_1020_pp0_iter5_reg[28]),
        .O(\acc_reg_1050[31]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'h6996)) 
    \acc_reg_1050[31]_i_9 
       (.I0(\acc_reg_1050[31]_i_5_n_0 ),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[28]),
        .I2(add_ln131_1_reg_1020_pp0_iter5_reg[28]),
        .I3(add_ln131_12_reg_1045[28]),
        .O(\acc_reg_1050[31]_i_9_n_0 ));
  FDRE \acc_reg_1050_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[19]_i_1_n_7 ),
        .Q(acc_reg_1050[16]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[19]_i_1_n_6 ),
        .Q(acc_reg_1050[17]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[19]_i_1_n_5 ),
        .Q(acc_reg_1050[18]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[19]_i_1_n_4 ),
        .Q(acc_reg_1050[19]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \acc_reg_1050_reg[19]_i_1 
       (.CI(\acc_reg_1050_reg[19]_i_2_n_0 ),
        .CO({\acc_reg_1050_reg[19]_i_1_n_0 ,\acc_reg_1050_reg[19]_i_1_n_1 ,\acc_reg_1050_reg[19]_i_1_n_2 ,\acc_reg_1050_reg[19]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_reg_1050[19]_i_3_n_0 ,\acc_reg_1050[19]_i_4_n_0 ,\acc_reg_1050[19]_i_5_n_0 ,\acc_reg_1050[19]_i_6_n_0 }),
        .O({\acc_reg_1050_reg[19]_i_1_n_4 ,\acc_reg_1050_reg[19]_i_1_n_5 ,\acc_reg_1050_reg[19]_i_1_n_6 ,\acc_reg_1050_reg[19]_i_1_n_7 }),
        .S({\acc_reg_1050[19]_i_7_n_0 ,\acc_reg_1050[19]_i_8_n_0 ,\acc_reg_1050[19]_i_9_n_0 ,\acc_reg_1050[19]_i_10_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \acc_reg_1050_reg[19]_i_11 
       (.CI(\acc_reg_1050_reg[19]_i_20_n_0 ),
        .CO({\acc_reg_1050_reg[19]_i_11_n_0 ,\acc_reg_1050_reg[19]_i_11_n_1 ,\acc_reg_1050_reg[19]_i_11_n_2 ,\acc_reg_1050_reg[19]_i_11_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_reg_1050[19]_i_21_n_0 ,\acc_reg_1050[19]_i_22_n_0 ,\acc_reg_1050[19]_i_23_n_0 ,\acc_reg_1050[19]_i_24_n_0 }),
        .O(\NLW_acc_reg_1050_reg[19]_i_11_O_UNCONNECTED [3:0]),
        .S({\acc_reg_1050[19]_i_25_n_0 ,\acc_reg_1050[19]_i_26_n_0 ,\acc_reg_1050[19]_i_27_n_0 ,\acc_reg_1050[19]_i_28_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \acc_reg_1050_reg[19]_i_2 
       (.CI(\acc_reg_1050_reg[19]_i_11_n_0 ),
        .CO({\acc_reg_1050_reg[19]_i_2_n_0 ,\acc_reg_1050_reg[19]_i_2_n_1 ,\acc_reg_1050_reg[19]_i_2_n_2 ,\acc_reg_1050_reg[19]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_reg_1050[19]_i_12_n_0 ,\acc_reg_1050[19]_i_13_n_0 ,\acc_reg_1050[19]_i_14_n_0 ,\acc_reg_1050[19]_i_15_n_0 }),
        .O(\NLW_acc_reg_1050_reg[19]_i_2_O_UNCONNECTED [3:0]),
        .S({\acc_reg_1050[19]_i_16_n_0 ,\acc_reg_1050[19]_i_17_n_0 ,\acc_reg_1050[19]_i_18_n_0 ,\acc_reg_1050[19]_i_19_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \acc_reg_1050_reg[19]_i_20 
       (.CI(\acc_reg_1050_reg[19]_i_29_n_0 ),
        .CO({\acc_reg_1050_reg[19]_i_20_n_0 ,\acc_reg_1050_reg[19]_i_20_n_1 ,\acc_reg_1050_reg[19]_i_20_n_2 ,\acc_reg_1050_reg[19]_i_20_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_reg_1050[19]_i_30_n_0 ,\acc_reg_1050[19]_i_31_n_0 ,\acc_reg_1050[19]_i_32_n_0 ,\acc_reg_1050[19]_i_33_n_0 }),
        .O(\NLW_acc_reg_1050_reg[19]_i_20_O_UNCONNECTED [3:0]),
        .S({\acc_reg_1050[19]_i_34_n_0 ,\acc_reg_1050[19]_i_35_n_0 ,\acc_reg_1050[19]_i_36_n_0 ,\acc_reg_1050[19]_i_37_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \acc_reg_1050_reg[19]_i_29 
       (.CI(1'b0),
        .CO({\acc_reg_1050_reg[19]_i_29_n_0 ,\acc_reg_1050_reg[19]_i_29_n_1 ,\acc_reg_1050_reg[19]_i_29_n_2 ,\acc_reg_1050_reg[19]_i_29_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_reg_1050[19]_i_38_n_0 ,\acc_reg_1050[19]_i_39_n_0 ,\acc_reg_1050[19]_i_40_n_0 ,1'b0}),
        .O(\NLW_acc_reg_1050_reg[19]_i_29_O_UNCONNECTED [3:0]),
        .S({\acc_reg_1050[19]_i_41_n_0 ,\acc_reg_1050[19]_i_42_n_0 ,\acc_reg_1050[19]_i_43_n_0 ,\acc_reg_1050[19]_i_44_n_0 }));
  FDRE \acc_reg_1050_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[23]_i_1_n_7 ),
        .Q(acc_reg_1050[20]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[23]_i_1_n_6 ),
        .Q(acc_reg_1050[21]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[23]_i_1_n_5 ),
        .Q(acc_reg_1050[22]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[23]_i_1_n_4 ),
        .Q(acc_reg_1050[23]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \acc_reg_1050_reg[23]_i_1 
       (.CI(\acc_reg_1050_reg[19]_i_1_n_0 ),
        .CO({\acc_reg_1050_reg[23]_i_1_n_0 ,\acc_reg_1050_reg[23]_i_1_n_1 ,\acc_reg_1050_reg[23]_i_1_n_2 ,\acc_reg_1050_reg[23]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_reg_1050[23]_i_2_n_0 ,\acc_reg_1050[23]_i_3_n_0 ,\acc_reg_1050[23]_i_4_n_0 ,\acc_reg_1050[23]_i_5_n_0 }),
        .O({\acc_reg_1050_reg[23]_i_1_n_4 ,\acc_reg_1050_reg[23]_i_1_n_5 ,\acc_reg_1050_reg[23]_i_1_n_6 ,\acc_reg_1050_reg[23]_i_1_n_7 }),
        .S({\acc_reg_1050[23]_i_6_n_0 ,\acc_reg_1050[23]_i_7_n_0 ,\acc_reg_1050[23]_i_8_n_0 ,\acc_reg_1050[23]_i_9_n_0 }));
  FDRE \acc_reg_1050_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[27]_i_1_n_7 ),
        .Q(acc_reg_1050[24]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[25] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[27]_i_1_n_6 ),
        .Q(acc_reg_1050[25]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[27]_i_1_n_5 ),
        .Q(acc_reg_1050[26]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[27]_i_1_n_4 ),
        .Q(acc_reg_1050[27]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \acc_reg_1050_reg[27]_i_1 
       (.CI(\acc_reg_1050_reg[23]_i_1_n_0 ),
        .CO({\acc_reg_1050_reg[27]_i_1_n_0 ,\acc_reg_1050_reg[27]_i_1_n_1 ,\acc_reg_1050_reg[27]_i_1_n_2 ,\acc_reg_1050_reg[27]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_reg_1050[27]_i_2_n_0 ,\acc_reg_1050[27]_i_3_n_0 ,\acc_reg_1050[27]_i_4_n_0 ,\acc_reg_1050[27]_i_5_n_0 }),
        .O({\acc_reg_1050_reg[27]_i_1_n_4 ,\acc_reg_1050_reg[27]_i_1_n_5 ,\acc_reg_1050_reg[27]_i_1_n_6 ,\acc_reg_1050_reg[27]_i_1_n_7 }),
        .S({\acc_reg_1050[27]_i_6_n_0 ,\acc_reg_1050[27]_i_7_n_0 ,\acc_reg_1050[27]_i_8_n_0 ,\acc_reg_1050[27]_i_9_n_0 }));
  FDRE \acc_reg_1050_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[31]_i_1_n_7 ),
        .Q(acc_reg_1050[28]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[29] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[31]_i_1_n_6 ),
        .Q(acc_reg_1050[29]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[30] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\acc_reg_1050_reg[31]_i_1_n_5 ),
        .Q(acc_reg_1050[30]),
        .R(1'b0));
  FDRE \acc_reg_1050_reg[31] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp_1_fu_709_p4),
        .Q(acc_reg_1050[31]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \acc_reg_1050_reg[31]_i_1 
       (.CI(\acc_reg_1050_reg[27]_i_1_n_0 ),
        .CO({\acc_reg_1050_reg[31]_i_1_n_0 ,\acc_reg_1050_reg[31]_i_1_n_1 ,\acc_reg_1050_reg[31]_i_1_n_2 ,\acc_reg_1050_reg[31]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\acc_reg_1050[31]_i_2_n_0 ,\acc_reg_1050[31]_i_3_n_0 ,\acc_reg_1050[31]_i_4_n_0 ,\acc_reg_1050[31]_i_5_n_0 }),
        .O({tmp_1_fu_709_p4,\acc_reg_1050_reg[31]_i_1_n_5 ,\acc_reg_1050_reg[31]_i_1_n_6 ,\acc_reg_1050_reg[31]_i_1_n_7 }),
        .S({\acc_reg_1050[31]_i_6_n_0 ,\acc_reg_1050[31]_i_7_n_0 ,\acc_reg_1050[31]_i_8_n_0 ,\acc_reg_1050[31]_i_9_n_0 }));
  FDRE \add_ln131_11_reg_1040_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[0]),
        .Q(add_ln131_11_reg_1040[0]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[10]),
        .Q(add_ln131_11_reg_1040[10]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[11]),
        .Q(add_ln131_11_reg_1040[11]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[12]),
        .Q(add_ln131_11_reg_1040[12]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[13]),
        .Q(add_ln131_11_reg_1040[13]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[14]),
        .Q(add_ln131_11_reg_1040[14]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[15]),
        .Q(add_ln131_11_reg_1040[15]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[16]),
        .Q(add_ln131_11_reg_1040[16]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[17]),
        .Q(add_ln131_11_reg_1040[17]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[18]),
        .Q(add_ln131_11_reg_1040[18]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[19]),
        .Q(add_ln131_11_reg_1040[19]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[1]),
        .Q(add_ln131_11_reg_1040[1]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[20]),
        .Q(add_ln131_11_reg_1040[20]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[21]),
        .Q(add_ln131_11_reg_1040[21]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[22]),
        .Q(add_ln131_11_reg_1040[22]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[23]),
        .Q(add_ln131_11_reg_1040[23]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[24]),
        .Q(add_ln131_11_reg_1040[24]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[25] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[25]),
        .Q(add_ln131_11_reg_1040[25]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[26]),
        .Q(add_ln131_11_reg_1040[26]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[27]),
        .Q(add_ln131_11_reg_1040[27]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[28]),
        .Q(add_ln131_11_reg_1040[28]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[2]),
        .Q(add_ln131_11_reg_1040[2]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[3]),
        .Q(add_ln131_11_reg_1040[3]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[4]),
        .Q(add_ln131_11_reg_1040[4]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[5]),
        .Q(add_ln131_11_reg_1040[5]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[6]),
        .Q(add_ln131_11_reg_1040[6]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[7]),
        .Q(add_ln131_11_reg_1040[7]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[8]),
        .Q(add_ln131_11_reg_1040[8]),
        .R(1'b0));
  FDRE \add_ln131_11_reg_1040_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_11_fu_652_p2[9]),
        .Q(add_ln131_11_reg_1040[9]),
        .R(1'b0));
  (* HLUTNM = "lutpair10" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[11]_i_2 
       (.I0(add_ln131_6_reg_1030[10]),
        .I1(add_ln131_7_reg_1035[10]),
        .I2(add_ln131_11_reg_1040[10]),
        .O(\add_ln131_12_reg_1045[11]_i_2_n_0 ));
  (* HLUTNM = "lutpair9" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[11]_i_3 
       (.I0(add_ln131_6_reg_1030[9]),
        .I1(add_ln131_7_reg_1035[9]),
        .I2(add_ln131_11_reg_1040[9]),
        .O(\add_ln131_12_reg_1045[11]_i_3_n_0 ));
  (* HLUTNM = "lutpair8" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[11]_i_4 
       (.I0(add_ln131_6_reg_1030[8]),
        .I1(add_ln131_7_reg_1035[8]),
        .I2(add_ln131_11_reg_1040[8]),
        .O(\add_ln131_12_reg_1045[11]_i_4_n_0 ));
  (* HLUTNM = "lutpair7" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[11]_i_5 
       (.I0(add_ln131_6_reg_1030[7]),
        .I1(add_ln131_7_reg_1035[7]),
        .I2(add_ln131_11_reg_1040[7]),
        .O(\add_ln131_12_reg_1045[11]_i_5_n_0 ));
  (* HLUTNM = "lutpair11" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[11]_i_6 
       (.I0(add_ln131_6_reg_1030[11]),
        .I1(add_ln131_7_reg_1035[11]),
        .I2(add_ln131_11_reg_1040[11]),
        .I3(\add_ln131_12_reg_1045[11]_i_2_n_0 ),
        .O(\add_ln131_12_reg_1045[11]_i_6_n_0 ));
  (* HLUTNM = "lutpair10" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[11]_i_7 
       (.I0(add_ln131_6_reg_1030[10]),
        .I1(add_ln131_7_reg_1035[10]),
        .I2(add_ln131_11_reg_1040[10]),
        .I3(\add_ln131_12_reg_1045[11]_i_3_n_0 ),
        .O(\add_ln131_12_reg_1045[11]_i_7_n_0 ));
  (* HLUTNM = "lutpair9" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[11]_i_8 
       (.I0(add_ln131_6_reg_1030[9]),
        .I1(add_ln131_7_reg_1035[9]),
        .I2(add_ln131_11_reg_1040[9]),
        .I3(\add_ln131_12_reg_1045[11]_i_4_n_0 ),
        .O(\add_ln131_12_reg_1045[11]_i_8_n_0 ));
  (* HLUTNM = "lutpair8" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[11]_i_9 
       (.I0(add_ln131_6_reg_1030[8]),
        .I1(add_ln131_7_reg_1035[8]),
        .I2(add_ln131_11_reg_1040[8]),
        .I3(\add_ln131_12_reg_1045[11]_i_5_n_0 ),
        .O(\add_ln131_12_reg_1045[11]_i_9_n_0 ));
  (* HLUTNM = "lutpair14" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[15]_i_2 
       (.I0(add_ln131_6_reg_1030[14]),
        .I1(add_ln131_7_reg_1035[14]),
        .I2(add_ln131_11_reg_1040[14]),
        .O(\add_ln131_12_reg_1045[15]_i_2_n_0 ));
  (* HLUTNM = "lutpair13" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[15]_i_3 
       (.I0(add_ln131_6_reg_1030[13]),
        .I1(add_ln131_7_reg_1035[13]),
        .I2(add_ln131_11_reg_1040[13]),
        .O(\add_ln131_12_reg_1045[15]_i_3_n_0 ));
  (* HLUTNM = "lutpair12" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[15]_i_4 
       (.I0(add_ln131_6_reg_1030[12]),
        .I1(add_ln131_7_reg_1035[12]),
        .I2(add_ln131_11_reg_1040[12]),
        .O(\add_ln131_12_reg_1045[15]_i_4_n_0 ));
  (* HLUTNM = "lutpair11" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[15]_i_5 
       (.I0(add_ln131_6_reg_1030[11]),
        .I1(add_ln131_7_reg_1035[11]),
        .I2(add_ln131_11_reg_1040[11]),
        .O(\add_ln131_12_reg_1045[15]_i_5_n_0 ));
  (* HLUTNM = "lutpair15" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[15]_i_6 
       (.I0(add_ln131_6_reg_1030[15]),
        .I1(add_ln131_7_reg_1035[15]),
        .I2(add_ln131_11_reg_1040[15]),
        .I3(\add_ln131_12_reg_1045[15]_i_2_n_0 ),
        .O(\add_ln131_12_reg_1045[15]_i_6_n_0 ));
  (* HLUTNM = "lutpair14" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[15]_i_7 
       (.I0(add_ln131_6_reg_1030[14]),
        .I1(add_ln131_7_reg_1035[14]),
        .I2(add_ln131_11_reg_1040[14]),
        .I3(\add_ln131_12_reg_1045[15]_i_3_n_0 ),
        .O(\add_ln131_12_reg_1045[15]_i_7_n_0 ));
  (* HLUTNM = "lutpair13" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[15]_i_8 
       (.I0(add_ln131_6_reg_1030[13]),
        .I1(add_ln131_7_reg_1035[13]),
        .I2(add_ln131_11_reg_1040[13]),
        .I3(\add_ln131_12_reg_1045[15]_i_4_n_0 ),
        .O(\add_ln131_12_reg_1045[15]_i_8_n_0 ));
  (* HLUTNM = "lutpair12" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[15]_i_9 
       (.I0(add_ln131_6_reg_1030[12]),
        .I1(add_ln131_7_reg_1035[12]),
        .I2(add_ln131_11_reg_1040[12]),
        .I3(\add_ln131_12_reg_1045[15]_i_5_n_0 ),
        .O(\add_ln131_12_reg_1045[15]_i_9_n_0 ));
  (* HLUTNM = "lutpair18" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[19]_i_2 
       (.I0(add_ln131_6_reg_1030[18]),
        .I1(add_ln131_7_reg_1035[18]),
        .I2(add_ln131_11_reg_1040[18]),
        .O(\add_ln131_12_reg_1045[19]_i_2_n_0 ));
  (* HLUTNM = "lutpair17" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[19]_i_3 
       (.I0(add_ln131_6_reg_1030[17]),
        .I1(add_ln131_7_reg_1035[17]),
        .I2(add_ln131_11_reg_1040[17]),
        .O(\add_ln131_12_reg_1045[19]_i_3_n_0 ));
  (* HLUTNM = "lutpair16" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[19]_i_4 
       (.I0(add_ln131_6_reg_1030[16]),
        .I1(add_ln131_7_reg_1035[16]),
        .I2(add_ln131_11_reg_1040[16]),
        .O(\add_ln131_12_reg_1045[19]_i_4_n_0 ));
  (* HLUTNM = "lutpair15" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[19]_i_5 
       (.I0(add_ln131_6_reg_1030[15]),
        .I1(add_ln131_7_reg_1035[15]),
        .I2(add_ln131_11_reg_1040[15]),
        .O(\add_ln131_12_reg_1045[19]_i_5_n_0 ));
  (* HLUTNM = "lutpair19" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[19]_i_6 
       (.I0(add_ln131_6_reg_1030[19]),
        .I1(add_ln131_7_reg_1035[19]),
        .I2(add_ln131_11_reg_1040[19]),
        .I3(\add_ln131_12_reg_1045[19]_i_2_n_0 ),
        .O(\add_ln131_12_reg_1045[19]_i_6_n_0 ));
  (* HLUTNM = "lutpair18" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[19]_i_7 
       (.I0(add_ln131_6_reg_1030[18]),
        .I1(add_ln131_7_reg_1035[18]),
        .I2(add_ln131_11_reg_1040[18]),
        .I3(\add_ln131_12_reg_1045[19]_i_3_n_0 ),
        .O(\add_ln131_12_reg_1045[19]_i_7_n_0 ));
  (* HLUTNM = "lutpair17" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[19]_i_8 
       (.I0(add_ln131_6_reg_1030[17]),
        .I1(add_ln131_7_reg_1035[17]),
        .I2(add_ln131_11_reg_1040[17]),
        .I3(\add_ln131_12_reg_1045[19]_i_4_n_0 ),
        .O(\add_ln131_12_reg_1045[19]_i_8_n_0 ));
  (* HLUTNM = "lutpair16" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[19]_i_9 
       (.I0(add_ln131_6_reg_1030[16]),
        .I1(add_ln131_7_reg_1035[16]),
        .I2(add_ln131_11_reg_1040[16]),
        .I3(\add_ln131_12_reg_1045[19]_i_5_n_0 ),
        .O(\add_ln131_12_reg_1045[19]_i_9_n_0 ));
  (* HLUTNM = "lutpair22" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[23]_i_2 
       (.I0(add_ln131_6_reg_1030[22]),
        .I1(add_ln131_7_reg_1035[22]),
        .I2(add_ln131_11_reg_1040[22]),
        .O(\add_ln131_12_reg_1045[23]_i_2_n_0 ));
  (* HLUTNM = "lutpair21" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[23]_i_3 
       (.I0(add_ln131_6_reg_1030[21]),
        .I1(add_ln131_7_reg_1035[21]),
        .I2(add_ln131_11_reg_1040[21]),
        .O(\add_ln131_12_reg_1045[23]_i_3_n_0 ));
  (* HLUTNM = "lutpair20" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[23]_i_4 
       (.I0(add_ln131_6_reg_1030[20]),
        .I1(add_ln131_7_reg_1035[20]),
        .I2(add_ln131_11_reg_1040[20]),
        .O(\add_ln131_12_reg_1045[23]_i_4_n_0 ));
  (* HLUTNM = "lutpair19" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[23]_i_5 
       (.I0(add_ln131_6_reg_1030[19]),
        .I1(add_ln131_7_reg_1035[19]),
        .I2(add_ln131_11_reg_1040[19]),
        .O(\add_ln131_12_reg_1045[23]_i_5_n_0 ));
  (* HLUTNM = "lutpair23" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[23]_i_6 
       (.I0(add_ln131_6_reg_1030[23]),
        .I1(add_ln131_7_reg_1035[23]),
        .I2(add_ln131_11_reg_1040[23]),
        .I3(\add_ln131_12_reg_1045[23]_i_2_n_0 ),
        .O(\add_ln131_12_reg_1045[23]_i_6_n_0 ));
  (* HLUTNM = "lutpair22" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[23]_i_7 
       (.I0(add_ln131_6_reg_1030[22]),
        .I1(add_ln131_7_reg_1035[22]),
        .I2(add_ln131_11_reg_1040[22]),
        .I3(\add_ln131_12_reg_1045[23]_i_3_n_0 ),
        .O(\add_ln131_12_reg_1045[23]_i_7_n_0 ));
  (* HLUTNM = "lutpair21" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[23]_i_8 
       (.I0(add_ln131_6_reg_1030[21]),
        .I1(add_ln131_7_reg_1035[21]),
        .I2(add_ln131_11_reg_1040[21]),
        .I3(\add_ln131_12_reg_1045[23]_i_4_n_0 ),
        .O(\add_ln131_12_reg_1045[23]_i_8_n_0 ));
  (* HLUTNM = "lutpair20" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[23]_i_9 
       (.I0(add_ln131_6_reg_1030[20]),
        .I1(add_ln131_7_reg_1035[20]),
        .I2(add_ln131_11_reg_1040[20]),
        .I3(\add_ln131_12_reg_1045[23]_i_5_n_0 ),
        .O(\add_ln131_12_reg_1045[23]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'h96)) 
    \add_ln131_12_reg_1045[27]_i_2 
       (.I0(add_ln131_6_reg_1030[27]),
        .I1(add_ln131_7_reg_1035[27]),
        .I2(add_ln131_11_reg_1040[27]),
        .O(\add_ln131_12_reg_1045[27]_i_2_n_0 ));
  (* HLUTNM = "lutpair25" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[27]_i_3 
       (.I0(add_ln131_6_reg_1030[25]),
        .I1(add_ln131_7_reg_1035[25]),
        .I2(add_ln131_11_reg_1040[25]),
        .O(\add_ln131_12_reg_1045[27]_i_3_n_0 ));
  (* HLUTNM = "lutpair24" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[27]_i_4 
       (.I0(add_ln131_6_reg_1030[24]),
        .I1(add_ln131_7_reg_1035[24]),
        .I2(add_ln131_11_reg_1040[24]),
        .O(\add_ln131_12_reg_1045[27]_i_4_n_0 ));
  (* HLUTNM = "lutpair23" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[27]_i_5 
       (.I0(add_ln131_6_reg_1030[23]),
        .I1(add_ln131_7_reg_1035[23]),
        .I2(add_ln131_11_reg_1040[23]),
        .O(\add_ln131_12_reg_1045[27]_i_5_n_0 ));
  LUT6 #(
    .INIT(64'h6969699669969696)) 
    \add_ln131_12_reg_1045[27]_i_6 
       (.I0(add_ln131_11_reg_1040[27]),
        .I1(add_ln131_7_reg_1035[27]),
        .I2(add_ln131_6_reg_1030[27]),
        .I3(add_ln131_11_reg_1040[26]),
        .I4(add_ln131_7_reg_1035[26]),
        .I5(add_ln131_6_reg_1030[26]),
        .O(\add_ln131_12_reg_1045[27]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[27]_i_7 
       (.I0(\add_ln131_12_reg_1045[27]_i_3_n_0 ),
        .I1(add_ln131_7_reg_1035[26]),
        .I2(add_ln131_6_reg_1030[26]),
        .I3(add_ln131_11_reg_1040[26]),
        .O(\add_ln131_12_reg_1045[27]_i_7_n_0 ));
  (* HLUTNM = "lutpair25" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[27]_i_8 
       (.I0(add_ln131_6_reg_1030[25]),
        .I1(add_ln131_7_reg_1035[25]),
        .I2(add_ln131_11_reg_1040[25]),
        .I3(\add_ln131_12_reg_1045[27]_i_4_n_0 ),
        .O(\add_ln131_12_reg_1045[27]_i_8_n_0 ));
  (* HLUTNM = "lutpair24" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[27]_i_9 
       (.I0(add_ln131_6_reg_1030[24]),
        .I1(add_ln131_7_reg_1035[24]),
        .I2(add_ln131_11_reg_1040[24]),
        .I3(\add_ln131_12_reg_1045[27]_i_5_n_0 ),
        .O(\add_ln131_12_reg_1045[27]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'h28)) 
    \add_ln131_12_reg_1045[29]_i_2 
       (.I0(add_ln131_11_reg_1040[27]),
        .I1(add_ln131_6_reg_1030[27]),
        .I2(add_ln131_7_reg_1035[27]),
        .O(\add_ln131_12_reg_1045[29]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'hBFF4)) 
    \add_ln131_12_reg_1045[29]_i_3 
       (.I0(add_ln131_7_reg_1035[27]),
        .I1(add_ln131_6_reg_1030[27]),
        .I2(add_ln131_7_reg_1035[28]),
        .I3(add_ln131_11_reg_1040[28]),
        .O(\add_ln131_12_reg_1045[29]_i_3_n_0 ));
  LUT5 #(
    .INIT(32'h3C69963C)) 
    \add_ln131_12_reg_1045[29]_i_4 
       (.I0(add_ln131_11_reg_1040[27]),
        .I1(add_ln131_11_reg_1040[28]),
        .I2(add_ln131_7_reg_1035[28]),
        .I3(add_ln131_7_reg_1035[27]),
        .I4(add_ln131_6_reg_1030[27]),
        .O(\add_ln131_12_reg_1045[29]_i_4_n_0 ));
  (* HLUTNM = "lutpair2" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[3]_i_2 
       (.I0(add_ln131_6_reg_1030[2]),
        .I1(add_ln131_7_reg_1035[2]),
        .I2(add_ln131_11_reg_1040[2]),
        .O(\add_ln131_12_reg_1045[3]_i_2_n_0 ));
  (* HLUTNM = "lutpair1" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[3]_i_3 
       (.I0(add_ln131_6_reg_1030[1]),
        .I1(add_ln131_7_reg_1035[1]),
        .I2(add_ln131_11_reg_1040[1]),
        .O(\add_ln131_12_reg_1045[3]_i_3_n_0 ));
  (* HLUTNM = "lutpair0" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[3]_i_4 
       (.I0(add_ln131_6_reg_1030[0]),
        .I1(add_ln131_7_reg_1035[0]),
        .I2(add_ln131_11_reg_1040[0]),
        .O(\add_ln131_12_reg_1045[3]_i_4_n_0 ));
  (* HLUTNM = "lutpair3" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[3]_i_5 
       (.I0(add_ln131_6_reg_1030[3]),
        .I1(add_ln131_7_reg_1035[3]),
        .I2(add_ln131_11_reg_1040[3]),
        .I3(\add_ln131_12_reg_1045[3]_i_2_n_0 ),
        .O(\add_ln131_12_reg_1045[3]_i_5_n_0 ));
  (* HLUTNM = "lutpair2" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[3]_i_6 
       (.I0(add_ln131_6_reg_1030[2]),
        .I1(add_ln131_7_reg_1035[2]),
        .I2(add_ln131_11_reg_1040[2]),
        .I3(\add_ln131_12_reg_1045[3]_i_3_n_0 ),
        .O(\add_ln131_12_reg_1045[3]_i_6_n_0 ));
  (* HLUTNM = "lutpair1" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[3]_i_7 
       (.I0(add_ln131_6_reg_1030[1]),
        .I1(add_ln131_7_reg_1035[1]),
        .I2(add_ln131_11_reg_1040[1]),
        .I3(\add_ln131_12_reg_1045[3]_i_4_n_0 ),
        .O(\add_ln131_12_reg_1045[3]_i_7_n_0 ));
  (* HLUTNM = "lutpair0" *) 
  LUT3 #(
    .INIT(8'h96)) 
    \add_ln131_12_reg_1045[3]_i_8 
       (.I0(add_ln131_6_reg_1030[0]),
        .I1(add_ln131_7_reg_1035[0]),
        .I2(add_ln131_11_reg_1040[0]),
        .O(\add_ln131_12_reg_1045[3]_i_8_n_0 ));
  (* HLUTNM = "lutpair6" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[7]_i_2 
       (.I0(add_ln131_6_reg_1030[6]),
        .I1(add_ln131_7_reg_1035[6]),
        .I2(add_ln131_11_reg_1040[6]),
        .O(\add_ln131_12_reg_1045[7]_i_2_n_0 ));
  (* HLUTNM = "lutpair5" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[7]_i_3 
       (.I0(add_ln131_6_reg_1030[5]),
        .I1(add_ln131_7_reg_1035[5]),
        .I2(add_ln131_11_reg_1040[5]),
        .O(\add_ln131_12_reg_1045[7]_i_3_n_0 ));
  (* HLUTNM = "lutpair4" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[7]_i_4 
       (.I0(add_ln131_6_reg_1030[4]),
        .I1(add_ln131_7_reg_1035[4]),
        .I2(add_ln131_11_reg_1040[4]),
        .O(\add_ln131_12_reg_1045[7]_i_4_n_0 ));
  (* HLUTNM = "lutpair3" *) 
  LUT3 #(
    .INIT(8'hE8)) 
    \add_ln131_12_reg_1045[7]_i_5 
       (.I0(add_ln131_6_reg_1030[3]),
        .I1(add_ln131_7_reg_1035[3]),
        .I2(add_ln131_11_reg_1040[3]),
        .O(\add_ln131_12_reg_1045[7]_i_5_n_0 ));
  (* HLUTNM = "lutpair7" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[7]_i_6 
       (.I0(add_ln131_6_reg_1030[7]),
        .I1(add_ln131_7_reg_1035[7]),
        .I2(add_ln131_11_reg_1040[7]),
        .I3(\add_ln131_12_reg_1045[7]_i_2_n_0 ),
        .O(\add_ln131_12_reg_1045[7]_i_6_n_0 ));
  (* HLUTNM = "lutpair6" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[7]_i_7 
       (.I0(add_ln131_6_reg_1030[6]),
        .I1(add_ln131_7_reg_1035[6]),
        .I2(add_ln131_11_reg_1040[6]),
        .I3(\add_ln131_12_reg_1045[7]_i_3_n_0 ),
        .O(\add_ln131_12_reg_1045[7]_i_7_n_0 ));
  (* HLUTNM = "lutpair5" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[7]_i_8 
       (.I0(add_ln131_6_reg_1030[5]),
        .I1(add_ln131_7_reg_1035[5]),
        .I2(add_ln131_11_reg_1040[5]),
        .I3(\add_ln131_12_reg_1045[7]_i_4_n_0 ),
        .O(\add_ln131_12_reg_1045[7]_i_8_n_0 ));
  (* HLUTNM = "lutpair4" *) 
  LUT4 #(
    .INIT(16'h6996)) 
    \add_ln131_12_reg_1045[7]_i_9 
       (.I0(add_ln131_6_reg_1030[4]),
        .I1(add_ln131_7_reg_1035[4]),
        .I2(add_ln131_11_reg_1040[4]),
        .I3(\add_ln131_12_reg_1045[7]_i_5_n_0 ),
        .O(\add_ln131_12_reg_1045[7]_i_9_n_0 ));
  FDRE \add_ln131_12_reg_1045_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[0]),
        .Q(add_ln131_12_reg_1045[0]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[10]),
        .Q(add_ln131_12_reg_1045[10]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[11]),
        .Q(add_ln131_12_reg_1045[11]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_12_reg_1045_reg[11]_i_1 
       (.CI(\add_ln131_12_reg_1045_reg[7]_i_1_n_0 ),
        .CO({\add_ln131_12_reg_1045_reg[11]_i_1_n_0 ,\add_ln131_12_reg_1045_reg[11]_i_1_n_1 ,\add_ln131_12_reg_1045_reg[11]_i_1_n_2 ,\add_ln131_12_reg_1045_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\add_ln131_12_reg_1045[11]_i_2_n_0 ,\add_ln131_12_reg_1045[11]_i_3_n_0 ,\add_ln131_12_reg_1045[11]_i_4_n_0 ,\add_ln131_12_reg_1045[11]_i_5_n_0 }),
        .O(add_ln131_12_fu_672_p2[11:8]),
        .S({\add_ln131_12_reg_1045[11]_i_6_n_0 ,\add_ln131_12_reg_1045[11]_i_7_n_0 ,\add_ln131_12_reg_1045[11]_i_8_n_0 ,\add_ln131_12_reg_1045[11]_i_9_n_0 }));
  FDRE \add_ln131_12_reg_1045_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[12]),
        .Q(add_ln131_12_reg_1045[12]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[13]),
        .Q(add_ln131_12_reg_1045[13]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[14]),
        .Q(add_ln131_12_reg_1045[14]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[15]),
        .Q(add_ln131_12_reg_1045[15]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_12_reg_1045_reg[15]_i_1 
       (.CI(\add_ln131_12_reg_1045_reg[11]_i_1_n_0 ),
        .CO({\add_ln131_12_reg_1045_reg[15]_i_1_n_0 ,\add_ln131_12_reg_1045_reg[15]_i_1_n_1 ,\add_ln131_12_reg_1045_reg[15]_i_1_n_2 ,\add_ln131_12_reg_1045_reg[15]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\add_ln131_12_reg_1045[15]_i_2_n_0 ,\add_ln131_12_reg_1045[15]_i_3_n_0 ,\add_ln131_12_reg_1045[15]_i_4_n_0 ,\add_ln131_12_reg_1045[15]_i_5_n_0 }),
        .O(add_ln131_12_fu_672_p2[15:12]),
        .S({\add_ln131_12_reg_1045[15]_i_6_n_0 ,\add_ln131_12_reg_1045[15]_i_7_n_0 ,\add_ln131_12_reg_1045[15]_i_8_n_0 ,\add_ln131_12_reg_1045[15]_i_9_n_0 }));
  FDRE \add_ln131_12_reg_1045_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[16]),
        .Q(add_ln131_12_reg_1045[16]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[17]),
        .Q(add_ln131_12_reg_1045[17]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[18]),
        .Q(add_ln131_12_reg_1045[18]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[19]),
        .Q(add_ln131_12_reg_1045[19]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_12_reg_1045_reg[19]_i_1 
       (.CI(\add_ln131_12_reg_1045_reg[15]_i_1_n_0 ),
        .CO({\add_ln131_12_reg_1045_reg[19]_i_1_n_0 ,\add_ln131_12_reg_1045_reg[19]_i_1_n_1 ,\add_ln131_12_reg_1045_reg[19]_i_1_n_2 ,\add_ln131_12_reg_1045_reg[19]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\add_ln131_12_reg_1045[19]_i_2_n_0 ,\add_ln131_12_reg_1045[19]_i_3_n_0 ,\add_ln131_12_reg_1045[19]_i_4_n_0 ,\add_ln131_12_reg_1045[19]_i_5_n_0 }),
        .O(add_ln131_12_fu_672_p2[19:16]),
        .S({\add_ln131_12_reg_1045[19]_i_6_n_0 ,\add_ln131_12_reg_1045[19]_i_7_n_0 ,\add_ln131_12_reg_1045[19]_i_8_n_0 ,\add_ln131_12_reg_1045[19]_i_9_n_0 }));
  FDRE \add_ln131_12_reg_1045_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[1]),
        .Q(add_ln131_12_reg_1045[1]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[20]),
        .Q(add_ln131_12_reg_1045[20]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[21]),
        .Q(add_ln131_12_reg_1045[21]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[22]),
        .Q(add_ln131_12_reg_1045[22]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[23]),
        .Q(add_ln131_12_reg_1045[23]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_12_reg_1045_reg[23]_i_1 
       (.CI(\add_ln131_12_reg_1045_reg[19]_i_1_n_0 ),
        .CO({\add_ln131_12_reg_1045_reg[23]_i_1_n_0 ,\add_ln131_12_reg_1045_reg[23]_i_1_n_1 ,\add_ln131_12_reg_1045_reg[23]_i_1_n_2 ,\add_ln131_12_reg_1045_reg[23]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\add_ln131_12_reg_1045[23]_i_2_n_0 ,\add_ln131_12_reg_1045[23]_i_3_n_0 ,\add_ln131_12_reg_1045[23]_i_4_n_0 ,\add_ln131_12_reg_1045[23]_i_5_n_0 }),
        .O(add_ln131_12_fu_672_p2[23:20]),
        .S({\add_ln131_12_reg_1045[23]_i_6_n_0 ,\add_ln131_12_reg_1045[23]_i_7_n_0 ,\add_ln131_12_reg_1045[23]_i_8_n_0 ,\add_ln131_12_reg_1045[23]_i_9_n_0 }));
  FDRE \add_ln131_12_reg_1045_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[24]),
        .Q(add_ln131_12_reg_1045[24]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[25] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[25]),
        .Q(add_ln131_12_reg_1045[25]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[26]),
        .Q(add_ln131_12_reg_1045[26]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[27]),
        .Q(add_ln131_12_reg_1045[27]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_12_reg_1045_reg[27]_i_1 
       (.CI(\add_ln131_12_reg_1045_reg[23]_i_1_n_0 ),
        .CO({\add_ln131_12_reg_1045_reg[27]_i_1_n_0 ,\add_ln131_12_reg_1045_reg[27]_i_1_n_1 ,\add_ln131_12_reg_1045_reg[27]_i_1_n_2 ,\add_ln131_12_reg_1045_reg[27]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\add_ln131_12_reg_1045[27]_i_2_n_0 ,\add_ln131_12_reg_1045[27]_i_3_n_0 ,\add_ln131_12_reg_1045[27]_i_4_n_0 ,\add_ln131_12_reg_1045[27]_i_5_n_0 }),
        .O(add_ln131_12_fu_672_p2[27:24]),
        .S({\add_ln131_12_reg_1045[27]_i_6_n_0 ,\add_ln131_12_reg_1045[27]_i_7_n_0 ,\add_ln131_12_reg_1045[27]_i_8_n_0 ,\add_ln131_12_reg_1045[27]_i_9_n_0 }));
  FDRE \add_ln131_12_reg_1045_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[28]),
        .Q(add_ln131_12_reg_1045[28]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[29] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[29]),
        .Q(add_ln131_12_reg_1045[29]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_12_reg_1045_reg[29]_i_1 
       (.CI(\add_ln131_12_reg_1045_reg[27]_i_1_n_0 ),
        .CO({\NLW_add_ln131_12_reg_1045_reg[29]_i_1_CO_UNCONNECTED [3:1],\add_ln131_12_reg_1045_reg[29]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,\add_ln131_12_reg_1045[29]_i_2_n_0 }),
        .O({\NLW_add_ln131_12_reg_1045_reg[29]_i_1_O_UNCONNECTED [3:2],add_ln131_12_fu_672_p2[29:28]}),
        .S({1'b0,1'b0,\add_ln131_12_reg_1045[29]_i_3_n_0 ,\add_ln131_12_reg_1045[29]_i_4_n_0 }));
  FDRE \add_ln131_12_reg_1045_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[2]),
        .Q(add_ln131_12_reg_1045[2]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[3]),
        .Q(add_ln131_12_reg_1045[3]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_12_reg_1045_reg[3]_i_1 
       (.CI(1'b0),
        .CO({\add_ln131_12_reg_1045_reg[3]_i_1_n_0 ,\add_ln131_12_reg_1045_reg[3]_i_1_n_1 ,\add_ln131_12_reg_1045_reg[3]_i_1_n_2 ,\add_ln131_12_reg_1045_reg[3]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\add_ln131_12_reg_1045[3]_i_2_n_0 ,\add_ln131_12_reg_1045[3]_i_3_n_0 ,\add_ln131_12_reg_1045[3]_i_4_n_0 ,1'b0}),
        .O(add_ln131_12_fu_672_p2[3:0]),
        .S({\add_ln131_12_reg_1045[3]_i_5_n_0 ,\add_ln131_12_reg_1045[3]_i_6_n_0 ,\add_ln131_12_reg_1045[3]_i_7_n_0 ,\add_ln131_12_reg_1045[3]_i_8_n_0 }));
  FDRE \add_ln131_12_reg_1045_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[4]),
        .Q(add_ln131_12_reg_1045[4]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[5]),
        .Q(add_ln131_12_reg_1045[5]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[6]),
        .Q(add_ln131_12_reg_1045[6]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[7]),
        .Q(add_ln131_12_reg_1045[7]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_12_reg_1045_reg[7]_i_1 
       (.CI(\add_ln131_12_reg_1045_reg[3]_i_1_n_0 ),
        .CO({\add_ln131_12_reg_1045_reg[7]_i_1_n_0 ,\add_ln131_12_reg_1045_reg[7]_i_1_n_1 ,\add_ln131_12_reg_1045_reg[7]_i_1_n_2 ,\add_ln131_12_reg_1045_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\add_ln131_12_reg_1045[7]_i_2_n_0 ,\add_ln131_12_reg_1045[7]_i_3_n_0 ,\add_ln131_12_reg_1045[7]_i_4_n_0 ,\add_ln131_12_reg_1045[7]_i_5_n_0 }),
        .O(add_ln131_12_fu_672_p2[7:4]),
        .S({\add_ln131_12_reg_1045[7]_i_6_n_0 ,\add_ln131_12_reg_1045[7]_i_7_n_0 ,\add_ln131_12_reg_1045[7]_i_8_n_0 ,\add_ln131_12_reg_1045[7]_i_9_n_0 }));
  FDRE \add_ln131_12_reg_1045_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[8]),
        .Q(add_ln131_12_reg_1045[8]),
        .R(1'b0));
  FDRE \add_ln131_12_reg_1045_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_12_fu_672_p2[9]),
        .Q(add_ln131_12_reg_1045[9]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[0]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[0]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[10]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[10]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[11]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[11]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[12]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[12]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[13]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[13]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[14]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[14]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[15]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[15]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[16]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[16]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[17]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[17]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[18]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[18]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[19]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[19]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[1]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[1]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[20]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[20]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[21]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[21]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[22]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[22]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[23]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[23]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[24]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[24]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[25] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[25]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[25]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[26]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[26]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[27]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[27]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[28]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[28]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[29] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[29]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[29]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[2]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[2]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[30] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[30]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[30]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[31] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[31]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[31]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[32] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[32]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[32]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[3]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[3]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[4]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[4]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[5]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[5]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[6]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[6]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[7]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[7]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[8]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[8]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_pp0_iter5_reg_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_1_reg_1020[9]),
        .Q(add_ln131_1_reg_1020_pp0_iter5_reg[9]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[0] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_32),
        .Q(add_ln131_1_reg_1020[0]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[10] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_22),
        .Q(add_ln131_1_reg_1020[10]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[11] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_21),
        .Q(add_ln131_1_reg_1020[11]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[12] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_20),
        .Q(add_ln131_1_reg_1020[12]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[13] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_19),
        .Q(add_ln131_1_reg_1020[13]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[14] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_18),
        .Q(add_ln131_1_reg_1020[14]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[15] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_17),
        .Q(add_ln131_1_reg_1020[15]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[16] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_16),
        .Q(add_ln131_1_reg_1020[16]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[17] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_15),
        .Q(add_ln131_1_reg_1020[17]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[18] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_14),
        .Q(add_ln131_1_reg_1020[18]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[19] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_13),
        .Q(add_ln131_1_reg_1020[19]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[1] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_31),
        .Q(add_ln131_1_reg_1020[1]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[20] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_12),
        .Q(add_ln131_1_reg_1020[20]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[21] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_11),
        .Q(add_ln131_1_reg_1020[21]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[22] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_10),
        .Q(add_ln131_1_reg_1020[22]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[23] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_9),
        .Q(add_ln131_1_reg_1020[23]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[24] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_8),
        .Q(add_ln131_1_reg_1020[24]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[25] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_7),
        .Q(add_ln131_1_reg_1020[25]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[26] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_6),
        .Q(add_ln131_1_reg_1020[26]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[27] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_5),
        .Q(add_ln131_1_reg_1020[27]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[28] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_4),
        .Q(add_ln131_1_reg_1020[28]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[29] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_3),
        .Q(add_ln131_1_reg_1020[29]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[2] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_30),
        .Q(add_ln131_1_reg_1020[2]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[30] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_2),
        .Q(add_ln131_1_reg_1020[30]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[31] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_1),
        .Q(add_ln131_1_reg_1020[31]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[32] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_0),
        .Q(add_ln131_1_reg_1020[32]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[3] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_29),
        .Q(add_ln131_1_reg_1020[3]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[4] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_28),
        .Q(add_ln131_1_reg_1020[4]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[5] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_27),
        .Q(add_ln131_1_reg_1020[5]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[6] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_26),
        .Q(add_ln131_1_reg_1020[6]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[7] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_25),
        .Q(add_ln131_1_reg_1020[7]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[8] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_24),
        .Q(add_ln131_1_reg_1020[8]),
        .R(1'b0));
  FDRE \add_ln131_1_reg_1020_reg[9] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_23),
        .Q(add_ln131_1_reg_1020[9]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[0]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[0]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[10]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[10]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[11]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[11]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[12]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[12]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[13]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[13]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[14]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[14]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[15]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[15]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[16]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[16]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[17]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[17]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[18]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[18]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[19]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[19]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[1]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[1]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[20]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[20]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[21]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[21]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[22]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[22]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[23]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[23]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[24]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[24]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[25] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[25]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[25]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[26]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[26]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[27]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[27]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[28]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[28]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[29] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[29]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[29]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[2]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[2]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[30] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[30]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[30]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[31] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[31]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[31]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[32] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[32]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[32]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[3]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[3]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[4]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[4]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[5]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[5]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[6]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[6]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[7]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[7]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[8]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[8]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_pp0_iter5_reg_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_reg_1025[9]),
        .Q(add_ln131_4_reg_1025_pp0_iter5_reg[9]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[0]),
        .Q(add_ln131_4_reg_1025[0]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[10]),
        .Q(add_ln131_4_reg_1025[10]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[11]),
        .Q(add_ln131_4_reg_1025[11]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[12]),
        .Q(add_ln131_4_reg_1025[12]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[13]),
        .Q(add_ln131_4_reg_1025[13]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[14]),
        .Q(add_ln131_4_reg_1025[14]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[15]),
        .Q(add_ln131_4_reg_1025[15]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[16]),
        .Q(add_ln131_4_reg_1025[16]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[17]),
        .Q(add_ln131_4_reg_1025[17]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[18]),
        .Q(add_ln131_4_reg_1025[18]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[19]),
        .Q(add_ln131_4_reg_1025[19]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[1]),
        .Q(add_ln131_4_reg_1025[1]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[20]),
        .Q(add_ln131_4_reg_1025[20]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[21]),
        .Q(add_ln131_4_reg_1025[21]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[22]),
        .Q(add_ln131_4_reg_1025[22]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[23]),
        .Q(add_ln131_4_reg_1025[23]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[24]),
        .Q(add_ln131_4_reg_1025[24]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[25] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[25]),
        .Q(add_ln131_4_reg_1025[25]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[26]),
        .Q(add_ln131_4_reg_1025[26]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[27]),
        .Q(add_ln131_4_reg_1025[27]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[28]),
        .Q(add_ln131_4_reg_1025[28]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[29] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[29]),
        .Q(add_ln131_4_reg_1025[29]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[2]),
        .Q(add_ln131_4_reg_1025[2]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[30] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[30]),
        .Q(add_ln131_4_reg_1025[30]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[31] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[31]),
        .Q(add_ln131_4_reg_1025[31]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[32] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[32]),
        .Q(add_ln131_4_reg_1025[32]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[3]),
        .Q(add_ln131_4_reg_1025[3]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[4]),
        .Q(add_ln131_4_reg_1025[4]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[5]),
        .Q(add_ln131_4_reg_1025[5]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[6]),
        .Q(add_ln131_4_reg_1025[6]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[7]),
        .Q(add_ln131_4_reg_1025[7]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[8]),
        .Q(add_ln131_4_reg_1025[8]),
        .R(1'b0));
  FDRE \add_ln131_4_reg_1025_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(add_ln131_4_fu_644_p2[9]),
        .Q(add_ln131_4_reg_1025[9]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[0] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_27),
        .Q(add_ln131_6_reg_1030[0]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[10] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_17),
        .Q(add_ln131_6_reg_1030[10]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[11] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_16),
        .Q(add_ln131_6_reg_1030[11]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[12] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_15),
        .Q(add_ln131_6_reg_1030[12]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[13] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_14),
        .Q(add_ln131_6_reg_1030[13]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[14] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_13),
        .Q(add_ln131_6_reg_1030[14]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[15] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_12),
        .Q(add_ln131_6_reg_1030[15]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[16] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_11),
        .Q(add_ln131_6_reg_1030[16]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[17] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_10),
        .Q(add_ln131_6_reg_1030[17]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[18] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_9),
        .Q(add_ln131_6_reg_1030[18]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[19] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_8),
        .Q(add_ln131_6_reg_1030[19]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[1] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_26),
        .Q(add_ln131_6_reg_1030[1]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[20] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_7),
        .Q(add_ln131_6_reg_1030[20]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[21] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_6),
        .Q(add_ln131_6_reg_1030[21]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[22] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_5),
        .Q(add_ln131_6_reg_1030[22]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[23] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_4),
        .Q(add_ln131_6_reg_1030[23]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[24] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_3),
        .Q(add_ln131_6_reg_1030[24]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[25] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_2),
        .Q(add_ln131_6_reg_1030[25]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[26] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_1),
        .Q(add_ln131_6_reg_1030[26]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[27] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_0),
        .Q(add_ln131_6_reg_1030[27]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[2] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_25),
        .Q(add_ln131_6_reg_1030[2]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[3] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_24),
        .Q(add_ln131_6_reg_1030[3]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[4] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_23),
        .Q(add_ln131_6_reg_1030[4]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[5] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_22),
        .Q(add_ln131_6_reg_1030[5]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[6] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_21),
        .Q(add_ln131_6_reg_1030[6]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[7] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_20),
        .Q(add_ln131_6_reg_1030[7]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[8] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_19),
        .Q(add_ln131_6_reg_1030[8]),
        .R(1'b0));
  FDRE \add_ln131_6_reg_1030_reg[9] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_18),
        .Q(add_ln131_6_reg_1030[9]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[0] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_28),
        .Q(add_ln131_7_reg_1035[0]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[10] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_18),
        .Q(add_ln131_7_reg_1035[10]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[11] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_17),
        .Q(add_ln131_7_reg_1035[11]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[12] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_16),
        .Q(add_ln131_7_reg_1035[12]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[13] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_15),
        .Q(add_ln131_7_reg_1035[13]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[14] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_14),
        .Q(add_ln131_7_reg_1035[14]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[15] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_13),
        .Q(add_ln131_7_reg_1035[15]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[16] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_12),
        .Q(add_ln131_7_reg_1035[16]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[17] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_11),
        .Q(add_ln131_7_reg_1035[17]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[18] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_10),
        .Q(add_ln131_7_reg_1035[18]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[19] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_9),
        .Q(add_ln131_7_reg_1035[19]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[1] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_27),
        .Q(add_ln131_7_reg_1035[1]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[20] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_8),
        .Q(add_ln131_7_reg_1035[20]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[21] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_7),
        .Q(add_ln131_7_reg_1035[21]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[22] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_6),
        .Q(add_ln131_7_reg_1035[22]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[23] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_5),
        .Q(add_ln131_7_reg_1035[23]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[24] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_4),
        .Q(add_ln131_7_reg_1035[24]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[25] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_3),
        .Q(add_ln131_7_reg_1035[25]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[26] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_2),
        .Q(add_ln131_7_reg_1035[26]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[27] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_1),
        .Q(add_ln131_7_reg_1035[27]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[28] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_0),
        .Q(add_ln131_7_reg_1035[28]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[2] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_26),
        .Q(add_ln131_7_reg_1035[2]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[3] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_25),
        .Q(add_ln131_7_reg_1035[3]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[4] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_24),
        .Q(add_ln131_7_reg_1035[4]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[5] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_23),
        .Q(add_ln131_7_reg_1035[5]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[6] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_22),
        .Q(add_ln131_7_reg_1035[6]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[7] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_21),
        .Q(add_ln131_7_reg_1035[7]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[8] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_20),
        .Q(add_ln131_7_reg_1035[8]),
        .R(1'b0));
  FDRE \add_ln131_7_reg_1035_reg[9] 
       (.C(ap_clk),
        .CE(add_ln131_1_reg_10200),
        .D(ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_19),
        .Q(add_ln131_7_reg_1035[9]),
        .R(1'b0));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_11ns_28_4_1 am_addmul_16s_16s_11ns_28_4_1_U4
       (.PCOUT({am_addmul_16s_16s_11ns_28_4_1_U4_n_0,am_addmul_16s_16s_11ns_28_4_1_U4_n_1,am_addmul_16s_16s_11ns_28_4_1_U4_n_2,am_addmul_16s_16s_11ns_28_4_1_U4_n_3,am_addmul_16s_16s_11ns_28_4_1_U4_n_4,am_addmul_16s_16s_11ns_28_4_1_U4_n_5,am_addmul_16s_16s_11ns_28_4_1_U4_n_6,am_addmul_16s_16s_11ns_28_4_1_U4_n_7,am_addmul_16s_16s_11ns_28_4_1_U4_n_8,am_addmul_16s_16s_11ns_28_4_1_U4_n_9,am_addmul_16s_16s_11ns_28_4_1_U4_n_10,am_addmul_16s_16s_11ns_28_4_1_U4_n_11,am_addmul_16s_16s_11ns_28_4_1_U4_n_12,am_addmul_16s_16s_11ns_28_4_1_U4_n_13,am_addmul_16s_16s_11ns_28_4_1_U4_n_14,am_addmul_16s_16s_11ns_28_4_1_U4_n_15,am_addmul_16s_16s_11ns_28_4_1_U4_n_16,am_addmul_16s_16s_11ns_28_4_1_U4_n_17,am_addmul_16s_16s_11ns_28_4_1_U4_n_18,am_addmul_16s_16s_11ns_28_4_1_U4_n_19,am_addmul_16s_16s_11ns_28_4_1_U4_n_20,am_addmul_16s_16s_11ns_28_4_1_U4_n_21,am_addmul_16s_16s_11ns_28_4_1_U4_n_22,am_addmul_16s_16s_11ns_28_4_1_U4_n_23,am_addmul_16s_16s_11ns_28_4_1_U4_n_24,am_addmul_16s_16s_11ns_28_4_1_U4_n_25,am_addmul_16s_16s_11ns_28_4_1_U4_n_26,am_addmul_16s_16s_11ns_28_4_1_U4_n_27,am_addmul_16s_16s_11ns_28_4_1_U4_n_28,am_addmul_16s_16s_11ns_28_4_1_U4_n_29,am_addmul_16s_16s_11ns_28_4_1_U4_n_30,am_addmul_16s_16s_11ns_28_4_1_U4_n_31,am_addmul_16s_16s_11ns_28_4_1_U4_n_32,am_addmul_16s_16s_11ns_28_4_1_U4_n_33,am_addmul_16s_16s_11ns_28_4_1_U4_n_34,am_addmul_16s_16s_11ns_28_4_1_U4_n_35,am_addmul_16s_16s_11ns_28_4_1_U4_n_36,am_addmul_16s_16s_11ns_28_4_1_U4_n_37,am_addmul_16s_16s_11ns_28_4_1_U4_n_38,am_addmul_16s_16s_11ns_28_4_1_U4_n_39,am_addmul_16s_16s_11ns_28_4_1_U4_n_40,am_addmul_16s_16s_11ns_28_4_1_U4_n_41,am_addmul_16s_16s_11ns_28_4_1_U4_n_42,am_addmul_16s_16s_11ns_28_4_1_U4_n_43,am_addmul_16s_16s_11ns_28_4_1_U4_n_44,am_addmul_16s_16s_11ns_28_4_1_U4_n_45,am_addmul_16s_16s_11ns_28_4_1_U4_n_46,am_addmul_16s_16s_11ns_28_4_1_U4_n_47}),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_11s_28_4_1 am_addmul_16s_16s_11s_28_4_1_U7
       (.A(in_r_TDATA_int_regslice),
        .PCOUT({am_addmul_16s_16s_11s_28_4_1_U7_n_0,am_addmul_16s_16s_11s_28_4_1_U7_n_1,am_addmul_16s_16s_11s_28_4_1_U7_n_2,am_addmul_16s_16s_11s_28_4_1_U7_n_3,am_addmul_16s_16s_11s_28_4_1_U7_n_4,am_addmul_16s_16s_11s_28_4_1_U7_n_5,am_addmul_16s_16s_11s_28_4_1_U7_n_6,am_addmul_16s_16s_11s_28_4_1_U7_n_7,am_addmul_16s_16s_11s_28_4_1_U7_n_8,am_addmul_16s_16s_11s_28_4_1_U7_n_9,am_addmul_16s_16s_11s_28_4_1_U7_n_10,am_addmul_16s_16s_11s_28_4_1_U7_n_11,am_addmul_16s_16s_11s_28_4_1_U7_n_12,am_addmul_16s_16s_11s_28_4_1_U7_n_13,am_addmul_16s_16s_11s_28_4_1_U7_n_14,am_addmul_16s_16s_11s_28_4_1_U7_n_15,am_addmul_16s_16s_11s_28_4_1_U7_n_16,am_addmul_16s_16s_11s_28_4_1_U7_n_17,am_addmul_16s_16s_11s_28_4_1_U7_n_18,am_addmul_16s_16s_11s_28_4_1_U7_n_19,am_addmul_16s_16s_11s_28_4_1_U7_n_20,am_addmul_16s_16s_11s_28_4_1_U7_n_21,am_addmul_16s_16s_11s_28_4_1_U7_n_22,am_addmul_16s_16s_11s_28_4_1_U7_n_23,am_addmul_16s_16s_11s_28_4_1_U7_n_24,am_addmul_16s_16s_11s_28_4_1_U7_n_25,am_addmul_16s_16s_11s_28_4_1_U7_n_26,am_addmul_16s_16s_11s_28_4_1_U7_n_27,am_addmul_16s_16s_11s_28_4_1_U7_n_28,am_addmul_16s_16s_11s_28_4_1_U7_n_29,am_addmul_16s_16s_11s_28_4_1_U7_n_30,am_addmul_16s_16s_11s_28_4_1_U7_n_31,am_addmul_16s_16s_11s_28_4_1_U7_n_32,am_addmul_16s_16s_11s_28_4_1_U7_n_33,am_addmul_16s_16s_11s_28_4_1_U7_n_34,am_addmul_16s_16s_11s_28_4_1_U7_n_35,am_addmul_16s_16s_11s_28_4_1_U7_n_36,am_addmul_16s_16s_11s_28_4_1_U7_n_37,am_addmul_16s_16s_11s_28_4_1_U7_n_38,am_addmul_16s_16s_11s_28_4_1_U7_n_39,am_addmul_16s_16s_11s_28_4_1_U7_n_40,am_addmul_16s_16s_11s_28_4_1_U7_n_41,am_addmul_16s_16s_11s_28_4_1_U7_n_42,am_addmul_16s_16s_11s_28_4_1_U7_n_43,am_addmul_16s_16s_11s_28_4_1_U7_n_44,am_addmul_16s_16s_11s_28_4_1_U7_n_45,am_addmul_16s_16s_11s_28_4_1_U7_n_46,am_addmul_16s_16s_11s_28_4_1_U7_n_47}),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_12s_29_4_1 am_addmul_16s_16s_12s_29_4_1_U5
       (.PCOUT({am_addmul_16s_16s_12s_29_4_1_U5_n_0,am_addmul_16s_16s_12s_29_4_1_U5_n_1,am_addmul_16s_16s_12s_29_4_1_U5_n_2,am_addmul_16s_16s_12s_29_4_1_U5_n_3,am_addmul_16s_16s_12s_29_4_1_U5_n_4,am_addmul_16s_16s_12s_29_4_1_U5_n_5,am_addmul_16s_16s_12s_29_4_1_U5_n_6,am_addmul_16s_16s_12s_29_4_1_U5_n_7,am_addmul_16s_16s_12s_29_4_1_U5_n_8,am_addmul_16s_16s_12s_29_4_1_U5_n_9,am_addmul_16s_16s_12s_29_4_1_U5_n_10,am_addmul_16s_16s_12s_29_4_1_U5_n_11,am_addmul_16s_16s_12s_29_4_1_U5_n_12,am_addmul_16s_16s_12s_29_4_1_U5_n_13,am_addmul_16s_16s_12s_29_4_1_U5_n_14,am_addmul_16s_16s_12s_29_4_1_U5_n_15,am_addmul_16s_16s_12s_29_4_1_U5_n_16,am_addmul_16s_16s_12s_29_4_1_U5_n_17,am_addmul_16s_16s_12s_29_4_1_U5_n_18,am_addmul_16s_16s_12s_29_4_1_U5_n_19,am_addmul_16s_16s_12s_29_4_1_U5_n_20,am_addmul_16s_16s_12s_29_4_1_U5_n_21,am_addmul_16s_16s_12s_29_4_1_U5_n_22,am_addmul_16s_16s_12s_29_4_1_U5_n_23,am_addmul_16s_16s_12s_29_4_1_U5_n_24,am_addmul_16s_16s_12s_29_4_1_U5_n_25,am_addmul_16s_16s_12s_29_4_1_U5_n_26,am_addmul_16s_16s_12s_29_4_1_U5_n_27,am_addmul_16s_16s_12s_29_4_1_U5_n_28,am_addmul_16s_16s_12s_29_4_1_U5_n_29,am_addmul_16s_16s_12s_29_4_1_U5_n_30,am_addmul_16s_16s_12s_29_4_1_U5_n_31,am_addmul_16s_16s_12s_29_4_1_U5_n_32,am_addmul_16s_16s_12s_29_4_1_U5_n_33,am_addmul_16s_16s_12s_29_4_1_U5_n_34,am_addmul_16s_16s_12s_29_4_1_U5_n_35,am_addmul_16s_16s_12s_29_4_1_U5_n_36,am_addmul_16s_16s_12s_29_4_1_U5_n_37,am_addmul_16s_16s_12s_29_4_1_U5_n_38,am_addmul_16s_16s_12s_29_4_1_U5_n_39,am_addmul_16s_16s_12s_29_4_1_U5_n_40,am_addmul_16s_16s_12s_29_4_1_U5_n_41,am_addmul_16s_16s_12s_29_4_1_U5_n_42,am_addmul_16s_16s_12s_29_4_1_U5_n_43,am_addmul_16s_16s_12s_29_4_1_U5_n_44,am_addmul_16s_16s_12s_29_4_1_U5_n_45,am_addmul_16s_16s_12s_29_4_1_U5_n_46,am_addmul_16s_16s_12s_29_4_1_U5_n_47}),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_12s_29_4_1_0 am_addmul_16s_16s_12s_29_4_1_U6
       (.PCOUT({am_addmul_16s_16s_12s_29_4_1_U6_n_0,am_addmul_16s_16s_12s_29_4_1_U6_n_1,am_addmul_16s_16s_12s_29_4_1_U6_n_2,am_addmul_16s_16s_12s_29_4_1_U6_n_3,am_addmul_16s_16s_12s_29_4_1_U6_n_4,am_addmul_16s_16s_12s_29_4_1_U6_n_5,am_addmul_16s_16s_12s_29_4_1_U6_n_6,am_addmul_16s_16s_12s_29_4_1_U6_n_7,am_addmul_16s_16s_12s_29_4_1_U6_n_8,am_addmul_16s_16s_12s_29_4_1_U6_n_9,am_addmul_16s_16s_12s_29_4_1_U6_n_10,am_addmul_16s_16s_12s_29_4_1_U6_n_11,am_addmul_16s_16s_12s_29_4_1_U6_n_12,am_addmul_16s_16s_12s_29_4_1_U6_n_13,am_addmul_16s_16s_12s_29_4_1_U6_n_14,am_addmul_16s_16s_12s_29_4_1_U6_n_15,am_addmul_16s_16s_12s_29_4_1_U6_n_16,am_addmul_16s_16s_12s_29_4_1_U6_n_17,am_addmul_16s_16s_12s_29_4_1_U6_n_18,am_addmul_16s_16s_12s_29_4_1_U6_n_19,am_addmul_16s_16s_12s_29_4_1_U6_n_20,am_addmul_16s_16s_12s_29_4_1_U6_n_21,am_addmul_16s_16s_12s_29_4_1_U6_n_22,am_addmul_16s_16s_12s_29_4_1_U6_n_23,am_addmul_16s_16s_12s_29_4_1_U6_n_24,am_addmul_16s_16s_12s_29_4_1_U6_n_25,am_addmul_16s_16s_12s_29_4_1_U6_n_26,am_addmul_16s_16s_12s_29_4_1_U6_n_27,am_addmul_16s_16s_12s_29_4_1_U6_n_28,am_addmul_16s_16s_12s_29_4_1_U6_n_29,am_addmul_16s_16s_12s_29_4_1_U6_n_30,am_addmul_16s_16s_12s_29_4_1_U6_n_31,am_addmul_16s_16s_12s_29_4_1_U6_n_32,am_addmul_16s_16s_12s_29_4_1_U6_n_33,am_addmul_16s_16s_12s_29_4_1_U6_n_34,am_addmul_16s_16s_12s_29_4_1_U6_n_35,am_addmul_16s_16s_12s_29_4_1_U6_n_36,am_addmul_16s_16s_12s_29_4_1_U6_n_37,am_addmul_16s_16s_12s_29_4_1_U6_n_38,am_addmul_16s_16s_12s_29_4_1_U6_n_39,am_addmul_16s_16s_12s_29_4_1_U6_n_40,am_addmul_16s_16s_12s_29_4_1_U6_n_41,am_addmul_16s_16s_12s_29_4_1_U6_n_42,am_addmul_16s_16s_12s_29_4_1_U6_n_43,am_addmul_16s_16s_12s_29_4_1_U6_n_44,am_addmul_16s_16s_12s_29_4_1_U6_n_45,am_addmul_16s_16s_12s_29_4_1_U6_n_46,am_addmul_16s_16s_12s_29_4_1_U6_n_47}),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_14ns_31_4_1 am_addmul_16s_16s_14ns_31_4_1_U3
       (.D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15),
        .PCOUT({am_addmul_16s_16s_14ns_31_4_1_U3_n_0,am_addmul_16s_16s_14ns_31_4_1_U3_n_1,am_addmul_16s_16s_14ns_31_4_1_U3_n_2,am_addmul_16s_16s_14ns_31_4_1_U3_n_3,am_addmul_16s_16s_14ns_31_4_1_U3_n_4,am_addmul_16s_16s_14ns_31_4_1_U3_n_5,am_addmul_16s_16s_14ns_31_4_1_U3_n_6,am_addmul_16s_16s_14ns_31_4_1_U3_n_7,am_addmul_16s_16s_14ns_31_4_1_U3_n_8,am_addmul_16s_16s_14ns_31_4_1_U3_n_9,am_addmul_16s_16s_14ns_31_4_1_U3_n_10,am_addmul_16s_16s_14ns_31_4_1_U3_n_11,am_addmul_16s_16s_14ns_31_4_1_U3_n_12,am_addmul_16s_16s_14ns_31_4_1_U3_n_13,am_addmul_16s_16s_14ns_31_4_1_U3_n_14,am_addmul_16s_16s_14ns_31_4_1_U3_n_15,am_addmul_16s_16s_14ns_31_4_1_U3_n_16,am_addmul_16s_16s_14ns_31_4_1_U3_n_17,am_addmul_16s_16s_14ns_31_4_1_U3_n_18,am_addmul_16s_16s_14ns_31_4_1_U3_n_19,am_addmul_16s_16s_14ns_31_4_1_U3_n_20,am_addmul_16s_16s_14ns_31_4_1_U3_n_21,am_addmul_16s_16s_14ns_31_4_1_U3_n_22,am_addmul_16s_16s_14ns_31_4_1_U3_n_23,am_addmul_16s_16s_14ns_31_4_1_U3_n_24,am_addmul_16s_16s_14ns_31_4_1_U3_n_25,am_addmul_16s_16s_14ns_31_4_1_U3_n_26,am_addmul_16s_16s_14ns_31_4_1_U3_n_27,am_addmul_16s_16s_14ns_31_4_1_U3_n_28,am_addmul_16s_16s_14ns_31_4_1_U3_n_29,am_addmul_16s_16s_14ns_31_4_1_U3_n_30,am_addmul_16s_16s_14ns_31_4_1_U3_n_31,am_addmul_16s_16s_14ns_31_4_1_U3_n_32,am_addmul_16s_16s_14ns_31_4_1_U3_n_33,am_addmul_16s_16s_14ns_31_4_1_U3_n_34,am_addmul_16s_16s_14ns_31_4_1_U3_n_35,am_addmul_16s_16s_14ns_31_4_1_U3_n_36,am_addmul_16s_16s_14ns_31_4_1_U3_n_37,am_addmul_16s_16s_14ns_31_4_1_U3_n_38,am_addmul_16s_16s_14ns_31_4_1_U3_n_39,am_addmul_16s_16s_14ns_31_4_1_U3_n_40,am_addmul_16s_16s_14ns_31_4_1_U3_n_41,am_addmul_16s_16s_14ns_31_4_1_U3_n_42,am_addmul_16s_16s_14ns_31_4_1_U3_n_43,am_addmul_16s_16s_14ns_31_4_1_U3_n_44,am_addmul_16s_16s_14ns_31_4_1_U3_n_45,am_addmul_16s_16s_14ns_31_4_1_U3_n_46,am_addmul_16s_16s_14ns_31_4_1_U3_n_47}),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_15ns_33_4_1 am_addmul_16s_16s_15ns_33_4_1_U2
       (.A(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12),
        .ACOUT({am_addmul_16s_16s_15ns_33_4_1_U2_n_0,am_addmul_16s_16s_15ns_33_4_1_U2_n_1,am_addmul_16s_16s_15ns_33_4_1_U2_n_2,am_addmul_16s_16s_15ns_33_4_1_U2_n_3,am_addmul_16s_16s_15ns_33_4_1_U2_n_4,am_addmul_16s_16s_15ns_33_4_1_U2_n_5,am_addmul_16s_16s_15ns_33_4_1_U2_n_6,am_addmul_16s_16s_15ns_33_4_1_U2_n_7,am_addmul_16s_16s_15ns_33_4_1_U2_n_8,am_addmul_16s_16s_15ns_33_4_1_U2_n_9,am_addmul_16s_16s_15ns_33_4_1_U2_n_10,am_addmul_16s_16s_15ns_33_4_1_U2_n_11,am_addmul_16s_16s_15ns_33_4_1_U2_n_12,am_addmul_16s_16s_15ns_33_4_1_U2_n_13,am_addmul_16s_16s_15ns_33_4_1_U2_n_14,am_addmul_16s_16s_15ns_33_4_1_U2_n_15,am_addmul_16s_16s_15ns_33_4_1_U2_n_16,am_addmul_16s_16s_15ns_33_4_1_U2_n_17,am_addmul_16s_16s_15ns_33_4_1_U2_n_18,am_addmul_16s_16s_15ns_33_4_1_U2_n_19,am_addmul_16s_16s_15ns_33_4_1_U2_n_20,am_addmul_16s_16s_15ns_33_4_1_U2_n_21,am_addmul_16s_16s_15ns_33_4_1_U2_n_22,am_addmul_16s_16s_15ns_33_4_1_U2_n_23,am_addmul_16s_16s_15ns_33_4_1_U2_n_24,am_addmul_16s_16s_15ns_33_4_1_U2_n_25,am_addmul_16s_16s_15ns_33_4_1_U2_n_26,am_addmul_16s_16s_15ns_33_4_1_U2_n_27,am_addmul_16s_16s_15ns_33_4_1_U2_n_28,am_addmul_16s_16s_15ns_33_4_1_U2_n_29}),
        .PCOUT({am_addmul_16s_16s_15ns_33_4_1_U2_n_30,am_addmul_16s_16s_15ns_33_4_1_U2_n_31,am_addmul_16s_16s_15ns_33_4_1_U2_n_32,am_addmul_16s_16s_15ns_33_4_1_U2_n_33,am_addmul_16s_16s_15ns_33_4_1_U2_n_34,am_addmul_16s_16s_15ns_33_4_1_U2_n_35,am_addmul_16s_16s_15ns_33_4_1_U2_n_36,am_addmul_16s_16s_15ns_33_4_1_U2_n_37,am_addmul_16s_16s_15ns_33_4_1_U2_n_38,am_addmul_16s_16s_15ns_33_4_1_U2_n_39,am_addmul_16s_16s_15ns_33_4_1_U2_n_40,am_addmul_16s_16s_15ns_33_4_1_U2_n_41,am_addmul_16s_16s_15ns_33_4_1_U2_n_42,am_addmul_16s_16s_15ns_33_4_1_U2_n_43,am_addmul_16s_16s_15ns_33_4_1_U2_n_44,am_addmul_16s_16s_15ns_33_4_1_U2_n_45,am_addmul_16s_16s_15ns_33_4_1_U2_n_46,am_addmul_16s_16s_15ns_33_4_1_U2_n_47,am_addmul_16s_16s_15ns_33_4_1_U2_n_48,am_addmul_16s_16s_15ns_33_4_1_U2_n_49,am_addmul_16s_16s_15ns_33_4_1_U2_n_50,am_addmul_16s_16s_15ns_33_4_1_U2_n_51,am_addmul_16s_16s_15ns_33_4_1_U2_n_52,am_addmul_16s_16s_15ns_33_4_1_U2_n_53,am_addmul_16s_16s_15ns_33_4_1_U2_n_54,am_addmul_16s_16s_15ns_33_4_1_U2_n_55,am_addmul_16s_16s_15ns_33_4_1_U2_n_56,am_addmul_16s_16s_15ns_33_4_1_U2_n_57,am_addmul_16s_16s_15ns_33_4_1_U2_n_58,am_addmul_16s_16s_15ns_33_4_1_U2_n_59,am_addmul_16s_16s_15ns_33_4_1_U2_n_60,am_addmul_16s_16s_15ns_33_4_1_U2_n_61,am_addmul_16s_16s_15ns_33_4_1_U2_n_62,am_addmul_16s_16s_15ns_33_4_1_U2_n_63,am_addmul_16s_16s_15ns_33_4_1_U2_n_64,am_addmul_16s_16s_15ns_33_4_1_U2_n_65,am_addmul_16s_16s_15ns_33_4_1_U2_n_66,am_addmul_16s_16s_15ns_33_4_1_U2_n_67,am_addmul_16s_16s_15ns_33_4_1_U2_n_68,am_addmul_16s_16s_15ns_33_4_1_U2_n_69,am_addmul_16s_16s_15ns_33_4_1_U2_n_70,am_addmul_16s_16s_15ns_33_4_1_U2_n_71,am_addmul_16s_16s_15ns_33_4_1_U2_n_72,am_addmul_16s_16s_15ns_33_4_1_U2_n_73,am_addmul_16s_16s_15ns_33_4_1_U2_n_74,am_addmul_16s_16s_15ns_33_4_1_U2_n_75,am_addmul_16s_16s_15ns_33_4_1_U2_n_76,am_addmul_16s_16s_15ns_33_4_1_U2_n_77}),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_10s_28s_28_4_1 ama_addmuladd_16s_16s_10s_28s_28_4_1_U14
       (.D(add_ln131_11_fu_652_p2),
        .DI(ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_27),
        .P(ama_addmuladd_16s_16s_10s_28s_28_4_1_U14_n_0),
        .PCOUT({am_addmul_16s_16s_11s_28_4_1_U7_n_0,am_addmul_16s_16s_11s_28_4_1_U7_n_1,am_addmul_16s_16s_11s_28_4_1_U7_n_2,am_addmul_16s_16s_11s_28_4_1_U7_n_3,am_addmul_16s_16s_11s_28_4_1_U7_n_4,am_addmul_16s_16s_11s_28_4_1_U7_n_5,am_addmul_16s_16s_11s_28_4_1_U7_n_6,am_addmul_16s_16s_11s_28_4_1_U7_n_7,am_addmul_16s_16s_11s_28_4_1_U7_n_8,am_addmul_16s_16s_11s_28_4_1_U7_n_9,am_addmul_16s_16s_11s_28_4_1_U7_n_10,am_addmul_16s_16s_11s_28_4_1_U7_n_11,am_addmul_16s_16s_11s_28_4_1_U7_n_12,am_addmul_16s_16s_11s_28_4_1_U7_n_13,am_addmul_16s_16s_11s_28_4_1_U7_n_14,am_addmul_16s_16s_11s_28_4_1_U7_n_15,am_addmul_16s_16s_11s_28_4_1_U7_n_16,am_addmul_16s_16s_11s_28_4_1_U7_n_17,am_addmul_16s_16s_11s_28_4_1_U7_n_18,am_addmul_16s_16s_11s_28_4_1_U7_n_19,am_addmul_16s_16s_11s_28_4_1_U7_n_20,am_addmul_16s_16s_11s_28_4_1_U7_n_21,am_addmul_16s_16s_11s_28_4_1_U7_n_22,am_addmul_16s_16s_11s_28_4_1_U7_n_23,am_addmul_16s_16s_11s_28_4_1_U7_n_24,am_addmul_16s_16s_11s_28_4_1_U7_n_25,am_addmul_16s_16s_11s_28_4_1_U7_n_26,am_addmul_16s_16s_11s_28_4_1_U7_n_27,am_addmul_16s_16s_11s_28_4_1_U7_n_28,am_addmul_16s_16s_11s_28_4_1_U7_n_29,am_addmul_16s_16s_11s_28_4_1_U7_n_30,am_addmul_16s_16s_11s_28_4_1_U7_n_31,am_addmul_16s_16s_11s_28_4_1_U7_n_32,am_addmul_16s_16s_11s_28_4_1_U7_n_33,am_addmul_16s_16s_11s_28_4_1_U7_n_34,am_addmul_16s_16s_11s_28_4_1_U7_n_35,am_addmul_16s_16s_11s_28_4_1_U7_n_36,am_addmul_16s_16s_11s_28_4_1_U7_n_37,am_addmul_16s_16s_11s_28_4_1_U7_n_38,am_addmul_16s_16s_11s_28_4_1_U7_n_39,am_addmul_16s_16s_11s_28_4_1_U7_n_40,am_addmul_16s_16s_11s_28_4_1_U7_n_41,am_addmul_16s_16s_11s_28_4_1_U7_n_42,am_addmul_16s_16s_11s_28_4_1_U7_n_43,am_addmul_16s_16s_11s_28_4_1_U7_n_44,am_addmul_16s_16s_11s_28_4_1_U7_n_45,am_addmul_16s_16s_11s_28_4_1_U7_n_46,am_addmul_16s_16s_11s_28_4_1_U7_n_47}),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9),
        .S(ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_29),
        .\add_ln131_11_reg_1040_reg[27] ({ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_0,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_1,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_2,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_3,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_4,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_5,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_6,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_7,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_8,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_9,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_10,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_11,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_12,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_13,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_14,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_15,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_16,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_17,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_18,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_19,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_20,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_21,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_22,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_23,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_24,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_25,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_26}),
        .\add_ln131_11_reg_1040_reg[28] (ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_28),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1 ama_addmuladd_16s_16s_12s_29s_29_4_1_U12
       (.D({ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_0,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_1,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_2,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_3,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_4,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_5,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_6,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_7,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_8,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_9,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_10,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_11,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_12,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_13,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_14,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_15,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_16,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_17,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_18,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_19,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_20,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_21,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_22,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_23,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_24,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_25,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_26,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_27,ama_addmuladd_16s_16s_12s_29s_29_4_1_U12_n_28}),
        .PCOUT({am_addmul_16s_16s_12s_29_4_1_U5_n_0,am_addmul_16s_16s_12s_29_4_1_U5_n_1,am_addmul_16s_16s_12s_29_4_1_U5_n_2,am_addmul_16s_16s_12s_29_4_1_U5_n_3,am_addmul_16s_16s_12s_29_4_1_U5_n_4,am_addmul_16s_16s_12s_29_4_1_U5_n_5,am_addmul_16s_16s_12s_29_4_1_U5_n_6,am_addmul_16s_16s_12s_29_4_1_U5_n_7,am_addmul_16s_16s_12s_29_4_1_U5_n_8,am_addmul_16s_16s_12s_29_4_1_U5_n_9,am_addmul_16s_16s_12s_29_4_1_U5_n_10,am_addmul_16s_16s_12s_29_4_1_U5_n_11,am_addmul_16s_16s_12s_29_4_1_U5_n_12,am_addmul_16s_16s_12s_29_4_1_U5_n_13,am_addmul_16s_16s_12s_29_4_1_U5_n_14,am_addmul_16s_16s_12s_29_4_1_U5_n_15,am_addmul_16s_16s_12s_29_4_1_U5_n_16,am_addmul_16s_16s_12s_29_4_1_U5_n_17,am_addmul_16s_16s_12s_29_4_1_U5_n_18,am_addmul_16s_16s_12s_29_4_1_U5_n_19,am_addmul_16s_16s_12s_29_4_1_U5_n_20,am_addmul_16s_16s_12s_29_4_1_U5_n_21,am_addmul_16s_16s_12s_29_4_1_U5_n_22,am_addmul_16s_16s_12s_29_4_1_U5_n_23,am_addmul_16s_16s_12s_29_4_1_U5_n_24,am_addmul_16s_16s_12s_29_4_1_U5_n_25,am_addmul_16s_16s_12s_29_4_1_U5_n_26,am_addmul_16s_16s_12s_29_4_1_U5_n_27,am_addmul_16s_16s_12s_29_4_1_U5_n_28,am_addmul_16s_16s_12s_29_4_1_U5_n_29,am_addmul_16s_16s_12s_29_4_1_U5_n_30,am_addmul_16s_16s_12s_29_4_1_U5_n_31,am_addmul_16s_16s_12s_29_4_1_U5_n_32,am_addmul_16s_16s_12s_29_4_1_U5_n_33,am_addmul_16s_16s_12s_29_4_1_U5_n_34,am_addmul_16s_16s_12s_29_4_1_U5_n_35,am_addmul_16s_16s_12s_29_4_1_U5_n_36,am_addmul_16s_16s_12s_29_4_1_U5_n_37,am_addmul_16s_16s_12s_29_4_1_U5_n_38,am_addmul_16s_16s_12s_29_4_1_U5_n_39,am_addmul_16s_16s_12s_29_4_1_U5_n_40,am_addmul_16s_16s_12s_29_4_1_U5_n_41,am_addmul_16s_16s_12s_29_4_1_U5_n_42,am_addmul_16s_16s_12s_29_4_1_U5_n_43,am_addmul_16s_16s_12s_29_4_1_U5_n_44,am_addmul_16s_16s_12s_29_4_1_U5_n_45,am_addmul_16s_16s_12s_29_4_1_U5_n_46,am_addmul_16s_16s_12s_29_4_1_U5_n_47}),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_1 ama_addmuladd_16s_16s_12s_29s_29_4_1_U13
       (.DI(ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_27),
        .P(ama_addmuladd_16s_16s_10s_28s_28_4_1_U14_n_0),
        .PCOUT({am_addmul_16s_16s_12s_29_4_1_U6_n_0,am_addmul_16s_16s_12s_29_4_1_U6_n_1,am_addmul_16s_16s_12s_29_4_1_U6_n_2,am_addmul_16s_16s_12s_29_4_1_U6_n_3,am_addmul_16s_16s_12s_29_4_1_U6_n_4,am_addmul_16s_16s_12s_29_4_1_U6_n_5,am_addmul_16s_16s_12s_29_4_1_U6_n_6,am_addmul_16s_16s_12s_29_4_1_U6_n_7,am_addmul_16s_16s_12s_29_4_1_U6_n_8,am_addmul_16s_16s_12s_29_4_1_U6_n_9,am_addmul_16s_16s_12s_29_4_1_U6_n_10,am_addmul_16s_16s_12s_29_4_1_U6_n_11,am_addmul_16s_16s_12s_29_4_1_U6_n_12,am_addmul_16s_16s_12s_29_4_1_U6_n_13,am_addmul_16s_16s_12s_29_4_1_U6_n_14,am_addmul_16s_16s_12s_29_4_1_U6_n_15,am_addmul_16s_16s_12s_29_4_1_U6_n_16,am_addmul_16s_16s_12s_29_4_1_U6_n_17,am_addmul_16s_16s_12s_29_4_1_U6_n_18,am_addmul_16s_16s_12s_29_4_1_U6_n_19,am_addmul_16s_16s_12s_29_4_1_U6_n_20,am_addmul_16s_16s_12s_29_4_1_U6_n_21,am_addmul_16s_16s_12s_29_4_1_U6_n_22,am_addmul_16s_16s_12s_29_4_1_U6_n_23,am_addmul_16s_16s_12s_29_4_1_U6_n_24,am_addmul_16s_16s_12s_29_4_1_U6_n_25,am_addmul_16s_16s_12s_29_4_1_U6_n_26,am_addmul_16s_16s_12s_29_4_1_U6_n_27,am_addmul_16s_16s_12s_29_4_1_U6_n_28,am_addmul_16s_16s_12s_29_4_1_U6_n_29,am_addmul_16s_16s_12s_29_4_1_U6_n_30,am_addmul_16s_16s_12s_29_4_1_U6_n_31,am_addmul_16s_16s_12s_29_4_1_U6_n_32,am_addmul_16s_16s_12s_29_4_1_U6_n_33,am_addmul_16s_16s_12s_29_4_1_U6_n_34,am_addmul_16s_16s_12s_29_4_1_U6_n_35,am_addmul_16s_16s_12s_29_4_1_U6_n_36,am_addmul_16s_16s_12s_29_4_1_U6_n_37,am_addmul_16s_16s_12s_29_4_1_U6_n_38,am_addmul_16s_16s_12s_29_4_1_U6_n_39,am_addmul_16s_16s_12s_29_4_1_U6_n_40,am_addmul_16s_16s_12s_29_4_1_U6_n_41,am_addmul_16s_16s_12s_29_4_1_U6_n_42,am_addmul_16s_16s_12s_29_4_1_U6_n_43,am_addmul_16s_16s_12s_29_4_1_U6_n_44,am_addmul_16s_16s_12s_29_4_1_U6_n_45,am_addmul_16s_16s_12s_29_4_1_U6_n_46,am_addmul_16s_16s_12s_29_4_1_U6_n_47}),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7),
        .S(ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_29),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg({ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_0,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_1,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_2,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_3,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_4,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_5,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_6,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_7,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_8,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_9,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_10,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_11,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_12,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_13,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_14,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_15,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_16,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_17,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_18,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_19,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_20,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_21,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_22,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_23,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_24,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_25,ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_26}),
        .p_reg_reg_0(ama_addmuladd_16s_16s_12s_29s_29_4_1_U13_n_28),
        .p_reg_reg_1(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_13ns_31s_31_4_1 ama_addmuladd_16s_16s_13ns_31s_31_4_1_U10
       (.D(add_ln131_4_fu_644_p2),
        .DI(ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_31),
        .P(ama_addmuladd_16s_16s_13ns_31s_31_4_1_U10_n_0),
        .PCOUT({am_addmul_16s_16s_14ns_31_4_1_U3_n_0,am_addmul_16s_16s_14ns_31_4_1_U3_n_1,am_addmul_16s_16s_14ns_31_4_1_U3_n_2,am_addmul_16s_16s_14ns_31_4_1_U3_n_3,am_addmul_16s_16s_14ns_31_4_1_U3_n_4,am_addmul_16s_16s_14ns_31_4_1_U3_n_5,am_addmul_16s_16s_14ns_31_4_1_U3_n_6,am_addmul_16s_16s_14ns_31_4_1_U3_n_7,am_addmul_16s_16s_14ns_31_4_1_U3_n_8,am_addmul_16s_16s_14ns_31_4_1_U3_n_9,am_addmul_16s_16s_14ns_31_4_1_U3_n_10,am_addmul_16s_16s_14ns_31_4_1_U3_n_11,am_addmul_16s_16s_14ns_31_4_1_U3_n_12,am_addmul_16s_16s_14ns_31_4_1_U3_n_13,am_addmul_16s_16s_14ns_31_4_1_U3_n_14,am_addmul_16s_16s_14ns_31_4_1_U3_n_15,am_addmul_16s_16s_14ns_31_4_1_U3_n_16,am_addmul_16s_16s_14ns_31_4_1_U3_n_17,am_addmul_16s_16s_14ns_31_4_1_U3_n_18,am_addmul_16s_16s_14ns_31_4_1_U3_n_19,am_addmul_16s_16s_14ns_31_4_1_U3_n_20,am_addmul_16s_16s_14ns_31_4_1_U3_n_21,am_addmul_16s_16s_14ns_31_4_1_U3_n_22,am_addmul_16s_16s_14ns_31_4_1_U3_n_23,am_addmul_16s_16s_14ns_31_4_1_U3_n_24,am_addmul_16s_16s_14ns_31_4_1_U3_n_25,am_addmul_16s_16s_14ns_31_4_1_U3_n_26,am_addmul_16s_16s_14ns_31_4_1_U3_n_27,am_addmul_16s_16s_14ns_31_4_1_U3_n_28,am_addmul_16s_16s_14ns_31_4_1_U3_n_29,am_addmul_16s_16s_14ns_31_4_1_U3_n_30,am_addmul_16s_16s_14ns_31_4_1_U3_n_31,am_addmul_16s_16s_14ns_31_4_1_U3_n_32,am_addmul_16s_16s_14ns_31_4_1_U3_n_33,am_addmul_16s_16s_14ns_31_4_1_U3_n_34,am_addmul_16s_16s_14ns_31_4_1_U3_n_35,am_addmul_16s_16s_14ns_31_4_1_U3_n_36,am_addmul_16s_16s_14ns_31_4_1_U3_n_37,am_addmul_16s_16s_14ns_31_4_1_U3_n_38,am_addmul_16s_16s_14ns_31_4_1_U3_n_39,am_addmul_16s_16s_14ns_31_4_1_U3_n_40,am_addmul_16s_16s_14ns_31_4_1_U3_n_41,am_addmul_16s_16s_14ns_31_4_1_U3_n_42,am_addmul_16s_16s_14ns_31_4_1_U3_n_43,am_addmul_16s_16s_14ns_31_4_1_U3_n_44,am_addmul_16s_16s_14ns_31_4_1_U3_n_45,am_addmul_16s_16s_14ns_31_4_1_U3_n_46,am_addmul_16s_16s_14ns_31_4_1_U3_n_47}),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1),
        .S({ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_32,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_33}),
        .\add_ln131_4_reg_1025_reg[31] ({ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_0,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_1,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_2,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_3,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_4,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_5,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_6,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_7,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_8,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_9,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_10,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_11,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_12,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_13,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_14,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_15,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_16,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_17,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_18,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_19,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_20,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_21,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_22,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_23,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_24,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_25,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_26,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_27,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_28,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_29,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_30}),
        .\add_ln131_4_reg_1025_reg[32] (ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_34),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_14ns_33s_33_4_1 ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9
       (.ACOUT({am_addmul_16s_16s_15ns_33_4_1_U2_n_0,am_addmul_16s_16s_15ns_33_4_1_U2_n_1,am_addmul_16s_16s_15ns_33_4_1_U2_n_2,am_addmul_16s_16s_15ns_33_4_1_U2_n_3,am_addmul_16s_16s_15ns_33_4_1_U2_n_4,am_addmul_16s_16s_15ns_33_4_1_U2_n_5,am_addmul_16s_16s_15ns_33_4_1_U2_n_6,am_addmul_16s_16s_15ns_33_4_1_U2_n_7,am_addmul_16s_16s_15ns_33_4_1_U2_n_8,am_addmul_16s_16s_15ns_33_4_1_U2_n_9,am_addmul_16s_16s_15ns_33_4_1_U2_n_10,am_addmul_16s_16s_15ns_33_4_1_U2_n_11,am_addmul_16s_16s_15ns_33_4_1_U2_n_12,am_addmul_16s_16s_15ns_33_4_1_U2_n_13,am_addmul_16s_16s_15ns_33_4_1_U2_n_14,am_addmul_16s_16s_15ns_33_4_1_U2_n_15,am_addmul_16s_16s_15ns_33_4_1_U2_n_16,am_addmul_16s_16s_15ns_33_4_1_U2_n_17,am_addmul_16s_16s_15ns_33_4_1_U2_n_18,am_addmul_16s_16s_15ns_33_4_1_U2_n_19,am_addmul_16s_16s_15ns_33_4_1_U2_n_20,am_addmul_16s_16s_15ns_33_4_1_U2_n_21,am_addmul_16s_16s_15ns_33_4_1_U2_n_22,am_addmul_16s_16s_15ns_33_4_1_U2_n_23,am_addmul_16s_16s_15ns_33_4_1_U2_n_24,am_addmul_16s_16s_15ns_33_4_1_U2_n_25,am_addmul_16s_16s_15ns_33_4_1_U2_n_26,am_addmul_16s_16s_15ns_33_4_1_U2_n_27,am_addmul_16s_16s_15ns_33_4_1_U2_n_28,am_addmul_16s_16s_15ns_33_4_1_U2_n_29}),
        .DI(ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_31),
        .P(ama_addmuladd_16s_16s_13ns_31s_31_4_1_U10_n_0),
        .PCOUT({am_addmul_16s_16s_15ns_33_4_1_U2_n_30,am_addmul_16s_16s_15ns_33_4_1_U2_n_31,am_addmul_16s_16s_15ns_33_4_1_U2_n_32,am_addmul_16s_16s_15ns_33_4_1_U2_n_33,am_addmul_16s_16s_15ns_33_4_1_U2_n_34,am_addmul_16s_16s_15ns_33_4_1_U2_n_35,am_addmul_16s_16s_15ns_33_4_1_U2_n_36,am_addmul_16s_16s_15ns_33_4_1_U2_n_37,am_addmul_16s_16s_15ns_33_4_1_U2_n_38,am_addmul_16s_16s_15ns_33_4_1_U2_n_39,am_addmul_16s_16s_15ns_33_4_1_U2_n_40,am_addmul_16s_16s_15ns_33_4_1_U2_n_41,am_addmul_16s_16s_15ns_33_4_1_U2_n_42,am_addmul_16s_16s_15ns_33_4_1_U2_n_43,am_addmul_16s_16s_15ns_33_4_1_U2_n_44,am_addmul_16s_16s_15ns_33_4_1_U2_n_45,am_addmul_16s_16s_15ns_33_4_1_U2_n_46,am_addmul_16s_16s_15ns_33_4_1_U2_n_47,am_addmul_16s_16s_15ns_33_4_1_U2_n_48,am_addmul_16s_16s_15ns_33_4_1_U2_n_49,am_addmul_16s_16s_15ns_33_4_1_U2_n_50,am_addmul_16s_16s_15ns_33_4_1_U2_n_51,am_addmul_16s_16s_15ns_33_4_1_U2_n_52,am_addmul_16s_16s_15ns_33_4_1_U2_n_53,am_addmul_16s_16s_15ns_33_4_1_U2_n_54,am_addmul_16s_16s_15ns_33_4_1_U2_n_55,am_addmul_16s_16s_15ns_33_4_1_U2_n_56,am_addmul_16s_16s_15ns_33_4_1_U2_n_57,am_addmul_16s_16s_15ns_33_4_1_U2_n_58,am_addmul_16s_16s_15ns_33_4_1_U2_n_59,am_addmul_16s_16s_15ns_33_4_1_U2_n_60,am_addmul_16s_16s_15ns_33_4_1_U2_n_61,am_addmul_16s_16s_15ns_33_4_1_U2_n_62,am_addmul_16s_16s_15ns_33_4_1_U2_n_63,am_addmul_16s_16s_15ns_33_4_1_U2_n_64,am_addmul_16s_16s_15ns_33_4_1_U2_n_65,am_addmul_16s_16s_15ns_33_4_1_U2_n_66,am_addmul_16s_16s_15ns_33_4_1_U2_n_67,am_addmul_16s_16s_15ns_33_4_1_U2_n_68,am_addmul_16s_16s_15ns_33_4_1_U2_n_69,am_addmul_16s_16s_15ns_33_4_1_U2_n_70,am_addmul_16s_16s_15ns_33_4_1_U2_n_71,am_addmul_16s_16s_15ns_33_4_1_U2_n_72,am_addmul_16s_16s_15ns_33_4_1_U2_n_73,am_addmul_16s_16s_15ns_33_4_1_U2_n_74,am_addmul_16s_16s_15ns_33_4_1_U2_n_75,am_addmul_16s_16s_15ns_33_4_1_U2_n_76,am_addmul_16s_16s_15ns_33_4_1_U2_n_77}),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8),
        .S({ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_32,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_33}),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg({ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_0,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_1,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_2,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_3,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_4,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_5,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_6,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_7,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_8,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_9,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_10,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_11,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_12,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_13,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_14,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_15,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_16,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_17,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_18,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_19,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_20,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_21,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_22,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_23,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_24,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_25,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_26,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_27,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_28,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_29,ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_30}),
        .p_reg_reg_0(ama_addmuladd_16s_16s_14ns_33s_33_4_1_U9_n_34));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_15ns_25s_32_4_1 ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1
       (.A(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12),
        .ACOUT({ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_0,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_1,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_2,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_3,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_4,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_5,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_6,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_7,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_8,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_9,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_10,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_11,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_12,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_13,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_14,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_15,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_16,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_17,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_18,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_19,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_20,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_21,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_22,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_23,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_24,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_25,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_26,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_27,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_28,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_29}),
        .P({ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_30,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_31,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_32,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_33,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_34,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_35,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_36,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_37,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_38,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_39,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_40,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_41,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_42,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_43,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_44,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_45,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_46,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_47,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_48,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_49,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_50,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_51,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_52,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_53,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_54,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_55,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_56,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_57,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_58,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_59,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_60,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_61}),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg(p_shl1_cast_fu_624_p1));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_15ns_32s_33_4_1 ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8
       (.A(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12),
        .ACOUT({ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_0,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_1,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_2,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_3,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_4,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_5,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_6,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_7,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_8,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_9,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_10,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_11,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_12,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_13,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_14,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_15,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_16,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_17,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_18,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_19,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_20,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_21,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_22,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_23,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_24,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_25,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_26,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_27,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_28,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_29}),
        .D({ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_0,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_1,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_2,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_3,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_4,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_5,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_6,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_7,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_8,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_9,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_10,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_11,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_12,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_13,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_14,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_15,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_16,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_17,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_18,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_19,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_20,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_21,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_22,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_23,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_24,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_25,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_26,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_27,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_28,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_29,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_30,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_31,ama_addmuladd_16s_16s_15ns_32s_33_4_1_U8_n_32}),
        .P({ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_30,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_31,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_32,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_33,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_34,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_35,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_36,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_37,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_38,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_39,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_40,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_41,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_42,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_43,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_44,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_45,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_46,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_47,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_48,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_49,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_50,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_51,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_52,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_53,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_54,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_55,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_56,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_57,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_58,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_59,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_60,ama_addmuladd_16s_16s_15ns_25s_32_4_1_U1_n_61}),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_7s_28s_28_4_1 ama_addmuladd_16s_16s_7s_28s_28_4_1_U11
       (.D({ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_0,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_1,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_2,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_3,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_4,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_5,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_6,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_7,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_8,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_9,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_10,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_11,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_12,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_13,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_14,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_15,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_16,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_17,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_18,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_19,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_20,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_21,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_22,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_23,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_24,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_25,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_26,ama_addmuladd_16s_16s_7s_28s_28_4_1_U11_n_27}),
        .PCOUT({am_addmul_16s_16s_11ns_28_4_1_U4_n_0,am_addmul_16s_16s_11ns_28_4_1_U4_n_1,am_addmul_16s_16s_11ns_28_4_1_U4_n_2,am_addmul_16s_16s_11ns_28_4_1_U4_n_3,am_addmul_16s_16s_11ns_28_4_1_U4_n_4,am_addmul_16s_16s_11ns_28_4_1_U4_n_5,am_addmul_16s_16s_11ns_28_4_1_U4_n_6,am_addmul_16s_16s_11ns_28_4_1_U4_n_7,am_addmul_16s_16s_11ns_28_4_1_U4_n_8,am_addmul_16s_16s_11ns_28_4_1_U4_n_9,am_addmul_16s_16s_11ns_28_4_1_U4_n_10,am_addmul_16s_16s_11ns_28_4_1_U4_n_11,am_addmul_16s_16s_11ns_28_4_1_U4_n_12,am_addmul_16s_16s_11ns_28_4_1_U4_n_13,am_addmul_16s_16s_11ns_28_4_1_U4_n_14,am_addmul_16s_16s_11ns_28_4_1_U4_n_15,am_addmul_16s_16s_11ns_28_4_1_U4_n_16,am_addmul_16s_16s_11ns_28_4_1_U4_n_17,am_addmul_16s_16s_11ns_28_4_1_U4_n_18,am_addmul_16s_16s_11ns_28_4_1_U4_n_19,am_addmul_16s_16s_11ns_28_4_1_U4_n_20,am_addmul_16s_16s_11ns_28_4_1_U4_n_21,am_addmul_16s_16s_11ns_28_4_1_U4_n_22,am_addmul_16s_16s_11ns_28_4_1_U4_n_23,am_addmul_16s_16s_11ns_28_4_1_U4_n_24,am_addmul_16s_16s_11ns_28_4_1_U4_n_25,am_addmul_16s_16s_11ns_28_4_1_U4_n_26,am_addmul_16s_16s_11ns_28_4_1_U4_n_27,am_addmul_16s_16s_11ns_28_4_1_U4_n_28,am_addmul_16s_16s_11ns_28_4_1_U4_n_29,am_addmul_16s_16s_11ns_28_4_1_U4_n_30,am_addmul_16s_16s_11ns_28_4_1_U4_n_31,am_addmul_16s_16s_11ns_28_4_1_U4_n_32,am_addmul_16s_16s_11ns_28_4_1_U4_n_33,am_addmul_16s_16s_11ns_28_4_1_U4_n_34,am_addmul_16s_16s_11ns_28_4_1_U4_n_35,am_addmul_16s_16s_11ns_28_4_1_U4_n_36,am_addmul_16s_16s_11ns_28_4_1_U4_n_37,am_addmul_16s_16s_11ns_28_4_1_U4_n_38,am_addmul_16s_16s_11ns_28_4_1_U4_n_39,am_addmul_16s_16s_11ns_28_4_1_U4_n_40,am_addmul_16s_16s_11ns_28_4_1_U4_n_41,am_addmul_16s_16s_11ns_28_4_1_U4_n_42,am_addmul_16s_16s_11ns_28_4_1_U4_n_43,am_addmul_16s_16s_11ns_28_4_1_U4_n_44,am_addmul_16s_16s_11ns_28_4_1_U4_n_45,am_addmul_16s_16s_11ns_28_4_1_U4_n_46,am_addmul_16s_16s_11ns_28_4_1_U4_n_47}),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay));
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
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter5_reg
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(ap_enable_reg_pp0_iter4),
        .Q(ap_enable_reg_pp0_iter5),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter6_reg
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(ap_enable_reg_pp0_iter5),
        .Q(ap_enable_reg_pp0_iter6),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter7_reg
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(ap_enable_reg_pp0_iter6),
        .Q(ap_enable_reg_pp0_iter7),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter8_reg
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(ap_enable_reg_pp0_iter7),
        .Q(ap_enable_reg_pp0_iter8),
        .R(ap_rst_n_inv));
  FDRE \din_data_reg_893_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[0]),
        .Q(din_data_reg_893[0]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[10]),
        .Q(din_data_reg_893[10]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[11]),
        .Q(din_data_reg_893[11]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[12]),
        .Q(din_data_reg_893[12]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[13]),
        .Q(din_data_reg_893[13]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[14]),
        .Q(din_data_reg_893[14]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[15]),
        .Q(din_data_reg_893[15]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[1]),
        .Q(din_data_reg_893[1]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[2]),
        .Q(din_data_reg_893[2]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[3]),
        .Q(din_data_reg_893[3]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[4]),
        .Q(din_data_reg_893[4]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[5]),
        .Q(din_data_reg_893[5]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[6]),
        .Q(din_data_reg_893[6]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[7]),
        .Q(din_data_reg_893[7]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[8]),
        .Q(din_data_reg_893[8]),
        .R(1'b0));
  FDRE \din_data_reg_893_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[9]),
        .Q(din_data_reg_893[9]),
        .R(1'b0));
  (* srl_bus_name = "inst/\\din_last_reg_898_pp0_iter5_reg_reg " *) 
  (* srl_name = "inst/\\din_last_reg_898_pp0_iter5_reg_reg[0]_srl6 " *) 
  SRL16E \din_last_reg_898_pp0_iter5_reg_reg[0]_srl6 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b1),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_r_TLAST_int_regslice),
        .Q(\din_last_reg_898_pp0_iter5_reg_reg[0]_srl6_n_0 ));
  FDRE \din_last_reg_898_pp0_iter6_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\din_last_reg_898_pp0_iter5_reg_reg[0]_srl6_n_0 ),
        .Q(din_last_reg_898_pp0_iter6_reg),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[9]),
        .R(1'b0));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[0]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[0]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[0]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[0]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[10]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[10]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[10]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[10]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[11]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[11]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[11]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[11]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[12]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[12]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[12]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[12]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[13]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[13]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[13]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[13]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[14]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[14]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[14]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[14]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[15]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[15]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[15]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[15]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[1]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[1]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[1]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[1]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[2]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[2]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[2]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[2]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[3]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[3]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[3]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[3]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[4]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[4]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[4]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[4]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[5]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[5]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[5]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[5]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[6]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[6]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[6]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[6]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[7]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[7]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[7]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[7]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[8]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[8]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[8]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[8]_srl3_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[9]_srl3 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[9]_srl3 
       (.A0(1'b0),
        .A1(1'b1),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[9]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[9]_srl3_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[0]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[10]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[11]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[12]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[13]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[14]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[15]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[1]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[2]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[3]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[4]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[5]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[6]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[7]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[8]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_11_reg[9]_srl3_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[9]),
        .R(1'b0));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[0]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[0]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[0]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[0]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[10]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[10]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[10]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[10]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[11]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[11]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[11]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[11]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[12]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[12]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[12]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[12]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[13]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[13]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[13]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[13]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[14]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[14]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[14]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[14]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[15]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[15]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[15]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[15]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[1]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[1]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[1]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[1]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[2]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[2]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[2]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[2]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[3]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[3]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[3]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[3]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[4]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[4]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[4]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[4]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[5]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[5]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[5]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[5]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[6]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[6]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[6]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[6]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[7]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[7]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[7]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[7]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[8]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[8]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[8]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[8]_srl2_n_0 ));
  (* srl_bus_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg " *) 
  (* srl_name = "inst/\\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[9]_srl2 " *) 
  SRL16E #(
    .INIT(16'h0000)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[9]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_12[9]),
        .Q(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[9]_srl2_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[0]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[10]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[11]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[12]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[13]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[14]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[15]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[1]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[2]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[3]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[4]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[5]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[6]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[7]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[8]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_14_reg[9]_srl2_n_0 ),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[9]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[0]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[10]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[11]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[12]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[13]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[14]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[15]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[1]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[2]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[3]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[4]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[5]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[6]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[7]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[8]),
        .R(1'b0));
  FDRE \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_32_reg_909_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_34_reg_914_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_36_reg_919_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_0[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_38_reg_924_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_17[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_40_reg_929_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_15[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_16[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[0] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[10] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[11] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[12] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[13] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[14] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[15] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[1] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[2] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[3] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[4] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[5] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[6] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[7] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[8] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[9] 
       (.C(ap_clk),
        .CE(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .D(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_30_reg_903[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[0]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[10]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[11]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[12]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[13]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[14]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[15]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[1]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[2]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[3]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[4]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[5]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[6]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[7]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[8]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[9]),
        .Q(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_load_reg_949_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_1[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_load_reg_954_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_2[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_3[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_load_reg_959_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_4[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_5[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_load_reg_964_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_6[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_load_reg_969_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_8[9]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[0]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[10]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[10]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[11]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[11]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[12]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[12]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[13]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[13]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[14]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[14]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[15]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[15]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[1]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[2]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[2]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[3]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[3]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[4]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[4]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[5]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[5]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[6]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[6]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[7]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[7]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[8]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[8]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(in_r_TDATA_int_regslice[9]),
        .Q(fsk_lpf_stream_axis_0_stream_axis_ap_int_16_0ul_0ul_0ul_0_delay_9[9]),
        .R(1'b0));
  LUT3 #(
    .INIT(8'h0E)) 
    \icmp_ln139_reg_1055[0]_i_1 
       (.I0(tmp_1_fu_709_p4__0[1]),
        .I1(tmp_1_fu_709_p4),
        .I2(tmp_1_fu_709_p4__0[2]),
        .O(\icmp_ln139_reg_1055[0]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hE00E)) 
    \icmp_ln139_reg_1055[0]_i_3 
       (.I0(add_ln131_1_reg_1020_pp0_iter5_reg[30]),
        .I1(add_ln131_4_reg_1025_pp0_iter5_reg[30]),
        .I2(add_ln131_1_reg_1020_pp0_iter5_reg[31]),
        .I3(add_ln131_4_reg_1025_pp0_iter5_reg[31]),
        .O(\icmp_ln139_reg_1055[0]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'hEFF1)) 
    \icmp_ln139_reg_1055[0]_i_4 
       (.I0(add_ln131_4_reg_1025_pp0_iter5_reg[31]),
        .I1(add_ln131_1_reg_1020_pp0_iter5_reg[31]),
        .I2(add_ln131_1_reg_1020_pp0_iter5_reg[32]),
        .I3(add_ln131_4_reg_1025_pp0_iter5_reg[32]),
        .O(\icmp_ln139_reg_1055[0]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'hE11E0FF00FF01EE1)) 
    \icmp_ln139_reg_1055[0]_i_5 
       (.I0(add_ln131_4_reg_1025_pp0_iter5_reg[30]),
        .I1(add_ln131_1_reg_1020_pp0_iter5_reg[30]),
        .I2(add_ln131_4_reg_1025_pp0_iter5_reg[32]),
        .I3(add_ln131_1_reg_1020_pp0_iter5_reg[32]),
        .I4(add_ln131_4_reg_1025_pp0_iter5_reg[31]),
        .I5(add_ln131_1_reg_1020_pp0_iter5_reg[31]),
        .O(\icmp_ln139_reg_1055[0]_i_5_n_0 ));
  FDRE \icmp_ln139_reg_1055_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\icmp_ln139_reg_1055[0]_i_1_n_0 ),
        .Q(icmp_ln139_reg_1055),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \icmp_ln139_reg_1055_reg[0]_i_2 
       (.CI(\acc_reg_1050_reg[31]_i_1_n_0 ),
        .CO({\NLW_icmp_ln139_reg_1055_reg[0]_i_2_CO_UNCONNECTED [3:1],\icmp_ln139_reg_1055_reg[0]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,\icmp_ln139_reg_1055[0]_i_3_n_0 }),
        .O({\NLW_icmp_ln139_reg_1055_reg[0]_i_2_O_UNCONNECTED [3:2],tmp_1_fu_709_p4__0}),
        .S({1'b0,1'b0,\icmp_ln139_reg_1055[0]_i_4_n_0 ,\icmp_ln139_reg_1055[0]_i_5_n_0 }));
  LUT2 #(
    .INIT(4'h2)) 
    \icmp_ln142_reg_1061[0]_i_2 
       (.I0(tmp_1_fu_709_p4__0[2]),
        .I1(tmp_1_fu_709_p4__0[1]),
        .O(\icmp_ln142_reg_1061[0]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \icmp_ln142_reg_1061[0]_i_3 
       (.I0(tmp_1_fu_709_p4),
        .O(\icmp_ln142_reg_1061[0]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h8)) 
    \icmp_ln142_reg_1061[0]_i_4 
       (.I0(tmp_1_fu_709_p4__0[1]),
        .I1(tmp_1_fu_709_p4__0[2]),
        .O(\icmp_ln142_reg_1061[0]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h2)) 
    \icmp_ln142_reg_1061[0]_i_5 
       (.I0(tmp_1_fu_709_p4),
        .I1(\acc_reg_1050_reg[31]_i_1_n_5 ),
        .O(\icmp_ln142_reg_1061[0]_i_5_n_0 ));
  FDRE \icmp_ln142_reg_1061_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln142_fu_725_p2),
        .Q(icmp_ln142_reg_1061),
        .R(1'b0));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \icmp_ln142_reg_1061_reg[0]_i_1 
       (.CI(1'b0),
        .CO({\NLW_icmp_ln142_reg_1061_reg[0]_i_1_CO_UNCONNECTED [3:2],icmp_ln142_fu_725_p2,\icmp_ln142_reg_1061_reg[0]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,\icmp_ln142_reg_1061[0]_i_2_n_0 ,\icmp_ln142_reg_1061[0]_i_3_n_0 }),
        .O(\NLW_icmp_ln142_reg_1061_reg[0]_i_1_O_UNCONNECTED [3:0]),
        .S({1'b0,1'b0,\icmp_ln142_reg_1061[0]_i_4_n_0 ,\icmp_ln142_reg_1061[0]_i_5_n_0 }));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_regslice_both regslice_both_in_r_V_data_V_U
       (.\B_V_data_1_state_reg[1]_0 (in_r_TREADY),
        .\B_V_data_1_state_reg[1]_1 (regslice_both_out_r_V_data_V_U_n_1),
        .D(in_r_TDATA_int_regslice),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_r_TDATA(in_r_TDATA),
        .in_r_TVALID(in_r_TVALID),
        .in_r_TVALID_int_regslice(in_r_TVALID_int_regslice));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_regslice_both__parameterized1 regslice_both_in_r_V_last_V_U
       (.\B_V_data_1_state_reg[0]_0 (regslice_both_out_r_V_data_V_U_n_1),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_r_TLAST(in_r_TLAST),
        .in_r_TLAST_int_regslice(in_r_TLAST_int_regslice),
        .in_r_TVALID(in_r_TVALID));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_regslice_both_2 regslice_both_out_r_V_data_V_U
       (.\B_V_data_1_state_reg[0]_0 (out_r_TVALID),
        .\B_V_data_1_state_reg[0]_1 (regslice_both_out_r_V_data_V_U_n_1),
        .E(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_70),
        .Q(acc_reg_1050),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter1(ap_enable_reg_pp0_iter1),
        .ap_enable_reg_pp0_iter4(ap_enable_reg_pp0_iter4),
        .ap_enable_reg_pp0_iter4_reg(add_ln131_1_reg_10200),
        .ap_enable_reg_pp0_iter7(ap_enable_reg_pp0_iter7),
        .ap_enable_reg_pp0_iter8(ap_enable_reg_pp0_iter8),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .icmp_ln139_reg_1055(icmp_ln139_reg_1055),
        .icmp_ln142_reg_1061(icmp_ln142_reg_1061),
        .in_r_TVALID_int_regslice(in_r_TVALID_int_regslice),
        .out_r_TDATA(out_r_TDATA),
        .out_r_TREADY(out_r_TREADY));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_regslice_both__parameterized1_3 regslice_both_out_r_V_last_V_U
       (.\B_V_data_1_state_reg[1]_0 (regslice_both_out_r_V_data_V_U_n_1),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter7(ap_enable_reg_pp0_iter7),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .din_last_reg_898_pp0_iter6_reg(din_last_reg_898_pp0_iter6_reg),
        .out_r_TLAST(out_r_TLAST),
        .out_r_TREADY(out_r_TREADY));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[11]_i_2 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[11]),
        .I1(din_data_reg_893[11]),
        .O(\tmp29_reg_974[11]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[11]_i_3 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[10]),
        .I1(din_data_reg_893[10]),
        .O(\tmp29_reg_974[11]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[11]_i_4 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[9]),
        .I1(din_data_reg_893[9]),
        .O(\tmp29_reg_974[11]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[11]_i_5 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[8]),
        .I1(din_data_reg_893[8]),
        .O(\tmp29_reg_974[11]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \tmp29_reg_974[15]_i_2 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[15]),
        .O(\tmp29_reg_974[15]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[15]_i_3 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[15]),
        .I1(din_data_reg_893[15]),
        .O(\tmp29_reg_974[15]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[15]_i_4 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[14]),
        .I1(din_data_reg_893[14]),
        .O(\tmp29_reg_974[15]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[15]_i_5 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[13]),
        .I1(din_data_reg_893[13]),
        .O(\tmp29_reg_974[15]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[15]_i_6 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[12]),
        .I1(din_data_reg_893[12]),
        .O(\tmp29_reg_974[15]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[3]_i_2 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[3]),
        .I1(din_data_reg_893[3]),
        .O(\tmp29_reg_974[3]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[3]_i_3 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[2]),
        .I1(din_data_reg_893[2]),
        .O(\tmp29_reg_974[3]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[3]_i_4 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[1]),
        .I1(din_data_reg_893[1]),
        .O(\tmp29_reg_974[3]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[3]_i_5 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[0]),
        .I1(din_data_reg_893[0]),
        .O(\tmp29_reg_974[3]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[7]_i_2 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[7]),
        .I1(din_data_reg_893[7]),
        .O(\tmp29_reg_974[7]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[7]_i_3 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[6]),
        .I1(din_data_reg_893[6]),
        .O(\tmp29_reg_974[7]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[7]_i_4 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[5]),
        .I1(din_data_reg_893[5]),
        .O(\tmp29_reg_974[7]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \tmp29_reg_974[7]_i_5 
       (.I0(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[4]),
        .I1(din_data_reg_893[4]),
        .O(\tmp29_reg_974[7]_i_5_n_0 ));
  FDRE \tmp29_reg_974_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[0]),
        .Q(p_shl1_cast_fu_624_p1[2]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[10]),
        .Q(p_shl1_cast_fu_624_p1[12]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[11]),
        .Q(p_shl1_cast_fu_624_p1[13]),
        .R(1'b0));
  CARRY4 \tmp29_reg_974_reg[11]_i_1 
       (.CI(\tmp29_reg_974_reg[7]_i_1_n_0 ),
        .CO({\tmp29_reg_974_reg[11]_i_1_n_0 ,\tmp29_reg_974_reg[11]_i_1_n_1 ,\tmp29_reg_974_reg[11]_i_1_n_2 ,\tmp29_reg_974_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[11:8]),
        .O(tmp29_fu_600_p2[11:8]),
        .S({\tmp29_reg_974[11]_i_2_n_0 ,\tmp29_reg_974[11]_i_3_n_0 ,\tmp29_reg_974[11]_i_4_n_0 ,\tmp29_reg_974[11]_i_5_n_0 }));
  FDRE \tmp29_reg_974_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[12]),
        .Q(p_shl1_cast_fu_624_p1[14]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[13]),
        .Q(p_shl1_cast_fu_624_p1[15]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[14]),
        .Q(p_shl1_cast_fu_624_p1[16]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[15]),
        .Q(p_shl1_cast_fu_624_p1[17]),
        .R(1'b0));
  CARRY4 \tmp29_reg_974_reg[15]_i_1 
       (.CI(\tmp29_reg_974_reg[11]_i_1_n_0 ),
        .CO({\tmp29_reg_974_reg[15]_i_1_n_0 ,\tmp29_reg_974_reg[15]_i_1_n_1 ,\tmp29_reg_974_reg[15]_i_1_n_2 ,\tmp29_reg_974_reg[15]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\tmp29_reg_974[15]_i_2_n_0 ,fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[14:12]}),
        .O(tmp29_fu_600_p2[15:12]),
        .S({\tmp29_reg_974[15]_i_3_n_0 ,\tmp29_reg_974[15]_i_4_n_0 ,\tmp29_reg_974[15]_i_5_n_0 ,\tmp29_reg_974[15]_i_6_n_0 }));
  FDRE \tmp29_reg_974_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[16]),
        .Q(p_shl1_cast_fu_624_p1[18]),
        .R(1'b0));
  CARRY4 \tmp29_reg_974_reg[16]_i_1 
       (.CI(\tmp29_reg_974_reg[15]_i_1_n_0 ),
        .CO(\NLW_tmp29_reg_974_reg[16]_i_1_CO_UNCONNECTED [3:0]),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_tmp29_reg_974_reg[16]_i_1_O_UNCONNECTED [3:1],tmp29_fu_600_p2[16]}),
        .S({1'b0,1'b0,1'b0,1'b1}));
  FDRE \tmp29_reg_974_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[1]),
        .Q(p_shl1_cast_fu_624_p1[3]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[2]),
        .Q(p_shl1_cast_fu_624_p1[4]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[3]),
        .Q(p_shl1_cast_fu_624_p1[5]),
        .R(1'b0));
  CARRY4 \tmp29_reg_974_reg[3]_i_1 
       (.CI(1'b0),
        .CO({\tmp29_reg_974_reg[3]_i_1_n_0 ,\tmp29_reg_974_reg[3]_i_1_n_1 ,\tmp29_reg_974_reg[3]_i_1_n_2 ,\tmp29_reg_974_reg[3]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[3:0]),
        .O(tmp29_fu_600_p2[3:0]),
        .S({\tmp29_reg_974[3]_i_2_n_0 ,\tmp29_reg_974[3]_i_3_n_0 ,\tmp29_reg_974[3]_i_4_n_0 ,\tmp29_reg_974[3]_i_5_n_0 }));
  FDRE \tmp29_reg_974_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[4]),
        .Q(p_shl1_cast_fu_624_p1[6]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[5]),
        .Q(p_shl1_cast_fu_624_p1[7]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[6]),
        .Q(p_shl1_cast_fu_624_p1[8]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[7]),
        .Q(p_shl1_cast_fu_624_p1[9]),
        .R(1'b0));
  CARRY4 \tmp29_reg_974_reg[7]_i_1 
       (.CI(\tmp29_reg_974_reg[3]_i_1_n_0 ),
        .CO({\tmp29_reg_974_reg[7]_i_1_n_0 ,\tmp29_reg_974_reg[7]_i_1_n_1 ,\tmp29_reg_974_reg[7]_i_1_n_2 ,\tmp29_reg_974_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[7:4]),
        .O(tmp29_fu_600_p2[7:4]),
        .S({\tmp29_reg_974[7]_i_2_n_0 ,\tmp29_reg_974[7]_i_3_n_0 ,\tmp29_reg_974[7]_i_4_n_0 ,\tmp29_reg_974[7]_i_5_n_0 }));
  FDRE \tmp29_reg_974_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[8]),
        .Q(p_shl1_cast_fu_624_p1[10]),
        .R(1'b0));
  FDRE \tmp29_reg_974_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(tmp29_fu_600_p2[9]),
        .Q(p_shl1_cast_fu_624_p1[11]),
        .R(1'b0));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_11ns_28_4_1
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg;

  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_11ns_28_4_1_DSP48_3 fsk_lpf_am_addmul_16s_16s_11ns_28_4_1_DSP48_3_U
       (.PCOUT(PCOUT),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_11ns_28_4_1_DSP48_3
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_0);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_0;

  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg_0;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_P_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b1,1'b0,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P(NLW_p_reg_reg_P_UNCONNECTED[47:0]),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_11s_28_4_1
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    A);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]A;

  wire [15:0]A;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_11s_28_4_1_DSP48_5 fsk_lpf_am_addmul_16s_16s_11s_28_4_1_DSP48_5_U
       (.A(A),
        .PCOUT(PCOUT),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_11s_28_4_1_DSP48_5
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    A);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]A;

  wire [15:0]A;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
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
  wire [47:0]NLW_p_reg_reg_P_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1,1'b1,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P(NLW_p_reg_reg_P_UNCONNECTED[47:0]),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_12s_29_4_1
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg;

  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_12s_29_4_1_DSP48_4_5 fsk_lpf_am_addmul_16s_16s_12s_29_4_1_DSP48_4_U
       (.PCOUT(PCOUT),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg));
endmodule

(* ORIG_REF_NAME = "fsk_lpf_am_addmul_16s_16s_12s_29_4_1" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_12s_29_4_1_0
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg;

  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_12s_29_4_1_DSP48_4 fsk_lpf_am_addmul_16s_16s_12s_29_4_1_DSP48_4_U
       (.PCOUT(PCOUT),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_12s_29_4_1_DSP48_4
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_0);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_0;

  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg_0;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_P_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0,1'b1,1'b0,1'b0,1'b1,1'b1,1'b0,1'b0,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P(NLW_p_reg_reg_P_UNCONNECTED[47:0]),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT(PCOUT),
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

(* ORIG_REF_NAME = "fsk_lpf_am_addmul_16s_16s_12s_29_4_1_DSP48_4" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_12s_29_4_1_DSP48_4_5
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_0);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_0;

  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg_0;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_P_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b1,1'b1,1'b0,1'b0,1'b1,1'b1,1'b1,1'b1,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P(NLW_p_reg_reg_P_UNCONNECTED[47:0]),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_14ns_31_4_1
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    D,
    Q);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]D;
  input [15:0]Q;

  wire [15:0]D;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_14ns_31_4_1_DSP48_2 fsk_lpf_am_addmul_16s_16s_14ns_31_4_1_DSP48_2_U
       (.D(D),
        .PCOUT(PCOUT),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_14ns_31_4_1_DSP48_2
   (PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    D,
    Q);
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]D;
  input [15:0]Q;

  wire [15:0]D;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
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
  wire [47:0]NLW_p_reg_reg_P_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b1,1'b1,1'b0,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({D[15],D[15],D[15],D[15],D[15],D[15],D[15],D[15],D[15],D}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P(NLW_p_reg_reg_P_UNCONNECTED[47:0]),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_15ns_33_4_1
   (ACOUT,
    PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    A);
  output [29:0]ACOUT;
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]A;

  wire [15:0]A;
  wire [29:0]ACOUT;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_15ns_33_4_1_DSP48_1 fsk_lpf_am_addmul_16s_16s_15ns_33_4_1_DSP48_1_U
       (.A(A),
        .ACOUT(ACOUT),
        .PCOUT(PCOUT),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_am_addmul_16s_16s_15ns_33_4_1_DSP48_1
   (ACOUT,
    PCOUT,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    A);
  output [29:0]ACOUT;
  output [47:0]PCOUT;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]A;

  wire [15:0]A;
  wire [29:0]ACOUT;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_P_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(ACOUT),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P(NLW_p_reg_reg_P_UNCONNECTED[47:0]),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_10s_28s_28_4_1
   (P,
    D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg,
    PCOUT,
    DI,
    S,
    \add_ln131_11_reg_1040_reg[28] ,
    \add_ln131_11_reg_1040_reg[27] );
  output [0:0]P;
  output [28:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg;
  input [47:0]PCOUT;
  input [0:0]DI;
  input [0:0]S;
  input [0:0]\add_ln131_11_reg_1040_reg[28] ;
  input [26:0]\add_ln131_11_reg_1040_reg[27] ;

  wire [28:0]D;
  wire [0:0]DI;
  wire [0:0]P;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire [0:0]S;
  wire [26:0]\add_ln131_11_reg_1040_reg[27] ;
  wire [0:0]\add_ln131_11_reg_1040_reg[28] ;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_10s_28s_28_4_1_DSP48_11 fsk_lpf_ama_addmuladd_16s_16s_10s_28s_28_4_1_DSP48_11_U
       (.D(D),
        .DI(DI),
        .P(P),
        .PCOUT(PCOUT),
        .Q(Q),
        .S(S),
        .\add_ln131_11_reg_1040_reg[27] (\add_ln131_11_reg_1040_reg[27] ),
        .\add_ln131_11_reg_1040_reg[28] (\add_ln131_11_reg_1040_reg[28] ),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_10s_28s_28_4_1_DSP48_11
   (P,
    D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_0,
    PCOUT,
    DI,
    S,
    \add_ln131_11_reg_1040_reg[28] ,
    \add_ln131_11_reg_1040_reg[27] );
  output [0:0]P;
  output [28:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_0;
  input [47:0]PCOUT;
  input [0:0]DI;
  input [0:0]S;
  input [0:0]\add_ln131_11_reg_1040_reg[28] ;
  input [26:0]\add_ln131_11_reg_1040_reg[27] ;

  wire [28:0]D;
  wire [0:0]DI;
  wire [0:0]P;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire [0:0]S;
  wire \add_ln131_11_reg_1040[11]_i_2_n_0 ;
  wire \add_ln131_11_reg_1040[11]_i_3_n_0 ;
  wire \add_ln131_11_reg_1040[11]_i_4_n_0 ;
  wire \add_ln131_11_reg_1040[11]_i_5_n_0 ;
  wire \add_ln131_11_reg_1040[15]_i_2_n_0 ;
  wire \add_ln131_11_reg_1040[15]_i_3_n_0 ;
  wire \add_ln131_11_reg_1040[15]_i_4_n_0 ;
  wire \add_ln131_11_reg_1040[15]_i_5_n_0 ;
  wire \add_ln131_11_reg_1040[19]_i_2_n_0 ;
  wire \add_ln131_11_reg_1040[19]_i_3_n_0 ;
  wire \add_ln131_11_reg_1040[19]_i_4_n_0 ;
  wire \add_ln131_11_reg_1040[19]_i_5_n_0 ;
  wire \add_ln131_11_reg_1040[23]_i_2_n_0 ;
  wire \add_ln131_11_reg_1040[23]_i_3_n_0 ;
  wire \add_ln131_11_reg_1040[23]_i_4_n_0 ;
  wire \add_ln131_11_reg_1040[23]_i_5_n_0 ;
  wire \add_ln131_11_reg_1040[27]_i_4_n_0 ;
  wire \add_ln131_11_reg_1040[27]_i_5_n_0 ;
  wire \add_ln131_11_reg_1040[27]_i_6_n_0 ;
  wire \add_ln131_11_reg_1040[3]_i_2_n_0 ;
  wire \add_ln131_11_reg_1040[3]_i_3_n_0 ;
  wire \add_ln131_11_reg_1040[3]_i_4_n_0 ;
  wire \add_ln131_11_reg_1040[3]_i_5_n_0 ;
  wire \add_ln131_11_reg_1040[7]_i_2_n_0 ;
  wire \add_ln131_11_reg_1040[7]_i_3_n_0 ;
  wire \add_ln131_11_reg_1040[7]_i_4_n_0 ;
  wire \add_ln131_11_reg_1040[7]_i_5_n_0 ;
  wire \add_ln131_11_reg_1040_reg[11]_i_1_n_0 ;
  wire \add_ln131_11_reg_1040_reg[11]_i_1_n_1 ;
  wire \add_ln131_11_reg_1040_reg[11]_i_1_n_2 ;
  wire \add_ln131_11_reg_1040_reg[11]_i_1_n_3 ;
  wire \add_ln131_11_reg_1040_reg[15]_i_1_n_0 ;
  wire \add_ln131_11_reg_1040_reg[15]_i_1_n_1 ;
  wire \add_ln131_11_reg_1040_reg[15]_i_1_n_2 ;
  wire \add_ln131_11_reg_1040_reg[15]_i_1_n_3 ;
  wire \add_ln131_11_reg_1040_reg[19]_i_1_n_0 ;
  wire \add_ln131_11_reg_1040_reg[19]_i_1_n_1 ;
  wire \add_ln131_11_reg_1040_reg[19]_i_1_n_2 ;
  wire \add_ln131_11_reg_1040_reg[19]_i_1_n_3 ;
  wire \add_ln131_11_reg_1040_reg[23]_i_1_n_0 ;
  wire \add_ln131_11_reg_1040_reg[23]_i_1_n_1 ;
  wire \add_ln131_11_reg_1040_reg[23]_i_1_n_2 ;
  wire \add_ln131_11_reg_1040_reg[23]_i_1_n_3 ;
  wire [26:0]\add_ln131_11_reg_1040_reg[27] ;
  wire \add_ln131_11_reg_1040_reg[27]_i_1_n_0 ;
  wire \add_ln131_11_reg_1040_reg[27]_i_1_n_1 ;
  wire \add_ln131_11_reg_1040_reg[27]_i_1_n_2 ;
  wire \add_ln131_11_reg_1040_reg[27]_i_1_n_3 ;
  wire [0:0]\add_ln131_11_reg_1040_reg[28] ;
  wire \add_ln131_11_reg_1040_reg[3]_i_1_n_0 ;
  wire \add_ln131_11_reg_1040_reg[3]_i_1_n_1 ;
  wire \add_ln131_11_reg_1040_reg[3]_i_1_n_2 ;
  wire \add_ln131_11_reg_1040_reg[3]_i_1_n_3 ;
  wire \add_ln131_11_reg_1040_reg[7]_i_1_n_0 ;
  wire \add_ln131_11_reg_1040_reg[7]_i_1_n_1 ;
  wire \add_ln131_11_reg_1040_reg[7]_i_1_n_2 ;
  wire \add_ln131_11_reg_1040_reg[7]_i_1_n_3 ;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg_0;
  wire p_reg_reg_n_100;
  wire p_reg_reg_n_101;
  wire p_reg_reg_n_102;
  wire p_reg_reg_n_103;
  wire p_reg_reg_n_104;
  wire p_reg_reg_n_105;
  wire p_reg_reg_n_79;
  wire p_reg_reg_n_80;
  wire p_reg_reg_n_81;
  wire p_reg_reg_n_82;
  wire p_reg_reg_n_83;
  wire p_reg_reg_n_84;
  wire p_reg_reg_n_85;
  wire p_reg_reg_n_86;
  wire p_reg_reg_n_87;
  wire p_reg_reg_n_88;
  wire p_reg_reg_n_89;
  wire p_reg_reg_n_90;
  wire p_reg_reg_n_91;
  wire p_reg_reg_n_92;
  wire p_reg_reg_n_93;
  wire p_reg_reg_n_94;
  wire p_reg_reg_n_95;
  wire p_reg_reg_n_96;
  wire p_reg_reg_n_97;
  wire p_reg_reg_n_98;
  wire p_reg_reg_n_99;
  wire [3:0]\NLW_add_ln131_11_reg_1040_reg[28]_i_1_CO_UNCONNECTED ;
  wire [3:1]\NLW_add_ln131_11_reg_1040_reg[28]_i_1_O_UNCONNECTED ;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:28]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;

  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[11]_i_2 
       (.I0(p_reg_reg_n_94),
        .I1(\add_ln131_11_reg_1040_reg[27] [11]),
        .O(\add_ln131_11_reg_1040[11]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[11]_i_3 
       (.I0(p_reg_reg_n_95),
        .I1(\add_ln131_11_reg_1040_reg[27] [10]),
        .O(\add_ln131_11_reg_1040[11]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[11]_i_4 
       (.I0(p_reg_reg_n_96),
        .I1(\add_ln131_11_reg_1040_reg[27] [9]),
        .O(\add_ln131_11_reg_1040[11]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[11]_i_5 
       (.I0(p_reg_reg_n_97),
        .I1(\add_ln131_11_reg_1040_reg[27] [8]),
        .O(\add_ln131_11_reg_1040[11]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[15]_i_2 
       (.I0(p_reg_reg_n_90),
        .I1(\add_ln131_11_reg_1040_reg[27] [15]),
        .O(\add_ln131_11_reg_1040[15]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[15]_i_3 
       (.I0(p_reg_reg_n_91),
        .I1(\add_ln131_11_reg_1040_reg[27] [14]),
        .O(\add_ln131_11_reg_1040[15]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[15]_i_4 
       (.I0(p_reg_reg_n_92),
        .I1(\add_ln131_11_reg_1040_reg[27] [13]),
        .O(\add_ln131_11_reg_1040[15]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[15]_i_5 
       (.I0(p_reg_reg_n_93),
        .I1(\add_ln131_11_reg_1040_reg[27] [12]),
        .O(\add_ln131_11_reg_1040[15]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[19]_i_2 
       (.I0(p_reg_reg_n_86),
        .I1(\add_ln131_11_reg_1040_reg[27] [19]),
        .O(\add_ln131_11_reg_1040[19]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[19]_i_3 
       (.I0(p_reg_reg_n_87),
        .I1(\add_ln131_11_reg_1040_reg[27] [18]),
        .O(\add_ln131_11_reg_1040[19]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[19]_i_4 
       (.I0(p_reg_reg_n_88),
        .I1(\add_ln131_11_reg_1040_reg[27] [17]),
        .O(\add_ln131_11_reg_1040[19]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[19]_i_5 
       (.I0(p_reg_reg_n_89),
        .I1(\add_ln131_11_reg_1040_reg[27] [16]),
        .O(\add_ln131_11_reg_1040[19]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[23]_i_2 
       (.I0(p_reg_reg_n_82),
        .I1(\add_ln131_11_reg_1040_reg[27] [23]),
        .O(\add_ln131_11_reg_1040[23]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[23]_i_3 
       (.I0(p_reg_reg_n_83),
        .I1(\add_ln131_11_reg_1040_reg[27] [22]),
        .O(\add_ln131_11_reg_1040[23]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[23]_i_4 
       (.I0(p_reg_reg_n_84),
        .I1(\add_ln131_11_reg_1040_reg[27] [21]),
        .O(\add_ln131_11_reg_1040[23]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[23]_i_5 
       (.I0(p_reg_reg_n_85),
        .I1(\add_ln131_11_reg_1040_reg[27] [20]),
        .O(\add_ln131_11_reg_1040[23]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[27]_i_4 
       (.I0(p_reg_reg_n_79),
        .I1(\add_ln131_11_reg_1040_reg[27] [26]),
        .O(\add_ln131_11_reg_1040[27]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[27]_i_5 
       (.I0(p_reg_reg_n_80),
        .I1(\add_ln131_11_reg_1040_reg[27] [25]),
        .O(\add_ln131_11_reg_1040[27]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[27]_i_6 
       (.I0(p_reg_reg_n_81),
        .I1(\add_ln131_11_reg_1040_reg[27] [24]),
        .O(\add_ln131_11_reg_1040[27]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[3]_i_2 
       (.I0(p_reg_reg_n_102),
        .I1(\add_ln131_11_reg_1040_reg[27] [3]),
        .O(\add_ln131_11_reg_1040[3]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[3]_i_3 
       (.I0(p_reg_reg_n_103),
        .I1(\add_ln131_11_reg_1040_reg[27] [2]),
        .O(\add_ln131_11_reg_1040[3]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[3]_i_4 
       (.I0(p_reg_reg_n_104),
        .I1(\add_ln131_11_reg_1040_reg[27] [1]),
        .O(\add_ln131_11_reg_1040[3]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[3]_i_5 
       (.I0(p_reg_reg_n_105),
        .I1(\add_ln131_11_reg_1040_reg[27] [0]),
        .O(\add_ln131_11_reg_1040[3]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[7]_i_2 
       (.I0(p_reg_reg_n_98),
        .I1(\add_ln131_11_reg_1040_reg[27] [7]),
        .O(\add_ln131_11_reg_1040[7]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[7]_i_3 
       (.I0(p_reg_reg_n_99),
        .I1(\add_ln131_11_reg_1040_reg[27] [6]),
        .O(\add_ln131_11_reg_1040[7]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[7]_i_4 
       (.I0(p_reg_reg_n_100),
        .I1(\add_ln131_11_reg_1040_reg[27] [5]),
        .O(\add_ln131_11_reg_1040[7]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[7]_i_5 
       (.I0(p_reg_reg_n_101),
        .I1(\add_ln131_11_reg_1040_reg[27] [4]),
        .O(\add_ln131_11_reg_1040[7]_i_5_n_0 ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_11_reg_1040_reg[11]_i_1 
       (.CI(\add_ln131_11_reg_1040_reg[7]_i_1_n_0 ),
        .CO({\add_ln131_11_reg_1040_reg[11]_i_1_n_0 ,\add_ln131_11_reg_1040_reg[11]_i_1_n_1 ,\add_ln131_11_reg_1040_reg[11]_i_1_n_2 ,\add_ln131_11_reg_1040_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_94,p_reg_reg_n_95,p_reg_reg_n_96,p_reg_reg_n_97}),
        .O(D[11:8]),
        .S({\add_ln131_11_reg_1040[11]_i_2_n_0 ,\add_ln131_11_reg_1040[11]_i_3_n_0 ,\add_ln131_11_reg_1040[11]_i_4_n_0 ,\add_ln131_11_reg_1040[11]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_11_reg_1040_reg[15]_i_1 
       (.CI(\add_ln131_11_reg_1040_reg[11]_i_1_n_0 ),
        .CO({\add_ln131_11_reg_1040_reg[15]_i_1_n_0 ,\add_ln131_11_reg_1040_reg[15]_i_1_n_1 ,\add_ln131_11_reg_1040_reg[15]_i_1_n_2 ,\add_ln131_11_reg_1040_reg[15]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_90,p_reg_reg_n_91,p_reg_reg_n_92,p_reg_reg_n_93}),
        .O(D[15:12]),
        .S({\add_ln131_11_reg_1040[15]_i_2_n_0 ,\add_ln131_11_reg_1040[15]_i_3_n_0 ,\add_ln131_11_reg_1040[15]_i_4_n_0 ,\add_ln131_11_reg_1040[15]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_11_reg_1040_reg[19]_i_1 
       (.CI(\add_ln131_11_reg_1040_reg[15]_i_1_n_0 ),
        .CO({\add_ln131_11_reg_1040_reg[19]_i_1_n_0 ,\add_ln131_11_reg_1040_reg[19]_i_1_n_1 ,\add_ln131_11_reg_1040_reg[19]_i_1_n_2 ,\add_ln131_11_reg_1040_reg[19]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_86,p_reg_reg_n_87,p_reg_reg_n_88,p_reg_reg_n_89}),
        .O(D[19:16]),
        .S({\add_ln131_11_reg_1040[19]_i_2_n_0 ,\add_ln131_11_reg_1040[19]_i_3_n_0 ,\add_ln131_11_reg_1040[19]_i_4_n_0 ,\add_ln131_11_reg_1040[19]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_11_reg_1040_reg[23]_i_1 
       (.CI(\add_ln131_11_reg_1040_reg[19]_i_1_n_0 ),
        .CO({\add_ln131_11_reg_1040_reg[23]_i_1_n_0 ,\add_ln131_11_reg_1040_reg[23]_i_1_n_1 ,\add_ln131_11_reg_1040_reg[23]_i_1_n_2 ,\add_ln131_11_reg_1040_reg[23]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_82,p_reg_reg_n_83,p_reg_reg_n_84,p_reg_reg_n_85}),
        .O(D[23:20]),
        .S({\add_ln131_11_reg_1040[23]_i_2_n_0 ,\add_ln131_11_reg_1040[23]_i_3_n_0 ,\add_ln131_11_reg_1040[23]_i_4_n_0 ,\add_ln131_11_reg_1040[23]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_11_reg_1040_reg[27]_i_1 
       (.CI(\add_ln131_11_reg_1040_reg[23]_i_1_n_0 ),
        .CO({\add_ln131_11_reg_1040_reg[27]_i_1_n_0 ,\add_ln131_11_reg_1040_reg[27]_i_1_n_1 ,\add_ln131_11_reg_1040_reg[27]_i_1_n_2 ,\add_ln131_11_reg_1040_reg[27]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({DI,p_reg_reg_n_79,p_reg_reg_n_80,p_reg_reg_n_81}),
        .O(D[27:24]),
        .S({S,\add_ln131_11_reg_1040[27]_i_4_n_0 ,\add_ln131_11_reg_1040[27]_i_5_n_0 ,\add_ln131_11_reg_1040[27]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_11_reg_1040_reg[28]_i_1 
       (.CI(\add_ln131_11_reg_1040_reg[27]_i_1_n_0 ),
        .CO(\NLW_add_ln131_11_reg_1040_reg[28]_i_1_CO_UNCONNECTED [3:0]),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_add_ln131_11_reg_1040_reg[28]_i_1_O_UNCONNECTED [3:1],D[28]}),
        .S({1'b0,1'b0,1'b0,\add_ln131_11_reg_1040_reg[28] }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_11_reg_1040_reg[3]_i_1 
       (.CI(1'b0),
        .CO({\add_ln131_11_reg_1040_reg[3]_i_1_n_0 ,\add_ln131_11_reg_1040_reg[3]_i_1_n_1 ,\add_ln131_11_reg_1040_reg[3]_i_1_n_2 ,\add_ln131_11_reg_1040_reg[3]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_102,p_reg_reg_n_103,p_reg_reg_n_104,p_reg_reg_n_105}),
        .O(D[3:0]),
        .S({\add_ln131_11_reg_1040[3]_i_2_n_0 ,\add_ln131_11_reg_1040[3]_i_3_n_0 ,\add_ln131_11_reg_1040[3]_i_4_n_0 ,\add_ln131_11_reg_1040[3]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_11_reg_1040_reg[7]_i_1 
       (.CI(\add_ln131_11_reg_1040_reg[3]_i_1_n_0 ),
        .CO({\add_ln131_11_reg_1040_reg[7]_i_1_n_0 ,\add_ln131_11_reg_1040_reg[7]_i_1_n_1 ,\add_ln131_11_reg_1040_reg[7]_i_1_n_2 ,\add_ln131_11_reg_1040_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_98,p_reg_reg_n_99,p_reg_reg_n_100,p_reg_reg_n_101}),
        .O(D[7:4]),
        .S({\add_ln131_11_reg_1040[7]_i_2_n_0 ,\add_ln131_11_reg_1040[7]_i_3_n_0 ,\add_ln131_11_reg_1040[7]_i_4_n_0 ,\add_ln131_11_reg_1040[7]_i_5_n_0 }));
  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0,1'b0,1'b1,1'b1,1'b1,1'b0,1'b1}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:28],P,p_reg_reg_n_79,p_reg_reg_n_80,p_reg_reg_n_81,p_reg_reg_n_82,p_reg_reg_n_83,p_reg_reg_n_84,p_reg_reg_n_85,p_reg_reg_n_86,p_reg_reg_n_87,p_reg_reg_n_88,p_reg_reg_n_89,p_reg_reg_n_90,p_reg_reg_n_91,p_reg_reg_n_92,p_reg_reg_n_93,p_reg_reg_n_94,p_reg_reg_n_95,p_reg_reg_n_96,p_reg_reg_n_97,p_reg_reg_n_98,p_reg_reg_n_99,p_reg_reg_n_100,p_reg_reg_n_101,p_reg_reg_n_102,p_reg_reg_n_103,p_reg_reg_n_104,p_reg_reg_n_105}),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1
   (D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg,
    PCOUT);
  output [28:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg;
  input [47:0]PCOUT;

  wire [28:0]D;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_DSP48_10_4 fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_DSP48_10_U
       (.D(D),
        .PCOUT(PCOUT),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg));
endmodule

(* ORIG_REF_NAME = "fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_1
   (p_reg_reg,
    DI,
    p_reg_reg_0,
    S,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_1,
    PCOUT,
    P);
  output [26:0]p_reg_reg;
  output [0:0]DI;
  output [0:0]p_reg_reg_0;
  output [0:0]S;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_1;
  input [47:0]PCOUT;
  input [0:0]P;

  wire [0:0]DI;
  wire [0:0]P;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire [0:0]S;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [26:0]p_reg_reg;
  wire [0:0]p_reg_reg_0;
  wire [15:0]p_reg_reg_1;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_DSP48_10 fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_DSP48_10_U
       (.DI(DI),
        .P(P),
        .PCOUT(PCOUT),
        .Q(Q),
        .S(S),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg),
        .p_reg_reg_1(p_reg_reg_0),
        .p_reg_reg_2(p_reg_reg_1));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_DSP48_10
   (p_reg_reg_0,
    DI,
    p_reg_reg_1,
    S,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_2,
    PCOUT,
    P);
  output [26:0]p_reg_reg_0;
  output [0:0]DI;
  output [0:0]p_reg_reg_1;
  output [0:0]S;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_2;
  input [47:0]PCOUT;
  input [0:0]P;

  wire [0:0]DI;
  wire [0:0]P;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire [0:0]S;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [26:0]p_reg_reg_0;
  wire [0:0]p_reg_reg_1;
  wire [15:0]p_reg_reg_2;
  wire p_reg_reg_n_77;
  wire p_reg_reg_n_78;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:29]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;

  LUT1 #(
    .INIT(2'h1)) 
    \add_ln131_11_reg_1040[27]_i_2 
       (.I0(p_reg_reg_n_78),
        .O(DI));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_11_reg_1040[27]_i_3 
       (.I0(p_reg_reg_n_78),
        .I1(P),
        .O(S));
  LUT2 #(
    .INIT(4'h9)) 
    \add_ln131_11_reg_1040[28]_i_2 
       (.I0(p_reg_reg_n_78),
        .I1(p_reg_reg_n_77),
        .O(p_reg_reg_1));
  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2[15],p_reg_reg_2}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b0,1'b0,1'b0,1'b1}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:29],p_reg_reg_n_77,p_reg_reg_n_78,p_reg_reg_0}),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN(PCOUT),
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

(* ORIG_REF_NAME = "fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_DSP48_10" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_12s_29s_29_4_1_DSP48_10_4
   (D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_0,
    PCOUT);
  output [28:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_0;
  input [47:0]PCOUT;

  wire [28:0]D;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg_0;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:29]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,1'b1,1'b1,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:29],D}),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_13ns_31s_31_4_1
   (P,
    D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg,
    PCOUT,
    \add_ln131_4_reg_1025_reg[31] ,
    DI,
    S,
    \add_ln131_4_reg_1025_reg[32] );
  output [0:0]P;
  output [32:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg;
  input [47:0]PCOUT;
  input [30:0]\add_ln131_4_reg_1025_reg[31] ;
  input [0:0]DI;
  input [1:0]S;
  input [0:0]\add_ln131_4_reg_1025_reg[32] ;

  wire [32:0]D;
  wire [0:0]DI;
  wire [0:0]P;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire [1:0]S;
  wire [30:0]\add_ln131_4_reg_1025_reg[31] ;
  wire [0:0]\add_ln131_4_reg_1025_reg[32] ;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_13ns_31s_31_4_1_DSP48_8 fsk_lpf_ama_addmuladd_16s_16s_13ns_31s_31_4_1_DSP48_8_U
       (.D(D),
        .DI(DI),
        .P(P),
        .PCOUT(PCOUT),
        .Q(Q),
        .S(S),
        .\add_ln131_4_reg_1025_reg[31] (\add_ln131_4_reg_1025_reg[31] ),
        .\add_ln131_4_reg_1025_reg[32] (\add_ln131_4_reg_1025_reg[32] ),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_13ns_31s_31_4_1_DSP48_8
   (P,
    D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_0,
    PCOUT,
    \add_ln131_4_reg_1025_reg[31] ,
    DI,
    S,
    \add_ln131_4_reg_1025_reg[32] );
  output [0:0]P;
  output [32:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_0;
  input [47:0]PCOUT;
  input [30:0]\add_ln131_4_reg_1025_reg[31] ;
  input [0:0]DI;
  input [1:0]S;
  input [0:0]\add_ln131_4_reg_1025_reg[32] ;

  wire [32:0]D;
  wire [0:0]DI;
  wire [0:0]P;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire [1:0]S;
  wire \add_ln131_4_reg_1025[11]_i_2_n_0 ;
  wire \add_ln131_4_reg_1025[11]_i_3_n_0 ;
  wire \add_ln131_4_reg_1025[11]_i_4_n_0 ;
  wire \add_ln131_4_reg_1025[11]_i_5_n_0 ;
  wire \add_ln131_4_reg_1025[15]_i_2_n_0 ;
  wire \add_ln131_4_reg_1025[15]_i_3_n_0 ;
  wire \add_ln131_4_reg_1025[15]_i_4_n_0 ;
  wire \add_ln131_4_reg_1025[15]_i_5_n_0 ;
  wire \add_ln131_4_reg_1025[19]_i_2_n_0 ;
  wire \add_ln131_4_reg_1025[19]_i_3_n_0 ;
  wire \add_ln131_4_reg_1025[19]_i_4_n_0 ;
  wire \add_ln131_4_reg_1025[19]_i_5_n_0 ;
  wire \add_ln131_4_reg_1025[23]_i_2_n_0 ;
  wire \add_ln131_4_reg_1025[23]_i_3_n_0 ;
  wire \add_ln131_4_reg_1025[23]_i_4_n_0 ;
  wire \add_ln131_4_reg_1025[23]_i_5_n_0 ;
  wire \add_ln131_4_reg_1025[27]_i_2_n_0 ;
  wire \add_ln131_4_reg_1025[27]_i_3_n_0 ;
  wire \add_ln131_4_reg_1025[27]_i_4_n_0 ;
  wire \add_ln131_4_reg_1025[27]_i_5_n_0 ;
  wire \add_ln131_4_reg_1025[31]_i_5_n_0 ;
  wire \add_ln131_4_reg_1025[31]_i_6_n_0 ;
  wire \add_ln131_4_reg_1025[3]_i_2_n_0 ;
  wire \add_ln131_4_reg_1025[3]_i_3_n_0 ;
  wire \add_ln131_4_reg_1025[3]_i_4_n_0 ;
  wire \add_ln131_4_reg_1025[3]_i_5_n_0 ;
  wire \add_ln131_4_reg_1025[7]_i_2_n_0 ;
  wire \add_ln131_4_reg_1025[7]_i_3_n_0 ;
  wire \add_ln131_4_reg_1025[7]_i_4_n_0 ;
  wire \add_ln131_4_reg_1025[7]_i_5_n_0 ;
  wire \add_ln131_4_reg_1025_reg[11]_i_1_n_0 ;
  wire \add_ln131_4_reg_1025_reg[11]_i_1_n_1 ;
  wire \add_ln131_4_reg_1025_reg[11]_i_1_n_2 ;
  wire \add_ln131_4_reg_1025_reg[11]_i_1_n_3 ;
  wire \add_ln131_4_reg_1025_reg[15]_i_1_n_0 ;
  wire \add_ln131_4_reg_1025_reg[15]_i_1_n_1 ;
  wire \add_ln131_4_reg_1025_reg[15]_i_1_n_2 ;
  wire \add_ln131_4_reg_1025_reg[15]_i_1_n_3 ;
  wire \add_ln131_4_reg_1025_reg[19]_i_1_n_0 ;
  wire \add_ln131_4_reg_1025_reg[19]_i_1_n_1 ;
  wire \add_ln131_4_reg_1025_reg[19]_i_1_n_2 ;
  wire \add_ln131_4_reg_1025_reg[19]_i_1_n_3 ;
  wire \add_ln131_4_reg_1025_reg[23]_i_1_n_0 ;
  wire \add_ln131_4_reg_1025_reg[23]_i_1_n_1 ;
  wire \add_ln131_4_reg_1025_reg[23]_i_1_n_2 ;
  wire \add_ln131_4_reg_1025_reg[23]_i_1_n_3 ;
  wire \add_ln131_4_reg_1025_reg[27]_i_1_n_0 ;
  wire \add_ln131_4_reg_1025_reg[27]_i_1_n_1 ;
  wire \add_ln131_4_reg_1025_reg[27]_i_1_n_2 ;
  wire \add_ln131_4_reg_1025_reg[27]_i_1_n_3 ;
  wire [30:0]\add_ln131_4_reg_1025_reg[31] ;
  wire \add_ln131_4_reg_1025_reg[31]_i_1_n_0 ;
  wire \add_ln131_4_reg_1025_reg[31]_i_1_n_1 ;
  wire \add_ln131_4_reg_1025_reg[31]_i_1_n_2 ;
  wire \add_ln131_4_reg_1025_reg[31]_i_1_n_3 ;
  wire [0:0]\add_ln131_4_reg_1025_reg[32] ;
  wire \add_ln131_4_reg_1025_reg[3]_i_1_n_0 ;
  wire \add_ln131_4_reg_1025_reg[3]_i_1_n_1 ;
  wire \add_ln131_4_reg_1025_reg[3]_i_1_n_2 ;
  wire \add_ln131_4_reg_1025_reg[3]_i_1_n_3 ;
  wire \add_ln131_4_reg_1025_reg[7]_i_1_n_0 ;
  wire \add_ln131_4_reg_1025_reg[7]_i_1_n_1 ;
  wire \add_ln131_4_reg_1025_reg[7]_i_1_n_2 ;
  wire \add_ln131_4_reg_1025_reg[7]_i_1_n_3 ;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg_0;
  wire p_reg_reg_n_100;
  wire p_reg_reg_n_101;
  wire p_reg_reg_n_102;
  wire p_reg_reg_n_103;
  wire p_reg_reg_n_104;
  wire p_reg_reg_n_105;
  wire p_reg_reg_n_76;
  wire p_reg_reg_n_77;
  wire p_reg_reg_n_78;
  wire p_reg_reg_n_79;
  wire p_reg_reg_n_80;
  wire p_reg_reg_n_81;
  wire p_reg_reg_n_82;
  wire p_reg_reg_n_83;
  wire p_reg_reg_n_84;
  wire p_reg_reg_n_85;
  wire p_reg_reg_n_86;
  wire p_reg_reg_n_87;
  wire p_reg_reg_n_88;
  wire p_reg_reg_n_89;
  wire p_reg_reg_n_90;
  wire p_reg_reg_n_91;
  wire p_reg_reg_n_92;
  wire p_reg_reg_n_93;
  wire p_reg_reg_n_94;
  wire p_reg_reg_n_95;
  wire p_reg_reg_n_96;
  wire p_reg_reg_n_97;
  wire p_reg_reg_n_98;
  wire p_reg_reg_n_99;
  wire [3:0]\NLW_add_ln131_4_reg_1025_reg[32]_i_1_CO_UNCONNECTED ;
  wire [3:1]\NLW_add_ln131_4_reg_1025_reg[32]_i_1_O_UNCONNECTED ;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:31]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;

  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[11]_i_2 
       (.I0(p_reg_reg_n_94),
        .I1(\add_ln131_4_reg_1025_reg[31] [11]),
        .O(\add_ln131_4_reg_1025[11]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[11]_i_3 
       (.I0(p_reg_reg_n_95),
        .I1(\add_ln131_4_reg_1025_reg[31] [10]),
        .O(\add_ln131_4_reg_1025[11]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[11]_i_4 
       (.I0(p_reg_reg_n_96),
        .I1(\add_ln131_4_reg_1025_reg[31] [9]),
        .O(\add_ln131_4_reg_1025[11]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[11]_i_5 
       (.I0(p_reg_reg_n_97),
        .I1(\add_ln131_4_reg_1025_reg[31] [8]),
        .O(\add_ln131_4_reg_1025[11]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[15]_i_2 
       (.I0(p_reg_reg_n_90),
        .I1(\add_ln131_4_reg_1025_reg[31] [15]),
        .O(\add_ln131_4_reg_1025[15]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[15]_i_3 
       (.I0(p_reg_reg_n_91),
        .I1(\add_ln131_4_reg_1025_reg[31] [14]),
        .O(\add_ln131_4_reg_1025[15]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[15]_i_4 
       (.I0(p_reg_reg_n_92),
        .I1(\add_ln131_4_reg_1025_reg[31] [13]),
        .O(\add_ln131_4_reg_1025[15]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[15]_i_5 
       (.I0(p_reg_reg_n_93),
        .I1(\add_ln131_4_reg_1025_reg[31] [12]),
        .O(\add_ln131_4_reg_1025[15]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[19]_i_2 
       (.I0(p_reg_reg_n_86),
        .I1(\add_ln131_4_reg_1025_reg[31] [19]),
        .O(\add_ln131_4_reg_1025[19]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[19]_i_3 
       (.I0(p_reg_reg_n_87),
        .I1(\add_ln131_4_reg_1025_reg[31] [18]),
        .O(\add_ln131_4_reg_1025[19]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[19]_i_4 
       (.I0(p_reg_reg_n_88),
        .I1(\add_ln131_4_reg_1025_reg[31] [17]),
        .O(\add_ln131_4_reg_1025[19]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[19]_i_5 
       (.I0(p_reg_reg_n_89),
        .I1(\add_ln131_4_reg_1025_reg[31] [16]),
        .O(\add_ln131_4_reg_1025[19]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[23]_i_2 
       (.I0(p_reg_reg_n_82),
        .I1(\add_ln131_4_reg_1025_reg[31] [23]),
        .O(\add_ln131_4_reg_1025[23]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[23]_i_3 
       (.I0(p_reg_reg_n_83),
        .I1(\add_ln131_4_reg_1025_reg[31] [22]),
        .O(\add_ln131_4_reg_1025[23]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[23]_i_4 
       (.I0(p_reg_reg_n_84),
        .I1(\add_ln131_4_reg_1025_reg[31] [21]),
        .O(\add_ln131_4_reg_1025[23]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[23]_i_5 
       (.I0(p_reg_reg_n_85),
        .I1(\add_ln131_4_reg_1025_reg[31] [20]),
        .O(\add_ln131_4_reg_1025[23]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[27]_i_2 
       (.I0(p_reg_reg_n_78),
        .I1(\add_ln131_4_reg_1025_reg[31] [27]),
        .O(\add_ln131_4_reg_1025[27]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[27]_i_3 
       (.I0(p_reg_reg_n_79),
        .I1(\add_ln131_4_reg_1025_reg[31] [26]),
        .O(\add_ln131_4_reg_1025[27]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[27]_i_4 
       (.I0(p_reg_reg_n_80),
        .I1(\add_ln131_4_reg_1025_reg[31] [25]),
        .O(\add_ln131_4_reg_1025[27]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[27]_i_5 
       (.I0(p_reg_reg_n_81),
        .I1(\add_ln131_4_reg_1025_reg[31] [24]),
        .O(\add_ln131_4_reg_1025[27]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[31]_i_5 
       (.I0(p_reg_reg_n_76),
        .I1(\add_ln131_4_reg_1025_reg[31] [29]),
        .O(\add_ln131_4_reg_1025[31]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[31]_i_6 
       (.I0(p_reg_reg_n_77),
        .I1(\add_ln131_4_reg_1025_reg[31] [28]),
        .O(\add_ln131_4_reg_1025[31]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[3]_i_2 
       (.I0(p_reg_reg_n_102),
        .I1(\add_ln131_4_reg_1025_reg[31] [3]),
        .O(\add_ln131_4_reg_1025[3]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[3]_i_3 
       (.I0(p_reg_reg_n_103),
        .I1(\add_ln131_4_reg_1025_reg[31] [2]),
        .O(\add_ln131_4_reg_1025[3]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[3]_i_4 
       (.I0(p_reg_reg_n_104),
        .I1(\add_ln131_4_reg_1025_reg[31] [1]),
        .O(\add_ln131_4_reg_1025[3]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[3]_i_5 
       (.I0(p_reg_reg_n_105),
        .I1(\add_ln131_4_reg_1025_reg[31] [0]),
        .O(\add_ln131_4_reg_1025[3]_i_5_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[7]_i_2 
       (.I0(p_reg_reg_n_98),
        .I1(\add_ln131_4_reg_1025_reg[31] [7]),
        .O(\add_ln131_4_reg_1025[7]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[7]_i_3 
       (.I0(p_reg_reg_n_99),
        .I1(\add_ln131_4_reg_1025_reg[31] [6]),
        .O(\add_ln131_4_reg_1025[7]_i_3_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[7]_i_4 
       (.I0(p_reg_reg_n_100),
        .I1(\add_ln131_4_reg_1025_reg[31] [5]),
        .O(\add_ln131_4_reg_1025[7]_i_4_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[7]_i_5 
       (.I0(p_reg_reg_n_101),
        .I1(\add_ln131_4_reg_1025_reg[31] [4]),
        .O(\add_ln131_4_reg_1025[7]_i_5_n_0 ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[11]_i_1 
       (.CI(\add_ln131_4_reg_1025_reg[7]_i_1_n_0 ),
        .CO({\add_ln131_4_reg_1025_reg[11]_i_1_n_0 ,\add_ln131_4_reg_1025_reg[11]_i_1_n_1 ,\add_ln131_4_reg_1025_reg[11]_i_1_n_2 ,\add_ln131_4_reg_1025_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_94,p_reg_reg_n_95,p_reg_reg_n_96,p_reg_reg_n_97}),
        .O(D[11:8]),
        .S({\add_ln131_4_reg_1025[11]_i_2_n_0 ,\add_ln131_4_reg_1025[11]_i_3_n_0 ,\add_ln131_4_reg_1025[11]_i_4_n_0 ,\add_ln131_4_reg_1025[11]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[15]_i_1 
       (.CI(\add_ln131_4_reg_1025_reg[11]_i_1_n_0 ),
        .CO({\add_ln131_4_reg_1025_reg[15]_i_1_n_0 ,\add_ln131_4_reg_1025_reg[15]_i_1_n_1 ,\add_ln131_4_reg_1025_reg[15]_i_1_n_2 ,\add_ln131_4_reg_1025_reg[15]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_90,p_reg_reg_n_91,p_reg_reg_n_92,p_reg_reg_n_93}),
        .O(D[15:12]),
        .S({\add_ln131_4_reg_1025[15]_i_2_n_0 ,\add_ln131_4_reg_1025[15]_i_3_n_0 ,\add_ln131_4_reg_1025[15]_i_4_n_0 ,\add_ln131_4_reg_1025[15]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[19]_i_1 
       (.CI(\add_ln131_4_reg_1025_reg[15]_i_1_n_0 ),
        .CO({\add_ln131_4_reg_1025_reg[19]_i_1_n_0 ,\add_ln131_4_reg_1025_reg[19]_i_1_n_1 ,\add_ln131_4_reg_1025_reg[19]_i_1_n_2 ,\add_ln131_4_reg_1025_reg[19]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_86,p_reg_reg_n_87,p_reg_reg_n_88,p_reg_reg_n_89}),
        .O(D[19:16]),
        .S({\add_ln131_4_reg_1025[19]_i_2_n_0 ,\add_ln131_4_reg_1025[19]_i_3_n_0 ,\add_ln131_4_reg_1025[19]_i_4_n_0 ,\add_ln131_4_reg_1025[19]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[23]_i_1 
       (.CI(\add_ln131_4_reg_1025_reg[19]_i_1_n_0 ),
        .CO({\add_ln131_4_reg_1025_reg[23]_i_1_n_0 ,\add_ln131_4_reg_1025_reg[23]_i_1_n_1 ,\add_ln131_4_reg_1025_reg[23]_i_1_n_2 ,\add_ln131_4_reg_1025_reg[23]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_82,p_reg_reg_n_83,p_reg_reg_n_84,p_reg_reg_n_85}),
        .O(D[23:20]),
        .S({\add_ln131_4_reg_1025[23]_i_2_n_0 ,\add_ln131_4_reg_1025[23]_i_3_n_0 ,\add_ln131_4_reg_1025[23]_i_4_n_0 ,\add_ln131_4_reg_1025[23]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[27]_i_1 
       (.CI(\add_ln131_4_reg_1025_reg[23]_i_1_n_0 ),
        .CO({\add_ln131_4_reg_1025_reg[27]_i_1_n_0 ,\add_ln131_4_reg_1025_reg[27]_i_1_n_1 ,\add_ln131_4_reg_1025_reg[27]_i_1_n_2 ,\add_ln131_4_reg_1025_reg[27]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_78,p_reg_reg_n_79,p_reg_reg_n_80,p_reg_reg_n_81}),
        .O(D[27:24]),
        .S({\add_ln131_4_reg_1025[27]_i_2_n_0 ,\add_ln131_4_reg_1025[27]_i_3_n_0 ,\add_ln131_4_reg_1025[27]_i_4_n_0 ,\add_ln131_4_reg_1025[27]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[31]_i_1 
       (.CI(\add_ln131_4_reg_1025_reg[27]_i_1_n_0 ),
        .CO({\add_ln131_4_reg_1025_reg[31]_i_1_n_0 ,\add_ln131_4_reg_1025_reg[31]_i_1_n_1 ,\add_ln131_4_reg_1025_reg[31]_i_1_n_2 ,\add_ln131_4_reg_1025_reg[31]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({\add_ln131_4_reg_1025_reg[31] [30],DI,p_reg_reg_n_76,p_reg_reg_n_77}),
        .O(D[31:28]),
        .S({S,\add_ln131_4_reg_1025[31]_i_5_n_0 ,\add_ln131_4_reg_1025[31]_i_6_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[32]_i_1 
       (.CI(\add_ln131_4_reg_1025_reg[31]_i_1_n_0 ),
        .CO(\NLW_add_ln131_4_reg_1025_reg[32]_i_1_CO_UNCONNECTED [3:0]),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_add_ln131_4_reg_1025_reg[32]_i_1_O_UNCONNECTED [3:1],D[32]}),
        .S({1'b0,1'b0,1'b0,\add_ln131_4_reg_1025_reg[32] }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[3]_i_1 
       (.CI(1'b0),
        .CO({\add_ln131_4_reg_1025_reg[3]_i_1_n_0 ,\add_ln131_4_reg_1025_reg[3]_i_1_n_1 ,\add_ln131_4_reg_1025_reg[3]_i_1_n_2 ,\add_ln131_4_reg_1025_reg[3]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_102,p_reg_reg_n_103,p_reg_reg_n_104,p_reg_reg_n_105}),
        .O(D[3:0]),
        .S({\add_ln131_4_reg_1025[3]_i_2_n_0 ,\add_ln131_4_reg_1025[3]_i_3_n_0 ,\add_ln131_4_reg_1025[3]_i_4_n_0 ,\add_ln131_4_reg_1025[3]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \add_ln131_4_reg_1025_reg[7]_i_1 
       (.CI(\add_ln131_4_reg_1025_reg[3]_i_1_n_0 ),
        .CO({\add_ln131_4_reg_1025_reg[7]_i_1_n_0 ,\add_ln131_4_reg_1025_reg[7]_i_1_n_1 ,\add_ln131_4_reg_1025_reg[7]_i_1_n_2 ,\add_ln131_4_reg_1025_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({p_reg_reg_n_98,p_reg_reg_n_99,p_reg_reg_n_100,p_reg_reg_n_101}),
        .O(D[7:4]),
        .S({\add_ln131_4_reg_1025[7]_i_2_n_0 ,\add_ln131_4_reg_1025[7]_i_3_n_0 ,\add_ln131_4_reg_1025[7]_i_4_n_0 ,\add_ln131_4_reg_1025[7]_i_5_n_0 }));
  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b0,1'b1,1'b1,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:31],P,p_reg_reg_n_76,p_reg_reg_n_77,p_reg_reg_n_78,p_reg_reg_n_79,p_reg_reg_n_80,p_reg_reg_n_81,p_reg_reg_n_82,p_reg_reg_n_83,p_reg_reg_n_84,p_reg_reg_n_85,p_reg_reg_n_86,p_reg_reg_n_87,p_reg_reg_n_88,p_reg_reg_n_89,p_reg_reg_n_90,p_reg_reg_n_91,p_reg_reg_n_92,p_reg_reg_n_93,p_reg_reg_n_94,p_reg_reg_n_95,p_reg_reg_n_96,p_reg_reg_n_97,p_reg_reg_n_98,p_reg_reg_n_99,p_reg_reg_n_100,p_reg_reg_n_101,p_reg_reg_n_102,p_reg_reg_n_103,p_reg_reg_n_104,p_reg_reg_n_105}),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_14ns_33s_33_4_1
   (p_reg_reg,
    DI,
    S,
    p_reg_reg_0,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    ACOUT,
    PCOUT,
    P);
  output [30:0]p_reg_reg;
  output [0:0]DI;
  output [1:0]S;
  output [0:0]p_reg_reg_0;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [29:0]ACOUT;
  input [47:0]PCOUT;
  input [0:0]P;

  wire [29:0]ACOUT;
  wire [0:0]DI;
  wire [0:0]P;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire [1:0]S;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [30:0]p_reg_reg;
  wire [0:0]p_reg_reg_0;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_14ns_33s_33_4_1_DSP48_7 fsk_lpf_ama_addmuladd_16s_16s_14ns_33s_33_4_1_DSP48_7_U
       (.ACOUT(ACOUT),
        .DI(DI),
        .P(p_reg_reg),
        .PCOUT(PCOUT),
        .Q(Q),
        .S(S),
        .\add_ln131_4_reg_1025_reg[31] (P),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg_0));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_14ns_33s_33_4_1_DSP48_7
   (P,
    DI,
    S,
    p_reg_reg_0,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    ACOUT,
    PCOUT,
    \add_ln131_4_reg_1025_reg[31] );
  output [30:0]P;
  output [0:0]DI;
  output [1:0]S;
  output [0:0]p_reg_reg_0;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [29:0]ACOUT;
  input [47:0]PCOUT;
  input [0:0]\add_ln131_4_reg_1025_reg[31] ;

  wire [29:0]ACOUT;
  wire [0:0]DI;
  wire [30:0]P;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire [1:0]S;
  wire [0:0]\add_ln131_4_reg_1025_reg[31] ;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [0:0]p_reg_reg_0;
  wire p_reg_reg_n_73;
  wire p_reg_reg_n_74;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:33]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;

  LUT1 #(
    .INIT(2'h1)) 
    \add_ln131_4_reg_1025[31]_i_2 
       (.I0(P[30]),
        .O(DI));
  LUT2 #(
    .INIT(4'h9)) 
    \add_ln131_4_reg_1025[31]_i_3 
       (.I0(P[30]),
        .I1(p_reg_reg_n_74),
        .O(S[1]));
  LUT2 #(
    .INIT(4'h6)) 
    \add_ln131_4_reg_1025[31]_i_4 
       (.I0(P[30]),
        .I1(\add_ln131_4_reg_1025_reg[31] ),
        .O(S[0]));
  LUT2 #(
    .INIT(4'h9)) 
    \add_ln131_4_reg_1025[32]_i_2 
       (.I0(p_reg_reg_n_74),
        .I1(p_reg_reg_n_73),
        .O(p_reg_reg_0));
  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .MREG(1),
    .OPMODEREG(0),
    .PATTERN(48'h000000000000),
    .PREG(1),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .ACIN(ACOUT),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b1,1'b0,1'b1,1'b1,1'b1,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:33],p_reg_reg_n_73,p_reg_reg_n_74,P}),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_15ns_25s_32_4_1
   (ACOUT,
    P,
    ap_block_pp0_stage0_11001,
    ap_clk,
    A,
    Q,
    p_reg_reg);
  output [29:0]ACOUT;
  output [31:0]P;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]A;
  input [15:0]Q;
  input [16:0]p_reg_reg;

  wire [15:0]A;
  wire [29:0]ACOUT;
  wire [31:0]P;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [16:0]p_reg_reg;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_15ns_25s_32_4_1_DSP48_0 fsk_lpf_ama_addmuladd_16s_16s_15ns_25s_32_4_1_DSP48_0_U
       (.A(A),
        .ACOUT(ACOUT),
        .P(P),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_15ns_25s_32_4_1_DSP48_0
   (ACOUT,
    P,
    ap_block_pp0_stage0_11001,
    ap_clk,
    A,
    Q,
    p_reg_reg_0);
  output [29:0]ACOUT;
  output [31:0]P;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]A;
  input [15:0]Q;
  input [16:0]p_reg_reg_0;

  wire [15:0]A;
  wire [29:0]ACOUT;
  wire [24:7]C;
  wire [31:0]P;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [16:0]p_reg_reg_0;
  wire p_reg_reg_i_10_n_0;
  wire p_reg_reg_i_11_n_0;
  wire p_reg_reg_i_12_n_0;
  wire p_reg_reg_i_13_n_0;
  wire p_reg_reg_i_14_n_0;
  wire p_reg_reg_i_15_n_0;
  wire p_reg_reg_i_16_n_0;
  wire p_reg_reg_i_17_n_0;
  wire p_reg_reg_i_18_n_0;
  wire p_reg_reg_i_19_n_0;
  wire p_reg_reg_i_1_n_3;
  wire p_reg_reg_i_20_n_0;
  wire p_reg_reg_i_21_n_0;
  wire p_reg_reg_i_22_n_0;
  wire p_reg_reg_i_23_n_0;
  wire p_reg_reg_i_24_n_0;
  wire p_reg_reg_i_25_n_0;
  wire p_reg_reg_i_26_n_0;
  wire p_reg_reg_i_27_n_0;
  wire p_reg_reg_i_28_n_0;
  wire p_reg_reg_i_29_n_0;
  wire p_reg_reg_i_2_n_0;
  wire p_reg_reg_i_2_n_1;
  wire p_reg_reg_i_2_n_2;
  wire p_reg_reg_i_2_n_3;
  wire p_reg_reg_i_30_n_0;
  wire p_reg_reg_i_31_n_0;
  wire p_reg_reg_i_32_n_0;
  wire p_reg_reg_i_33_n_0;
  wire p_reg_reg_i_3_n_0;
  wire p_reg_reg_i_3_n_1;
  wire p_reg_reg_i_3_n_2;
  wire p_reg_reg_i_3_n_3;
  wire p_reg_reg_i_4_n_0;
  wire p_reg_reg_i_4_n_1;
  wire p_reg_reg_i_4_n_2;
  wire p_reg_reg_i_4_n_3;
  wire p_reg_reg_i_5_n_0;
  wire p_reg_reg_i_5_n_1;
  wire p_reg_reg_i_5_n_2;
  wire p_reg_reg_i_5_n_3;
  wire p_reg_reg_i_6_n_0;
  wire p_reg_reg_i_7_n_0;
  wire p_reg_reg_i_8_n_0;
  wire p_reg_reg_i_9_n_0;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:32]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;
  wire [3:1]NLW_p_reg_reg_i_1_CO_UNCONNECTED;
  wire [3:2]NLW_p_reg_reg_i_1_O_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
    .AUTORESET_PATDET("NO_RESET"),
    .A_INPUT("DIRECT"),
    .BCASCREG(0),
    .BREG(0),
    .B_INPUT("DIRECT"),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .CREG(0),
    .DREG(1),
    .INMODEREG(0),
    .MASK(48'h3FFFFFFFFFFF),
    .MREG(1),
    .OPMODEREG(0),
    .PATTERN(48'h000000000000),
    .PREG(1),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(ACOUT),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,1'b1,1'b1,1'b1}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C[24],C,p_reg_reg_0[4:0],1'b0,1'b0}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b1,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:32],P}),
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
  CARRY4 p_reg_reg_i_1
       (.CI(p_reg_reg_i_2_n_0),
        .CO({NLW_p_reg_reg_i_1_CO_UNCONNECTED[3:1],p_reg_reg_i_1_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,p_reg_reg_0[16]}),
        .O({NLW_p_reg_reg_i_1_O_UNCONNECTED[3:2],C[24:23]}),
        .S({1'b0,1'b0,1'b1,p_reg_reg_i_6_n_0}));
  LUT3 #(
    .INIT(8'h87)) 
    p_reg_reg_i_10
       (.I0(p_reg_reg_0[16]),
        .I1(p_reg_reg_0[11]),
        .I2(p_reg_reg_0[12]),
        .O(p_reg_reg_i_10_n_0));
  LUT2 #(
    .INIT(4'h9)) 
    p_reg_reg_i_11
       (.I0(p_reg_reg_0[16]),
        .I1(p_reg_reg_0[11]),
        .O(p_reg_reg_i_11_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_12
       (.I0(p_reg_reg_0[14]),
        .I1(p_reg_reg_0[9]),
        .O(p_reg_reg_i_12_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_13
       (.I0(p_reg_reg_0[13]),
        .I1(p_reg_reg_0[8]),
        .O(p_reg_reg_i_13_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_14
       (.I0(p_reg_reg_0[12]),
        .I1(p_reg_reg_0[7]),
        .O(p_reg_reg_i_14_n_0));
  LUT4 #(
    .INIT(16'h9699)) 
    p_reg_reg_i_15
       (.I0(p_reg_reg_0[11]),
        .I1(p_reg_reg_0[16]),
        .I2(p_reg_reg_0[10]),
        .I3(p_reg_reg_0[15]),
        .O(p_reg_reg_i_15_n_0));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_16
       (.I0(p_reg_reg_0[9]),
        .I1(p_reg_reg_0[14]),
        .I2(p_reg_reg_0[10]),
        .I3(p_reg_reg_0[15]),
        .O(p_reg_reg_i_16_n_0));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_17
       (.I0(p_reg_reg_0[8]),
        .I1(p_reg_reg_0[13]),
        .I2(p_reg_reg_0[9]),
        .I3(p_reg_reg_0[14]),
        .O(p_reg_reg_i_17_n_0));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_18
       (.I0(p_reg_reg_0[7]),
        .I1(p_reg_reg_0[12]),
        .I2(p_reg_reg_0[8]),
        .I3(p_reg_reg_0[13]),
        .O(p_reg_reg_i_18_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_19
       (.I0(p_reg_reg_0[11]),
        .I1(p_reg_reg_0[6]),
        .O(p_reg_reg_i_19_n_0));
  CARRY4 p_reg_reg_i_2
       (.CI(p_reg_reg_i_3_n_0),
        .CO({p_reg_reg_i_2_n_0,p_reg_reg_i_2_n_1,p_reg_reg_i_2_n_2,p_reg_reg_i_2_n_3}),
        .CYINIT(1'b0),
        .DI(p_reg_reg_0[15:12]),
        .O(C[22:19]),
        .S({p_reg_reg_i_7_n_0,p_reg_reg_i_8_n_0,p_reg_reg_i_9_n_0,p_reg_reg_i_10_n_0}));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_20
       (.I0(p_reg_reg_0[10]),
        .I1(p_reg_reg_0[5]),
        .O(p_reg_reg_i_20_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_21
       (.I0(p_reg_reg_0[9]),
        .I1(p_reg_reg_0[4]),
        .O(p_reg_reg_i_21_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_22
       (.I0(p_reg_reg_0[8]),
        .I1(p_reg_reg_0[3]),
        .O(p_reg_reg_i_22_n_0));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_23
       (.I0(p_reg_reg_0[6]),
        .I1(p_reg_reg_0[11]),
        .I2(p_reg_reg_0[7]),
        .I3(p_reg_reg_0[12]),
        .O(p_reg_reg_i_23_n_0));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_24
       (.I0(p_reg_reg_0[5]),
        .I1(p_reg_reg_0[10]),
        .I2(p_reg_reg_0[6]),
        .I3(p_reg_reg_0[11]),
        .O(p_reg_reg_i_24_n_0));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_25
       (.I0(p_reg_reg_0[4]),
        .I1(p_reg_reg_0[9]),
        .I2(p_reg_reg_0[5]),
        .I3(p_reg_reg_0[10]),
        .O(p_reg_reg_i_25_n_0));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_26
       (.I0(p_reg_reg_0[3]),
        .I1(p_reg_reg_0[8]),
        .I2(p_reg_reg_0[4]),
        .I3(p_reg_reg_0[9]),
        .O(p_reg_reg_i_26_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_27
       (.I0(p_reg_reg_0[7]),
        .I1(p_reg_reg_0[2]),
        .O(p_reg_reg_i_27_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    p_reg_reg_i_28
       (.I0(p_reg_reg_0[6]),
        .I1(p_reg_reg_0[1]),
        .O(p_reg_reg_i_28_n_0));
  LUT2 #(
    .INIT(4'hB)) 
    p_reg_reg_i_29
       (.I0(p_reg_reg_0[5]),
        .I1(p_reg_reg_0[0]),
        .O(p_reg_reg_i_29_n_0));
  CARRY4 p_reg_reg_i_3
       (.CI(p_reg_reg_i_4_n_0),
        .CO({p_reg_reg_i_3_n_0,p_reg_reg_i_3_n_1,p_reg_reg_i_3_n_2,p_reg_reg_i_3_n_3}),
        .CYINIT(1'b0),
        .DI({p_reg_reg_i_11_n_0,p_reg_reg_i_12_n_0,p_reg_reg_i_13_n_0,p_reg_reg_i_14_n_0}),
        .O(C[18:15]),
        .S({p_reg_reg_i_15_n_0,p_reg_reg_i_16_n_0,p_reg_reg_i_17_n_0,p_reg_reg_i_18_n_0}));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_30
       (.I0(p_reg_reg_0[2]),
        .I1(p_reg_reg_0[7]),
        .I2(p_reg_reg_0[3]),
        .I3(p_reg_reg_0[8]),
        .O(p_reg_reg_i_30_n_0));
  LUT4 #(
    .INIT(16'hB44B)) 
    p_reg_reg_i_31
       (.I0(p_reg_reg_0[1]),
        .I1(p_reg_reg_0[6]),
        .I2(p_reg_reg_0[2]),
        .I3(p_reg_reg_0[7]),
        .O(p_reg_reg_i_31_n_0));
  LUT4 #(
    .INIT(16'h2DD2)) 
    p_reg_reg_i_32
       (.I0(p_reg_reg_0[0]),
        .I1(p_reg_reg_0[5]),
        .I2(p_reg_reg_0[1]),
        .I3(p_reg_reg_0[6]),
        .O(p_reg_reg_i_32_n_0));
  LUT2 #(
    .INIT(4'h6)) 
    p_reg_reg_i_33
       (.I0(p_reg_reg_0[5]),
        .I1(p_reg_reg_0[0]),
        .O(p_reg_reg_i_33_n_0));
  CARRY4 p_reg_reg_i_4
       (.CI(p_reg_reg_i_5_n_0),
        .CO({p_reg_reg_i_4_n_0,p_reg_reg_i_4_n_1,p_reg_reg_i_4_n_2,p_reg_reg_i_4_n_3}),
        .CYINIT(1'b0),
        .DI({p_reg_reg_i_19_n_0,p_reg_reg_i_20_n_0,p_reg_reg_i_21_n_0,p_reg_reg_i_22_n_0}),
        .O(C[14:11]),
        .S({p_reg_reg_i_23_n_0,p_reg_reg_i_24_n_0,p_reg_reg_i_25_n_0,p_reg_reg_i_26_n_0}));
  CARRY4 p_reg_reg_i_5
       (.CI(1'b0),
        .CO({p_reg_reg_i_5_n_0,p_reg_reg_i_5_n_1,p_reg_reg_i_5_n_2,p_reg_reg_i_5_n_3}),
        .CYINIT(1'b0),
        .DI({p_reg_reg_i_27_n_0,p_reg_reg_i_28_n_0,p_reg_reg_i_29_n_0,1'b0}),
        .O(C[10:7]),
        .S({p_reg_reg_i_30_n_0,p_reg_reg_i_31_n_0,p_reg_reg_i_32_n_0,p_reg_reg_i_33_n_0}));
  LUT2 #(
    .INIT(4'h9)) 
    p_reg_reg_i_6
       (.I0(p_reg_reg_0[15]),
        .I1(p_reg_reg_0[16]),
        .O(p_reg_reg_i_6_n_0));
  LUT2 #(
    .INIT(4'h9)) 
    p_reg_reg_i_7
       (.I0(p_reg_reg_0[14]),
        .I1(p_reg_reg_0[15]),
        .O(p_reg_reg_i_7_n_0));
  LUT2 #(
    .INIT(4'h9)) 
    p_reg_reg_i_8
       (.I0(p_reg_reg_0[13]),
        .I1(p_reg_reg_0[14]),
        .O(p_reg_reg_i_8_n_0));
  LUT2 #(
    .INIT(4'h9)) 
    p_reg_reg_i_9
       (.I0(p_reg_reg_0[12]),
        .I1(p_reg_reg_0[13]),
        .O(p_reg_reg_i_9_n_0));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_15ns_32s_33_4_1
   (D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    A,
    ACOUT,
    P);
  output [32:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]A;
  input [29:0]ACOUT;
  input [31:0]P;

  wire [15:0]A;
  wire [29:0]ACOUT;
  wire [32:0]D;
  wire [31:0]P;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_15ns_32s_33_4_1_DSP48_6 fsk_lpf_ama_addmuladd_16s_16s_15ns_32s_33_4_1_DSP48_6_U
       (.A(A),
        .ACOUT(ACOUT),
        .D(D),
        .P(P),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_15ns_32s_33_4_1_DSP48_6
   (D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    A,
    ACOUT,
    P);
  output [32:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]A;
  input [29:0]ACOUT;
  input [31:0]P;

  wire [15:0]A;
  wire [29:0]ACOUT;
  wire [32:0]D;
  wire [31:0]P;
  wire ap_block_pp0_stage0_11001;
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
  wire [47:33]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
    .AUTORESET_PATDET("NO_RESET"),
    .A_INPUT("CASCADE"),
    .BCASCREG(0),
    .BREG(0),
    .B_INPUT("DIRECT"),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .CREG(0),
    .DREG(1),
    .INMODEREG(0),
    .MASK(48'h3FFFFFFFFFFF),
    .MREG(1),
    .OPMODEREG(0),
    .PATTERN(48'h000000000000),
    .PREG(1),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .ACIN(ACOUT),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,1'b1,1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,1'b1,1'b1,1'b0,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P[31],P}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b1,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:33],D}),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_7s_28s_28_4_1
   (D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg,
    PCOUT);
  output [27:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg;
  input [47:0]PCOUT;

  wire [27:0]D;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_7s_28s_28_4_1_DSP48_9 fsk_lpf_ama_addmuladd_16s_16s_7s_28s_28_4_1_DSP48_9_U
       (.D(D),
        .PCOUT(PCOUT),
        .Q(Q),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .p_reg_reg_0(p_reg_reg));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_ama_addmuladd_16s_16s_7s_28s_28_4_1_DSP48_9
   (D,
    ap_block_pp0_stage0_11001,
    ap_clk,
    Q,
    p_reg_reg_0,
    PCOUT);
  output [27:0]D;
  input ap_block_pp0_stage0_11001;
  input ap_clk;
  input [15:0]Q;
  input [15:0]p_reg_reg_0;
  input [47:0]PCOUT;

  wire [27:0]D;
  wire [47:0]PCOUT;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire [15:0]p_reg_reg_0;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:28]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;

  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
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
    .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"),
    .USE_PATTERN_DETECT("NO_PATDET"),
    .USE_SIMD("ONE48")) 
    p_reg_reg
       (.A({p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0[15],p_reg_reg_0}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b0,1'b1,1'b1,1'b0,1'b1,1'b0}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(ap_block_pp0_stage0_11001),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(ap_block_pp0_stage0_11001),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(1'b0),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(ap_block_pp0_stage0_11001),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q[15],Q}),
        .INMODE({1'b0,1'b0,1'b1,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:28],D}),
        .PATTERNBDETECT(NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN(PCOUT),
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

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_regslice_both
   (\B_V_data_1_state_reg[1]_0 ,
    ap_rst_n_inv,
    in_r_TVALID_int_regslice,
    D,
    ap_clk,
    \B_V_data_1_state_reg[1]_1 ,
    in_r_TVALID,
    ap_rst_n,
    in_r_TDATA);
  output \B_V_data_1_state_reg[1]_0 ;
  output ap_rst_n_inv;
  output in_r_TVALID_int_regslice;
  output [15:0]D;
  input ap_clk;
  input \B_V_data_1_state_reg[1]_1 ;
  input in_r_TVALID;
  input ap_rst_n;
  input [15:0]in_r_TDATA;

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
  wire B_V_data_1_sel_rd_i_1__0_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__1_n_0 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire \B_V_data_1_state_reg[1]_1 ;
  wire [15:0]D;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [15:0]in_r_TDATA;
  wire in_r_TVALID;
  wire in_r_TVALID_int_regslice;

  LUT3 #(
    .INIT(8'h0D)) 
    \B_V_data_1_payload_A[15]_i_1 
       (.I0(in_r_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_load_A));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[0]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[10]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[11]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[12]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[13]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[14]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[15] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[15]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[1]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[3]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[4]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[5]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[6]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[7]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[8]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_A),
        .D(in_r_TDATA[9]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .R(1'b0));
  LUT3 #(
    .INIT(8'hA2)) 
    \B_V_data_1_payload_B[15]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(in_r_TVALID_int_regslice),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .O(B_V_data_1_load_B));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[0]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[10]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[11]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[12]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[13]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[14]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[15] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[15]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[1]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[3]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[4]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[5]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[6]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[7]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[8]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_r_TDATA[9]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT3 #(
    .INIT(8'hB4)) 
    B_V_data_1_sel_rd_i_1__0
       (.I0(\B_V_data_1_state_reg[1]_1 ),
        .I1(in_r_TVALID_int_regslice),
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
    B_V_data_1_sel_wr_i_1__1
       (.I0(in_r_TVALID),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hAAA080A0)) 
    \B_V_data_1_state[0]_i_1__1 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg[1]_1 ),
        .I2(in_r_TVALID_int_regslice),
        .I3(\B_V_data_1_state_reg[1]_0 ),
        .I4(in_r_TVALID),
        .O(\B_V_data_1_state[0]_i_1__1_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_state[1]_i_1__2 
       (.I0(ap_rst_n),
        .O(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT4 #(
    .INIT(16'h77F7)) 
    \B_V_data_1_state[1]_i_2 
       (.I0(\B_V_data_1_state_reg[1]_1 ),
        .I1(in_r_TVALID_int_regslice),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(in_r_TVALID),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__1_n_0 ),
        .Q(in_r_TVALID_int_regslice),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg[1]_0 ),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[0]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .O(D[0]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[10]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(B_V_data_1_sel),
        .O(D[10]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[11]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I2(B_V_data_1_sel),
        .O(D[11]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[12]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(B_V_data_1_sel),
        .O(D[12]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[13]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I2(B_V_data_1_sel),
        .O(D[13]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[14]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I2(B_V_data_1_sel),
        .O(D[14]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[15]_i_2 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(B_V_data_1_sel),
        .O(D[15]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[1]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I2(B_V_data_1_sel),
        .O(D[1]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[2]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(B_V_data_1_sel),
        .O(D[2]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[3]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I2(B_V_data_1_sel),
        .O(D[3]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[4]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(B_V_data_1_sel),
        .O(D[4]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[5]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I2(B_V_data_1_sel),
        .O(D[5]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[6]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(B_V_data_1_sel),
        .O(D[6]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[7]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I2(B_V_data_1_sel),
        .O(D[7]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[8]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(B_V_data_1_sel),
        .O(D[8]));
  LUT3 #(
    .INIT(8'hAC)) 
    \din_data_reg_893[9]_i_1 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I2(B_V_data_1_sel),
        .O(D[9]));
endmodule

(* ORIG_REF_NAME = "fsk_lpf_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_regslice_both_2
   (\B_V_data_1_state_reg[0]_0 ,
    \B_V_data_1_state_reg[0]_1 ,
    E,
    ap_enable_reg_pp0_iter4_reg,
    ap_block_pp0_stage0_11001,
    out_r_TDATA,
    ap_rst_n_inv,
    ap_clk,
    out_r_TREADY,
    in_r_TVALID_int_regslice,
    ap_enable_reg_pp0_iter7,
    ap_enable_reg_pp0_iter8,
    ap_rst_n,
    ap_enable_reg_pp0_iter1,
    ap_enable_reg_pp0_iter4,
    icmp_ln142_reg_1061,
    icmp_ln139_reg_1055,
    Q);
  output \B_V_data_1_state_reg[0]_0 ;
  output \B_V_data_1_state_reg[0]_1 ;
  output [0:0]E;
  output [0:0]ap_enable_reg_pp0_iter4_reg;
  output ap_block_pp0_stage0_11001;
  output [15:0]out_r_TDATA;
  input ap_rst_n_inv;
  input ap_clk;
  input out_r_TREADY;
  input in_r_TVALID_int_regslice;
  input ap_enable_reg_pp0_iter7;
  input ap_enable_reg_pp0_iter8;
  input ap_rst_n;
  input ap_enable_reg_pp0_iter1;
  input ap_enable_reg_pp0_iter4;
  input icmp_ln142_reg_1061;
  input icmp_ln139_reg_1055;
  input [15:0]Q;

  wire \B_V_data_1_payload_A[15]_i_1__0_n_0 ;
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
  wire B_V_data_1_sel_rd_i_1__1_n_0;
  wire B_V_data_1_sel_rd_reg_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__0_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg[0]_1 ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire [0:0]E;
  wire [15:0]Q;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter4;
  wire [0:0]ap_enable_reg_pp0_iter4_reg;
  wire ap_enable_reg_pp0_iter7;
  wire ap_enable_reg_pp0_iter8;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire icmp_ln139_reg_1055;
  wire icmp_ln142_reg_1061;
  wire in_r_TVALID_int_regslice;
  wire [15:0]out_r_TDATA;
  wire [15:0]out_r_TDATA_int_regslice;
  wire out_r_TREADY;

  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[0]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[0]),
        .O(out_r_TDATA_int_regslice[0]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[10]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[10]),
        .O(out_r_TDATA_int_regslice[10]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[11]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[11]),
        .O(out_r_TDATA_int_regslice[11]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[12]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[12]),
        .O(out_r_TDATA_int_regslice[12]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[13]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[13]),
        .O(out_r_TDATA_int_regslice[13]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[14]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[14]),
        .O(out_r_TDATA_int_regslice[14]));
  LUT3 #(
    .INIT(8'h0B)) 
    \B_V_data_1_payload_A[15]_i_1__0 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(\B_V_data_1_payload_A[15]_i_1__0_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'h32)) 
    \B_V_data_1_payload_A[15]_i_2 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[15]),
        .O(out_r_TDATA_int_regslice[15]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[1]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[1]),
        .O(out_r_TDATA_int_regslice[1]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[2]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[2]),
        .O(out_r_TDATA_int_regslice[2]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[3]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[3]),
        .O(out_r_TDATA_int_regslice[3]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[4]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[4]),
        .O(out_r_TDATA_int_regslice[4]));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[5]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[5]),
        .O(out_r_TDATA_int_regslice[5]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[6]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[6]),
        .O(out_r_TDATA_int_regslice[6]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[7]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[7]),
        .O(out_r_TDATA_int_regslice[7]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[8]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[8]),
        .O(out_r_TDATA_int_regslice[8]));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hDC)) 
    \B_V_data_1_payload_A[9]_i_1 
       (.I0(icmp_ln142_reg_1061),
        .I1(icmp_ln139_reg_1055),
        .I2(Q[9]),
        .O(out_r_TDATA_int_regslice[9]));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[0]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[10]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[11]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[12]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[13] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[13]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[14] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[14]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[15] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[15]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[1]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[3]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[4]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[5]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[6]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[7]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[8]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[9]),
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
        .D(out_r_TDATA_int_regslice[0]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[10] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[10]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[11] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[11]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[12] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[12]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[13] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[13]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[14] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[14]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[15] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[15]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[1]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[3]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[4] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[4]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[5] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[5]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[6] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[6]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[7] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[7]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[8] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[8]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[9] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_B[15]_i_1__0_n_0 ),
        .D(out_r_TDATA_int_regslice[9]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__1
       (.I0(out_r_TREADY),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(B_V_data_1_sel_rd_i_1__1_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__1_n_0),
        .Q(B_V_data_1_sel_rd_reg_n_0),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT4 #(
    .INIT(16'hDF20)) 
    B_V_data_1_sel_wr_i_1__0
       (.I0(ap_enable_reg_pp0_iter7),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__0_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__0_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'h20A0A8A820A020A0)) 
    \B_V_data_1_state[0]_i_1 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(out_r_TREADY),
        .I4(\B_V_data_1_state_reg[0]_1 ),
        .I5(ap_enable_reg_pp0_iter7),
        .O(\B_V_data_1_state[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFBFBFBFBF3FBFBFB)) 
    \B_V_data_1_state[1]_i_1 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(out_r_TREADY),
        .I3(in_r_TVALID_int_regslice),
        .I4(ap_enable_reg_pp0_iter7),
        .I5(ap_enable_reg_pp0_iter8),
        .O(B_V_data_1_state));
  LUT6 #(
    .INIT(64'h5F55FFFF5555DDDD)) 
    \B_V_data_1_state[1]_i_3 
       (.I0(in_r_TVALID_int_regslice),
        .I1(ap_enable_reg_pp0_iter7),
        .I2(out_r_TREADY),
        .I3(\B_V_data_1_state_reg[0]_0 ),
        .I4(\B_V_data_1_state_reg_n_0_[1] ),
        .I5(ap_enable_reg_pp0_iter8),
        .O(\B_V_data_1_state_reg[0]_1 ));
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
    .INIT(4'h2)) 
    \add_ln131_1_reg_1020[32]_i_1 
       (.I0(ap_enable_reg_pp0_iter4),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .O(ap_enable_reg_pp0_iter4_reg));
  LUT6 #(
    .INIT(64'hCC4CDD5D00000000)) 
    \din_data_reg_893[15]_i_1 
       (.I0(ap_enable_reg_pp0_iter8),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(out_r_TREADY),
        .I4(ap_enable_reg_pp0_iter7),
        .I5(in_r_TVALID_int_regslice),
        .O(ap_block_pp0_stage0_11001));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \fsk_lpf_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_hls_stream_hls_axis_ap_int_16_0ul_0ul_0ul_0_delay_7[15]_i_1 
       (.I0(ap_enable_reg_pp0_iter1),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .O(E));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[0]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[0]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[10]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[10]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[11]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[11]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[12]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[12]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[13]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[13]));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[14]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[14]));
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[15]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[15]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[1]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[1]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[2]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[2]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[3]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[3]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[4]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[4]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[5]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[5]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[6]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[6]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[7]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[7]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[8]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[8]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_r_TDATA[9]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_r_TDATA[9]));
endmodule

(* ORIG_REF_NAME = "fsk_lpf_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_regslice_both__parameterized1
   (in_r_TLAST_int_regslice,
    ap_rst_n_inv,
    ap_clk,
    ap_rst_n,
    \B_V_data_1_state_reg[0]_0 ,
    in_r_TVALID,
    in_r_TLAST);
  output in_r_TLAST_int_regslice;
  input ap_rst_n_inv;
  input ap_clk;
  input ap_rst_n;
  input \B_V_data_1_state_reg[0]_0 ;
  input in_r_TVALID;
  input [0:0]in_r_TLAST;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__0_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__2_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__0_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [0:0]in_r_TLAST;
  wire in_r_TLAST_int_regslice;
  wire in_r_TVALID;

  LUT5 #(
    .INIT(32'hFFAE00A2)) 
    \B_V_data_1_payload_A[0]_i_1__0 
       (.I0(in_r_TLAST),
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
    \B_V_data_1_payload_B[0]_i_1 
       (.I0(in_r_TLAST),
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
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
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
    B_V_data_1_sel_wr_i_1__2
       (.I0(in_r_TVALID),
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
    .INIT(32'hAAA080A0)) 
    \B_V_data_1_state[0]_i_1__0 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(in_r_TVALID),
        .O(\B_V_data_1_state[0]_i_1__0_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'h77F7)) 
    \B_V_data_1_state[1]_i_1__0 
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(in_r_TVALID),
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
    \din_last_reg_898_pp0_iter5_reg_reg[0]_srl6_i_1 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(in_r_TLAST_int_regslice));
endmodule

(* ORIG_REF_NAME = "fsk_lpf_regslice_both" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_lpf_regslice_both__parameterized1_3
   (out_r_TLAST,
    ap_rst_n_inv,
    ap_clk,
    out_r_TREADY,
    \B_V_data_1_state_reg[1]_0 ,
    ap_enable_reg_pp0_iter7,
    ap_rst_n,
    din_last_reg_898_pp0_iter6_reg);
  output [0:0]out_r_TLAST;
  input ap_rst_n_inv;
  input ap_clk;
  input out_r_TREADY;
  input \B_V_data_1_state_reg[1]_0 ;
  input ap_enable_reg_pp0_iter7;
  input ap_rst_n;
  input din_last_reg_898_pp0_iter6_reg;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__1_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1__0_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__2_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__2_n_0 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter7;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire din_last_reg_898_pp0_iter6_reg;
  wire [0:0]out_r_TLAST;
  wire out_r_TREADY;

  LUT5 #(
    .INIT(32'hFFAE00A2)) 
    \B_V_data_1_payload_A[0]_i_1__1 
       (.I0(din_last_reg_898_pp0_iter6_reg),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(B_V_data_1_sel_wr),
        .I4(B_V_data_1_payload_A),
        .O(\B_V_data_1_payload_A[0]_i_1__1_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[0]_i_1__1_n_0 ),
        .Q(B_V_data_1_payload_A),
        .R(1'b0));
  LUT5 #(
    .INIT(32'hBBFB8808)) 
    \B_V_data_1_payload_B[0]_i_1__0 
       (.I0(din_last_reg_898_pp0_iter6_reg),
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
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__2
       (.I0(out_r_TREADY),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__2_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__2_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT4 #(
    .INIT(16'hDF20)) 
    B_V_data_1_sel_wr_i_1
       (.I0(ap_enable_reg_pp0_iter7),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'h0888A8A808880888)) 
    \B_V_data_1_state[0]_i_1__2 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(out_r_TREADY),
        .I4(\B_V_data_1_state_reg[1]_0 ),
        .I5(ap_enable_reg_pp0_iter7),
        .O(\B_V_data_1_state[0]_i_1__2_n_0 ));
  LUT5 #(
    .INIT(32'hFDF5FDFD)) 
    \B_V_data_1_state[1]_i_1__1 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(out_r_TREADY),
        .I3(\B_V_data_1_state_reg[1]_0 ),
        .I4(ap_enable_reg_pp0_iter7),
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
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_r_TLAST[0]_INST_0 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(out_r_TLAST));
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
