// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 12:24:46 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_fsk_discriminator_0_0/system_fsk_discriminator_0_0_sim_netlist.v
// Design      : system_fsk_discriminator_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "system_fsk_discriminator_0_0,fsk_discriminator,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "HLS" *) 
(* X_CORE_INFO = "fsk_discriminator,Vivado 2023.1" *) (* hls_module = "yes" *) 
(* NotValidForBitStream *)
module system_fsk_discriminator_0_0
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
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TDATA" *) output [15:0]out_stream_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TLAST" *) output [0:0]out_stream_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TKEEP" *) output [1:0]out_stream_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME out_stream, TDATA_NUM_BYTES 2, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) output [1:0]out_stream_TSTRB;

  wire \<const0> ;
  wire \<const1> ;
  wire ap_clk;
  wire ap_rst_n;
  wire [31:0]in_stream_TDATA;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TREADY;
  wire in_stream_TVALID;
  wire [15:0]out_stream_TDATA;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire out_stream_TVALID;
  wire [1:0]NLW_inst_out_stream_TKEEP_UNCONNECTED;
  wire [1:0]NLW_inst_out_stream_TSTRB_UNCONNECTED;

  assign out_stream_TKEEP[1] = \<const1> ;
  assign out_stream_TKEEP[0] = \<const1> ;
  assign out_stream_TSTRB[1] = \<const0> ;
  assign out_stream_TSTRB[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  VCC VCC
       (.P(\<const1> ));
  (* SDX_KERNEL = "true" *) 
  (* SDX_KERNEL_SYNTH_INST = "inst" *) 
  (* SDX_KERNEL_TYPE = "hls" *) 
  (* ap_ST_fsm_pp0_stage0 = "1'b1" *) 
  system_fsk_discriminator_0_0_fsk_discriminator inst
       (.ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .in_stream_TDATA(in_stream_TDATA),
        .in_stream_TKEEP({1'b0,1'b0,1'b0,1'b0}),
        .in_stream_TLAST(in_stream_TLAST),
        .in_stream_TREADY(in_stream_TREADY),
        .in_stream_TSTRB({1'b0,1'b0,1'b0,1'b0}),
        .in_stream_TVALID(in_stream_TVALID),
        .out_stream_TDATA(out_stream_TDATA),
        .out_stream_TKEEP(NLW_inst_out_stream_TKEEP_UNCONNECTED[1:0]),
        .out_stream_TLAST(out_stream_TLAST),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TSTRB(NLW_inst_out_stream_TSTRB_UNCONNECTED[1:0]),
        .out_stream_TVALID(out_stream_TVALID));
endmodule

(* ORIG_REF_NAME = "fsk_discriminator" *) (* ap_ST_fsm_pp0_stage0 = "1'b1" *) (* hls_module = "yes" *) 
module system_fsk_discriminator_0_0_fsk_discriminator
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
  output [15:0]out_stream_TDATA;
  output out_stream_TVALID;
  input out_stream_TREADY;
  output [1:0]out_stream_TKEEP;
  output [1:0]out_stream_TSTRB;
  output [0:0]out_stream_TLAST;

  wire \<const0> ;
  wire [0:0]add_ln49_fu_218_p2;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter2;
  wire ap_enable_reg_pp0_iter3;
  wire ap_enable_reg_pp0_iter4;
  wire ap_rst_n;
  wire ap_rst_n_inv;
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
  wire i_old0;
  wire icmp_ln23_2_fu_441_p2;
  wire icmp_ln52_fu_225_p2;
  wire icmp_ln52_reg_494;
  wire icmp_ln52_reg_494_pp0_iter1_reg;
  wire icmp_ln52_reg_494_pp0_iter2_reg;
  wire icmp_ln52_reg_494_pp0_iter3_reg;
  wire [31:0]in_stream_TDATA;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TLAST_int_regslice;
  wire in_stream_TREADY;
  wire in_stream_TVALID;
  wire in_stream_TVALID_int_regslice;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_10;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_11;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_12;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_13;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_14;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_15;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_16;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_17;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_18;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_19;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_2;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_20;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_21;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_22;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_23;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_24;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_25;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_26;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_27;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_28;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_29;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_3;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_30;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_31;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_32;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_33;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_34;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_35;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_36;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_37;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_39;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_4;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_40;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_41;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_42;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_43;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_44;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_45;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_46;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_47;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_48;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_49;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_5;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_50;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_51;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_52;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_6;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_7;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_8;
  wire mac_mulsub_16s_16s_32s_32_4_1_U2_n_9;
  wire [15:0]out_stream_TDATA;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire out_stream_TVALID;
  wire out_stream_TVALID_int_regslice;
  wire \pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2_n_0 ;
  wire pkt_last_V_reg_489_pp0_iter2_reg;
  wire [15:0]q_curr_fu_351_p3;
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
  wire regslice_both_in_stream_V_data_V_U_n_51;
  wire regslice_both_in_stream_V_data_V_U_n_52;
  wire regslice_both_in_stream_V_data_V_U_n_53;
  wire regslice_both_in_stream_V_data_V_U_n_54;
  wire regslice_both_in_stream_V_data_V_U_n_55;
  wire regslice_both_in_stream_V_data_V_U_n_56;
  wire regslice_both_in_stream_V_data_V_U_n_57;
  wire regslice_both_in_stream_V_data_V_U_n_58;
  wire regslice_both_in_stream_V_data_V_U_n_59;
  wire regslice_both_in_stream_V_data_V_U_n_60;
  wire regslice_both_in_stream_V_data_V_U_n_61;
  wire regslice_both_in_stream_V_data_V_U_n_62;
  wire regslice_both_in_stream_V_data_V_U_n_63;
  wire regslice_both_in_stream_V_data_V_U_n_64;
  wire regslice_both_in_stream_V_data_V_U_n_65;
  wire regslice_both_in_stream_V_data_V_U_n_66;
  wire regslice_both_in_stream_V_data_V_U_n_67;
  wire regslice_both_in_stream_V_data_V_U_n_68;
  wire regslice_both_in_stream_V_data_V_U_n_69;
  wire regslice_both_in_stream_V_data_V_U_n_70;
  wire regslice_both_in_stream_V_data_V_U_n_71;
  wire regslice_both_in_stream_V_data_V_U_n_72;
  wire regslice_both_in_stream_V_data_V_U_n_73;
  wire regslice_both_in_stream_V_data_V_U_n_74;
  wire regslice_both_in_stream_V_data_V_U_n_75;
  wire regslice_both_in_stream_V_data_V_U_n_76;
  wire regslice_both_in_stream_V_data_V_U_n_77;
  wire regslice_both_in_stream_V_data_V_U_n_78;
  wire regslice_both_in_stream_V_data_V_U_n_79;
  wire regslice_both_in_stream_V_data_V_U_n_80;
  wire regslice_both_in_stream_V_data_V_U_n_81;
  wire regslice_both_in_stream_V_data_V_U_n_82;
  wire regslice_both_in_stream_V_data_V_U_n_83;
  wire regslice_both_in_stream_V_data_V_U_n_84;
  wire regslice_both_in_stream_V_data_V_U_n_85;
  wire regslice_both_in_stream_V_data_V_U_n_86;
  wire regslice_both_in_stream_V_data_V_U_n_87;
  wire regslice_both_in_stream_V_data_V_U_n_88;
  wire regslice_both_in_stream_V_data_V_U_n_89;
  wire regslice_both_in_stream_V_data_V_U_n_90;
  wire regslice_both_in_stream_V_data_V_U_n_91;
  wire regslice_both_in_stream_V_data_V_U_n_92;
  wire regslice_both_in_stream_V_data_V_U_n_93;
  wire regslice_both_in_stream_V_data_V_U_n_94;
  wire regslice_both_in_stream_V_data_V_U_n_95;
  wire regslice_both_in_stream_V_data_V_U_n_96;
  wire regslice_both_in_stream_V_data_V_U_n_97;
  wire regslice_both_in_stream_V_data_V_U_n_98;
  wire regslice_both_out_stream_V_data_V_U_n_2;
  wire regslice_both_out_stream_V_data_V_U_n_5;
  wire regslice_both_out_stream_V_data_V_U_n_6;
  wire [31:0]sum_i_reg;
  wire [31:0]sum_q_reg;
  wire term1_reg_518_reg_n_106;
  wire term1_reg_518_reg_n_107;
  wire term1_reg_518_reg_n_108;
  wire term1_reg_518_reg_n_109;
  wire term1_reg_518_reg_n_110;
  wire term1_reg_518_reg_n_111;
  wire term1_reg_518_reg_n_112;
  wire term1_reg_518_reg_n_113;
  wire term1_reg_518_reg_n_114;
  wire term1_reg_518_reg_n_115;
  wire term1_reg_518_reg_n_116;
  wire term1_reg_518_reg_n_117;
  wire term1_reg_518_reg_n_118;
  wire term1_reg_518_reg_n_119;
  wire term1_reg_518_reg_n_120;
  wire term1_reg_518_reg_n_121;
  wire term1_reg_518_reg_n_122;
  wire term1_reg_518_reg_n_123;
  wire term1_reg_518_reg_n_124;
  wire term1_reg_518_reg_n_125;
  wire term1_reg_518_reg_n_126;
  wire term1_reg_518_reg_n_127;
  wire term1_reg_518_reg_n_128;
  wire term1_reg_518_reg_n_129;
  wire term1_reg_518_reg_n_130;
  wire term1_reg_518_reg_n_131;
  wire term1_reg_518_reg_n_132;
  wire term1_reg_518_reg_n_133;
  wire term1_reg_518_reg_n_134;
  wire term1_reg_518_reg_n_135;
  wire term1_reg_518_reg_n_136;
  wire term1_reg_518_reg_n_137;
  wire term1_reg_518_reg_n_138;
  wire term1_reg_518_reg_n_139;
  wire term1_reg_518_reg_n_140;
  wire term1_reg_518_reg_n_141;
  wire term1_reg_518_reg_n_142;
  wire term1_reg_518_reg_n_143;
  wire term1_reg_518_reg_n_144;
  wire term1_reg_518_reg_n_145;
  wire term1_reg_518_reg_n_146;
  wire term1_reg_518_reg_n_147;
  wire term1_reg_518_reg_n_148;
  wire term1_reg_518_reg_n_149;
  wire term1_reg_518_reg_n_150;
  wire term1_reg_518_reg_n_151;
  wire term1_reg_518_reg_n_152;
  wire term1_reg_518_reg_n_153;
  wire [10:0]tmp_2_fu_426_p4;
  wire [3:3]\NLW_counter_reg[28]_i_1_CO_UNCONNECTED ;
  wire NLW_term1_reg_518_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_term1_reg_518_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_term1_reg_518_reg_OVERFLOW_UNCONNECTED;
  wire NLW_term1_reg_518_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_term1_reg_518_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_term1_reg_518_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_term1_reg_518_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_term1_reg_518_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_term1_reg_518_reg_CARRYOUT_UNCONNECTED;
  wire [47:0]NLW_term1_reg_518_reg_P_UNCONNECTED;

  assign out_stream_TKEEP[1] = \<const0> ;
  assign out_stream_TKEEP[0] = \<const0> ;
  assign out_stream_TSTRB[1] = \<const0> ;
  assign out_stream_TSTRB[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
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
  LUT1 #(
    .INIT(2'h1)) 
    \counter[0]_i_2 
       (.I0(counter_reg[0]),
        .O(add_ln49_fu_218_p2));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[0]_i_1_n_7 ),
        .Q(counter_reg[0]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \counter_reg[0]_i_1 
       (.CI(1'b0),
        .CO({\counter_reg[0]_i_1_n_0 ,\counter_reg[0]_i_1_n_1 ,\counter_reg[0]_i_1_n_2 ,\counter_reg[0]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O({\counter_reg[0]_i_1_n_4 ,\counter_reg[0]_i_1_n_5 ,\counter_reg[0]_i_1_n_6 ,\counter_reg[0]_i_1_n_7 }),
        .S({counter_reg[3:1],add_ln49_fu_218_p2}));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[8]_i_1_n_5 ),
        .Q(counter_reg[10]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[8]_i_1_n_4 ),
        .Q(counter_reg[11]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[12]_i_1_n_7 ),
        .Q(counter_reg[12]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
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
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[12]_i_1_n_6 ),
        .Q(counter_reg[13]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[12]_i_1_n_5 ),
        .Q(counter_reg[14]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[12]_i_1_n_4 ),
        .Q(counter_reg[15]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[16]_i_1_n_7 ),
        .Q(counter_reg[16]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
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
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[16]_i_1_n_6 ),
        .Q(counter_reg[17]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[16]_i_1_n_5 ),
        .Q(counter_reg[18]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[16]_i_1_n_4 ),
        .Q(counter_reg[19]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[0]_i_1_n_6 ),
        .Q(counter_reg[1]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[20]_i_1_n_7 ),
        .Q(counter_reg[20]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
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
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[20]_i_1_n_6 ),
        .Q(counter_reg[21]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[20]_i_1_n_5 ),
        .Q(counter_reg[22]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[20]_i_1_n_4 ),
        .Q(counter_reg[23]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[24]_i_1_n_7 ),
        .Q(counter_reg[24]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
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
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[24]_i_1_n_6 ),
        .Q(counter_reg[25]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[24]_i_1_n_5 ),
        .Q(counter_reg[26]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[24]_i_1_n_4 ),
        .Q(counter_reg[27]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[28]_i_1_n_7 ),
        .Q(counter_reg[28]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
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
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[28]_i_1_n_6 ),
        .Q(counter_reg[29]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[0]_i_1_n_5 ),
        .Q(counter_reg[2]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[30] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[28]_i_1_n_5 ),
        .Q(counter_reg[30]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[31] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[28]_i_1_n_4 ),
        .Q(counter_reg[31]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[0]_i_1_n_4 ),
        .Q(counter_reg[3]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[4]_i_1_n_7 ),
        .Q(counter_reg[4]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
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
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[4]_i_1_n_6 ),
        .Q(counter_reg[5]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[4]_i_1_n_5 ),
        .Q(counter_reg[6]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[4]_i_1_n_4 ),
        .Q(counter_reg[7]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \counter_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[8]_i_1_n_7 ),
        .Q(counter_reg[8]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
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
        .CE(ap_block_pp0_stage0_11001),
        .D(\counter_reg[8]_i_1_n_6 ),
        .Q(counter_reg[9]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  LUT4 #(
    .INIT(16'h4000)) 
    \icmp_ln52_reg_494[0]_i_1 
       (.I0(mac_mulsub_16s_16s_32s_32_4_1_U2_n_4),
        .I1(mac_mulsub_16s_16s_32s_32_4_1_U2_n_5),
        .I2(mac_mulsub_16s_16s_32s_32_4_1_U2_n_2),
        .I3(mac_mulsub_16s_16s_32s_32_4_1_U2_n_3),
        .O(icmp_ln52_fu_225_p2));
  FDRE \icmp_ln52_reg_494_pp0_iter1_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln52_reg_494),
        .Q(icmp_ln52_reg_494_pp0_iter1_reg),
        .R(1'b0));
  FDRE \icmp_ln52_reg_494_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln52_reg_494_pp0_iter1_reg),
        .Q(icmp_ln52_reg_494_pp0_iter2_reg),
        .R(1'b0));
  FDRE \icmp_ln52_reg_494_pp0_iter3_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln52_reg_494_pp0_iter2_reg),
        .Q(icmp_ln52_reg_494_pp0_iter3_reg),
        .R(1'b0));
  FDRE \icmp_ln52_reg_494_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(icmp_ln52_fu_225_p2),
        .Q(icmp_ln52_reg_494),
        .R(1'b0));
  system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1 mac_mulsub_16s_16s_32s_32_4_1_U2
       (.A({regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_20,regslice_both_in_stream_V_data_V_U_n_21,regslice_both_in_stream_V_data_V_U_n_22,regslice_both_in_stream_V_data_V_U_n_23,regslice_both_in_stream_V_data_V_U_n_24,regslice_both_in_stream_V_data_V_U_n_25,regslice_both_in_stream_V_data_V_U_n_26,regslice_both_in_stream_V_data_V_U_n_27,regslice_both_in_stream_V_data_V_U_n_28,regslice_both_in_stream_V_data_V_U_n_29,regslice_both_in_stream_V_data_V_U_n_30,regslice_both_in_stream_V_data_V_U_n_31,regslice_both_in_stream_V_data_V_U_n_32,regslice_both_in_stream_V_data_V_U_n_33,regslice_both_in_stream_V_data_V_U_n_34}),
        .B(q_curr_fu_351_p3),
        .CO(icmp_ln23_2_fu_441_p2),
        .P({tmp_2_fu_426_p4[10],tmp_2_fu_426_p4[0]}),
        .PCOUT({term1_reg_518_reg_n_106,term1_reg_518_reg_n_107,term1_reg_518_reg_n_108,term1_reg_518_reg_n_109,term1_reg_518_reg_n_110,term1_reg_518_reg_n_111,term1_reg_518_reg_n_112,term1_reg_518_reg_n_113,term1_reg_518_reg_n_114,term1_reg_518_reg_n_115,term1_reg_518_reg_n_116,term1_reg_518_reg_n_117,term1_reg_518_reg_n_118,term1_reg_518_reg_n_119,term1_reg_518_reg_n_120,term1_reg_518_reg_n_121,term1_reg_518_reg_n_122,term1_reg_518_reg_n_123,term1_reg_518_reg_n_124,term1_reg_518_reg_n_125,term1_reg_518_reg_n_126,term1_reg_518_reg_n_127,term1_reg_518_reg_n_128,term1_reg_518_reg_n_129,term1_reg_518_reg_n_130,term1_reg_518_reg_n_131,term1_reg_518_reg_n_132,term1_reg_518_reg_n_133,term1_reg_518_reg_n_134,term1_reg_518_reg_n_135,term1_reg_518_reg_n_136,term1_reg_518_reg_n_137,term1_reg_518_reg_n_138,term1_reg_518_reg_n_139,term1_reg_518_reg_n_140,term1_reg_518_reg_n_141,term1_reg_518_reg_n_142,term1_reg_518_reg_n_143,term1_reg_518_reg_n_144,term1_reg_518_reg_n_145,term1_reg_518_reg_n_146,term1_reg_518_reg_n_147,term1_reg_518_reg_n_148,term1_reg_518_reg_n_149,term1_reg_518_reg_n_150,term1_reg_518_reg_n_151,term1_reg_518_reg_n_152,term1_reg_518_reg_n_153}),
        .S({mac_mulsub_16s_16s_32s_32_4_1_U2_n_6,mac_mulsub_16s_16s_32s_32_4_1_U2_n_7,mac_mulsub_16s_16s_32s_32_4_1_U2_n_8}),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .\counter_reg[12] (mac_mulsub_16s_16s_32s_32_4_1_U2_n_2),
        .\counter_reg[16] (mac_mulsub_16s_16s_32s_32_4_1_U2_n_4),
        .\counter_reg[28] (mac_mulsub_16s_16s_32s_32_4_1_U2_n_5),
        .\counter_reg[8] (mac_mulsub_16s_16s_32s_32_4_1_U2_n_3),
        .out(counter_reg),
        .p_reg_reg(mac_mulsub_16s_16s_32s_32_4_1_U2_n_36),
        .p_reg_reg_0(mac_mulsub_16s_16s_32s_32_4_1_U2_n_37),
        .p_reg_reg_1(mac_mulsub_16s_16s_32s_32_4_1_U2_n_39),
        .p_reg_reg_10(mac_mulsub_16s_16s_32s_32_4_1_U2_n_48),
        .p_reg_reg_11(mac_mulsub_16s_16s_32s_32_4_1_U2_n_49),
        .p_reg_reg_12(mac_mulsub_16s_16s_32s_32_4_1_U2_n_50),
        .p_reg_reg_13(mac_mulsub_16s_16s_32s_32_4_1_U2_n_51),
        .p_reg_reg_14(mac_mulsub_16s_16s_32s_32_4_1_U2_n_52),
        .p_reg_reg_15(regslice_both_out_stream_V_data_V_U_n_6),
        .p_reg_reg_2(mac_mulsub_16s_16s_32s_32_4_1_U2_n_40),
        .p_reg_reg_3(mac_mulsub_16s_16s_32s_32_4_1_U2_n_41),
        .p_reg_reg_4(mac_mulsub_16s_16s_32s_32_4_1_U2_n_42),
        .p_reg_reg_5(mac_mulsub_16s_16s_32s_32_4_1_U2_n_43),
        .p_reg_reg_6(mac_mulsub_16s_16s_32s_32_4_1_U2_n_44),
        .p_reg_reg_7(mac_mulsub_16s_16s_32s_32_4_1_U2_n_45),
        .p_reg_reg_8(mac_mulsub_16s_16s_32s_32_4_1_U2_n_46),
        .p_reg_reg_9(mac_mulsub_16s_16s_32s_32_4_1_U2_n_47),
        .sum_i_reg(sum_i_reg[31:16]),
        .\sum_i_reg[22] ({mac_mulsub_16s_16s_32s_32_4_1_U2_n_9,mac_mulsub_16s_16s_32s_32_4_1_U2_n_10,mac_mulsub_16s_16s_32s_32_4_1_U2_n_11,mac_mulsub_16s_16s_32s_32_4_1_U2_n_12}),
        .\sum_i_reg[26] ({mac_mulsub_16s_16s_32s_32_4_1_U2_n_13,mac_mulsub_16s_16s_32s_32_4_1_U2_n_14,mac_mulsub_16s_16s_32s_32_4_1_U2_n_15,mac_mulsub_16s_16s_32s_32_4_1_U2_n_16}),
        .\sum_i_reg[30] ({mac_mulsub_16s_16s_32s_32_4_1_U2_n_17,mac_mulsub_16s_16s_32s_32_4_1_U2_n_18,mac_mulsub_16s_16s_32s_32_4_1_U2_n_19,mac_mulsub_16s_16s_32s_32_4_1_U2_n_20}),
        .sum_q_reg(sum_q_reg[31:16]),
        .\sum_q_reg[18] ({mac_mulsub_16s_16s_32s_32_4_1_U2_n_21,mac_mulsub_16s_16s_32s_32_4_1_U2_n_22,mac_mulsub_16s_16s_32s_32_4_1_U2_n_23}),
        .\sum_q_reg[22] ({mac_mulsub_16s_16s_32s_32_4_1_U2_n_24,mac_mulsub_16s_16s_32s_32_4_1_U2_n_25,mac_mulsub_16s_16s_32s_32_4_1_U2_n_26,mac_mulsub_16s_16s_32s_32_4_1_U2_n_27}),
        .\sum_q_reg[26] ({mac_mulsub_16s_16s_32s_32_4_1_U2_n_28,mac_mulsub_16s_16s_32s_32_4_1_U2_n_29,mac_mulsub_16s_16s_32s_32_4_1_U2_n_30,mac_mulsub_16s_16s_32s_32_4_1_U2_n_31}),
        .\sum_q_reg[30] ({mac_mulsub_16s_16s_32s_32_4_1_U2_n_32,mac_mulsub_16s_16s_32s_32_4_1_U2_n_33,mac_mulsub_16s_16s_32s_32_4_1_U2_n_34,mac_mulsub_16s_16s_32s_32_4_1_U2_n_35}));
  (* srl_bus_name = "inst/\\pkt_last_V_reg_489_pp0_iter1_reg_reg " *) 
  (* srl_name = "inst/\\pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2 " *) 
  SRL16E \pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2 
       (.A0(1'b1),
        .A1(1'b0),
        .A2(1'b0),
        .A3(1'b0),
        .CE(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D(in_stream_TLAST_int_regslice),
        .Q(\pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2_n_0 ));
  FDRE \pkt_last_V_reg_489_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(\pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2_n_0 ),
        .Q(pkt_last_V_reg_489_pp0_iter2_reg),
        .R(1'b0));
  system_fsk_discriminator_0_0_fsk_discriminator_regslice_both regslice_both_in_stream_V_data_V_U
       (.A({regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_20,regslice_both_in_stream_V_data_V_U_n_21,regslice_both_in_stream_V_data_V_U_n_22,regslice_both_in_stream_V_data_V_U_n_23,regslice_both_in_stream_V_data_V_U_n_24,regslice_both_in_stream_V_data_V_U_n_25,regslice_both_in_stream_V_data_V_U_n_26,regslice_both_in_stream_V_data_V_U_n_27,regslice_both_in_stream_V_data_V_U_n_28,regslice_both_in_stream_V_data_V_U_n_29,regslice_both_in_stream_V_data_V_U_n_30,regslice_both_in_stream_V_data_V_U_n_31,regslice_both_in_stream_V_data_V_U_n_32,regslice_both_in_stream_V_data_V_U_n_33,regslice_both_in_stream_V_data_V_U_n_34}),
        .B(q_curr_fu_351_p3),
        .\B_V_data_1_state_reg[1]_0 (in_stream_TREADY),
        .\B_V_data_1_state_reg[1]_1 (regslice_both_out_stream_V_data_V_U_n_2),
        .O({regslice_both_in_stream_V_data_V_U_n_35,regslice_both_in_stream_V_data_V_U_n_36,regslice_both_in_stream_V_data_V_U_n_37,regslice_both_in_stream_V_data_V_U_n_38}),
        .S({mac_mulsub_16s_16s_32s_32_4_1_U2_n_6,mac_mulsub_16s_16s_32s_32_4_1_U2_n_7,mac_mulsub_16s_16s_32s_32_4_1_U2_n_8}),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_stream_TDATA(in_stream_TDATA),
        .in_stream_TVALID(in_stream_TVALID),
        .in_stream_TVALID_int_regslice(in_stream_TVALID_int_regslice),
        .p_reg_reg({mac_mulsub_16s_16s_32s_32_4_1_U2_n_9,mac_mulsub_16s_16s_32s_32_4_1_U2_n_10,mac_mulsub_16s_16s_32s_32_4_1_U2_n_11,mac_mulsub_16s_16s_32s_32_4_1_U2_n_12}),
        .p_reg_reg_0({mac_mulsub_16s_16s_32s_32_4_1_U2_n_13,mac_mulsub_16s_16s_32s_32_4_1_U2_n_14,mac_mulsub_16s_16s_32s_32_4_1_U2_n_15,mac_mulsub_16s_16s_32s_32_4_1_U2_n_16}),
        .p_reg_reg_1({mac_mulsub_16s_16s_32s_32_4_1_U2_n_21,mac_mulsub_16s_16s_32s_32_4_1_U2_n_22,mac_mulsub_16s_16s_32s_32_4_1_U2_n_23}),
        .p_reg_reg_2({mac_mulsub_16s_16s_32s_32_4_1_U2_n_24,mac_mulsub_16s_16s_32s_32_4_1_U2_n_25,mac_mulsub_16s_16s_32s_32_4_1_U2_n_26,mac_mulsub_16s_16s_32s_32_4_1_U2_n_27}),
        .p_reg_reg_3({mac_mulsub_16s_16s_32s_32_4_1_U2_n_28,mac_mulsub_16s_16s_32s_32_4_1_U2_n_29,mac_mulsub_16s_16s_32s_32_4_1_U2_n_30,mac_mulsub_16s_16s_32s_32_4_1_U2_n_31}),
        .sum_i_reg(sum_i_reg),
        .\sum_i_reg[11] ({regslice_both_in_stream_V_data_V_U_n_43,regslice_both_in_stream_V_data_V_U_n_44,regslice_both_in_stream_V_data_V_U_n_45,regslice_both_in_stream_V_data_V_U_n_46}),
        .\sum_i_reg[15] ({regslice_both_in_stream_V_data_V_U_n_47,regslice_both_in_stream_V_data_V_U_n_48,regslice_both_in_stream_V_data_V_U_n_49,regslice_both_in_stream_V_data_V_U_n_50}),
        .\sum_i_reg[19] ({regslice_both_in_stream_V_data_V_U_n_51,regslice_both_in_stream_V_data_V_U_n_52,regslice_both_in_stream_V_data_V_U_n_53,regslice_both_in_stream_V_data_V_U_n_54}),
        .\sum_i_reg[23] ({regslice_both_in_stream_V_data_V_U_n_55,regslice_both_in_stream_V_data_V_U_n_56,regslice_both_in_stream_V_data_V_U_n_57,regslice_both_in_stream_V_data_V_U_n_58}),
        .\sum_i_reg[27] ({regslice_both_in_stream_V_data_V_U_n_59,regslice_both_in_stream_V_data_V_U_n_60,regslice_both_in_stream_V_data_V_U_n_61,regslice_both_in_stream_V_data_V_U_n_62}),
        .\sum_i_reg[30] ({regslice_both_in_stream_V_data_V_U_n_63,regslice_both_in_stream_V_data_V_U_n_64,regslice_both_in_stream_V_data_V_U_n_65,regslice_both_in_stream_V_data_V_U_n_66}),
        .\sum_i_reg[7] ({regslice_both_in_stream_V_data_V_U_n_39,regslice_both_in_stream_V_data_V_U_n_40,regslice_both_in_stream_V_data_V_U_n_41,regslice_both_in_stream_V_data_V_U_n_42}),
        .sum_q_reg(sum_q_reg),
        .\sum_q_reg[11] ({regslice_both_in_stream_V_data_V_U_n_75,regslice_both_in_stream_V_data_V_U_n_76,regslice_both_in_stream_V_data_V_U_n_77,regslice_both_in_stream_V_data_V_U_n_78}),
        .\sum_q_reg[15] ({regslice_both_in_stream_V_data_V_U_n_79,regslice_both_in_stream_V_data_V_U_n_80,regslice_both_in_stream_V_data_V_U_n_81,regslice_both_in_stream_V_data_V_U_n_82}),
        .\sum_q_reg[19] ({regslice_both_in_stream_V_data_V_U_n_83,regslice_both_in_stream_V_data_V_U_n_84,regslice_both_in_stream_V_data_V_U_n_85,regslice_both_in_stream_V_data_V_U_n_86}),
        .\sum_q_reg[23] ({regslice_both_in_stream_V_data_V_U_n_87,regslice_both_in_stream_V_data_V_U_n_88,regslice_both_in_stream_V_data_V_U_n_89,regslice_both_in_stream_V_data_V_U_n_90}),
        .\sum_q_reg[27] ({regslice_both_in_stream_V_data_V_U_n_91,regslice_both_in_stream_V_data_V_U_n_92,regslice_both_in_stream_V_data_V_U_n_93,regslice_both_in_stream_V_data_V_U_n_94}),
        .\sum_q_reg[30] ({regslice_both_in_stream_V_data_V_U_n_95,regslice_both_in_stream_V_data_V_U_n_96,regslice_both_in_stream_V_data_V_U_n_97,regslice_both_in_stream_V_data_V_U_n_98}),
        .\sum_q_reg[3] ({regslice_both_in_stream_V_data_V_U_n_67,regslice_both_in_stream_V_data_V_U_n_68,regslice_both_in_stream_V_data_V_U_n_69,regslice_both_in_stream_V_data_V_U_n_70}),
        .\sum_q_reg[7] ({regslice_both_in_stream_V_data_V_U_n_71,regslice_both_in_stream_V_data_V_U_n_72,regslice_both_in_stream_V_data_V_U_n_73,regslice_both_in_stream_V_data_V_U_n_74}),
        .term1_reg_518_reg_i_70_0({mac_mulsub_16s_16s_32s_32_4_1_U2_n_32,mac_mulsub_16s_16s_32s_32_4_1_U2_n_33,mac_mulsub_16s_16s_32s_32_4_1_U2_n_34,mac_mulsub_16s_16s_32s_32_4_1_U2_n_35}),
        .term1_reg_518_reg_i_99_0({mac_mulsub_16s_16s_32s_32_4_1_U2_n_17,mac_mulsub_16s_16s_32s_32_4_1_U2_n_18,mac_mulsub_16s_16s_32s_32_4_1_U2_n_19,mac_mulsub_16s_16s_32s_32_4_1_U2_n_20}));
  system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1 regslice_both_in_stream_V_last_V_U
       (.\B_V_data_1_state_reg[1]_0 (regslice_both_out_stream_V_data_V_U_n_2),
        .ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_stream_TLAST(in_stream_TLAST),
        .in_stream_TLAST_int_regslice(in_stream_TLAST_int_regslice),
        .in_stream_TVALID(in_stream_TVALID));
  system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized2 regslice_both_out_stream_V_data_V_U
       (.\B_V_data_1_payload_A_reg[0]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_52),
        .\B_V_data_1_payload_A_reg[10]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_42),
        .\B_V_data_1_payload_A_reg[11]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_41),
        .\B_V_data_1_payload_A_reg[12]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_40),
        .\B_V_data_1_payload_A_reg[13]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_39),
        .\B_V_data_1_payload_A_reg[14]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_37),
        .\B_V_data_1_payload_A_reg[15]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_36),
        .\B_V_data_1_payload_A_reg[1]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_51),
        .\B_V_data_1_payload_A_reg[2]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_50),
        .\B_V_data_1_payload_A_reg[3]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_49),
        .\B_V_data_1_payload_A_reg[4]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_48),
        .\B_V_data_1_payload_A_reg[5]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_47),
        .\B_V_data_1_payload_A_reg[6]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_46),
        .\B_V_data_1_payload_A_reg[7]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_45),
        .\B_V_data_1_payload_A_reg[8]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_44),
        .\B_V_data_1_payload_A_reg[9]_0 (mac_mulsub_16s_16s_32s_32_4_1_U2_n_43),
        .\B_V_data_1_state_reg[0]_0 (out_stream_TVALID),
        .\B_V_data_1_state_reg[0]_1 (regslice_both_out_stream_V_data_V_U_n_2),
        .\B_V_data_1_state_reg[0]_2 (regslice_both_out_stream_V_data_V_U_n_6),
        .CO(icmp_ln23_2_fu_441_p2),
        .P({tmp_2_fu_426_p4[10],tmp_2_fu_426_p4[0]}),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter1(ap_enable_reg_pp0_iter1),
        .ap_enable_reg_pp0_iter3(ap_enable_reg_pp0_iter3),
        .ap_enable_reg_pp0_iter4(ap_enable_reg_pp0_iter4),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .i_old0(i_old0),
        .icmp_ln52_reg_494(icmp_ln52_reg_494),
        .icmp_ln52_reg_494_pp0_iter2_reg(icmp_ln52_reg_494_pp0_iter2_reg),
        .icmp_ln52_reg_494_pp0_iter3_reg(icmp_ln52_reg_494_pp0_iter3_reg),
        .\icmp_ln52_reg_494_reg[0] (regslice_both_out_stream_V_data_V_U_n_5),
        .in_stream_TVALID_int_regslice(in_stream_TVALID_int_regslice),
        .out_stream_TDATA(out_stream_TDATA),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TVALID_int_regslice(out_stream_TVALID_int_regslice),
        .p_reg_reg(mac_mulsub_16s_16s_32s_32_4_1_U2_n_3),
        .p_reg_reg_0(mac_mulsub_16s_16s_32s_32_4_1_U2_n_2),
        .p_reg_reg_1(mac_mulsub_16s_16s_32s_32_4_1_U2_n_5),
        .p_reg_reg_2(mac_mulsub_16s_16s_32s_32_4_1_U2_n_4));
  system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1_0 regslice_both_out_stream_V_last_V_U
       (.ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .out_stream_TLAST(out_stream_TLAST),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TVALID_int_regslice(out_stream_TVALID_int_regslice),
        .pkt_last_V_reg_489_pp0_iter2_reg(pkt_last_V_reg_489_pp0_iter2_reg));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_38),
        .Q(sum_i_reg[0]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_44),
        .Q(sum_i_reg[10]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_43),
        .Q(sum_i_reg[11]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_50),
        .Q(sum_i_reg[12]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_49),
        .Q(sum_i_reg[13]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_48),
        .Q(sum_i_reg[14]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_47),
        .Q(sum_i_reg[15]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_54),
        .Q(sum_i_reg[16]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_53),
        .Q(sum_i_reg[17]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_52),
        .Q(sum_i_reg[18]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_51),
        .Q(sum_i_reg[19]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_37),
        .Q(sum_i_reg[1]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_58),
        .Q(sum_i_reg[20]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_57),
        .Q(sum_i_reg[21]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_56),
        .Q(sum_i_reg[22]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_55),
        .Q(sum_i_reg[23]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_62),
        .Q(sum_i_reg[24]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[25] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_61),
        .Q(sum_i_reg[25]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_60),
        .Q(sum_i_reg[26]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_59),
        .Q(sum_i_reg[27]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_66),
        .Q(sum_i_reg[28]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[29] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_65),
        .Q(sum_i_reg[29]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_36),
        .Q(sum_i_reg[2]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[30] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_64),
        .Q(sum_i_reg[30]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[31] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_63),
        .Q(sum_i_reg[31]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_35),
        .Q(sum_i_reg[3]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_42),
        .Q(sum_i_reg[4]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_41),
        .Q(sum_i_reg[5]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_40),
        .Q(sum_i_reg[6]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_39),
        .Q(sum_i_reg[7]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_46),
        .Q(sum_i_reg[8]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_i_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_45),
        .Q(sum_i_reg[9]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[0] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_70),
        .Q(sum_q_reg[0]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[10] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_76),
        .Q(sum_q_reg[10]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[11] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_75),
        .Q(sum_q_reg[11]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[12] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_82),
        .Q(sum_q_reg[12]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[13] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_81),
        .Q(sum_q_reg[13]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[14] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_80),
        .Q(sum_q_reg[14]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[15] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_79),
        .Q(sum_q_reg[15]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[16] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_86),
        .Q(sum_q_reg[16]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[17] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_85),
        .Q(sum_q_reg[17]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[18] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_84),
        .Q(sum_q_reg[18]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[19] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_83),
        .Q(sum_q_reg[19]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[1] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_69),
        .Q(sum_q_reg[1]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[20] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_90),
        .Q(sum_q_reg[20]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[21] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_89),
        .Q(sum_q_reg[21]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[22] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_88),
        .Q(sum_q_reg[22]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[23] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_87),
        .Q(sum_q_reg[23]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[24] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_94),
        .Q(sum_q_reg[24]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[25] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_93),
        .Q(sum_q_reg[25]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[26] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_92),
        .Q(sum_q_reg[26]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[27] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_91),
        .Q(sum_q_reg[27]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[28] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_98),
        .Q(sum_q_reg[28]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[29] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_97),
        .Q(sum_q_reg[29]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[2] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_68),
        .Q(sum_q_reg[2]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[30] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_96),
        .Q(sum_q_reg[30]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[31] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_95),
        .Q(sum_q_reg[31]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[3] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_67),
        .Q(sum_q_reg[3]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[4] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_74),
        .Q(sum_q_reg[4]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[5] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_73),
        .Q(sum_q_reg[5]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[6] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_72),
        .Q(sum_q_reg[6]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[7] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_71),
        .Q(sum_q_reg[7]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[8] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_78),
        .Q(sum_q_reg[8]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  FDRE #(
    .INIT(1'b0)) 
    \sum_q_reg[9] 
       (.C(ap_clk),
        .CE(ap_block_pp0_stage0_11001),
        .D(regslice_both_in_stream_V_data_V_U_n_77),
        .Q(sum_q_reg[9]),
        .R(regslice_both_out_stream_V_data_V_U_n_6));
  DSP48E1 #(
    .ACASCREG(2),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(2),
    .AUTORESET_PATDET("NO_RESET"),
    .A_INPUT("DIRECT"),
    .BCASCREG(1),
    .BREG(1),
    .B_INPUT("DIRECT"),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .CREG(0),
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
    term1_reg_518_reg
       (.A({regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_19,regslice_both_in_stream_V_data_V_U_n_20,regslice_both_in_stream_V_data_V_U_n_21,regslice_both_in_stream_V_data_V_U_n_22,regslice_both_in_stream_V_data_V_U_n_23,regslice_both_in_stream_V_data_V_U_n_24,regslice_both_in_stream_V_data_V_U_n_25,regslice_both_in_stream_V_data_V_U_n_26,regslice_both_in_stream_V_data_V_U_n_27,regslice_both_in_stream_V_data_V_U_n_28,regslice_both_in_stream_V_data_V_U_n_29,regslice_both_in_stream_V_data_V_U_n_30,regslice_both_in_stream_V_data_V_U_n_31,regslice_both_in_stream_V_data_V_U_n_32,regslice_both_in_stream_V_data_V_U_n_33,regslice_both_in_stream_V_data_V_U_n_34}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_term1_reg_518_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b0,1'b0}),
        .B({q_curr_fu_351_p3[15],q_curr_fu_351_p3[15],q_curr_fu_351_p3}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_term1_reg_518_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_term1_reg_518_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_term1_reg_518_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(regslice_both_out_stream_V_data_V_U_n_6),
        .CEA2(i_old0),
        .CEAD(1'b0),
        .CEALUMODE(1'b0),
        .CEB1(1'b0),
        .CEB2(regslice_both_out_stream_V_data_V_U_n_6),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(1'b0),
        .CEINMODE(1'b0),
        .CEM(1'b0),
        .CEP(regslice_both_out_stream_V_data_V_U_n_5),
        .CLK(ap_clk),
        .D({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .INMODE({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_term1_reg_518_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_term1_reg_518_reg_OVERFLOW_UNCONNECTED),
        .P(NLW_term1_reg_518_reg_P_UNCONNECTED[47:0]),
        .PATTERNBDETECT(NLW_term1_reg_518_reg_PATTERNBDETECT_UNCONNECTED),
        .PATTERNDETECT(NLW_term1_reg_518_reg_PATTERNDETECT_UNCONNECTED),
        .PCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .PCOUT({term1_reg_518_reg_n_106,term1_reg_518_reg_n_107,term1_reg_518_reg_n_108,term1_reg_518_reg_n_109,term1_reg_518_reg_n_110,term1_reg_518_reg_n_111,term1_reg_518_reg_n_112,term1_reg_518_reg_n_113,term1_reg_518_reg_n_114,term1_reg_518_reg_n_115,term1_reg_518_reg_n_116,term1_reg_518_reg_n_117,term1_reg_518_reg_n_118,term1_reg_518_reg_n_119,term1_reg_518_reg_n_120,term1_reg_518_reg_n_121,term1_reg_518_reg_n_122,term1_reg_518_reg_n_123,term1_reg_518_reg_n_124,term1_reg_518_reg_n_125,term1_reg_518_reg_n_126,term1_reg_518_reg_n_127,term1_reg_518_reg_n_128,term1_reg_518_reg_n_129,term1_reg_518_reg_n_130,term1_reg_518_reg_n_131,term1_reg_518_reg_n_132,term1_reg_518_reg_n_133,term1_reg_518_reg_n_134,term1_reg_518_reg_n_135,term1_reg_518_reg_n_136,term1_reg_518_reg_n_137,term1_reg_518_reg_n_138,term1_reg_518_reg_n_139,term1_reg_518_reg_n_140,term1_reg_518_reg_n_141,term1_reg_518_reg_n_142,term1_reg_518_reg_n_143,term1_reg_518_reg_n_144,term1_reg_518_reg_n_145,term1_reg_518_reg_n_146,term1_reg_518_reg_n_147,term1_reg_518_reg_n_148,term1_reg_518_reg_n_149,term1_reg_518_reg_n_150,term1_reg_518_reg_n_151,term1_reg_518_reg_n_152,term1_reg_518_reg_n_153}),
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
        .UNDERFLOW(NLW_term1_reg_518_reg_UNDERFLOW_UNCONNECTED));
endmodule

(* ORIG_REF_NAME = "fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1" *) 
module system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1
   (P,
    \counter_reg[12] ,
    \counter_reg[8] ,
    \counter_reg[16] ,
    \counter_reg[28] ,
    S,
    \sum_i_reg[22] ,
    \sum_i_reg[26] ,
    \sum_i_reg[30] ,
    \sum_q_reg[18] ,
    \sum_q_reg[22] ,
    \sum_q_reg[26] ,
    \sum_q_reg[30] ,
    p_reg_reg,
    p_reg_reg_0,
    CO,
    p_reg_reg_1,
    p_reg_reg_2,
    p_reg_reg_3,
    p_reg_reg_4,
    p_reg_reg_5,
    p_reg_reg_6,
    p_reg_reg_7,
    p_reg_reg_8,
    p_reg_reg_9,
    p_reg_reg_10,
    p_reg_reg_11,
    p_reg_reg_12,
    p_reg_reg_13,
    p_reg_reg_14,
    ap_block_pp0_stage0_11001,
    p_reg_reg_15,
    ap_clk,
    B,
    A,
    PCOUT,
    out,
    sum_i_reg,
    sum_q_reg);
  output [1:0]P;
  output \counter_reg[12] ;
  output \counter_reg[8] ;
  output \counter_reg[16] ;
  output \counter_reg[28] ;
  output [2:0]S;
  output [3:0]\sum_i_reg[22] ;
  output [3:0]\sum_i_reg[26] ;
  output [3:0]\sum_i_reg[30] ;
  output [2:0]\sum_q_reg[18] ;
  output [3:0]\sum_q_reg[22] ;
  output [3:0]\sum_q_reg[26] ;
  output [3:0]\sum_q_reg[30] ;
  output p_reg_reg;
  output p_reg_reg_0;
  output [0:0]CO;
  output p_reg_reg_1;
  output p_reg_reg_2;
  output p_reg_reg_3;
  output p_reg_reg_4;
  output p_reg_reg_5;
  output p_reg_reg_6;
  output p_reg_reg_7;
  output p_reg_reg_8;
  output p_reg_reg_9;
  output p_reg_reg_10;
  output p_reg_reg_11;
  output p_reg_reg_12;
  output p_reg_reg_13;
  output p_reg_reg_14;
  input ap_block_pp0_stage0_11001;
  input p_reg_reg_15;
  input ap_clk;
  input [15:0]B;
  input [15:0]A;
  input [47:0]PCOUT;
  input [31:0]out;
  input [15:0]sum_i_reg;
  input [15:0]sum_q_reg;

  wire [15:0]A;
  wire [15:0]B;
  wire [0:0]CO;
  wire [1:0]P;
  wire [47:0]PCOUT;
  wire [2:0]S;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire \counter_reg[12] ;
  wire \counter_reg[16] ;
  wire \counter_reg[28] ;
  wire \counter_reg[8] ;
  wire [31:0]out;
  wire p_reg_reg;
  wire p_reg_reg_0;
  wire p_reg_reg_1;
  wire p_reg_reg_10;
  wire p_reg_reg_11;
  wire p_reg_reg_12;
  wire p_reg_reg_13;
  wire p_reg_reg_14;
  wire p_reg_reg_15;
  wire p_reg_reg_2;
  wire p_reg_reg_3;
  wire p_reg_reg_4;
  wire p_reg_reg_5;
  wire p_reg_reg_6;
  wire p_reg_reg_7;
  wire p_reg_reg_8;
  wire p_reg_reg_9;
  wire [15:0]sum_i_reg;
  wire [3:0]\sum_i_reg[22] ;
  wire [3:0]\sum_i_reg[26] ;
  wire [3:0]\sum_i_reg[30] ;
  wire [15:0]sum_q_reg;
  wire [2:0]\sum_q_reg[18] ;
  wire [3:0]\sum_q_reg[22] ;
  wire [3:0]\sum_q_reg[26] ;
  wire [3:0]\sum_q_reg[30] ;

  system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0 fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0_U
       (.A(A),
        .B(B),
        .CO(CO),
        .P(P),
        .PCOUT(PCOUT),
        .S(S),
        .ap_block_pp0_stage0_11001(ap_block_pp0_stage0_11001),
        .ap_clk(ap_clk),
        .\counter_reg[12] (\counter_reg[12] ),
        .\counter_reg[16] (\counter_reg[16] ),
        .\counter_reg[28] (\counter_reg[28] ),
        .\counter_reg[8] (\counter_reg[8] ),
        .out(out),
        .p_reg_reg_0(p_reg_reg),
        .p_reg_reg_1(p_reg_reg_0),
        .p_reg_reg_10(p_reg_reg_9),
        .p_reg_reg_11(p_reg_reg_10),
        .p_reg_reg_12(p_reg_reg_11),
        .p_reg_reg_13(p_reg_reg_12),
        .p_reg_reg_14(p_reg_reg_13),
        .p_reg_reg_15(p_reg_reg_14),
        .p_reg_reg_16(p_reg_reg_15),
        .p_reg_reg_2(p_reg_reg_1),
        .p_reg_reg_3(p_reg_reg_2),
        .p_reg_reg_4(p_reg_reg_3),
        .p_reg_reg_5(p_reg_reg_4),
        .p_reg_reg_6(p_reg_reg_5),
        .p_reg_reg_7(p_reg_reg_6),
        .p_reg_reg_8(p_reg_reg_7),
        .p_reg_reg_9(p_reg_reg_8),
        .sum_i_reg(sum_i_reg),
        .\sum_i_reg[22] (\sum_i_reg[22] ),
        .\sum_i_reg[26] (\sum_i_reg[26] ),
        .\sum_i_reg[30] (\sum_i_reg[30] ),
        .sum_q_reg(sum_q_reg),
        .\sum_q_reg[18] (\sum_q_reg[18] ),
        .\sum_q_reg[22] (\sum_q_reg[22] ),
        .\sum_q_reg[26] (\sum_q_reg[26] ),
        .\sum_q_reg[30] (\sum_q_reg[30] ));
endmodule

(* ORIG_REF_NAME = "fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0" *) 
module system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0
   (P,
    \counter_reg[12] ,
    \counter_reg[8] ,
    \counter_reg[16] ,
    \counter_reg[28] ,
    S,
    \sum_i_reg[22] ,
    \sum_i_reg[26] ,
    \sum_i_reg[30] ,
    \sum_q_reg[18] ,
    \sum_q_reg[22] ,
    \sum_q_reg[26] ,
    \sum_q_reg[30] ,
    p_reg_reg_0,
    p_reg_reg_1,
    CO,
    p_reg_reg_2,
    p_reg_reg_3,
    p_reg_reg_4,
    p_reg_reg_5,
    p_reg_reg_6,
    p_reg_reg_7,
    p_reg_reg_8,
    p_reg_reg_9,
    p_reg_reg_10,
    p_reg_reg_11,
    p_reg_reg_12,
    p_reg_reg_13,
    p_reg_reg_14,
    p_reg_reg_15,
    ap_block_pp0_stage0_11001,
    p_reg_reg_16,
    ap_clk,
    B,
    A,
    PCOUT,
    out,
    sum_i_reg,
    sum_q_reg);
  output [1:0]P;
  output \counter_reg[12] ;
  output \counter_reg[8] ;
  output \counter_reg[16] ;
  output \counter_reg[28] ;
  output [2:0]S;
  output [3:0]\sum_i_reg[22] ;
  output [3:0]\sum_i_reg[26] ;
  output [3:0]\sum_i_reg[30] ;
  output [2:0]\sum_q_reg[18] ;
  output [3:0]\sum_q_reg[22] ;
  output [3:0]\sum_q_reg[26] ;
  output [3:0]\sum_q_reg[30] ;
  output p_reg_reg_0;
  output p_reg_reg_1;
  output [0:0]CO;
  output p_reg_reg_2;
  output p_reg_reg_3;
  output p_reg_reg_4;
  output p_reg_reg_5;
  output p_reg_reg_6;
  output p_reg_reg_7;
  output p_reg_reg_8;
  output p_reg_reg_9;
  output p_reg_reg_10;
  output p_reg_reg_11;
  output p_reg_reg_12;
  output p_reg_reg_13;
  output p_reg_reg_14;
  output p_reg_reg_15;
  input ap_block_pp0_stage0_11001;
  input p_reg_reg_16;
  input ap_clk;
  input [15:0]B;
  input [15:0]A;
  input [47:0]PCOUT;
  input [31:0]out;
  input [15:0]sum_i_reg;
  input [15:0]sum_q_reg;

  wire [15:0]A;
  wire [15:0]B;
  wire \B_V_data_1_payload_A[14]_i_10_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_11_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_12_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_13_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_14_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_15_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_16_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_17_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_18_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_19_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_6_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_8_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_9_n_0 ;
  wire \B_V_data_1_payload_A_reg[14]_i_5_n_3 ;
  wire \B_V_data_1_payload_A_reg[14]_i_7_n_0 ;
  wire \B_V_data_1_payload_A_reg[14]_i_7_n_1 ;
  wire \B_V_data_1_payload_A_reg[14]_i_7_n_2 ;
  wire \B_V_data_1_payload_A_reg[14]_i_7_n_3 ;
  wire [0:0]CO;
  wire [1:0]P;
  wire [47:0]PCOUT;
  wire [2:0]S;
  wire [31:1]add_ln49_fu_218_p2;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire \counter_reg[12] ;
  wire \counter_reg[16] ;
  wire \counter_reg[28] ;
  wire \counter_reg[8] ;
  wire [31:0]out;
  wire p_reg_reg_0;
  wire p_reg_reg_1;
  wire p_reg_reg_10;
  wire p_reg_reg_11;
  wire p_reg_reg_12;
  wire p_reg_reg_13;
  wire p_reg_reg_14;
  wire p_reg_reg_15;
  wire p_reg_reg_16;
  wire p_reg_reg_2;
  wire p_reg_reg_3;
  wire p_reg_reg_4;
  wire p_reg_reg_5;
  wire p_reg_reg_6;
  wire p_reg_reg_7;
  wire p_reg_reg_8;
  wire p_reg_reg_9;
  wire p_reg_reg_n_100;
  wire p_reg_reg_n_101;
  wire p_reg_reg_n_102;
  wire p_reg_reg_n_103;
  wire p_reg_reg_n_104;
  wire p_reg_reg_n_105;
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
  wire [15:0]sum_i_reg;
  wire [3:0]\sum_i_reg[22] ;
  wire [3:0]\sum_i_reg[26] ;
  wire [3:0]\sum_i_reg[30] ;
  wire [15:0]sum_q_reg;
  wire [2:0]\sum_q_reg[18] ;
  wire [3:0]\sum_q_reg[22] ;
  wire [3:0]\sum_q_reg[26] ;
  wire [3:0]\sum_q_reg[30] ;
  wire term1_reg_518_reg_i_123_n_0;
  wire term1_reg_518_reg_i_123_n_1;
  wire term1_reg_518_reg_i_123_n_2;
  wire term1_reg_518_reg_i_123_n_3;
  wire term1_reg_518_reg_i_54_n_0;
  wire term1_reg_518_reg_i_54_n_1;
  wire term1_reg_518_reg_i_54_n_2;
  wire term1_reg_518_reg_i_54_n_3;
  wire term1_reg_518_reg_i_55_n_0;
  wire term1_reg_518_reg_i_55_n_1;
  wire term1_reg_518_reg_i_55_n_2;
  wire term1_reg_518_reg_i_55_n_3;
  wire term1_reg_518_reg_i_56_n_0;
  wire term1_reg_518_reg_i_57_n_0;
  wire term1_reg_518_reg_i_57_n_1;
  wire term1_reg_518_reg_i_57_n_2;
  wire term1_reg_518_reg_i_57_n_3;
  wire term1_reg_518_reg_i_58_n_0;
  wire term1_reg_518_reg_i_58_n_1;
  wire term1_reg_518_reg_i_58_n_2;
  wire term1_reg_518_reg_i_58_n_3;
  wire term1_reg_518_reg_i_59_n_0;
  wire term1_reg_518_reg_i_60_n_0;
  wire term1_reg_518_reg_i_60_n_1;
  wire term1_reg_518_reg_i_60_n_2;
  wire term1_reg_518_reg_i_60_n_3;
  wire term1_reg_518_reg_i_61_n_2;
  wire term1_reg_518_reg_i_61_n_3;
  wire term1_reg_518_reg_i_62_n_0;
  wire term1_reg_518_reg_i_63_n_0;
  wire term1_reg_518_reg_i_63_n_1;
  wire term1_reg_518_reg_i_63_n_2;
  wire term1_reg_518_reg_i_63_n_3;
  wire term1_reg_518_reg_i_64_n_0;
  wire [9:1]tmp_2_fu_426_p4;
  wire [3:2]\NLW_B_V_data_1_payload_A_reg[14]_i_5_CO_UNCONNECTED ;
  wire [3:0]\NLW_B_V_data_1_payload_A_reg[14]_i_5_O_UNCONNECTED ;
  wire [3:0]\NLW_B_V_data_1_payload_A_reg[14]_i_7_O_UNCONNECTED ;
  wire NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED;
  wire NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED;
  wire NLW_p_reg_reg_OVERFLOW_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED;
  wire NLW_p_reg_reg_UNDERFLOW_UNCONNECTED;
  wire [29:0]NLW_p_reg_reg_ACOUT_UNCONNECTED;
  wire [17:0]NLW_p_reg_reg_BCOUT_UNCONNECTED;
  wire [3:0]NLW_p_reg_reg_CARRYOUT_UNCONNECTED;
  wire [47:32]NLW_p_reg_reg_P_UNCONNECTED;
  wire [47:0]NLW_p_reg_reg_PCOUT_UNCONNECTED;
  wire [3:2]NLW_term1_reg_518_reg_i_61_CO_UNCONNECTED;
  wire [3:3]NLW_term1_reg_518_reg_i_61_O_UNCONNECTED;

  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[0]_i_1 
       (.I0(p_reg_reg_n_99),
        .I1(CO),
        .O(p_reg_reg_15));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[10]_i_1 
       (.I0(p_reg_reg_n_89),
        .I1(CO),
        .O(p_reg_reg_5));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[11]_i_1 
       (.I0(p_reg_reg_n_88),
        .I1(CO),
        .O(p_reg_reg_4));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[12]_i_1 
       (.I0(p_reg_reg_n_87),
        .I1(CO),
        .O(p_reg_reg_3));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[13]_i_1 
       (.I0(p_reg_reg_n_86),
        .I1(CO),
        .O(p_reg_reg_2));
  LUT2 #(
    .INIT(4'h8)) 
    \B_V_data_1_payload_A[14]_i_10 
       (.I0(tmp_2_fu_426_p4[9]),
        .I1(P[1]),
        .O(\B_V_data_1_payload_A[14]_i_10_n_0 ));
  LUT2 #(
    .INIT(4'h8)) 
    \B_V_data_1_payload_A[14]_i_11 
       (.I0(tmp_2_fu_426_p4[7]),
        .I1(tmp_2_fu_426_p4[8]),
        .O(\B_V_data_1_payload_A[14]_i_11_n_0 ));
  LUT2 #(
    .INIT(4'h7)) 
    \B_V_data_1_payload_A[14]_i_12 
       (.I0(tmp_2_fu_426_p4[5]),
        .I1(tmp_2_fu_426_p4[6]),
        .O(\B_V_data_1_payload_A[14]_i_12_n_0 ));
  LUT2 #(
    .INIT(4'h7)) 
    \B_V_data_1_payload_A[14]_i_13 
       (.I0(tmp_2_fu_426_p4[3]),
        .I1(tmp_2_fu_426_p4[4]),
        .O(\B_V_data_1_payload_A[14]_i_13_n_0 ));
  LUT2 #(
    .INIT(4'h7)) 
    \B_V_data_1_payload_A[14]_i_14 
       (.I0(tmp_2_fu_426_p4[1]),
        .I1(tmp_2_fu_426_p4[2]),
        .O(\B_V_data_1_payload_A[14]_i_14_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_payload_A[14]_i_15 
       (.I0(P[0]),
        .O(\B_V_data_1_payload_A[14]_i_15_n_0 ));
  LUT2 #(
    .INIT(4'h8)) 
    \B_V_data_1_payload_A[14]_i_16 
       (.I0(tmp_2_fu_426_p4[5]),
        .I1(tmp_2_fu_426_p4[6]),
        .O(\B_V_data_1_payload_A[14]_i_16_n_0 ));
  LUT2 #(
    .INIT(4'h8)) 
    \B_V_data_1_payload_A[14]_i_17 
       (.I0(tmp_2_fu_426_p4[3]),
        .I1(tmp_2_fu_426_p4[4]),
        .O(\B_V_data_1_payload_A[14]_i_17_n_0 ));
  LUT2 #(
    .INIT(4'h8)) 
    \B_V_data_1_payload_A[14]_i_18 
       (.I0(tmp_2_fu_426_p4[1]),
        .I1(tmp_2_fu_426_p4[2]),
        .O(\B_V_data_1_payload_A[14]_i_18_n_0 ));
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[14]_i_19 
       (.I0(P[0]),
        .I1(p_reg_reg_n_85),
        .O(\B_V_data_1_payload_A[14]_i_19_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[14]_i_3 
       (.I0(p_reg_reg_n_85),
        .I1(CO),
        .O(p_reg_reg_1));
  LUT6 #(
    .INIT(64'hAAAAAAAAAAAAAAAB)) 
    \B_V_data_1_payload_A[14]_i_4 
       (.I0(P[1]),
        .I1(tmp_2_fu_426_p4[3]),
        .I2(tmp_2_fu_426_p4[2]),
        .I3(tmp_2_fu_426_p4[5]),
        .I4(tmp_2_fu_426_p4[4]),
        .I5(\B_V_data_1_payload_A[14]_i_6_n_0 ),
        .O(p_reg_reg_0));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFFFFE)) 
    \B_V_data_1_payload_A[14]_i_6 
       (.I0(tmp_2_fu_426_p4[8]),
        .I1(tmp_2_fu_426_p4[9]),
        .I2(tmp_2_fu_426_p4[6]),
        .I3(tmp_2_fu_426_p4[7]),
        .I4(tmp_2_fu_426_p4[1]),
        .I5(P[0]),
        .O(\B_V_data_1_payload_A[14]_i_6_n_0 ));
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[14]_i_8 
       (.I0(P[1]),
        .I1(tmp_2_fu_426_p4[9]),
        .O(\B_V_data_1_payload_A[14]_i_8_n_0 ));
  LUT2 #(
    .INIT(4'h7)) 
    \B_V_data_1_payload_A[14]_i_9 
       (.I0(tmp_2_fu_426_p4[7]),
        .I1(tmp_2_fu_426_p4[8]),
        .O(\B_V_data_1_payload_A[14]_i_9_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[1]_i_1 
       (.I0(p_reg_reg_n_98),
        .I1(CO),
        .O(p_reg_reg_14));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[2]_i_1 
       (.I0(p_reg_reg_n_97),
        .I1(CO),
        .O(p_reg_reg_13));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[3]_i_1 
       (.I0(p_reg_reg_n_96),
        .I1(CO),
        .O(p_reg_reg_12));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[4]_i_1 
       (.I0(p_reg_reg_n_95),
        .I1(CO),
        .O(p_reg_reg_11));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[5]_i_1 
       (.I0(p_reg_reg_n_94),
        .I1(CO),
        .O(p_reg_reg_10));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[6]_i_1 
       (.I0(p_reg_reg_n_93),
        .I1(CO),
        .O(p_reg_reg_9));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[7]_i_1 
       (.I0(p_reg_reg_n_92),
        .I1(CO),
        .O(p_reg_reg_8));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[8]_i_1 
       (.I0(p_reg_reg_n_91),
        .I1(CO),
        .O(p_reg_reg_7));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \B_V_data_1_payload_A[9]_i_1 
       (.I0(p_reg_reg_n_90),
        .I1(CO),
        .O(p_reg_reg_6));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \B_V_data_1_payload_A_reg[14]_i_5 
       (.CI(\B_V_data_1_payload_A_reg[14]_i_7_n_0 ),
        .CO({\NLW_B_V_data_1_payload_A_reg[14]_i_5_CO_UNCONNECTED [3:2],CO,\B_V_data_1_payload_A_reg[14]_i_5_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,\B_V_data_1_payload_A[14]_i_8_n_0 ,\B_V_data_1_payload_A[14]_i_9_n_0 }),
        .O(\NLW_B_V_data_1_payload_A_reg[14]_i_5_O_UNCONNECTED [3:0]),
        .S({1'b0,1'b0,\B_V_data_1_payload_A[14]_i_10_n_0 ,\B_V_data_1_payload_A[14]_i_11_n_0 }));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \B_V_data_1_payload_A_reg[14]_i_7 
       (.CI(1'b0),
        .CO({\B_V_data_1_payload_A_reg[14]_i_7_n_0 ,\B_V_data_1_payload_A_reg[14]_i_7_n_1 ,\B_V_data_1_payload_A_reg[14]_i_7_n_2 ,\B_V_data_1_payload_A_reg[14]_i_7_n_3 }),
        .CYINIT(1'b0),
        .DI({\B_V_data_1_payload_A[14]_i_12_n_0 ,\B_V_data_1_payload_A[14]_i_13_n_0 ,\B_V_data_1_payload_A[14]_i_14_n_0 ,\B_V_data_1_payload_A[14]_i_15_n_0 }),
        .O(\NLW_B_V_data_1_payload_A_reg[14]_i_7_O_UNCONNECTED [3:0]),
        .S({\B_V_data_1_payload_A[14]_i_16_n_0 ,\B_V_data_1_payload_A[14]_i_17_n_0 ,\B_V_data_1_payload_A[14]_i_18_n_0 ,\B_V_data_1_payload_A[14]_i_19_n_0 }));
  DSP48E1 #(
    .ACASCREG(1),
    .ADREG(1),
    .ALUMODEREG(0),
    .AREG(1),
    .AUTORESET_PATDET("NO_RESET"),
    .A_INPUT("DIRECT"),
    .BCASCREG(2),
    .BREG(2),
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
       (.A({A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A[15],A}),
        .ACIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .ACOUT(NLW_p_reg_reg_ACOUT_UNCONNECTED[29:0]),
        .ALUMODE({1'b0,1'b0,1'b1,1'b1}),
        .B({B[15],B[15],B}),
        .BCIN({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .BCOUT(NLW_p_reg_reg_BCOUT_UNCONNECTED[17:0]),
        .C({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .CARRYCASCIN(1'b0),
        .CARRYCASCOUT(NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED),
        .CARRYIN(1'b0),
        .CARRYINSEL({1'b0,1'b0,1'b0}),
        .CARRYOUT(NLW_p_reg_reg_CARRYOUT_UNCONNECTED[3:0]),
        .CEA1(1'b0),
        .CEA2(ap_block_pp0_stage0_11001),
        .CEAD(1'b0),
        .CEALUMODE(1'b0),
        .CEB1(p_reg_reg_16),
        .CEB2(ap_block_pp0_stage0_11001),
        .CEC(1'b0),
        .CECARRYIN(1'b0),
        .CECTRL(1'b0),
        .CED(1'b0),
        .CEINMODE(1'b0),
        .CEM(ap_block_pp0_stage0_11001),
        .CEP(ap_block_pp0_stage0_11001),
        .CLK(ap_clk),
        .D({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .INMODE({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .MULTSIGNIN(1'b0),
        .MULTSIGNOUT(NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED),
        .OPMODE({1'b0,1'b0,1'b1,1'b0,1'b1,1'b0,1'b1}),
        .OVERFLOW(NLW_p_reg_reg_OVERFLOW_UNCONNECTED),
        .P({NLW_p_reg_reg_P_UNCONNECTED[47:32],P[1],tmp_2_fu_426_p4,P[0],p_reg_reg_n_85,p_reg_reg_n_86,p_reg_reg_n_87,p_reg_reg_n_88,p_reg_reg_n_89,p_reg_reg_n_90,p_reg_reg_n_91,p_reg_reg_n_92,p_reg_reg_n_93,p_reg_reg_n_94,p_reg_reg_n_95,p_reg_reg_n_96,p_reg_reg_n_97,p_reg_reg_n_98,p_reg_reg_n_99,p_reg_reg_n_100,p_reg_reg_n_101,p_reg_reg_n_102,p_reg_reg_n_103,p_reg_reg_n_104,p_reg_reg_n_105}),
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
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_101
       (.I0(sum_i_reg[10]),
        .I1(sum_i_reg[11]),
        .O(\sum_i_reg[26] [3]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_102
       (.I0(sum_i_reg[9]),
        .I1(sum_i_reg[10]),
        .O(\sum_i_reg[26] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_103
       (.I0(sum_i_reg[8]),
        .I1(sum_i_reg[9]),
        .O(\sum_i_reg[26] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_104
       (.I0(sum_i_reg[7]),
        .I1(sum_i_reg[8]),
        .O(\sum_i_reg[26] [0]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_105
       (.I0(sum_i_reg[6]),
        .I1(sum_i_reg[7]),
        .O(\sum_i_reg[22] [3]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_106
       (.I0(sum_i_reg[5]),
        .I1(sum_i_reg[6]),
        .O(\sum_i_reg[22] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_107
       (.I0(sum_i_reg[4]),
        .I1(sum_i_reg[5]),
        .O(\sum_i_reg[22] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_108
       (.I0(sum_i_reg[3]),
        .I1(sum_i_reg[4]),
        .O(\sum_i_reg[22] [0]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_110
       (.I0(sum_i_reg[2]),
        .I1(sum_i_reg[3]),
        .O(S[2]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_111
       (.I0(sum_i_reg[1]),
        .I1(sum_i_reg[2]),
        .O(S[1]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_112
       (.I0(sum_i_reg[0]),
        .I1(sum_i_reg[1]),
        .O(S[0]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_123
       (.CI(term1_reg_518_reg_i_63_n_0),
        .CO({term1_reg_518_reg_i_123_n_0,term1_reg_518_reg_i_123_n_1,term1_reg_518_reg_i_123_n_2,term1_reg_518_reg_i_123_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln49_fu_218_p2[24:21]),
        .S(out[24:21]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_124
       (.I0(sum_q_reg[14]),
        .I1(sum_q_reg[15]),
        .O(\sum_q_reg[30] [3]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_125
       (.I0(sum_q_reg[13]),
        .I1(sum_q_reg[14]),
        .O(\sum_q_reg[30] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_126
       (.I0(sum_q_reg[12]),
        .I1(sum_q_reg[13]),
        .O(\sum_q_reg[30] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_127
       (.I0(sum_q_reg[11]),
        .I1(sum_q_reg[12]),
        .O(\sum_q_reg[30] [0]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_133
       (.I0(sum_i_reg[14]),
        .I1(sum_i_reg[15]),
        .O(\sum_i_reg[30] [3]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_134
       (.I0(sum_i_reg[13]),
        .I1(sum_i_reg[14]),
        .O(\sum_i_reg[30] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_135
       (.I0(sum_i_reg[12]),
        .I1(sum_i_reg[13]),
        .O(\sum_i_reg[30] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_136
       (.I0(sum_i_reg[11]),
        .I1(sum_i_reg[12]),
        .O(\sum_i_reg[30] [0]));
  LUT5 #(
    .INIT(32'h00008000)) 
    term1_reg_518_reg_i_36
       (.I0(add_ln49_fu_218_p2[6]),
        .I1(add_ln49_fu_218_p2[7]),
        .I2(add_ln49_fu_218_p2[4]),
        .I3(add_ln49_fu_218_p2[5]),
        .I4(term1_reg_518_reg_i_56_n_0),
        .O(\counter_reg[8] ));
  LUT5 #(
    .INIT(32'h00000001)) 
    term1_reg_518_reg_i_37
       (.I0(add_ln49_fu_218_p2[12]),
        .I1(add_ln49_fu_218_p2[13]),
        .I2(add_ln49_fu_218_p2[14]),
        .I3(add_ln49_fu_218_p2[15]),
        .I4(term1_reg_518_reg_i_59_n_0),
        .O(\counter_reg[12] ));
  LUT5 #(
    .INIT(32'h00000001)) 
    term1_reg_518_reg_i_38
       (.I0(add_ln49_fu_218_p2[28]),
        .I1(add_ln49_fu_218_p2[29]),
        .I2(add_ln49_fu_218_p2[31]),
        .I3(add_ln49_fu_218_p2[30]),
        .I4(term1_reg_518_reg_i_62_n_0),
        .O(\counter_reg[28] ));
  LUT5 #(
    .INIT(32'hFFFEFFFF)) 
    term1_reg_518_reg_i_39
       (.I0(add_ln49_fu_218_p2[16]),
        .I1(add_ln49_fu_218_p2[17]),
        .I2(add_ln49_fu_218_p2[18]),
        .I3(add_ln49_fu_218_p2[19]),
        .I4(term1_reg_518_reg_i_64_n_0),
        .O(\counter_reg[16] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_54
       (.CI(term1_reg_518_reg_i_55_n_0),
        .CO({term1_reg_518_reg_i_54_n_0,term1_reg_518_reg_i_54_n_1,term1_reg_518_reg_i_54_n_2,term1_reg_518_reg_i_54_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln49_fu_218_p2[8:5]),
        .S(out[8:5]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_55
       (.CI(1'b0),
        .CO({term1_reg_518_reg_i_55_n_0,term1_reg_518_reg_i_55_n_1,term1_reg_518_reg_i_55_n_2,term1_reg_518_reg_i_55_n_3}),
        .CYINIT(out[0]),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln49_fu_218_p2[4:1]),
        .S(out[4:1]));
  LUT4 #(
    .INIT(16'hDFFF)) 
    term1_reg_518_reg_i_56
       (.I0(add_ln49_fu_218_p2[1]),
        .I1(out[0]),
        .I2(add_ln49_fu_218_p2[3]),
        .I3(add_ln49_fu_218_p2[2]),
        .O(term1_reg_518_reg_i_56_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_57
       (.CI(term1_reg_518_reg_i_54_n_0),
        .CO({term1_reg_518_reg_i_57_n_0,term1_reg_518_reg_i_57_n_1,term1_reg_518_reg_i_57_n_2,term1_reg_518_reg_i_57_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln49_fu_218_p2[12:9]),
        .S(out[12:9]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_58
       (.CI(term1_reg_518_reg_i_57_n_0),
        .CO({term1_reg_518_reg_i_58_n_0,term1_reg_518_reg_i_58_n_1,term1_reg_518_reg_i_58_n_2,term1_reg_518_reg_i_58_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln49_fu_218_p2[16:13]),
        .S(out[16:13]));
  LUT4 #(
    .INIT(16'hFFFE)) 
    term1_reg_518_reg_i_59
       (.I0(add_ln49_fu_218_p2[9]),
        .I1(add_ln49_fu_218_p2[8]),
        .I2(add_ln49_fu_218_p2[11]),
        .I3(add_ln49_fu_218_p2[10]),
        .O(term1_reg_518_reg_i_59_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_60
       (.CI(term1_reg_518_reg_i_123_n_0),
        .CO({term1_reg_518_reg_i_60_n_0,term1_reg_518_reg_i_60_n_1,term1_reg_518_reg_i_60_n_2,term1_reg_518_reg_i_60_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln49_fu_218_p2[28:25]),
        .S(out[28:25]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_61
       (.CI(term1_reg_518_reg_i_60_n_0),
        .CO({NLW_term1_reg_518_reg_i_61_CO_UNCONNECTED[3:2],term1_reg_518_reg_i_61_n_2,term1_reg_518_reg_i_61_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({NLW_term1_reg_518_reg_i_61_O_UNCONNECTED[3],add_ln49_fu_218_p2[31:29]}),
        .S({1'b0,out[31:29]}));
  LUT4 #(
    .INIT(16'hFFFE)) 
    term1_reg_518_reg_i_62
       (.I0(add_ln49_fu_218_p2[25]),
        .I1(add_ln49_fu_218_p2[24]),
        .I2(add_ln49_fu_218_p2[27]),
        .I3(add_ln49_fu_218_p2[26]),
        .O(term1_reg_518_reg_i_62_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_63
       (.CI(term1_reg_518_reg_i_58_n_0),
        .CO({term1_reg_518_reg_i_63_n_0,term1_reg_518_reg_i_63_n_1,term1_reg_518_reg_i_63_n_2,term1_reg_518_reg_i_63_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(add_ln49_fu_218_p2[20:17]),
        .S(out[20:17]));
  LUT4 #(
    .INIT(16'h0001)) 
    term1_reg_518_reg_i_64
       (.I0(add_ln49_fu_218_p2[23]),
        .I1(add_ln49_fu_218_p2[22]),
        .I2(add_ln49_fu_218_p2[21]),
        .I3(add_ln49_fu_218_p2[20]),
        .O(term1_reg_518_reg_i_64_n_0));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_72
       (.I0(sum_q_reg[10]),
        .I1(sum_q_reg[11]),
        .O(\sum_q_reg[26] [3]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_73
       (.I0(sum_q_reg[9]),
        .I1(sum_q_reg[10]),
        .O(\sum_q_reg[26] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_74
       (.I0(sum_q_reg[8]),
        .I1(sum_q_reg[9]),
        .O(\sum_q_reg[26] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_75
       (.I0(sum_q_reg[7]),
        .I1(sum_q_reg[8]),
        .O(\sum_q_reg[26] [0]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_76
       (.I0(sum_q_reg[6]),
        .I1(sum_q_reg[7]),
        .O(\sum_q_reg[22] [3]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_77
       (.I0(sum_q_reg[5]),
        .I1(sum_q_reg[6]),
        .O(\sum_q_reg[22] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_78
       (.I0(sum_q_reg[4]),
        .I1(sum_q_reg[5]),
        .O(\sum_q_reg[22] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_79
       (.I0(sum_q_reg[3]),
        .I1(sum_q_reg[4]),
        .O(\sum_q_reg[22] [0]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_81
       (.I0(sum_q_reg[2]),
        .I1(sum_q_reg[3]),
        .O(\sum_q_reg[18] [2]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_82
       (.I0(sum_q_reg[1]),
        .I1(sum_q_reg[2]),
        .O(\sum_q_reg[18] [1]));
  LUT2 #(
    .INIT(4'h9)) 
    term1_reg_518_reg_i_83
       (.I0(sum_q_reg[0]),
        .I1(sum_q_reg[1]),
        .O(\sum_q_reg[18] [0]));
endmodule

(* ORIG_REF_NAME = "fsk_discriminator_regslice_both" *) 
module system_fsk_discriminator_0_0_fsk_discriminator_regslice_both
   (\B_V_data_1_state_reg[1]_0 ,
    ap_rst_n_inv,
    in_stream_TVALID_int_regslice,
    B,
    A,
    O,
    \sum_i_reg[7] ,
    \sum_i_reg[11] ,
    \sum_i_reg[15] ,
    \sum_i_reg[19] ,
    \sum_i_reg[23] ,
    \sum_i_reg[27] ,
    \sum_i_reg[30] ,
    \sum_q_reg[3] ,
    \sum_q_reg[7] ,
    \sum_q_reg[11] ,
    \sum_q_reg[15] ,
    \sum_q_reg[19] ,
    \sum_q_reg[23] ,
    \sum_q_reg[27] ,
    \sum_q_reg[30] ,
    ap_clk,
    ap_rst_n,
    \B_V_data_1_state_reg[1]_1 ,
    in_stream_TVALID,
    sum_i_reg,
    sum_q_reg,
    S,
    p_reg_reg,
    p_reg_reg_0,
    term1_reg_518_reg_i_99_0,
    p_reg_reg_1,
    p_reg_reg_2,
    p_reg_reg_3,
    term1_reg_518_reg_i_70_0,
    in_stream_TDATA);
  output \B_V_data_1_state_reg[1]_0 ;
  output ap_rst_n_inv;
  output in_stream_TVALID_int_regslice;
  output [15:0]B;
  output [15:0]A;
  output [3:0]O;
  output [3:0]\sum_i_reg[7] ;
  output [3:0]\sum_i_reg[11] ;
  output [3:0]\sum_i_reg[15] ;
  output [3:0]\sum_i_reg[19] ;
  output [3:0]\sum_i_reg[23] ;
  output [3:0]\sum_i_reg[27] ;
  output [3:0]\sum_i_reg[30] ;
  output [3:0]\sum_q_reg[3] ;
  output [3:0]\sum_q_reg[7] ;
  output [3:0]\sum_q_reg[11] ;
  output [3:0]\sum_q_reg[15] ;
  output [3:0]\sum_q_reg[19] ;
  output [3:0]\sum_q_reg[23] ;
  output [3:0]\sum_q_reg[27] ;
  output [3:0]\sum_q_reg[30] ;
  input ap_clk;
  input ap_rst_n;
  input \B_V_data_1_state_reg[1]_1 ;
  input in_stream_TVALID;
  input [31:0]sum_i_reg;
  input [31:0]sum_q_reg;
  input [2:0]S;
  input [3:0]p_reg_reg;
  input [3:0]p_reg_reg_0;
  input [3:0]term1_reg_518_reg_i_99_0;
  input [2:0]p_reg_reg_1;
  input [3:0]p_reg_reg_2;
  input [3:0]p_reg_reg_3;
  input [3:0]term1_reg_518_reg_i_70_0;
  input [31:0]in_stream_TDATA;

  wire [15:0]A;
  wire [15:0]B;
  wire B_V_data_1_load_B;
  wire \B_V_data_1_payload_A[31]_i_1_n_0 ;
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
  wire B_V_data_1_sel_rd_i_1__0_n_0;
  wire B_V_data_1_sel_rd_reg_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1_n_0;
  wire \B_V_data_1_state[0]_i_1_n_0 ;
  wire \B_V_data_1_state[1]_i_2_n_0 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire \B_V_data_1_state_reg[1]_1 ;
  wire [3:0]O;
  wire [2:0]S;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire icmp_ln23_1_fu_321_p2;
  wire icmp_ln23_fu_267_p2;
  wire [31:0]in_stream_TDATA;
  wire in_stream_TVALID;
  wire in_stream_TVALID_int_regslice;
  wire [3:0]p_reg_reg;
  wire [3:0]p_reg_reg_0;
  wire [2:0]p_reg_reg_1;
  wire [3:0]p_reg_reg_2;
  wire [3:0]p_reg_reg_3;
  wire \sum_i[0]_i_2_n_0 ;
  wire \sum_i[0]_i_3_n_0 ;
  wire \sum_i[0]_i_4_n_0 ;
  wire \sum_i[0]_i_5_n_0 ;
  wire \sum_i[12]_i_2_n_0 ;
  wire \sum_i[12]_i_3_n_0 ;
  wire \sum_i[12]_i_4_n_0 ;
  wire \sum_i[12]_i_5_n_0 ;
  wire \sum_i[16]_i_2_n_0 ;
  wire \sum_i[16]_i_3_n_0 ;
  wire \sum_i[16]_i_4_n_0 ;
  wire \sum_i[16]_i_5_n_0 ;
  wire \sum_i[20]_i_2_n_0 ;
  wire \sum_i[20]_i_3_n_0 ;
  wire \sum_i[20]_i_4_n_0 ;
  wire \sum_i[20]_i_5_n_0 ;
  wire \sum_i[24]_i_2_n_0 ;
  wire \sum_i[24]_i_3_n_0 ;
  wire \sum_i[24]_i_4_n_0 ;
  wire \sum_i[24]_i_5_n_0 ;
  wire \sum_i[28]_i_2_n_0 ;
  wire \sum_i[28]_i_3_n_0 ;
  wire \sum_i[28]_i_4_n_0 ;
  wire \sum_i[28]_i_5_n_0 ;
  wire \sum_i[4]_i_2_n_0 ;
  wire \sum_i[4]_i_3_n_0 ;
  wire \sum_i[4]_i_4_n_0 ;
  wire \sum_i[4]_i_5_n_0 ;
  wire \sum_i[8]_i_2_n_0 ;
  wire \sum_i[8]_i_3_n_0 ;
  wire \sum_i[8]_i_4_n_0 ;
  wire \sum_i[8]_i_5_n_0 ;
  wire [31:0]sum_i_reg;
  wire \sum_i_reg[0]_i_1_n_0 ;
  wire \sum_i_reg[0]_i_1_n_1 ;
  wire \sum_i_reg[0]_i_1_n_2 ;
  wire \sum_i_reg[0]_i_1_n_3 ;
  wire [3:0]\sum_i_reg[11] ;
  wire \sum_i_reg[12]_i_1_n_0 ;
  wire \sum_i_reg[12]_i_1_n_1 ;
  wire \sum_i_reg[12]_i_1_n_2 ;
  wire \sum_i_reg[12]_i_1_n_3 ;
  wire [3:0]\sum_i_reg[15] ;
  wire \sum_i_reg[16]_i_1_n_0 ;
  wire \sum_i_reg[16]_i_1_n_1 ;
  wire \sum_i_reg[16]_i_1_n_2 ;
  wire \sum_i_reg[16]_i_1_n_3 ;
  wire [3:0]\sum_i_reg[19] ;
  wire \sum_i_reg[20]_i_1_n_0 ;
  wire \sum_i_reg[20]_i_1_n_1 ;
  wire \sum_i_reg[20]_i_1_n_2 ;
  wire \sum_i_reg[20]_i_1_n_3 ;
  wire [3:0]\sum_i_reg[23] ;
  wire \sum_i_reg[24]_i_1_n_0 ;
  wire \sum_i_reg[24]_i_1_n_1 ;
  wire \sum_i_reg[24]_i_1_n_2 ;
  wire \sum_i_reg[24]_i_1_n_3 ;
  wire [3:0]\sum_i_reg[27] ;
  wire \sum_i_reg[28]_i_1_n_1 ;
  wire \sum_i_reg[28]_i_1_n_2 ;
  wire \sum_i_reg[28]_i_1_n_3 ;
  wire [3:0]\sum_i_reg[30] ;
  wire \sum_i_reg[4]_i_1_n_0 ;
  wire \sum_i_reg[4]_i_1_n_1 ;
  wire \sum_i_reg[4]_i_1_n_2 ;
  wire \sum_i_reg[4]_i_1_n_3 ;
  wire [3:0]\sum_i_reg[7] ;
  wire \sum_i_reg[8]_i_1_n_0 ;
  wire \sum_i_reg[8]_i_1_n_1 ;
  wire \sum_i_reg[8]_i_1_n_2 ;
  wire \sum_i_reg[8]_i_1_n_3 ;
  wire \sum_q[0]_i_2_n_0 ;
  wire \sum_q[0]_i_3_n_0 ;
  wire \sum_q[0]_i_4_n_0 ;
  wire \sum_q[0]_i_5_n_0 ;
  wire \sum_q[12]_i_2_n_0 ;
  wire \sum_q[12]_i_3_n_0 ;
  wire \sum_q[12]_i_4_n_0 ;
  wire \sum_q[12]_i_5_n_0 ;
  wire \sum_q[16]_i_2_n_0 ;
  wire \sum_q[16]_i_3_n_0 ;
  wire \sum_q[16]_i_4_n_0 ;
  wire \sum_q[16]_i_5_n_0 ;
  wire \sum_q[20]_i_2_n_0 ;
  wire \sum_q[20]_i_3_n_0 ;
  wire \sum_q[20]_i_4_n_0 ;
  wire \sum_q[20]_i_5_n_0 ;
  wire \sum_q[24]_i_2_n_0 ;
  wire \sum_q[24]_i_3_n_0 ;
  wire \sum_q[24]_i_4_n_0 ;
  wire \sum_q[24]_i_5_n_0 ;
  wire \sum_q[28]_i_2_n_0 ;
  wire \sum_q[28]_i_3_n_0 ;
  wire \sum_q[28]_i_4_n_0 ;
  wire \sum_q[28]_i_5_n_0 ;
  wire \sum_q[4]_i_2_n_0 ;
  wire \sum_q[4]_i_3_n_0 ;
  wire \sum_q[4]_i_4_n_0 ;
  wire \sum_q[4]_i_5_n_0 ;
  wire \sum_q[8]_i_2_n_0 ;
  wire \sum_q[8]_i_3_n_0 ;
  wire \sum_q[8]_i_4_n_0 ;
  wire \sum_q[8]_i_5_n_0 ;
  wire [31:0]sum_q_reg;
  wire \sum_q_reg[0]_i_1_n_0 ;
  wire \sum_q_reg[0]_i_1_n_1 ;
  wire \sum_q_reg[0]_i_1_n_2 ;
  wire \sum_q_reg[0]_i_1_n_3 ;
  wire [3:0]\sum_q_reg[11] ;
  wire \sum_q_reg[12]_i_1_n_0 ;
  wire \sum_q_reg[12]_i_1_n_1 ;
  wire \sum_q_reg[12]_i_1_n_2 ;
  wire \sum_q_reg[12]_i_1_n_3 ;
  wire [3:0]\sum_q_reg[15] ;
  wire \sum_q_reg[16]_i_1_n_0 ;
  wire \sum_q_reg[16]_i_1_n_1 ;
  wire \sum_q_reg[16]_i_1_n_2 ;
  wire \sum_q_reg[16]_i_1_n_3 ;
  wire [3:0]\sum_q_reg[19] ;
  wire \sum_q_reg[20]_i_1_n_0 ;
  wire \sum_q_reg[20]_i_1_n_1 ;
  wire \sum_q_reg[20]_i_1_n_2 ;
  wire \sum_q_reg[20]_i_1_n_3 ;
  wire [3:0]\sum_q_reg[23] ;
  wire \sum_q_reg[24]_i_1_n_0 ;
  wire \sum_q_reg[24]_i_1_n_1 ;
  wire \sum_q_reg[24]_i_1_n_2 ;
  wire \sum_q_reg[24]_i_1_n_3 ;
  wire [3:0]\sum_q_reg[27] ;
  wire \sum_q_reg[28]_i_1_n_1 ;
  wire \sum_q_reg[28]_i_1_n_2 ;
  wire \sum_q_reg[28]_i_1_n_3 ;
  wire [3:0]\sum_q_reg[30] ;
  wire [3:0]\sum_q_reg[3] ;
  wire \sum_q_reg[4]_i_1_n_0 ;
  wire \sum_q_reg[4]_i_1_n_1 ;
  wire \sum_q_reg[4]_i_1_n_2 ;
  wire \sum_q_reg[4]_i_1_n_3 ;
  wire [3:0]\sum_q_reg[7] ;
  wire \sum_q_reg[8]_i_1_n_0 ;
  wire \sum_q_reg[8]_i_1_n_1 ;
  wire \sum_q_reg[8]_i_1_n_2 ;
  wire \sum_q_reg[8]_i_1_n_3 ;
  wire term1_reg_518_reg_i_100_n_0;
  wire term1_reg_518_reg_i_109_n_0;
  wire term1_reg_518_reg_i_113_n_0;
  wire term1_reg_518_reg_i_114_n_0;
  wire term1_reg_518_reg_i_115_n_0;
  wire term1_reg_518_reg_i_116_n_0;
  wire term1_reg_518_reg_i_117_n_0;
  wire term1_reg_518_reg_i_118_n_0;
  wire term1_reg_518_reg_i_118_n_1;
  wire term1_reg_518_reg_i_118_n_2;
  wire term1_reg_518_reg_i_118_n_3;
  wire term1_reg_518_reg_i_119_n_0;
  wire term1_reg_518_reg_i_120_n_0;
  wire term1_reg_518_reg_i_121_n_0;
  wire term1_reg_518_reg_i_122_n_0;
  wire term1_reg_518_reg_i_128_n_0;
  wire term1_reg_518_reg_i_128_n_1;
  wire term1_reg_518_reg_i_128_n_2;
  wire term1_reg_518_reg_i_128_n_3;
  wire term1_reg_518_reg_i_129_n_0;
  wire term1_reg_518_reg_i_130_n_0;
  wire term1_reg_518_reg_i_131_n_0;
  wire term1_reg_518_reg_i_132_n_0;
  wire term1_reg_518_reg_i_137_n_0;
  wire term1_reg_518_reg_i_137_n_1;
  wire term1_reg_518_reg_i_137_n_2;
  wire term1_reg_518_reg_i_137_n_3;
  wire term1_reg_518_reg_i_138_n_0;
  wire term1_reg_518_reg_i_139_n_0;
  wire term1_reg_518_reg_i_140_n_0;
  wire term1_reg_518_reg_i_141_n_0;
  wire term1_reg_518_reg_i_142_n_0;
  wire term1_reg_518_reg_i_143_n_0;
  wire term1_reg_518_reg_i_144_n_0;
  wire term1_reg_518_reg_i_145_n_0;
  wire term1_reg_518_reg_i_146_n_0;
  wire term1_reg_518_reg_i_147_n_0;
  wire term1_reg_518_reg_i_148_n_0;
  wire term1_reg_518_reg_i_149_n_0;
  wire term1_reg_518_reg_i_40_n_1;
  wire term1_reg_518_reg_i_40_n_2;
  wire term1_reg_518_reg_i_40_n_3;
  wire term1_reg_518_reg_i_41_n_0;
  wire term1_reg_518_reg_i_42_n_0;
  wire term1_reg_518_reg_i_42_n_1;
  wire term1_reg_518_reg_i_42_n_2;
  wire term1_reg_518_reg_i_42_n_3;
  wire term1_reg_518_reg_i_42_n_6;
  wire term1_reg_518_reg_i_42_n_7;
  wire term1_reg_518_reg_i_43_n_0;
  wire term1_reg_518_reg_i_43_n_1;
  wire term1_reg_518_reg_i_43_n_2;
  wire term1_reg_518_reg_i_43_n_3;
  wire term1_reg_518_reg_i_43_n_4;
  wire term1_reg_518_reg_i_43_n_5;
  wire term1_reg_518_reg_i_43_n_6;
  wire term1_reg_518_reg_i_43_n_7;
  wire term1_reg_518_reg_i_44_n_0;
  wire term1_reg_518_reg_i_44_n_1;
  wire term1_reg_518_reg_i_44_n_2;
  wire term1_reg_518_reg_i_44_n_3;
  wire term1_reg_518_reg_i_44_n_4;
  wire term1_reg_518_reg_i_44_n_5;
  wire term1_reg_518_reg_i_44_n_6;
  wire term1_reg_518_reg_i_44_n_7;
  wire term1_reg_518_reg_i_45_n_0;
  wire term1_reg_518_reg_i_45_n_1;
  wire term1_reg_518_reg_i_45_n_2;
  wire term1_reg_518_reg_i_45_n_3;
  wire term1_reg_518_reg_i_45_n_4;
  wire term1_reg_518_reg_i_45_n_5;
  wire term1_reg_518_reg_i_45_n_6;
  wire term1_reg_518_reg_i_45_n_7;
  wire term1_reg_518_reg_i_46_n_0;
  wire term1_reg_518_reg_i_46_n_1;
  wire term1_reg_518_reg_i_46_n_2;
  wire term1_reg_518_reg_i_46_n_3;
  wire term1_reg_518_reg_i_46_n_4;
  wire term1_reg_518_reg_i_47_n_1;
  wire term1_reg_518_reg_i_47_n_2;
  wire term1_reg_518_reg_i_47_n_3;
  wire term1_reg_518_reg_i_48_n_0;
  wire term1_reg_518_reg_i_49_n_0;
  wire term1_reg_518_reg_i_49_n_1;
  wire term1_reg_518_reg_i_49_n_2;
  wire term1_reg_518_reg_i_49_n_3;
  wire term1_reg_518_reg_i_49_n_6;
  wire term1_reg_518_reg_i_49_n_7;
  wire term1_reg_518_reg_i_50_n_0;
  wire term1_reg_518_reg_i_50_n_1;
  wire term1_reg_518_reg_i_50_n_2;
  wire term1_reg_518_reg_i_50_n_3;
  wire term1_reg_518_reg_i_50_n_4;
  wire term1_reg_518_reg_i_50_n_5;
  wire term1_reg_518_reg_i_50_n_6;
  wire term1_reg_518_reg_i_50_n_7;
  wire term1_reg_518_reg_i_51_n_0;
  wire term1_reg_518_reg_i_51_n_1;
  wire term1_reg_518_reg_i_51_n_2;
  wire term1_reg_518_reg_i_51_n_3;
  wire term1_reg_518_reg_i_51_n_4;
  wire term1_reg_518_reg_i_51_n_5;
  wire term1_reg_518_reg_i_51_n_6;
  wire term1_reg_518_reg_i_51_n_7;
  wire term1_reg_518_reg_i_52_n_0;
  wire term1_reg_518_reg_i_52_n_1;
  wire term1_reg_518_reg_i_52_n_2;
  wire term1_reg_518_reg_i_52_n_3;
  wire term1_reg_518_reg_i_52_n_4;
  wire term1_reg_518_reg_i_52_n_5;
  wire term1_reg_518_reg_i_52_n_6;
  wire term1_reg_518_reg_i_52_n_7;
  wire term1_reg_518_reg_i_53_n_0;
  wire term1_reg_518_reg_i_53_n_1;
  wire term1_reg_518_reg_i_53_n_2;
  wire term1_reg_518_reg_i_53_n_3;
  wire term1_reg_518_reg_i_53_n_4;
  wire term1_reg_518_reg_i_65_n_0;
  wire term1_reg_518_reg_i_66_n_0;
  wire term1_reg_518_reg_i_67_n_0;
  wire term1_reg_518_reg_i_68_n_1;
  wire term1_reg_518_reg_i_68_n_2;
  wire term1_reg_518_reg_i_68_n_3;
  wire term1_reg_518_reg_i_69_n_0;
  wire [3:0]term1_reg_518_reg_i_70_0;
  wire term1_reg_518_reg_i_70_n_0;
  wire term1_reg_518_reg_i_71_n_0;
  wire term1_reg_518_reg_i_80_n_0;
  wire term1_reg_518_reg_i_84_n_0;
  wire term1_reg_518_reg_i_85_n_0;
  wire term1_reg_518_reg_i_86_n_0;
  wire term1_reg_518_reg_i_87_n_0;
  wire term1_reg_518_reg_i_88_n_0;
  wire term1_reg_518_reg_i_89_n_0;
  wire term1_reg_518_reg_i_89_n_1;
  wire term1_reg_518_reg_i_89_n_2;
  wire term1_reg_518_reg_i_89_n_3;
  wire term1_reg_518_reg_i_90_n_0;
  wire term1_reg_518_reg_i_91_n_0;
  wire term1_reg_518_reg_i_92_n_0;
  wire term1_reg_518_reg_i_93_n_0;
  wire term1_reg_518_reg_i_94_n_0;
  wire term1_reg_518_reg_i_95_n_0;
  wire term1_reg_518_reg_i_96_n_0;
  wire term1_reg_518_reg_i_97_n_1;
  wire term1_reg_518_reg_i_97_n_2;
  wire term1_reg_518_reg_i_97_n_3;
  wire term1_reg_518_reg_i_98_n_0;
  wire [3:0]term1_reg_518_reg_i_99_0;
  wire term1_reg_518_reg_i_99_n_0;
  wire [5:0]tmp_1_fu_305_p4;
  wire [5:0]tmp_fu_251_p4;
  wire [3:3]\NLW_sum_i_reg[28]_i_1_CO_UNCONNECTED ;
  wire [3:3]\NLW_sum_q_reg[28]_i_1_CO_UNCONNECTED ;
  wire [3:0]NLW_term1_reg_518_reg_i_118_O_UNCONNECTED;
  wire [3:0]NLW_term1_reg_518_reg_i_128_O_UNCONNECTED;
  wire [3:0]NLW_term1_reg_518_reg_i_137_O_UNCONNECTED;
  wire [3:0]NLW_term1_reg_518_reg_i_40_O_UNCONNECTED;
  wire [2:0]NLW_term1_reg_518_reg_i_46_O_UNCONNECTED;
  wire [3:0]NLW_term1_reg_518_reg_i_47_O_UNCONNECTED;
  wire [2:0]NLW_term1_reg_518_reg_i_53_O_UNCONNECTED;
  wire [3:3]NLW_term1_reg_518_reg_i_68_CO_UNCONNECTED;
  wire [3:0]NLW_term1_reg_518_reg_i_89_O_UNCONNECTED;
  wire [3:3]NLW_term1_reg_518_reg_i_97_CO_UNCONNECTED;

  LUT3 #(
    .INIT(8'h0D)) 
    \B_V_data_1_payload_A[31]_i_1 
       (.I0(in_stream_TVALID_int_regslice),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(\B_V_data_1_payload_A[31]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[0]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[10]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[11]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[12]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[13] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[13]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[14] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[14]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[15] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[15]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[16] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[16]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[17] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[17]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[18] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[18]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[19] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[19]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[1]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[20] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[20]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[21] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[21]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[21] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[22] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[22]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[22] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[23] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[23]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[23] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[24] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[24]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[24] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[25] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[25]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[25] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[26] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[26]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[26] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[27] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[27]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[27] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[28] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[28]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[28] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[29] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[29]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[29] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[2]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[30] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[30]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[30] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[31] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[31]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[3]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[4]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[5]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[6]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[7]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[8]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[31]_i_1_n_0 ),
        .D(in_stream_TDATA[9]),
        .Q(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .R(1'b0));
  LUT3 #(
    .INIT(8'hA2)) 
    \B_V_data_1_payload_B[31]_i_1 
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
  FDRE \B_V_data_1_payload_B_reg[16] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[16]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[17] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[17]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[18] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[18]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[19] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[19]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[1]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[20] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[20]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[21] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[21]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[21] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[22] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[22]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[22] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[23] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[23]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[23] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[24] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[24]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[24] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[25] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[25]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[25] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[26] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[26]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[26] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[27] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[27]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[27] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[28] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[28]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[28] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[29] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[29]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[29] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[2]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[30] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[30]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[30] ),
        .R(1'b0));
  FDRE \B_V_data_1_payload_B_reg[31] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(in_stream_TDATA[31]),
        .Q(\B_V_data_1_payload_B_reg_n_0_[31] ),
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
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hB4)) 
    B_V_data_1_sel_rd_i_1__0
       (.I0(\B_V_data_1_state_reg[1]_1 ),
        .I1(in_stream_TVALID_int_regslice),
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
    B_V_data_1_sel_wr_i_1
       (.I0(in_stream_TVALID),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'hA8AAA000)) 
    \B_V_data_1_state[0]_i_1 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg[1]_1 ),
        .I2(in_stream_TVALID),
        .I3(\B_V_data_1_state_reg[1]_0 ),
        .I4(in_stream_TVALID_int_regslice),
        .O(\B_V_data_1_state[0]_i_1_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_state[1]_i_1 
       (.I0(ap_rst_n),
        .O(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT4 #(
    .INIT(16'h77F7)) 
    \B_V_data_1_state[1]_i_2 
       (.I0(\B_V_data_1_state_reg[1]_1 ),
        .I1(in_stream_TVALID_int_regslice),
        .I2(\B_V_data_1_state_reg[1]_0 ),
        .I3(in_stream_TVALID),
        .O(\B_V_data_1_state[1]_i_2_n_0 ));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1_n_0 ),
        .Q(in_stream_TVALID_int_regslice),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[1]_i_2_n_0 ),
        .Q(\B_V_data_1_state_reg[1]_0 ),
        .R(ap_rst_n_inv));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[0]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I3(sum_i_reg[3]),
        .O(\sum_i[0]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[0]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I3(sum_i_reg[2]),
        .O(\sum_i[0]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[0]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I3(sum_i_reg[1]),
        .O(\sum_i[0]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[0]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I3(sum_i_reg[0]),
        .O(\sum_i[0]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[12]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[15]),
        .O(\sum_i[12]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[12]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I3(sum_i_reg[14]),
        .O(\sum_i[12]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[12]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I3(sum_i_reg[13]),
        .O(\sum_i[12]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[12]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I3(sum_i_reg[12]),
        .O(\sum_i[12]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[16]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[19]),
        .O(\sum_i[16]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[16]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[18]),
        .O(\sum_i[16]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[16]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[17]),
        .O(\sum_i[16]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[16]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[16]),
        .O(\sum_i[16]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[20]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[23]),
        .O(\sum_i[20]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[20]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[22]),
        .O(\sum_i[20]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[20]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[21]),
        .O(\sum_i[20]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[20]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[20]),
        .O(\sum_i[20]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[24]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[27]),
        .O(\sum_i[24]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[24]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[26]),
        .O(\sum_i[24]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[24]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[25]),
        .O(\sum_i[24]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[24]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[24]),
        .O(\sum_i[24]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[28]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[31]),
        .O(\sum_i[28]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[28]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[30]),
        .O(\sum_i[28]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[28]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[29]),
        .O(\sum_i[28]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[28]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[28]),
        .O(\sum_i[28]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[4]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I3(sum_i_reg[7]),
        .O(\sum_i[4]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[4]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I3(sum_i_reg[6]),
        .O(\sum_i[4]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[4]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I3(sum_i_reg[5]),
        .O(\sum_i[4]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[4]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I3(sum_i_reg[4]),
        .O(\sum_i[4]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[8]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I3(sum_i_reg[11]),
        .O(\sum_i[8]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[8]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I3(sum_i_reg[10]),
        .O(\sum_i[8]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[8]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I3(sum_i_reg[9]),
        .O(\sum_i[8]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_i[8]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I3(sum_i_reg[8]),
        .O(\sum_i[8]_i_5_n_0 ));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_i_reg[0]_i_1 
       (.CI(1'b0),
        .CO({\sum_i_reg[0]_i_1_n_0 ,\sum_i_reg[0]_i_1_n_1 ,\sum_i_reg[0]_i_1_n_2 ,\sum_i_reg[0]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_i_reg[3:0]),
        .O(O),
        .S({\sum_i[0]_i_2_n_0 ,\sum_i[0]_i_3_n_0 ,\sum_i[0]_i_4_n_0 ,\sum_i[0]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_i_reg[12]_i_1 
       (.CI(\sum_i_reg[8]_i_1_n_0 ),
        .CO({\sum_i_reg[12]_i_1_n_0 ,\sum_i_reg[12]_i_1_n_1 ,\sum_i_reg[12]_i_1_n_2 ,\sum_i_reg[12]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_i_reg[15:12]),
        .O(\sum_i_reg[15] ),
        .S({\sum_i[12]_i_2_n_0 ,\sum_i[12]_i_3_n_0 ,\sum_i[12]_i_4_n_0 ,\sum_i[12]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_i_reg[16]_i_1 
       (.CI(\sum_i_reg[12]_i_1_n_0 ),
        .CO({\sum_i_reg[16]_i_1_n_0 ,\sum_i_reg[16]_i_1_n_1 ,\sum_i_reg[16]_i_1_n_2 ,\sum_i_reg[16]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_i_reg[19:16]),
        .O(\sum_i_reg[19] ),
        .S({\sum_i[16]_i_2_n_0 ,\sum_i[16]_i_3_n_0 ,\sum_i[16]_i_4_n_0 ,\sum_i[16]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_i_reg[20]_i_1 
       (.CI(\sum_i_reg[16]_i_1_n_0 ),
        .CO({\sum_i_reg[20]_i_1_n_0 ,\sum_i_reg[20]_i_1_n_1 ,\sum_i_reg[20]_i_1_n_2 ,\sum_i_reg[20]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_i_reg[23:20]),
        .O(\sum_i_reg[23] ),
        .S({\sum_i[20]_i_2_n_0 ,\sum_i[20]_i_3_n_0 ,\sum_i[20]_i_4_n_0 ,\sum_i[20]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_i_reg[24]_i_1 
       (.CI(\sum_i_reg[20]_i_1_n_0 ),
        .CO({\sum_i_reg[24]_i_1_n_0 ,\sum_i_reg[24]_i_1_n_1 ,\sum_i_reg[24]_i_1_n_2 ,\sum_i_reg[24]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_i_reg[27:24]),
        .O(\sum_i_reg[27] ),
        .S({\sum_i[24]_i_2_n_0 ,\sum_i[24]_i_3_n_0 ,\sum_i[24]_i_4_n_0 ,\sum_i[24]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_i_reg[28]_i_1 
       (.CI(\sum_i_reg[24]_i_1_n_0 ),
        .CO({\NLW_sum_i_reg[28]_i_1_CO_UNCONNECTED [3],\sum_i_reg[28]_i_1_n_1 ,\sum_i_reg[28]_i_1_n_2 ,\sum_i_reg[28]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,sum_i_reg[30:28]}),
        .O(\sum_i_reg[30] ),
        .S({\sum_i[28]_i_2_n_0 ,\sum_i[28]_i_3_n_0 ,\sum_i[28]_i_4_n_0 ,\sum_i[28]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_i_reg[4]_i_1 
       (.CI(\sum_i_reg[0]_i_1_n_0 ),
        .CO({\sum_i_reg[4]_i_1_n_0 ,\sum_i_reg[4]_i_1_n_1 ,\sum_i_reg[4]_i_1_n_2 ,\sum_i_reg[4]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_i_reg[7:4]),
        .O(\sum_i_reg[7] ),
        .S({\sum_i[4]_i_2_n_0 ,\sum_i[4]_i_3_n_0 ,\sum_i[4]_i_4_n_0 ,\sum_i[4]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_i_reg[8]_i_1 
       (.CI(\sum_i_reg[4]_i_1_n_0 ),
        .CO({\sum_i_reg[8]_i_1_n_0 ,\sum_i_reg[8]_i_1_n_1 ,\sum_i_reg[8]_i_1_n_2 ,\sum_i_reg[8]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_i_reg[11:8]),
        .O(\sum_i_reg[11] ),
        .S({\sum_i[8]_i_2_n_0 ,\sum_i[8]_i_3_n_0 ,\sum_i[8]_i_4_n_0 ,\sum_i[8]_i_5_n_0 }));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[0]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .I3(sum_q_reg[3]),
        .O(\sum_q[0]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[0]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .I3(sum_q_reg[2]),
        .O(\sum_q[0]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[0]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .I3(sum_q_reg[1]),
        .O(\sum_q[0]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[0]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .I3(sum_q_reg[0]),
        .O(\sum_q[0]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[12]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[15]),
        .O(\sum_q[12]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[12]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[30] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[30] ),
        .I3(sum_q_reg[14]),
        .O(\sum_q[12]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[12]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[29] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[29] ),
        .I3(sum_q_reg[13]),
        .O(\sum_q[12]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[12]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[28] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[28] ),
        .I3(sum_q_reg[12]),
        .O(\sum_q[12]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[16]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[19]),
        .O(\sum_q[16]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[16]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[18]),
        .O(\sum_q[16]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[16]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[17]),
        .O(\sum_q[16]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[16]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[16]),
        .O(\sum_q[16]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[20]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[23]),
        .O(\sum_q[20]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[20]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[22]),
        .O(\sum_q[20]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[20]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[21]),
        .O(\sum_q[20]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[20]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[20]),
        .O(\sum_q[20]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[24]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[27]),
        .O(\sum_q[24]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[24]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[26]),
        .O(\sum_q[24]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[24]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[25]),
        .O(\sum_q[24]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[24]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[24]),
        .O(\sum_q[24]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[28]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[31]),
        .O(\sum_q[28]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[28]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[30]),
        .O(\sum_q[28]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[28]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[29]),
        .O(\sum_q[28]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[28]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[28]),
        .O(\sum_q[28]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[4]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[23] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[23] ),
        .I3(sum_q_reg[7]),
        .O(\sum_q[4]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[4]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[22] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[22] ),
        .I3(sum_q_reg[6]),
        .O(\sum_q[4]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[4]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[21] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[21] ),
        .I3(sum_q_reg[5]),
        .O(\sum_q[4]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[4]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .I3(sum_q_reg[4]),
        .O(\sum_q[4]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[8]_i_2 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[27] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[27] ),
        .I3(sum_q_reg[11]),
        .O(\sum_q[8]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[8]_i_3 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[26] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[26] ),
        .I3(sum_q_reg[10]),
        .O(\sum_q[8]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[8]_i_4 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[25] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[25] ),
        .I3(sum_q_reg[9]),
        .O(\sum_q[8]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h1BE4)) 
    \sum_q[8]_i_5 
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[24] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[24] ),
        .I3(sum_q_reg[8]),
        .O(\sum_q[8]_i_5_n_0 ));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_q_reg[0]_i_1 
       (.CI(1'b0),
        .CO({\sum_q_reg[0]_i_1_n_0 ,\sum_q_reg[0]_i_1_n_1 ,\sum_q_reg[0]_i_1_n_2 ,\sum_q_reg[0]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_q_reg[3:0]),
        .O(\sum_q_reg[3] ),
        .S({\sum_q[0]_i_2_n_0 ,\sum_q[0]_i_3_n_0 ,\sum_q[0]_i_4_n_0 ,\sum_q[0]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_q_reg[12]_i_1 
       (.CI(\sum_q_reg[8]_i_1_n_0 ),
        .CO({\sum_q_reg[12]_i_1_n_0 ,\sum_q_reg[12]_i_1_n_1 ,\sum_q_reg[12]_i_1_n_2 ,\sum_q_reg[12]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_q_reg[15:12]),
        .O(\sum_q_reg[15] ),
        .S({\sum_q[12]_i_2_n_0 ,\sum_q[12]_i_3_n_0 ,\sum_q[12]_i_4_n_0 ,\sum_q[12]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_q_reg[16]_i_1 
       (.CI(\sum_q_reg[12]_i_1_n_0 ),
        .CO({\sum_q_reg[16]_i_1_n_0 ,\sum_q_reg[16]_i_1_n_1 ,\sum_q_reg[16]_i_1_n_2 ,\sum_q_reg[16]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_q_reg[19:16]),
        .O(\sum_q_reg[19] ),
        .S({\sum_q[16]_i_2_n_0 ,\sum_q[16]_i_3_n_0 ,\sum_q[16]_i_4_n_0 ,\sum_q[16]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_q_reg[20]_i_1 
       (.CI(\sum_q_reg[16]_i_1_n_0 ),
        .CO({\sum_q_reg[20]_i_1_n_0 ,\sum_q_reg[20]_i_1_n_1 ,\sum_q_reg[20]_i_1_n_2 ,\sum_q_reg[20]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_q_reg[23:20]),
        .O(\sum_q_reg[23] ),
        .S({\sum_q[20]_i_2_n_0 ,\sum_q[20]_i_3_n_0 ,\sum_q[20]_i_4_n_0 ,\sum_q[20]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_q_reg[24]_i_1 
       (.CI(\sum_q_reg[20]_i_1_n_0 ),
        .CO({\sum_q_reg[24]_i_1_n_0 ,\sum_q_reg[24]_i_1_n_1 ,\sum_q_reg[24]_i_1_n_2 ,\sum_q_reg[24]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_q_reg[27:24]),
        .O(\sum_q_reg[27] ),
        .S({\sum_q[24]_i_2_n_0 ,\sum_q[24]_i_3_n_0 ,\sum_q[24]_i_4_n_0 ,\sum_q[24]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_q_reg[28]_i_1 
       (.CI(\sum_q_reg[24]_i_1_n_0 ),
        .CO({\NLW_sum_q_reg[28]_i_1_CO_UNCONNECTED [3],\sum_q_reg[28]_i_1_n_1 ,\sum_q_reg[28]_i_1_n_2 ,\sum_q_reg[28]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,sum_q_reg[30:28]}),
        .O(\sum_q_reg[30] ),
        .S({\sum_q[28]_i_2_n_0 ,\sum_q[28]_i_3_n_0 ,\sum_q[28]_i_4_n_0 ,\sum_q[28]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_q_reg[4]_i_1 
       (.CI(\sum_q_reg[0]_i_1_n_0 ),
        .CO({\sum_q_reg[4]_i_1_n_0 ,\sum_q_reg[4]_i_1_n_1 ,\sum_q_reg[4]_i_1_n_2 ,\sum_q_reg[4]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_q_reg[7:4]),
        .O(\sum_q_reg[7] ),
        .S({\sum_q[4]_i_2_n_0 ,\sum_q[4]_i_3_n_0 ,\sum_q[4]_i_4_n_0 ,\sum_q[4]_i_5_n_0 }));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sum_q_reg[8]_i_1 
       (.CI(\sum_q_reg[4]_i_1_n_0 ),
        .CO({\sum_q_reg[8]_i_1_n_0 ,\sum_q_reg[8]_i_1_n_1 ,\sum_q_reg[8]_i_1_n_2 ,\sum_q_reg[8]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI(sum_q_reg[11:8]),
        .O(\sum_q_reg[11] ),
        .S({\sum_q[8]_i_2_n_0 ,\sum_q[8]_i_3_n_0 ,\sum_q[8]_i_4_n_0 ,\sum_q[8]_i_5_n_0 }));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_10
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_43_n_7),
        .O(B[9]));
  LUT2 #(
    .INIT(4'h2)) 
    term1_reg_518_reg_i_100
       (.I0(tmp_fu_251_p4[0]),
        .I1(term1_reg_518_reg_i_49_n_6),
        .O(term1_reg_518_reg_i_100_n_0));
  LUT3 #(
    .INIT(8'h1B)) 
    term1_reg_518_reg_i_109
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .O(term1_reg_518_reg_i_109_n_0));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_11
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_44_n_4),
        .O(B[8]));
  LUT4 #(
    .INIT(16'h1BE4)) 
    term1_reg_518_reg_i_113
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[16]),
        .O(term1_reg_518_reg_i_113_n_0));
  LUT4 #(
    .INIT(16'h1BE4)) 
    term1_reg_518_reg_i_114
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I3(sum_i_reg[15]),
        .O(term1_reg_518_reg_i_114_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_115
       (.I0(sum_i_reg[14]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .O(term1_reg_518_reg_i_115_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_116
       (.I0(sum_i_reg[13]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .O(term1_reg_518_reg_i_116_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_117
       (.I0(sum_i_reg[12]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .O(term1_reg_518_reg_i_117_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_118
       (.CI(term1_reg_518_reg_i_137_n_0),
        .CO({term1_reg_518_reg_i_118_n_0,term1_reg_518_reg_i_118_n_1,term1_reg_518_reg_i_118_n_2,term1_reg_518_reg_i_118_n_3}),
        .CYINIT(1'b0),
        .DI(sum_i_reg[7:4]),
        .O(NLW_term1_reg_518_reg_i_118_O_UNCONNECTED[3:0]),
        .S({term1_reg_518_reg_i_138_n_0,term1_reg_518_reg_i_139_n_0,term1_reg_518_reg_i_140_n_0,term1_reg_518_reg_i_141_n_0}));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_119
       (.I0(sum_i_reg[11]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .O(term1_reg_518_reg_i_119_n_0));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_12
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_44_n_5),
        .O(B[7]));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_120
       (.I0(sum_i_reg[10]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .O(term1_reg_518_reg_i_120_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_121
       (.I0(sum_i_reg[9]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .O(term1_reg_518_reg_i_121_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_122
       (.I0(sum_i_reg[8]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .O(term1_reg_518_reg_i_122_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_128
       (.CI(1'b0),
        .CO({term1_reg_518_reg_i_128_n_0,term1_reg_518_reg_i_128_n_1,term1_reg_518_reg_i_128_n_2,term1_reg_518_reg_i_128_n_3}),
        .CYINIT(1'b0),
        .DI(sum_q_reg[3:0]),
        .O(NLW_term1_reg_518_reg_i_128_O_UNCONNECTED[3:0]),
        .S({term1_reg_518_reg_i_142_n_0,term1_reg_518_reg_i_143_n_0,term1_reg_518_reg_i_144_n_0,term1_reg_518_reg_i_145_n_0}));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_129
       (.I0(sum_q_reg[7]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[23] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[23] ),
        .O(term1_reg_518_reg_i_129_n_0));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_13
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_44_n_6),
        .O(B[6]));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_130
       (.I0(sum_q_reg[6]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[22] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[22] ),
        .O(term1_reg_518_reg_i_130_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_131
       (.I0(sum_q_reg[5]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[21] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[21] ),
        .O(term1_reg_518_reg_i_131_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_132
       (.I0(sum_q_reg[4]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[20] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[20] ),
        .O(term1_reg_518_reg_i_132_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_137
       (.CI(1'b0),
        .CO({term1_reg_518_reg_i_137_n_0,term1_reg_518_reg_i_137_n_1,term1_reg_518_reg_i_137_n_2,term1_reg_518_reg_i_137_n_3}),
        .CYINIT(1'b0),
        .DI(sum_i_reg[3:0]),
        .O(NLW_term1_reg_518_reg_i_137_O_UNCONNECTED[3:0]),
        .S({term1_reg_518_reg_i_146_n_0,term1_reg_518_reg_i_147_n_0,term1_reg_518_reg_i_148_n_0,term1_reg_518_reg_i_149_n_0}));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_138
       (.I0(sum_i_reg[7]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .O(term1_reg_518_reg_i_138_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_139
       (.I0(sum_i_reg[6]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .O(term1_reg_518_reg_i_139_n_0));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_14
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_44_n_7),
        .O(B[5]));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_140
       (.I0(sum_i_reg[5]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .O(term1_reg_518_reg_i_140_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_141
       (.I0(sum_i_reg[4]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .O(term1_reg_518_reg_i_141_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_142
       (.I0(sum_q_reg[3]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[19] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[19] ),
        .O(term1_reg_518_reg_i_142_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_143
       (.I0(sum_q_reg[2]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[18] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[18] ),
        .O(term1_reg_518_reg_i_143_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_144
       (.I0(sum_q_reg[1]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[17] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[17] ),
        .O(term1_reg_518_reg_i_144_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_145
       (.I0(sum_q_reg[0]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[16] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[16] ),
        .O(term1_reg_518_reg_i_145_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_146
       (.I0(sum_i_reg[3]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .O(term1_reg_518_reg_i_146_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_147
       (.I0(sum_i_reg[2]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .O(term1_reg_518_reg_i_147_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_148
       (.I0(sum_i_reg[1]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .O(term1_reg_518_reg_i_148_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_149
       (.I0(sum_i_reg[0]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .O(term1_reg_518_reg_i_149_n_0));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_15
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_45_n_4),
        .O(B[4]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_16
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_45_n_5),
        .O(B[3]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_17
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_45_n_6),
        .O(B[2]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_18
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_45_n_7),
        .O(B[1]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_19
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_46_n_4),
        .O(B[0]));
  LUT3 #(
    .INIT(8'h32)) 
    term1_reg_518_reg_i_20
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(tmp_fu_251_p4[0]),
        .O(A[15]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_21
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_49_n_6),
        .O(A[14]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_22
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_49_n_7),
        .O(A[13]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_23
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_50_n_4),
        .O(A[12]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_24
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_50_n_5),
        .O(A[11]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_25
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_50_n_6),
        .O(A[10]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_26
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_50_n_7),
        .O(A[9]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_27
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_51_n_4),
        .O(A[8]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_28
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_51_n_5),
        .O(A[7]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_29
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_51_n_6),
        .O(A[6]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_30
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_51_n_7),
        .O(A[5]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_31
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_52_n_4),
        .O(A[4]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_32
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_52_n_5),
        .O(A[3]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_33
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_52_n_6),
        .O(A[2]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_34
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_52_n_7),
        .O(A[1]));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_35
       (.I0(icmp_ln23_fu_267_p2),
        .I1(term1_reg_518_reg_i_48_n_0),
        .I2(term1_reg_518_reg_i_53_n_4),
        .O(A[0]));
  LUT3 #(
    .INIT(8'h32)) 
    term1_reg_518_reg_i_4
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(tmp_1_fu_305_p4[0]),
        .O(B[15]));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 term1_reg_518_reg_i_40
       (.CI(1'b0),
        .CO({icmp_ln23_1_fu_321_p2,term1_reg_518_reg_i_40_n_1,term1_reg_518_reg_i_40_n_2,term1_reg_518_reg_i_40_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,term1_reg_518_reg_i_65_n_0,term1_reg_518_reg_i_66_n_0,term1_reg_518_reg_i_67_n_0}),
        .O(NLW_term1_reg_518_reg_i_40_O_UNCONNECTED[3:0]),
        .S({tmp_1_fu_305_p4[5],term1_reg_518_reg_i_69_n_0,term1_reg_518_reg_i_70_n_0,term1_reg_518_reg_i_71_n_0}));
  LUT6 #(
    .INIT(64'h00000000FFFFFFFE)) 
    term1_reg_518_reg_i_41
       (.I0(tmp_1_fu_305_p4[4]),
        .I1(tmp_1_fu_305_p4[3]),
        .I2(tmp_1_fu_305_p4[2]),
        .I3(tmp_1_fu_305_p4[1]),
        .I4(tmp_1_fu_305_p4[0]),
        .I5(tmp_1_fu_305_p4[5]),
        .O(term1_reg_518_reg_i_41_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_42
       (.CI(term1_reg_518_reg_i_43_n_0),
        .CO({term1_reg_518_reg_i_42_n_0,term1_reg_518_reg_i_42_n_1,term1_reg_518_reg_i_42_n_2,term1_reg_518_reg_i_42_n_3}),
        .CYINIT(1'b0),
        .DI(sum_q_reg[26:23]),
        .O({tmp_1_fu_305_p4[1:0],term1_reg_518_reg_i_42_n_6,term1_reg_518_reg_i_42_n_7}),
        .S(p_reg_reg_3));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_43
       (.CI(term1_reg_518_reg_i_44_n_0),
        .CO({term1_reg_518_reg_i_43_n_0,term1_reg_518_reg_i_43_n_1,term1_reg_518_reg_i_43_n_2,term1_reg_518_reg_i_43_n_3}),
        .CYINIT(1'b0),
        .DI(sum_q_reg[22:19]),
        .O({term1_reg_518_reg_i_43_n_4,term1_reg_518_reg_i_43_n_5,term1_reg_518_reg_i_43_n_6,term1_reg_518_reg_i_43_n_7}),
        .S(p_reg_reg_2));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_44
       (.CI(term1_reg_518_reg_i_45_n_0),
        .CO({term1_reg_518_reg_i_44_n_0,term1_reg_518_reg_i_44_n_1,term1_reg_518_reg_i_44_n_2,term1_reg_518_reg_i_44_n_3}),
        .CYINIT(1'b0),
        .DI({sum_q_reg[18:16],term1_reg_518_reg_i_80_n_0}),
        .O({term1_reg_518_reg_i_44_n_4,term1_reg_518_reg_i_44_n_5,term1_reg_518_reg_i_44_n_6,term1_reg_518_reg_i_44_n_7}),
        .S({p_reg_reg_1,term1_reg_518_reg_i_84_n_0}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_45
       (.CI(term1_reg_518_reg_i_46_n_0),
        .CO({term1_reg_518_reg_i_45_n_0,term1_reg_518_reg_i_45_n_1,term1_reg_518_reg_i_45_n_2,term1_reg_518_reg_i_45_n_3}),
        .CYINIT(1'b0),
        .DI(sum_q_reg[15:12]),
        .O({term1_reg_518_reg_i_45_n_4,term1_reg_518_reg_i_45_n_5,term1_reg_518_reg_i_45_n_6,term1_reg_518_reg_i_45_n_7}),
        .S({term1_reg_518_reg_i_85_n_0,term1_reg_518_reg_i_86_n_0,term1_reg_518_reg_i_87_n_0,term1_reg_518_reg_i_88_n_0}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_46
       (.CI(term1_reg_518_reg_i_89_n_0),
        .CO({term1_reg_518_reg_i_46_n_0,term1_reg_518_reg_i_46_n_1,term1_reg_518_reg_i_46_n_2,term1_reg_518_reg_i_46_n_3}),
        .CYINIT(1'b0),
        .DI(sum_q_reg[11:8]),
        .O({term1_reg_518_reg_i_46_n_4,NLW_term1_reg_518_reg_i_46_O_UNCONNECTED[2:0]}),
        .S({term1_reg_518_reg_i_90_n_0,term1_reg_518_reg_i_91_n_0,term1_reg_518_reg_i_92_n_0,term1_reg_518_reg_i_93_n_0}));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 term1_reg_518_reg_i_47
       (.CI(1'b0),
        .CO({icmp_ln23_fu_267_p2,term1_reg_518_reg_i_47_n_1,term1_reg_518_reg_i_47_n_2,term1_reg_518_reg_i_47_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,term1_reg_518_reg_i_94_n_0,term1_reg_518_reg_i_95_n_0,term1_reg_518_reg_i_96_n_0}),
        .O(NLW_term1_reg_518_reg_i_47_O_UNCONNECTED[3:0]),
        .S({tmp_fu_251_p4[5],term1_reg_518_reg_i_98_n_0,term1_reg_518_reg_i_99_n_0,term1_reg_518_reg_i_100_n_0}));
  LUT6 #(
    .INIT(64'h00000000FFFFFFFE)) 
    term1_reg_518_reg_i_48
       (.I0(tmp_fu_251_p4[4]),
        .I1(tmp_fu_251_p4[3]),
        .I2(tmp_fu_251_p4[2]),
        .I3(tmp_fu_251_p4[1]),
        .I4(tmp_fu_251_p4[0]),
        .I5(tmp_fu_251_p4[5]),
        .O(term1_reg_518_reg_i_48_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_49
       (.CI(term1_reg_518_reg_i_50_n_0),
        .CO({term1_reg_518_reg_i_49_n_0,term1_reg_518_reg_i_49_n_1,term1_reg_518_reg_i_49_n_2,term1_reg_518_reg_i_49_n_3}),
        .CYINIT(1'b0),
        .DI(sum_i_reg[26:23]),
        .O({tmp_fu_251_p4[1:0],term1_reg_518_reg_i_49_n_6,term1_reg_518_reg_i_49_n_7}),
        .S(p_reg_reg_0));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_5
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_42_n_6),
        .O(B[14]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_50
       (.CI(term1_reg_518_reg_i_51_n_0),
        .CO({term1_reg_518_reg_i_50_n_0,term1_reg_518_reg_i_50_n_1,term1_reg_518_reg_i_50_n_2,term1_reg_518_reg_i_50_n_3}),
        .CYINIT(1'b0),
        .DI(sum_i_reg[22:19]),
        .O({term1_reg_518_reg_i_50_n_4,term1_reg_518_reg_i_50_n_5,term1_reg_518_reg_i_50_n_6,term1_reg_518_reg_i_50_n_7}),
        .S(p_reg_reg));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_51
       (.CI(term1_reg_518_reg_i_52_n_0),
        .CO({term1_reg_518_reg_i_51_n_0,term1_reg_518_reg_i_51_n_1,term1_reg_518_reg_i_51_n_2,term1_reg_518_reg_i_51_n_3}),
        .CYINIT(1'b0),
        .DI({sum_i_reg[18:16],term1_reg_518_reg_i_109_n_0}),
        .O({term1_reg_518_reg_i_51_n_4,term1_reg_518_reg_i_51_n_5,term1_reg_518_reg_i_51_n_6,term1_reg_518_reg_i_51_n_7}),
        .S({S,term1_reg_518_reg_i_113_n_0}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_52
       (.CI(term1_reg_518_reg_i_53_n_0),
        .CO({term1_reg_518_reg_i_52_n_0,term1_reg_518_reg_i_52_n_1,term1_reg_518_reg_i_52_n_2,term1_reg_518_reg_i_52_n_3}),
        .CYINIT(1'b0),
        .DI(sum_i_reg[15:12]),
        .O({term1_reg_518_reg_i_52_n_4,term1_reg_518_reg_i_52_n_5,term1_reg_518_reg_i_52_n_6,term1_reg_518_reg_i_52_n_7}),
        .S({term1_reg_518_reg_i_114_n_0,term1_reg_518_reg_i_115_n_0,term1_reg_518_reg_i_116_n_0,term1_reg_518_reg_i_117_n_0}));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_53
       (.CI(term1_reg_518_reg_i_118_n_0),
        .CO({term1_reg_518_reg_i_53_n_0,term1_reg_518_reg_i_53_n_1,term1_reg_518_reg_i_53_n_2,term1_reg_518_reg_i_53_n_3}),
        .CYINIT(1'b0),
        .DI(sum_i_reg[11:8]),
        .O({term1_reg_518_reg_i_53_n_4,NLW_term1_reg_518_reg_i_53_O_UNCONNECTED[2:0]}),
        .S({term1_reg_518_reg_i_119_n_0,term1_reg_518_reg_i_120_n_0,term1_reg_518_reg_i_121_n_0,term1_reg_518_reg_i_122_n_0}));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_6
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_42_n_7),
        .O(B[13]));
  LUT2 #(
    .INIT(4'h7)) 
    term1_reg_518_reg_i_65
       (.I0(tmp_1_fu_305_p4[3]),
        .I1(tmp_1_fu_305_p4[4]),
        .O(term1_reg_518_reg_i_65_n_0));
  LUT2 #(
    .INIT(4'h7)) 
    term1_reg_518_reg_i_66
       (.I0(tmp_1_fu_305_p4[1]),
        .I1(tmp_1_fu_305_p4[2]),
        .O(term1_reg_518_reg_i_66_n_0));
  LUT1 #(
    .INIT(2'h1)) 
    term1_reg_518_reg_i_67
       (.I0(tmp_1_fu_305_p4[0]),
        .O(term1_reg_518_reg_i_67_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_68
       (.CI(term1_reg_518_reg_i_42_n_0),
        .CO({NLW_term1_reg_518_reg_i_68_CO_UNCONNECTED[3],term1_reg_518_reg_i_68_n_1,term1_reg_518_reg_i_68_n_2,term1_reg_518_reg_i_68_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,sum_q_reg[29:27]}),
        .O(tmp_1_fu_305_p4[5:2]),
        .S(term1_reg_518_reg_i_70_0));
  LUT2 #(
    .INIT(4'h8)) 
    term1_reg_518_reg_i_69
       (.I0(tmp_1_fu_305_p4[3]),
        .I1(tmp_1_fu_305_p4[4]),
        .O(term1_reg_518_reg_i_69_n_0));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_7
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_43_n_4),
        .O(B[12]));
  LUT2 #(
    .INIT(4'h8)) 
    term1_reg_518_reg_i_70
       (.I0(tmp_1_fu_305_p4[1]),
        .I1(tmp_1_fu_305_p4[2]),
        .O(term1_reg_518_reg_i_70_n_0));
  LUT2 #(
    .INIT(4'h2)) 
    term1_reg_518_reg_i_71
       (.I0(tmp_1_fu_305_p4[0]),
        .I1(term1_reg_518_reg_i_42_n_6),
        .O(term1_reg_518_reg_i_71_n_0));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_8
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_43_n_5),
        .O(B[11]));
  LUT3 #(
    .INIT(8'h1B)) 
    term1_reg_518_reg_i_80
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .O(term1_reg_518_reg_i_80_n_0));
  LUT4 #(
    .INIT(16'h1BE4)) 
    term1_reg_518_reg_i_84
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[16]),
        .O(term1_reg_518_reg_i_84_n_0));
  LUT4 #(
    .INIT(16'h1BE4)) 
    term1_reg_518_reg_i_85
       (.I0(B_V_data_1_sel_rd_reg_n_0),
        .I1(\B_V_data_1_payload_A_reg_n_0_[31] ),
        .I2(\B_V_data_1_payload_B_reg_n_0_[31] ),
        .I3(sum_q_reg[15]),
        .O(term1_reg_518_reg_i_85_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_86
       (.I0(sum_q_reg[14]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[30] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[30] ),
        .O(term1_reg_518_reg_i_86_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_87
       (.I0(sum_q_reg[13]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[29] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[29] ),
        .O(term1_reg_518_reg_i_87_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_88
       (.I0(sum_q_reg[12]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[28] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[28] ),
        .O(term1_reg_518_reg_i_88_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_89
       (.CI(term1_reg_518_reg_i_128_n_0),
        .CO({term1_reg_518_reg_i_89_n_0,term1_reg_518_reg_i_89_n_1,term1_reg_518_reg_i_89_n_2,term1_reg_518_reg_i_89_n_3}),
        .CYINIT(1'b0),
        .DI(sum_q_reg[7:4]),
        .O(NLW_term1_reg_518_reg_i_89_O_UNCONNECTED[3:0]),
        .S({term1_reg_518_reg_i_129_n_0,term1_reg_518_reg_i_130_n_0,term1_reg_518_reg_i_131_n_0,term1_reg_518_reg_i_132_n_0}));
  LUT3 #(
    .INIT(8'hDC)) 
    term1_reg_518_reg_i_9
       (.I0(icmp_ln23_1_fu_321_p2),
        .I1(term1_reg_518_reg_i_41_n_0),
        .I2(term1_reg_518_reg_i_43_n_6),
        .O(B[10]));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_90
       (.I0(sum_q_reg[11]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[27] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[27] ),
        .O(term1_reg_518_reg_i_90_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_91
       (.I0(sum_q_reg[10]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[26] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[26] ),
        .O(term1_reg_518_reg_i_91_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_92
       (.I0(sum_q_reg[9]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[25] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[25] ),
        .O(term1_reg_518_reg_i_92_n_0));
  LUT4 #(
    .INIT(16'h569A)) 
    term1_reg_518_reg_i_93
       (.I0(sum_q_reg[8]),
        .I1(B_V_data_1_sel_rd_reg_n_0),
        .I2(\B_V_data_1_payload_A_reg_n_0_[24] ),
        .I3(\B_V_data_1_payload_B_reg_n_0_[24] ),
        .O(term1_reg_518_reg_i_93_n_0));
  LUT2 #(
    .INIT(4'h7)) 
    term1_reg_518_reg_i_94
       (.I0(tmp_fu_251_p4[3]),
        .I1(tmp_fu_251_p4[4]),
        .O(term1_reg_518_reg_i_94_n_0));
  LUT2 #(
    .INIT(4'h7)) 
    term1_reg_518_reg_i_95
       (.I0(tmp_fu_251_p4[1]),
        .I1(tmp_fu_251_p4[2]),
        .O(term1_reg_518_reg_i_95_n_0));
  LUT1 #(
    .INIT(2'h1)) 
    term1_reg_518_reg_i_96
       (.I0(tmp_fu_251_p4[0]),
        .O(term1_reg_518_reg_i_96_n_0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 term1_reg_518_reg_i_97
       (.CI(term1_reg_518_reg_i_49_n_0),
        .CO({NLW_term1_reg_518_reg_i_97_CO_UNCONNECTED[3],term1_reg_518_reg_i_97_n_1,term1_reg_518_reg_i_97_n_2,term1_reg_518_reg_i_97_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,sum_i_reg[29:27]}),
        .O(tmp_fu_251_p4[5:2]),
        .S(term1_reg_518_reg_i_99_0));
  LUT2 #(
    .INIT(4'h8)) 
    term1_reg_518_reg_i_98
       (.I0(tmp_fu_251_p4[3]),
        .I1(tmp_fu_251_p4[4]),
        .O(term1_reg_518_reg_i_98_n_0));
  LUT2 #(
    .INIT(4'h8)) 
    term1_reg_518_reg_i_99
       (.I0(tmp_fu_251_p4[1]),
        .I1(tmp_fu_251_p4[2]),
        .O(term1_reg_518_reg_i_99_n_0));
endmodule

(* ORIG_REF_NAME = "fsk_discriminator_regslice_both" *) 
module system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1
   (in_stream_TLAST_int_regslice,
    ap_rst_n_inv,
    ap_clk,
    ap_rst_n,
    \B_V_data_1_state_reg[1]_0 ,
    in_stream_TVALID,
    in_stream_TLAST);
  output in_stream_TLAST_int_regslice;
  input ap_rst_n_inv;
  input ap_clk;
  input ap_rst_n;
  input \B_V_data_1_state_reg[1]_0 ;
  input in_stream_TVALID;
  input [0:0]in_stream_TLAST;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__0_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__1_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__0_n_0;
  wire \B_V_data_1_state[0]_i_1__0_n_0 ;
  wire \B_V_data_1_state[1]_i_1__0_n_0 ;
  wire \B_V_data_1_state_reg[1]_0 ;
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
    \B_V_data_1_payload_A[0]_i_1__0 
       (.I0(in_stream_TLAST),
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
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hB4)) 
    B_V_data_1_sel_rd_i_1__1
       (.I0(\B_V_data_1_state_reg[1]_0 ),
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
    B_V_data_1_sel_wr_i_1__0
       (.I0(in_stream_TVALID),
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
    .INIT(32'hA8AAA000)) 
    \B_V_data_1_state[0]_i_1__0 
       (.I0(ap_rst_n),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(in_stream_TVALID),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(\B_V_data_1_state_reg_n_0_[0] ),
        .O(\B_V_data_1_state[0]_i_1__0_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT4 #(
    .INIT(16'h77F7)) 
    \B_V_data_1_state[1]_i_1__0 
       (.I0(\B_V_data_1_state_reg[1]_0 ),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(in_stream_TVALID),
        .O(\B_V_data_1_state[1]_i_1__0_n_0 ));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__0_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[1]_i_1__0_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2_i_1 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(in_stream_TLAST_int_regslice));
endmodule

(* ORIG_REF_NAME = "fsk_discriminator_regslice_both" *) 
module system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1_0
   (out_stream_TLAST,
    ap_rst_n_inv,
    ap_clk,
    ap_rst_n,
    out_stream_TREADY,
    out_stream_TVALID_int_regslice,
    pkt_last_V_reg_489_pp0_iter2_reg);
  output [0:0]out_stream_TLAST;
  input ap_rst_n_inv;
  input ap_clk;
  input ap_rst_n;
  input out_stream_TREADY;
  input out_stream_TVALID_int_regslice;
  input pkt_last_V_reg_489_pp0_iter2_reg;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__1_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1__0_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel_rd_i_1__2_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__2_n_0;
  wire \B_V_data_1_state[0]_i_1__2_n_0 ;
  wire \B_V_data_1_state[1]_i_1__2_n_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire out_stream_TVALID_int_regslice;
  wire pkt_last_V_reg_489_pp0_iter2_reg;

  LUT5 #(
    .INIT(32'hFFAE00A2)) 
    \B_V_data_1_payload_A[0]_i_1__1 
       (.I0(pkt_last_V_reg_489_pp0_iter2_reg),
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
       (.I0(pkt_last_V_reg_489_pp0_iter2_reg),
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
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__2
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
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
    .INIT(32'hA2AAA000)) 
    \B_V_data_1_state[0]_i_1__2 
       (.I0(ap_rst_n),
        .I1(out_stream_TREADY),
        .I2(out_stream_TVALID_int_regslice),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(\B_V_data_1_state_reg_n_0_[0] ),
        .O(\B_V_data_1_state[0]_i_1__2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT4 #(
    .INIT(16'hBBFB)) 
    \B_V_data_1_state[1]_i_1__2 
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(out_stream_TVALID_int_regslice),
        .O(\B_V_data_1_state[1]_i_1__2_n_0 ));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__2_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[0] ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[1]_i_1__2_n_0 ),
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

(* ORIG_REF_NAME = "fsk_discriminator_regslice_both" *) 
module system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized2
   (\B_V_data_1_state_reg[0]_0 ,
    i_old0,
    \B_V_data_1_state_reg[0]_1 ,
    out_stream_TVALID_int_regslice,
    ap_block_pp0_stage0_11001,
    \icmp_ln52_reg_494_reg[0] ,
    \B_V_data_1_state_reg[0]_2 ,
    out_stream_TDATA,
    ap_rst_n_inv,
    ap_clk,
    ap_enable_reg_pp0_iter1,
    icmp_ln52_reg_494,
    ap_rst_n,
    out_stream_TREADY,
    ap_enable_reg_pp0_iter3,
    icmp_ln52_reg_494_pp0_iter2_reg,
    in_stream_TVALID_int_regslice,
    icmp_ln52_reg_494_pp0_iter3_reg,
    ap_enable_reg_pp0_iter4,
    \B_V_data_1_payload_A_reg[15]_0 ,
    p_reg_reg,
    p_reg_reg_0,
    p_reg_reg_1,
    p_reg_reg_2,
    \B_V_data_1_payload_A_reg[14]_0 ,
    \B_V_data_1_payload_A_reg[13]_0 ,
    \B_V_data_1_payload_A_reg[12]_0 ,
    \B_V_data_1_payload_A_reg[11]_0 ,
    \B_V_data_1_payload_A_reg[10]_0 ,
    \B_V_data_1_payload_A_reg[9]_0 ,
    \B_V_data_1_payload_A_reg[8]_0 ,
    \B_V_data_1_payload_A_reg[7]_0 ,
    \B_V_data_1_payload_A_reg[6]_0 ,
    \B_V_data_1_payload_A_reg[5]_0 ,
    \B_V_data_1_payload_A_reg[4]_0 ,
    \B_V_data_1_payload_A_reg[3]_0 ,
    \B_V_data_1_payload_A_reg[2]_0 ,
    \B_V_data_1_payload_A_reg[1]_0 ,
    \B_V_data_1_payload_A_reg[0]_0 ,
    P,
    CO);
  output \B_V_data_1_state_reg[0]_0 ;
  output i_old0;
  output \B_V_data_1_state_reg[0]_1 ;
  output out_stream_TVALID_int_regslice;
  output ap_block_pp0_stage0_11001;
  output \icmp_ln52_reg_494_reg[0] ;
  output \B_V_data_1_state_reg[0]_2 ;
  output [15:0]out_stream_TDATA;
  input ap_rst_n_inv;
  input ap_clk;
  input ap_enable_reg_pp0_iter1;
  input icmp_ln52_reg_494;
  input ap_rst_n;
  input out_stream_TREADY;
  input ap_enable_reg_pp0_iter3;
  input icmp_ln52_reg_494_pp0_iter2_reg;
  input in_stream_TVALID_int_regslice;
  input icmp_ln52_reg_494_pp0_iter3_reg;
  input ap_enable_reg_pp0_iter4;
  input \B_V_data_1_payload_A_reg[15]_0 ;
  input p_reg_reg;
  input p_reg_reg_0;
  input p_reg_reg_1;
  input p_reg_reg_2;
  input \B_V_data_1_payload_A_reg[14]_0 ;
  input \B_V_data_1_payload_A_reg[13]_0 ;
  input \B_V_data_1_payload_A_reg[12]_0 ;
  input \B_V_data_1_payload_A_reg[11]_0 ;
  input \B_V_data_1_payload_A_reg[10]_0 ;
  input \B_V_data_1_payload_A_reg[9]_0 ;
  input \B_V_data_1_payload_A_reg[8]_0 ;
  input \B_V_data_1_payload_A_reg[7]_0 ;
  input \B_V_data_1_payload_A_reg[6]_0 ;
  input \B_V_data_1_payload_A_reg[5]_0 ;
  input \B_V_data_1_payload_A_reg[4]_0 ;
  input \B_V_data_1_payload_A_reg[3]_0 ;
  input \B_V_data_1_payload_A_reg[2]_0 ;
  input \B_V_data_1_payload_A_reg[1]_0 ;
  input \B_V_data_1_payload_A_reg[0]_0 ;
  input [1:0]P;
  input [0:0]CO;

  wire B_V_data_1_load_B;
  wire \B_V_data_1_payload_A[14]_i_1_n_0 ;
  wire \B_V_data_1_payload_A[14]_i_2_n_0 ;
  wire \B_V_data_1_payload_A[15]_i_1_n_0 ;
  wire \B_V_data_1_payload_A_reg[0]_0 ;
  wire \B_V_data_1_payload_A_reg[10]_0 ;
  wire \B_V_data_1_payload_A_reg[11]_0 ;
  wire \B_V_data_1_payload_A_reg[12]_0 ;
  wire \B_V_data_1_payload_A_reg[13]_0 ;
  wire \B_V_data_1_payload_A_reg[14]_0 ;
  wire \B_V_data_1_payload_A_reg[15]_0 ;
  wire \B_V_data_1_payload_A_reg[1]_0 ;
  wire \B_V_data_1_payload_A_reg[2]_0 ;
  wire \B_V_data_1_payload_A_reg[3]_0 ;
  wire \B_V_data_1_payload_A_reg[4]_0 ;
  wire \B_V_data_1_payload_A_reg[5]_0 ;
  wire \B_V_data_1_payload_A_reg[6]_0 ;
  wire \B_V_data_1_payload_A_reg[7]_0 ;
  wire \B_V_data_1_payload_A_reg[8]_0 ;
  wire \B_V_data_1_payload_A_reg[9]_0 ;
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
  wire \B_V_data_1_payload_B[14]_i_1_n_0 ;
  wire \B_V_data_1_payload_B[15]_i_1_n_0 ;
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
  wire B_V_data_1_sel_rd_i_1_n_0;
  wire B_V_data_1_sel_rd_reg_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__1_n_0;
  wire \B_V_data_1_state[0]_i_1__1_n_0 ;
  wire \B_V_data_1_state[1]_i_1__1_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg[0]_1 ;
  wire \B_V_data_1_state_reg[0]_2 ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire [0:0]CO;
  wire [1:0]P;
  wire ap_block_pp0_stage0_11001;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter141_out;
  wire ap_enable_reg_pp0_iter3;
  wire ap_enable_reg_pp0_iter4;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire i_old0;
  wire icmp_ln52_reg_494;
  wire icmp_ln52_reg_494_pp0_iter2_reg;
  wire icmp_ln52_reg_494_pp0_iter3_reg;
  wire \icmp_ln52_reg_494_reg[0] ;
  wire in_stream_TVALID_int_regslice;
  wire [15:0]out_stream_TDATA;
  wire out_stream_TREADY;
  wire out_stream_TVALID_int_regslice;
  wire p_reg_reg;
  wire p_reg_reg_0;
  wire p_reg_reg_1;
  wire p_reg_reg_2;

  LUT4 #(
    .INIT(16'h0045)) 
    \B_V_data_1_payload_A[14]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(\B_V_data_1_payload_A_reg[15]_0 ),
        .O(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  LUT3 #(
    .INIT(8'h0D)) 
    \B_V_data_1_payload_A[14]_i_2 
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_wr),
        .O(\B_V_data_1_payload_A[14]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hF888FFFFF8880000)) 
    \B_V_data_1_payload_A[15]_i_1 
       (.I0(P[1]),
        .I1(P[0]),
        .I2(CO),
        .I3(\B_V_data_1_payload_A_reg[15]_0 ),
        .I4(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .I5(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .O(\B_V_data_1_payload_A[15]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[0]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[10] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[10]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[11] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[11]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[12] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[12]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[13] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[13]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[14] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[14]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[15] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[15]_i_1_n_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .R(1'b0));
  FDSE \B_V_data_1_payload_A_reg[1] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[1]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[2] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[2]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[3] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[3]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[4] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[4]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[5] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[5]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[6] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[6]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[7] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[7]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[8] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[8]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_A_reg[9] 
       (.C(ap_clk),
        .CE(\B_V_data_1_payload_A[14]_i_2_n_0 ),
        .D(\B_V_data_1_payload_A_reg[9]_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .S(\B_V_data_1_payload_A[14]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'h00B0)) 
    \B_V_data_1_payload_B[14]_i_1 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(B_V_data_1_sel_wr),
        .I3(\B_V_data_1_payload_A_reg[15]_0 ),
        .O(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  LUT3 #(
    .INIT(8'hA2)) 
    \B_V_data_1_payload_B[14]_i_2 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_load_B));
  LUT6 #(
    .INIT(64'hF888FFFFF8880000)) 
    \B_V_data_1_payload_B[15]_i_1 
       (.I0(P[1]),
        .I1(P[0]),
        .I2(CO),
        .I3(\B_V_data_1_payload_A_reg[15]_0 ),
        .I4(B_V_data_1_load_B),
        .I5(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .O(\B_V_data_1_payload_B[15]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[0]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[10] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[10]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[11] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[11]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[12] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[12]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[13] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[13]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[14] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[14]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[15] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[15]_i_1_n_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .R(1'b0));
  FDSE \B_V_data_1_payload_B_reg[1] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[1]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[2] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[2]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[3] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[3]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[4] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[4]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[5] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[5]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[6] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[6]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[7] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[7]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[8] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[8]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  FDSE \B_V_data_1_payload_B_reg[9] 
       (.C(ap_clk),
        .CE(B_V_data_1_load_B),
        .D(\B_V_data_1_payload_A_reg[9]_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .S(\B_V_data_1_payload_B[14]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(B_V_data_1_sel_rd_i_1_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1_n_0),
        .Q(B_V_data_1_sel_rd_reg_n_0),
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
    .INIT(32'hA2AAA000)) 
    \B_V_data_1_state[0]_i_1__1 
       (.I0(ap_rst_n),
        .I1(out_stream_TREADY),
        .I2(out_stream_TVALID_int_regslice),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(\B_V_data_1_state_reg[0]_0 ),
        .O(\B_V_data_1_state[0]_i_1__1_n_0 ));
  LUT3 #(
    .INIT(8'h20)) 
    \B_V_data_1_state[0]_i_2 
       (.I0(ap_enable_reg_pp0_iter3),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .I2(icmp_ln52_reg_494_pp0_iter2_reg),
        .O(out_stream_TVALID_int_regslice));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT4 #(
    .INIT(16'hBBFB)) 
    \B_V_data_1_state[1]_i_1__1 
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(out_stream_TVALID_int_regslice),
        .O(\B_V_data_1_state[1]_i_1__1_n_0 ));
  LUT5 #(
    .INIT(32'hBBFBBBBB)) 
    \B_V_data_1_state[1]_i_3 
       (.I0(ap_enable_reg_pp0_iter141_out),
        .I1(in_stream_TVALID_int_regslice),
        .I2(ap_enable_reg_pp0_iter3),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(icmp_ln52_reg_494_pp0_iter2_reg),
        .O(\B_V_data_1_state_reg[0]_1 ));
  LUT5 #(
    .INIT(32'h3F220000)) 
    \B_V_data_1_state[1]_i_4 
       (.I0(icmp_ln52_reg_494_pp0_iter3_reg),
        .I1(\B_V_data_1_state_reg_n_0_[1] ),
        .I2(out_stream_TREADY),
        .I3(\B_V_data_1_state_reg[0]_0 ),
        .I4(ap_enable_reg_pp0_iter4),
        .O(ap_enable_reg_pp0_iter141_out));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1__1_n_0 ),
        .Q(\B_V_data_1_state_reg[0]_0 ),
        .R(1'b0));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[1]_i_1__1_n_0 ),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[0]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[0]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[10]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[10]));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[11]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[11]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[12]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[12]));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[13]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[13]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[14]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[14]));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[15]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[15]));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[1]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[1]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[2]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[2]));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[3]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[3]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[4]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[4]));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[5]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[5]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[6]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[6]));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[7]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[7]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[8]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[8]));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    \out_stream_TDATA[9]_INST_0 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA[9]));
  LUT1 #(
    .INIT(2'h1)) 
    p_reg_reg_i_1
       (.I0(\B_V_data_1_state_reg[0]_1 ),
        .O(ap_block_pp0_stage0_11001));
  LUT5 #(
    .INIT(32'h00000080)) 
    term1_reg_518_reg_i_1
       (.I0(p_reg_reg),
        .I1(p_reg_reg_0),
        .I2(p_reg_reg_1),
        .I3(p_reg_reg_2),
        .I4(\B_V_data_1_state_reg[0]_1 ),
        .O(\B_V_data_1_state_reg[0]_2 ));
  LUT3 #(
    .INIT(8'h20)) 
    term1_reg_518_reg_i_2
       (.I0(ap_enable_reg_pp0_iter1),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .I2(icmp_ln52_reg_494),
        .O(i_old0));
  LUT2 #(
    .INIT(4'h2)) 
    term1_reg_518_reg_i_3
       (.I0(icmp_ln52_reg_494),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .O(\icmp_ln52_reg_494_reg[0] ));
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
