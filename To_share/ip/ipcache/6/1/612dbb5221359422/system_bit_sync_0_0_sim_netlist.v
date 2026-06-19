// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 12:24:47 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_bit_sync_0_0/system_bit_sync_0_0_sim_netlist.v
// Design      : system_bit_sync_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "system_bit_sync_0_0,bit_sync,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "HLS" *) 
(* X_CORE_INFO = "bit_sync,Vivado 2023.1" *) (* hls_module = "yes" *) 
(* NotValidForBitStream *)
module system_bit_sync_0_0
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
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TDATA" *) input [15:0]in_stream_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TLAST" *) input [0:0]in_stream_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TKEEP" *) input [1:0]in_stream_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 in_stream TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME in_stream, TDATA_NUM_BYTES 2, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input [1:0]in_stream_TSTRB;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TVALID" *) output out_stream_TVALID;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TREADY" *) input out_stream_TREADY;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TDATA" *) output [7:0]out_stream_TDATA;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TLAST" *) output [0:0]out_stream_TLAST;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TKEEP" *) output [0:0]out_stream_TKEEP;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 out_stream TSTRB" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME out_stream, TDATA_NUM_BYTES 1, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) output [0:0]out_stream_TSTRB;

  wire \<const0> ;
  wire \<const1> ;
  wire ap_clk;
  wire ap_rst_n;
  wire [15:0]in_stream_TDATA;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TREADY;
  wire in_stream_TVALID;
  wire [0:0]\^out_stream_TDATA ;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire out_stream_TVALID;
  wire [7:1]NLW_inst_out_stream_TDATA_UNCONNECTED;
  wire [0:0]NLW_inst_out_stream_TKEEP_UNCONNECTED;
  wire [0:0]NLW_inst_out_stream_TSTRB_UNCONNECTED;

  assign out_stream_TDATA[7] = \<const0> ;
  assign out_stream_TDATA[6] = \<const0> ;
  assign out_stream_TDATA[5] = \<const0> ;
  assign out_stream_TDATA[4] = \<const0> ;
  assign out_stream_TDATA[3] = \<const0> ;
  assign out_stream_TDATA[2] = \<const0> ;
  assign out_stream_TDATA[1] = \<const0> ;
  assign out_stream_TDATA[0] = \^out_stream_TDATA [0];
  assign out_stream_TKEEP[0] = \<const1> ;
  assign out_stream_TSTRB[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  VCC VCC
       (.P(\<const1> ));
  (* SDX_KERNEL = "true" *) 
  (* SDX_KERNEL_SYNTH_INST = "inst" *) 
  (* SDX_KERNEL_TYPE = "hls" *) 
  (* ap_ST_fsm_pp0_stage0 = "1'b1" *) 
  system_bit_sync_0_0_bit_sync inst
       (.ap_clk(ap_clk),
        .ap_rst_n(ap_rst_n),
        .in_stream_TDATA(in_stream_TDATA),
        .in_stream_TKEEP({1'b0,1'b0}),
        .in_stream_TLAST(in_stream_TLAST),
        .in_stream_TREADY(in_stream_TREADY),
        .in_stream_TSTRB({1'b0,1'b0}),
        .in_stream_TVALID(in_stream_TVALID),
        .out_stream_TDATA({NLW_inst_out_stream_TDATA_UNCONNECTED[7:1],\^out_stream_TDATA }),
        .out_stream_TKEEP(NLW_inst_out_stream_TKEEP_UNCONNECTED[0]),
        .out_stream_TLAST(out_stream_TLAST),
        .out_stream_TREADY(out_stream_TREADY),
        .out_stream_TSTRB(NLW_inst_out_stream_TSTRB_UNCONNECTED[0]),
        .out_stream_TVALID(out_stream_TVALID));
endmodule

(* ORIG_REF_NAME = "bit_sync" *) (* ap_ST_fsm_pp0_stage0 = "1'b1" *) (* hls_module = "yes" *) 
module system_bit_sync_0_0_bit_sync
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
  input [15:0]in_stream_TDATA;
  input in_stream_TVALID;
  output in_stream_TREADY;
  input [1:0]in_stream_TKEEP;
  input [1:0]in_stream_TSTRB;
  input [0:0]in_stream_TLAST;
  output [7:0]out_stream_TDATA;
  output out_stream_TVALID;
  input out_stream_TREADY;
  output [0:0]out_stream_TKEEP;
  output [0:0]out_stream_TSTRB;
  output [0:0]out_stream_TLAST;

  wire \<const0> ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel0;
  wire [31:1]add_ln60_fu_137_p2;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter2;
  wire ap_enable_reg_pp0_iter3;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire curr_sign_fu_104_p2;
  wire curr_sign_reg_177;
  wire curr_sign_reg_177_pp0_iter1_reg;
  wire icmp_ln45_reg_184_pp0_iter2_reg;
  wire \icmp_ln45_reg_184_reg_n_0_[0] ;
  wire [15:0]in_stream_TDATA;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TLAST_int_regslice;
  wire in_stream_TREADY;
  wire in_stream_TVALID;
  wire in_stream_TVALID_int_regslice;
  wire [0:0]\^out_stream_TDATA ;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire out_stream_TVALID;
  wire p_1_in;
  wire phase_counter;
  wire \phase_counter[0]_i_1_n_0 ;
  wire \phase_counter_reg_n_0_[0] ;
  wire \phase_counter_reg_n_0_[10] ;
  wire \phase_counter_reg_n_0_[11] ;
  wire \phase_counter_reg_n_0_[12] ;
  wire \phase_counter_reg_n_0_[13] ;
  wire \phase_counter_reg_n_0_[14] ;
  wire \phase_counter_reg_n_0_[15] ;
  wire \phase_counter_reg_n_0_[16] ;
  wire \phase_counter_reg_n_0_[17] ;
  wire \phase_counter_reg_n_0_[18] ;
  wire \phase_counter_reg_n_0_[19] ;
  wire \phase_counter_reg_n_0_[1] ;
  wire \phase_counter_reg_n_0_[20] ;
  wire \phase_counter_reg_n_0_[21] ;
  wire \phase_counter_reg_n_0_[22] ;
  wire \phase_counter_reg_n_0_[23] ;
  wire \phase_counter_reg_n_0_[24] ;
  wire \phase_counter_reg_n_0_[25] ;
  wire \phase_counter_reg_n_0_[26] ;
  wire \phase_counter_reg_n_0_[27] ;
  wire \phase_counter_reg_n_0_[28] ;
  wire \phase_counter_reg_n_0_[29] ;
  wire \phase_counter_reg_n_0_[2] ;
  wire \phase_counter_reg_n_0_[30] ;
  wire \phase_counter_reg_n_0_[31] ;
  wire \phase_counter_reg_n_0_[3] ;
  wire \phase_counter_reg_n_0_[4] ;
  wire \phase_counter_reg_n_0_[5] ;
  wire \phase_counter_reg_n_0_[6] ;
  wire \phase_counter_reg_n_0_[7] ;
  wire \phase_counter_reg_n_0_[8] ;
  wire \phase_counter_reg_n_0_[9] ;
  wire pkt_last_V_reg_172;
  wire pkt_last_V_reg_172_pp0_iter1_reg;
  wire prev_sign;
  wire regslice_both_out_stream_V_data_V_U_n_35;
  wire regslice_both_out_stream_V_data_V_U_n_37;
  wire regslice_both_out_stream_V_data_V_U_n_38;
  wire regslice_both_out_stream_V_data_V_U_n_39;
  wire regslice_both_out_stream_V_data_V_U_n_40;
  wire regslice_both_out_stream_V_data_V_U_n_41;
  wire regslice_both_out_stream_V_data_V_U_n_42;

  assign out_stream_TDATA[7] = \<const0> ;
  assign out_stream_TDATA[6] = \<const0> ;
  assign out_stream_TDATA[5] = \<const0> ;
  assign out_stream_TDATA[4] = \<const0> ;
  assign out_stream_TDATA[3] = \<const0> ;
  assign out_stream_TDATA[2] = \<const0> ;
  assign out_stream_TDATA[1] = \<const0> ;
  assign out_stream_TDATA[0] = \^out_stream_TDATA [0];
  assign out_stream_TKEEP[0] = \<const0> ;
  assign out_stream_TSTRB[0] = \<const0> ;
  GND GND
       (.G(\<const0> ));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter1_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_out_stream_V_data_V_U_n_38),
        .Q(ap_enable_reg_pp0_iter1),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter2_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_out_stream_V_data_V_U_n_40),
        .Q(ap_enable_reg_pp0_iter2),
        .R(ap_rst_n_inv));
  FDRE #(
    .INIT(1'b0)) 
    ap_enable_reg_pp0_iter3_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_out_stream_V_data_V_U_n_39),
        .Q(ap_enable_reg_pp0_iter3),
        .R(ap_rst_n_inv));
  FDRE \curr_sign_reg_177_pp0_iter1_reg_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(curr_sign_reg_177),
        .Q(curr_sign_reg_177_pp0_iter1_reg),
        .R(1'b0));
  FDRE \curr_sign_reg_177_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(curr_sign_fu_104_p2),
        .Q(curr_sign_reg_177),
        .R(1'b0));
  FDRE \icmp_ln45_reg_184_pp0_iter2_reg_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(\icmp_ln45_reg_184_reg_n_0_[0] ),
        .Q(icmp_ln45_reg_184_pp0_iter2_reg),
        .R(1'b0));
  FDRE \icmp_ln45_reg_184_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_out_stream_V_data_V_U_n_35),
        .Q(\icmp_ln45_reg_184_reg_n_0_[0] ),
        .R(1'b0));
  LUT4 #(
    .INIT(16'h1455)) 
    \phase_counter[0]_i_1 
       (.I0(p_1_in),
        .I1(prev_sign),
        .I2(curr_sign_reg_177),
        .I3(\phase_counter_reg_n_0_[0] ),
        .O(\phase_counter[0]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[0] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(\phase_counter[0]_i_1_n_0 ),
        .Q(\phase_counter_reg_n_0_[0] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[10] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[10]),
        .Q(\phase_counter_reg_n_0_[10] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[11] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[11]),
        .Q(\phase_counter_reg_n_0_[11] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[12] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[12]),
        .Q(\phase_counter_reg_n_0_[12] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[13] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[13]),
        .Q(\phase_counter_reg_n_0_[13] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[14] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[14]),
        .Q(\phase_counter_reg_n_0_[14] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[15] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[15]),
        .Q(\phase_counter_reg_n_0_[15] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[16] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[16]),
        .Q(\phase_counter_reg_n_0_[16] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[17] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[17]),
        .Q(\phase_counter_reg_n_0_[17] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[18] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[18]),
        .Q(\phase_counter_reg_n_0_[18] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[19] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[19]),
        .Q(\phase_counter_reg_n_0_[19] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[1] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[1]),
        .Q(\phase_counter_reg_n_0_[1] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[20] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[20]),
        .Q(\phase_counter_reg_n_0_[20] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[21] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[21]),
        .Q(\phase_counter_reg_n_0_[21] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[22] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[22]),
        .Q(\phase_counter_reg_n_0_[22] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[23] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[23]),
        .Q(\phase_counter_reg_n_0_[23] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[24] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[24]),
        .Q(\phase_counter_reg_n_0_[24] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[25] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[25]),
        .Q(\phase_counter_reg_n_0_[25] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[26] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[26]),
        .Q(\phase_counter_reg_n_0_[26] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[27] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[27]),
        .Q(\phase_counter_reg_n_0_[27] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[28] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[28]),
        .Q(\phase_counter_reg_n_0_[28] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[29] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[29]),
        .Q(\phase_counter_reg_n_0_[29] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[2] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[2]),
        .Q(\phase_counter_reg_n_0_[2] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[30] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[30]),
        .Q(\phase_counter_reg_n_0_[30] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[31] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[31]),
        .Q(\phase_counter_reg_n_0_[31] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[3] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[3]),
        .Q(\phase_counter_reg_n_0_[3] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[4] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[4]),
        .Q(\phase_counter_reg_n_0_[4] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[5] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[5]),
        .Q(\phase_counter_reg_n_0_[5] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[6] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[6]),
        .Q(\phase_counter_reg_n_0_[6] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[7] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[7]),
        .Q(\phase_counter_reg_n_0_[7] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[8] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[8]),
        .Q(\phase_counter_reg_n_0_[8] ),
        .R(phase_counter));
  FDRE #(
    .INIT(1'b0)) 
    \phase_counter_reg[9] 
       (.C(ap_clk),
        .CE(regslice_both_out_stream_V_data_V_U_n_37),
        .D(add_ln60_fu_137_p2[9]),
        .Q(\phase_counter_reg_n_0_[9] ),
        .R(phase_counter));
  FDRE \pkt_last_V_reg_172_pp0_iter1_reg_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(pkt_last_V_reg_172),
        .Q(pkt_last_V_reg_172_pp0_iter1_reg),
        .R(1'b0));
  FDRE \pkt_last_V_reg_172_reg[0] 
       (.C(ap_clk),
        .CE(B_V_data_1_sel0),
        .D(in_stream_TLAST_int_regslice),
        .Q(pkt_last_V_reg_172),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \prev_sign_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(regslice_both_out_stream_V_data_V_U_n_41),
        .Q(prev_sign),
        .R(1'b0));
  system_bit_sync_0_0_bit_sync_regslice_both regslice_both_in_stream_V_data_V_U
       (.B_V_data_1_sel(B_V_data_1_sel),
        .B_V_data_1_sel0(B_V_data_1_sel0),
        .B_V_data_1_sel_rd_reg_0(regslice_both_out_stream_V_data_V_U_n_42),
        .\B_V_data_1_state_reg[1]_0 (in_stream_TREADY),
        .ap_clk(ap_clk),
        .ap_rst_n_inv(ap_rst_n_inv),
        .curr_sign_fu_104_p2(curr_sign_fu_104_p2),
        .in_stream_TDATA(in_stream_TDATA),
        .in_stream_TVALID(in_stream_TVALID),
        .in_stream_TVALID_int_regslice(in_stream_TVALID_int_regslice));
  system_bit_sync_0_0_bit_sync_regslice_both__parameterized1 regslice_both_in_stream_V_last_V_U
       (.B_V_data_1_sel0(B_V_data_1_sel0),
        .ap_clk(ap_clk),
        .ap_rst_n_inv(ap_rst_n_inv),
        .in_stream_TLAST(in_stream_TLAST),
        .in_stream_TLAST_int_regslice(in_stream_TLAST_int_regslice),
        .in_stream_TVALID(in_stream_TVALID));
  system_bit_sync_0_0_bit_sync_regslice_both__parameterized2 regslice_both_out_stream_V_data_V_U
       (.B_V_data_1_sel(B_V_data_1_sel),
        .B_V_data_1_sel0(B_V_data_1_sel0),
        .B_V_data_1_sel_rd_reg_0(regslice_both_out_stream_V_data_V_U_n_42),
        .\B_V_data_1_state_reg[0]_0 (out_stream_TVALID),
        .\B_V_data_1_state_reg[0]_1 (\icmp_ln45_reg_184_reg_n_0_[0] ),
        .CO(p_1_in),
        .D(add_ln60_fu_137_p2),
        .E(regslice_both_out_stream_V_data_V_U_n_37),
        .Q({\phase_counter_reg_n_0_[31] ,\phase_counter_reg_n_0_[30] ,\phase_counter_reg_n_0_[29] ,\phase_counter_reg_n_0_[28] ,\phase_counter_reg_n_0_[27] ,\phase_counter_reg_n_0_[26] ,\phase_counter_reg_n_0_[25] ,\phase_counter_reg_n_0_[24] ,\phase_counter_reg_n_0_[23] ,\phase_counter_reg_n_0_[22] ,\phase_counter_reg_n_0_[21] ,\phase_counter_reg_n_0_[20] ,\phase_counter_reg_n_0_[19] ,\phase_counter_reg_n_0_[18] ,\phase_counter_reg_n_0_[17] ,\phase_counter_reg_n_0_[16] ,\phase_counter_reg_n_0_[15] ,\phase_counter_reg_n_0_[14] ,\phase_counter_reg_n_0_[13] ,\phase_counter_reg_n_0_[12] ,\phase_counter_reg_n_0_[11] ,\phase_counter_reg_n_0_[10] ,\phase_counter_reg_n_0_[9] ,\phase_counter_reg_n_0_[8] ,\phase_counter_reg_n_0_[7] ,\phase_counter_reg_n_0_[6] ,\phase_counter_reg_n_0_[5] ,\phase_counter_reg_n_0_[4] ,\phase_counter_reg_n_0_[3] ,\phase_counter_reg_n_0_[2] ,\phase_counter_reg_n_0_[1] ,\phase_counter_reg_n_0_[0] }),
        .SR(phase_counter),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter1(ap_enable_reg_pp0_iter1),
        .ap_enable_reg_pp0_iter1_reg(regslice_both_out_stream_V_data_V_U_n_38),
        .ap_enable_reg_pp0_iter1_reg_0(regslice_both_out_stream_V_data_V_U_n_40),
        .ap_enable_reg_pp0_iter2(ap_enable_reg_pp0_iter2),
        .ap_enable_reg_pp0_iter2_reg(regslice_both_out_stream_V_data_V_U_n_39),
        .ap_enable_reg_pp0_iter3(ap_enable_reg_pp0_iter3),
        .ap_rst_n_inv(ap_rst_n_inv),
        .curr_sign_reg_177(curr_sign_reg_177),
        .curr_sign_reg_177_pp0_iter1_reg(curr_sign_reg_177_pp0_iter1_reg),
        .\curr_sign_reg_177_reg[0] (regslice_both_out_stream_V_data_V_U_n_41),
        .icmp_ln45_reg_184_pp0_iter2_reg(icmp_ln45_reg_184_pp0_iter2_reg),
        .\icmp_ln45_reg_184_reg[0] (regslice_both_out_stream_V_data_V_U_n_35),
        .in_stream_TVALID_int_regslice(in_stream_TVALID_int_regslice),
        .out_stream_TDATA(\^out_stream_TDATA ),
        .out_stream_TREADY(out_stream_TREADY),
        .prev_sign(prev_sign));
  system_bit_sync_0_0_bit_sync_regslice_both__parameterized1_0 regslice_both_out_stream_V_last_V_U
       (.B_V_data_1_sel0(B_V_data_1_sel0),
        .\B_V_data_1_state_reg[0]_0 (\icmp_ln45_reg_184_reg_n_0_[0] ),
        .ap_clk(ap_clk),
        .ap_enable_reg_pp0_iter2(ap_enable_reg_pp0_iter2),
        .ap_rst_n(ap_rst_n),
        .ap_rst_n_inv(ap_rst_n_inv),
        .out_stream_TLAST(out_stream_TLAST),
        .out_stream_TREADY(out_stream_TREADY),
        .pkt_last_V_reg_172_pp0_iter1_reg(pkt_last_V_reg_172_pp0_iter1_reg));
endmodule

(* ORIG_REF_NAME = "bit_sync_regslice_both" *) 
module system_bit_sync_0_0_bit_sync_regslice_both
   (\B_V_data_1_state_reg[1]_0 ,
    in_stream_TVALID_int_regslice,
    B_V_data_1_sel,
    curr_sign_fu_104_p2,
    ap_rst_n_inv,
    ap_clk,
    B_V_data_1_sel_rd_reg_0,
    B_V_data_1_sel0,
    in_stream_TVALID,
    in_stream_TDATA);
  output \B_V_data_1_state_reg[1]_0 ;
  output in_stream_TVALID_int_regslice;
  output B_V_data_1_sel;
  output [0:0]curr_sign_fu_104_p2;
  input ap_rst_n_inv;
  input ap_clk;
  input B_V_data_1_sel_rd_reg_0;
  input B_V_data_1_sel0;
  input in_stream_TVALID;
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
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_reg_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__2_n_0 ;
  wire \B_V_data_1_state_reg[1]_0 ;
  wire ap_clk;
  wire ap_rst_n_inv;
  wire [0:0]curr_sign_fu_104_p2;
  wire \curr_sign_reg_177[0]_i_10_n_0 ;
  wire \curr_sign_reg_177[0]_i_11_n_0 ;
  wire \curr_sign_reg_177[0]_i_12_n_0 ;
  wire \curr_sign_reg_177[0]_i_13_n_0 ;
  wire \curr_sign_reg_177[0]_i_14_n_0 ;
  wire \curr_sign_reg_177[0]_i_15_n_0 ;
  wire \curr_sign_reg_177[0]_i_16_n_0 ;
  wire \curr_sign_reg_177[0]_i_17_n_0 ;
  wire \curr_sign_reg_177[0]_i_18_n_0 ;
  wire \curr_sign_reg_177[0]_i_19_n_0 ;
  wire \curr_sign_reg_177[0]_i_20_n_0 ;
  wire \curr_sign_reg_177[0]_i_5_n_0 ;
  wire \curr_sign_reg_177[0]_i_6_n_0 ;
  wire \curr_sign_reg_177[0]_i_7_n_0 ;
  wire \curr_sign_reg_177[0]_i_8_n_0 ;
  wire \curr_sign_reg_177[0]_i_9_n_0 ;
  wire \curr_sign_reg_177_reg[0]_i_2_n_1 ;
  wire \curr_sign_reg_177_reg[0]_i_2_n_2 ;
  wire \curr_sign_reg_177_reg[0]_i_2_n_3 ;
  wire \curr_sign_reg_177_reg[0]_i_4_n_0 ;
  wire \curr_sign_reg_177_reg[0]_i_4_n_1 ;
  wire \curr_sign_reg_177_reg[0]_i_4_n_2 ;
  wire \curr_sign_reg_177_reg[0]_i_4_n_3 ;
  wire [15:0]in_stream_TDATA;
  wire in_stream_TVALID;
  wire in_stream_TVALID_int_regslice;
  wire [3:0]\NLW_curr_sign_reg_177_reg[0]_i_2_O_UNCONNECTED ;
  wire [3:0]\NLW_curr_sign_reg_177_reg[0]_i_4_O_UNCONNECTED ;

  LUT3 #(
    .INIT(8'h45)) 
    \B_V_data_1_payload_A[15]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(in_stream_TVALID_int_regslice),
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
    .INIT(8'h8A)) 
    \B_V_data_1_payload_B[15]_i_1 
       (.I0(B_V_data_1_sel_wr),
        .I1(\B_V_data_1_state_reg[1]_0 ),
        .I2(in_stream_TVALID_int_regslice),
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
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_reg_0),
        .Q(B_V_data_1_sel),
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
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT4 #(
    .INIT(16'hD8F8)) 
    \B_V_data_1_state[0]_i_1__2 
       (.I0(\B_V_data_1_state_reg[1]_0 ),
        .I1(in_stream_TVALID),
        .I2(in_stream_TVALID_int_regslice),
        .I3(B_V_data_1_sel0),
        .O(\B_V_data_1_state[0]_i_1__2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
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
  LUT5 #(
    .INIT(32'h00053035)) 
    \curr_sign_reg_177[0]_i_10 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .O(\curr_sign_reg_177[0]_i_10_n_0 ));
  LUT5 #(
    .INIT(32'h00053035)) 
    \curr_sign_reg_177[0]_i_11 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .O(\curr_sign_reg_177[0]_i_11_n_0 ));
  LUT5 #(
    .INIT(32'h00053035)) 
    \curr_sign_reg_177[0]_i_12 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .O(\curr_sign_reg_177[0]_i_12_n_0 ));
  LUT5 #(
    .INIT(32'hFFFCAFAC)) 
    \curr_sign_reg_177[0]_i_13 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .O(\curr_sign_reg_177[0]_i_13_n_0 ));
  LUT5 #(
    .INIT(32'hFFFCAFAC)) 
    \curr_sign_reg_177[0]_i_14 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .O(\curr_sign_reg_177[0]_i_14_n_0 ));
  LUT5 #(
    .INIT(32'hFFFCAFAC)) 
    \curr_sign_reg_177[0]_i_15 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .O(\curr_sign_reg_177[0]_i_15_n_0 ));
  LUT5 #(
    .INIT(32'hFFFCAFAC)) 
    \curr_sign_reg_177[0]_i_16 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .O(\curr_sign_reg_177[0]_i_16_n_0 ));
  LUT5 #(
    .INIT(32'h00053035)) 
    \curr_sign_reg_177[0]_i_17 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[6] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[6] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[7] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[7] ),
        .O(\curr_sign_reg_177[0]_i_17_n_0 ));
  LUT5 #(
    .INIT(32'h00053035)) 
    \curr_sign_reg_177[0]_i_18 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[4] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[4] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[5] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[5] ),
        .O(\curr_sign_reg_177[0]_i_18_n_0 ));
  LUT5 #(
    .INIT(32'h00053035)) 
    \curr_sign_reg_177[0]_i_19 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[2] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[2] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[3] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[3] ),
        .O(\curr_sign_reg_177[0]_i_19_n_0 ));
  LUT5 #(
    .INIT(32'h00053035)) 
    \curr_sign_reg_177[0]_i_20 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[1] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[1] ),
        .O(\curr_sign_reg_177[0]_i_20_n_0 ));
  LUT5 #(
    .INIT(32'h000AC0CA)) 
    \curr_sign_reg_177[0]_i_5 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .O(\curr_sign_reg_177[0]_i_5_n_0 ));
  LUT5 #(
    .INIT(32'hFFFCAFAC)) 
    \curr_sign_reg_177[0]_i_6 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[12] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[12] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[13] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[13] ),
        .O(\curr_sign_reg_177[0]_i_6_n_0 ));
  LUT5 #(
    .INIT(32'hFFFCAFAC)) 
    \curr_sign_reg_177[0]_i_7 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[10] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[10] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[11] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[11] ),
        .O(\curr_sign_reg_177[0]_i_7_n_0 ));
  LUT5 #(
    .INIT(32'hFFFCAFAC)) 
    \curr_sign_reg_177[0]_i_8 
       (.I0(\B_V_data_1_payload_B_reg_n_0_[8] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[8] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[9] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[9] ),
        .O(\curr_sign_reg_177[0]_i_8_n_0 ));
  LUT5 #(
    .INIT(32'h00053035)) 
    \curr_sign_reg_177[0]_i_9 
       (.I0(\B_V_data_1_payload_A_reg_n_0_[14] ),
        .I1(\B_V_data_1_payload_B_reg_n_0_[14] ),
        .I2(B_V_data_1_sel),
        .I3(\B_V_data_1_payload_A_reg_n_0_[15] ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[15] ),
        .O(\curr_sign_reg_177[0]_i_9_n_0 ));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \curr_sign_reg_177_reg[0]_i_2 
       (.CI(\curr_sign_reg_177_reg[0]_i_4_n_0 ),
        .CO({curr_sign_fu_104_p2,\curr_sign_reg_177_reg[0]_i_2_n_1 ,\curr_sign_reg_177_reg[0]_i_2_n_2 ,\curr_sign_reg_177_reg[0]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({\curr_sign_reg_177[0]_i_5_n_0 ,\curr_sign_reg_177[0]_i_6_n_0 ,\curr_sign_reg_177[0]_i_7_n_0 ,\curr_sign_reg_177[0]_i_8_n_0 }),
        .O(\NLW_curr_sign_reg_177_reg[0]_i_2_O_UNCONNECTED [3:0]),
        .S({\curr_sign_reg_177[0]_i_9_n_0 ,\curr_sign_reg_177[0]_i_10_n_0 ,\curr_sign_reg_177[0]_i_11_n_0 ,\curr_sign_reg_177[0]_i_12_n_0 }));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \curr_sign_reg_177_reg[0]_i_4 
       (.CI(1'b0),
        .CO({\curr_sign_reg_177_reg[0]_i_4_n_0 ,\curr_sign_reg_177_reg[0]_i_4_n_1 ,\curr_sign_reg_177_reg[0]_i_4_n_2 ,\curr_sign_reg_177_reg[0]_i_4_n_3 }),
        .CYINIT(1'b0),
        .DI({\curr_sign_reg_177[0]_i_13_n_0 ,\curr_sign_reg_177[0]_i_14_n_0 ,\curr_sign_reg_177[0]_i_15_n_0 ,\curr_sign_reg_177[0]_i_16_n_0 }),
        .O(\NLW_curr_sign_reg_177_reg[0]_i_4_O_UNCONNECTED [3:0]),
        .S({\curr_sign_reg_177[0]_i_17_n_0 ,\curr_sign_reg_177[0]_i_18_n_0 ,\curr_sign_reg_177[0]_i_19_n_0 ,\curr_sign_reg_177[0]_i_20_n_0 }));
endmodule

(* ORIG_REF_NAME = "bit_sync_regslice_both" *) 
module system_bit_sync_0_0_bit_sync_regslice_both__parameterized1
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
  wire \B_V_data_1_payload_A[0]_i_1_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_i_1_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__2_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__1_n_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_rst_n_inv;
  wire [0:0]in_stream_TLAST;
  wire in_stream_TLAST_int_regslice;
  wire in_stream_TVALID;

  LUT5 #(
    .INIT(32'hEFEE2022)) 
    \B_V_data_1_payload_A[0]_i_1 
       (.I0(in_stream_TLAST),
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
       (.I0(in_stream_TLAST),
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
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
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
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'hD8F8)) 
    \B_V_data_1_state[0]_i_1__1 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(in_stream_TVALID),
        .I2(\B_V_data_1_state_reg_n_0_[0] ),
        .I3(B_V_data_1_sel0),
        .O(\B_V_data_1_state[0]_i_1__1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'hDFDD)) 
    \B_V_data_1_state[1]_i_1__1 
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(B_V_data_1_sel0),
        .I2(in_stream_TVALID),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .O(B_V_data_1_state));
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
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \pkt_last_V_reg_172[0]_i_1 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(in_stream_TLAST_int_regslice));
endmodule

(* ORIG_REF_NAME = "bit_sync_regslice_both" *) 
module system_bit_sync_0_0_bit_sync_regslice_both__parameterized1_0
   (ap_rst_n_inv,
    out_stream_TLAST,
    ap_clk,
    out_stream_TREADY,
    B_V_data_1_sel0,
    ap_enable_reg_pp0_iter2,
    \B_V_data_1_state_reg[0]_0 ,
    ap_rst_n,
    pkt_last_V_reg_172_pp0_iter1_reg);
  output ap_rst_n_inv;
  output [0:0]out_stream_TLAST;
  input ap_clk;
  input out_stream_TREADY;
  input B_V_data_1_sel0;
  input ap_enable_reg_pp0_iter2;
  input \B_V_data_1_state_reg[0]_0 ;
  input ap_rst_n;
  input pkt_last_V_reg_172_pp0_iter1_reg;

  wire B_V_data_1_payload_A;
  wire \B_V_data_1_payload_A[0]_i_1__1_n_0 ;
  wire B_V_data_1_payload_B;
  wire \B_V_data_1_payload_B[0]_i_1__1_n_0 ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_i_1__2_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1__0_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg_n_0_[0] ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter2;
  wire ap_rst_n;
  wire ap_rst_n_inv;
  wire [0:0]out_stream_TLAST;
  wire out_stream_TREADY;
  wire pkt_last_V_reg_172_pp0_iter1_reg;

  LUT5 #(
    .INIT(32'hEFEE2022)) 
    \B_V_data_1_payload_A[0]_i_1__1 
       (.I0(pkt_last_V_reg_172_pp0_iter1_reg),
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
       (.I0(pkt_last_V_reg_172_pp0_iter1_reg),
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
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__2
       (.I0(\B_V_data_1_state_reg_n_0_[0] ),
        .I1(out_stream_TREADY),
        .I2(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_i_1__2_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__2_n_0),
        .Q(B_V_data_1_sel),
        .R(ap_rst_n_inv));
  LUT5 #(
    .INIT(32'h7FFF8000)) 
    B_V_data_1_sel_wr_i_1
       (.I0(B_V_data_1_sel0),
        .I1(ap_enable_reg_pp0_iter2),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'h8080FF80FF00FF00)) 
    \B_V_data_1_state[0]_i_1__0 
       (.I0(B_V_data_1_sel0),
        .I1(ap_enable_reg_pp0_iter2),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(\B_V_data_1_state_reg_n_0_[0] ),
        .I4(out_stream_TREADY),
        .I5(\B_V_data_1_state_reg_n_0_[1] ),
        .O(\B_V_data_1_state[0]_i_1__0_n_0 ));
  LUT6 #(
    .INIT(64'hBBFBFBFBFBFBFBFB)) 
    \B_V_data_1_state[1]_i_1__0 
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg_n_0_[0] ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(B_V_data_1_sel0),
        .I4(ap_enable_reg_pp0_iter2),
        .I5(\B_V_data_1_state_reg[0]_0 ),
        .O(B_V_data_1_state));
  LUT1 #(
    .INIT(2'h1)) 
    \B_V_data_1_state[1]_i_1__2 
       (.I0(ap_rst_n),
        .O(ap_rst_n_inv));
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
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \out_stream_TLAST[0]_INST_0 
       (.I0(B_V_data_1_payload_B),
        .I1(B_V_data_1_sel),
        .I2(B_V_data_1_payload_A),
        .O(out_stream_TLAST));
endmodule

(* ORIG_REF_NAME = "bit_sync_regslice_both" *) 
module system_bit_sync_0_0_bit_sync_regslice_both__parameterized2
   (out_stream_TDATA,
    \B_V_data_1_state_reg[0]_0 ,
    D,
    CO,
    B_V_data_1_sel0,
    \icmp_ln45_reg_184_reg[0] ,
    SR,
    E,
    ap_enable_reg_pp0_iter1_reg,
    ap_enable_reg_pp0_iter2_reg,
    ap_enable_reg_pp0_iter1_reg_0,
    \curr_sign_reg_177_reg[0] ,
    B_V_data_1_sel_rd_reg_0,
    ap_rst_n_inv,
    ap_clk,
    out_stream_TREADY,
    ap_enable_reg_pp0_iter2,
    \B_V_data_1_state_reg[0]_1 ,
    in_stream_TVALID_int_regslice,
    icmp_ln45_reg_184_pp0_iter2_reg,
    ap_enable_reg_pp0_iter3,
    Q,
    curr_sign_reg_177,
    prev_sign,
    curr_sign_reg_177_pp0_iter1_reg,
    ap_enable_reg_pp0_iter1,
    B_V_data_1_sel);
  output [0:0]out_stream_TDATA;
  output \B_V_data_1_state_reg[0]_0 ;
  output [30:0]D;
  output [0:0]CO;
  output B_V_data_1_sel0;
  output \icmp_ln45_reg_184_reg[0] ;
  output [0:0]SR;
  output [0:0]E;
  output ap_enable_reg_pp0_iter1_reg;
  output ap_enable_reg_pp0_iter2_reg;
  output ap_enable_reg_pp0_iter1_reg_0;
  output \curr_sign_reg_177_reg[0] ;
  output B_V_data_1_sel_rd_reg_0;
  input ap_rst_n_inv;
  input ap_clk;
  input out_stream_TREADY;
  input ap_enable_reg_pp0_iter2;
  input \B_V_data_1_state_reg[0]_1 ;
  input in_stream_TVALID_int_regslice;
  input icmp_ln45_reg_184_pp0_iter2_reg;
  input ap_enable_reg_pp0_iter3;
  input [31:0]Q;
  input curr_sign_reg_177;
  input prev_sign;
  input curr_sign_reg_177_pp0_iter1_reg;
  input ap_enable_reg_pp0_iter1;
  input B_V_data_1_sel;

  wire \B_V_data_1_payload_A[0]_i_1__0_n_0 ;
  wire \B_V_data_1_payload_A_reg_n_0_[0] ;
  wire \B_V_data_1_payload_B[0]_i_1__0_n_0 ;
  wire \B_V_data_1_payload_B_reg_n_0_[0] ;
  wire B_V_data_1_sel;
  wire B_V_data_1_sel0;
  wire B_V_data_1_sel_rd_i_1__1_n_0;
  wire B_V_data_1_sel_rd_reg_0;
  wire B_V_data_1_sel_rd_reg_n_0;
  wire B_V_data_1_sel_wr;
  wire B_V_data_1_sel_wr_i_1__0_n_0;
  wire [1:1]B_V_data_1_state;
  wire \B_V_data_1_state[0]_i_1_n_0 ;
  wire \B_V_data_1_state_reg[0]_0 ;
  wire \B_V_data_1_state_reg[0]_1 ;
  wire \B_V_data_1_state_reg_n_0_[1] ;
  wire [0:0]CO;
  wire [30:0]D;
  wire [0:0]E;
  wire [31:0]Q;
  wire [0:0]SR;
  wire ap_clk;
  wire ap_enable_reg_pp0_iter1;
  wire ap_enable_reg_pp0_iter1_reg;
  wire ap_enable_reg_pp0_iter1_reg_0;
  wire ap_enable_reg_pp0_iter2;
  wire ap_enable_reg_pp0_iter2_reg;
  wire ap_enable_reg_pp0_iter3;
  wire ap_rst_n_inv;
  wire curr_sign_reg_177;
  wire \curr_sign_reg_177[0]_i_3_n_0 ;
  wire curr_sign_reg_177_pp0_iter1_reg;
  wire \curr_sign_reg_177_reg[0] ;
  wire \icmp_ln45_reg_184[0]_i_10_n_0 ;
  wire \icmp_ln45_reg_184[0]_i_2_n_0 ;
  wire \icmp_ln45_reg_184[0]_i_3_n_0 ;
  wire \icmp_ln45_reg_184[0]_i_4_n_0 ;
  wire \icmp_ln45_reg_184[0]_i_5_n_0 ;
  wire \icmp_ln45_reg_184[0]_i_6_n_0 ;
  wire \icmp_ln45_reg_184[0]_i_7_n_0 ;
  wire \icmp_ln45_reg_184[0]_i_8_n_0 ;
  wire \icmp_ln45_reg_184[0]_i_9_n_0 ;
  wire icmp_ln45_reg_184_pp0_iter2_reg;
  wire \icmp_ln45_reg_184_reg[0] ;
  wire in_stream_TVALID_int_regslice;
  wire [0:0]out_stream_TDATA;
  wire out_stream_TREADY;
  wire \phase_counter[31]_i_10_n_0 ;
  wire \phase_counter[31]_i_11_n_0 ;
  wire \phase_counter[31]_i_12_n_0 ;
  wire \phase_counter[31]_i_13_n_0 ;
  wire \phase_counter[31]_i_14_n_0 ;
  wire \phase_counter[31]_i_16_n_0 ;
  wire \phase_counter[31]_i_17_n_0 ;
  wire \phase_counter[31]_i_18_n_0 ;
  wire \phase_counter[31]_i_19_n_0 ;
  wire \phase_counter[31]_i_20_n_0 ;
  wire \phase_counter[31]_i_21_n_0 ;
  wire \phase_counter[31]_i_22_n_0 ;
  wire \phase_counter[31]_i_23_n_0 ;
  wire \phase_counter[31]_i_25_n_0 ;
  wire \phase_counter[31]_i_26_n_0 ;
  wire \phase_counter[31]_i_27_n_0 ;
  wire \phase_counter[31]_i_28_n_0 ;
  wire \phase_counter[31]_i_29_n_0 ;
  wire \phase_counter[31]_i_30_n_0 ;
  wire \phase_counter[31]_i_31_n_0 ;
  wire \phase_counter[31]_i_32_n_0 ;
  wire \phase_counter[31]_i_33_n_0 ;
  wire \phase_counter[31]_i_34_n_0 ;
  wire \phase_counter[31]_i_35_n_0 ;
  wire \phase_counter[31]_i_36_n_0 ;
  wire \phase_counter[31]_i_37_n_0 ;
  wire \phase_counter[31]_i_38_n_0 ;
  wire \phase_counter[31]_i_9_n_0 ;
  wire \phase_counter_reg[12]_i_1_n_0 ;
  wire \phase_counter_reg[12]_i_1_n_1 ;
  wire \phase_counter_reg[12]_i_1_n_2 ;
  wire \phase_counter_reg[12]_i_1_n_3 ;
  wire \phase_counter_reg[16]_i_1_n_0 ;
  wire \phase_counter_reg[16]_i_1_n_1 ;
  wire \phase_counter_reg[16]_i_1_n_2 ;
  wire \phase_counter_reg[16]_i_1_n_3 ;
  wire \phase_counter_reg[20]_i_1_n_0 ;
  wire \phase_counter_reg[20]_i_1_n_1 ;
  wire \phase_counter_reg[20]_i_1_n_2 ;
  wire \phase_counter_reg[20]_i_1_n_3 ;
  wire \phase_counter_reg[24]_i_1_n_0 ;
  wire \phase_counter_reg[24]_i_1_n_1 ;
  wire \phase_counter_reg[24]_i_1_n_2 ;
  wire \phase_counter_reg[24]_i_1_n_3 ;
  wire \phase_counter_reg[28]_i_1_n_0 ;
  wire \phase_counter_reg[28]_i_1_n_1 ;
  wire \phase_counter_reg[28]_i_1_n_2 ;
  wire \phase_counter_reg[28]_i_1_n_3 ;
  wire \phase_counter_reg[31]_i_15_n_0 ;
  wire \phase_counter_reg[31]_i_15_n_1 ;
  wire \phase_counter_reg[31]_i_15_n_2 ;
  wire \phase_counter_reg[31]_i_15_n_3 ;
  wire \phase_counter_reg[31]_i_24_n_0 ;
  wire \phase_counter_reg[31]_i_24_n_1 ;
  wire \phase_counter_reg[31]_i_24_n_2 ;
  wire \phase_counter_reg[31]_i_24_n_3 ;
  wire \phase_counter_reg[31]_i_3_n_2 ;
  wire \phase_counter_reg[31]_i_3_n_3 ;
  wire \phase_counter_reg[31]_i_4_n_2 ;
  wire \phase_counter_reg[31]_i_4_n_3 ;
  wire \phase_counter_reg[31]_i_8_n_0 ;
  wire \phase_counter_reg[31]_i_8_n_1 ;
  wire \phase_counter_reg[31]_i_8_n_2 ;
  wire \phase_counter_reg[31]_i_8_n_3 ;
  wire \phase_counter_reg[4]_i_1_n_0 ;
  wire \phase_counter_reg[4]_i_1_n_1 ;
  wire \phase_counter_reg[4]_i_1_n_2 ;
  wire \phase_counter_reg[4]_i_1_n_3 ;
  wire \phase_counter_reg[8]_i_1_n_0 ;
  wire \phase_counter_reg[8]_i_1_n_1 ;
  wire \phase_counter_reg[8]_i_1_n_2 ;
  wire \phase_counter_reg[8]_i_1_n_3 ;
  wire prev_sign;
  wire [31:0]select_ln40_fu_123_p3;
  wire [3:0]\NLW_phase_counter_reg[31]_i_15_O_UNCONNECTED ;
  wire [3:0]\NLW_phase_counter_reg[31]_i_24_O_UNCONNECTED ;
  wire [3:2]\NLW_phase_counter_reg[31]_i_3_CO_UNCONNECTED ;
  wire [3:3]\NLW_phase_counter_reg[31]_i_3_O_UNCONNECTED ;
  wire [3:3]\NLW_phase_counter_reg[31]_i_4_CO_UNCONNECTED ;
  wire [3:0]\NLW_phase_counter_reg[31]_i_4_O_UNCONNECTED ;
  wire [3:0]\NLW_phase_counter_reg[31]_i_8_O_UNCONNECTED ;

  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'hAC)) 
    B_V_data_1_data_out
       (.I0(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .I1(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(out_stream_TDATA));
  LUT5 #(
    .INIT(32'hEFEE2022)) 
    \B_V_data_1_payload_A[0]_i_1__0 
       (.I0(curr_sign_reg_177_pp0_iter1_reg),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg[0]_0 ),
        .I4(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .O(\B_V_data_1_payload_A[0]_i_1__0_n_0 ));
  FDRE \B_V_data_1_payload_A_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_A[0]_i_1__0_n_0 ),
        .Q(\B_V_data_1_payload_A_reg_n_0_[0] ),
        .R(1'b0));
  LUT5 #(
    .INIT(32'hBFBB8088)) 
    \B_V_data_1_payload_B[0]_i_1__0 
       (.I0(curr_sign_reg_177_pp0_iter1_reg),
        .I1(B_V_data_1_sel_wr),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg[0]_0 ),
        .I4(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .O(\B_V_data_1_payload_B[0]_i_1__0_n_0 ));
  FDRE \B_V_data_1_payload_B_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_payload_B[0]_i_1__0_n_0 ),
        .Q(\B_V_data_1_payload_B_reg_n_0_[0] ),
        .R(1'b0));
  LUT2 #(
    .INIT(4'h6)) 
    B_V_data_1_sel_rd_i_1__0
       (.I0(B_V_data_1_sel0),
        .I1(B_V_data_1_sel),
        .O(B_V_data_1_sel_rd_reg_0));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'h78)) 
    B_V_data_1_sel_rd_i_1__1
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(out_stream_TREADY),
        .I2(B_V_data_1_sel_rd_reg_n_0),
        .O(B_V_data_1_sel_rd_i_1__1_n_0));
  FDRE B_V_data_1_sel_rd_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_rd_i_1__1_n_0),
        .Q(B_V_data_1_sel_rd_reg_n_0),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT4 #(
    .INIT(16'h7F80)) 
    B_V_data_1_sel_wr_i_1__0
       (.I0(\B_V_data_1_state_reg[0]_1 ),
        .I1(ap_enable_reg_pp0_iter2),
        .I2(B_V_data_1_sel0),
        .I3(B_V_data_1_sel_wr),
        .O(B_V_data_1_sel_wr_i_1__0_n_0));
  FDRE B_V_data_1_sel_wr_reg
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_sel_wr_i_1__0_n_0),
        .Q(B_V_data_1_sel_wr),
        .R(ap_rst_n_inv));
  LUT6 #(
    .INIT(64'hFF2A2A2A2A2A2A2A)) 
    \B_V_data_1_state[0]_i_1 
       (.I0(\B_V_data_1_state_reg[0]_0 ),
        .I1(out_stream_TREADY),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(\B_V_data_1_state_reg[0]_1 ),
        .I4(ap_enable_reg_pp0_iter2),
        .I5(B_V_data_1_sel0),
        .O(\B_V_data_1_state[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hBBFBFBFBFBFBFBFB)) 
    \B_V_data_1_state[1]_i_1 
       (.I0(out_stream_TREADY),
        .I1(\B_V_data_1_state_reg[0]_0 ),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(B_V_data_1_sel0),
        .I4(ap_enable_reg_pp0_iter2),
        .I5(\B_V_data_1_state_reg[0]_1 ),
        .O(B_V_data_1_state));
  FDRE \B_V_data_1_state_reg[0] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(\B_V_data_1_state[0]_i_1_n_0 ),
        .Q(\B_V_data_1_state_reg[0]_0 ),
        .R(ap_rst_n_inv));
  FDRE \B_V_data_1_state_reg[1] 
       (.C(ap_clk),
        .CE(1'b1),
        .D(B_V_data_1_state),
        .Q(\B_V_data_1_state_reg_n_0_[1] ),
        .R(ap_rst_n_inv));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT2 #(
    .INIT(4'hE)) 
    ap_enable_reg_pp0_iter1_i_1
       (.I0(B_V_data_1_sel0),
        .I1(ap_enable_reg_pp0_iter1),
        .O(ap_enable_reg_pp0_iter1_reg));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    ap_enable_reg_pp0_iter2_i_1
       (.I0(ap_enable_reg_pp0_iter1),
        .I1(B_V_data_1_sel0),
        .I2(ap_enable_reg_pp0_iter2),
        .O(ap_enable_reg_pp0_iter1_reg_0));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    ap_enable_reg_pp0_iter3_i_1
       (.I0(ap_enable_reg_pp0_iter2),
        .I1(B_V_data_1_sel0),
        .I2(ap_enable_reg_pp0_iter3),
        .O(ap_enable_reg_pp0_iter2_reg));
  LUT5 #(
    .INIT(32'h80888888)) 
    \curr_sign_reg_177[0]_i_1 
       (.I0(\curr_sign_reg_177[0]_i_3_n_0 ),
        .I1(in_stream_TVALID_int_regslice),
        .I2(\B_V_data_1_state_reg_n_0_[1] ),
        .I3(ap_enable_reg_pp0_iter2),
        .I4(\B_V_data_1_state_reg[0]_1 ),
        .O(B_V_data_1_sel0));
  LUT5 #(
    .INIT(32'h8A8FFFFF)) 
    \curr_sign_reg_177[0]_i_3 
       (.I0(\B_V_data_1_state_reg_n_0_[1] ),
        .I1(out_stream_TREADY),
        .I2(\B_V_data_1_state_reg[0]_0 ),
        .I3(icmp_ln45_reg_184_pp0_iter2_reg),
        .I4(ap_enable_reg_pp0_iter3),
        .O(\curr_sign_reg_177[0]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0000AAAA0300AAAA)) 
    \icmp_ln45_reg_184[0]_i_1 
       (.I0(\B_V_data_1_state_reg[0]_1 ),
        .I1(\icmp_ln45_reg_184[0]_i_2_n_0 ),
        .I2(\icmp_ln45_reg_184[0]_i_3_n_0 ),
        .I3(\icmp_ln45_reg_184[0]_i_4_n_0 ),
        .I4(B_V_data_1_sel0),
        .I5(\icmp_ln45_reg_184[0]_i_5_n_0 ),
        .O(\icmp_ln45_reg_184_reg[0] ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFDFFFFFD)) 
    \icmp_ln45_reg_184[0]_i_10 
       (.I0(Q[2]),
        .I1(Q[14]),
        .I2(Q[12]),
        .I3(curr_sign_reg_177),
        .I4(prev_sign),
        .I5(Q[24]),
        .O(\icmp_ln45_reg_184[0]_i_10_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFAFAFAFAE)) 
    \icmp_ln45_reg_184[0]_i_2 
       (.I0(\icmp_ln45_reg_184[0]_i_6_n_0 ),
        .I1(Q[19]),
        .I2(\icmp_ln45_reg_184[0]_i_7_n_0 ),
        .I3(Q[8]),
        .I4(Q[28]),
        .I5(\icmp_ln45_reg_184[0]_i_8_n_0 ),
        .O(\icmp_ln45_reg_184[0]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFF33333332)) 
    \icmp_ln45_reg_184[0]_i_3 
       (.I0(Q[25]),
        .I1(\icmp_ln45_reg_184[0]_i_7_n_0 ),
        .I2(Q[16]),
        .I3(Q[21]),
        .I4(Q[15]),
        .I5(\icmp_ln45_reg_184[0]_i_9_n_0 ),
        .O(\icmp_ln45_reg_184[0]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000004)) 
    \icmp_ln45_reg_184[0]_i_4 
       (.I0(Q[31]),
        .I1(Q[3]),
        .I2(\icmp_ln45_reg_184[0]_i_7_n_0 ),
        .I3(Q[17]),
        .I4(Q[10]),
        .I5(\icmp_ln45_reg_184[0]_i_10_n_0 ),
        .O(\icmp_ln45_reg_184[0]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT5 #(
    .INIT(32'hF00FE00E)) 
    \icmp_ln45_reg_184[0]_i_5 
       (.I0(Q[0]),
        .I1(Q[6]),
        .I2(curr_sign_reg_177),
        .I3(prev_sign),
        .I4(Q[23]),
        .O(\icmp_ln45_reg_184[0]_i_5_n_0 ));
  LUT6 #(
    .INIT(64'hFF0000FFFE0000FE)) 
    \icmp_ln45_reg_184[0]_i_6 
       (.I0(Q[18]),
        .I1(Q[30]),
        .I2(Q[26]),
        .I3(curr_sign_reg_177),
        .I4(prev_sign),
        .I5(Q[29]),
        .O(\icmp_ln45_reg_184[0]_i_6_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \icmp_ln45_reg_184[0]_i_7 
       (.I0(prev_sign),
        .I1(curr_sign_reg_177),
        .O(\icmp_ln45_reg_184[0]_i_7_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFFFFE)) 
    \icmp_ln45_reg_184[0]_i_8 
       (.I0(Q[1]),
        .I1(Q[5]),
        .I2(Q[22]),
        .I3(Q[9]),
        .I4(Q[11]),
        .I5(Q[4]),
        .O(\icmp_ln45_reg_184[0]_i_8_n_0 ));
  LUT6 #(
    .INIT(64'hFF0000FFFE0000FE)) 
    \icmp_ln45_reg_184[0]_i_9 
       (.I0(Q[7]),
        .I1(Q[13]),
        .I2(Q[20]),
        .I3(curr_sign_reg_177),
        .I4(prev_sign),
        .I5(Q[27]),
        .O(\icmp_ln45_reg_184[0]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[12]_i_2 
       (.I0(Q[12]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[12]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[12]_i_3 
       (.I0(Q[11]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[11]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[12]_i_4 
       (.I0(Q[10]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[10]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[12]_i_5 
       (.I0(Q[9]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[9]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[16]_i_2 
       (.I0(Q[16]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[16]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[16]_i_3 
       (.I0(Q[15]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[15]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[16]_i_4 
       (.I0(Q[14]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[14]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[16]_i_5 
       (.I0(Q[13]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[13]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[20]_i_2 
       (.I0(Q[20]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[20]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[20]_i_3 
       (.I0(Q[19]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[19]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[20]_i_4 
       (.I0(Q[18]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[18]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[20]_i_5 
       (.I0(Q[17]),
        .I1(prev_sign),
        .I2(curr_sign_reg_177),
        .O(select_ln40_fu_123_p3[17]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[24]_i_2 
       (.I0(Q[24]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[24]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[24]_i_3 
       (.I0(Q[23]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[23]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[24]_i_4 
       (.I0(Q[22]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[22]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[24]_i_5 
       (.I0(Q[21]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[21]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[28]_i_2 
       (.I0(Q[28]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[28]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[28]_i_3 
       (.I0(Q[27]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[27]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[28]_i_4 
       (.I0(Q[26]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[26]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[28]_i_5 
       (.I0(Q[25]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[25]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \phase_counter[31]_i_1 
       (.I0(CO),
        .I1(E),
        .O(SR));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_10 
       (.I0(D[28]),
        .I1(D[27]),
        .O(\phase_counter[31]_i_10_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_11 
       (.I0(D[26]),
        .I1(D[25]),
        .O(\phase_counter[31]_i_11_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_12 
       (.I0(D[29]),
        .I1(D[30]),
        .O(\phase_counter[31]_i_12_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_13 
       (.I0(D[27]),
        .I1(D[28]),
        .O(\phase_counter[31]_i_13_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_14 
       (.I0(D[25]),
        .I1(D[26]),
        .O(\phase_counter[31]_i_14_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_16 
       (.I0(D[24]),
        .I1(D[23]),
        .O(\phase_counter[31]_i_16_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_17 
       (.I0(D[22]),
        .I1(D[21]),
        .O(\phase_counter[31]_i_17_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_18 
       (.I0(D[20]),
        .I1(D[19]),
        .O(\phase_counter[31]_i_18_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_19 
       (.I0(D[18]),
        .I1(D[17]),
        .O(\phase_counter[31]_i_19_n_0 ));
  LUT6 #(
    .INIT(64'hAA2A000000000000)) 
    \phase_counter[31]_i_2 
       (.I0(ap_enable_reg_pp0_iter1),
        .I1(\B_V_data_1_state_reg[0]_1 ),
        .I2(ap_enable_reg_pp0_iter2),
        .I3(\B_V_data_1_state_reg_n_0_[1] ),
        .I4(in_stream_TVALID_int_regslice),
        .I5(\curr_sign_reg_177[0]_i_3_n_0 ),
        .O(E));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_20 
       (.I0(D[23]),
        .I1(D[24]),
        .O(\phase_counter[31]_i_20_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_21 
       (.I0(D[21]),
        .I1(D[22]),
        .O(\phase_counter[31]_i_21_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_22 
       (.I0(D[19]),
        .I1(D[20]),
        .O(\phase_counter[31]_i_22_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_23 
       (.I0(D[17]),
        .I1(D[18]),
        .O(\phase_counter[31]_i_23_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_25 
       (.I0(D[16]),
        .I1(D[15]),
        .O(\phase_counter[31]_i_25_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_26 
       (.I0(D[14]),
        .I1(D[13]),
        .O(\phase_counter[31]_i_26_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_27 
       (.I0(D[12]),
        .I1(D[11]),
        .O(\phase_counter[31]_i_27_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_28 
       (.I0(D[10]),
        .I1(D[9]),
        .O(\phase_counter[31]_i_28_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_29 
       (.I0(D[15]),
        .I1(D[16]),
        .O(\phase_counter[31]_i_29_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_30 
       (.I0(D[13]),
        .I1(D[14]),
        .O(\phase_counter[31]_i_30_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_31 
       (.I0(D[11]),
        .I1(D[12]),
        .O(\phase_counter[31]_i_31_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_32 
       (.I0(D[9]),
        .I1(D[10]),
        .O(\phase_counter[31]_i_32_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_33 
       (.I0(D[8]),
        .I1(D[7]),
        .O(\phase_counter[31]_i_33_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \phase_counter[31]_i_34 
       (.I0(D[6]),
        .I1(D[5]),
        .O(\phase_counter[31]_i_34_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_35 
       (.I0(D[7]),
        .I1(D[8]),
        .O(\phase_counter[31]_i_35_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \phase_counter[31]_i_36 
       (.I0(D[5]),
        .I1(D[6]),
        .O(\phase_counter[31]_i_36_n_0 ));
  LUT2 #(
    .INIT(4'h2)) 
    \phase_counter[31]_i_37 
       (.I0(D[3]),
        .I1(D[4]),
        .O(\phase_counter[31]_i_37_n_0 ));
  LUT2 #(
    .INIT(4'h2)) 
    \phase_counter[31]_i_38 
       (.I0(D[1]),
        .I1(D[2]),
        .O(\phase_counter[31]_i_38_n_0 ));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[31]_i_5 
       (.I0(Q[31]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[31]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[31]_i_6 
       (.I0(Q[30]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[30]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[31]_i_7 
       (.I0(Q[29]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[29]));
  LUT2 #(
    .INIT(4'h2)) 
    \phase_counter[31]_i_9 
       (.I0(D[29]),
        .I1(D[30]),
        .O(\phase_counter[31]_i_9_n_0 ));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[4]_i_2 
       (.I0(Q[0]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[0]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[4]_i_3 
       (.I0(Q[4]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[4]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[4]_i_4 
       (.I0(Q[3]),
        .I1(prev_sign),
        .I2(curr_sign_reg_177),
        .O(select_ln40_fu_123_p3[3]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[4]_i_5 
       (.I0(Q[2]),
        .I1(prev_sign),
        .I2(curr_sign_reg_177),
        .O(select_ln40_fu_123_p3[2]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[4]_i_6 
       (.I0(Q[1]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[1]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[8]_i_2 
       (.I0(Q[8]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[8]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[8]_i_3 
       (.I0(Q[7]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[7]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[8]_i_4 
       (.I0(Q[6]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[6]));
  LUT3 #(
    .INIT(8'h82)) 
    \phase_counter[8]_i_5 
       (.I0(Q[5]),
        .I1(curr_sign_reg_177),
        .I2(prev_sign),
        .O(select_ln40_fu_123_p3[5]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \phase_counter_reg[12]_i_1 
       (.CI(\phase_counter_reg[8]_i_1_n_0 ),
        .CO({\phase_counter_reg[12]_i_1_n_0 ,\phase_counter_reg[12]_i_1_n_1 ,\phase_counter_reg[12]_i_1_n_2 ,\phase_counter_reg[12]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(D[11:8]),
        .S(select_ln40_fu_123_p3[12:9]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \phase_counter_reg[16]_i_1 
       (.CI(\phase_counter_reg[12]_i_1_n_0 ),
        .CO({\phase_counter_reg[16]_i_1_n_0 ,\phase_counter_reg[16]_i_1_n_1 ,\phase_counter_reg[16]_i_1_n_2 ,\phase_counter_reg[16]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(D[15:12]),
        .S(select_ln40_fu_123_p3[16:13]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \phase_counter_reg[20]_i_1 
       (.CI(\phase_counter_reg[16]_i_1_n_0 ),
        .CO({\phase_counter_reg[20]_i_1_n_0 ,\phase_counter_reg[20]_i_1_n_1 ,\phase_counter_reg[20]_i_1_n_2 ,\phase_counter_reg[20]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(D[19:16]),
        .S(select_ln40_fu_123_p3[20:17]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \phase_counter_reg[24]_i_1 
       (.CI(\phase_counter_reg[20]_i_1_n_0 ),
        .CO({\phase_counter_reg[24]_i_1_n_0 ,\phase_counter_reg[24]_i_1_n_1 ,\phase_counter_reg[24]_i_1_n_2 ,\phase_counter_reg[24]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(D[23:20]),
        .S(select_ln40_fu_123_p3[24:21]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \phase_counter_reg[28]_i_1 
       (.CI(\phase_counter_reg[24]_i_1_n_0 ),
        .CO({\phase_counter_reg[28]_i_1_n_0 ,\phase_counter_reg[28]_i_1_n_1 ,\phase_counter_reg[28]_i_1_n_2 ,\phase_counter_reg[28]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(D[27:24]),
        .S(select_ln40_fu_123_p3[28:25]));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \phase_counter_reg[31]_i_15 
       (.CI(\phase_counter_reg[31]_i_24_n_0 ),
        .CO({\phase_counter_reg[31]_i_15_n_0 ,\phase_counter_reg[31]_i_15_n_1 ,\phase_counter_reg[31]_i_15_n_2 ,\phase_counter_reg[31]_i_15_n_3 }),
        .CYINIT(1'b0),
        .DI({\phase_counter[31]_i_25_n_0 ,\phase_counter[31]_i_26_n_0 ,\phase_counter[31]_i_27_n_0 ,\phase_counter[31]_i_28_n_0 }),
        .O(\NLW_phase_counter_reg[31]_i_15_O_UNCONNECTED [3:0]),
        .S({\phase_counter[31]_i_29_n_0 ,\phase_counter[31]_i_30_n_0 ,\phase_counter[31]_i_31_n_0 ,\phase_counter[31]_i_32_n_0 }));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \phase_counter_reg[31]_i_24 
       (.CI(1'b0),
        .CO({\phase_counter_reg[31]_i_24_n_0 ,\phase_counter_reg[31]_i_24_n_1 ,\phase_counter_reg[31]_i_24_n_2 ,\phase_counter_reg[31]_i_24_n_3 }),
        .CYINIT(1'b0),
        .DI({\phase_counter[31]_i_33_n_0 ,\phase_counter[31]_i_34_n_0 ,D[4],D[2]}),
        .O(\NLW_phase_counter_reg[31]_i_24_O_UNCONNECTED [3:0]),
        .S({\phase_counter[31]_i_35_n_0 ,\phase_counter[31]_i_36_n_0 ,\phase_counter[31]_i_37_n_0 ,\phase_counter[31]_i_38_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \phase_counter_reg[31]_i_3 
       (.CI(\phase_counter_reg[28]_i_1_n_0 ),
        .CO({\NLW_phase_counter_reg[31]_i_3_CO_UNCONNECTED [3:2],\phase_counter_reg[31]_i_3_n_2 ,\phase_counter_reg[31]_i_3_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_phase_counter_reg[31]_i_3_O_UNCONNECTED [3],D[30:28]}),
        .S({1'b0,select_ln40_fu_123_p3[31:29]}));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \phase_counter_reg[31]_i_4 
       (.CI(\phase_counter_reg[31]_i_8_n_0 ),
        .CO({\NLW_phase_counter_reg[31]_i_4_CO_UNCONNECTED [3],CO,\phase_counter_reg[31]_i_4_n_2 ,\phase_counter_reg[31]_i_4_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,\phase_counter[31]_i_9_n_0 ,\phase_counter[31]_i_10_n_0 ,\phase_counter[31]_i_11_n_0 }),
        .O(\NLW_phase_counter_reg[31]_i_4_O_UNCONNECTED [3:0]),
        .S({1'b0,\phase_counter[31]_i_12_n_0 ,\phase_counter[31]_i_13_n_0 ,\phase_counter[31]_i_14_n_0 }));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 \phase_counter_reg[31]_i_8 
       (.CI(\phase_counter_reg[31]_i_15_n_0 ),
        .CO({\phase_counter_reg[31]_i_8_n_0 ,\phase_counter_reg[31]_i_8_n_1 ,\phase_counter_reg[31]_i_8_n_2 ,\phase_counter_reg[31]_i_8_n_3 }),
        .CYINIT(1'b0),
        .DI({\phase_counter[31]_i_16_n_0 ,\phase_counter[31]_i_17_n_0 ,\phase_counter[31]_i_18_n_0 ,\phase_counter[31]_i_19_n_0 }),
        .O(\NLW_phase_counter_reg[31]_i_8_O_UNCONNECTED [3:0]),
        .S({\phase_counter[31]_i_20_n_0 ,\phase_counter[31]_i_21_n_0 ,\phase_counter[31]_i_22_n_0 ,\phase_counter[31]_i_23_n_0 }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \phase_counter_reg[4]_i_1 
       (.CI(1'b0),
        .CO({\phase_counter_reg[4]_i_1_n_0 ,\phase_counter_reg[4]_i_1_n_1 ,\phase_counter_reg[4]_i_1_n_2 ,\phase_counter_reg[4]_i_1_n_3 }),
        .CYINIT(select_ln40_fu_123_p3[0]),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(D[3:0]),
        .S(select_ln40_fu_123_p3[4:1]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \phase_counter_reg[8]_i_1 
       (.CI(\phase_counter_reg[4]_i_1_n_0 ),
        .CO({\phase_counter_reg[8]_i_1_n_0 ,\phase_counter_reg[8]_i_1_n_1 ,\phase_counter_reg[8]_i_1_n_2 ,\phase_counter_reg[8]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(D[7:4]),
        .S(select_ln40_fu_123_p3[8:5]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    \prev_sign[0]_i_1 
       (.I0(curr_sign_reg_177),
        .I1(E),
        .I2(prev_sign),
        .O(\curr_sign_reg_177_reg[0] ));
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
