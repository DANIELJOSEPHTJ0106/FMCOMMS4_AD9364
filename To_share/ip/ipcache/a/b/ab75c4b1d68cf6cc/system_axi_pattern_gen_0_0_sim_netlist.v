// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 18:02:49 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_axi_pattern_gen_0_0/system_axi_pattern_gen_0_0_sim_netlist.v
// Design      : system_axi_pattern_gen_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "system_axi_pattern_gen_0_0,axi_pattern_gen,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "axi_pattern_gen,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module system_axi_pattern_gen_0_0
   (aclk,
    aresetn,
    m_axis_tdata,
    m_axis_tuser,
    m_axis_tvalid,
    m_axis_tready,
    m_axis_tlast);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF m_axis, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input aclk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input aresetn;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TDATA" *) output [7:0]m_axis_tdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TUSER" *) output [12:0]m_axis_tuser;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TVALID" *) output m_axis_tvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TREADY" *) input m_axis_tready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TLAST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME m_axis, TDATA_NUM_BYTES 1, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 13, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 0, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, LAYERED_METADATA undef, INSERT_VIP 0" *) output m_axis_tlast;

  wire \<const0> ;
  wire aclk;
  wire aresetn;
  wire [0:0]\^m_axis_tdata ;
  wire m_axis_tready;
  wire [12:0]m_axis_tuser;
  wire m_axis_tvalid;

  assign m_axis_tdata[7] = \<const0> ;
  assign m_axis_tdata[6] = \<const0> ;
  assign m_axis_tdata[5] = \<const0> ;
  assign m_axis_tdata[4] = \<const0> ;
  assign m_axis_tdata[3] = \<const0> ;
  assign m_axis_tdata[2] = \<const0> ;
  assign m_axis_tdata[1] = \<const0> ;
  assign m_axis_tdata[0] = \^m_axis_tdata [0];
  assign m_axis_tlast = \<const0> ;
  GND GND
       (.G(\<const0> ));
  system_axi_pattern_gen_0_0_axi_pattern_gen inst
       (.Q(m_axis_tuser),
        .aclk(aclk),
        .aresetn(aresetn),
        .m_axis_tdata(\^m_axis_tdata ),
        .m_axis_tready(m_axis_tready),
        .m_axis_tvalid(m_axis_tvalid));
endmodule

(* ORIG_REF_NAME = "axi_pattern_gen" *) 
module system_axi_pattern_gen_0_0_axi_pattern_gen
   (Q,
    m_axis_tdata,
    m_axis_tvalid,
    m_axis_tready,
    aclk,
    aresetn);
  output [12:0]Q;
  output [0:0]m_axis_tdata;
  output m_axis_tvalid;
  input m_axis_tready;
  input aclk;
  input aresetn;

  wire [12:0]Q;
  wire aclk;
  wire aresetn;
  wire \bit_cntr[0]_i_1_n_0 ;
  wire \bit_cntr[1]_i_1_n_0 ;
  wire \bit_cntr[2]_i_1_n_0 ;
  wire \bit_cntr[2]_i_2_n_0 ;
  wire \bit_cntr_reg_n_0_[0] ;
  wire \bit_cntr_reg_n_0_[1] ;
  wire \bit_cntr_reg_n_0_[2] ;
  wire [0:0]m_axis_tdata;
  wire m_axis_tready;
  wire m_axis_tvalid;
  wire m_axis_tvalid_i_1_n_0;
  wire [12:0]sample_counter;
  wire sample_counter0_carry__0_n_0;
  wire sample_counter0_carry__0_n_1;
  wire sample_counter0_carry__0_n_2;
  wire sample_counter0_carry__0_n_3;
  wire sample_counter0_carry__0_n_4;
  wire sample_counter0_carry__0_n_5;
  wire sample_counter0_carry__0_n_6;
  wire sample_counter0_carry__0_n_7;
  wire sample_counter0_carry__1_n_1;
  wire sample_counter0_carry__1_n_2;
  wire sample_counter0_carry__1_n_3;
  wire sample_counter0_carry__1_n_4;
  wire sample_counter0_carry__1_n_5;
  wire sample_counter0_carry__1_n_6;
  wire sample_counter0_carry__1_n_7;
  wire sample_counter0_carry_n_0;
  wire sample_counter0_carry_n_1;
  wire sample_counter0_carry_n_2;
  wire sample_counter0_carry_n_3;
  wire sample_counter0_carry_n_4;
  wire sample_counter0_carry_n_5;
  wire sample_counter0_carry_n_6;
  wire sample_counter0_carry_n_7;
  wire \sample_counter[12]_i_2_n_0 ;
  wire \sample_counter[12]_i_3_n_0 ;
  wire \sample_counter[12]_i_4_n_0 ;
  wire \temp_cntr[0]_i_1_n_0 ;
  wire \temp_cntr[1]_i_1_n_0 ;
  wire \temp_cntr[2]_i_1_n_0 ;
  wire \temp_cntr[2]_i_2_n_0 ;
  wire \temp_cntr[3]_i_1_n_0 ;
  wire \temp_cntr[4]_i_1_n_0 ;
  wire \temp_cntr[5]_i_1_n_0 ;
  wire \temp_cntr[6]_i_1_n_0 ;
  wire \temp_cntr[7]_i_1_n_0 ;
  wire \temp_cntr[7]_i_2_n_0 ;
  wire \temp_cntr[7]_i_3_n_0 ;
  wire \temp_cntr_reg_n_0_[0] ;
  wire \temp_cntr_reg_n_0_[1] ;
  wire \temp_cntr_reg_n_0_[2] ;
  wire \temp_cntr_reg_n_0_[3] ;
  wire \temp_cntr_reg_n_0_[4] ;
  wire \temp_cntr_reg_n_0_[5] ;
  wire \temp_cntr_reg_n_0_[6] ;
  wire \temp_cntr_reg_n_0_[7] ;
  wire [3:3]NLW_sample_counter0_carry__1_CO_UNCONNECTED;

  LUT1 #(
    .INIT(2'h1)) 
    \bit_cntr[0]_i_1 
       (.I0(\bit_cntr_reg_n_0_[0] ),
        .O(\bit_cntr[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT2 #(
    .INIT(4'h9)) 
    \bit_cntr[1]_i_1 
       (.I0(\bit_cntr_reg_n_0_[1] ),
        .I1(\bit_cntr_reg_n_0_[0] ),
        .O(\bit_cntr[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000800)) 
    \bit_cntr[2]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(m_axis_tready),
        .I2(Q[8]),
        .I3(Q[7]),
        .I4(Q[0]),
        .I5(\sample_counter[12]_i_4_n_0 ),
        .O(\bit_cntr[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hE1)) 
    \bit_cntr[2]_i_2 
       (.I0(\bit_cntr_reg_n_0_[1] ),
        .I1(\bit_cntr_reg_n_0_[0] ),
        .I2(\bit_cntr_reg_n_0_[2] ),
        .O(\bit_cntr[2]_i_2_n_0 ));
  FDPE \bit_cntr_reg[0] 
       (.C(aclk),
        .CE(\bit_cntr[2]_i_1_n_0 ),
        .D(\bit_cntr[0]_i_1_n_0 ),
        .PRE(\sample_counter[12]_i_2_n_0 ),
        .Q(\bit_cntr_reg_n_0_[0] ));
  FDPE \bit_cntr_reg[1] 
       (.C(aclk),
        .CE(\bit_cntr[2]_i_1_n_0 ),
        .D(\bit_cntr[1]_i_1_n_0 ),
        .PRE(\sample_counter[12]_i_2_n_0 ),
        .Q(\bit_cntr_reg_n_0_[1] ));
  FDPE \bit_cntr_reg[2] 
       (.C(aclk),
        .CE(\bit_cntr[2]_i_1_n_0 ),
        .D(\bit_cntr[2]_i_2_n_0 ),
        .PRE(\sample_counter[12]_i_2_n_0 ),
        .Q(\bit_cntr_reg_n_0_[2] ));
  LUT6 #(
    .INIT(64'h0030BBB8001144B8)) 
    \m_axis_tdata[0]_INST_0 
       (.I0(\bit_cntr_reg_n_0_[0] ),
        .I1(\temp_cntr_reg_n_0_[0] ),
        .I2(\bit_cntr_reg_n_0_[1] ),
        .I3(\temp_cntr_reg_n_0_[1] ),
        .I4(\temp_cntr_reg_n_0_[2] ),
        .I5(\bit_cntr_reg_n_0_[2] ),
        .O(m_axis_tdata));
  LUT3 #(
    .INIT(8'hF8)) 
    m_axis_tvalid_i_1
       (.I0(m_axis_tready),
        .I1(aresetn),
        .I2(m_axis_tvalid),
        .O(m_axis_tvalid_i_1_n_0));
  FDRE m_axis_tvalid_reg
       (.C(aclk),
        .CE(1'b1),
        .D(m_axis_tvalid_i_1_n_0),
        .Q(m_axis_tvalid),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 sample_counter0_carry
       (.CI(1'b0),
        .CO({sample_counter0_carry_n_0,sample_counter0_carry_n_1,sample_counter0_carry_n_2,sample_counter0_carry_n_3}),
        .CYINIT(Q[0]),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({sample_counter0_carry_n_4,sample_counter0_carry_n_5,sample_counter0_carry_n_6,sample_counter0_carry_n_7}),
        .S(Q[4:1]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 sample_counter0_carry__0
       (.CI(sample_counter0_carry_n_0),
        .CO({sample_counter0_carry__0_n_0,sample_counter0_carry__0_n_1,sample_counter0_carry__0_n_2,sample_counter0_carry__0_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({sample_counter0_carry__0_n_4,sample_counter0_carry__0_n_5,sample_counter0_carry__0_n_6,sample_counter0_carry__0_n_7}),
        .S(Q[8:5]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 sample_counter0_carry__1
       (.CI(sample_counter0_carry__0_n_0),
        .CO({NLW_sample_counter0_carry__1_CO_UNCONNECTED[3],sample_counter0_carry__1_n_1,sample_counter0_carry__1_n_2,sample_counter0_carry__1_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({sample_counter0_carry__1_n_4,sample_counter0_carry__1_n_5,sample_counter0_carry__1_n_6,sample_counter0_carry__1_n_7}),
        .S(Q[12:9]));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \sample_counter[0]_i_1 
       (.I0(Q[0]),
        .O(sample_counter[0]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[10]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry__1_n_6),
        .O(sample_counter[10]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[11]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry__1_n_5),
        .O(sample_counter[11]));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[12]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry__1_n_4),
        .O(sample_counter[12]));
  LUT1 #(
    .INIT(2'h1)) 
    \sample_counter[12]_i_2 
       (.I0(aresetn),
        .O(\sample_counter[12]_i_2_n_0 ));
  LUT5 #(
    .INIT(32'h80000000)) 
    \sample_counter[12]_i_3 
       (.I0(Q[6]),
        .I1(Q[1]),
        .I2(Q[7]),
        .I3(Q[11]),
        .I4(Q[5]),
        .O(\sample_counter[12]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFDFFFFFFFFFFF)) 
    \sample_counter[12]_i_4 
       (.I0(Q[12]),
        .I1(Q[9]),
        .I2(Q[2]),
        .I3(Q[4]),
        .I4(Q[10]),
        .I5(Q[3]),
        .O(\sample_counter[12]_i_4_n_0 ));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[1]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry_n_7),
        .O(sample_counter[1]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[2]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry_n_6),
        .O(sample_counter[2]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[3]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry_n_5),
        .O(sample_counter[3]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[4]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry_n_4),
        .O(sample_counter[4]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[5]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry__0_n_7),
        .O(sample_counter[5]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[6]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry__0_n_6),
        .O(sample_counter[6]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[7]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry__0_n_5),
        .O(sample_counter[7]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[8]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry__0_n_4),
        .O(sample_counter[8]));
  LUT5 #(
    .INIT(32'hFFDF0000)) 
    \sample_counter[9]_i_1 
       (.I0(\sample_counter[12]_i_3_n_0 ),
        .I1(Q[8]),
        .I2(Q[0]),
        .I3(\sample_counter[12]_i_4_n_0 ),
        .I4(sample_counter0_carry__1_n_7),
        .O(sample_counter[9]));
  FDCE \sample_counter_reg[0] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[0]),
        .Q(Q[0]));
  FDCE \sample_counter_reg[10] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[10]),
        .Q(Q[10]));
  FDCE \sample_counter_reg[11] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[11]),
        .Q(Q[11]));
  FDCE \sample_counter_reg[12] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[12]),
        .Q(Q[12]));
  FDCE \sample_counter_reg[1] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[1]),
        .Q(Q[1]));
  FDCE \sample_counter_reg[2] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[2]),
        .Q(Q[2]));
  FDCE \sample_counter_reg[3] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[3]),
        .Q(Q[3]));
  FDCE \sample_counter_reg[4] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[4]),
        .Q(Q[4]));
  FDCE \sample_counter_reg[5] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[5]),
        .Q(Q[5]));
  FDCE \sample_counter_reg[6] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[6]),
        .Q(Q[6]));
  FDCE \sample_counter_reg[7] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[7]),
        .Q(Q[7]));
  FDCE \sample_counter_reg[8] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[8]),
        .Q(Q[8]));
  FDCE \sample_counter_reg[9] 
       (.C(aclk),
        .CE(m_axis_tready),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(sample_counter[9]),
        .Q(Q[9]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h32333333)) 
    \temp_cntr[0]_i_1 
       (.I0(\temp_cntr_reg_n_0_[1] ),
        .I1(\temp_cntr_reg_n_0_[0] ),
        .I2(\temp_cntr_reg_n_0_[6] ),
        .I3(\temp_cntr_reg_n_0_[2] ),
        .I4(\temp_cntr[2]_i_2_n_0 ),
        .O(\temp_cntr[0]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'h6)) 
    \temp_cntr[1]_i_1 
       (.I0(\temp_cntr_reg_n_0_[0] ),
        .I1(\temp_cntr_reg_n_0_[1] ),
        .O(\temp_cntr[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h76887788)) 
    \temp_cntr[2]_i_1 
       (.I0(\temp_cntr_reg_n_0_[1] ),
        .I1(\temp_cntr_reg_n_0_[0] ),
        .I2(\temp_cntr_reg_n_0_[6] ),
        .I3(\temp_cntr_reg_n_0_[2] ),
        .I4(\temp_cntr[2]_i_2_n_0 ),
        .O(\temp_cntr[2]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'h0001)) 
    \temp_cntr[2]_i_2 
       (.I0(\temp_cntr_reg_n_0_[4] ),
        .I1(\temp_cntr_reg_n_0_[3] ),
        .I2(\temp_cntr_reg_n_0_[7] ),
        .I3(\temp_cntr_reg_n_0_[5] ),
        .O(\temp_cntr[2]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'h6AAA)) 
    \temp_cntr[3]_i_1 
       (.I0(\temp_cntr_reg_n_0_[3] ),
        .I1(\temp_cntr_reg_n_0_[1] ),
        .I2(\temp_cntr_reg_n_0_[2] ),
        .I3(\temp_cntr_reg_n_0_[0] ),
        .O(\temp_cntr[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT5 #(
    .INIT(32'h6AAAAAAA)) 
    \temp_cntr[4]_i_1 
       (.I0(\temp_cntr_reg_n_0_[4] ),
        .I1(\temp_cntr_reg_n_0_[0] ),
        .I2(\temp_cntr_reg_n_0_[2] ),
        .I3(\temp_cntr_reg_n_0_[1] ),
        .I4(\temp_cntr_reg_n_0_[3] ),
        .O(\temp_cntr[4]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h6AAAAAAAAAAAAAAA)) 
    \temp_cntr[5]_i_1 
       (.I0(\temp_cntr_reg_n_0_[5] ),
        .I1(\temp_cntr_reg_n_0_[3] ),
        .I2(\temp_cntr_reg_n_0_[1] ),
        .I3(\temp_cntr_reg_n_0_[2] ),
        .I4(\temp_cntr_reg_n_0_[0] ),
        .I5(\temp_cntr_reg_n_0_[4] ),
        .O(\temp_cntr[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \temp_cntr[6]_i_1 
       (.I0(\temp_cntr_reg_n_0_[6] ),
        .I1(\temp_cntr[7]_i_3_n_0 ),
        .O(\temp_cntr[6]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'h0002)) 
    \temp_cntr[7]_i_1 
       (.I0(\bit_cntr[2]_i_1_n_0 ),
        .I1(\bit_cntr_reg_n_0_[1] ),
        .I2(\bit_cntr_reg_n_0_[0] ),
        .I3(\bit_cntr_reg_n_0_[2] ),
        .O(\temp_cntr[7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'h6A)) 
    \temp_cntr[7]_i_2 
       (.I0(\temp_cntr_reg_n_0_[7] ),
        .I1(\temp_cntr[7]_i_3_n_0 ),
        .I2(\temp_cntr_reg_n_0_[6] ),
        .O(\temp_cntr[7]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    \temp_cntr[7]_i_3 
       (.I0(\temp_cntr_reg_n_0_[5] ),
        .I1(\temp_cntr_reg_n_0_[3] ),
        .I2(\temp_cntr_reg_n_0_[1] ),
        .I3(\temp_cntr_reg_n_0_[2] ),
        .I4(\temp_cntr_reg_n_0_[0] ),
        .I5(\temp_cntr_reg_n_0_[4] ),
        .O(\temp_cntr[7]_i_3_n_0 ));
  FDCE \temp_cntr_reg[0] 
       (.C(aclk),
        .CE(\temp_cntr[7]_i_1_n_0 ),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(\temp_cntr[0]_i_1_n_0 ),
        .Q(\temp_cntr_reg_n_0_[0] ));
  FDCE \temp_cntr_reg[1] 
       (.C(aclk),
        .CE(\temp_cntr[7]_i_1_n_0 ),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(\temp_cntr[1]_i_1_n_0 ),
        .Q(\temp_cntr_reg_n_0_[1] ));
  FDCE \temp_cntr_reg[2] 
       (.C(aclk),
        .CE(\temp_cntr[7]_i_1_n_0 ),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(\temp_cntr[2]_i_1_n_0 ),
        .Q(\temp_cntr_reg_n_0_[2] ));
  FDCE \temp_cntr_reg[3] 
       (.C(aclk),
        .CE(\temp_cntr[7]_i_1_n_0 ),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(\temp_cntr[3]_i_1_n_0 ),
        .Q(\temp_cntr_reg_n_0_[3] ));
  FDCE \temp_cntr_reg[4] 
       (.C(aclk),
        .CE(\temp_cntr[7]_i_1_n_0 ),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(\temp_cntr[4]_i_1_n_0 ),
        .Q(\temp_cntr_reg_n_0_[4] ));
  FDCE \temp_cntr_reg[5] 
       (.C(aclk),
        .CE(\temp_cntr[7]_i_1_n_0 ),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(\temp_cntr[5]_i_1_n_0 ),
        .Q(\temp_cntr_reg_n_0_[5] ));
  FDCE \temp_cntr_reg[6] 
       (.C(aclk),
        .CE(\temp_cntr[7]_i_1_n_0 ),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(\temp_cntr[6]_i_1_n_0 ),
        .Q(\temp_cntr_reg_n_0_[6] ));
  FDCE \temp_cntr_reg[7] 
       (.C(aclk),
        .CE(\temp_cntr[7]_i_1_n_0 ),
        .CLR(\sample_counter[12]_i_2_n_0 ),
        .D(\temp_cntr[7]_i_2_n_0 ),
        .Q(\temp_cntr_reg_n_0_[7] ));
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
