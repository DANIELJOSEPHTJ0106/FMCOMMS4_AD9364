// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Wed Jan  7 18:03:56 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_valid_high_upd_0_0_sim_netlist.v
// Design      : system_valid_high_upd_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "system_valid_high_upd_0_0,valid_high_upd,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "valid_high_upd,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (aclk,
    aresetn,
    s_axis_tdata,
    s_axis_tvalid,
    m_axis_tdata,
    m_axis_tuser,
    m_axis_tvalid,
    m_axis_tready,
    m_axis_tlast);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF m_axis:s_axis, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input aclk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input aresetn;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TDATA" *) input [7:0]s_axis_tdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TVALID" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME s_axis, TDATA_NUM_BYTES 1, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 0, HAS_TSTRB 0, HAS_TKEEP 0, HAS_TLAST 0, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, LAYERED_METADATA undef, INSERT_VIP 0" *) input s_axis_tvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TDATA" *) output [7:0]m_axis_tdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TUSER" *) output [12:0]m_axis_tuser;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TVALID" *) output m_axis_tvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TREADY" *) input m_axis_tready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TLAST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME m_axis, TDATA_NUM_BYTES 1, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 13, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 0, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, LAYERED_METADATA undef, INSERT_VIP 0" *) output m_axis_tlast;

  wire \<const0> ;
  wire aclk;
  wire aresetn;
  wire [7:0]m_axis_tdata;
  wire m_axis_tready;
  wire [12:0]m_axis_tuser;
  wire m_axis_tvalid;
  wire [7:0]s_axis_tdata;
  wire s_axis_tvalid;

  assign m_axis_tlast = \<const0> ;
  GND GND
       (.G(\<const0> ));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_valid_high_upd inst
       (.aclk(aclk),
        .aresetn(aresetn),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tready(m_axis_tready),
        .m_axis_tuser(m_axis_tuser),
        .m_axis_tvalid(m_axis_tvalid),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_valid_high_upd
   (m_axis_tuser,
    m_axis_tdata,
    m_axis_tvalid,
    s_axis_tvalid,
    s_axis_tdata,
    aclk,
    m_axis_tready,
    aresetn);
  output [12:0]m_axis_tuser;
  output [7:0]m_axis_tdata;
  output m_axis_tvalid;
  input s_axis_tvalid;
  input [7:0]s_axis_tdata;
  input aclk;
  input m_axis_tready;
  input aresetn;

  wire aclk;
  wire aresetn;
  wire [7:0]m_axis_tdata;
  wire m_axis_tready;
  wire [12:0]m_axis_tuser;
  wire m_axis_tvalid;
  wire [7:0]s_axis_tdata;
  wire s_axis_tvalid;
  wire [12:1]sample_counter0;
  wire sample_counter0_carry__0_n_0;
  wire sample_counter0_carry__0_n_1;
  wire sample_counter0_carry__0_n_2;
  wire sample_counter0_carry__0_n_3;
  wire sample_counter0_carry__1_n_1;
  wire sample_counter0_carry__1_n_2;
  wire sample_counter0_carry__1_n_3;
  wire sample_counter0_carry_n_0;
  wire sample_counter0_carry_n_1;
  wire sample_counter0_carry_n_2;
  wire sample_counter0_carry_n_3;
  wire \sample_counter[11]_i_2_n_0 ;
  wire \sample_counter[11]_i_3_n_0 ;
  wire \sample_counter[11]_i_4_n_0 ;
  wire \sample_counter[11]_i_5_n_0 ;
  wire \sample_counter[12]_i_1_n_0 ;
  wire \sample_counter[12]_i_3_n_0 ;
  wire \sample_counter[12]_i_4_n_0 ;
  wire \sample_counter[12]_i_5_n_0 ;
  wire \sample_counter[12]_i_6_n_0 ;
  wire \sample_counter[3]_i_2_n_0 ;
  wire \sample_counter[3]_i_3_n_0 ;
  wire \sample_counter[3]_i_4_n_0 ;
  wire \sample_counter[3]_i_5_n_0 ;
  wire \sample_counter[3]_i_6_n_0 ;
  wire \sample_counter[7]_i_2_n_0 ;
  wire \sample_counter[7]_i_3_n_0 ;
  wire \sample_counter[7]_i_4_n_0 ;
  wire \sample_counter[7]_i_5_n_0 ;
  wire \sample_counter_reg[11]_i_1_n_0 ;
  wire \sample_counter_reg[11]_i_1_n_1 ;
  wire \sample_counter_reg[11]_i_1_n_2 ;
  wire \sample_counter_reg[11]_i_1_n_3 ;
  wire \sample_counter_reg[11]_i_1_n_4 ;
  wire \sample_counter_reg[11]_i_1_n_5 ;
  wire \sample_counter_reg[11]_i_1_n_6 ;
  wire \sample_counter_reg[11]_i_1_n_7 ;
  wire \sample_counter_reg[12]_i_2_n_7 ;
  wire \sample_counter_reg[3]_i_1_n_0 ;
  wire \sample_counter_reg[3]_i_1_n_1 ;
  wire \sample_counter_reg[3]_i_1_n_2 ;
  wire \sample_counter_reg[3]_i_1_n_3 ;
  wire \sample_counter_reg[3]_i_1_n_4 ;
  wire \sample_counter_reg[3]_i_1_n_5 ;
  wire \sample_counter_reg[3]_i_1_n_6 ;
  wire \sample_counter_reg[3]_i_1_n_7 ;
  wire \sample_counter_reg[7]_i_1_n_0 ;
  wire \sample_counter_reg[7]_i_1_n_1 ;
  wire \sample_counter_reg[7]_i_1_n_2 ;
  wire \sample_counter_reg[7]_i_1_n_3 ;
  wire \sample_counter_reg[7]_i_1_n_4 ;
  wire \sample_counter_reg[7]_i_1_n_5 ;
  wire \sample_counter_reg[7]_i_1_n_6 ;
  wire \sample_counter_reg[7]_i_1_n_7 ;
  wire start_cntr_i_1_n_0;
  wire start_cntr_reg_n_0;
  wire \temp_data[7]_i_1_n_0 ;
  wire [3:3]NLW_sample_counter0_carry__1_CO_UNCONNECTED;
  wire [3:0]\NLW_sample_counter_reg[12]_i_2_CO_UNCONNECTED ;
  wire [3:1]\NLW_sample_counter_reg[12]_i_2_O_UNCONNECTED ;

  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 sample_counter0_carry
       (.CI(1'b0),
        .CO({sample_counter0_carry_n_0,sample_counter0_carry_n_1,sample_counter0_carry_n_2,sample_counter0_carry_n_3}),
        .CYINIT(m_axis_tuser[0]),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sample_counter0[4:1]),
        .S(m_axis_tuser[4:1]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 sample_counter0_carry__0
       (.CI(sample_counter0_carry_n_0),
        .CO({sample_counter0_carry__0_n_0,sample_counter0_carry__0_n_1,sample_counter0_carry__0_n_2,sample_counter0_carry__0_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sample_counter0[8:5]),
        .S(m_axis_tuser[8:5]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 sample_counter0_carry__1
       (.CI(sample_counter0_carry__0_n_0),
        .CO({NLW_sample_counter0_carry__1_CO_UNCONNECTED[3],sample_counter0_carry__1_n_1,sample_counter0_carry__1_n_2,sample_counter0_carry__1_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(sample_counter0[12:9]),
        .S(m_axis_tuser[12:9]));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[11]_i_2 
       (.I0(sample_counter0[11]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[11]),
        .O(\sample_counter[11]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[11]_i_3 
       (.I0(sample_counter0[10]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[10]),
        .O(\sample_counter[11]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[11]_i_4 
       (.I0(sample_counter0[9]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[9]),
        .O(\sample_counter[11]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[11]_i_5 
       (.I0(sample_counter0[8]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[8]),
        .O(\sample_counter[11]_i_5_n_0 ));
  LUT3 #(
    .INIT(8'hE0)) 
    \sample_counter[12]_i_1 
       (.I0(start_cntr_reg_n_0),
        .I1(s_axis_tvalid),
        .I2(m_axis_tready),
        .O(\sample_counter[12]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[12]_i_3 
       (.I0(sample_counter0[12]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[12]),
        .O(\sample_counter[12]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hEFFFFFFFFFFFFFFF)) 
    \sample_counter[12]_i_4 
       (.I0(\sample_counter[12]_i_5_n_0 ),
        .I1(\sample_counter[12]_i_6_n_0 ),
        .I2(m_axis_tuser[7]),
        .I3(m_axis_tuser[1]),
        .I4(m_axis_tuser[5]),
        .I5(m_axis_tuser[6]),
        .O(\sample_counter[12]_i_4_n_0 ));
  LUT5 #(
    .INIT(32'hBFFFFFFF)) 
    \sample_counter[12]_i_5 
       (.I0(m_axis_tuser[9]),
        .I1(m_axis_tuser[2]),
        .I2(m_axis_tuser[0]),
        .I3(m_axis_tuser[11]),
        .I4(m_axis_tuser[12]),
        .O(\sample_counter[12]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'hFFF7)) 
    \sample_counter[12]_i_6 
       (.I0(m_axis_tuser[3]),
        .I1(m_axis_tuser[4]),
        .I2(m_axis_tuser[8]),
        .I3(m_axis_tuser[10]),
        .O(\sample_counter[12]_i_6_n_0 ));
  LUT3 #(
    .INIT(8'h38)) 
    \sample_counter[3]_i_2 
       (.I0(\sample_counter[12]_i_4_n_0 ),
        .I1(start_cntr_reg_n_0),
        .I2(m_axis_tuser[0]),
        .O(\sample_counter[3]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[3]_i_3 
       (.I0(sample_counter0[3]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[3]),
        .O(\sample_counter[3]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[3]_i_4 
       (.I0(sample_counter0[2]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[2]),
        .O(\sample_counter[3]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[3]_i_5 
       (.I0(sample_counter0[1]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[1]),
        .O(\sample_counter[3]_i_5_n_0 ));
  LUT3 #(
    .INIT(8'h45)) 
    \sample_counter[3]_i_6 
       (.I0(m_axis_tuser[0]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .O(\sample_counter[3]_i_6_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[7]_i_2 
       (.I0(sample_counter0[7]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[7]),
        .O(\sample_counter[7]_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[7]_i_3 
       (.I0(sample_counter0[6]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[6]),
        .O(\sample_counter[7]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[7]_i_4 
       (.I0(sample_counter0[5]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[5]),
        .O(\sample_counter[7]_i_4_n_0 ));
  LUT4 #(
    .INIT(16'h8F80)) 
    \sample_counter[7]_i_5 
       (.I0(sample_counter0[4]),
        .I1(\sample_counter[12]_i_4_n_0 ),
        .I2(start_cntr_reg_n_0),
        .I3(m_axis_tuser[4]),
        .O(\sample_counter[7]_i_5_n_0 ));
  FDCE \sample_counter_reg[0] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[3]_i_1_n_7 ),
        .Q(m_axis_tuser[0]));
  FDCE \sample_counter_reg[10] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[11]_i_1_n_5 ),
        .Q(m_axis_tuser[10]));
  FDCE \sample_counter_reg[11] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[11]_i_1_n_4 ),
        .Q(m_axis_tuser[11]));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sample_counter_reg[11]_i_1 
       (.CI(\sample_counter_reg[7]_i_1_n_0 ),
        .CO({\sample_counter_reg[11]_i_1_n_0 ,\sample_counter_reg[11]_i_1_n_1 ,\sample_counter_reg[11]_i_1_n_2 ,\sample_counter_reg[11]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\sample_counter_reg[11]_i_1_n_4 ,\sample_counter_reg[11]_i_1_n_5 ,\sample_counter_reg[11]_i_1_n_6 ,\sample_counter_reg[11]_i_1_n_7 }),
        .S({\sample_counter[11]_i_2_n_0 ,\sample_counter[11]_i_3_n_0 ,\sample_counter[11]_i_4_n_0 ,\sample_counter[11]_i_5_n_0 }));
  FDCE \sample_counter_reg[12] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[12]_i_2_n_7 ),
        .Q(m_axis_tuser[12]));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sample_counter_reg[12]_i_2 
       (.CI(\sample_counter_reg[11]_i_1_n_0 ),
        .CO(\NLW_sample_counter_reg[12]_i_2_CO_UNCONNECTED [3:0]),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_sample_counter_reg[12]_i_2_O_UNCONNECTED [3:1],\sample_counter_reg[12]_i_2_n_7 }),
        .S({1'b0,1'b0,1'b0,\sample_counter[12]_i_3_n_0 }));
  FDCE \sample_counter_reg[1] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[3]_i_1_n_6 ),
        .Q(m_axis_tuser[1]));
  FDCE \sample_counter_reg[2] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[3]_i_1_n_5 ),
        .Q(m_axis_tuser[2]));
  FDCE \sample_counter_reg[3] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[3]_i_1_n_4 ),
        .Q(m_axis_tuser[3]));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sample_counter_reg[3]_i_1 
       (.CI(1'b0),
        .CO({\sample_counter_reg[3]_i_1_n_0 ,\sample_counter_reg[3]_i_1_n_1 ,\sample_counter_reg[3]_i_1_n_2 ,\sample_counter_reg[3]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,\sample_counter[3]_i_2_n_0 }),
        .O({\sample_counter_reg[3]_i_1_n_4 ,\sample_counter_reg[3]_i_1_n_5 ,\sample_counter_reg[3]_i_1_n_6 ,\sample_counter_reg[3]_i_1_n_7 }),
        .S({\sample_counter[3]_i_3_n_0 ,\sample_counter[3]_i_4_n_0 ,\sample_counter[3]_i_5_n_0 ,\sample_counter[3]_i_6_n_0 }));
  FDCE \sample_counter_reg[4] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[7]_i_1_n_7 ),
        .Q(m_axis_tuser[4]));
  FDCE \sample_counter_reg[5] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[7]_i_1_n_6 ),
        .Q(m_axis_tuser[5]));
  FDCE \sample_counter_reg[6] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[7]_i_1_n_5 ),
        .Q(m_axis_tuser[6]));
  FDCE \sample_counter_reg[7] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[7]_i_1_n_4 ),
        .Q(m_axis_tuser[7]));
  (* ADDER_THRESHOLD = "11" *) 
  CARRY4 \sample_counter_reg[7]_i_1 
       (.CI(\sample_counter_reg[3]_i_1_n_0 ),
        .CO({\sample_counter_reg[7]_i_1_n_0 ,\sample_counter_reg[7]_i_1_n_1 ,\sample_counter_reg[7]_i_1_n_2 ,\sample_counter_reg[7]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\sample_counter_reg[7]_i_1_n_4 ,\sample_counter_reg[7]_i_1_n_5 ,\sample_counter_reg[7]_i_1_n_6 ,\sample_counter_reg[7]_i_1_n_7 }),
        .S({\sample_counter[7]_i_2_n_0 ,\sample_counter[7]_i_3_n_0 ,\sample_counter[7]_i_4_n_0 ,\sample_counter[7]_i_5_n_0 }));
  FDCE \sample_counter_reg[8] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[11]_i_1_n_7 ),
        .Q(m_axis_tuser[8]));
  FDCE \sample_counter_reg[9] 
       (.C(aclk),
        .CE(\sample_counter[12]_i_1_n_0 ),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(\sample_counter_reg[11]_i_1_n_6 ),
        .Q(m_axis_tuser[9]));
  LUT3 #(
    .INIT(8'hF8)) 
    start_cntr_i_1
       (.I0(m_axis_tready),
        .I1(s_axis_tvalid),
        .I2(start_cntr_reg_n_0),
        .O(start_cntr_i_1_n_0));
  FDCE start_cntr_reg
       (.C(aclk),
        .CE(1'b1),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(start_cntr_i_1_n_0),
        .Q(start_cntr_reg_n_0));
  LUT1 #(
    .INIT(2'h1)) 
    \temp_data[7]_i_1 
       (.I0(aresetn),
        .O(\temp_data[7]_i_1_n_0 ));
  FDCE \temp_data_reg[0] 
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(s_axis_tdata[0]),
        .Q(m_axis_tdata[0]));
  FDCE \temp_data_reg[1] 
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(s_axis_tdata[1]),
        .Q(m_axis_tdata[1]));
  FDCE \temp_data_reg[2] 
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(s_axis_tdata[2]),
        .Q(m_axis_tdata[2]));
  FDCE \temp_data_reg[3] 
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(s_axis_tdata[3]),
        .Q(m_axis_tdata[3]));
  FDCE \temp_data_reg[4] 
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(s_axis_tdata[4]),
        .Q(m_axis_tdata[4]));
  FDCE \temp_data_reg[5] 
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(s_axis_tdata[5]),
        .Q(m_axis_tdata[5]));
  FDCE \temp_data_reg[6] 
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(s_axis_tdata[6]),
        .Q(m_axis_tdata[6]));
  FDCE \temp_data_reg[7] 
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(s_axis_tdata[7]),
        .Q(m_axis_tdata[7]));
  FDCE temp_valid_reg
       (.C(aclk),
        .CE(s_axis_tvalid),
        .CLR(\temp_data[7]_i_1_n_0 ),
        .D(1'b1),
        .Q(m_axis_tvalid));
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
