// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Mon Jan 12 12:01:51 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_2_data_descrambler_0_1_sim_netlist.v
// Design      : design_2_data_descrambler_0_1
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_descrambler
   (dataout1,
    validout,
    clk,
    rst_n,
    validin,
    data_start,
    data_end,
    datain);
  output dataout1;
  output validout;
  input clk;
  input rst_n;
  input validin;
  input data_start;
  input data_end;
  input datain;

  wire \FSM_onehot_cntr_stm_nxt_reg[0]_i_1_n_0 ;
  wire \FSM_onehot_cntr_stm_nxt_reg[1]_i_1_n_0 ;
  wire \FSM_onehot_cntr_stm_nxt_reg[2]_i_1_n_0 ;
  wire \FSM_onehot_cntr_stm_nxt_reg_n_0_[0] ;
  wire \FSM_onehot_cntr_stm_nxt_reg_n_0_[1] ;
  wire \FSM_onehot_cntr_stm_nxt_reg_n_0_[2] ;
  wire \FSM_onehot_cntr_stm_reg_n_0_[0] ;
  wire \FSM_onehot_cntr_stm_reg_n_0_[1] ;
  wire \FSM_onehot_cntr_stm_reg_n_0_[2] ;
  wire \__1/i__n_0 ;
  wire clk;
  wire data_end;
  wire data_start;
  wire datain;
  wire dataout1;
  wire dataout1_i_1_n_0;
  wire [6:0]p_0_in;
  wire p_prev_prev_valid_in;
  wire prev_data_end;
  wire prev_data_in;
  wire prev_data_start;
  wire prev_prev_data_in;
  wire prev_prev_valid_in;
  wire prev_valid_in;
  wire rst_n;
  wire \state[0]_i_1_n_0 ;
  wire \state[1]_i_1_n_0 ;
  wire \state[2]_i_1_n_0 ;
  wire \state[3]_i_1_n_0 ;
  wire \state[4]_i_1_n_0 ;
  wire \state[5]_i_1_n_0 ;
  wire \state[6]_i_1_n_0 ;
  wire \state_reg_n_0_[6] ;
  wire validin;
  wire validout;
  wire validout_i_1_n_0;

  (* XILINX_LEGACY_PRIM = "LDP" *) 
  (* XILINX_TRANSFORM_PINMAP = "VCC:GE" *) 
  LDPE #(
    .INIT(1'b1)) 
    \FSM_onehot_cntr_stm_nxt_reg[0] 
       (.D(\FSM_onehot_cntr_stm_nxt_reg[0]_i_1_n_0 ),
        .G(rst_n),
        .GE(1'b1),
        .PRE(dataout1_i_1_n_0),
        .Q(\FSM_onehot_cntr_stm_nxt_reg_n_0_[0] ));
  LUT5 #(
    .INIT(32'hFFD0D0D0)) 
    \FSM_onehot_cntr_stm_nxt_reg[0]_i_1 
       (.I0(data_start),
        .I1(prev_data_start),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .I3(validin),
        .I4(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .O(\FSM_onehot_cntr_stm_nxt_reg[0]_i_1_n_0 ));
  (* XILINX_LEGACY_PRIM = "LDC" *) 
  (* XILINX_TRANSFORM_PINMAP = "VCC:GE" *) 
  LDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_cntr_stm_nxt_reg[1] 
       (.CLR(dataout1_i_1_n_0),
        .D(\FSM_onehot_cntr_stm_nxt_reg[1]_i_1_n_0 ),
        .G(rst_n),
        .GE(1'b1),
        .Q(\FSM_onehot_cntr_stm_nxt_reg_n_0_[1] ));
  LUT6 #(
    .INIT(64'hFFFF20FF20202020)) 
    \FSM_onehot_cntr_stm_nxt_reg[1]_i_1 
       (.I0(data_start),
        .I1(prev_data_start),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .I3(prev_data_end),
        .I4(data_end),
        .I5(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .O(\FSM_onehot_cntr_stm_nxt_reg[1]_i_1_n_0 ));
  (* XILINX_LEGACY_PRIM = "LDC" *) 
  (* XILINX_TRANSFORM_PINMAP = "VCC:GE" *) 
  LDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_cntr_stm_nxt_reg[2] 
       (.CLR(dataout1_i_1_n_0),
        .D(\FSM_onehot_cntr_stm_nxt_reg[2]_i_1_n_0 ),
        .G(rst_n),
        .GE(1'b1),
        .Q(\FSM_onehot_cntr_stm_nxt_reg_n_0_[2] ));
  LUT5 #(
    .INIT(32'h20FF2020)) 
    \FSM_onehot_cntr_stm_nxt_reg[2]_i_1 
       (.I0(prev_data_end),
        .I1(data_end),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .I3(validin),
        .I4(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .O(\FSM_onehot_cntr_stm_nxt_reg[2]_i_1_n_0 ));
  (* FSM_ENCODED_STATES = "iSTATE:010,iSTATE0:100,iSTATE1:001" *) 
  FDPE #(
    .INIT(1'b1)) 
    \FSM_onehot_cntr_stm_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\FSM_onehot_cntr_stm_nxt_reg_n_0_[0] ),
        .PRE(dataout1_i_1_n_0),
        .Q(\FSM_onehot_cntr_stm_reg_n_0_[0] ));
  (* FSM_ENCODED_STATES = "iSTATE:010,iSTATE0:100,iSTATE1:001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_cntr_stm_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .CLR(dataout1_i_1_n_0),
        .D(\FSM_onehot_cntr_stm_nxt_reg_n_0_[1] ),
        .Q(\FSM_onehot_cntr_stm_reg_n_0_[1] ));
  (* FSM_ENCODED_STATES = "iSTATE:010,iSTATE0:100,iSTATE1:001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_cntr_stm_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .CLR(dataout1_i_1_n_0),
        .D(\FSM_onehot_cntr_stm_nxt_reg_n_0_[2] ),
        .Q(\FSM_onehot_cntr_stm_reg_n_0_[2] ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h28282800)) 
    \__1/i_ 
       (.I0(p_prev_prev_valid_in),
        .I1(\state_reg_n_0_[6] ),
        .I2(p_0_in[0]),
        .I3(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .I4(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .O(\__1/i__n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    dataout1_i_1
       (.I0(rst_n),
        .O(dataout1_i_1_n_0));
  FDCE dataout1_reg
       (.C(clk),
        .CE(1'b1),
        .CLR(dataout1_i_1_n_0),
        .D(\__1/i__n_0 ),
        .Q(dataout1));
  FDRE p_prev_prev_data_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(prev_prev_data_in),
        .Q(p_0_in[0]),
        .R(dataout1_i_1_n_0));
  FDRE p_prev_prev_valid_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(prev_prev_valid_in),
        .Q(p_prev_prev_valid_in),
        .R(dataout1_i_1_n_0));
  FDRE prev_data_end_reg
       (.C(clk),
        .CE(1'b1),
        .D(data_end),
        .Q(prev_data_end),
        .R(dataout1_i_1_n_0));
  FDRE prev_data_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(datain),
        .Q(prev_data_in),
        .R(dataout1_i_1_n_0));
  FDRE prev_data_start_reg
       (.C(clk),
        .CE(1'b1),
        .D(data_start),
        .Q(prev_data_start),
        .R(dataout1_i_1_n_0));
  FDRE prev_prev_data_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(prev_data_in),
        .Q(prev_prev_data_in),
        .R(dataout1_i_1_n_0));
  FDRE prev_prev_valid_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(prev_valid_in),
        .Q(prev_prev_valid_in),
        .R(dataout1_i_1_n_0));
  FDRE prev_valid_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(validin),
        .Q(prev_valid_in),
        .R(dataout1_i_1_n_0));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hAB)) 
    \state[0]_i_1 
       (.I0(p_0_in[0]),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .O(\state[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hE0)) 
    \state[1]_i_1 
       (.I0(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .I2(p_0_in[1]),
        .O(\state[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'hAB)) 
    \state[2]_i_1 
       (.I0(p_0_in[2]),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .O(\state[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hAB)) 
    \state[3]_i_1 
       (.I0(p_0_in[3]),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .O(\state[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hAB)) 
    \state[4]_i_1 
       (.I0(p_0_in[4]),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .O(\state[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hE0)) 
    \state[5]_i_1 
       (.I0(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .I2(p_0_in[5]),
        .O(\state[5]_i_1_n_0 ));
  LUT3 #(
    .INIT(8'hAB)) 
    \state[6]_i_1 
       (.I0(p_0_in[6]),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .O(\state[6]_i_1_n_0 ));
  FDPE \state_reg[0] 
       (.C(clk),
        .CE(p_prev_prev_valid_in),
        .D(\state[0]_i_1_n_0 ),
        .PRE(dataout1_i_1_n_0),
        .Q(p_0_in[1]));
  FDCE \state_reg[1] 
       (.C(clk),
        .CE(p_prev_prev_valid_in),
        .CLR(dataout1_i_1_n_0),
        .D(\state[1]_i_1_n_0 ),
        .Q(p_0_in[2]));
  FDPE \state_reg[2] 
       (.C(clk),
        .CE(p_prev_prev_valid_in),
        .D(\state[2]_i_1_n_0 ),
        .PRE(dataout1_i_1_n_0),
        .Q(p_0_in[3]));
  FDPE \state_reg[3] 
       (.C(clk),
        .CE(p_prev_prev_valid_in),
        .D(\state[3]_i_1_n_0 ),
        .PRE(dataout1_i_1_n_0),
        .Q(p_0_in[4]));
  FDPE \state_reg[4] 
       (.C(clk),
        .CE(p_prev_prev_valid_in),
        .D(\state[4]_i_1_n_0 ),
        .PRE(dataout1_i_1_n_0),
        .Q(p_0_in[5]));
  FDCE \state_reg[5] 
       (.C(clk),
        .CE(p_prev_prev_valid_in),
        .CLR(dataout1_i_1_n_0),
        .D(\state[5]_i_1_n_0 ),
        .Q(p_0_in[6]));
  FDPE \state_reg[6] 
       (.C(clk),
        .CE(p_prev_prev_valid_in),
        .D(\state[6]_i_1_n_0 ),
        .PRE(dataout1_i_1_n_0),
        .Q(\state_reg_n_0_[6] ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT3 #(
    .INIT(8'hA8)) 
    validout_i_1
       (.I0(p_prev_prev_valid_in),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .O(validout_i_1_n_0));
  FDCE validout_reg
       (.C(clk),
        .CE(1'b1),
        .CLR(dataout1_i_1_n_0),
        .D(validout_i_1_n_0),
        .Q(validout));
endmodule

(* CHECK_LICENSE_TYPE = "design_2_data_descrambler_0_1,data_descrambler,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "data_descrambler,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (clk,
    rst_n,
    datain,
    validin,
    data_start,
    data_end,
    dataout1,
    validout);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clk, FREQ_HZ 61440000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, INSERT_VIP 0" *) input clk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input rst_n;
  input datain;
  input validin;
  input data_start;
  input data_end;
  output dataout1;
  output validout;

  wire clk;
  wire data_end;
  wire data_start;
  wire datain;
  wire dataout1;
  wire rst_n;
  wire validin;
  wire validout;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_descrambler inst
       (.clk(clk),
        .data_end(data_end),
        .data_start(data_start),
        .datain(datain),
        .dataout1(dataout1),
        .rst_n(rst_n),
        .validin(validin),
        .validout(validout));
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
