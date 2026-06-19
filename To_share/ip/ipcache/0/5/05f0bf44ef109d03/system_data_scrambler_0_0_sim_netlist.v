// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Wed Jan  7 18:03:57 2026
// Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_data_scrambler_0_0_sim_netlist.v
// Design      : system_data_scrambler_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_scrambler
   (validout,
    dataout,
    clk,
    validin,
    datain,
    preamble_start,
    preamble_end,
    rst_n);
  output validout;
  output [0:0]dataout;
  input clk;
  input validin;
  input datain;
  input preamble_start;
  input preamble_end;
  input rst_n;

  wire \FSM_onehot_cntr_stm[0]_i_1_n_0 ;
  wire \FSM_onehot_cntr_stm[1]_i_1_n_0 ;
  wire \FSM_onehot_cntr_stm[2]_i_1_n_0 ;
  wire \FSM_onehot_cntr_stm_reg_n_0_[0] ;
  wire \FSM_onehot_cntr_stm_reg_n_0_[1] ;
  wire \FSM_onehot_cntr_stm_reg_n_0_[2] ;
  wire clk;
  wire datain;
  wire [0:0]dataout;
  wire \dataout[0]_i_1_n_0 ;
  wire \dataout[0]_i_2_n_0 ;
  wire \dataout[0]_i_3_n_0 ;
  wire preamble_end;
  wire preamble_start;
  wire prev_data_in;
  wire prev_preamble_end;
  wire prev_preamble_start;
  wire prev_prev_data_in;
  wire prev_prev_valid_in;
  wire prev_valid_in;
  wire rst_n;
  wire [5:1]state;
  wire \state[0]_i_1_n_0 ;
  wire \state[2]_i_1_n_0 ;
  wire \state[3]_i_1_n_0 ;
  wire \state[4]_i_1_n_0 ;
  wire \state[6]_i_1_n_0 ;
  wire \state_reg_n_0_[0] ;
  wire \state_reg_n_0_[1] ;
  wire \state_reg_n_0_[2] ;
  wire \state_reg_n_0_[3] ;
  wire \state_reg_n_0_[4] ;
  wire \state_reg_n_0_[5] ;
  wire \state_reg_n_0_[6] ;
  wire validin;
  wire validout;

  LUT5 #(
    .INIT(32'hFFD0D0D0)) 
    \FSM_onehot_cntr_stm[0]_i_1 
       (.I0(prev_preamble_start),
        .I1(preamble_start),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .I3(validin),
        .I4(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .O(\FSM_onehot_cntr_stm[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFF20FF20202020)) 
    \FSM_onehot_cntr_stm[1]_i_1 
       (.I0(prev_preamble_start),
        .I1(preamble_start),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .I3(prev_preamble_end),
        .I4(preamble_end),
        .I5(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .O(\FSM_onehot_cntr_stm[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h20FF2020)) 
    \FSM_onehot_cntr_stm[2]_i_1 
       (.I0(prev_preamble_end),
        .I1(preamble_end),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .I3(validin),
        .I4(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .O(\FSM_onehot_cntr_stm[2]_i_1_n_0 ));
  (* FSM_ENCODED_STATES = "iSTATE:010,iSTATE0:100,iSTATE1:001" *) 
  FDPE #(
    .INIT(1'b1)) 
    \FSM_onehot_cntr_stm_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\FSM_onehot_cntr_stm[0]_i_1_n_0 ),
        .PRE(\dataout[0]_i_2_n_0 ),
        .Q(\FSM_onehot_cntr_stm_reg_n_0_[0] ));
  (* FSM_ENCODED_STATES = "iSTATE:010,iSTATE0:100,iSTATE1:001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_cntr_stm_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\dataout[0]_i_2_n_0 ),
        .D(\FSM_onehot_cntr_stm[1]_i_1_n_0 ),
        .Q(\FSM_onehot_cntr_stm_reg_n_0_[1] ));
  (* FSM_ENCODED_STATES = "iSTATE:010,iSTATE0:100,iSTATE1:001" *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_cntr_stm_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\dataout[0]_i_2_n_0 ),
        .D(\FSM_onehot_cntr_stm[2]_i_1_n_0 ),
        .Q(\FSM_onehot_cntr_stm_reg_n_0_[2] ));
  LUT6 #(
    .INIT(64'hAA3CAAFFAA3CAA00)) 
    \dataout[0]_i_1 
       (.I0(prev_prev_data_in),
        .I1(\state_reg_n_0_[6] ),
        .I2(prev_data_in),
        .I3(\dataout[0]_i_3_n_0 ),
        .I4(prev_prev_valid_in),
        .I5(dataout),
        .O(\dataout[0]_i_1_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \dataout[0]_i_2 
       (.I0(rst_n),
        .O(\dataout[0]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \dataout[0]_i_3 
       (.I0(\FSM_onehot_cntr_stm_reg_n_0_[1] ),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[2] ),
        .O(\dataout[0]_i_3_n_0 ));
  FDCE \dataout_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\dataout[0]_i_2_n_0 ),
        .D(\dataout[0]_i_1_n_0 ),
        .Q(dataout));
  FDRE prev_data_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(datain),
        .Q(prev_data_in),
        .R(\dataout[0]_i_2_n_0 ));
  FDRE prev_preamble_end_reg
       (.C(clk),
        .CE(1'b1),
        .D(preamble_end),
        .Q(prev_preamble_end),
        .R(\dataout[0]_i_2_n_0 ));
  FDRE prev_preamble_start_reg
       (.C(clk),
        .CE(1'b1),
        .D(preamble_start),
        .Q(prev_preamble_start),
        .R(\dataout[0]_i_2_n_0 ));
  FDRE prev_prev_data_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(prev_data_in),
        .Q(prev_prev_data_in),
        .R(\dataout[0]_i_2_n_0 ));
  FDRE prev_prev_valid_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(prev_valid_in),
        .Q(prev_prev_valid_in),
        .R(\dataout[0]_i_2_n_0 ));
  FDRE prev_valid_in_reg
       (.C(clk),
        .CE(1'b1),
        .D(validin),
        .Q(prev_valid_in),
        .R(\dataout[0]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'h6F)) 
    \state[0]_i_1 
       (.I0(prev_data_in),
        .I1(\state_reg_n_0_[6] ),
        .I2(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .O(\state[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \state[1]_i_1 
       (.I0(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .I1(\state_reg_n_0_[0] ),
        .O(state[1]));
  LUT2 #(
    .INIT(4'hB)) 
    \state[2]_i_1 
       (.I0(\state_reg_n_0_[1] ),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .O(\state[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT2 #(
    .INIT(4'hB)) 
    \state[3]_i_1 
       (.I0(\state_reg_n_0_[2] ),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .O(\state[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT2 #(
    .INIT(4'hB)) 
    \state[4]_i_1 
       (.I0(\state_reg_n_0_[3] ),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .O(\state[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \state[5]_i_1 
       (.I0(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .I1(\state_reg_n_0_[4] ),
        .O(state[5]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT2 #(
    .INIT(4'hB)) 
    \state[6]_i_1 
       (.I0(\state_reg_n_0_[5] ),
        .I1(\FSM_onehot_cntr_stm_reg_n_0_[0] ),
        .O(\state[6]_i_1_n_0 ));
  FDPE \state_reg[0] 
       (.C(clk),
        .CE(prev_prev_valid_in),
        .D(\state[0]_i_1_n_0 ),
        .PRE(\dataout[0]_i_2_n_0 ),
        .Q(\state_reg_n_0_[0] ));
  FDCE \state_reg[1] 
       (.C(clk),
        .CE(prev_prev_valid_in),
        .CLR(\dataout[0]_i_2_n_0 ),
        .D(state[1]),
        .Q(\state_reg_n_0_[1] ));
  FDPE \state_reg[2] 
       (.C(clk),
        .CE(prev_prev_valid_in),
        .D(\state[2]_i_1_n_0 ),
        .PRE(\dataout[0]_i_2_n_0 ),
        .Q(\state_reg_n_0_[2] ));
  FDPE \state_reg[3] 
       (.C(clk),
        .CE(prev_prev_valid_in),
        .D(\state[3]_i_1_n_0 ),
        .PRE(\dataout[0]_i_2_n_0 ),
        .Q(\state_reg_n_0_[3] ));
  FDPE \state_reg[4] 
       (.C(clk),
        .CE(prev_prev_valid_in),
        .D(\state[4]_i_1_n_0 ),
        .PRE(\dataout[0]_i_2_n_0 ),
        .Q(\state_reg_n_0_[4] ));
  FDCE \state_reg[5] 
       (.C(clk),
        .CE(prev_prev_valid_in),
        .CLR(\dataout[0]_i_2_n_0 ),
        .D(state[5]),
        .Q(\state_reg_n_0_[5] ));
  FDPE \state_reg[6] 
       (.C(clk),
        .CE(prev_prev_valid_in),
        .D(\state[6]_i_1_n_0 ),
        .PRE(\dataout[0]_i_2_n_0 ),
        .Q(\state_reg_n_0_[6] ));
  FDCE validout_reg
       (.C(clk),
        .CE(1'b1),
        .CLR(\dataout[0]_i_2_n_0 ),
        .D(prev_prev_valid_in),
        .Q(validout));
endmodule

(* CHECK_LICENSE_TYPE = "system_data_scrambler_0_0,data_scrambler,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "data_scrambler,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (clk,
    rst_n,
    datain,
    validin,
    preamble_start,
    preamble_end,
    dataout,
    validout);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clk, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input clk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input rst_n;
  input datain;
  input validin;
  input preamble_start;
  input preamble_end;
  output [7:0]dataout;
  output validout;

  wire \<const0> ;
  wire clk;
  wire datain;
  wire [0:0]\^dataout ;
  wire preamble_end;
  wire preamble_start;
  wire rst_n;
  wire validin;
  wire validout;

  assign dataout[7] = \<const0> ;
  assign dataout[6] = \<const0> ;
  assign dataout[5] = \<const0> ;
  assign dataout[4] = \<const0> ;
  assign dataout[3] = \<const0> ;
  assign dataout[2] = \<const0> ;
  assign dataout[1] = \<const0> ;
  assign dataout[0] = \^dataout [0];
  GND GND
       (.G(\<const0> ));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_scrambler inst
       (.clk(clk),
        .datain(datain),
        .dataout(\^dataout ),
        .preamble_end(preamble_end),
        .preamble_start(preamble_start),
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
