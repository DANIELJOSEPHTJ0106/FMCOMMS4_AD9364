// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Mon Jan  5 13:22:36 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_bit_inverter_0_0/system_bit_inverter_0_0_sim_netlist.v
// Design      : system_bit_inverter_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "system_bit_inverter_0_0,bit_inverter,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "bit_inverter,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module system_bit_inverter_0_0
   (clk,
    rstn,
    data_in,
    valid_in,
    data_out,
    valid_out);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clk, ASSOCIATED_RESET rstn, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_sys_ps7_0_FCLK_CLK0, INSERT_VIP 0" *) input clk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rstn RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME rstn, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input rstn;
  input [7:0]data_in;
  input valid_in;
  output [7:0]data_out;
  output valid_out;

  wire clk;
  wire [7:0]data_in;
  wire [7:0]data_out;
  wire rstn;
  wire valid_in;
  wire valid_out;

  system_bit_inverter_0_0_bit_inverter inst
       (.clk(clk),
        .data_in(data_in),
        .data_out(data_out),
        .rstn(rstn),
        .valid_in(valid_in),
        .valid_out(valid_out));
endmodule

(* ORIG_REF_NAME = "bit_inverter" *) 
module system_bit_inverter_0_0_bit_inverter
   (data_out,
    valid_out,
    data_in,
    clk,
    valid_in,
    rstn);
  output [7:0]data_out;
  output valid_out;
  input [7:0]data_in;
  input clk;
  input valid_in;
  input rstn;

  wire clk;
  wire [7:0]data_in;
  wire [7:0]data_out;
  wire \data_reg[7]_i_2_n_0 ;
  wire [7:0]p_0_in;
  wire rstn;
  wire valid_in;
  wire valid_out;

  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[0]_i_1 
       (.I0(data_in[0]),
        .O(p_0_in[0]));
  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[1]_i_1 
       (.I0(data_in[1]),
        .O(p_0_in[1]));
  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[2]_i_1 
       (.I0(data_in[2]),
        .O(p_0_in[2]));
  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[3]_i_1 
       (.I0(data_in[3]),
        .O(p_0_in[3]));
  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[4]_i_1 
       (.I0(data_in[4]),
        .O(p_0_in[4]));
  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[5]_i_1 
       (.I0(data_in[5]),
        .O(p_0_in[5]));
  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[6]_i_1 
       (.I0(data_in[6]),
        .O(p_0_in[6]));
  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[7]_i_1 
       (.I0(data_in[7]),
        .O(p_0_in[7]));
  LUT1 #(
    .INIT(2'h1)) 
    \data_reg[7]_i_2 
       (.I0(rstn),
        .O(\data_reg[7]_i_2_n_0 ));
  FDCE \data_reg_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(p_0_in[0]),
        .Q(data_out[0]));
  FDCE \data_reg_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(p_0_in[1]),
        .Q(data_out[1]));
  FDCE \data_reg_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(p_0_in[2]),
        .Q(data_out[2]));
  FDCE \data_reg_reg[3] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(p_0_in[3]),
        .Q(data_out[3]));
  FDCE \data_reg_reg[4] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(p_0_in[4]),
        .Q(data_out[4]));
  FDCE \data_reg_reg[5] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(p_0_in[5]),
        .Q(data_out[5]));
  FDCE \data_reg_reg[6] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(p_0_in[6]),
        .Q(data_out[6]));
  FDCE \data_reg_reg[7] 
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(p_0_in[7]),
        .Q(data_out[7]));
  FDCE valid_reg_reg
       (.C(clk),
        .CE(1'b1),
        .CLR(\data_reg[7]_i_2_n_0 ),
        .D(valid_in),
        .Q(valid_out));
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
