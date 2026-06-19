// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
// Date        : Sat Jan  3 12:25:20 2026
// Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
// Command     : write_verilog -force -mode funcsim
//               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_ila_display_0_0/system_ila_display_0_0_sim_netlist.v
// Design      : system_ila_display_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg484-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "system_ila_display_0_0,ila_display,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* IP_DEFINITION_SOURCE = "module_ref" *) 
(* X_CORE_INFO = "ila_display,Vivado 2023.1" *) 
(* NotValidForBitStream *)
module system_ila_display_0_0
   (clock_61,
    clock_100,
    reset_n,
    pattern_gen_dat_out,
    pattern_gen_val_out,
    bsync_dat,
    bsync_val,
    ila_pattern_gen_dat,
    ila_pattern_gen_val,
    ila_bsync_dat,
    ila_bsync_val);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clock_61 CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clock_61, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0" *) input clock_61;
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clock_100 CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clock_100, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_sys_ps7_0_FCLK_CLK0, INSERT_VIP 0" *) input clock_100;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 reset_n RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input reset_n;
  input [7:0]pattern_gen_dat_out;
  input pattern_gen_val_out;
  input [7:0]bsync_dat;
  input bsync_val;
  output [7:0]ila_pattern_gen_dat;
  output ila_pattern_gen_val;
  output [7:0]ila_bsync_dat;
  output ila_bsync_val;

  wire [7:0]bsync_dat;
  wire bsync_val;
  wire clock_100;
  wire clock_61;
  wire [7:0]ila_bsync_dat;
  wire ila_bsync_val;
  wire [7:0]ila_pattern_gen_dat;
  wire ila_pattern_gen_val;
  wire [7:0]pattern_gen_dat_out;
  wire pattern_gen_val_out;
  wire reset_n;

  (* FIFO_DEPTH = "16'b0000001000000000" *) 
  system_ila_display_0_0_ila_display inst
       (.bsync_dat(bsync_dat),
        .bsync_val(bsync_val),
        .clock_100(clock_100),
        .clock_61(clock_61),
        .ila_bsync_dat(ila_bsync_dat),
        .ila_bsync_val(ila_bsync_val),
        .ila_pattern_gen_dat(ila_pattern_gen_dat),
        .ila_pattern_gen_val(ila_pattern_gen_val),
        .pattern_gen_dat_out(pattern_gen_dat_out),
        .pattern_gen_val_out(pattern_gen_val_out),
        .reset_n(reset_n));
endmodule

(* ORIG_REF_NAME = "fifo_generator_0" *) (* X_CORE_INFO = "fifo_generator_v13_2_8,Vivado 2023.1" *) 
module system_ila_display_0_0_fifo_generator_0
   (rst,
    wr_clk,
    rd_clk,
    din,
    wr_en,
    rd_en,
    dout,
    full,
    empty,
    wr_rst_busy,
    rd_rst_busy);
  input rst;
  (* syn_isclock = "1" *) input wr_clk;
  (* syn_isclock = "1" *) input rd_clk;
  input [31:0]din;
  input wr_en;
  input rd_en;
  output [31:0]dout;
  output full;
  output empty;
  output wr_rst_busy;
  output rd_rst_busy;


endmodule

(* ORIG_REF_NAME = "fifo_generator_0" *) (* X_CORE_INFO = "fifo_generator_v13_2_8,Vivado 2023.1" *) 
module system_ila_display_0_0_fifo_generator_0_HD1
   (empty,
    full,
    rd_clk,
    rd_en,
    rd_rst_busy,
    rst,
    wr_clk,
    wr_en,
    wr_rst_busy,
    din,
    dout);
  output empty;
  output full;
  (* syn_isclock = "1" *) input rd_clk;
  input rd_en;
  output rd_rst_busy;
  input rst;
  (* syn_isclock = "1" *) input wr_clk;
  input wr_en;
  output wr_rst_busy;
  input [31:0]din;
  output [31:0]dout;


endmodule

(* FIFO_DEPTH = "16'b0000001000000000" *) (* ORIG_REF_NAME = "ila_display" *) 
module system_ila_display_0_0_ila_display
   (clock_61,
    clock_100,
    reset_n,
    pattern_gen_dat_out,
    pattern_gen_val_out,
    bsync_dat,
    bsync_val,
    ila_pattern_gen_dat,
    ila_pattern_gen_val,
    ila_bsync_dat,
    ila_bsync_val);
  input clock_61;
  input clock_100;
  input reset_n;
  input [7:0]pattern_gen_dat_out;
  input pattern_gen_val_out;
  input [7:0]bsync_dat;
  input bsync_val;
  output [7:0]ila_pattern_gen_dat;
  output ila_pattern_gen_val;
  output [7:0]ila_bsync_dat;
  output ila_bsync_val;

  wire \FSM_onehot_acc_state1[0]_i_1_n_0 ;
  wire \FSM_onehot_acc_state1[0]_i_2_n_0 ;
  wire \FSM_onehot_acc_state1[1]_i_1_n_0 ;
  wire \FSM_onehot_acc_state1[1]_i_2_n_0 ;
  wire \FSM_onehot_acc_state1[1]_i_3_n_0 ;
  wire \FSM_onehot_acc_state1[1]_i_4_n_0 ;
  wire \FSM_onehot_acc_state1[2]_i_1_n_0 ;
  wire \FSM_onehot_acc_state1[2]_i_2_n_0 ;
  wire \FSM_onehot_acc_state1[2]_i_3_n_0 ;
  wire \FSM_onehot_acc_state1_reg_n_0_[0] ;
  wire \FSM_onehot_acc_state1_reg_n_0_[1] ;
  wire \FSM_onehot_acc_state1_reg_n_0_[2] ;
  wire \FSM_onehot_acc_state2[0]_i_1_n_0 ;
  wire \FSM_onehot_acc_state2[0]_i_2_n_0 ;
  wire \FSM_onehot_acc_state2[1]_i_1_n_0 ;
  wire \FSM_onehot_acc_state2[1]_i_2_n_0 ;
  wire \FSM_onehot_acc_state2[1]_i_3_n_0 ;
  wire \FSM_onehot_acc_state2[1]_i_4_n_0 ;
  wire \FSM_onehot_acc_state2[2]_i_1_n_0 ;
  wire \FSM_onehot_acc_state2[2]_i_2_n_0 ;
  wire \FSM_onehot_acc_state2[2]_i_3_n_0 ;
  wire \FSM_onehot_acc_state2_reg_n_0_[0] ;
  wire \FSM_onehot_acc_state2_reg_n_0_[1] ;
  wire \FSM_onehot_acc_state2_reg_n_0_[2] ;
  wire \FSM_sequential_push_stm1[0]_inv_i_1_n_0 ;
  wire \FSM_sequential_push_stm1[0]_inv_i_2_n_0 ;
  wire \FSM_sequential_push_stm1[0]_inv_i_3_n_0 ;
  wire \FSM_sequential_push_stm1[0]_inv_i_4_n_0 ;
  wire \FSM_sequential_push_stm1[1]_i_1_n_0 ;
  wire \FSM_sequential_push_stm1[1]_i_2_n_0 ;
  wire \FSM_sequential_push_stm1[1]_i_3_n_0 ;
  wire \FSM_sequential_push_stm2[0]_inv_i_1_n_0 ;
  wire \FSM_sequential_push_stm2[0]_inv_i_2_n_0 ;
  wire \FSM_sequential_push_stm2[0]_inv_i_3_n_0 ;
  wire \FSM_sequential_push_stm2[0]_inv_i_4_n_0 ;
  wire \FSM_sequential_push_stm2[1]_i_1_n_0 ;
  wire \FSM_sequential_push_stm2[1]_i_2_n_0 ;
  wire \FSM_sequential_push_stm2[1]_i_3_n_0 ;
  wire bit_sync_cntr;
  wire \bit_sync_cntr[0]_i_1_n_0 ;
  wire \bit_sync_cntr[10]_i_1_n_0 ;
  wire \bit_sync_cntr[11]_i_1_n_0 ;
  wire \bit_sync_cntr[12]_i_1_n_0 ;
  wire \bit_sync_cntr[13]_i_1_n_0 ;
  wire \bit_sync_cntr[14]_i_1_n_0 ;
  wire \bit_sync_cntr[15]_i_2_n_0 ;
  wire \bit_sync_cntr[1]_i_1_n_0 ;
  wire \bit_sync_cntr[2]_i_1_n_0 ;
  wire \bit_sync_cntr[3]_i_1_n_0 ;
  wire \bit_sync_cntr[4]_i_1_n_0 ;
  wire \bit_sync_cntr[5]_i_1_n_0 ;
  wire \bit_sync_cntr[6]_i_1_n_0 ;
  wire \bit_sync_cntr[7]_i_1_n_0 ;
  wire \bit_sync_cntr[8]_i_1_n_0 ;
  wire \bit_sync_cntr[9]_i_1_n_0 ;
  wire \bit_sync_cntr_reg[12]_i_2_n_0 ;
  wire \bit_sync_cntr_reg[12]_i_2_n_1 ;
  wire \bit_sync_cntr_reg[12]_i_2_n_2 ;
  wire \bit_sync_cntr_reg[12]_i_2_n_3 ;
  wire \bit_sync_cntr_reg[12]_i_2_n_4 ;
  wire \bit_sync_cntr_reg[12]_i_2_n_5 ;
  wire \bit_sync_cntr_reg[12]_i_2_n_6 ;
  wire \bit_sync_cntr_reg[12]_i_2_n_7 ;
  wire \bit_sync_cntr_reg[15]_i_3_n_2 ;
  wire \bit_sync_cntr_reg[15]_i_3_n_3 ;
  wire \bit_sync_cntr_reg[15]_i_3_n_5 ;
  wire \bit_sync_cntr_reg[15]_i_3_n_6 ;
  wire \bit_sync_cntr_reg[15]_i_3_n_7 ;
  wire \bit_sync_cntr_reg[4]_i_2_n_0 ;
  wire \bit_sync_cntr_reg[4]_i_2_n_1 ;
  wire \bit_sync_cntr_reg[4]_i_2_n_2 ;
  wire \bit_sync_cntr_reg[4]_i_2_n_3 ;
  wire \bit_sync_cntr_reg[4]_i_2_n_4 ;
  wire \bit_sync_cntr_reg[4]_i_2_n_5 ;
  wire \bit_sync_cntr_reg[4]_i_2_n_6 ;
  wire \bit_sync_cntr_reg[4]_i_2_n_7 ;
  wire \bit_sync_cntr_reg[8]_i_2_n_0 ;
  wire \bit_sync_cntr_reg[8]_i_2_n_1 ;
  wire \bit_sync_cntr_reg[8]_i_2_n_2 ;
  wire \bit_sync_cntr_reg[8]_i_2_n_3 ;
  wire \bit_sync_cntr_reg[8]_i_2_n_4 ;
  wire \bit_sync_cntr_reg[8]_i_2_n_5 ;
  wire \bit_sync_cntr_reg[8]_i_2_n_6 ;
  wire \bit_sync_cntr_reg[8]_i_2_n_7 ;
  wire \bit_sync_cntr_reg_n_0_[0] ;
  wire \bit_sync_cntr_reg_n_0_[10] ;
  wire \bit_sync_cntr_reg_n_0_[11] ;
  wire \bit_sync_cntr_reg_n_0_[12] ;
  wire \bit_sync_cntr_reg_n_0_[13] ;
  wire \bit_sync_cntr_reg_n_0_[14] ;
  wire \bit_sync_cntr_reg_n_0_[15] ;
  wire \bit_sync_cntr_reg_n_0_[1] ;
  wire \bit_sync_cntr_reg_n_0_[2] ;
  wire \bit_sync_cntr_reg_n_0_[3] ;
  wire \bit_sync_cntr_reg_n_0_[4] ;
  wire \bit_sync_cntr_reg_n_0_[5] ;
  wire \bit_sync_cntr_reg_n_0_[6] ;
  wire \bit_sync_cntr_reg_n_0_[7] ;
  wire \bit_sync_cntr_reg_n_0_[8] ;
  wire \bit_sync_cntr_reg_n_0_[9] ;
  wire [7:0]bsync_dat;
  wire bsync_val;
  wire clock_100;
  wire clock_61;
  wire [15:1]data0;
  wire fifo_din1;
  wire \fifo_din1_reg_n_0_[0] ;
  wire \fifo_din1_reg_n_0_[1] ;
  wire \fifo_din1_reg_n_0_[2] ;
  wire \fifo_din1_reg_n_0_[3] ;
  wire \fifo_din1_reg_n_0_[4] ;
  wire \fifo_din1_reg_n_0_[5] ;
  wire \fifo_din1_reg_n_0_[6] ;
  wire \fifo_din1_reg_n_0_[7] ;
  wire fifo_din2;
  wire \fifo_din2_reg_n_0_[0] ;
  wire \fifo_din2_reg_n_0_[1] ;
  wire \fifo_din2_reg_n_0_[2] ;
  wire \fifo_din2_reg_n_0_[3] ;
  wire \fifo_din2_reg_n_0_[4] ;
  wire \fifo_din2_reg_n_0_[5] ;
  wire \fifo_din2_reg_n_0_[6] ;
  wire \fifo_din2_reg_n_0_[7] ;
  wire [7:0]fifo_dout1;
  wire [7:0]fifo_dout2;
  wire fifo_rd_en1_i_1_n_0;
  wire fifo_rd_en1_reg_n_0;
  wire fifo_rd_en2_i_1_n_0;
  wire fifo_rd_en2_reg_n_0;
  wire fifo_wr_en1_reg_n_0;
  wire fifo_wr_en2_reg_n_0;
  wire \get_data1[0]_i_1_n_0 ;
  wire \get_data1[1]_i_1_n_0 ;
  wire \get_data1[2]_i_1_n_0 ;
  wire \get_data1[3]_i_1_n_0 ;
  wire \get_data1[4]_i_1_n_0 ;
  wire \get_data1[5]_i_1_n_0 ;
  wire \get_data1[6]_i_1_n_0 ;
  wire \get_data1[7]_i_1_n_0 ;
  wire \get_data1[7]_i_2_n_0 ;
  wire \get_data2[0]_i_1_n_0 ;
  wire \get_data2[1]_i_1_n_0 ;
  wire \get_data2[2]_i_1_n_0 ;
  wire \get_data2[3]_i_1_n_0 ;
  wire \get_data2[4]_i_1_n_0 ;
  wire \get_data2[5]_i_1_n_0 ;
  wire \get_data2[6]_i_1_n_0 ;
  wire \get_data2[7]_i_1_n_0 ;
  wire [7:0]ila_bsync_dat;
  wire ila_bsync_val;
  wire [7:0]ila_pattern_gen_dat;
  wire ila_pattern_gen_val;
  wire [15:1]in4;
  wire pattern_cntr;
  wire \pattern_cntr1[0]_i_1_n_0 ;
  wire \pattern_cntr1[10]_i_1_n_0 ;
  wire \pattern_cntr1[11]_i_1_n_0 ;
  wire \pattern_cntr1[12]_i_1_n_0 ;
  wire \pattern_cntr1[13]_i_1_n_0 ;
  wire \pattern_cntr1[14]_i_1_n_0 ;
  wire \pattern_cntr1[15]_i_1_n_0 ;
  wire \pattern_cntr1[15]_i_2_n_0 ;
  wire \pattern_cntr1[15]_i_3_n_0 ;
  wire \pattern_cntr1[15]_i_4_n_0 ;
  wire \pattern_cntr1[15]_i_5_n_0 ;
  wire \pattern_cntr1[15]_i_6_n_0 ;
  wire \pattern_cntr1[1]_i_1_n_0 ;
  wire \pattern_cntr1[2]_i_1_n_0 ;
  wire \pattern_cntr1[3]_i_1_n_0 ;
  wire \pattern_cntr1[4]_i_1_n_0 ;
  wire \pattern_cntr1[5]_i_1_n_0 ;
  wire \pattern_cntr1[6]_i_1_n_0 ;
  wire \pattern_cntr1[7]_i_1_n_0 ;
  wire \pattern_cntr1[8]_i_1_n_0 ;
  wire \pattern_cntr1[9]_i_1_n_0 ;
  wire \pattern_cntr1_reg[12]_i_2_n_0 ;
  wire \pattern_cntr1_reg[12]_i_2_n_1 ;
  wire \pattern_cntr1_reg[12]_i_2_n_2 ;
  wire \pattern_cntr1_reg[12]_i_2_n_3 ;
  wire \pattern_cntr1_reg[12]_i_2_n_4 ;
  wire \pattern_cntr1_reg[12]_i_2_n_5 ;
  wire \pattern_cntr1_reg[12]_i_2_n_6 ;
  wire \pattern_cntr1_reg[12]_i_2_n_7 ;
  wire \pattern_cntr1_reg[15]_i_7_n_2 ;
  wire \pattern_cntr1_reg[15]_i_7_n_3 ;
  wire \pattern_cntr1_reg[15]_i_7_n_5 ;
  wire \pattern_cntr1_reg[15]_i_7_n_6 ;
  wire \pattern_cntr1_reg[15]_i_7_n_7 ;
  wire \pattern_cntr1_reg[4]_i_2_n_0 ;
  wire \pattern_cntr1_reg[4]_i_2_n_1 ;
  wire \pattern_cntr1_reg[4]_i_2_n_2 ;
  wire \pattern_cntr1_reg[4]_i_2_n_3 ;
  wire \pattern_cntr1_reg[4]_i_2_n_4 ;
  wire \pattern_cntr1_reg[4]_i_2_n_5 ;
  wire \pattern_cntr1_reg[4]_i_2_n_6 ;
  wire \pattern_cntr1_reg[4]_i_2_n_7 ;
  wire \pattern_cntr1_reg[8]_i_2_n_0 ;
  wire \pattern_cntr1_reg[8]_i_2_n_1 ;
  wire \pattern_cntr1_reg[8]_i_2_n_2 ;
  wire \pattern_cntr1_reg[8]_i_2_n_3 ;
  wire \pattern_cntr1_reg[8]_i_2_n_4 ;
  wire \pattern_cntr1_reg[8]_i_2_n_5 ;
  wire \pattern_cntr1_reg[8]_i_2_n_6 ;
  wire \pattern_cntr1_reg[8]_i_2_n_7 ;
  wire \pattern_cntr1_reg_n_0_[0] ;
  wire \pattern_cntr1_reg_n_0_[10] ;
  wire \pattern_cntr1_reg_n_0_[11] ;
  wire \pattern_cntr1_reg_n_0_[12] ;
  wire \pattern_cntr1_reg_n_0_[13] ;
  wire \pattern_cntr1_reg_n_0_[14] ;
  wire \pattern_cntr1_reg_n_0_[15] ;
  wire \pattern_cntr1_reg_n_0_[1] ;
  wire \pattern_cntr1_reg_n_0_[2] ;
  wire \pattern_cntr1_reg_n_0_[3] ;
  wire \pattern_cntr1_reg_n_0_[4] ;
  wire \pattern_cntr1_reg_n_0_[5] ;
  wire \pattern_cntr1_reg_n_0_[6] ;
  wire \pattern_cntr1_reg_n_0_[7] ;
  wire \pattern_cntr1_reg_n_0_[8] ;
  wire \pattern_cntr1_reg_n_0_[9] ;
  wire \pattern_cntr2[0]_i_1_n_0 ;
  wire \pattern_cntr2[10]_i_1_n_0 ;
  wire \pattern_cntr2[11]_i_1_n_0 ;
  wire \pattern_cntr2[12]_i_1_n_0 ;
  wire \pattern_cntr2[13]_i_1_n_0 ;
  wire \pattern_cntr2[14]_i_1_n_0 ;
  wire \pattern_cntr2[15]_i_1_n_0 ;
  wire \pattern_cntr2[15]_i_2_n_0 ;
  wire \pattern_cntr2[15]_i_3_n_0 ;
  wire \pattern_cntr2[15]_i_4_n_0 ;
  wire \pattern_cntr2[15]_i_5_n_0 ;
  wire \pattern_cntr2[15]_i_6_n_0 ;
  wire \pattern_cntr2[1]_i_1_n_0 ;
  wire \pattern_cntr2[2]_i_1_n_0 ;
  wire \pattern_cntr2[3]_i_1_n_0 ;
  wire \pattern_cntr2[4]_i_1_n_0 ;
  wire \pattern_cntr2[5]_i_1_n_0 ;
  wire \pattern_cntr2[6]_i_1_n_0 ;
  wire \pattern_cntr2[7]_i_1_n_0 ;
  wire \pattern_cntr2[8]_i_1_n_0 ;
  wire \pattern_cntr2[9]_i_1_n_0 ;
  wire \pattern_cntr2_reg[12]_i_2_n_0 ;
  wire \pattern_cntr2_reg[12]_i_2_n_1 ;
  wire \pattern_cntr2_reg[12]_i_2_n_2 ;
  wire \pattern_cntr2_reg[12]_i_2_n_3 ;
  wire \pattern_cntr2_reg[15]_i_7_n_2 ;
  wire \pattern_cntr2_reg[15]_i_7_n_3 ;
  wire \pattern_cntr2_reg[4]_i_2_n_0 ;
  wire \pattern_cntr2_reg[4]_i_2_n_1 ;
  wire \pattern_cntr2_reg[4]_i_2_n_2 ;
  wire \pattern_cntr2_reg[4]_i_2_n_3 ;
  wire \pattern_cntr2_reg[8]_i_2_n_0 ;
  wire \pattern_cntr2_reg[8]_i_2_n_1 ;
  wire \pattern_cntr2_reg[8]_i_2_n_2 ;
  wire \pattern_cntr2_reg[8]_i_2_n_3 ;
  wire \pattern_cntr2_reg_n_0_[0] ;
  wire \pattern_cntr2_reg_n_0_[10] ;
  wire \pattern_cntr2_reg_n_0_[11] ;
  wire \pattern_cntr2_reg_n_0_[12] ;
  wire \pattern_cntr2_reg_n_0_[13] ;
  wire \pattern_cntr2_reg_n_0_[14] ;
  wire \pattern_cntr2_reg_n_0_[15] ;
  wire \pattern_cntr2_reg_n_0_[1] ;
  wire \pattern_cntr2_reg_n_0_[2] ;
  wire \pattern_cntr2_reg_n_0_[3] ;
  wire \pattern_cntr2_reg_n_0_[4] ;
  wire \pattern_cntr2_reg_n_0_[5] ;
  wire \pattern_cntr2_reg_n_0_[6] ;
  wire \pattern_cntr2_reg_n_0_[7] ;
  wire \pattern_cntr2_reg_n_0_[8] ;
  wire \pattern_cntr2_reg_n_0_[9] ;
  wire \pattern_cntr[0]_i_1_n_0 ;
  wire \pattern_cntr[10]_i_1_n_0 ;
  wire \pattern_cntr[11]_i_1_n_0 ;
  wire \pattern_cntr[12]_i_1_n_0 ;
  wire \pattern_cntr[13]_i_1_n_0 ;
  wire \pattern_cntr[14]_i_1_n_0 ;
  wire \pattern_cntr[15]_i_2_n_0 ;
  wire \pattern_cntr[1]_i_1_n_0 ;
  wire \pattern_cntr[2]_i_1_n_0 ;
  wire \pattern_cntr[3]_i_1_n_0 ;
  wire \pattern_cntr[4]_i_1_n_0 ;
  wire \pattern_cntr[5]_i_1_n_0 ;
  wire \pattern_cntr[6]_i_1_n_0 ;
  wire \pattern_cntr[7]_i_1_n_0 ;
  wire \pattern_cntr[8]_i_1_n_0 ;
  wire \pattern_cntr[9]_i_1_n_0 ;
  wire \pattern_cntr_reg[12]_i_2_n_0 ;
  wire \pattern_cntr_reg[12]_i_2_n_1 ;
  wire \pattern_cntr_reg[12]_i_2_n_2 ;
  wire \pattern_cntr_reg[12]_i_2_n_3 ;
  wire \pattern_cntr_reg[15]_i_3_n_2 ;
  wire \pattern_cntr_reg[15]_i_3_n_3 ;
  wire \pattern_cntr_reg[4]_i_2_n_0 ;
  wire \pattern_cntr_reg[4]_i_2_n_1 ;
  wire \pattern_cntr_reg[4]_i_2_n_2 ;
  wire \pattern_cntr_reg[4]_i_2_n_3 ;
  wire \pattern_cntr_reg[8]_i_2_n_0 ;
  wire \pattern_cntr_reg[8]_i_2_n_1 ;
  wire \pattern_cntr_reg[8]_i_2_n_2 ;
  wire \pattern_cntr_reg[8]_i_2_n_3 ;
  wire \pattern_cntr_reg_n_0_[0] ;
  wire \pattern_cntr_reg_n_0_[10] ;
  wire \pattern_cntr_reg_n_0_[11] ;
  wire \pattern_cntr_reg_n_0_[12] ;
  wire \pattern_cntr_reg_n_0_[13] ;
  wire \pattern_cntr_reg_n_0_[14] ;
  wire \pattern_cntr_reg_n_0_[15] ;
  wire \pattern_cntr_reg_n_0_[1] ;
  wire \pattern_cntr_reg_n_0_[2] ;
  wire \pattern_cntr_reg_n_0_[3] ;
  wire \pattern_cntr_reg_n_0_[4] ;
  wire \pattern_cntr_reg_n_0_[5] ;
  wire \pattern_cntr_reg_n_0_[6] ;
  wire \pattern_cntr_reg_n_0_[7] ;
  wire \pattern_cntr_reg_n_0_[8] ;
  wire \pattern_cntr_reg_n_0_[9] ;
  wire [7:0]pattern_gen_dat_out;
  wire pattern_gen_val_out;
  wire push_now;
  wire push_now_i_1_n_0;
  wire [0:0]push_stm1;
  wire [0:0]push_stm2;
  wire reset_n;
  wire send_done1;
  wire send_done1__0;
  wire send_done1_i_1_n_0;
  wire send_done2;
  wire send_done2__0;
  wire send_done2_i_1_n_0;
  (* async_reg = "true" *) wire sync_push_now_0;
  (* async_reg = "true" *) wire sync_push_now_1;
  (* async_reg = "true" *) wire sync_send_done1_0;
  (* async_reg = "true" *) wire sync_send_done1_1;
  (* async_reg = "true" *) wire sync_send_done2_0;
  (* async_reg = "true" *) wire sync_send_done2_1;
  wire [3:2]\NLW_bit_sync_cntr_reg[15]_i_3_CO_UNCONNECTED ;
  wire [3:3]\NLW_bit_sync_cntr_reg[15]_i_3_O_UNCONNECTED ;
  wire NLW_dut1_empty_UNCONNECTED;
  wire NLW_dut1_full_UNCONNECTED;
  wire NLW_dut1_rd_rst_busy_UNCONNECTED;
  wire NLW_dut1_wr_rst_busy_UNCONNECTED;
  wire [31:8]NLW_dut1_dout_UNCONNECTED;
  wire NLW_dut2_empty_UNCONNECTED;
  wire NLW_dut2_full_UNCONNECTED;
  wire NLW_dut2_rd_rst_busy_UNCONNECTED;
  wire NLW_dut2_wr_rst_busy_UNCONNECTED;
  wire [31:8]NLW_dut2_dout_UNCONNECTED;
  wire [3:2]\NLW_pattern_cntr1_reg[15]_i_7_CO_UNCONNECTED ;
  wire [3:3]\NLW_pattern_cntr1_reg[15]_i_7_O_UNCONNECTED ;
  wire [3:2]\NLW_pattern_cntr2_reg[15]_i_7_CO_UNCONNECTED ;
  wire [3:3]\NLW_pattern_cntr2_reg[15]_i_7_O_UNCONNECTED ;
  wire [3:2]\NLW_pattern_cntr_reg[15]_i_3_CO_UNCONNECTED ;
  wire [3:3]\NLW_pattern_cntr_reg[15]_i_3_O_UNCONNECTED ;

  LUT6 #(
    .INIT(64'hCCCFCFCFCCC8C8C8)) 
    \FSM_onehot_acc_state1[0]_i_1 
       (.I0(sync_send_done1_1),
        .I1(\FSM_onehot_acc_state1_reg_n_0_[2] ),
        .I2(\FSM_onehot_acc_state1[0]_i_2_n_0 ),
        .I3(\FSM_onehot_acc_state1[2]_i_2_n_0 ),
        .I4(\FSM_onehot_acc_state1[2]_i_3_n_0 ),
        .I5(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .O(\FSM_onehot_acc_state1[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \FSM_onehot_acc_state1[0]_i_2 
       (.I0(push_now),
        .I1(\FSM_onehot_acc_state1_reg_n_0_[1] ),
        .O(\FSM_onehot_acc_state1[0]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hABBBBBBBA8888888)) 
    \FSM_onehot_acc_state1[1]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(\FSM_onehot_acc_state1[1]_i_2_n_0 ),
        .I2(\FSM_onehot_acc_state1[2]_i_2_n_0 ),
        .I3(\FSM_onehot_acc_state1[1]_i_3_n_0 ),
        .I4(\FSM_onehot_acc_state1[1]_i_4_n_0 ),
        .I5(\FSM_onehot_acc_state1_reg_n_0_[1] ),
        .O(\FSM_onehot_acc_state1[1]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hF888)) 
    \FSM_onehot_acc_state1[1]_i_2 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[1] ),
        .I1(push_now),
        .I2(\FSM_onehot_acc_state1_reg_n_0_[2] ),
        .I3(sync_send_done1_1),
        .O(\FSM_onehot_acc_state1[1]_i_2_n_0 ));
  LUT5 #(
    .INIT(32'h80000000)) 
    \FSM_onehot_acc_state1[1]_i_3 
       (.I0(\pattern_cntr_reg_n_0_[0] ),
        .I1(\pattern_cntr_reg_n_0_[1] ),
        .I2(\pattern_cntr_reg_n_0_[2] ),
        .I3(\pattern_cntr_reg_n_0_[4] ),
        .I4(\pattern_cntr_reg_n_0_[3] ),
        .O(\FSM_onehot_acc_state1[1]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000008000)) 
    \FSM_onehot_acc_state1[1]_i_4 
       (.I0(\pattern_cntr_reg_n_0_[7] ),
        .I1(\pattern_cntr_reg_n_0_[8] ),
        .I2(\pattern_cntr_reg_n_0_[5] ),
        .I3(\pattern_cntr_reg_n_0_[6] ),
        .I4(\pattern_cntr_reg_n_0_[10] ),
        .I5(\pattern_cntr_reg_n_0_[9] ),
        .O(\FSM_onehot_acc_state1[1]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'hFF00FC44FC44FC44)) 
    \FSM_onehot_acc_state1[2]_i_1 
       (.I0(sync_send_done1_1),
        .I1(\FSM_onehot_acc_state1_reg_n_0_[2] ),
        .I2(push_now),
        .I3(\FSM_onehot_acc_state1_reg_n_0_[1] ),
        .I4(\FSM_onehot_acc_state1[2]_i_2_n_0 ),
        .I5(\FSM_onehot_acc_state1[2]_i_3_n_0 ),
        .O(\FSM_onehot_acc_state1[2]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0000000100000000)) 
    \FSM_onehot_acc_state1[2]_i_2 
       (.I0(\pattern_cntr_reg_n_0_[13] ),
        .I1(\pattern_cntr_reg_n_0_[14] ),
        .I2(\pattern_cntr_reg_n_0_[11] ),
        .I3(\pattern_cntr_reg_n_0_[12] ),
        .I4(\pattern_cntr_reg_n_0_[15] ),
        .I5(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .O(\FSM_onehot_acc_state1[2]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    \FSM_onehot_acc_state1[2]_i_3 
       (.I0(\pattern_cntr_reg_n_0_[3] ),
        .I1(\pattern_cntr_reg_n_0_[4] ),
        .I2(\pattern_cntr_reg_n_0_[2] ),
        .I3(\pattern_cntr_reg_n_0_[1] ),
        .I4(\pattern_cntr_reg_n_0_[0] ),
        .I5(\FSM_onehot_acc_state1[1]_i_4_n_0 ),
        .O(\FSM_onehot_acc_state1[2]_i_3_n_0 ));
  (* FSM_ENCODED_STATES = "iSTATE:001,iSTATE0:010,iSTATE1:100," *) 
  FDPE #(
    .INIT(1'b1)) 
    \FSM_onehot_acc_state1_reg[0] 
       (.C(clock_61),
        .CE(1'b1),
        .D(\FSM_onehot_acc_state1[0]_i_1_n_0 ),
        .PRE(\get_data1[7]_i_2_n_0 ),
        .Q(\FSM_onehot_acc_state1_reg_n_0_[0] ));
  (* FSM_ENCODED_STATES = "iSTATE:001,iSTATE0:010,iSTATE1:100," *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_acc_state1_reg[1] 
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\FSM_onehot_acc_state1[1]_i_1_n_0 ),
        .Q(\FSM_onehot_acc_state1_reg_n_0_[1] ));
  (* FSM_ENCODED_STATES = "iSTATE:001,iSTATE0:010,iSTATE1:100," *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_acc_state1_reg[2] 
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\FSM_onehot_acc_state1[2]_i_1_n_0 ),
        .Q(\FSM_onehot_acc_state1_reg_n_0_[2] ));
  LUT6 #(
    .INIT(64'hCCCFCFCFCCC8C8C8)) 
    \FSM_onehot_acc_state2[0]_i_1 
       (.I0(sync_send_done2_1),
        .I1(\FSM_onehot_acc_state2_reg_n_0_[2] ),
        .I2(\FSM_onehot_acc_state2[0]_i_2_n_0 ),
        .I3(\FSM_onehot_acc_state2[2]_i_2_n_0 ),
        .I4(\FSM_onehot_acc_state2[2]_i_3_n_0 ),
        .I5(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .O(\FSM_onehot_acc_state2[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \FSM_onehot_acc_state2[0]_i_2 
       (.I0(push_now),
        .I1(\FSM_onehot_acc_state2_reg_n_0_[1] ),
        .O(\FSM_onehot_acc_state2[0]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hABBBBBBBA8888888)) 
    \FSM_onehot_acc_state2[1]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\FSM_onehot_acc_state2[1]_i_2_n_0 ),
        .I2(\FSM_onehot_acc_state2[2]_i_2_n_0 ),
        .I3(\FSM_onehot_acc_state2[1]_i_3_n_0 ),
        .I4(\FSM_onehot_acc_state2[1]_i_4_n_0 ),
        .I5(\FSM_onehot_acc_state2_reg_n_0_[1] ),
        .O(\FSM_onehot_acc_state2[1]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hF888)) 
    \FSM_onehot_acc_state2[1]_i_2 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[1] ),
        .I1(push_now),
        .I2(\FSM_onehot_acc_state2_reg_n_0_[2] ),
        .I3(sync_send_done2_1),
        .O(\FSM_onehot_acc_state2[1]_i_2_n_0 ));
  LUT5 #(
    .INIT(32'h80000000)) 
    \FSM_onehot_acc_state2[1]_i_3 
       (.I0(\bit_sync_cntr_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg_n_0_[1] ),
        .I2(\bit_sync_cntr_reg_n_0_[2] ),
        .I3(\bit_sync_cntr_reg_n_0_[4] ),
        .I4(\bit_sync_cntr_reg_n_0_[3] ),
        .O(\FSM_onehot_acc_state2[1]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000008000)) 
    \FSM_onehot_acc_state2[1]_i_4 
       (.I0(\bit_sync_cntr_reg_n_0_[7] ),
        .I1(\bit_sync_cntr_reg_n_0_[8] ),
        .I2(\bit_sync_cntr_reg_n_0_[5] ),
        .I3(\bit_sync_cntr_reg_n_0_[6] ),
        .I4(\bit_sync_cntr_reg_n_0_[10] ),
        .I5(\bit_sync_cntr_reg_n_0_[9] ),
        .O(\FSM_onehot_acc_state2[1]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'hFF00FC44FC44FC44)) 
    \FSM_onehot_acc_state2[2]_i_1 
       (.I0(sync_send_done2_1),
        .I1(\FSM_onehot_acc_state2_reg_n_0_[2] ),
        .I2(push_now),
        .I3(\FSM_onehot_acc_state2_reg_n_0_[1] ),
        .I4(\FSM_onehot_acc_state2[2]_i_2_n_0 ),
        .I5(\FSM_onehot_acc_state2[2]_i_3_n_0 ),
        .O(\FSM_onehot_acc_state2[2]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0000000100000000)) 
    \FSM_onehot_acc_state2[2]_i_2 
       (.I0(\bit_sync_cntr_reg_n_0_[13] ),
        .I1(\bit_sync_cntr_reg_n_0_[14] ),
        .I2(\bit_sync_cntr_reg_n_0_[11] ),
        .I3(\bit_sync_cntr_reg_n_0_[12] ),
        .I4(\bit_sync_cntr_reg_n_0_[15] ),
        .I5(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .O(\FSM_onehot_acc_state2[2]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    \FSM_onehot_acc_state2[2]_i_3 
       (.I0(\bit_sync_cntr_reg_n_0_[3] ),
        .I1(\bit_sync_cntr_reg_n_0_[4] ),
        .I2(\bit_sync_cntr_reg_n_0_[2] ),
        .I3(\bit_sync_cntr_reg_n_0_[1] ),
        .I4(\bit_sync_cntr_reg_n_0_[0] ),
        .I5(\FSM_onehot_acc_state2[1]_i_4_n_0 ),
        .O(\FSM_onehot_acc_state2[2]_i_3_n_0 ));
  (* FSM_ENCODED_STATES = "iSTATE:001,iSTATE0:010,iSTATE1:100," *) 
  FDPE #(
    .INIT(1'b1)) 
    \FSM_onehot_acc_state2_reg[0] 
       (.C(clock_61),
        .CE(1'b1),
        .D(\FSM_onehot_acc_state2[0]_i_1_n_0 ),
        .PRE(\get_data1[7]_i_2_n_0 ),
        .Q(\FSM_onehot_acc_state2_reg_n_0_[0] ));
  (* FSM_ENCODED_STATES = "iSTATE:001,iSTATE0:010,iSTATE1:100," *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_acc_state2_reg[1] 
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\FSM_onehot_acc_state2[1]_i_1_n_0 ),
        .Q(\FSM_onehot_acc_state2_reg_n_0_[1] ));
  (* FSM_ENCODED_STATES = "iSTATE:001,iSTATE0:010,iSTATE1:100," *) 
  FDCE #(
    .INIT(1'b0)) 
    \FSM_onehot_acc_state2_reg[2] 
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\FSM_onehot_acc_state2[2]_i_1_n_0 ),
        .Q(\FSM_onehot_acc_state2_reg_n_0_[2] ));
  LUT5 #(
    .INIT(32'h00D1FF00)) 
    \FSM_sequential_push_stm1[0]_inv_i_1 
       (.I0(sync_push_now_1),
        .I1(send_done1),
        .I2(\FSM_sequential_push_stm1[0]_inv_i_2_n_0 ),
        .I3(\FSM_sequential_push_stm1[1]_i_2_n_0 ),
        .I4(push_stm1),
        .O(\FSM_sequential_push_stm1[0]_inv_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \FSM_sequential_push_stm1[0]_inv_i_2 
       (.I0(\FSM_sequential_push_stm1[0]_inv_i_3_n_0 ),
        .I1(\FSM_sequential_push_stm1[0]_inv_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_4_n_0 ),
        .I3(\pattern_cntr1[15]_i_3_n_0 ),
        .O(\FSM_sequential_push_stm1[0]_inv_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h7FFF)) 
    \FSM_sequential_push_stm1[0]_inv_i_3 
       (.I0(\pattern_cntr1_reg_n_0_[6] ),
        .I1(\pattern_cntr1_reg_n_0_[5] ),
        .I2(\pattern_cntr1_reg_n_0_[8] ),
        .I3(\pattern_cntr1_reg_n_0_[7] ),
        .O(\FSM_sequential_push_stm1[0]_inv_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'h7FFF)) 
    \FSM_sequential_push_stm1[0]_inv_i_4 
       (.I0(\pattern_cntr1_reg_n_0_[2] ),
        .I1(\pattern_cntr1_reg_n_0_[1] ),
        .I2(\pattern_cntr1_reg_n_0_[4] ),
        .I3(\pattern_cntr1_reg_n_0_[3] ),
        .O(\FSM_sequential_push_stm1[0]_inv_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'hD2)) 
    \FSM_sequential_push_stm1[1]_i_1 
       (.I0(\FSM_sequential_push_stm1[1]_i_2_n_0 ),
        .I1(push_stm1),
        .I2(send_done1),
        .O(\FSM_sequential_push_stm1[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0000000100000000)) 
    \FSM_sequential_push_stm1[1]_i_2 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1_reg_n_0_[2] ),
        .I3(push_stm1),
        .I4(\pattern_cntr1_reg_n_0_[1] ),
        .I5(\FSM_sequential_push_stm1[1]_i_3_n_0 ),
        .O(\FSM_sequential_push_stm1[1]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000001)) 
    \FSM_sequential_push_stm1[1]_i_3 
       (.I0(\pattern_cntr1_reg_n_0_[5] ),
        .I1(\pattern_cntr1_reg_n_0_[6] ),
        .I2(\pattern_cntr1_reg_n_0_[3] ),
        .I3(\pattern_cntr1_reg_n_0_[4] ),
        .I4(\pattern_cntr1_reg_n_0_[8] ),
        .I5(\pattern_cntr1_reg_n_0_[7] ),
        .O(\FSM_sequential_push_stm1[1]_i_3_n_0 ));
  (* FSM_ENCODED_STATES = "iSTATE:00,iSTATE0:01,iSTATE1:10,iSTATE2:11" *) 
  (* inverted = "yes" *) 
  FDPE \FSM_sequential_push_stm1_reg[0]_inv 
       (.C(clock_100),
        .CE(1'b1),
        .D(\FSM_sequential_push_stm1[0]_inv_i_1_n_0 ),
        .PRE(\get_data1[7]_i_2_n_0 ),
        .Q(push_stm1));
  (* FSM_ENCODED_STATES = "iSTATE:00,iSTATE0:01,iSTATE1:10,iSTATE2:11" *) 
  FDCE \FSM_sequential_push_stm1_reg[1] 
       (.C(clock_100),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\FSM_sequential_push_stm1[1]_i_1_n_0 ),
        .Q(send_done1));
  LUT5 #(
    .INIT(32'h00D1FF00)) 
    \FSM_sequential_push_stm2[0]_inv_i_1 
       (.I0(sync_push_now_1),
        .I1(send_done2),
        .I2(\FSM_sequential_push_stm2[0]_inv_i_2_n_0 ),
        .I3(\FSM_sequential_push_stm2[1]_i_2_n_0 ),
        .I4(push_stm2),
        .O(\FSM_sequential_push_stm2[0]_inv_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \FSM_sequential_push_stm2[0]_inv_i_2 
       (.I0(\FSM_sequential_push_stm2[0]_inv_i_3_n_0 ),
        .I1(\FSM_sequential_push_stm2[0]_inv_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_4_n_0 ),
        .I3(\pattern_cntr2[15]_i_3_n_0 ),
        .O(\FSM_sequential_push_stm2[0]_inv_i_2_n_0 ));
  LUT4 #(
    .INIT(16'h7FFF)) 
    \FSM_sequential_push_stm2[0]_inv_i_3 
       (.I0(\pattern_cntr2_reg_n_0_[6] ),
        .I1(\pattern_cntr2_reg_n_0_[5] ),
        .I2(\pattern_cntr2_reg_n_0_[8] ),
        .I3(\pattern_cntr2_reg_n_0_[7] ),
        .O(\FSM_sequential_push_stm2[0]_inv_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT4 #(
    .INIT(16'h7FFF)) 
    \FSM_sequential_push_stm2[0]_inv_i_4 
       (.I0(\pattern_cntr2_reg_n_0_[2] ),
        .I1(\pattern_cntr2_reg_n_0_[1] ),
        .I2(\pattern_cntr2_reg_n_0_[4] ),
        .I3(\pattern_cntr2_reg_n_0_[3] ),
        .O(\FSM_sequential_push_stm2[0]_inv_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'hD2)) 
    \FSM_sequential_push_stm2[1]_i_1 
       (.I0(\FSM_sequential_push_stm2[1]_i_2_n_0 ),
        .I1(push_stm2),
        .I2(send_done2),
        .O(\FSM_sequential_push_stm2[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0000000100000000)) 
    \FSM_sequential_push_stm2[1]_i_2 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2_reg_n_0_[2] ),
        .I3(push_stm2),
        .I4(\pattern_cntr2_reg_n_0_[1] ),
        .I5(\FSM_sequential_push_stm2[1]_i_3_n_0 ),
        .O(\FSM_sequential_push_stm2[1]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h0000000000000001)) 
    \FSM_sequential_push_stm2[1]_i_3 
       (.I0(\pattern_cntr2_reg_n_0_[5] ),
        .I1(\pattern_cntr2_reg_n_0_[6] ),
        .I2(\pattern_cntr2_reg_n_0_[3] ),
        .I3(\pattern_cntr2_reg_n_0_[4] ),
        .I4(\pattern_cntr2_reg_n_0_[8] ),
        .I5(\pattern_cntr2_reg_n_0_[7] ),
        .O(\FSM_sequential_push_stm2[1]_i_3_n_0 ));
  (* FSM_ENCODED_STATES = "iSTATE:00,iSTATE0:01,iSTATE1:10,iSTATE2:11" *) 
  (* inverted = "yes" *) 
  FDPE \FSM_sequential_push_stm2_reg[0]_inv 
       (.C(clock_100),
        .CE(1'b1),
        .D(\FSM_sequential_push_stm2[0]_inv_i_1_n_0 ),
        .PRE(\get_data1[7]_i_2_n_0 ),
        .Q(push_stm2));
  (* FSM_ENCODED_STATES = "iSTATE:00,iSTATE0:01,iSTATE1:10,iSTATE2:11" *) 
  FDCE \FSM_sequential_push_stm2_reg[1] 
       (.C(clock_100),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\FSM_sequential_push_stm2[1]_i_1_n_0 ),
        .Q(send_done2));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \bit_sync_cntr[0]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg_n_0_[0] ),
        .O(\bit_sync_cntr[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[10]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[12]_i_2_n_6 ),
        .O(\bit_sync_cntr[10]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[11]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[12]_i_2_n_5 ),
        .O(\bit_sync_cntr[11]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[12]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[12]_i_2_n_4 ),
        .O(\bit_sync_cntr[12]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[13]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[15]_i_3_n_7 ),
        .O(\bit_sync_cntr[13]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[14]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[15]_i_3_n_6 ),
        .O(\bit_sync_cntr[14]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hF888)) 
    \bit_sync_cntr[15]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[1] ),
        .I1(push_now),
        .I2(bsync_val),
        .I3(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .O(bit_sync_cntr));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[15]_i_2 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[15]_i_3_n_5 ),
        .O(\bit_sync_cntr[15]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[1]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[4]_i_2_n_7 ),
        .O(\bit_sync_cntr[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[2]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[4]_i_2_n_6 ),
        .O(\bit_sync_cntr[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[3]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[4]_i_2_n_5 ),
        .O(\bit_sync_cntr[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[4]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[4]_i_2_n_4 ),
        .O(\bit_sync_cntr[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[5]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[8]_i_2_n_7 ),
        .O(\bit_sync_cntr[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[6]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[8]_i_2_n_6 ),
        .O(\bit_sync_cntr[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[7]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[8]_i_2_n_5 ),
        .O(\bit_sync_cntr[7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[8]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[8]_i_2_n_4 ),
        .O(\bit_sync_cntr[8]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \bit_sync_cntr[9]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(\bit_sync_cntr_reg[12]_i_2_n_7 ),
        .O(\bit_sync_cntr[9]_i_1_n_0 ));
  FDCE \bit_sync_cntr_reg[0] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[0]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[0] ));
  FDCE \bit_sync_cntr_reg[10] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[10]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[10] ));
  FDCE \bit_sync_cntr_reg[11] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[11]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[11] ));
  FDCE \bit_sync_cntr_reg[12] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[12]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[12] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \bit_sync_cntr_reg[12]_i_2 
       (.CI(\bit_sync_cntr_reg[8]_i_2_n_0 ),
        .CO({\bit_sync_cntr_reg[12]_i_2_n_0 ,\bit_sync_cntr_reg[12]_i_2_n_1 ,\bit_sync_cntr_reg[12]_i_2_n_2 ,\bit_sync_cntr_reg[12]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\bit_sync_cntr_reg[12]_i_2_n_4 ,\bit_sync_cntr_reg[12]_i_2_n_5 ,\bit_sync_cntr_reg[12]_i_2_n_6 ,\bit_sync_cntr_reg[12]_i_2_n_7 }),
        .S({\bit_sync_cntr_reg_n_0_[12] ,\bit_sync_cntr_reg_n_0_[11] ,\bit_sync_cntr_reg_n_0_[10] ,\bit_sync_cntr_reg_n_0_[9] }));
  FDCE \bit_sync_cntr_reg[13] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[13]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[13] ));
  FDCE \bit_sync_cntr_reg[14] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[14]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[14] ));
  FDCE \bit_sync_cntr_reg[15] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[15]_i_2_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[15] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \bit_sync_cntr_reg[15]_i_3 
       (.CI(\bit_sync_cntr_reg[12]_i_2_n_0 ),
        .CO({\NLW_bit_sync_cntr_reg[15]_i_3_CO_UNCONNECTED [3:2],\bit_sync_cntr_reg[15]_i_3_n_2 ,\bit_sync_cntr_reg[15]_i_3_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_bit_sync_cntr_reg[15]_i_3_O_UNCONNECTED [3],\bit_sync_cntr_reg[15]_i_3_n_5 ,\bit_sync_cntr_reg[15]_i_3_n_6 ,\bit_sync_cntr_reg[15]_i_3_n_7 }),
        .S({1'b0,\bit_sync_cntr_reg_n_0_[15] ,\bit_sync_cntr_reg_n_0_[14] ,\bit_sync_cntr_reg_n_0_[13] }));
  FDCE \bit_sync_cntr_reg[1] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[1]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[1] ));
  FDCE \bit_sync_cntr_reg[2] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[2]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[2] ));
  FDCE \bit_sync_cntr_reg[3] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[3]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[3] ));
  FDCE \bit_sync_cntr_reg[4] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[4]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[4] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \bit_sync_cntr_reg[4]_i_2 
       (.CI(1'b0),
        .CO({\bit_sync_cntr_reg[4]_i_2_n_0 ,\bit_sync_cntr_reg[4]_i_2_n_1 ,\bit_sync_cntr_reg[4]_i_2_n_2 ,\bit_sync_cntr_reg[4]_i_2_n_3 }),
        .CYINIT(\bit_sync_cntr_reg_n_0_[0] ),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\bit_sync_cntr_reg[4]_i_2_n_4 ,\bit_sync_cntr_reg[4]_i_2_n_5 ,\bit_sync_cntr_reg[4]_i_2_n_6 ,\bit_sync_cntr_reg[4]_i_2_n_7 }),
        .S({\bit_sync_cntr_reg_n_0_[4] ,\bit_sync_cntr_reg_n_0_[3] ,\bit_sync_cntr_reg_n_0_[2] ,\bit_sync_cntr_reg_n_0_[1] }));
  FDCE \bit_sync_cntr_reg[5] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[5]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[5] ));
  FDCE \bit_sync_cntr_reg[6] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[6]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[6] ));
  FDCE \bit_sync_cntr_reg[7] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[7]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[7] ));
  FDCE \bit_sync_cntr_reg[8] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[8]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[8] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \bit_sync_cntr_reg[8]_i_2 
       (.CI(\bit_sync_cntr_reg[4]_i_2_n_0 ),
        .CO({\bit_sync_cntr_reg[8]_i_2_n_0 ,\bit_sync_cntr_reg[8]_i_2_n_1 ,\bit_sync_cntr_reg[8]_i_2_n_2 ,\bit_sync_cntr_reg[8]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\bit_sync_cntr_reg[8]_i_2_n_4 ,\bit_sync_cntr_reg[8]_i_2_n_5 ,\bit_sync_cntr_reg[8]_i_2_n_6 ,\bit_sync_cntr_reg[8]_i_2_n_7 }),
        .S({\bit_sync_cntr_reg_n_0_[8] ,\bit_sync_cntr_reg_n_0_[7] ,\bit_sync_cntr_reg_n_0_[6] ,\bit_sync_cntr_reg_n_0_[5] }));
  FDCE \bit_sync_cntr_reg[9] 
       (.C(clock_61),
        .CE(bit_sync_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\bit_sync_cntr[9]_i_1_n_0 ),
        .Q(\bit_sync_cntr_reg_n_0_[9] ));
  (* x_core_info = "fifo_generator_v13_2_8,Vivado 2023.1" *) 
  system_ila_display_0_0_fifo_generator_0 dut1
       (.din({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,\fifo_din1_reg_n_0_[7] ,\fifo_din1_reg_n_0_[6] ,\fifo_din1_reg_n_0_[5] ,\fifo_din1_reg_n_0_[4] ,\fifo_din1_reg_n_0_[3] ,\fifo_din1_reg_n_0_[2] ,\fifo_din1_reg_n_0_[1] ,\fifo_din1_reg_n_0_[0] }),
        .dout({NLW_dut1_dout_UNCONNECTED[31:8],fifo_dout1}),
        .empty(NLW_dut1_empty_UNCONNECTED),
        .full(NLW_dut1_full_UNCONNECTED),
        .rd_clk(clock_100),
        .rd_en(fifo_rd_en1_reg_n_0),
        .rd_rst_busy(NLW_dut1_rd_rst_busy_UNCONNECTED),
        .rst(\get_data1[7]_i_2_n_0 ),
        .wr_clk(clock_61),
        .wr_en(fifo_wr_en1_reg_n_0),
        .wr_rst_busy(NLW_dut1_wr_rst_busy_UNCONNECTED));
  (* x_core_info = "fifo_generator_v13_2_8,Vivado 2023.1" *) 
  system_ila_display_0_0_fifo_generator_0_HD1 dut2
       (.din({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,\fifo_din2_reg_n_0_[7] ,\fifo_din2_reg_n_0_[6] ,\fifo_din2_reg_n_0_[5] ,\fifo_din2_reg_n_0_[4] ,\fifo_din2_reg_n_0_[3] ,\fifo_din2_reg_n_0_[2] ,\fifo_din2_reg_n_0_[1] ,\fifo_din2_reg_n_0_[0] }),
        .dout({NLW_dut2_dout_UNCONNECTED[31:8],fifo_dout2}),
        .empty(NLW_dut2_empty_UNCONNECTED),
        .full(NLW_dut2_full_UNCONNECTED),
        .rd_clk(clock_100),
        .rd_en(fifo_rd_en2_reg_n_0),
        .rd_rst_busy(NLW_dut2_rd_rst_busy_UNCONNECTED),
        .rst(\get_data1[7]_i_2_n_0 ),
        .wr_clk(clock_61),
        .wr_en(fifo_wr_en2_reg_n_0),
        .wr_rst_busy(NLW_dut2_wr_rst_busy_UNCONNECTED));
  LUT2 #(
    .INIT(4'h8)) 
    \fifo_din1[7]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(pattern_gen_val_out),
        .O(fifo_din1));
  FDCE \fifo_din1_reg[0] 
       (.C(clock_61),
        .CE(fifo_din1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(pattern_gen_dat_out[0]),
        .Q(\fifo_din1_reg_n_0_[0] ));
  FDCE \fifo_din1_reg[1] 
       (.C(clock_61),
        .CE(fifo_din1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(pattern_gen_dat_out[1]),
        .Q(\fifo_din1_reg_n_0_[1] ));
  FDCE \fifo_din1_reg[2] 
       (.C(clock_61),
        .CE(fifo_din1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(pattern_gen_dat_out[2]),
        .Q(\fifo_din1_reg_n_0_[2] ));
  FDCE \fifo_din1_reg[3] 
       (.C(clock_61),
        .CE(fifo_din1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(pattern_gen_dat_out[3]),
        .Q(\fifo_din1_reg_n_0_[3] ));
  FDCE \fifo_din1_reg[4] 
       (.C(clock_61),
        .CE(fifo_din1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(pattern_gen_dat_out[4]),
        .Q(\fifo_din1_reg_n_0_[4] ));
  FDCE \fifo_din1_reg[5] 
       (.C(clock_61),
        .CE(fifo_din1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(pattern_gen_dat_out[5]),
        .Q(\fifo_din1_reg_n_0_[5] ));
  FDCE \fifo_din1_reg[6] 
       (.C(clock_61),
        .CE(fifo_din1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(pattern_gen_dat_out[6]),
        .Q(\fifo_din1_reg_n_0_[6] ));
  FDCE \fifo_din1_reg[7] 
       (.C(clock_61),
        .CE(fifo_din1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(pattern_gen_dat_out[7]),
        .Q(\fifo_din1_reg_n_0_[7] ));
  LUT2 #(
    .INIT(4'h8)) 
    \fifo_din2[7]_i_1 
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[0] ),
        .I1(bsync_val),
        .O(fifo_din2));
  FDCE \fifo_din2_reg[0] 
       (.C(clock_61),
        .CE(fifo_din2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(bsync_dat[0]),
        .Q(\fifo_din2_reg_n_0_[0] ));
  FDCE \fifo_din2_reg[1] 
       (.C(clock_61),
        .CE(fifo_din2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(bsync_dat[1]),
        .Q(\fifo_din2_reg_n_0_[1] ));
  FDCE \fifo_din2_reg[2] 
       (.C(clock_61),
        .CE(fifo_din2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(bsync_dat[2]),
        .Q(\fifo_din2_reg_n_0_[2] ));
  FDCE \fifo_din2_reg[3] 
       (.C(clock_61),
        .CE(fifo_din2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(bsync_dat[3]),
        .Q(\fifo_din2_reg_n_0_[3] ));
  FDCE \fifo_din2_reg[4] 
       (.C(clock_61),
        .CE(fifo_din2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(bsync_dat[4]),
        .Q(\fifo_din2_reg_n_0_[4] ));
  FDCE \fifo_din2_reg[5] 
       (.C(clock_61),
        .CE(fifo_din2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(bsync_dat[5]),
        .Q(\fifo_din2_reg_n_0_[5] ));
  FDCE \fifo_din2_reg[6] 
       (.C(clock_61),
        .CE(fifo_din2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(bsync_dat[6]),
        .Q(\fifo_din2_reg_n_0_[6] ));
  FDCE \fifo_din2_reg[7] 
       (.C(clock_61),
        .CE(fifo_din2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(bsync_dat[7]),
        .Q(\fifo_din2_reg_n_0_[7] ));
  LUT5 #(
    .INIT(32'hCFAFC0AF)) 
    fifo_rd_en1_i_1
       (.I0(sync_push_now_1),
        .I1(\FSM_sequential_push_stm1[0]_inv_i_2_n_0 ),
        .I2(push_stm1),
        .I3(send_done1),
        .I4(fifo_rd_en1_reg_n_0),
        .O(fifo_rd_en1_i_1_n_0));
  FDCE fifo_rd_en1_reg
       (.C(clock_100),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(fifo_rd_en1_i_1_n_0),
        .Q(fifo_rd_en1_reg_n_0));
  LUT5 #(
    .INIT(32'hCFAFC0AF)) 
    fifo_rd_en2_i_1
       (.I0(sync_push_now_1),
        .I1(\FSM_sequential_push_stm2[0]_inv_i_2_n_0 ),
        .I2(push_stm2),
        .I3(send_done2),
        .I4(fifo_rd_en2_reg_n_0),
        .O(fifo_rd_en2_i_1_n_0));
  FDCE fifo_rd_en2_reg
       (.C(clock_100),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(fifo_rd_en2_i_1_n_0),
        .Q(fifo_rd_en2_reg_n_0));
  FDCE fifo_wr_en1_reg
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(fifo_din1),
        .Q(fifo_wr_en1_reg_n_0));
  FDCE fifo_wr_en2_reg
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(fifo_din2),
        .Q(fifo_wr_en2_reg_n_0));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data1[0]_i_1 
       (.I0(send_done1),
        .I1(fifo_dout1[0]),
        .O(\get_data1[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data1[1]_i_1 
       (.I0(send_done1),
        .I1(fifo_dout1[1]),
        .O(\get_data1[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data1[2]_i_1 
       (.I0(send_done1),
        .I1(fifo_dout1[2]),
        .O(\get_data1[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data1[3]_i_1 
       (.I0(send_done1),
        .I1(fifo_dout1[3]),
        .O(\get_data1[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data1[4]_i_1 
       (.I0(send_done1),
        .I1(fifo_dout1[4]),
        .O(\get_data1[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data1[5]_i_1 
       (.I0(send_done1),
        .I1(fifo_dout1[5]),
        .O(\get_data1[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data1[6]_i_1 
       (.I0(send_done1),
        .I1(fifo_dout1[6]),
        .O(\get_data1[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data1[7]_i_1 
       (.I0(send_done1),
        .I1(fifo_dout1[7]),
        .O(\get_data1[7]_i_1_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \get_data1[7]_i_2 
       (.I0(reset_n),
        .O(\get_data1[7]_i_2_n_0 ));
  FDCE \get_data1_reg[0] 
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data1[0]_i_1_n_0 ),
        .Q(ila_pattern_gen_dat[0]));
  FDCE \get_data1_reg[1] 
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data1[1]_i_1_n_0 ),
        .Q(ila_pattern_gen_dat[1]));
  FDCE \get_data1_reg[2] 
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data1[2]_i_1_n_0 ),
        .Q(ila_pattern_gen_dat[2]));
  FDCE \get_data1_reg[3] 
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data1[3]_i_1_n_0 ),
        .Q(ila_pattern_gen_dat[3]));
  FDCE \get_data1_reg[4] 
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data1[4]_i_1_n_0 ),
        .Q(ila_pattern_gen_dat[4]));
  FDCE \get_data1_reg[5] 
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data1[5]_i_1_n_0 ),
        .Q(ila_pattern_gen_dat[5]));
  FDCE \get_data1_reg[6] 
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data1[6]_i_1_n_0 ),
        .Q(ila_pattern_gen_dat[6]));
  FDCE \get_data1_reg[7] 
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data1[7]_i_1_n_0 ),
        .Q(ila_pattern_gen_dat[7]));
  FDCE get_data1_valid_reg
       (.C(clock_100),
        .CE(push_stm1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(send_done1),
        .Q(ila_pattern_gen_val));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data2[0]_i_1 
       (.I0(send_done2),
        .I1(fifo_dout2[0]),
        .O(\get_data2[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data2[1]_i_1 
       (.I0(send_done2),
        .I1(fifo_dout2[1]),
        .O(\get_data2[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data2[2]_i_1 
       (.I0(send_done2),
        .I1(fifo_dout2[2]),
        .O(\get_data2[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data2[3]_i_1 
       (.I0(send_done2),
        .I1(fifo_dout2[3]),
        .O(\get_data2[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data2[4]_i_1 
       (.I0(send_done2),
        .I1(fifo_dout2[4]),
        .O(\get_data2[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data2[5]_i_1 
       (.I0(send_done2),
        .I1(fifo_dout2[5]),
        .O(\get_data2[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data2[6]_i_1 
       (.I0(send_done2),
        .I1(fifo_dout2[6]),
        .O(\get_data2[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \get_data2[7]_i_1 
       (.I0(send_done2),
        .I1(fifo_dout2[7]),
        .O(\get_data2[7]_i_1_n_0 ));
  FDCE \get_data2_reg[0] 
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data2[0]_i_1_n_0 ),
        .Q(ila_bsync_dat[0]));
  FDCE \get_data2_reg[1] 
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data2[1]_i_1_n_0 ),
        .Q(ila_bsync_dat[1]));
  FDCE \get_data2_reg[2] 
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data2[2]_i_1_n_0 ),
        .Q(ila_bsync_dat[2]));
  FDCE \get_data2_reg[3] 
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data2[3]_i_1_n_0 ),
        .Q(ila_bsync_dat[3]));
  FDCE \get_data2_reg[4] 
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data2[4]_i_1_n_0 ),
        .Q(ila_bsync_dat[4]));
  FDCE \get_data2_reg[5] 
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data2[5]_i_1_n_0 ),
        .Q(ila_bsync_dat[5]));
  FDCE \get_data2_reg[6] 
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data2[6]_i_1_n_0 ),
        .Q(ila_bsync_dat[6]));
  FDCE \get_data2_reg[7] 
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\get_data2[7]_i_1_n_0 ),
        .Q(ila_bsync_dat[7]));
  FDCE get_data2_valid_reg
       (.C(clock_100),
        .CE(push_stm2),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(send_done2),
        .Q(ila_bsync_val));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \pattern_cntr1[0]_i_1 
       (.I0(\pattern_cntr1_reg_n_0_[0] ),
        .O(\pattern_cntr1[0]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[10]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[12]_i_2_n_6 ),
        .O(\pattern_cntr1[10]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[11]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[12]_i_2_n_5 ),
        .O(\pattern_cntr1[11]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[12]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[12]_i_2_n_4 ),
        .O(\pattern_cntr1[12]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[13]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[15]_i_7_n_7 ),
        .O(\pattern_cntr1[13]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[14]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[15]_i_7_n_6 ),
        .O(\pattern_cntr1[14]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'hD)) 
    \pattern_cntr1[15]_i_1 
       (.I0(push_stm1),
        .I1(send_done1),
        .O(\pattern_cntr1[15]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[15]_i_2 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[15]_i_7_n_5 ),
        .O(\pattern_cntr1[15]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT4 #(
    .INIT(16'hFFFD)) 
    \pattern_cntr1[15]_i_3 
       (.I0(\pattern_cntr1_reg_n_0_[0] ),
        .I1(\pattern_cntr1_reg_n_0_[9] ),
        .I2(\pattern_cntr1_reg_n_0_[11] ),
        .I3(\pattern_cntr1_reg_n_0_[10] ),
        .O(\pattern_cntr1[15]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \pattern_cntr1[15]_i_4 
       (.I0(\pattern_cntr1_reg_n_0_[13] ),
        .I1(\pattern_cntr1_reg_n_0_[12] ),
        .I2(\pattern_cntr1_reg_n_0_[15] ),
        .I3(\pattern_cntr1_reg_n_0_[14] ),
        .O(\pattern_cntr1[15]_i_4_n_0 ));
  LUT5 #(
    .INIT(32'h7FFFFFFE)) 
    \pattern_cntr1[15]_i_5 
       (.I0(\pattern_cntr1_reg_n_0_[7] ),
        .I1(\pattern_cntr1_reg_n_0_[6] ),
        .I2(\pattern_cntr1_reg_n_0_[1] ),
        .I3(push_stm1),
        .I4(\pattern_cntr1_reg_n_0_[8] ),
        .O(\pattern_cntr1[15]_i_5_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT5 #(
    .INIT(32'h7FFFFFFE)) 
    \pattern_cntr1[15]_i_6 
       (.I0(\pattern_cntr1_reg_n_0_[3] ),
        .I1(\pattern_cntr1_reg_n_0_[2] ),
        .I2(\pattern_cntr1_reg_n_0_[1] ),
        .I3(\pattern_cntr1_reg_n_0_[5] ),
        .I4(\pattern_cntr1_reg_n_0_[4] ),
        .O(\pattern_cntr1[15]_i_6_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[1]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[4]_i_2_n_7 ),
        .O(\pattern_cntr1[1]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[2]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[4]_i_2_n_6 ),
        .O(\pattern_cntr1[2]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[3]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[4]_i_2_n_5 ),
        .O(\pattern_cntr1[3]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[4]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[4]_i_2_n_4 ),
        .O(\pattern_cntr1[4]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[5]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[8]_i_2_n_7 ),
        .O(\pattern_cntr1[5]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[6]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[8]_i_2_n_6 ),
        .O(\pattern_cntr1[6]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[7]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[8]_i_2_n_5 ),
        .O(\pattern_cntr1[7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[8]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[8]_i_2_n_4 ),
        .O(\pattern_cntr1[8]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr1[9]_i_1 
       (.I0(\pattern_cntr1[15]_i_3_n_0 ),
        .I1(\pattern_cntr1[15]_i_4_n_0 ),
        .I2(\pattern_cntr1[15]_i_5_n_0 ),
        .I3(\pattern_cntr1[15]_i_6_n_0 ),
        .I4(\pattern_cntr1_reg[12]_i_2_n_7 ),
        .O(\pattern_cntr1[9]_i_1_n_0 ));
  FDCE \pattern_cntr1_reg[0] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[0]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[0] ));
  FDCE \pattern_cntr1_reg[10] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[10]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[10] ));
  FDCE \pattern_cntr1_reg[11] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[11]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[11] ));
  FDCE \pattern_cntr1_reg[12] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[12]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[12] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr1_reg[12]_i_2 
       (.CI(\pattern_cntr1_reg[8]_i_2_n_0 ),
        .CO({\pattern_cntr1_reg[12]_i_2_n_0 ,\pattern_cntr1_reg[12]_i_2_n_1 ,\pattern_cntr1_reg[12]_i_2_n_2 ,\pattern_cntr1_reg[12]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\pattern_cntr1_reg[12]_i_2_n_4 ,\pattern_cntr1_reg[12]_i_2_n_5 ,\pattern_cntr1_reg[12]_i_2_n_6 ,\pattern_cntr1_reg[12]_i_2_n_7 }),
        .S({\pattern_cntr1_reg_n_0_[12] ,\pattern_cntr1_reg_n_0_[11] ,\pattern_cntr1_reg_n_0_[10] ,\pattern_cntr1_reg_n_0_[9] }));
  FDCE \pattern_cntr1_reg[13] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[13]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[13] ));
  FDCE \pattern_cntr1_reg[14] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[14]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[14] ));
  FDCE \pattern_cntr1_reg[15] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[15]_i_2_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[15] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr1_reg[15]_i_7 
       (.CI(\pattern_cntr1_reg[12]_i_2_n_0 ),
        .CO({\NLW_pattern_cntr1_reg[15]_i_7_CO_UNCONNECTED [3:2],\pattern_cntr1_reg[15]_i_7_n_2 ,\pattern_cntr1_reg[15]_i_7_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_pattern_cntr1_reg[15]_i_7_O_UNCONNECTED [3],\pattern_cntr1_reg[15]_i_7_n_5 ,\pattern_cntr1_reg[15]_i_7_n_6 ,\pattern_cntr1_reg[15]_i_7_n_7 }),
        .S({1'b0,\pattern_cntr1_reg_n_0_[15] ,\pattern_cntr1_reg_n_0_[14] ,\pattern_cntr1_reg_n_0_[13] }));
  FDCE \pattern_cntr1_reg[1] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[1]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[1] ));
  FDCE \pattern_cntr1_reg[2] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[2]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[2] ));
  FDCE \pattern_cntr1_reg[3] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[3]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[3] ));
  FDCE \pattern_cntr1_reg[4] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[4]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[4] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr1_reg[4]_i_2 
       (.CI(1'b0),
        .CO({\pattern_cntr1_reg[4]_i_2_n_0 ,\pattern_cntr1_reg[4]_i_2_n_1 ,\pattern_cntr1_reg[4]_i_2_n_2 ,\pattern_cntr1_reg[4]_i_2_n_3 }),
        .CYINIT(\pattern_cntr1_reg_n_0_[0] ),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\pattern_cntr1_reg[4]_i_2_n_4 ,\pattern_cntr1_reg[4]_i_2_n_5 ,\pattern_cntr1_reg[4]_i_2_n_6 ,\pattern_cntr1_reg[4]_i_2_n_7 }),
        .S({\pattern_cntr1_reg_n_0_[4] ,\pattern_cntr1_reg_n_0_[3] ,\pattern_cntr1_reg_n_0_[2] ,\pattern_cntr1_reg_n_0_[1] }));
  FDCE \pattern_cntr1_reg[5] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[5]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[5] ));
  FDCE \pattern_cntr1_reg[6] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[6]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[6] ));
  FDCE \pattern_cntr1_reg[7] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[7]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[7] ));
  FDCE \pattern_cntr1_reg[8] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[8]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[8] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr1_reg[8]_i_2 
       (.CI(\pattern_cntr1_reg[4]_i_2_n_0 ),
        .CO({\pattern_cntr1_reg[8]_i_2_n_0 ,\pattern_cntr1_reg[8]_i_2_n_1 ,\pattern_cntr1_reg[8]_i_2_n_2 ,\pattern_cntr1_reg[8]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\pattern_cntr1_reg[8]_i_2_n_4 ,\pattern_cntr1_reg[8]_i_2_n_5 ,\pattern_cntr1_reg[8]_i_2_n_6 ,\pattern_cntr1_reg[8]_i_2_n_7 }),
        .S({\pattern_cntr1_reg_n_0_[8] ,\pattern_cntr1_reg_n_0_[7] ,\pattern_cntr1_reg_n_0_[6] ,\pattern_cntr1_reg_n_0_[5] }));
  FDCE \pattern_cntr1_reg[9] 
       (.C(clock_100),
        .CE(\pattern_cntr1[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr1[9]_i_1_n_0 ),
        .Q(\pattern_cntr1_reg_n_0_[9] ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \pattern_cntr2[0]_i_1 
       (.I0(\pattern_cntr2_reg_n_0_[0] ),
        .O(\pattern_cntr2[0]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[10]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[10]),
        .O(\pattern_cntr2[10]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[11]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[11]),
        .O(\pattern_cntr2[11]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[12]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[12]),
        .O(\pattern_cntr2[12]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[13]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[13]),
        .O(\pattern_cntr2[13]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[14]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[14]),
        .O(\pattern_cntr2[14]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'hD)) 
    \pattern_cntr2[15]_i_1 
       (.I0(push_stm2),
        .I1(send_done2),
        .O(\pattern_cntr2[15]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[15]_i_2 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[15]),
        .O(\pattern_cntr2[15]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT4 #(
    .INIT(16'hFFFD)) 
    \pattern_cntr2[15]_i_3 
       (.I0(\pattern_cntr2_reg_n_0_[0] ),
        .I1(\pattern_cntr2_reg_n_0_[9] ),
        .I2(\pattern_cntr2_reg_n_0_[11] ),
        .I3(\pattern_cntr2_reg_n_0_[10] ),
        .O(\pattern_cntr2[15]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \pattern_cntr2[15]_i_4 
       (.I0(\pattern_cntr2_reg_n_0_[13] ),
        .I1(\pattern_cntr2_reg_n_0_[12] ),
        .I2(\pattern_cntr2_reg_n_0_[15] ),
        .I3(\pattern_cntr2_reg_n_0_[14] ),
        .O(\pattern_cntr2[15]_i_4_n_0 ));
  LUT5 #(
    .INIT(32'h7FFFFFFE)) 
    \pattern_cntr2[15]_i_5 
       (.I0(\pattern_cntr2_reg_n_0_[7] ),
        .I1(\pattern_cntr2_reg_n_0_[6] ),
        .I2(\pattern_cntr2_reg_n_0_[1] ),
        .I3(push_stm2),
        .I4(\pattern_cntr2_reg_n_0_[8] ),
        .O(\pattern_cntr2[15]_i_5_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h7FFFFFFE)) 
    \pattern_cntr2[15]_i_6 
       (.I0(\pattern_cntr2_reg_n_0_[3] ),
        .I1(\pattern_cntr2_reg_n_0_[2] ),
        .I2(\pattern_cntr2_reg_n_0_[1] ),
        .I3(\pattern_cntr2_reg_n_0_[5] ),
        .I4(\pattern_cntr2_reg_n_0_[4] ),
        .O(\pattern_cntr2[15]_i_6_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[1]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[1]),
        .O(\pattern_cntr2[1]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[2]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[2]),
        .O(\pattern_cntr2[2]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[3]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[3]),
        .O(\pattern_cntr2[3]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[4]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[4]),
        .O(\pattern_cntr2[4]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[5]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[5]),
        .O(\pattern_cntr2[5]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[6]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[6]),
        .O(\pattern_cntr2[6]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[7]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[7]),
        .O(\pattern_cntr2[7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[8]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[8]),
        .O(\pattern_cntr2[8]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFFFE0000)) 
    \pattern_cntr2[9]_i_1 
       (.I0(\pattern_cntr2[15]_i_3_n_0 ),
        .I1(\pattern_cntr2[15]_i_4_n_0 ),
        .I2(\pattern_cntr2[15]_i_5_n_0 ),
        .I3(\pattern_cntr2[15]_i_6_n_0 ),
        .I4(data0[9]),
        .O(\pattern_cntr2[9]_i_1_n_0 ));
  FDCE \pattern_cntr2_reg[0] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[0]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[0] ));
  FDCE \pattern_cntr2_reg[10] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[10]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[10] ));
  FDCE \pattern_cntr2_reg[11] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[11]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[11] ));
  FDCE \pattern_cntr2_reg[12] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[12]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[12] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr2_reg[12]_i_2 
       (.CI(\pattern_cntr2_reg[8]_i_2_n_0 ),
        .CO({\pattern_cntr2_reg[12]_i_2_n_0 ,\pattern_cntr2_reg[12]_i_2_n_1 ,\pattern_cntr2_reg[12]_i_2_n_2 ,\pattern_cntr2_reg[12]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(data0[12:9]),
        .S({\pattern_cntr2_reg_n_0_[12] ,\pattern_cntr2_reg_n_0_[11] ,\pattern_cntr2_reg_n_0_[10] ,\pattern_cntr2_reg_n_0_[9] }));
  FDCE \pattern_cntr2_reg[13] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[13]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[13] ));
  FDCE \pattern_cntr2_reg[14] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[14]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[14] ));
  FDCE \pattern_cntr2_reg[15] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[15]_i_2_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[15] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr2_reg[15]_i_7 
       (.CI(\pattern_cntr2_reg[12]_i_2_n_0 ),
        .CO({\NLW_pattern_cntr2_reg[15]_i_7_CO_UNCONNECTED [3:2],\pattern_cntr2_reg[15]_i_7_n_2 ,\pattern_cntr2_reg[15]_i_7_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_pattern_cntr2_reg[15]_i_7_O_UNCONNECTED [3],data0[15:13]}),
        .S({1'b0,\pattern_cntr2_reg_n_0_[15] ,\pattern_cntr2_reg_n_0_[14] ,\pattern_cntr2_reg_n_0_[13] }));
  FDCE \pattern_cntr2_reg[1] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[1]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[1] ));
  FDCE \pattern_cntr2_reg[2] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[2]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[2] ));
  FDCE \pattern_cntr2_reg[3] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[3]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[3] ));
  FDCE \pattern_cntr2_reg[4] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[4]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[4] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr2_reg[4]_i_2 
       (.CI(1'b0),
        .CO({\pattern_cntr2_reg[4]_i_2_n_0 ,\pattern_cntr2_reg[4]_i_2_n_1 ,\pattern_cntr2_reg[4]_i_2_n_2 ,\pattern_cntr2_reg[4]_i_2_n_3 }),
        .CYINIT(\pattern_cntr2_reg_n_0_[0] ),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(data0[4:1]),
        .S({\pattern_cntr2_reg_n_0_[4] ,\pattern_cntr2_reg_n_0_[3] ,\pattern_cntr2_reg_n_0_[2] ,\pattern_cntr2_reg_n_0_[1] }));
  FDCE \pattern_cntr2_reg[5] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[5]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[5] ));
  FDCE \pattern_cntr2_reg[6] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[6]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[6] ));
  FDCE \pattern_cntr2_reg[7] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[7]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[7] ));
  FDCE \pattern_cntr2_reg[8] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[8]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[8] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr2_reg[8]_i_2 
       (.CI(\pattern_cntr2_reg[4]_i_2_n_0 ),
        .CO({\pattern_cntr2_reg[8]_i_2_n_0 ,\pattern_cntr2_reg[8]_i_2_n_1 ,\pattern_cntr2_reg[8]_i_2_n_2 ,\pattern_cntr2_reg[8]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(data0[8:5]),
        .S({\pattern_cntr2_reg_n_0_[8] ,\pattern_cntr2_reg_n_0_[7] ,\pattern_cntr2_reg_n_0_[6] ,\pattern_cntr2_reg_n_0_[5] }));
  FDCE \pattern_cntr2_reg[9] 
       (.C(clock_100),
        .CE(\pattern_cntr2[15]_i_1_n_0 ),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr2[9]_i_1_n_0 ),
        .Q(\pattern_cntr2_reg_n_0_[9] ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \pattern_cntr[0]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(\pattern_cntr_reg_n_0_[0] ),
        .O(\pattern_cntr[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[10]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[10]),
        .O(\pattern_cntr[10]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[11]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[11]),
        .O(\pattern_cntr[11]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[12]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[12]),
        .O(\pattern_cntr[12]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[13]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[13]),
        .O(\pattern_cntr[13]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[14]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[14]),
        .O(\pattern_cntr[14]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hF888)) 
    \pattern_cntr[15]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[1] ),
        .I1(push_now),
        .I2(pattern_gen_val_out),
        .I3(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .O(pattern_cntr));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[15]_i_2 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[15]),
        .O(\pattern_cntr[15]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[1]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[1]),
        .O(\pattern_cntr[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[2]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[2]),
        .O(\pattern_cntr[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[3]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[3]),
        .O(\pattern_cntr[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[4]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[4]),
        .O(\pattern_cntr[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[5]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[5]),
        .O(\pattern_cntr[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[6]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[6]),
        .O(\pattern_cntr[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[7]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[7]),
        .O(\pattern_cntr[7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[8]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[8]),
        .O(\pattern_cntr[8]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \pattern_cntr[9]_i_1 
       (.I0(\FSM_onehot_acc_state1_reg_n_0_[0] ),
        .I1(in4[9]),
        .O(\pattern_cntr[9]_i_1_n_0 ));
  FDCE \pattern_cntr_reg[0] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[0]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[0] ));
  FDCE \pattern_cntr_reg[10] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[10]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[10] ));
  FDCE \pattern_cntr_reg[11] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[11]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[11] ));
  FDCE \pattern_cntr_reg[12] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[12]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[12] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr_reg[12]_i_2 
       (.CI(\pattern_cntr_reg[8]_i_2_n_0 ),
        .CO({\pattern_cntr_reg[12]_i_2_n_0 ,\pattern_cntr_reg[12]_i_2_n_1 ,\pattern_cntr_reg[12]_i_2_n_2 ,\pattern_cntr_reg[12]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(in4[12:9]),
        .S({\pattern_cntr_reg_n_0_[12] ,\pattern_cntr_reg_n_0_[11] ,\pattern_cntr_reg_n_0_[10] ,\pattern_cntr_reg_n_0_[9] }));
  FDCE \pattern_cntr_reg[13] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[13]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[13] ));
  FDCE \pattern_cntr_reg[14] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[14]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[14] ));
  FDCE \pattern_cntr_reg[15] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[15]_i_2_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[15] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr_reg[15]_i_3 
       (.CI(\pattern_cntr_reg[12]_i_2_n_0 ),
        .CO({\NLW_pattern_cntr_reg[15]_i_3_CO_UNCONNECTED [3:2],\pattern_cntr_reg[15]_i_3_n_2 ,\pattern_cntr_reg[15]_i_3_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_pattern_cntr_reg[15]_i_3_O_UNCONNECTED [3],in4[15:13]}),
        .S({1'b0,\pattern_cntr_reg_n_0_[15] ,\pattern_cntr_reg_n_0_[14] ,\pattern_cntr_reg_n_0_[13] }));
  FDCE \pattern_cntr_reg[1] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[1]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[1] ));
  FDCE \pattern_cntr_reg[2] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[2]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[2] ));
  FDCE \pattern_cntr_reg[3] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[3]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[3] ));
  FDCE \pattern_cntr_reg[4] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[4]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[4] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr_reg[4]_i_2 
       (.CI(1'b0),
        .CO({\pattern_cntr_reg[4]_i_2_n_0 ,\pattern_cntr_reg[4]_i_2_n_1 ,\pattern_cntr_reg[4]_i_2_n_2 ,\pattern_cntr_reg[4]_i_2_n_3 }),
        .CYINIT(\pattern_cntr_reg_n_0_[0] ),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(in4[4:1]),
        .S({\pattern_cntr_reg_n_0_[4] ,\pattern_cntr_reg_n_0_[3] ,\pattern_cntr_reg_n_0_[2] ,\pattern_cntr_reg_n_0_[1] }));
  FDCE \pattern_cntr_reg[5] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[5]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[5] ));
  FDCE \pattern_cntr_reg[6] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[6]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[6] ));
  FDCE \pattern_cntr_reg[7] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[7]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[7] ));
  FDCE \pattern_cntr_reg[8] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[8]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[8] ));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \pattern_cntr_reg[8]_i_2 
       (.CI(\pattern_cntr_reg[4]_i_2_n_0 ),
        .CO({\pattern_cntr_reg[8]_i_2_n_0 ,\pattern_cntr_reg[8]_i_2_n_1 ,\pattern_cntr_reg[8]_i_2_n_2 ,\pattern_cntr_reg[8]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(in4[8:5]),
        .S({\pattern_cntr_reg_n_0_[8] ,\pattern_cntr_reg_n_0_[7] ,\pattern_cntr_reg_n_0_[6] ,\pattern_cntr_reg_n_0_[5] }));
  FDCE \pattern_cntr_reg[9] 
       (.C(clock_61),
        .CE(pattern_cntr),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(\pattern_cntr[9]_i_1_n_0 ),
        .Q(\pattern_cntr_reg_n_0_[9] ));
  LUT2 #(
    .INIT(4'h8)) 
    push_now_i_1
       (.I0(\FSM_onehot_acc_state2_reg_n_0_[1] ),
        .I1(\FSM_onehot_acc_state1_reg_n_0_[1] ),
        .O(push_now_i_1_n_0));
  FDCE push_now_reg
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(push_now_i_1_n_0),
        .Q(push_now));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT4 #(
    .INIT(16'hCF40)) 
    send_done1_i_1
       (.I0(\FSM_sequential_push_stm1[0]_inv_i_2_n_0 ),
        .I1(send_done1),
        .I2(push_stm1),
        .I3(send_done1__0),
        .O(send_done1_i_1_n_0));
  FDCE send_done1_reg
       (.C(clock_100),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(send_done1_i_1_n_0),
        .Q(send_done1__0));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT4 #(
    .INIT(16'hCF40)) 
    send_done2_i_1
       (.I0(\FSM_sequential_push_stm2[0]_inv_i_2_n_0 ),
        .I1(send_done2),
        .I2(push_stm2),
        .I3(send_done2__0),
        .O(send_done2_i_1_n_0));
  FDCE send_done2_reg
       (.C(clock_100),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(send_done2_i_1_n_0),
        .Q(send_done2__0));
  (* ASYNC_REG *) 
  (* KEEP = "yes" *) 
  FDCE sync_push_now_0_reg
       (.C(clock_100),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(push_now),
        .Q(sync_push_now_0));
  (* ASYNC_REG *) 
  (* KEEP = "yes" *) 
  FDCE sync_push_now_1_reg
       (.C(clock_100),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(sync_push_now_0),
        .Q(sync_push_now_1));
  (* ASYNC_REG *) 
  (* KEEP = "yes" *) 
  FDCE sync_send_done1_0_reg
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(send_done1__0),
        .Q(sync_send_done1_0));
  (* ASYNC_REG *) 
  (* KEEP = "yes" *) 
  FDCE sync_send_done1_1_reg
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(sync_send_done1_0),
        .Q(sync_send_done1_1));
  (* ASYNC_REG *) 
  (* KEEP = "yes" *) 
  FDCE sync_send_done2_0_reg
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(send_done2__0),
        .Q(sync_send_done2_0));
  (* ASYNC_REG *) 
  (* KEEP = "yes" *) 
  FDCE sync_send_done2_1_reg
       (.C(clock_61),
        .CE(1'b1),
        .CLR(\get_data1[7]_i_2_n_0 ),
        .D(sync_send_done2_0),
        .Q(sync_send_done2_1));
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
