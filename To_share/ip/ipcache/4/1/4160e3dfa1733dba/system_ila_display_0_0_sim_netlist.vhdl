-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Sat Jan  3 12:25:20 2026
-- Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim
--               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_ila_display_0_0/system_ila_display_0_0_sim_netlist.vhdl
-- Design      : system_ila_display_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_ila_display_0_0_ila_display is
  port (
    clock_61 : in STD_LOGIC;
    clock_100 : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    pattern_gen_dat_out : in STD_LOGIC_VECTOR ( 7 downto 0 );
    pattern_gen_val_out : in STD_LOGIC;
    bsync_dat : in STD_LOGIC_VECTOR ( 7 downto 0 );
    bsync_val : in STD_LOGIC;
    ila_pattern_gen_dat : out STD_LOGIC_VECTOR ( 7 downto 0 );
    ila_pattern_gen_val : out STD_LOGIC;
    ila_bsync_dat : out STD_LOGIC_VECTOR ( 7 downto 0 );
    ila_bsync_val : out STD_LOGIC
  );
  attribute FIFO_DEPTH : string;
  attribute FIFO_DEPTH of system_ila_display_0_0_ila_display : entity is "16'b0000001000000000";
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of system_ila_display_0_0_ila_display : entity is "ila_display";
end system_ila_display_0_0_ila_display;

architecture STRUCTURE of system_ila_display_0_0_ila_display is
  component system_ila_display_0_0_fifo_generator_0 is
  port (
    rst : in STD_LOGIC;
    wr_clk : in STD_LOGIC;
    rd_clk : in STD_LOGIC;
    din : in STD_LOGIC_VECTOR ( 31 downto 0 );
    wr_en : in STD_LOGIC;
    rd_en : in STD_LOGIC;
    dout : out STD_LOGIC_VECTOR ( 31 downto 0 );
    full : out STD_LOGIC;
    empty : out STD_LOGIC;
    wr_rst_busy : out STD_LOGIC;
    rd_rst_busy : out STD_LOGIC
  );
  end component system_ila_display_0_0_fifo_generator_0;
  component system_ila_display_0_0_fifo_generator_0_HD1 is
  port (
    empty : out STD_LOGIC;
    full : out STD_LOGIC;
    rd_clk : in STD_LOGIC;
    rd_en : in STD_LOGIC;
    rd_rst_busy : out STD_LOGIC;
    rst : in STD_LOGIC;
    wr_clk : in STD_LOGIC;
    wr_en : in STD_LOGIC;
    wr_rst_busy : out STD_LOGIC;
    din : in STD_LOGIC_VECTOR ( 31 downto 0 );
    dout : out STD_LOGIC_VECTOR ( 31 downto 0 )
  );
  end component system_ila_display_0_0_fifo_generator_0_HD1;
  signal \FSM_onehot_acc_state1[0]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1[0]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1[1]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1[1]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1[1]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1[1]_i_4_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1[2]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1[2]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1[2]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1_reg_n_0_[0]\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1_reg_n_0_[1]\ : STD_LOGIC;
  signal \FSM_onehot_acc_state1_reg_n_0_[2]\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[0]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[0]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[1]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[1]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[1]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[1]_i_4_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[2]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[2]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2[2]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2_reg_n_0_[0]\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2_reg_n_0_[1]\ : STD_LOGIC;
  signal \FSM_onehot_acc_state2_reg_n_0_[2]\ : STD_LOGIC;
  signal \FSM_sequential_push_stm1[0]_inv_i_1_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm1[0]_inv_i_2_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm1[0]_inv_i_3_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm1[0]_inv_i_4_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm1[1]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm1[1]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm1[1]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm2[0]_inv_i_1_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm2[0]_inv_i_2_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm2[0]_inv_i_3_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm2[0]_inv_i_4_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm2[1]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm2[1]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_sequential_push_stm2[1]_i_3_n_0\ : STD_LOGIC;
  signal bit_sync_cntr : STD_LOGIC;
  signal \bit_sync_cntr[0]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[10]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[11]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[12]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[13]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[14]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[15]_i_2_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[1]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[2]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[3]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[4]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[5]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[6]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[7]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[8]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr[9]_i_1_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[12]_i_2_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[12]_i_2_n_1\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[12]_i_2_n_2\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[12]_i_2_n_3\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[12]_i_2_n_4\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[12]_i_2_n_5\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[12]_i_2_n_6\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[12]_i_2_n_7\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[15]_i_3_n_2\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[15]_i_3_n_3\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[15]_i_3_n_5\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[15]_i_3_n_6\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[15]_i_3_n_7\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[4]_i_2_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[4]_i_2_n_1\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[4]_i_2_n_2\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[4]_i_2_n_3\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[4]_i_2_n_4\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[4]_i_2_n_5\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[4]_i_2_n_6\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[4]_i_2_n_7\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[8]_i_2_n_0\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[8]_i_2_n_1\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[8]_i_2_n_2\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[8]_i_2_n_3\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[8]_i_2_n_4\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[8]_i_2_n_5\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[8]_i_2_n_6\ : STD_LOGIC;
  signal \bit_sync_cntr_reg[8]_i_2_n_7\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[0]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[10]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[11]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[12]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[13]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[14]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[15]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[1]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[2]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[3]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[4]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[5]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[6]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[7]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[8]\ : STD_LOGIC;
  signal \bit_sync_cntr_reg_n_0_[9]\ : STD_LOGIC;
  signal data0 : STD_LOGIC_VECTOR ( 15 downto 1 );
  signal fifo_din1 : STD_LOGIC;
  signal \fifo_din1_reg_n_0_[0]\ : STD_LOGIC;
  signal \fifo_din1_reg_n_0_[1]\ : STD_LOGIC;
  signal \fifo_din1_reg_n_0_[2]\ : STD_LOGIC;
  signal \fifo_din1_reg_n_0_[3]\ : STD_LOGIC;
  signal \fifo_din1_reg_n_0_[4]\ : STD_LOGIC;
  signal \fifo_din1_reg_n_0_[5]\ : STD_LOGIC;
  signal \fifo_din1_reg_n_0_[6]\ : STD_LOGIC;
  signal \fifo_din1_reg_n_0_[7]\ : STD_LOGIC;
  signal fifo_din2 : STD_LOGIC;
  signal \fifo_din2_reg_n_0_[0]\ : STD_LOGIC;
  signal \fifo_din2_reg_n_0_[1]\ : STD_LOGIC;
  signal \fifo_din2_reg_n_0_[2]\ : STD_LOGIC;
  signal \fifo_din2_reg_n_0_[3]\ : STD_LOGIC;
  signal \fifo_din2_reg_n_0_[4]\ : STD_LOGIC;
  signal \fifo_din2_reg_n_0_[5]\ : STD_LOGIC;
  signal \fifo_din2_reg_n_0_[6]\ : STD_LOGIC;
  signal \fifo_din2_reg_n_0_[7]\ : STD_LOGIC;
  signal fifo_dout1 : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal fifo_dout2 : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal fifo_rd_en1_i_1_n_0 : STD_LOGIC;
  signal fifo_rd_en1_reg_n_0 : STD_LOGIC;
  signal fifo_rd_en2_i_1_n_0 : STD_LOGIC;
  signal fifo_rd_en2_reg_n_0 : STD_LOGIC;
  signal fifo_wr_en1_reg_n_0 : STD_LOGIC;
  signal fifo_wr_en2_reg_n_0 : STD_LOGIC;
  signal \get_data1[0]_i_1_n_0\ : STD_LOGIC;
  signal \get_data1[1]_i_1_n_0\ : STD_LOGIC;
  signal \get_data1[2]_i_1_n_0\ : STD_LOGIC;
  signal \get_data1[3]_i_1_n_0\ : STD_LOGIC;
  signal \get_data1[4]_i_1_n_0\ : STD_LOGIC;
  signal \get_data1[5]_i_1_n_0\ : STD_LOGIC;
  signal \get_data1[6]_i_1_n_0\ : STD_LOGIC;
  signal \get_data1[7]_i_1_n_0\ : STD_LOGIC;
  signal \get_data1[7]_i_2_n_0\ : STD_LOGIC;
  signal \get_data2[0]_i_1_n_0\ : STD_LOGIC;
  signal \get_data2[1]_i_1_n_0\ : STD_LOGIC;
  signal \get_data2[2]_i_1_n_0\ : STD_LOGIC;
  signal \get_data2[3]_i_1_n_0\ : STD_LOGIC;
  signal \get_data2[4]_i_1_n_0\ : STD_LOGIC;
  signal \get_data2[5]_i_1_n_0\ : STD_LOGIC;
  signal \get_data2[6]_i_1_n_0\ : STD_LOGIC;
  signal \get_data2[7]_i_1_n_0\ : STD_LOGIC;
  signal in4 : STD_LOGIC_VECTOR ( 15 downto 1 );
  signal pattern_cntr : STD_LOGIC;
  signal \pattern_cntr1[0]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[10]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[11]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[12]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[13]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[14]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[15]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[15]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[15]_i_3_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[15]_i_4_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[15]_i_5_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[15]_i_6_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[1]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[2]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[3]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[4]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[5]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[6]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[7]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[8]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1[9]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr1_reg[12]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr1_reg[12]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr1_reg[12]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr1_reg[12]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr1_reg[12]_i_2_n_4\ : STD_LOGIC;
  signal \pattern_cntr1_reg[12]_i_2_n_5\ : STD_LOGIC;
  signal \pattern_cntr1_reg[12]_i_2_n_6\ : STD_LOGIC;
  signal \pattern_cntr1_reg[12]_i_2_n_7\ : STD_LOGIC;
  signal \pattern_cntr1_reg[15]_i_7_n_2\ : STD_LOGIC;
  signal \pattern_cntr1_reg[15]_i_7_n_3\ : STD_LOGIC;
  signal \pattern_cntr1_reg[15]_i_7_n_5\ : STD_LOGIC;
  signal \pattern_cntr1_reg[15]_i_7_n_6\ : STD_LOGIC;
  signal \pattern_cntr1_reg[15]_i_7_n_7\ : STD_LOGIC;
  signal \pattern_cntr1_reg[4]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr1_reg[4]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr1_reg[4]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr1_reg[4]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr1_reg[4]_i_2_n_4\ : STD_LOGIC;
  signal \pattern_cntr1_reg[4]_i_2_n_5\ : STD_LOGIC;
  signal \pattern_cntr1_reg[4]_i_2_n_6\ : STD_LOGIC;
  signal \pattern_cntr1_reg[4]_i_2_n_7\ : STD_LOGIC;
  signal \pattern_cntr1_reg[8]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr1_reg[8]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr1_reg[8]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr1_reg[8]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr1_reg[8]_i_2_n_4\ : STD_LOGIC;
  signal \pattern_cntr1_reg[8]_i_2_n_5\ : STD_LOGIC;
  signal \pattern_cntr1_reg[8]_i_2_n_6\ : STD_LOGIC;
  signal \pattern_cntr1_reg[8]_i_2_n_7\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[0]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[10]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[11]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[12]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[13]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[14]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[15]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[1]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[2]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[3]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[4]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[5]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[6]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[7]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[8]\ : STD_LOGIC;
  signal \pattern_cntr1_reg_n_0_[9]\ : STD_LOGIC;
  signal \pattern_cntr2[0]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[10]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[11]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[12]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[13]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[14]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[15]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[15]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[15]_i_3_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[15]_i_4_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[15]_i_5_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[15]_i_6_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[1]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[2]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[3]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[4]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[5]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[6]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[7]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[8]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2[9]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr2_reg[12]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr2_reg[12]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr2_reg[12]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr2_reg[12]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr2_reg[15]_i_7_n_2\ : STD_LOGIC;
  signal \pattern_cntr2_reg[15]_i_7_n_3\ : STD_LOGIC;
  signal \pattern_cntr2_reg[4]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr2_reg[4]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr2_reg[4]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr2_reg[4]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr2_reg[8]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr2_reg[8]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr2_reg[8]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr2_reg[8]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[0]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[10]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[11]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[12]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[13]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[14]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[15]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[1]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[2]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[3]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[4]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[5]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[6]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[7]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[8]\ : STD_LOGIC;
  signal \pattern_cntr2_reg_n_0_[9]\ : STD_LOGIC;
  signal \pattern_cntr[0]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[10]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[11]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[12]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[13]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[14]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[15]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr[1]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[2]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[3]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[4]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[5]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[6]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[7]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[8]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr[9]_i_1_n_0\ : STD_LOGIC;
  signal \pattern_cntr_reg[12]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr_reg[12]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr_reg[12]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr_reg[12]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr_reg[15]_i_3_n_2\ : STD_LOGIC;
  signal \pattern_cntr_reg[15]_i_3_n_3\ : STD_LOGIC;
  signal \pattern_cntr_reg[4]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr_reg[4]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr_reg[4]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr_reg[4]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr_reg[8]_i_2_n_0\ : STD_LOGIC;
  signal \pattern_cntr_reg[8]_i_2_n_1\ : STD_LOGIC;
  signal \pattern_cntr_reg[8]_i_2_n_2\ : STD_LOGIC;
  signal \pattern_cntr_reg[8]_i_2_n_3\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[0]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[10]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[11]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[12]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[13]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[14]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[15]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[1]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[2]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[3]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[4]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[5]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[6]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[7]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[8]\ : STD_LOGIC;
  signal \pattern_cntr_reg_n_0_[9]\ : STD_LOGIC;
  signal push_now : STD_LOGIC;
  signal push_now_i_1_n_0 : STD_LOGIC;
  signal push_stm1 : STD_LOGIC_VECTOR ( 0 to 0 );
  signal push_stm2 : STD_LOGIC_VECTOR ( 0 to 0 );
  signal send_done1 : STD_LOGIC;
  signal \send_done1__0\ : STD_LOGIC;
  signal send_done1_i_1_n_0 : STD_LOGIC;
  signal send_done2 : STD_LOGIC;
  signal \send_done2__0\ : STD_LOGIC;
  signal send_done2_i_1_n_0 : STD_LOGIC;
  signal sync_push_now_0 : STD_LOGIC;
  attribute async_reg : string;
  attribute async_reg of sync_push_now_0 : signal is "true";
  signal sync_push_now_1 : STD_LOGIC;
  attribute async_reg of sync_push_now_1 : signal is "true";
  signal sync_send_done1_0 : STD_LOGIC;
  attribute async_reg of sync_send_done1_0 : signal is "true";
  signal sync_send_done1_1 : STD_LOGIC;
  attribute async_reg of sync_send_done1_1 : signal is "true";
  signal sync_send_done2_0 : STD_LOGIC;
  attribute async_reg of sync_send_done2_0 : signal is "true";
  signal sync_send_done2_1 : STD_LOGIC;
  attribute async_reg of sync_send_done2_1 : signal is "true";
  signal \NLW_bit_sync_cntr_reg[15]_i_3_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_bit_sync_cntr_reg[15]_i_3_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal NLW_dut1_empty_UNCONNECTED : STD_LOGIC;
  signal NLW_dut1_full_UNCONNECTED : STD_LOGIC;
  signal NLW_dut1_rd_rst_busy_UNCONNECTED : STD_LOGIC;
  signal NLW_dut1_wr_rst_busy_UNCONNECTED : STD_LOGIC;
  signal NLW_dut1_dout_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 8 );
  signal NLW_dut2_empty_UNCONNECTED : STD_LOGIC;
  signal NLW_dut2_full_UNCONNECTED : STD_LOGIC;
  signal NLW_dut2_rd_rst_busy_UNCONNECTED : STD_LOGIC;
  signal NLW_dut2_wr_rst_busy_UNCONNECTED : STD_LOGIC;
  signal NLW_dut2_dout_UNCONNECTED : STD_LOGIC_VECTOR ( 31 downto 8 );
  signal \NLW_pattern_cntr1_reg[15]_i_7_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_pattern_cntr1_reg[15]_i_7_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_pattern_cntr2_reg[15]_i_7_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_pattern_cntr2_reg[15]_i_7_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_pattern_cntr_reg[15]_i_3_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_pattern_cntr_reg[15]_i_3_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_onehot_acc_state1[0]_i_2\ : label is "soft_lutpair22";
  attribute FSM_ENCODED_STATES : string;
  attribute FSM_ENCODED_STATES of \FSM_onehot_acc_state1_reg[0]\ : label is "iSTATE:001,iSTATE0:010,iSTATE1:100,";
  attribute FSM_ENCODED_STATES of \FSM_onehot_acc_state1_reg[1]\ : label is "iSTATE:001,iSTATE0:010,iSTATE1:100,";
  attribute FSM_ENCODED_STATES of \FSM_onehot_acc_state1_reg[2]\ : label is "iSTATE:001,iSTATE0:010,iSTATE1:100,";
  attribute SOFT_HLUTNM of \FSM_onehot_acc_state2[0]_i_2\ : label is "soft_lutpair22";
  attribute FSM_ENCODED_STATES of \FSM_onehot_acc_state2_reg[0]\ : label is "iSTATE:001,iSTATE0:010,iSTATE1:100,";
  attribute FSM_ENCODED_STATES of \FSM_onehot_acc_state2_reg[1]\ : label is "iSTATE:001,iSTATE0:010,iSTATE1:100,";
  attribute FSM_ENCODED_STATES of \FSM_onehot_acc_state2_reg[2]\ : label is "iSTATE:001,iSTATE0:010,iSTATE1:100,";
  attribute SOFT_HLUTNM of \FSM_sequential_push_stm1[0]_inv_i_4\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \FSM_sequential_push_stm1[1]_i_1\ : label is "soft_lutpair2";
  attribute FSM_ENCODED_STATES of \FSM_sequential_push_stm1_reg[0]_inv\ : label is "iSTATE:00,iSTATE0:01,iSTATE1:10,iSTATE2:11";
  attribute inverted : string;
  attribute inverted of \FSM_sequential_push_stm1_reg[0]_inv\ : label is "yes";
  attribute FSM_ENCODED_STATES of \FSM_sequential_push_stm1_reg[1]\ : label is "iSTATE:00,iSTATE0:01,iSTATE1:10,iSTATE2:11";
  attribute SOFT_HLUTNM of \FSM_sequential_push_stm2[0]_inv_i_4\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \FSM_sequential_push_stm2[1]_i_1\ : label is "soft_lutpair3";
  attribute FSM_ENCODED_STATES of \FSM_sequential_push_stm2_reg[0]_inv\ : label is "iSTATE:00,iSTATE0:01,iSTATE1:10,iSTATE2:11";
  attribute inverted of \FSM_sequential_push_stm2_reg[0]_inv\ : label is "yes";
  attribute FSM_ENCODED_STATES of \FSM_sequential_push_stm2_reg[1]\ : label is "iSTATE:00,iSTATE0:01,iSTATE1:10,iSTATE2:11";
  attribute SOFT_HLUTNM of \bit_sync_cntr[0]_i_1\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \bit_sync_cntr[10]_i_1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \bit_sync_cntr[11]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \bit_sync_cntr[12]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \bit_sync_cntr[13]_i_1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \bit_sync_cntr[14]_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \bit_sync_cntr[15]_i_2\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \bit_sync_cntr[1]_i_1\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \bit_sync_cntr[2]_i_1\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \bit_sync_cntr[3]_i_1\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \bit_sync_cntr[4]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \bit_sync_cntr[5]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \bit_sync_cntr[6]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \bit_sync_cntr[7]_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \bit_sync_cntr[8]_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \bit_sync_cntr[9]_i_1\ : label is "soft_lutpair13";
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \bit_sync_cntr_reg[12]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \bit_sync_cntr_reg[15]_i_3\ : label is 35;
  attribute ADDER_THRESHOLD of \bit_sync_cntr_reg[4]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \bit_sync_cntr_reg[8]_i_2\ : label is 35;
  attribute x_core_info : string;
  attribute x_core_info of dut1 : label is "fifo_generator_v13_2_8,Vivado 2023.1";
  attribute x_core_info of dut2 : label is "fifo_generator_v13_2_8,Vivado 2023.1";
  attribute SOFT_HLUTNM of \get_data1[0]_i_1\ : label is "soft_lutpair26";
  attribute SOFT_HLUTNM of \get_data1[1]_i_1\ : label is "soft_lutpair26";
  attribute SOFT_HLUTNM of \get_data1[2]_i_1\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \get_data1[3]_i_1\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \get_data1[4]_i_1\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \get_data1[5]_i_1\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \get_data1[6]_i_1\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \get_data1[7]_i_1\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \get_data2[0]_i_1\ : label is "soft_lutpair30";
  attribute SOFT_HLUTNM of \get_data2[1]_i_1\ : label is "soft_lutpair30";
  attribute SOFT_HLUTNM of \get_data2[2]_i_1\ : label is "soft_lutpair29";
  attribute SOFT_HLUTNM of \get_data2[3]_i_1\ : label is "soft_lutpair29";
  attribute SOFT_HLUTNM of \get_data2[4]_i_1\ : label is "soft_lutpair28";
  attribute SOFT_HLUTNM of \get_data2[5]_i_1\ : label is "soft_lutpair28";
  attribute SOFT_HLUTNM of \get_data2[6]_i_1\ : label is "soft_lutpair27";
  attribute SOFT_HLUTNM of \get_data2[7]_i_1\ : label is "soft_lutpair27";
  attribute SOFT_HLUTNM of \pattern_cntr1[0]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \pattern_cntr1[15]_i_3\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \pattern_cntr1[15]_i_6\ : label is "soft_lutpair1";
  attribute ADDER_THRESHOLD of \pattern_cntr1_reg[12]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr1_reg[15]_i_7\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr1_reg[4]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr1_reg[8]_i_2\ : label is 35;
  attribute SOFT_HLUTNM of \pattern_cntr2[0]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \pattern_cntr2[15]_i_3\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \pattern_cntr2[15]_i_6\ : label is "soft_lutpair0";
  attribute ADDER_THRESHOLD of \pattern_cntr2_reg[12]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr2_reg[15]_i_7\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr2_reg[4]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr2_reg[8]_i_2\ : label is 35;
  attribute SOFT_HLUTNM of \pattern_cntr[0]_i_1\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \pattern_cntr[10]_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \pattern_cntr[11]_i_1\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \pattern_cntr[12]_i_1\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \pattern_cntr[13]_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \pattern_cntr[14]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \pattern_cntr[15]_i_2\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \pattern_cntr[1]_i_1\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \pattern_cntr[2]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \pattern_cntr[3]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \pattern_cntr[4]_i_1\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \pattern_cntr[5]_i_1\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \pattern_cntr[6]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \pattern_cntr[7]_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \pattern_cntr[8]_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \pattern_cntr[9]_i_1\ : label is "soft_lutpair12";
  attribute ADDER_THRESHOLD of \pattern_cntr_reg[12]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr_reg[15]_i_3\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr_reg[4]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \pattern_cntr_reg[8]_i_2\ : label is 35;
  attribute SOFT_HLUTNM of send_done1_i_1 : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of send_done2_i_1 : label is "soft_lutpair3";
  attribute ASYNC_REG_boolean : boolean;
  attribute ASYNC_REG_boolean of sync_push_now_0_reg : label is std.standard.true;
  attribute KEEP : string;
  attribute KEEP of sync_push_now_0_reg : label is "yes";
  attribute ASYNC_REG_boolean of sync_push_now_1_reg : label is std.standard.true;
  attribute KEEP of sync_push_now_1_reg : label is "yes";
  attribute ASYNC_REG_boolean of sync_send_done1_0_reg : label is std.standard.true;
  attribute KEEP of sync_send_done1_0_reg : label is "yes";
  attribute ASYNC_REG_boolean of sync_send_done1_1_reg : label is std.standard.true;
  attribute KEEP of sync_send_done1_1_reg : label is "yes";
  attribute ASYNC_REG_boolean of sync_send_done2_0_reg : label is std.standard.true;
  attribute KEEP of sync_send_done2_0_reg : label is "yes";
  attribute ASYNC_REG_boolean of sync_send_done2_1_reg : label is std.standard.true;
  attribute KEEP of sync_send_done2_1_reg : label is "yes";
begin
\FSM_onehot_acc_state1[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"CCCFCFCFCCC8C8C8"
    )
        port map (
      I0 => sync_send_done1_1,
      I1 => \FSM_onehot_acc_state1_reg_n_0_[2]\,
      I2 => \FSM_onehot_acc_state1[0]_i_2_n_0\,
      I3 => \FSM_onehot_acc_state1[2]_i_2_n_0\,
      I4 => \FSM_onehot_acc_state1[2]_i_3_n_0\,
      I5 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      O => \FSM_onehot_acc_state1[0]_i_1_n_0\
    );
\FSM_onehot_acc_state1[0]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => push_now,
      I1 => \FSM_onehot_acc_state1_reg_n_0_[1]\,
      O => \FSM_onehot_acc_state1[0]_i_2_n_0\
    );
\FSM_onehot_acc_state1[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"ABBBBBBBA8888888"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => \FSM_onehot_acc_state1[1]_i_2_n_0\,
      I2 => \FSM_onehot_acc_state1[2]_i_2_n_0\,
      I3 => \FSM_onehot_acc_state1[1]_i_3_n_0\,
      I4 => \FSM_onehot_acc_state1[1]_i_4_n_0\,
      I5 => \FSM_onehot_acc_state1_reg_n_0_[1]\,
      O => \FSM_onehot_acc_state1[1]_i_1_n_0\
    );
\FSM_onehot_acc_state1[1]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F888"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[1]\,
      I1 => push_now,
      I2 => \FSM_onehot_acc_state1_reg_n_0_[2]\,
      I3 => sync_send_done1_1,
      O => \FSM_onehot_acc_state1[1]_i_2_n_0\
    );
\FSM_onehot_acc_state1[1]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80000000"
    )
        port map (
      I0 => \pattern_cntr_reg_n_0_[0]\,
      I1 => \pattern_cntr_reg_n_0_[1]\,
      I2 => \pattern_cntr_reg_n_0_[2]\,
      I3 => \pattern_cntr_reg_n_0_[4]\,
      I4 => \pattern_cntr_reg_n_0_[3]\,
      O => \FSM_onehot_acc_state1[1]_i_3_n_0\
    );
\FSM_onehot_acc_state1[1]_i_4\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000008000"
    )
        port map (
      I0 => \pattern_cntr_reg_n_0_[7]\,
      I1 => \pattern_cntr_reg_n_0_[8]\,
      I2 => \pattern_cntr_reg_n_0_[5]\,
      I3 => \pattern_cntr_reg_n_0_[6]\,
      I4 => \pattern_cntr_reg_n_0_[10]\,
      I5 => \pattern_cntr_reg_n_0_[9]\,
      O => \FSM_onehot_acc_state1[1]_i_4_n_0\
    );
\FSM_onehot_acc_state1[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FF00FC44FC44FC44"
    )
        port map (
      I0 => sync_send_done1_1,
      I1 => \FSM_onehot_acc_state1_reg_n_0_[2]\,
      I2 => push_now,
      I3 => \FSM_onehot_acc_state1_reg_n_0_[1]\,
      I4 => \FSM_onehot_acc_state1[2]_i_2_n_0\,
      I5 => \FSM_onehot_acc_state1[2]_i_3_n_0\,
      O => \FSM_onehot_acc_state1[2]_i_1_n_0\
    );
\FSM_onehot_acc_state1[2]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000100000000"
    )
        port map (
      I0 => \pattern_cntr_reg_n_0_[13]\,
      I1 => \pattern_cntr_reg_n_0_[14]\,
      I2 => \pattern_cntr_reg_n_0_[11]\,
      I3 => \pattern_cntr_reg_n_0_[12]\,
      I4 => \pattern_cntr_reg_n_0_[15]\,
      I5 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      O => \FSM_onehot_acc_state1[2]_i_2_n_0\
    );
\FSM_onehot_acc_state1[2]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8000000000000000"
    )
        port map (
      I0 => \pattern_cntr_reg_n_0_[3]\,
      I1 => \pattern_cntr_reg_n_0_[4]\,
      I2 => \pattern_cntr_reg_n_0_[2]\,
      I3 => \pattern_cntr_reg_n_0_[1]\,
      I4 => \pattern_cntr_reg_n_0_[0]\,
      I5 => \FSM_onehot_acc_state1[1]_i_4_n_0\,
      O => \FSM_onehot_acc_state1[2]_i_3_n_0\
    );
\FSM_onehot_acc_state1_reg[0]\: unisim.vcomponents.FDPE
    generic map(
      INIT => '1'
    )
        port map (
      C => clock_61,
      CE => '1',
      D => \FSM_onehot_acc_state1[0]_i_1_n_0\,
      PRE => \get_data1[7]_i_2_n_0\,
      Q => \FSM_onehot_acc_state1_reg_n_0_[0]\
    );
\FSM_onehot_acc_state1_reg[1]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => \FSM_onehot_acc_state1[1]_i_1_n_0\,
      Q => \FSM_onehot_acc_state1_reg_n_0_[1]\
    );
\FSM_onehot_acc_state1_reg[2]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => \FSM_onehot_acc_state1[2]_i_1_n_0\,
      Q => \FSM_onehot_acc_state1_reg_n_0_[2]\
    );
\FSM_onehot_acc_state2[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"CCCFCFCFCCC8C8C8"
    )
        port map (
      I0 => sync_send_done2_1,
      I1 => \FSM_onehot_acc_state2_reg_n_0_[2]\,
      I2 => \FSM_onehot_acc_state2[0]_i_2_n_0\,
      I3 => \FSM_onehot_acc_state2[2]_i_2_n_0\,
      I4 => \FSM_onehot_acc_state2[2]_i_3_n_0\,
      I5 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      O => \FSM_onehot_acc_state2[0]_i_1_n_0\
    );
\FSM_onehot_acc_state2[0]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => push_now,
      I1 => \FSM_onehot_acc_state2_reg_n_0_[1]\,
      O => \FSM_onehot_acc_state2[0]_i_2_n_0\
    );
\FSM_onehot_acc_state2[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"ABBBBBBBA8888888"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \FSM_onehot_acc_state2[1]_i_2_n_0\,
      I2 => \FSM_onehot_acc_state2[2]_i_2_n_0\,
      I3 => \FSM_onehot_acc_state2[1]_i_3_n_0\,
      I4 => \FSM_onehot_acc_state2[1]_i_4_n_0\,
      I5 => \FSM_onehot_acc_state2_reg_n_0_[1]\,
      O => \FSM_onehot_acc_state2[1]_i_1_n_0\
    );
\FSM_onehot_acc_state2[1]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F888"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[1]\,
      I1 => push_now,
      I2 => \FSM_onehot_acc_state2_reg_n_0_[2]\,
      I3 => sync_send_done2_1,
      O => \FSM_onehot_acc_state2[1]_i_2_n_0\
    );
\FSM_onehot_acc_state2[1]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80000000"
    )
        port map (
      I0 => \bit_sync_cntr_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg_n_0_[1]\,
      I2 => \bit_sync_cntr_reg_n_0_[2]\,
      I3 => \bit_sync_cntr_reg_n_0_[4]\,
      I4 => \bit_sync_cntr_reg_n_0_[3]\,
      O => \FSM_onehot_acc_state2[1]_i_3_n_0\
    );
\FSM_onehot_acc_state2[1]_i_4\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000008000"
    )
        port map (
      I0 => \bit_sync_cntr_reg_n_0_[7]\,
      I1 => \bit_sync_cntr_reg_n_0_[8]\,
      I2 => \bit_sync_cntr_reg_n_0_[5]\,
      I3 => \bit_sync_cntr_reg_n_0_[6]\,
      I4 => \bit_sync_cntr_reg_n_0_[10]\,
      I5 => \bit_sync_cntr_reg_n_0_[9]\,
      O => \FSM_onehot_acc_state2[1]_i_4_n_0\
    );
\FSM_onehot_acc_state2[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FF00FC44FC44FC44"
    )
        port map (
      I0 => sync_send_done2_1,
      I1 => \FSM_onehot_acc_state2_reg_n_0_[2]\,
      I2 => push_now,
      I3 => \FSM_onehot_acc_state2_reg_n_0_[1]\,
      I4 => \FSM_onehot_acc_state2[2]_i_2_n_0\,
      I5 => \FSM_onehot_acc_state2[2]_i_3_n_0\,
      O => \FSM_onehot_acc_state2[2]_i_1_n_0\
    );
\FSM_onehot_acc_state2[2]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000100000000"
    )
        port map (
      I0 => \bit_sync_cntr_reg_n_0_[13]\,
      I1 => \bit_sync_cntr_reg_n_0_[14]\,
      I2 => \bit_sync_cntr_reg_n_0_[11]\,
      I3 => \bit_sync_cntr_reg_n_0_[12]\,
      I4 => \bit_sync_cntr_reg_n_0_[15]\,
      I5 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      O => \FSM_onehot_acc_state2[2]_i_2_n_0\
    );
\FSM_onehot_acc_state2[2]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8000000000000000"
    )
        port map (
      I0 => \bit_sync_cntr_reg_n_0_[3]\,
      I1 => \bit_sync_cntr_reg_n_0_[4]\,
      I2 => \bit_sync_cntr_reg_n_0_[2]\,
      I3 => \bit_sync_cntr_reg_n_0_[1]\,
      I4 => \bit_sync_cntr_reg_n_0_[0]\,
      I5 => \FSM_onehot_acc_state2[1]_i_4_n_0\,
      O => \FSM_onehot_acc_state2[2]_i_3_n_0\
    );
\FSM_onehot_acc_state2_reg[0]\: unisim.vcomponents.FDPE
    generic map(
      INIT => '1'
    )
        port map (
      C => clock_61,
      CE => '1',
      D => \FSM_onehot_acc_state2[0]_i_1_n_0\,
      PRE => \get_data1[7]_i_2_n_0\,
      Q => \FSM_onehot_acc_state2_reg_n_0_[0]\
    );
\FSM_onehot_acc_state2_reg[1]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => \FSM_onehot_acc_state2[1]_i_1_n_0\,
      Q => \FSM_onehot_acc_state2_reg_n_0_[1]\
    );
\FSM_onehot_acc_state2_reg[2]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => \FSM_onehot_acc_state2[2]_i_1_n_0\,
      Q => \FSM_onehot_acc_state2_reg_n_0_[2]\
    );
\FSM_sequential_push_stm1[0]_inv_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00D1FF00"
    )
        port map (
      I0 => sync_push_now_1,
      I1 => send_done1,
      I2 => \FSM_sequential_push_stm1[0]_inv_i_2_n_0\,
      I3 => \FSM_sequential_push_stm1[1]_i_2_n_0\,
      I4 => push_stm1(0),
      O => \FSM_sequential_push_stm1[0]_inv_i_1_n_0\
    );
\FSM_sequential_push_stm1[0]_inv_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \FSM_sequential_push_stm1[0]_inv_i_3_n_0\,
      I1 => \FSM_sequential_push_stm1[0]_inv_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_4_n_0\,
      I3 => \pattern_cntr1[15]_i_3_n_0\,
      O => \FSM_sequential_push_stm1[0]_inv_i_2_n_0\
    );
\FSM_sequential_push_stm1[0]_inv_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => \pattern_cntr1_reg_n_0_[6]\,
      I1 => \pattern_cntr1_reg_n_0_[5]\,
      I2 => \pattern_cntr1_reg_n_0_[8]\,
      I3 => \pattern_cntr1_reg_n_0_[7]\,
      O => \FSM_sequential_push_stm1[0]_inv_i_3_n_0\
    );
\FSM_sequential_push_stm1[0]_inv_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => \pattern_cntr1_reg_n_0_[2]\,
      I1 => \pattern_cntr1_reg_n_0_[1]\,
      I2 => \pattern_cntr1_reg_n_0_[4]\,
      I3 => \pattern_cntr1_reg_n_0_[3]\,
      O => \FSM_sequential_push_stm1[0]_inv_i_4_n_0\
    );
\FSM_sequential_push_stm1[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"D2"
    )
        port map (
      I0 => \FSM_sequential_push_stm1[1]_i_2_n_0\,
      I1 => push_stm1(0),
      I2 => send_done1,
      O => \FSM_sequential_push_stm1[1]_i_1_n_0\
    );
\FSM_sequential_push_stm1[1]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000100000000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1_reg_n_0_[2]\,
      I3 => push_stm1(0),
      I4 => \pattern_cntr1_reg_n_0_[1]\,
      I5 => \FSM_sequential_push_stm1[1]_i_3_n_0\,
      O => \FSM_sequential_push_stm1[1]_i_2_n_0\
    );
\FSM_sequential_push_stm1[1]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000001"
    )
        port map (
      I0 => \pattern_cntr1_reg_n_0_[5]\,
      I1 => \pattern_cntr1_reg_n_0_[6]\,
      I2 => \pattern_cntr1_reg_n_0_[3]\,
      I3 => \pattern_cntr1_reg_n_0_[4]\,
      I4 => \pattern_cntr1_reg_n_0_[8]\,
      I5 => \pattern_cntr1_reg_n_0_[7]\,
      O => \FSM_sequential_push_stm1[1]_i_3_n_0\
    );
\FSM_sequential_push_stm1_reg[0]_inv\: unisim.vcomponents.FDPE
     port map (
      C => clock_100,
      CE => '1',
      D => \FSM_sequential_push_stm1[0]_inv_i_1_n_0\,
      PRE => \get_data1[7]_i_2_n_0\,
      Q => push_stm1(0)
    );
\FSM_sequential_push_stm1_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => \FSM_sequential_push_stm1[1]_i_1_n_0\,
      Q => send_done1
    );
\FSM_sequential_push_stm2[0]_inv_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00D1FF00"
    )
        port map (
      I0 => sync_push_now_1,
      I1 => send_done2,
      I2 => \FSM_sequential_push_stm2[0]_inv_i_2_n_0\,
      I3 => \FSM_sequential_push_stm2[1]_i_2_n_0\,
      I4 => push_stm2(0),
      O => \FSM_sequential_push_stm2[0]_inv_i_1_n_0\
    );
\FSM_sequential_push_stm2[0]_inv_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \FSM_sequential_push_stm2[0]_inv_i_3_n_0\,
      I1 => \FSM_sequential_push_stm2[0]_inv_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_4_n_0\,
      I3 => \pattern_cntr2[15]_i_3_n_0\,
      O => \FSM_sequential_push_stm2[0]_inv_i_2_n_0\
    );
\FSM_sequential_push_stm2[0]_inv_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => \pattern_cntr2_reg_n_0_[6]\,
      I1 => \pattern_cntr2_reg_n_0_[5]\,
      I2 => \pattern_cntr2_reg_n_0_[8]\,
      I3 => \pattern_cntr2_reg_n_0_[7]\,
      O => \FSM_sequential_push_stm2[0]_inv_i_3_n_0\
    );
\FSM_sequential_push_stm2[0]_inv_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => \pattern_cntr2_reg_n_0_[2]\,
      I1 => \pattern_cntr2_reg_n_0_[1]\,
      I2 => \pattern_cntr2_reg_n_0_[4]\,
      I3 => \pattern_cntr2_reg_n_0_[3]\,
      O => \FSM_sequential_push_stm2[0]_inv_i_4_n_0\
    );
\FSM_sequential_push_stm2[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"D2"
    )
        port map (
      I0 => \FSM_sequential_push_stm2[1]_i_2_n_0\,
      I1 => push_stm2(0),
      I2 => send_done2,
      O => \FSM_sequential_push_stm2[1]_i_1_n_0\
    );
\FSM_sequential_push_stm2[1]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000100000000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2_reg_n_0_[2]\,
      I3 => push_stm2(0),
      I4 => \pattern_cntr2_reg_n_0_[1]\,
      I5 => \FSM_sequential_push_stm2[1]_i_3_n_0\,
      O => \FSM_sequential_push_stm2[1]_i_2_n_0\
    );
\FSM_sequential_push_stm2[1]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000001"
    )
        port map (
      I0 => \pattern_cntr2_reg_n_0_[5]\,
      I1 => \pattern_cntr2_reg_n_0_[6]\,
      I2 => \pattern_cntr2_reg_n_0_[3]\,
      I3 => \pattern_cntr2_reg_n_0_[4]\,
      I4 => \pattern_cntr2_reg_n_0_[8]\,
      I5 => \pattern_cntr2_reg_n_0_[7]\,
      O => \FSM_sequential_push_stm2[1]_i_3_n_0\
    );
\FSM_sequential_push_stm2_reg[0]_inv\: unisim.vcomponents.FDPE
     port map (
      C => clock_100,
      CE => '1',
      D => \FSM_sequential_push_stm2[0]_inv_i_1_n_0\,
      PRE => \get_data1[7]_i_2_n_0\,
      Q => push_stm2(0)
    );
\FSM_sequential_push_stm2_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => \FSM_sequential_push_stm2[1]_i_1_n_0\,
      Q => send_done2
    );
\bit_sync_cntr[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg_n_0_[0]\,
      O => \bit_sync_cntr[0]_i_1_n_0\
    );
\bit_sync_cntr[10]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[12]_i_2_n_6\,
      O => \bit_sync_cntr[10]_i_1_n_0\
    );
\bit_sync_cntr[11]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[12]_i_2_n_5\,
      O => \bit_sync_cntr[11]_i_1_n_0\
    );
\bit_sync_cntr[12]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[12]_i_2_n_4\,
      O => \bit_sync_cntr[12]_i_1_n_0\
    );
\bit_sync_cntr[13]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[15]_i_3_n_7\,
      O => \bit_sync_cntr[13]_i_1_n_0\
    );
\bit_sync_cntr[14]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[15]_i_3_n_6\,
      O => \bit_sync_cntr[14]_i_1_n_0\
    );
\bit_sync_cntr[15]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F888"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[1]\,
      I1 => push_now,
      I2 => bsync_val,
      I3 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      O => bit_sync_cntr
    );
\bit_sync_cntr[15]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[15]_i_3_n_5\,
      O => \bit_sync_cntr[15]_i_2_n_0\
    );
\bit_sync_cntr[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[4]_i_2_n_7\,
      O => \bit_sync_cntr[1]_i_1_n_0\
    );
\bit_sync_cntr[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[4]_i_2_n_6\,
      O => \bit_sync_cntr[2]_i_1_n_0\
    );
\bit_sync_cntr[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[4]_i_2_n_5\,
      O => \bit_sync_cntr[3]_i_1_n_0\
    );
\bit_sync_cntr[4]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[4]_i_2_n_4\,
      O => \bit_sync_cntr[4]_i_1_n_0\
    );
\bit_sync_cntr[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[8]_i_2_n_7\,
      O => \bit_sync_cntr[5]_i_1_n_0\
    );
\bit_sync_cntr[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[8]_i_2_n_6\,
      O => \bit_sync_cntr[6]_i_1_n_0\
    );
\bit_sync_cntr[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[8]_i_2_n_5\,
      O => \bit_sync_cntr[7]_i_1_n_0\
    );
\bit_sync_cntr[8]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[8]_i_2_n_4\,
      O => \bit_sync_cntr[8]_i_1_n_0\
    );
\bit_sync_cntr[9]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => \bit_sync_cntr_reg[12]_i_2_n_7\,
      O => \bit_sync_cntr[9]_i_1_n_0\
    );
\bit_sync_cntr_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[0]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[0]\
    );
\bit_sync_cntr_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[10]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[10]\
    );
\bit_sync_cntr_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[11]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[11]\
    );
\bit_sync_cntr_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[12]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[12]\
    );
\bit_sync_cntr_reg[12]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \bit_sync_cntr_reg[8]_i_2_n_0\,
      CO(3) => \bit_sync_cntr_reg[12]_i_2_n_0\,
      CO(2) => \bit_sync_cntr_reg[12]_i_2_n_1\,
      CO(1) => \bit_sync_cntr_reg[12]_i_2_n_2\,
      CO(0) => \bit_sync_cntr_reg[12]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \bit_sync_cntr_reg[12]_i_2_n_4\,
      O(2) => \bit_sync_cntr_reg[12]_i_2_n_5\,
      O(1) => \bit_sync_cntr_reg[12]_i_2_n_6\,
      O(0) => \bit_sync_cntr_reg[12]_i_2_n_7\,
      S(3) => \bit_sync_cntr_reg_n_0_[12]\,
      S(2) => \bit_sync_cntr_reg_n_0_[11]\,
      S(1) => \bit_sync_cntr_reg_n_0_[10]\,
      S(0) => \bit_sync_cntr_reg_n_0_[9]\
    );
\bit_sync_cntr_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[13]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[13]\
    );
\bit_sync_cntr_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[14]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[14]\
    );
\bit_sync_cntr_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[15]_i_2_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[15]\
    );
\bit_sync_cntr_reg[15]_i_3\: unisim.vcomponents.CARRY4
     port map (
      CI => \bit_sync_cntr_reg[12]_i_2_n_0\,
      CO(3 downto 2) => \NLW_bit_sync_cntr_reg[15]_i_3_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \bit_sync_cntr_reg[15]_i_3_n_2\,
      CO(0) => \bit_sync_cntr_reg[15]_i_3_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_bit_sync_cntr_reg[15]_i_3_O_UNCONNECTED\(3),
      O(2) => \bit_sync_cntr_reg[15]_i_3_n_5\,
      O(1) => \bit_sync_cntr_reg[15]_i_3_n_6\,
      O(0) => \bit_sync_cntr_reg[15]_i_3_n_7\,
      S(3) => '0',
      S(2) => \bit_sync_cntr_reg_n_0_[15]\,
      S(1) => \bit_sync_cntr_reg_n_0_[14]\,
      S(0) => \bit_sync_cntr_reg_n_0_[13]\
    );
\bit_sync_cntr_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[1]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[1]\
    );
\bit_sync_cntr_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[2]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[2]\
    );
\bit_sync_cntr_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[3]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[3]\
    );
\bit_sync_cntr_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[4]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[4]\
    );
\bit_sync_cntr_reg[4]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \bit_sync_cntr_reg[4]_i_2_n_0\,
      CO(2) => \bit_sync_cntr_reg[4]_i_2_n_1\,
      CO(1) => \bit_sync_cntr_reg[4]_i_2_n_2\,
      CO(0) => \bit_sync_cntr_reg[4]_i_2_n_3\,
      CYINIT => \bit_sync_cntr_reg_n_0_[0]\,
      DI(3 downto 0) => B"0000",
      O(3) => \bit_sync_cntr_reg[4]_i_2_n_4\,
      O(2) => \bit_sync_cntr_reg[4]_i_2_n_5\,
      O(1) => \bit_sync_cntr_reg[4]_i_2_n_6\,
      O(0) => \bit_sync_cntr_reg[4]_i_2_n_7\,
      S(3) => \bit_sync_cntr_reg_n_0_[4]\,
      S(2) => \bit_sync_cntr_reg_n_0_[3]\,
      S(1) => \bit_sync_cntr_reg_n_0_[2]\,
      S(0) => \bit_sync_cntr_reg_n_0_[1]\
    );
\bit_sync_cntr_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[5]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[5]\
    );
\bit_sync_cntr_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[6]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[6]\
    );
\bit_sync_cntr_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[7]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[7]\
    );
\bit_sync_cntr_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[8]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[8]\
    );
\bit_sync_cntr_reg[8]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \bit_sync_cntr_reg[4]_i_2_n_0\,
      CO(3) => \bit_sync_cntr_reg[8]_i_2_n_0\,
      CO(2) => \bit_sync_cntr_reg[8]_i_2_n_1\,
      CO(1) => \bit_sync_cntr_reg[8]_i_2_n_2\,
      CO(0) => \bit_sync_cntr_reg[8]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \bit_sync_cntr_reg[8]_i_2_n_4\,
      O(2) => \bit_sync_cntr_reg[8]_i_2_n_5\,
      O(1) => \bit_sync_cntr_reg[8]_i_2_n_6\,
      O(0) => \bit_sync_cntr_reg[8]_i_2_n_7\,
      S(3) => \bit_sync_cntr_reg_n_0_[8]\,
      S(2) => \bit_sync_cntr_reg_n_0_[7]\,
      S(1) => \bit_sync_cntr_reg_n_0_[6]\,
      S(0) => \bit_sync_cntr_reg_n_0_[5]\
    );
\bit_sync_cntr_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => bit_sync_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \bit_sync_cntr[9]_i_1_n_0\,
      Q => \bit_sync_cntr_reg_n_0_[9]\
    );
dut1: component system_ila_display_0_0_fifo_generator_0
     port map (
      din(31 downto 8) => B"000000000000000000000000",
      din(7) => \fifo_din1_reg_n_0_[7]\,
      din(6) => \fifo_din1_reg_n_0_[6]\,
      din(5) => \fifo_din1_reg_n_0_[5]\,
      din(4) => \fifo_din1_reg_n_0_[4]\,
      din(3) => \fifo_din1_reg_n_0_[3]\,
      din(2) => \fifo_din1_reg_n_0_[2]\,
      din(1) => \fifo_din1_reg_n_0_[1]\,
      din(0) => \fifo_din1_reg_n_0_[0]\,
      dout(31 downto 8) => NLW_dut1_dout_UNCONNECTED(31 downto 8),
      dout(7 downto 0) => fifo_dout1(7 downto 0),
      empty => NLW_dut1_empty_UNCONNECTED,
      full => NLW_dut1_full_UNCONNECTED,
      rd_clk => clock_100,
      rd_en => fifo_rd_en1_reg_n_0,
      rd_rst_busy => NLW_dut1_rd_rst_busy_UNCONNECTED,
      rst => \get_data1[7]_i_2_n_0\,
      wr_clk => clock_61,
      wr_en => fifo_wr_en1_reg_n_0,
      wr_rst_busy => NLW_dut1_wr_rst_busy_UNCONNECTED
    );
dut2: component system_ila_display_0_0_fifo_generator_0_HD1
     port map (
      din(31 downto 8) => B"000000000000000000000000",
      din(7) => \fifo_din2_reg_n_0_[7]\,
      din(6) => \fifo_din2_reg_n_0_[6]\,
      din(5) => \fifo_din2_reg_n_0_[5]\,
      din(4) => \fifo_din2_reg_n_0_[4]\,
      din(3) => \fifo_din2_reg_n_0_[3]\,
      din(2) => \fifo_din2_reg_n_0_[2]\,
      din(1) => \fifo_din2_reg_n_0_[1]\,
      din(0) => \fifo_din2_reg_n_0_[0]\,
      dout(31 downto 8) => NLW_dut2_dout_UNCONNECTED(31 downto 8),
      dout(7 downto 0) => fifo_dout2(7 downto 0),
      empty => NLW_dut2_empty_UNCONNECTED,
      full => NLW_dut2_full_UNCONNECTED,
      rd_clk => clock_100,
      rd_en => fifo_rd_en2_reg_n_0,
      rd_rst_busy => NLW_dut2_rd_rst_busy_UNCONNECTED,
      rst => \get_data1[7]_i_2_n_0\,
      wr_clk => clock_61,
      wr_en => fifo_wr_en2_reg_n_0,
      wr_rst_busy => NLW_dut2_wr_rst_busy_UNCONNECTED
    );
\fifo_din1[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => pattern_gen_val_out,
      O => fifo_din1
    );
\fifo_din1_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din1,
      CLR => \get_data1[7]_i_2_n_0\,
      D => pattern_gen_dat_out(0),
      Q => \fifo_din1_reg_n_0_[0]\
    );
\fifo_din1_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din1,
      CLR => \get_data1[7]_i_2_n_0\,
      D => pattern_gen_dat_out(1),
      Q => \fifo_din1_reg_n_0_[1]\
    );
\fifo_din1_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din1,
      CLR => \get_data1[7]_i_2_n_0\,
      D => pattern_gen_dat_out(2),
      Q => \fifo_din1_reg_n_0_[2]\
    );
\fifo_din1_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din1,
      CLR => \get_data1[7]_i_2_n_0\,
      D => pattern_gen_dat_out(3),
      Q => \fifo_din1_reg_n_0_[3]\
    );
\fifo_din1_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din1,
      CLR => \get_data1[7]_i_2_n_0\,
      D => pattern_gen_dat_out(4),
      Q => \fifo_din1_reg_n_0_[4]\
    );
\fifo_din1_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din1,
      CLR => \get_data1[7]_i_2_n_0\,
      D => pattern_gen_dat_out(5),
      Q => \fifo_din1_reg_n_0_[5]\
    );
\fifo_din1_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din1,
      CLR => \get_data1[7]_i_2_n_0\,
      D => pattern_gen_dat_out(6),
      Q => \fifo_din1_reg_n_0_[6]\
    );
\fifo_din1_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din1,
      CLR => \get_data1[7]_i_2_n_0\,
      D => pattern_gen_dat_out(7),
      Q => \fifo_din1_reg_n_0_[7]\
    );
\fifo_din2[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[0]\,
      I1 => bsync_val,
      O => fifo_din2
    );
\fifo_din2_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din2,
      CLR => \get_data1[7]_i_2_n_0\,
      D => bsync_dat(0),
      Q => \fifo_din2_reg_n_0_[0]\
    );
\fifo_din2_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din2,
      CLR => \get_data1[7]_i_2_n_0\,
      D => bsync_dat(1),
      Q => \fifo_din2_reg_n_0_[1]\
    );
\fifo_din2_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din2,
      CLR => \get_data1[7]_i_2_n_0\,
      D => bsync_dat(2),
      Q => \fifo_din2_reg_n_0_[2]\
    );
\fifo_din2_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din2,
      CLR => \get_data1[7]_i_2_n_0\,
      D => bsync_dat(3),
      Q => \fifo_din2_reg_n_0_[3]\
    );
\fifo_din2_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din2,
      CLR => \get_data1[7]_i_2_n_0\,
      D => bsync_dat(4),
      Q => \fifo_din2_reg_n_0_[4]\
    );
\fifo_din2_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din2,
      CLR => \get_data1[7]_i_2_n_0\,
      D => bsync_dat(5),
      Q => \fifo_din2_reg_n_0_[5]\
    );
\fifo_din2_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din2,
      CLR => \get_data1[7]_i_2_n_0\,
      D => bsync_dat(6),
      Q => \fifo_din2_reg_n_0_[6]\
    );
\fifo_din2_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => fifo_din2,
      CLR => \get_data1[7]_i_2_n_0\,
      D => bsync_dat(7),
      Q => \fifo_din2_reg_n_0_[7]\
    );
fifo_rd_en1_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"CFAFC0AF"
    )
        port map (
      I0 => sync_push_now_1,
      I1 => \FSM_sequential_push_stm1[0]_inv_i_2_n_0\,
      I2 => push_stm1(0),
      I3 => send_done1,
      I4 => fifo_rd_en1_reg_n_0,
      O => fifo_rd_en1_i_1_n_0
    );
fifo_rd_en1_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => fifo_rd_en1_i_1_n_0,
      Q => fifo_rd_en1_reg_n_0
    );
fifo_rd_en2_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"CFAFC0AF"
    )
        port map (
      I0 => sync_push_now_1,
      I1 => \FSM_sequential_push_stm2[0]_inv_i_2_n_0\,
      I2 => push_stm2(0),
      I3 => send_done2,
      I4 => fifo_rd_en2_reg_n_0,
      O => fifo_rd_en2_i_1_n_0
    );
fifo_rd_en2_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => fifo_rd_en2_i_1_n_0,
      Q => fifo_rd_en2_reg_n_0
    );
fifo_wr_en1_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => fifo_din1,
      Q => fifo_wr_en1_reg_n_0
    );
fifo_wr_en2_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => fifo_din2,
      Q => fifo_wr_en2_reg_n_0
    );
\get_data1[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done1,
      I1 => fifo_dout1(0),
      O => \get_data1[0]_i_1_n_0\
    );
\get_data1[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done1,
      I1 => fifo_dout1(1),
      O => \get_data1[1]_i_1_n_0\
    );
\get_data1[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done1,
      I1 => fifo_dout1(2),
      O => \get_data1[2]_i_1_n_0\
    );
\get_data1[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done1,
      I1 => fifo_dout1(3),
      O => \get_data1[3]_i_1_n_0\
    );
\get_data1[4]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done1,
      I1 => fifo_dout1(4),
      O => \get_data1[4]_i_1_n_0\
    );
\get_data1[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done1,
      I1 => fifo_dout1(5),
      O => \get_data1[5]_i_1_n_0\
    );
\get_data1[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done1,
      I1 => fifo_dout1(6),
      O => \get_data1[6]_i_1_n_0\
    );
\get_data1[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done1,
      I1 => fifo_dout1(7),
      O => \get_data1[7]_i_1_n_0\
    );
\get_data1[7]_i_2\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => reset_n,
      O => \get_data1[7]_i_2_n_0\
    );
\get_data1_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data1[0]_i_1_n_0\,
      Q => ila_pattern_gen_dat(0)
    );
\get_data1_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data1[1]_i_1_n_0\,
      Q => ila_pattern_gen_dat(1)
    );
\get_data1_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data1[2]_i_1_n_0\,
      Q => ila_pattern_gen_dat(2)
    );
\get_data1_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data1[3]_i_1_n_0\,
      Q => ila_pattern_gen_dat(3)
    );
\get_data1_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data1[4]_i_1_n_0\,
      Q => ila_pattern_gen_dat(4)
    );
\get_data1_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data1[5]_i_1_n_0\,
      Q => ila_pattern_gen_dat(5)
    );
\get_data1_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data1[6]_i_1_n_0\,
      Q => ila_pattern_gen_dat(6)
    );
\get_data1_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data1[7]_i_1_n_0\,
      Q => ila_pattern_gen_dat(7)
    );
get_data1_valid_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm1(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => send_done1,
      Q => ila_pattern_gen_val
    );
\get_data2[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done2,
      I1 => fifo_dout2(0),
      O => \get_data2[0]_i_1_n_0\
    );
\get_data2[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done2,
      I1 => fifo_dout2(1),
      O => \get_data2[1]_i_1_n_0\
    );
\get_data2[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done2,
      I1 => fifo_dout2(2),
      O => \get_data2[2]_i_1_n_0\
    );
\get_data2[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done2,
      I1 => fifo_dout2(3),
      O => \get_data2[3]_i_1_n_0\
    );
\get_data2[4]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done2,
      I1 => fifo_dout2(4),
      O => \get_data2[4]_i_1_n_0\
    );
\get_data2[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done2,
      I1 => fifo_dout2(5),
      O => \get_data2[5]_i_1_n_0\
    );
\get_data2[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done2,
      I1 => fifo_dout2(6),
      O => \get_data2[6]_i_1_n_0\
    );
\get_data2[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => send_done2,
      I1 => fifo_dout2(7),
      O => \get_data2[7]_i_1_n_0\
    );
\get_data2_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data2[0]_i_1_n_0\,
      Q => ila_bsync_dat(0)
    );
\get_data2_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data2[1]_i_1_n_0\,
      Q => ila_bsync_dat(1)
    );
\get_data2_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data2[2]_i_1_n_0\,
      Q => ila_bsync_dat(2)
    );
\get_data2_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data2[3]_i_1_n_0\,
      Q => ila_bsync_dat(3)
    );
\get_data2_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data2[4]_i_1_n_0\,
      Q => ila_bsync_dat(4)
    );
\get_data2_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data2[5]_i_1_n_0\,
      Q => ila_bsync_dat(5)
    );
\get_data2_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data2[6]_i_1_n_0\,
      Q => ila_bsync_dat(6)
    );
\get_data2_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => \get_data2[7]_i_1_n_0\,
      Q => ila_bsync_dat(7)
    );
get_data2_valid_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => push_stm2(0),
      CLR => \get_data1[7]_i_2_n_0\,
      D => send_done2,
      Q => ila_bsync_val
    );
\pattern_cntr1[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \pattern_cntr1_reg_n_0_[0]\,
      O => \pattern_cntr1[0]_i_1_n_0\
    );
\pattern_cntr1[10]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[12]_i_2_n_6\,
      O => \pattern_cntr1[10]_i_1_n_0\
    );
\pattern_cntr1[11]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[12]_i_2_n_5\,
      O => \pattern_cntr1[11]_i_1_n_0\
    );
\pattern_cntr1[12]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[12]_i_2_n_4\,
      O => \pattern_cntr1[12]_i_1_n_0\
    );
\pattern_cntr1[13]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[15]_i_7_n_7\,
      O => \pattern_cntr1[13]_i_1_n_0\
    );
\pattern_cntr1[14]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[15]_i_7_n_6\,
      O => \pattern_cntr1[14]_i_1_n_0\
    );
\pattern_cntr1[15]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"D"
    )
        port map (
      I0 => push_stm1(0),
      I1 => send_done1,
      O => \pattern_cntr1[15]_i_1_n_0\
    );
\pattern_cntr1[15]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[15]_i_7_n_5\,
      O => \pattern_cntr1[15]_i_2_n_0\
    );
\pattern_cntr1[15]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFD"
    )
        port map (
      I0 => \pattern_cntr1_reg_n_0_[0]\,
      I1 => \pattern_cntr1_reg_n_0_[9]\,
      I2 => \pattern_cntr1_reg_n_0_[11]\,
      I3 => \pattern_cntr1_reg_n_0_[10]\,
      O => \pattern_cntr1[15]_i_3_n_0\
    );
\pattern_cntr1[15]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \pattern_cntr1_reg_n_0_[13]\,
      I1 => \pattern_cntr1_reg_n_0_[12]\,
      I2 => \pattern_cntr1_reg_n_0_[15]\,
      I3 => \pattern_cntr1_reg_n_0_[14]\,
      O => \pattern_cntr1[15]_i_4_n_0\
    );
\pattern_cntr1[15]_i_5\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFFFFFE"
    )
        port map (
      I0 => \pattern_cntr1_reg_n_0_[7]\,
      I1 => \pattern_cntr1_reg_n_0_[6]\,
      I2 => \pattern_cntr1_reg_n_0_[1]\,
      I3 => push_stm1(0),
      I4 => \pattern_cntr1_reg_n_0_[8]\,
      O => \pattern_cntr1[15]_i_5_n_0\
    );
\pattern_cntr1[15]_i_6\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFFFFFE"
    )
        port map (
      I0 => \pattern_cntr1_reg_n_0_[3]\,
      I1 => \pattern_cntr1_reg_n_0_[2]\,
      I2 => \pattern_cntr1_reg_n_0_[1]\,
      I3 => \pattern_cntr1_reg_n_0_[5]\,
      I4 => \pattern_cntr1_reg_n_0_[4]\,
      O => \pattern_cntr1[15]_i_6_n_0\
    );
\pattern_cntr1[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[4]_i_2_n_7\,
      O => \pattern_cntr1[1]_i_1_n_0\
    );
\pattern_cntr1[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[4]_i_2_n_6\,
      O => \pattern_cntr1[2]_i_1_n_0\
    );
\pattern_cntr1[3]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[4]_i_2_n_5\,
      O => \pattern_cntr1[3]_i_1_n_0\
    );
\pattern_cntr1[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[4]_i_2_n_4\,
      O => \pattern_cntr1[4]_i_1_n_0\
    );
\pattern_cntr1[5]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[8]_i_2_n_7\,
      O => \pattern_cntr1[5]_i_1_n_0\
    );
\pattern_cntr1[6]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[8]_i_2_n_6\,
      O => \pattern_cntr1[6]_i_1_n_0\
    );
\pattern_cntr1[7]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[8]_i_2_n_5\,
      O => \pattern_cntr1[7]_i_1_n_0\
    );
\pattern_cntr1[8]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[8]_i_2_n_4\,
      O => \pattern_cntr1[8]_i_1_n_0\
    );
\pattern_cntr1[9]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr1[15]_i_3_n_0\,
      I1 => \pattern_cntr1[15]_i_4_n_0\,
      I2 => \pattern_cntr1[15]_i_5_n_0\,
      I3 => \pattern_cntr1[15]_i_6_n_0\,
      I4 => \pattern_cntr1_reg[12]_i_2_n_7\,
      O => \pattern_cntr1[9]_i_1_n_0\
    );
\pattern_cntr1_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[0]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[0]\
    );
\pattern_cntr1_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[10]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[10]\
    );
\pattern_cntr1_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[11]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[11]\
    );
\pattern_cntr1_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[12]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[12]\
    );
\pattern_cntr1_reg[12]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr1_reg[8]_i_2_n_0\,
      CO(3) => \pattern_cntr1_reg[12]_i_2_n_0\,
      CO(2) => \pattern_cntr1_reg[12]_i_2_n_1\,
      CO(1) => \pattern_cntr1_reg[12]_i_2_n_2\,
      CO(0) => \pattern_cntr1_reg[12]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \pattern_cntr1_reg[12]_i_2_n_4\,
      O(2) => \pattern_cntr1_reg[12]_i_2_n_5\,
      O(1) => \pattern_cntr1_reg[12]_i_2_n_6\,
      O(0) => \pattern_cntr1_reg[12]_i_2_n_7\,
      S(3) => \pattern_cntr1_reg_n_0_[12]\,
      S(2) => \pattern_cntr1_reg_n_0_[11]\,
      S(1) => \pattern_cntr1_reg_n_0_[10]\,
      S(0) => \pattern_cntr1_reg_n_0_[9]\
    );
\pattern_cntr1_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[13]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[13]\
    );
\pattern_cntr1_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[14]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[14]\
    );
\pattern_cntr1_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[15]_i_2_n_0\,
      Q => \pattern_cntr1_reg_n_0_[15]\
    );
\pattern_cntr1_reg[15]_i_7\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr1_reg[12]_i_2_n_0\,
      CO(3 downto 2) => \NLW_pattern_cntr1_reg[15]_i_7_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \pattern_cntr1_reg[15]_i_7_n_2\,
      CO(0) => \pattern_cntr1_reg[15]_i_7_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_pattern_cntr1_reg[15]_i_7_O_UNCONNECTED\(3),
      O(2) => \pattern_cntr1_reg[15]_i_7_n_5\,
      O(1) => \pattern_cntr1_reg[15]_i_7_n_6\,
      O(0) => \pattern_cntr1_reg[15]_i_7_n_7\,
      S(3) => '0',
      S(2) => \pattern_cntr1_reg_n_0_[15]\,
      S(1) => \pattern_cntr1_reg_n_0_[14]\,
      S(0) => \pattern_cntr1_reg_n_0_[13]\
    );
\pattern_cntr1_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[1]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[1]\
    );
\pattern_cntr1_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[2]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[2]\
    );
\pattern_cntr1_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[3]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[3]\
    );
\pattern_cntr1_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[4]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[4]\
    );
\pattern_cntr1_reg[4]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \pattern_cntr1_reg[4]_i_2_n_0\,
      CO(2) => \pattern_cntr1_reg[4]_i_2_n_1\,
      CO(1) => \pattern_cntr1_reg[4]_i_2_n_2\,
      CO(0) => \pattern_cntr1_reg[4]_i_2_n_3\,
      CYINIT => \pattern_cntr1_reg_n_0_[0]\,
      DI(3 downto 0) => B"0000",
      O(3) => \pattern_cntr1_reg[4]_i_2_n_4\,
      O(2) => \pattern_cntr1_reg[4]_i_2_n_5\,
      O(1) => \pattern_cntr1_reg[4]_i_2_n_6\,
      O(0) => \pattern_cntr1_reg[4]_i_2_n_7\,
      S(3) => \pattern_cntr1_reg_n_0_[4]\,
      S(2) => \pattern_cntr1_reg_n_0_[3]\,
      S(1) => \pattern_cntr1_reg_n_0_[2]\,
      S(0) => \pattern_cntr1_reg_n_0_[1]\
    );
\pattern_cntr1_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[5]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[5]\
    );
\pattern_cntr1_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[6]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[6]\
    );
\pattern_cntr1_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[7]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[7]\
    );
\pattern_cntr1_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[8]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[8]\
    );
\pattern_cntr1_reg[8]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr1_reg[4]_i_2_n_0\,
      CO(3) => \pattern_cntr1_reg[8]_i_2_n_0\,
      CO(2) => \pattern_cntr1_reg[8]_i_2_n_1\,
      CO(1) => \pattern_cntr1_reg[8]_i_2_n_2\,
      CO(0) => \pattern_cntr1_reg[8]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \pattern_cntr1_reg[8]_i_2_n_4\,
      O(2) => \pattern_cntr1_reg[8]_i_2_n_5\,
      O(1) => \pattern_cntr1_reg[8]_i_2_n_6\,
      O(0) => \pattern_cntr1_reg[8]_i_2_n_7\,
      S(3) => \pattern_cntr1_reg_n_0_[8]\,
      S(2) => \pattern_cntr1_reg_n_0_[7]\,
      S(1) => \pattern_cntr1_reg_n_0_[6]\,
      S(0) => \pattern_cntr1_reg_n_0_[5]\
    );
\pattern_cntr1_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr1[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr1[9]_i_1_n_0\,
      Q => \pattern_cntr1_reg_n_0_[9]\
    );
\pattern_cntr2[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \pattern_cntr2_reg_n_0_[0]\,
      O => \pattern_cntr2[0]_i_1_n_0\
    );
\pattern_cntr2[10]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(10),
      O => \pattern_cntr2[10]_i_1_n_0\
    );
\pattern_cntr2[11]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(11),
      O => \pattern_cntr2[11]_i_1_n_0\
    );
\pattern_cntr2[12]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(12),
      O => \pattern_cntr2[12]_i_1_n_0\
    );
\pattern_cntr2[13]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(13),
      O => \pattern_cntr2[13]_i_1_n_0\
    );
\pattern_cntr2[14]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(14),
      O => \pattern_cntr2[14]_i_1_n_0\
    );
\pattern_cntr2[15]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"D"
    )
        port map (
      I0 => push_stm2(0),
      I1 => send_done2,
      O => \pattern_cntr2[15]_i_1_n_0\
    );
\pattern_cntr2[15]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(15),
      O => \pattern_cntr2[15]_i_2_n_0\
    );
\pattern_cntr2[15]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFD"
    )
        port map (
      I0 => \pattern_cntr2_reg_n_0_[0]\,
      I1 => \pattern_cntr2_reg_n_0_[9]\,
      I2 => \pattern_cntr2_reg_n_0_[11]\,
      I3 => \pattern_cntr2_reg_n_0_[10]\,
      O => \pattern_cntr2[15]_i_3_n_0\
    );
\pattern_cntr2[15]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \pattern_cntr2_reg_n_0_[13]\,
      I1 => \pattern_cntr2_reg_n_0_[12]\,
      I2 => \pattern_cntr2_reg_n_0_[15]\,
      I3 => \pattern_cntr2_reg_n_0_[14]\,
      O => \pattern_cntr2[15]_i_4_n_0\
    );
\pattern_cntr2[15]_i_5\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFFFFFE"
    )
        port map (
      I0 => \pattern_cntr2_reg_n_0_[7]\,
      I1 => \pattern_cntr2_reg_n_0_[6]\,
      I2 => \pattern_cntr2_reg_n_0_[1]\,
      I3 => push_stm2(0),
      I4 => \pattern_cntr2_reg_n_0_[8]\,
      O => \pattern_cntr2[15]_i_5_n_0\
    );
\pattern_cntr2[15]_i_6\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFFFFFE"
    )
        port map (
      I0 => \pattern_cntr2_reg_n_0_[3]\,
      I1 => \pattern_cntr2_reg_n_0_[2]\,
      I2 => \pattern_cntr2_reg_n_0_[1]\,
      I3 => \pattern_cntr2_reg_n_0_[5]\,
      I4 => \pattern_cntr2_reg_n_0_[4]\,
      O => \pattern_cntr2[15]_i_6_n_0\
    );
\pattern_cntr2[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(1),
      O => \pattern_cntr2[1]_i_1_n_0\
    );
\pattern_cntr2[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(2),
      O => \pattern_cntr2[2]_i_1_n_0\
    );
\pattern_cntr2[3]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(3),
      O => \pattern_cntr2[3]_i_1_n_0\
    );
\pattern_cntr2[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(4),
      O => \pattern_cntr2[4]_i_1_n_0\
    );
\pattern_cntr2[5]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(5),
      O => \pattern_cntr2[5]_i_1_n_0\
    );
\pattern_cntr2[6]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(6),
      O => \pattern_cntr2[6]_i_1_n_0\
    );
\pattern_cntr2[7]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(7),
      O => \pattern_cntr2[7]_i_1_n_0\
    );
\pattern_cntr2[8]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(8),
      O => \pattern_cntr2[8]_i_1_n_0\
    );
\pattern_cntr2[9]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFE0000"
    )
        port map (
      I0 => \pattern_cntr2[15]_i_3_n_0\,
      I1 => \pattern_cntr2[15]_i_4_n_0\,
      I2 => \pattern_cntr2[15]_i_5_n_0\,
      I3 => \pattern_cntr2[15]_i_6_n_0\,
      I4 => data0(9),
      O => \pattern_cntr2[9]_i_1_n_0\
    );
\pattern_cntr2_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[0]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[0]\
    );
\pattern_cntr2_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[10]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[10]\
    );
\pattern_cntr2_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[11]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[11]\
    );
\pattern_cntr2_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[12]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[12]\
    );
\pattern_cntr2_reg[12]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr2_reg[8]_i_2_n_0\,
      CO(3) => \pattern_cntr2_reg[12]_i_2_n_0\,
      CO(2) => \pattern_cntr2_reg[12]_i_2_n_1\,
      CO(1) => \pattern_cntr2_reg[12]_i_2_n_2\,
      CO(0) => \pattern_cntr2_reg[12]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => data0(12 downto 9),
      S(3) => \pattern_cntr2_reg_n_0_[12]\,
      S(2) => \pattern_cntr2_reg_n_0_[11]\,
      S(1) => \pattern_cntr2_reg_n_0_[10]\,
      S(0) => \pattern_cntr2_reg_n_0_[9]\
    );
\pattern_cntr2_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[13]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[13]\
    );
\pattern_cntr2_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[14]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[14]\
    );
\pattern_cntr2_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[15]_i_2_n_0\,
      Q => \pattern_cntr2_reg_n_0_[15]\
    );
\pattern_cntr2_reg[15]_i_7\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr2_reg[12]_i_2_n_0\,
      CO(3 downto 2) => \NLW_pattern_cntr2_reg[15]_i_7_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \pattern_cntr2_reg[15]_i_7_n_2\,
      CO(0) => \pattern_cntr2_reg[15]_i_7_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_pattern_cntr2_reg[15]_i_7_O_UNCONNECTED\(3),
      O(2 downto 0) => data0(15 downto 13),
      S(3) => '0',
      S(2) => \pattern_cntr2_reg_n_0_[15]\,
      S(1) => \pattern_cntr2_reg_n_0_[14]\,
      S(0) => \pattern_cntr2_reg_n_0_[13]\
    );
\pattern_cntr2_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[1]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[1]\
    );
\pattern_cntr2_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[2]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[2]\
    );
\pattern_cntr2_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[3]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[3]\
    );
\pattern_cntr2_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[4]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[4]\
    );
\pattern_cntr2_reg[4]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \pattern_cntr2_reg[4]_i_2_n_0\,
      CO(2) => \pattern_cntr2_reg[4]_i_2_n_1\,
      CO(1) => \pattern_cntr2_reg[4]_i_2_n_2\,
      CO(0) => \pattern_cntr2_reg[4]_i_2_n_3\,
      CYINIT => \pattern_cntr2_reg_n_0_[0]\,
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => data0(4 downto 1),
      S(3) => \pattern_cntr2_reg_n_0_[4]\,
      S(2) => \pattern_cntr2_reg_n_0_[3]\,
      S(1) => \pattern_cntr2_reg_n_0_[2]\,
      S(0) => \pattern_cntr2_reg_n_0_[1]\
    );
\pattern_cntr2_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[5]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[5]\
    );
\pattern_cntr2_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[6]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[6]\
    );
\pattern_cntr2_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[7]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[7]\
    );
\pattern_cntr2_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[8]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[8]\
    );
\pattern_cntr2_reg[8]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr2_reg[4]_i_2_n_0\,
      CO(3) => \pattern_cntr2_reg[8]_i_2_n_0\,
      CO(2) => \pattern_cntr2_reg[8]_i_2_n_1\,
      CO(1) => \pattern_cntr2_reg[8]_i_2_n_2\,
      CO(0) => \pattern_cntr2_reg[8]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => data0(8 downto 5),
      S(3) => \pattern_cntr2_reg_n_0_[8]\,
      S(2) => \pattern_cntr2_reg_n_0_[7]\,
      S(1) => \pattern_cntr2_reg_n_0_[6]\,
      S(0) => \pattern_cntr2_reg_n_0_[5]\
    );
\pattern_cntr2_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => \pattern_cntr2[15]_i_1_n_0\,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr2[9]_i_1_n_0\,
      Q => \pattern_cntr2_reg_n_0_[9]\
    );
\pattern_cntr[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => \pattern_cntr_reg_n_0_[0]\,
      O => \pattern_cntr[0]_i_1_n_0\
    );
\pattern_cntr[10]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(10),
      O => \pattern_cntr[10]_i_1_n_0\
    );
\pattern_cntr[11]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(11),
      O => \pattern_cntr[11]_i_1_n_0\
    );
\pattern_cntr[12]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(12),
      O => \pattern_cntr[12]_i_1_n_0\
    );
\pattern_cntr[13]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(13),
      O => \pattern_cntr[13]_i_1_n_0\
    );
\pattern_cntr[14]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(14),
      O => \pattern_cntr[14]_i_1_n_0\
    );
\pattern_cntr[15]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F888"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[1]\,
      I1 => push_now,
      I2 => pattern_gen_val_out,
      I3 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      O => pattern_cntr
    );
\pattern_cntr[15]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(15),
      O => \pattern_cntr[15]_i_2_n_0\
    );
\pattern_cntr[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(1),
      O => \pattern_cntr[1]_i_1_n_0\
    );
\pattern_cntr[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(2),
      O => \pattern_cntr[2]_i_1_n_0\
    );
\pattern_cntr[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(3),
      O => \pattern_cntr[3]_i_1_n_0\
    );
\pattern_cntr[4]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(4),
      O => \pattern_cntr[4]_i_1_n_0\
    );
\pattern_cntr[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(5),
      O => \pattern_cntr[5]_i_1_n_0\
    );
\pattern_cntr[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(6),
      O => \pattern_cntr[6]_i_1_n_0\
    );
\pattern_cntr[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(7),
      O => \pattern_cntr[7]_i_1_n_0\
    );
\pattern_cntr[8]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(8),
      O => \pattern_cntr[8]_i_1_n_0\
    );
\pattern_cntr[9]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state1_reg_n_0_[0]\,
      I1 => in4(9),
      O => \pattern_cntr[9]_i_1_n_0\
    );
\pattern_cntr_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[0]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[0]\
    );
\pattern_cntr_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[10]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[10]\
    );
\pattern_cntr_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[11]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[11]\
    );
\pattern_cntr_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[12]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[12]\
    );
\pattern_cntr_reg[12]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr_reg[8]_i_2_n_0\,
      CO(3) => \pattern_cntr_reg[12]_i_2_n_0\,
      CO(2) => \pattern_cntr_reg[12]_i_2_n_1\,
      CO(1) => \pattern_cntr_reg[12]_i_2_n_2\,
      CO(0) => \pattern_cntr_reg[12]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => in4(12 downto 9),
      S(3) => \pattern_cntr_reg_n_0_[12]\,
      S(2) => \pattern_cntr_reg_n_0_[11]\,
      S(1) => \pattern_cntr_reg_n_0_[10]\,
      S(0) => \pattern_cntr_reg_n_0_[9]\
    );
\pattern_cntr_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[13]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[13]\
    );
\pattern_cntr_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[14]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[14]\
    );
\pattern_cntr_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[15]_i_2_n_0\,
      Q => \pattern_cntr_reg_n_0_[15]\
    );
\pattern_cntr_reg[15]_i_3\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr_reg[12]_i_2_n_0\,
      CO(3 downto 2) => \NLW_pattern_cntr_reg[15]_i_3_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \pattern_cntr_reg[15]_i_3_n_2\,
      CO(0) => \pattern_cntr_reg[15]_i_3_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_pattern_cntr_reg[15]_i_3_O_UNCONNECTED\(3),
      O(2 downto 0) => in4(15 downto 13),
      S(3) => '0',
      S(2) => \pattern_cntr_reg_n_0_[15]\,
      S(1) => \pattern_cntr_reg_n_0_[14]\,
      S(0) => \pattern_cntr_reg_n_0_[13]\
    );
\pattern_cntr_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[1]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[1]\
    );
\pattern_cntr_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[2]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[2]\
    );
\pattern_cntr_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[3]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[3]\
    );
\pattern_cntr_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[4]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[4]\
    );
\pattern_cntr_reg[4]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \pattern_cntr_reg[4]_i_2_n_0\,
      CO(2) => \pattern_cntr_reg[4]_i_2_n_1\,
      CO(1) => \pattern_cntr_reg[4]_i_2_n_2\,
      CO(0) => \pattern_cntr_reg[4]_i_2_n_3\,
      CYINIT => \pattern_cntr_reg_n_0_[0]\,
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => in4(4 downto 1),
      S(3) => \pattern_cntr_reg_n_0_[4]\,
      S(2) => \pattern_cntr_reg_n_0_[3]\,
      S(1) => \pattern_cntr_reg_n_0_[2]\,
      S(0) => \pattern_cntr_reg_n_0_[1]\
    );
\pattern_cntr_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[5]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[5]\
    );
\pattern_cntr_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[6]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[6]\
    );
\pattern_cntr_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[7]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[7]\
    );
\pattern_cntr_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[8]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[8]\
    );
\pattern_cntr_reg[8]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \pattern_cntr_reg[4]_i_2_n_0\,
      CO(3) => \pattern_cntr_reg[8]_i_2_n_0\,
      CO(2) => \pattern_cntr_reg[8]_i_2_n_1\,
      CO(1) => \pattern_cntr_reg[8]_i_2_n_2\,
      CO(0) => \pattern_cntr_reg[8]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => in4(8 downto 5),
      S(3) => \pattern_cntr_reg_n_0_[8]\,
      S(2) => \pattern_cntr_reg_n_0_[7]\,
      S(1) => \pattern_cntr_reg_n_0_[6]\,
      S(0) => \pattern_cntr_reg_n_0_[5]\
    );
\pattern_cntr_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => pattern_cntr,
      CLR => \get_data1[7]_i_2_n_0\,
      D => \pattern_cntr[9]_i_1_n_0\,
      Q => \pattern_cntr_reg_n_0_[9]\
    );
push_now_i_1: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_acc_state2_reg_n_0_[1]\,
      I1 => \FSM_onehot_acc_state1_reg_n_0_[1]\,
      O => push_now_i_1_n_0
    );
push_now_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => push_now_i_1_n_0,
      Q => push_now
    );
send_done1_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"CF40"
    )
        port map (
      I0 => \FSM_sequential_push_stm1[0]_inv_i_2_n_0\,
      I1 => send_done1,
      I2 => push_stm1(0),
      I3 => \send_done1__0\,
      O => send_done1_i_1_n_0
    );
send_done1_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => send_done1_i_1_n_0,
      Q => \send_done1__0\
    );
send_done2_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"CF40"
    )
        port map (
      I0 => \FSM_sequential_push_stm2[0]_inv_i_2_n_0\,
      I1 => send_done2,
      I2 => push_stm2(0),
      I3 => \send_done2__0\,
      O => send_done2_i_1_n_0
    );
send_done2_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => send_done2_i_1_n_0,
      Q => \send_done2__0\
    );
sync_push_now_0_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => push_now,
      Q => sync_push_now_0
    );
sync_push_now_1_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_100,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => sync_push_now_0,
      Q => sync_push_now_1
    );
sync_send_done1_0_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => \send_done1__0\,
      Q => sync_send_done1_0
    );
sync_send_done1_1_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => sync_send_done1_0,
      Q => sync_send_done1_1
    );
sync_send_done2_0_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => \send_done2__0\,
      Q => sync_send_done2_0
    );
sync_send_done2_1_reg: unisim.vcomponents.FDCE
     port map (
      C => clock_61,
      CE => '1',
      CLR => \get_data1[7]_i_2_n_0\,
      D => sync_send_done2_0,
      Q => sync_send_done2_1
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_ila_display_0_0 is
  port (
    clock_61 : in STD_LOGIC;
    clock_100 : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    pattern_gen_dat_out : in STD_LOGIC_VECTOR ( 7 downto 0 );
    pattern_gen_val_out : in STD_LOGIC;
    bsync_dat : in STD_LOGIC_VECTOR ( 7 downto 0 );
    bsync_val : in STD_LOGIC;
    ila_pattern_gen_dat : out STD_LOGIC_VECTOR ( 7 downto 0 );
    ila_pattern_gen_val : out STD_LOGIC;
    ila_bsync_dat : out STD_LOGIC_VECTOR ( 7 downto 0 );
    ila_bsync_val : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of system_ila_display_0_0 : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of system_ila_display_0_0 : entity is "system_ila_display_0_0,ila_display,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of system_ila_display_0_0 : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of system_ila_display_0_0 : entity is "module_ref";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of system_ila_display_0_0 : entity is "ila_display,Vivado 2023.1";
end system_ila_display_0_0;

architecture STRUCTURE of system_ila_display_0_0 is
  attribute FIFO_DEPTH : string;
  attribute FIFO_DEPTH of inst : label is "16'b0000001000000000";
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of clock_100 : signal is "xilinx.com:signal:clock:1.0 clock_100 CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of clock_100 : signal is "XIL_INTERFACENAME clock_100, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_sys_ps7_0_FCLK_CLK0, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of clock_61 : signal is "xilinx.com:signal:clock:1.0 clock_61 CLK";
  attribute X_INTERFACE_PARAMETER of clock_61 : signal is "XIL_INTERFACENAME clock_61, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of reset_n : signal is "xilinx.com:signal:reset:1.0 reset_n RST";
  attribute X_INTERFACE_PARAMETER of reset_n : signal is "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
begin
inst: entity work.system_ila_display_0_0_ila_display
     port map (
      bsync_dat(7 downto 0) => bsync_dat(7 downto 0),
      bsync_val => bsync_val,
      clock_100 => clock_100,
      clock_61 => clock_61,
      ila_bsync_dat(7 downto 0) => ila_bsync_dat(7 downto 0),
      ila_bsync_val => ila_bsync_val,
      ila_pattern_gen_dat(7 downto 0) => ila_pattern_gen_dat(7 downto 0),
      ila_pattern_gen_val => ila_pattern_gen_val,
      pattern_gen_dat_out(7 downto 0) => pattern_gen_dat_out(7 downto 0),
      pattern_gen_val_out => pattern_gen_val_out,
      reset_n => reset_n
    );
end STRUCTURE;
