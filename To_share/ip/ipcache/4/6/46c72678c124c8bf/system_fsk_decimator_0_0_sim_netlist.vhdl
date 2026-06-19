-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Tue Feb  3 16:25:28 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_fsk_decimator_0_0_sim_netlist.vhdl
-- Design      : system_fsk_decimator_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both is
  port (
    \B_V_data_1_state_reg[1]_0\ : out STD_LOGIC;
    \B_V_data_1_state_reg[0]_0\ : out STD_LOGIC;
    \icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\ : out STD_LOGIC;
    trunc_ln2_reg_2930 : out STD_LOGIC;
    ap_enable_reg_pp0_iter1_reg : out STD_LOGIC;
    dec_counter_reg_5_sp_1 : out STD_LOGIC;
    dec_counter_reg_7_sp_1 : out STD_LOGIC;
    dec_counter_reg_6_sp_1 : out STD_LOGIC;
    \dec_counter_reg[5]_0\ : out STD_LOGIC;
    dec_counter_reg_4_sp_1 : out STD_LOGIC;
    dec_counter_reg_2_sp_1 : out STD_LOGIC;
    dec_counter_reg_3_sp_1 : out STD_LOGIC;
    dec_counter_reg_0_sp_1 : out STD_LOGIC;
    ap_block_pp0_stage0_11001 : out STD_LOGIC;
    \B_V_data_1_state_reg[1]_1\ : out STD_LOGIC;
    \B_V_data_1_state_reg[1]_2\ : out STD_LOGIC;
    \B_V_data_1_state_reg[0]_1\ : out STD_LOGIC;
    ap_enable_reg_pp0_iter1_reg_0 : out STD_LOGIC;
    dec_out_TDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    dec_out_TREADY : in STD_LOGIC;
    rx_in_TVALID_int_regslice : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_2\ : in STD_LOGIC;
    dec_counter_reg : in STD_LOGIC_VECTOR ( 7 downto 0 );
    ap_enable_reg_pp0_iter1 : in STD_LOGIC;
    icmp_ln40_reg_289 : in STD_LOGIC;
    icmp_ln40_reg_289_pp0_iter1_reg : in STD_LOGIC;
    ap_enable_reg_pp0_iter2 : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_3\ : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_4\ : in STD_LOGIC;
    B_V_data_1_sel_wr : in STD_LOGIC;
    B_V_data_1_sel : in STD_LOGIC;
    D : in STD_LOGIC_VECTOR ( 31 downto 0 )
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both is
  signal B_V_data_1_load_A : STD_LOGIC;
  signal B_V_data_1_load_B : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[16]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[17]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[18]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[19]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[20]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[21]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[22]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[23]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[24]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[25]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[26]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[27]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[28]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[29]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[2]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[30]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[31]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[3]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[4]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[5]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[6]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[7]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[8]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[9]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[16]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[17]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[18]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[19]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[20]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[21]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[22]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[23]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[24]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[25]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[26]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[27]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[28]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[29]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[2]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[30]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[31]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[3]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[4]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[5]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[6]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[7]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[8]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[9]\ : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_rd_reg_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_wr_0 : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__2_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[0]_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[1]_0\ : STD_LOGIC;
  signal \^ap_enable_reg_pp0_iter1_reg\ : STD_LOGIC;
  signal \dec_counter[4]_i_2_n_0\ : STD_LOGIC;
  signal \dec_counter[5]_i_2_n_0\ : STD_LOGIC;
  signal dec_counter_reg_0_sn_1 : STD_LOGIC;
  signal dec_counter_reg_2_sn_1 : STD_LOGIC;
  signal dec_counter_reg_3_sn_1 : STD_LOGIC;
  signal dec_counter_reg_4_sn_1 : STD_LOGIC;
  signal dec_counter_reg_5_sn_1 : STD_LOGIC;
  signal dec_counter_reg_6_sn_1 : STD_LOGIC;
  signal dec_counter_reg_7_sn_1 : STD_LOGIC;
  signal \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_2__1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \dec_counter[4]_i_2\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \dec_counter[5]_i_2\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \dec_counter[7]_i_1\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \dec_out_TDATA[0]_INST_0\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \dec_out_TDATA[10]_INST_0\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \dec_out_TDATA[11]_INST_0\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \dec_out_TDATA[12]_INST_0\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \dec_out_TDATA[13]_INST_0\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \dec_out_TDATA[14]_INST_0\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \dec_out_TDATA[15]_INST_0\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \dec_out_TDATA[16]_INST_0\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \dec_out_TDATA[17]_INST_0\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \dec_out_TDATA[18]_INST_0\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \dec_out_TDATA[19]_INST_0\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \dec_out_TDATA[1]_INST_0\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \dec_out_TDATA[20]_INST_0\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \dec_out_TDATA[21]_INST_0\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \dec_out_TDATA[22]_INST_0\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \dec_out_TDATA[23]_INST_0\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \dec_out_TDATA[24]_INST_0\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \dec_out_TDATA[25]_INST_0\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \dec_out_TDATA[26]_INST_0\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \dec_out_TDATA[27]_INST_0\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \dec_out_TDATA[28]_INST_0\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \dec_out_TDATA[29]_INST_0\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \dec_out_TDATA[2]_INST_0\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \dec_out_TDATA[30]_INST_0\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \dec_out_TDATA[3]_INST_0\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \dec_out_TDATA[4]_INST_0\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \dec_out_TDATA[5]_INST_0\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \dec_out_TDATA[6]_INST_0\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \dec_out_TDATA[7]_INST_0\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \dec_out_TDATA[8]_INST_0\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \dec_out_TDATA[9]_INST_0\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \pkt_rx_last_V_reg_284[0]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \trunc_ln2_reg_293[15]_i_1\ : label is "soft_lutpair0";
begin
  \B_V_data_1_state_reg[0]_0\ <= \^b_v_data_1_state_reg[0]_0\;
  \B_V_data_1_state_reg[1]_0\ <= \^b_v_data_1_state_reg[1]_0\;
  ap_enable_reg_pp0_iter1_reg <= \^ap_enable_reg_pp0_iter1_reg\;
  dec_counter_reg_0_sp_1 <= dec_counter_reg_0_sn_1;
  dec_counter_reg_2_sp_1 <= dec_counter_reg_2_sn_1;
  dec_counter_reg_3_sp_1 <= dec_counter_reg_3_sn_1;
  dec_counter_reg_4_sp_1 <= dec_counter_reg_4_sn_1;
  dec_counter_reg_5_sp_1 <= dec_counter_reg_5_sn_1;
  dec_counter_reg_6_sp_1 <= dec_counter_reg_6_sn_1;
  dec_counter_reg_7_sp_1 <= dec_counter_reg_7_sn_1;
  \icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\ <= \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\;
\B_V_data_1_payload_A[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"45"
    )
        port map (
      I0 => B_V_data_1_sel_wr_0,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => \^b_v_data_1_state_reg[0]_0\,
      O => B_V_data_1_load_A
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(0),
      Q => \B_V_data_1_payload_A_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(10),
      Q => \B_V_data_1_payload_A_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(11),
      Q => \B_V_data_1_payload_A_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(12),
      Q => \B_V_data_1_payload_A_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(13),
      Q => \B_V_data_1_payload_A_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(14),
      Q => \B_V_data_1_payload_A_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(15),
      Q => \B_V_data_1_payload_A_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(16),
      Q => \B_V_data_1_payload_A_reg_n_0_[16]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(17),
      Q => \B_V_data_1_payload_A_reg_n_0_[17]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(18),
      Q => \B_V_data_1_payload_A_reg_n_0_[18]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(19),
      Q => \B_V_data_1_payload_A_reg_n_0_[19]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(1),
      Q => \B_V_data_1_payload_A_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(20),
      Q => \B_V_data_1_payload_A_reg_n_0_[20]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(21),
      Q => \B_V_data_1_payload_A_reg_n_0_[21]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(22),
      Q => \B_V_data_1_payload_A_reg_n_0_[22]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(23),
      Q => \B_V_data_1_payload_A_reg_n_0_[23]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(24),
      Q => \B_V_data_1_payload_A_reg_n_0_[24]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(25),
      Q => \B_V_data_1_payload_A_reg_n_0_[25]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(26),
      Q => \B_V_data_1_payload_A_reg_n_0_[26]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(27),
      Q => \B_V_data_1_payload_A_reg_n_0_[27]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(28),
      Q => \B_V_data_1_payload_A_reg_n_0_[28]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(29),
      Q => \B_V_data_1_payload_A_reg_n_0_[29]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(2),
      Q => \B_V_data_1_payload_A_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(30),
      Q => \B_V_data_1_payload_A_reg_n_0_[30]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(31),
      Q => \B_V_data_1_payload_A_reg_n_0_[31]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(3),
      Q => \B_V_data_1_payload_A_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(4),
      Q => \B_V_data_1_payload_A_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(5),
      Q => \B_V_data_1_payload_A_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(6),
      Q => \B_V_data_1_payload_A_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(7),
      Q => \B_V_data_1_payload_A_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(8),
      Q => \B_V_data_1_payload_A_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(9),
      Q => \B_V_data_1_payload_A_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_payload_B[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"8A"
    )
        port map (
      I0 => B_V_data_1_sel_wr_0,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => \^b_v_data_1_state_reg[0]_0\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(0),
      Q => \B_V_data_1_payload_B_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(10),
      Q => \B_V_data_1_payload_B_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(11),
      Q => \B_V_data_1_payload_B_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(12),
      Q => \B_V_data_1_payload_B_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(13),
      Q => \B_V_data_1_payload_B_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(14),
      Q => \B_V_data_1_payload_B_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(15),
      Q => \B_V_data_1_payload_B_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(16),
      Q => \B_V_data_1_payload_B_reg_n_0_[16]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(17),
      Q => \B_V_data_1_payload_B_reg_n_0_[17]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(18),
      Q => \B_V_data_1_payload_B_reg_n_0_[18]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(19),
      Q => \B_V_data_1_payload_B_reg_n_0_[19]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(1),
      Q => \B_V_data_1_payload_B_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(20),
      Q => \B_V_data_1_payload_B_reg_n_0_[20]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(21),
      Q => \B_V_data_1_payload_B_reg_n_0_[21]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(22),
      Q => \B_V_data_1_payload_B_reg_n_0_[22]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(23),
      Q => \B_V_data_1_payload_B_reg_n_0_[23]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(24),
      Q => \B_V_data_1_payload_B_reg_n_0_[24]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(25),
      Q => \B_V_data_1_payload_B_reg_n_0_[25]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(26),
      Q => \B_V_data_1_payload_B_reg_n_0_[26]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(27),
      Q => \B_V_data_1_payload_B_reg_n_0_[27]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(28),
      Q => \B_V_data_1_payload_B_reg_n_0_[28]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(29),
      Q => \B_V_data_1_payload_B_reg_n_0_[29]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(2),
      Q => \B_V_data_1_payload_B_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(30),
      Q => \B_V_data_1_payload_B_reg_n_0_[30]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(31),
      Q => \B_V_data_1_payload_B_reg_n_0_[31]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(3),
      Q => \B_V_data_1_payload_B_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(4),
      Q => \B_V_data_1_payload_B_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(5),
      Q => \B_V_data_1_payload_B_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(6),
      Q => \B_V_data_1_payload_B_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(7),
      Q => \B_V_data_1_payload_B_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(8),
      Q => \B_V_data_1_payload_B_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(9),
      Q => \B_V_data_1_payload_B_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__0\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"08FFFFFFF7000000"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter1,
      I1 => icmp_ln40_reg_289,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => rx_in_TVALID_int_regslice,
      I4 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I5 => B_V_data_1_sel,
      O => ap_enable_reg_pp0_iter1_reg_0
    );
\B_V_data_1_sel_rd_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_0\,
      I1 => dec_out_TREADY,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => \B_V_data_1_sel_rd_i_1__1_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__1_n_0\,
      Q => B_V_data_1_sel_rd_reg_n_0,
      R => ap_rst_n_inv
    );
B_V_data_1_sel_wr_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FF7FFFFF00800000"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[1]_0\,
      I1 => rx_in_TVALID_int_regslice,
      I2 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I3 => \B_V_data_1_state_reg[0]_2\,
      I4 => \B_V_data_1_state_reg[0]_3\,
      I5 => B_V_data_1_sel_wr,
      O => \B_V_data_1_state_reg[1]_2\
    );
\B_V_data_1_sel_wr_i_1__0\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"7FFFFFFF80000000"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter1,
      I1 => icmp_ln40_reg_289,
      I2 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I3 => rx_in_TVALID_int_regslice,
      I4 => \^b_v_data_1_state_reg[1]_0\,
      I5 => B_V_data_1_sel_wr_0,
      O => \B_V_data_1_sel_wr_i_1__0_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__0_n_0\,
      Q => B_V_data_1_sel_wr_0,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00800000FFFFFFFF"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[1]_0\,
      I1 => rx_in_TVALID_int_regslice,
      I2 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I3 => \B_V_data_1_state_reg[0]_2\,
      I4 => \B_V_data_1_state_reg[0]_3\,
      I5 => \B_V_data_1_state_reg[0]_4\,
      O => \B_V_data_1_state_reg[1]_1\
    );
\B_V_data_1_state[0]_i_1__2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"70FC7070"
    )
        port map (
      I0 => dec_out_TREADY,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => \^b_v_data_1_state_reg[0]_0\,
      I3 => \B_V_data_1_state_reg[0]_2\,
      I4 => rx_in_TVALID_int_regslice,
      O => \B_V_data_1_state[0]_i_1__2_n_0\
    );
\B_V_data_1_state[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FDFDFDFDDDFDFDFD"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_0\,
      I1 => dec_out_TREADY,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => rx_in_TVALID_int_regslice,
      I4 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I5 => \B_V_data_1_state_reg[0]_2\,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state[1]_i_2__1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"08FF"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter1,
      I1 => icmp_ln40_reg_289,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => rx_in_TVALID_int_regslice,
      O => \^ap_enable_reg_pp0_iter1_reg\
    );
\B_V_data_1_state[1]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"C0DDFFFF"
    )
        port map (
      I0 => icmp_ln40_reg_289_pp0_iter1_reg,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => dec_out_TREADY,
      I3 => \^b_v_data_1_state_reg[0]_0\,
      I4 => ap_enable_reg_pp0_iter2,
      O => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__2_n_0\,
      Q => \^b_v_data_1_state_reg[0]_0\,
      R => ap_rst_n_inv
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \^b_v_data_1_state_reg[1]_0\,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter2_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFF777780880000"
    )
        port map (
      I0 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I1 => rx_in_TVALID_int_regslice,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => icmp_ln40_reg_289,
      I4 => ap_enable_reg_pp0_iter1,
      I5 => ap_enable_reg_pp0_iter2,
      O => \B_V_data_1_state_reg[0]_1\
    );
\dec_counter[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"5595AAAAAAAAAAAA"
    )
        port map (
      I0 => dec_counter_reg(0),
      I1 => ap_enable_reg_pp0_iter1,
      I2 => icmp_ln40_reg_289,
      I3 => \^b_v_data_1_state_reg[1]_0\,
      I4 => rx_in_TVALID_int_regslice,
      I5 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      O => dec_counter_reg_0_sn_1
    );
\dec_counter[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"AAAA6AAA"
    )
        port map (
      I0 => dec_counter_reg(2),
      I1 => dec_counter_reg(1),
      I2 => dec_counter_reg(0),
      I3 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I4 => \^ap_enable_reg_pp0_iter1_reg\,
      O => dec_counter_reg_2_sn_1
    );
\dec_counter[3]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"9AAAAAAAAAAAAAAA"
    )
        port map (
      I0 => dec_counter_reg(3),
      I1 => \^ap_enable_reg_pp0_iter1_reg\,
      I2 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I3 => dec_counter_reg(0),
      I4 => dec_counter_reg(1),
      I5 => dec_counter_reg(2),
      O => dec_counter_reg_3_sn_1
    );
\dec_counter[4]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6AAA6AAA6AAAAAAA"
    )
        port map (
      I0 => dec_counter_reg(4),
      I1 => \dec_counter[4]_i_2_n_0\,
      I2 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I3 => rx_in_TVALID_int_regslice,
      I4 => \^b_v_data_1_state_reg[1]_0\,
      I5 => \B_V_data_1_state_reg[0]_2\,
      O => dec_counter_reg_4_sn_1
    );
\dec_counter[4]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8000"
    )
        port map (
      I0 => dec_counter_reg(0),
      I1 => dec_counter_reg(1),
      I2 => dec_counter_reg(3),
      I3 => dec_counter_reg(2),
      O => \dec_counter[4]_i_2_n_0\
    );
\dec_counter[5]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"9AAA9AAA9AAAAAAA"
    )
        port map (
      I0 => dec_counter_reg(5),
      I1 => \dec_counter[5]_i_2_n_0\,
      I2 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I3 => rx_in_TVALID_int_regslice,
      I4 => \^b_v_data_1_state_reg[1]_0\,
      I5 => \B_V_data_1_state_reg[0]_2\,
      O => \dec_counter_reg[5]_0\
    );
\dec_counter[5]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFFFFFF"
    )
        port map (
      I0 => dec_counter_reg(4),
      I1 => dec_counter_reg(2),
      I2 => dec_counter_reg(3),
      I3 => dec_counter_reg(1),
      I4 => dec_counter_reg(0),
      O => \dec_counter[5]_i_2_n_0\
    );
\dec_counter[6]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6AAA6AAA6AAAAAAA"
    )
        port map (
      I0 => dec_counter_reg(6),
      I1 => dec_counter_reg_5_sn_1,
      I2 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I3 => rx_in_TVALID_int_regslice,
      I4 => \^b_v_data_1_state_reg[1]_0\,
      I5 => \B_V_data_1_state_reg[0]_2\,
      O => dec_counter_reg_6_sn_1
    );
\dec_counter[7]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"AAAA6AAA"
    )
        port map (
      I0 => dec_counter_reg(7),
      I1 => dec_counter_reg(6),
      I2 => dec_counter_reg_5_sn_1,
      I3 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I4 => \^ap_enable_reg_pp0_iter1_reg\,
      O => dec_counter_reg_7_sn_1
    );
\dec_out_TDATA[0]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(0)
    );
\dec_out_TDATA[10]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(10)
    );
\dec_out_TDATA[11]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(11)
    );
\dec_out_TDATA[12]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(12)
    );
\dec_out_TDATA[13]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(13)
    );
\dec_out_TDATA[14]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(14)
    );
\dec_out_TDATA[15]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(15)
    );
\dec_out_TDATA[16]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[16]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[16]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(16)
    );
\dec_out_TDATA[17]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[17]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[17]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(17)
    );
\dec_out_TDATA[18]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[18]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[18]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(18)
    );
\dec_out_TDATA[19]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[19]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[19]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(19)
    );
\dec_out_TDATA[1]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(1)
    );
\dec_out_TDATA[20]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[20]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[20]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(20)
    );
\dec_out_TDATA[21]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[21]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[21]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(21)
    );
\dec_out_TDATA[22]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[22]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[22]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(22)
    );
\dec_out_TDATA[23]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[23]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[23]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(23)
    );
\dec_out_TDATA[24]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[24]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[24]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(24)
    );
\dec_out_TDATA[25]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[25]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[25]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(25)
    );
\dec_out_TDATA[26]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[26]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[26]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(26)
    );
\dec_out_TDATA[27]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[27]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[27]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(27)
    );
\dec_out_TDATA[28]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[28]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[28]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(28)
    );
\dec_out_TDATA[29]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[29]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[29]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(29)
    );
\dec_out_TDATA[2]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(2)
    );
\dec_out_TDATA[30]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[30]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[30]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(30)
    );
\dec_out_TDATA[31]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(31)
    );
\dec_out_TDATA[3]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(3)
    );
\dec_out_TDATA[4]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(4)
    );
\dec_out_TDATA[5]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(5)
    );
\dec_out_TDATA[6]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(6)
    );
\dec_out_TDATA[7]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(7)
    );
\dec_out_TDATA[8]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(8)
    );
\dec_out_TDATA[9]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => dec_out_TDATA(9)
    );
\pkt_rx_last_V_reg_284[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80888888"
    )
        port map (
      I0 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I1 => rx_in_TVALID_int_regslice,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => icmp_ln40_reg_289,
      I4 => ap_enable_reg_pp0_iter1,
      O => ap_block_pp0_stage0_11001
    );
\trunc_ln2_reg_293[15]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"40000000"
    )
        port map (
      I0 => \^ap_enable_reg_pp0_iter1_reg\,
      I1 => \^icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\,
      I2 => dec_counter_reg(6),
      I3 => dec_counter_reg_5_sn_1,
      I4 => dec_counter_reg(7),
      O => trunc_ln2_reg_2930
    );
\trunc_ln2_reg_293[15]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8000000000000000"
    )
        port map (
      I0 => dec_counter_reg(5),
      I1 => dec_counter_reg(0),
      I2 => dec_counter_reg(1),
      I3 => dec_counter_reg(3),
      I4 => dec_counter_reg(2),
      I5 => dec_counter_reg(4),
      O => dec_counter_reg_5_sn_1
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both_0 is
  port (
    \B_V_data_1_state_reg[1]_0\ : out STD_LOGIC;
    rx_in_TVALID_int_regslice : out STD_LOGIC;
    B_V_data_1_sel : out STD_LOGIC;
    O : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_i_reg[7]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_i_reg[11]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_i_reg[14]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_i_reg[14]_0\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \B_V_data_1_payload_B_reg[15]_0\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    D : out STD_LOGIC_VECTOR ( 15 downto 0 );
    \acc_q_reg[3]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_q_reg[7]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_q_reg[11]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_q_reg[14]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_q_reg[14]_0\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \B_V_data_1_payload_B_reg[31]_0\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \acc_q_reg[21]\ : out STD_LOGIC_VECTOR ( 15 downto 0 );
    \B_V_data_1_state_reg[0]_0\ : out STD_LOGIC;
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    B_V_data_1_sel_rd_reg_0 : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_1\ : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_2\ : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_3\ : in STD_LOGIC;
    rx_in_TVALID : in STD_LOGIC;
    acc_i_reg : in STD_LOGIC_VECTOR ( 23 downto 0 );
    acc_q_reg : in STD_LOGIC_VECTOR ( 23 downto 0 );
    ap_enable_reg_pp0_iter1 : in STD_LOGIC;
    rx_in_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both_0 : entity is "fsk_decimator_regslice_both";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both_0;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both_0 is
  signal B_V_data_1_load_A : STD_LOGIC;
  signal B_V_data_1_load_B : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[16]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[17]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[18]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[19]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[20]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[21]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[22]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[23]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[24]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[25]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[26]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[27]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[28]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[29]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[2]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[30]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[31]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[3]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[4]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[5]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[6]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[7]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[8]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[9]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[16]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[17]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[18]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[19]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[20]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[21]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[22]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[23]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[24]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[25]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[26]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[27]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[28]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[29]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[2]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[30]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[31]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[3]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[4]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[5]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[6]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[7]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[8]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[9]\ : STD_LOGIC;
  signal \^b_v_data_1_sel\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__1_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[1]_0\ : STD_LOGIC;
  signal \acc_i[0]_i_2_n_0\ : STD_LOGIC;
  signal \acc_i[0]_i_3_n_0\ : STD_LOGIC;
  signal \acc_i[0]_i_4_n_0\ : STD_LOGIC;
  signal \acc_i[0]_i_5_n_0\ : STD_LOGIC;
  signal \acc_i[12]_i_2_n_0\ : STD_LOGIC;
  signal \acc_i[12]_i_3_n_0\ : STD_LOGIC;
  signal \acc_i[12]_i_4_n_0\ : STD_LOGIC;
  signal \acc_i[12]_i_5_n_0\ : STD_LOGIC;
  signal \acc_i[12]_i_6_n_0\ : STD_LOGIC;
  signal \acc_i[16]_i_2_n_0\ : STD_LOGIC;
  signal \acc_i[16]_i_3_n_0\ : STD_LOGIC;
  signal \acc_i[16]_i_4_n_0\ : STD_LOGIC;
  signal \acc_i[16]_i_5_n_0\ : STD_LOGIC;
  signal \acc_i[16]_i_6_n_0\ : STD_LOGIC;
  signal \acc_i[16]_i_7_n_0\ : STD_LOGIC;
  signal \acc_i[16]_i_8_n_0\ : STD_LOGIC;
  signal \acc_i[16]_i_9_n_0\ : STD_LOGIC;
  signal \acc_i[20]_i_2_n_0\ : STD_LOGIC;
  signal \acc_i[20]_i_3_n_0\ : STD_LOGIC;
  signal \acc_i[20]_i_4_n_0\ : STD_LOGIC;
  signal \acc_i[20]_i_5_n_0\ : STD_LOGIC;
  signal \acc_i[20]_i_6_n_0\ : STD_LOGIC;
  signal \acc_i[20]_i_7_n_0\ : STD_LOGIC;
  signal \acc_i[20]_i_8_n_0\ : STD_LOGIC;
  signal \acc_i[4]_i_2_n_0\ : STD_LOGIC;
  signal \acc_i[4]_i_3_n_0\ : STD_LOGIC;
  signal \acc_i[4]_i_4_n_0\ : STD_LOGIC;
  signal \acc_i[4]_i_5_n_0\ : STD_LOGIC;
  signal \acc_i[8]_i_2_n_0\ : STD_LOGIC;
  signal \acc_i[8]_i_3_n_0\ : STD_LOGIC;
  signal \acc_i[8]_i_4_n_0\ : STD_LOGIC;
  signal \acc_i[8]_i_5_n_0\ : STD_LOGIC;
  signal \acc_i_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \acc_i_reg[0]_i_1_n_1\ : STD_LOGIC;
  signal \acc_i_reg[0]_i_1_n_2\ : STD_LOGIC;
  signal \acc_i_reg[0]_i_1_n_3\ : STD_LOGIC;
  signal \acc_i_reg[12]_i_1_n_0\ : STD_LOGIC;
  signal \acc_i_reg[12]_i_1_n_1\ : STD_LOGIC;
  signal \acc_i_reg[12]_i_1_n_2\ : STD_LOGIC;
  signal \acc_i_reg[12]_i_1_n_3\ : STD_LOGIC;
  signal \acc_i_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \acc_i_reg[16]_i_1_n_1\ : STD_LOGIC;
  signal \acc_i_reg[16]_i_1_n_2\ : STD_LOGIC;
  signal \acc_i_reg[16]_i_1_n_3\ : STD_LOGIC;
  signal \acc_i_reg[20]_i_1_n_1\ : STD_LOGIC;
  signal \acc_i_reg[20]_i_1_n_2\ : STD_LOGIC;
  signal \acc_i_reg[20]_i_1_n_3\ : STD_LOGIC;
  signal \acc_i_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \acc_i_reg[4]_i_1_n_1\ : STD_LOGIC;
  signal \acc_i_reg[4]_i_1_n_2\ : STD_LOGIC;
  signal \acc_i_reg[4]_i_1_n_3\ : STD_LOGIC;
  signal \acc_i_reg[8]_i_1_n_0\ : STD_LOGIC;
  signal \acc_i_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \acc_i_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \acc_i_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal \acc_q[0]_i_2_n_0\ : STD_LOGIC;
  signal \acc_q[0]_i_3_n_0\ : STD_LOGIC;
  signal \acc_q[0]_i_4_n_0\ : STD_LOGIC;
  signal \acc_q[0]_i_5_n_0\ : STD_LOGIC;
  signal \acc_q[12]_i_2_n_0\ : STD_LOGIC;
  signal \acc_q[12]_i_3_n_0\ : STD_LOGIC;
  signal \acc_q[12]_i_4_n_0\ : STD_LOGIC;
  signal \acc_q[12]_i_5_n_0\ : STD_LOGIC;
  signal \acc_q[12]_i_6_n_0\ : STD_LOGIC;
  signal \acc_q[16]_i_2_n_0\ : STD_LOGIC;
  signal \acc_q[16]_i_3_n_0\ : STD_LOGIC;
  signal \acc_q[16]_i_4_n_0\ : STD_LOGIC;
  signal \acc_q[16]_i_5_n_0\ : STD_LOGIC;
  signal \acc_q[16]_i_6_n_0\ : STD_LOGIC;
  signal \acc_q[16]_i_7_n_0\ : STD_LOGIC;
  signal \acc_q[16]_i_8_n_0\ : STD_LOGIC;
  signal \acc_q[16]_i_9_n_0\ : STD_LOGIC;
  signal \acc_q[20]_i_2_n_0\ : STD_LOGIC;
  signal \acc_q[20]_i_3_n_0\ : STD_LOGIC;
  signal \acc_q[20]_i_4_n_0\ : STD_LOGIC;
  signal \acc_q[20]_i_5_n_0\ : STD_LOGIC;
  signal \acc_q[20]_i_6_n_0\ : STD_LOGIC;
  signal \acc_q[20]_i_7_n_0\ : STD_LOGIC;
  signal \acc_q[20]_i_8_n_0\ : STD_LOGIC;
  signal \acc_q[4]_i_2_n_0\ : STD_LOGIC;
  signal \acc_q[4]_i_3_n_0\ : STD_LOGIC;
  signal \acc_q[4]_i_4_n_0\ : STD_LOGIC;
  signal \acc_q[4]_i_5_n_0\ : STD_LOGIC;
  signal \acc_q[8]_i_2_n_0\ : STD_LOGIC;
  signal \acc_q[8]_i_3_n_0\ : STD_LOGIC;
  signal \acc_q[8]_i_4_n_0\ : STD_LOGIC;
  signal \acc_q[8]_i_5_n_0\ : STD_LOGIC;
  signal \acc_q_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \acc_q_reg[0]_i_1_n_1\ : STD_LOGIC;
  signal \acc_q_reg[0]_i_1_n_2\ : STD_LOGIC;
  signal \acc_q_reg[0]_i_1_n_3\ : STD_LOGIC;
  signal \acc_q_reg[12]_i_1_n_0\ : STD_LOGIC;
  signal \acc_q_reg[12]_i_1_n_1\ : STD_LOGIC;
  signal \acc_q_reg[12]_i_1_n_2\ : STD_LOGIC;
  signal \acc_q_reg[12]_i_1_n_3\ : STD_LOGIC;
  signal \acc_q_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \acc_q_reg[16]_i_1_n_1\ : STD_LOGIC;
  signal \acc_q_reg[16]_i_1_n_2\ : STD_LOGIC;
  signal \acc_q_reg[16]_i_1_n_3\ : STD_LOGIC;
  signal \acc_q_reg[20]_i_1_n_1\ : STD_LOGIC;
  signal \acc_q_reg[20]_i_1_n_2\ : STD_LOGIC;
  signal \acc_q_reg[20]_i_1_n_3\ : STD_LOGIC;
  signal \acc_q_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \acc_q_reg[4]_i_1_n_1\ : STD_LOGIC;
  signal \acc_q_reg[4]_i_1_n_2\ : STD_LOGIC;
  signal \acc_q_reg[4]_i_1_n_3\ : STD_LOGIC;
  signal \acc_q_reg[8]_i_1_n_0\ : STD_LOGIC;
  signal \acc_q_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \acc_q_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \acc_q_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal \^rx_in_tvalid_int_regslice\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[11]_i_2_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[11]_i_3_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[11]_i_4_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[11]_i_5_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[11]_i_6_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[15]_i_4_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[15]_i_5_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[15]_i_6_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[15]_i_7_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_10_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_11_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_12_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_13_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_14_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_15_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_3_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_4_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_5_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_6_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_8_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[3]_i_9_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[7]_i_2_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[7]_i_3_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[7]_i_4_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[7]_i_5_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293[7]_i_6_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[11]_i_1_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[11]_i_1_n_1\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[11]_i_1_n_2\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[11]_i_1_n_3\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[15]_i_2_n_1\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[15]_i_2_n_2\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[15]_i_2_n_3\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_1_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_1_n_1\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_1_n_2\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_1_n_3\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_2_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_2_n_1\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_2_n_2\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_2_n_3\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_7_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_7_n_1\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_7_n_2\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[3]_i_7_n_3\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[7]_i_1_n_0\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[7]_i_1_n_1\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[7]_i_1_n_2\ : STD_LOGIC;
  signal \trunc_ln2_reg_293_reg[7]_i_1_n_3\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[11]_i_2_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[11]_i_3_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[11]_i_4_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[11]_i_5_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[11]_i_6_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[15]_i_2_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[15]_i_3_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[15]_i_4_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[15]_i_5_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_10_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_11_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_12_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_13_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_14_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_15_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_3_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_4_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_5_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_6_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_8_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[3]_i_9_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[7]_i_2_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[7]_i_3_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[7]_i_4_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[7]_i_5_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298[7]_i_6_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[11]_i_1_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[11]_i_1_n_1\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[11]_i_1_n_2\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[11]_i_1_n_3\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[15]_i_1_n_1\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[15]_i_1_n_2\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[15]_i_1_n_3\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_1_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_1_n_1\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_1_n_2\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_1_n_3\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_2_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_2_n_1\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_2_n_2\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_2_n_3\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_7_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_7_n_1\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_7_n_2\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[3]_i_7_n_3\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[7]_i_1_n_0\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[7]_i_1_n_1\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[7]_i_1_n_2\ : STD_LOGIC;
  signal \trunc_ln3_reg_298_reg[7]_i_1_n_3\ : STD_LOGIC;
  signal \NLW_acc_i_reg[20]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_acc_q_reg[20]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_trunc_ln2_reg_293_reg[15]_i_2_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_trunc_ln2_reg_293_reg[3]_i_2_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_trunc_ln2_reg_293_reg[3]_i_7_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_trunc_ln3_reg_298_reg[15]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_trunc_ln3_reg_298_reg[3]_i_2_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_trunc_ln3_reg_298_reg[3]_i_7_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \trunc_ln2_reg_293_reg[11]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln2_reg_293_reg[15]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln2_reg_293_reg[3]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln2_reg_293_reg[3]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln2_reg_293_reg[3]_i_7\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln2_reg_293_reg[7]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln3_reg_298_reg[11]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln3_reg_298_reg[15]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln3_reg_298_reg[3]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln3_reg_298_reg[3]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln3_reg_298_reg[3]_i_7\ : label is 35;
  attribute ADDER_THRESHOLD of \trunc_ln3_reg_298_reg[7]_i_1\ : label is 35;
begin
  B_V_data_1_sel <= \^b_v_data_1_sel\;
  \B_V_data_1_state_reg[1]_0\ <= \^b_v_data_1_state_reg[1]_0\;
  rx_in_TVALID_int_regslice <= \^rx_in_tvalid_int_regslice\;
\B_V_data_1_payload_A[31]_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"45"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => \^rx_in_tvalid_int_regslice\,
      O => B_V_data_1_load_A
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(0),
      Q => \B_V_data_1_payload_A_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(10),
      Q => \B_V_data_1_payload_A_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(11),
      Q => \B_V_data_1_payload_A_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(12),
      Q => \B_V_data_1_payload_A_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(13),
      Q => \B_V_data_1_payload_A_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(14),
      Q => \B_V_data_1_payload_A_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(15),
      Q => \B_V_data_1_payload_A_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(16),
      Q => \B_V_data_1_payload_A_reg_n_0_[16]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(17),
      Q => \B_V_data_1_payload_A_reg_n_0_[17]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(18),
      Q => \B_V_data_1_payload_A_reg_n_0_[18]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(19),
      Q => \B_V_data_1_payload_A_reg_n_0_[19]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(1),
      Q => \B_V_data_1_payload_A_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(20),
      Q => \B_V_data_1_payload_A_reg_n_0_[20]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(21),
      Q => \B_V_data_1_payload_A_reg_n_0_[21]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(22),
      Q => \B_V_data_1_payload_A_reg_n_0_[22]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(23),
      Q => \B_V_data_1_payload_A_reg_n_0_[23]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(24),
      Q => \B_V_data_1_payload_A_reg_n_0_[24]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(25),
      Q => \B_V_data_1_payload_A_reg_n_0_[25]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(26),
      Q => \B_V_data_1_payload_A_reg_n_0_[26]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(27),
      Q => \B_V_data_1_payload_A_reg_n_0_[27]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(28),
      Q => \B_V_data_1_payload_A_reg_n_0_[28]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(29),
      Q => \B_V_data_1_payload_A_reg_n_0_[29]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(2),
      Q => \B_V_data_1_payload_A_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(30),
      Q => \B_V_data_1_payload_A_reg_n_0_[30]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(31),
      Q => \B_V_data_1_payload_A_reg_n_0_[31]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(3),
      Q => \B_V_data_1_payload_A_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(4),
      Q => \B_V_data_1_payload_A_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(5),
      Q => \B_V_data_1_payload_A_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(6),
      Q => \B_V_data_1_payload_A_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(7),
      Q => \B_V_data_1_payload_A_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(8),
      Q => \B_V_data_1_payload_A_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => rx_in_TDATA(9),
      Q => \B_V_data_1_payload_A_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_payload_B[31]_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"8A"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => \^rx_in_tvalid_int_regslice\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(0),
      Q => \B_V_data_1_payload_B_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(10),
      Q => \B_V_data_1_payload_B_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(11),
      Q => \B_V_data_1_payload_B_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(12),
      Q => \B_V_data_1_payload_B_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(13),
      Q => \B_V_data_1_payload_B_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(14),
      Q => \B_V_data_1_payload_B_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(15),
      Q => \B_V_data_1_payload_B_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(16),
      Q => \B_V_data_1_payload_B_reg_n_0_[16]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(17),
      Q => \B_V_data_1_payload_B_reg_n_0_[17]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(18),
      Q => \B_V_data_1_payload_B_reg_n_0_[18]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(19),
      Q => \B_V_data_1_payload_B_reg_n_0_[19]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(1),
      Q => \B_V_data_1_payload_B_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(20),
      Q => \B_V_data_1_payload_B_reg_n_0_[20]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(21),
      Q => \B_V_data_1_payload_B_reg_n_0_[21]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(22),
      Q => \B_V_data_1_payload_B_reg_n_0_[22]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(23),
      Q => \B_V_data_1_payload_B_reg_n_0_[23]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(24),
      Q => \B_V_data_1_payload_B_reg_n_0_[24]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(25),
      Q => \B_V_data_1_payload_B_reg_n_0_[25]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(26),
      Q => \B_V_data_1_payload_B_reg_n_0_[26]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(27),
      Q => \B_V_data_1_payload_B_reg_n_0_[27]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(28),
      Q => \B_V_data_1_payload_B_reg_n_0_[28]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(29),
      Q => \B_V_data_1_payload_B_reg_n_0_[29]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(2),
      Q => \B_V_data_1_payload_B_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(30),
      Q => \B_V_data_1_payload_B_reg_n_0_[30]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(31),
      Q => \B_V_data_1_payload_B_reg_n_0_[31]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(3),
      Q => \B_V_data_1_payload_B_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(4),
      Q => \B_V_data_1_payload_B_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(5),
      Q => \B_V_data_1_payload_B_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(6),
      Q => \B_V_data_1_payload_B_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(7),
      Q => \B_V_data_1_payload_B_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(8),
      Q => \B_V_data_1_payload_B_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => rx_in_TDATA(9),
      Q => \B_V_data_1_payload_B_reg_n_0_[9]\,
      R => '0'
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_sel_rd_reg_0,
      Q => \^b_v_data_1_sel\,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => rx_in_TVALID,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__1_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__1_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1__1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"D8F8D8F8D8F8F8F8"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[1]_0\,
      I1 => rx_in_TVALID,
      I2 => \^rx_in_tvalid_int_regslice\,
      I3 => \B_V_data_1_state_reg[0]_1\,
      I4 => \B_V_data_1_state_reg[0]_2\,
      I5 => \B_V_data_1_state_reg[0]_3\,
      O => \B_V_data_1_state[0]_i_1__1_n_0\
    );
\B_V_data_1_state[1]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"DDD5FFFFDDD5DDD5"
    )
        port map (
      I0 => \^rx_in_tvalid_int_regslice\,
      I1 => \B_V_data_1_state_reg[0]_1\,
      I2 => \B_V_data_1_state_reg[0]_2\,
      I3 => \B_V_data_1_state_reg[0]_3\,
      I4 => rx_in_TVALID,
      I5 => \^b_v_data_1_state_reg[1]_0\,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__1_n_0\,
      Q => \^rx_in_tvalid_int_regslice\,
      R => ap_rst_n_inv
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \^b_v_data_1_state_reg[1]_0\,
      R => ap_rst_n_inv
    );
\acc_i[0]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      I3 => acc_i_reg(3),
      O => \acc_i[0]_i_2_n_0\
    );
\acc_i[0]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I3 => acc_i_reg(2),
      O => \acc_i[0]_i_3_n_0\
    );
\acc_i[0]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      I3 => acc_i_reg(1),
      O => \acc_i[0]_i_4_n_0\
    );
\acc_i[0]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I3 => acc_i_reg(0),
      O => \acc_i[0]_i_5_n_0\
    );
\acc_i[12]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \acc_i[12]_i_2_n_0\
    );
\acc_i[12]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(15),
      O => \acc_i[12]_i_3_n_0\
    );
\acc_i[12]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I3 => acc_i_reg(14),
      O => \acc_i[12]_i_4_n_0\
    );
\acc_i[12]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      I3 => acc_i_reg(13),
      O => \acc_i[12]_i_5_n_0\
    );
\acc_i[12]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I3 => acc_i_reg(12),
      O => \acc_i[12]_i_6_n_0\
    );
\acc_i[16]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \acc_i[16]_i_2_n_0\
    );
\acc_i[16]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \acc_i[16]_i_3_n_0\
    );
\acc_i[16]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \acc_i[16]_i_4_n_0\
    );
\acc_i[16]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \acc_i[16]_i_5_n_0\
    );
\acc_i[16]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(19),
      O => \acc_i[16]_i_6_n_0\
    );
\acc_i[16]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(18),
      O => \acc_i[16]_i_7_n_0\
    );
\acc_i[16]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(17),
      O => \acc_i[16]_i_8_n_0\
    );
\acc_i[16]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(16),
      O => \acc_i[16]_i_9_n_0\
    );
\acc_i[20]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \acc_i[20]_i_2_n_0\
    );
\acc_i[20]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \acc_i[20]_i_3_n_0\
    );
\acc_i[20]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \acc_i[20]_i_4_n_0\
    );
\acc_i[20]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(23),
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      O => \acc_i[20]_i_5_n_0\
    );
\acc_i[20]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(22),
      O => \acc_i[20]_i_6_n_0\
    );
\acc_i[20]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(21),
      O => \acc_i[20]_i_7_n_0\
    );
\acc_i[20]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(20),
      O => \acc_i[20]_i_8_n_0\
    );
\acc_i[4]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      I3 => acc_i_reg(7),
      O => \acc_i[4]_i_2_n_0\
    );
\acc_i[4]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I3 => acc_i_reg(6),
      O => \acc_i[4]_i_3_n_0\
    );
\acc_i[4]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      I3 => acc_i_reg(5),
      O => \acc_i[4]_i_4_n_0\
    );
\acc_i[4]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I3 => acc_i_reg(4),
      O => \acc_i[4]_i_5_n_0\
    );
\acc_i[8]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      I3 => acc_i_reg(11),
      O => \acc_i[8]_i_2_n_0\
    );
\acc_i[8]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I3 => acc_i_reg(10),
      O => \acc_i[8]_i_3_n_0\
    );
\acc_i[8]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      I3 => acc_i_reg(9),
      O => \acc_i[8]_i_4_n_0\
    );
\acc_i[8]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I3 => acc_i_reg(8),
      O => \acc_i[8]_i_5_n_0\
    );
\acc_i_reg[0]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \acc_i_reg[0]_i_1_n_0\,
      CO(2) => \acc_i_reg[0]_i_1_n_1\,
      CO(1) => \acc_i_reg[0]_i_1_n_2\,
      CO(0) => \acc_i_reg[0]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_i_reg(3 downto 0),
      O(3 downto 0) => O(3 downto 0),
      S(3) => \acc_i[0]_i_2_n_0\,
      S(2) => \acc_i[0]_i_3_n_0\,
      S(1) => \acc_i[0]_i_4_n_0\,
      S(0) => \acc_i[0]_i_5_n_0\
    );
\acc_i_reg[12]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_i_reg[8]_i_1_n_0\,
      CO(3) => \acc_i_reg[12]_i_1_n_0\,
      CO(2) => \acc_i_reg[12]_i_1_n_1\,
      CO(1) => \acc_i_reg[12]_i_1_n_2\,
      CO(0) => \acc_i_reg[12]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \acc_i[12]_i_2_n_0\,
      DI(2 downto 0) => acc_i_reg(14 downto 12),
      O(3 downto 0) => \acc_i_reg[14]\(3 downto 0),
      S(3) => \acc_i[12]_i_3_n_0\,
      S(2) => \acc_i[12]_i_4_n_0\,
      S(1) => \acc_i[12]_i_5_n_0\,
      S(0) => \acc_i[12]_i_6_n_0\
    );
\acc_i_reg[16]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_i_reg[12]_i_1_n_0\,
      CO(3) => \acc_i_reg[16]_i_1_n_0\,
      CO(2) => \acc_i_reg[16]_i_1_n_1\,
      CO(1) => \acc_i_reg[16]_i_1_n_2\,
      CO(0) => \acc_i_reg[16]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \acc_i[16]_i_2_n_0\,
      DI(2) => \acc_i[16]_i_3_n_0\,
      DI(1) => \acc_i[16]_i_4_n_0\,
      DI(0) => \acc_i[16]_i_5_n_0\,
      O(3 downto 0) => \acc_i_reg[14]_0\(3 downto 0),
      S(3) => \acc_i[16]_i_6_n_0\,
      S(2) => \acc_i[16]_i_7_n_0\,
      S(1) => \acc_i[16]_i_8_n_0\,
      S(0) => \acc_i[16]_i_9_n_0\
    );
\acc_i_reg[20]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_i_reg[16]_i_1_n_0\,
      CO(3) => \NLW_acc_i_reg[20]_i_1_CO_UNCONNECTED\(3),
      CO(2) => \acc_i_reg[20]_i_1_n_1\,
      CO(1) => \acc_i_reg[20]_i_1_n_2\,
      CO(0) => \acc_i_reg[20]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2) => \acc_i[20]_i_2_n_0\,
      DI(1) => \acc_i[20]_i_3_n_0\,
      DI(0) => \acc_i[20]_i_4_n_0\,
      O(3 downto 0) => \B_V_data_1_payload_B_reg[15]_0\(3 downto 0),
      S(3) => \acc_i[20]_i_5_n_0\,
      S(2) => \acc_i[20]_i_6_n_0\,
      S(1) => \acc_i[20]_i_7_n_0\,
      S(0) => \acc_i[20]_i_8_n_0\
    );
\acc_i_reg[4]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_i_reg[0]_i_1_n_0\,
      CO(3) => \acc_i_reg[4]_i_1_n_0\,
      CO(2) => \acc_i_reg[4]_i_1_n_1\,
      CO(1) => \acc_i_reg[4]_i_1_n_2\,
      CO(0) => \acc_i_reg[4]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_i_reg(7 downto 4),
      O(3 downto 0) => \acc_i_reg[7]\(3 downto 0),
      S(3) => \acc_i[4]_i_2_n_0\,
      S(2) => \acc_i[4]_i_3_n_0\,
      S(1) => \acc_i[4]_i_4_n_0\,
      S(0) => \acc_i[4]_i_5_n_0\
    );
\acc_i_reg[8]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_i_reg[4]_i_1_n_0\,
      CO(3) => \acc_i_reg[8]_i_1_n_0\,
      CO(2) => \acc_i_reg[8]_i_1_n_1\,
      CO(1) => \acc_i_reg[8]_i_1_n_2\,
      CO(0) => \acc_i_reg[8]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_i_reg(11 downto 8),
      O(3 downto 0) => \acc_i_reg[11]\(3 downto 0),
      S(3) => \acc_i[8]_i_2_n_0\,
      S(2) => \acc_i[8]_i_3_n_0\,
      S(1) => \acc_i[8]_i_4_n_0\,
      S(0) => \acc_i[8]_i_5_n_0\
    );
\acc_q[0]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[19]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[19]\,
      I3 => acc_q_reg(3),
      O => \acc_q[0]_i_2_n_0\
    );
\acc_q[0]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[18]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[18]\,
      I3 => acc_q_reg(2),
      O => \acc_q[0]_i_3_n_0\
    );
\acc_q[0]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[17]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[17]\,
      I3 => acc_q_reg(1),
      O => \acc_q[0]_i_4_n_0\
    );
\acc_q[0]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[16]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[16]\,
      I3 => acc_q_reg(0),
      O => \acc_q[0]_i_5_n_0\
    );
\acc_q[12]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \acc_q[12]_i_2_n_0\
    );
\acc_q[12]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(15),
      O => \acc_q[12]_i_3_n_0\
    );
\acc_q[12]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[30]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[30]\,
      I3 => acc_q_reg(14),
      O => \acc_q[12]_i_4_n_0\
    );
\acc_q[12]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[29]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[29]\,
      I3 => acc_q_reg(13),
      O => \acc_q[12]_i_5_n_0\
    );
\acc_q[12]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[28]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[28]\,
      I3 => acc_q_reg(12),
      O => \acc_q[12]_i_6_n_0\
    );
\acc_q[16]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \acc_q[16]_i_2_n_0\
    );
\acc_q[16]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \acc_q[16]_i_3_n_0\
    );
\acc_q[16]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \acc_q[16]_i_4_n_0\
    );
\acc_q[16]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \acc_q[16]_i_5_n_0\
    );
\acc_q[16]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(19),
      O => \acc_q[16]_i_6_n_0\
    );
\acc_q[16]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(18),
      O => \acc_q[16]_i_7_n_0\
    );
\acc_q[16]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(17),
      O => \acc_q[16]_i_8_n_0\
    );
\acc_q[16]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(16),
      O => \acc_q[16]_i_9_n_0\
    );
\acc_q[20]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \acc_q[20]_i_2_n_0\
    );
\acc_q[20]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \acc_q[20]_i_3_n_0\
    );
\acc_q[20]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \acc_q[20]_i_4_n_0\
    );
\acc_q[20]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(23),
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      O => \acc_q[20]_i_5_n_0\
    );
\acc_q[20]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(22),
      O => \acc_q[20]_i_6_n_0\
    );
\acc_q[20]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(21),
      O => \acc_q[20]_i_7_n_0\
    );
\acc_q[20]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(20),
      O => \acc_q[20]_i_8_n_0\
    );
\acc_q[4]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[23]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[23]\,
      I3 => acc_q_reg(7),
      O => \acc_q[4]_i_2_n_0\
    );
\acc_q[4]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[22]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[22]\,
      I3 => acc_q_reg(6),
      O => \acc_q[4]_i_3_n_0\
    );
\acc_q[4]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[21]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[21]\,
      I3 => acc_q_reg(5),
      O => \acc_q[4]_i_4_n_0\
    );
\acc_q[4]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[20]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[20]\,
      I3 => acc_q_reg(4),
      O => \acc_q[4]_i_5_n_0\
    );
\acc_q[8]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[27]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[27]\,
      I3 => acc_q_reg(11),
      O => \acc_q[8]_i_2_n_0\
    );
\acc_q[8]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[26]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[26]\,
      I3 => acc_q_reg(10),
      O => \acc_q[8]_i_3_n_0\
    );
\acc_q[8]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[25]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[25]\,
      I3 => acc_q_reg(9),
      O => \acc_q[8]_i_4_n_0\
    );
\acc_q[8]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[24]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[24]\,
      I3 => acc_q_reg(8),
      O => \acc_q[8]_i_5_n_0\
    );
\acc_q_reg[0]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \acc_q_reg[0]_i_1_n_0\,
      CO(2) => \acc_q_reg[0]_i_1_n_1\,
      CO(1) => \acc_q_reg[0]_i_1_n_2\,
      CO(0) => \acc_q_reg[0]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_q_reg(3 downto 0),
      O(3 downto 0) => \acc_q_reg[3]\(3 downto 0),
      S(3) => \acc_q[0]_i_2_n_0\,
      S(2) => \acc_q[0]_i_3_n_0\,
      S(1) => \acc_q[0]_i_4_n_0\,
      S(0) => \acc_q[0]_i_5_n_0\
    );
\acc_q_reg[12]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_q_reg[8]_i_1_n_0\,
      CO(3) => \acc_q_reg[12]_i_1_n_0\,
      CO(2) => \acc_q_reg[12]_i_1_n_1\,
      CO(1) => \acc_q_reg[12]_i_1_n_2\,
      CO(0) => \acc_q_reg[12]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \acc_q[12]_i_2_n_0\,
      DI(2 downto 0) => acc_q_reg(14 downto 12),
      O(3 downto 0) => \acc_q_reg[14]\(3 downto 0),
      S(3) => \acc_q[12]_i_3_n_0\,
      S(2) => \acc_q[12]_i_4_n_0\,
      S(1) => \acc_q[12]_i_5_n_0\,
      S(0) => \acc_q[12]_i_6_n_0\
    );
\acc_q_reg[16]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_q_reg[12]_i_1_n_0\,
      CO(3) => \acc_q_reg[16]_i_1_n_0\,
      CO(2) => \acc_q_reg[16]_i_1_n_1\,
      CO(1) => \acc_q_reg[16]_i_1_n_2\,
      CO(0) => \acc_q_reg[16]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \acc_q[16]_i_2_n_0\,
      DI(2) => \acc_q[16]_i_3_n_0\,
      DI(1) => \acc_q[16]_i_4_n_0\,
      DI(0) => \acc_q[16]_i_5_n_0\,
      O(3 downto 0) => \acc_q_reg[14]_0\(3 downto 0),
      S(3) => \acc_q[16]_i_6_n_0\,
      S(2) => \acc_q[16]_i_7_n_0\,
      S(1) => \acc_q[16]_i_8_n_0\,
      S(0) => \acc_q[16]_i_9_n_0\
    );
\acc_q_reg[20]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_q_reg[16]_i_1_n_0\,
      CO(3) => \NLW_acc_q_reg[20]_i_1_CO_UNCONNECTED\(3),
      CO(2) => \acc_q_reg[20]_i_1_n_1\,
      CO(1) => \acc_q_reg[20]_i_1_n_2\,
      CO(0) => \acc_q_reg[20]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2) => \acc_q[20]_i_2_n_0\,
      DI(1) => \acc_q[20]_i_3_n_0\,
      DI(0) => \acc_q[20]_i_4_n_0\,
      O(3 downto 0) => \B_V_data_1_payload_B_reg[31]_0\(3 downto 0),
      S(3) => \acc_q[20]_i_5_n_0\,
      S(2) => \acc_q[20]_i_6_n_0\,
      S(1) => \acc_q[20]_i_7_n_0\,
      S(0) => \acc_q[20]_i_8_n_0\
    );
\acc_q_reg[4]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_q_reg[0]_i_1_n_0\,
      CO(3) => \acc_q_reg[4]_i_1_n_0\,
      CO(2) => \acc_q_reg[4]_i_1_n_1\,
      CO(1) => \acc_q_reg[4]_i_1_n_2\,
      CO(0) => \acc_q_reg[4]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_q_reg(7 downto 4),
      O(3 downto 0) => \acc_q_reg[7]\(3 downto 0),
      S(3) => \acc_q[4]_i_2_n_0\,
      S(2) => \acc_q[4]_i_3_n_0\,
      S(1) => \acc_q[4]_i_4_n_0\,
      S(0) => \acc_q[4]_i_5_n_0\
    );
\acc_q_reg[8]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \acc_q_reg[4]_i_1_n_0\,
      CO(3) => \acc_q_reg[8]_i_1_n_0\,
      CO(2) => \acc_q_reg[8]_i_1_n_1\,
      CO(1) => \acc_q_reg[8]_i_1_n_2\,
      CO(0) => \acc_q_reg[8]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_q_reg(11 downto 8),
      O(3 downto 0) => \acc_q_reg[11]\(3 downto 0),
      S(3) => \acc_q[8]_i_2_n_0\,
      S(2) => \acc_q[8]_i_3_n_0\,
      S(1) => \acc_q[8]_i_4_n_0\,
      S(0) => \acc_q[8]_i_5_n_0\
    );
ap_enable_reg_pp0_iter1_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"F8"
    )
        port map (
      I0 => \^rx_in_tvalid_int_regslice\,
      I1 => \B_V_data_1_state_reg[0]_1\,
      I2 => ap_enable_reg_pp0_iter1,
      O => \B_V_data_1_state_reg[0]_0\
    );
\trunc_ln2_reg_293[11]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"1D"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      O => \trunc_ln2_reg_293[11]_i_2_n_0\
    );
\trunc_ln2_reg_293[11]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_i_reg(18),
      I1 => acc_i_reg(19),
      O => \trunc_ln2_reg_293[11]_i_3_n_0\
    );
\trunc_ln2_reg_293[11]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_i_reg(17),
      I1 => acc_i_reg(18),
      O => \trunc_ln2_reg_293[11]_i_4_n_0\
    );
\trunc_ln2_reg_293[11]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_i_reg(16),
      I1 => acc_i_reg(17),
      O => \trunc_ln2_reg_293[11]_i_5_n_0\
    );
\trunc_ln2_reg_293[11]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(16),
      O => \trunc_ln2_reg_293[11]_i_6_n_0\
    );
\trunc_ln2_reg_293[15]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_i_reg(23),
      I1 => acc_i_reg(22),
      O => \trunc_ln2_reg_293[15]_i_4_n_0\
    );
\trunc_ln2_reg_293[15]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_i_reg(21),
      I1 => acc_i_reg(22),
      O => \trunc_ln2_reg_293[15]_i_5_n_0\
    );
\trunc_ln2_reg_293[15]_i_6\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_i_reg(20),
      I1 => acc_i_reg(21),
      O => \trunc_ln2_reg_293[15]_i_6_n_0\
    );
\trunc_ln2_reg_293[15]_i_7\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_i_reg(19),
      I1 => acc_i_reg(20),
      O => \trunc_ln2_reg_293[15]_i_7_n_0\
    );
\trunc_ln2_reg_293[3]_i_10\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(5),
      I1 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      O => \trunc_ln2_reg_293[3]_i_10_n_0\
    );
\trunc_ln2_reg_293[3]_i_11\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(4),
      I1 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      O => \trunc_ln2_reg_293[3]_i_11_n_0\
    );
\trunc_ln2_reg_293[3]_i_12\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(3),
      I1 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      O => \trunc_ln2_reg_293[3]_i_12_n_0\
    );
\trunc_ln2_reg_293[3]_i_13\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(2),
      I1 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      O => \trunc_ln2_reg_293[3]_i_13_n_0\
    );
\trunc_ln2_reg_293[3]_i_14\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(1),
      I1 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      O => \trunc_ln2_reg_293[3]_i_14_n_0\
    );
\trunc_ln2_reg_293[3]_i_15\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(0),
      I1 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      O => \trunc_ln2_reg_293[3]_i_15_n_0\
    );
\trunc_ln2_reg_293[3]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(11),
      I1 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      O => \trunc_ln2_reg_293[3]_i_3_n_0\
    );
\trunc_ln2_reg_293[3]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(10),
      I1 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      O => \trunc_ln2_reg_293[3]_i_4_n_0\
    );
\trunc_ln2_reg_293[3]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(9),
      I1 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      O => \trunc_ln2_reg_293[3]_i_5_n_0\
    );
\trunc_ln2_reg_293[3]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(8),
      I1 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      O => \trunc_ln2_reg_293[3]_i_6_n_0\
    );
\trunc_ln2_reg_293[3]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(7),
      I1 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      O => \trunc_ln2_reg_293[3]_i_8_n_0\
    );
\trunc_ln2_reg_293[3]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(6),
      I1 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      O => \trunc_ln2_reg_293[3]_i_9_n_0\
    );
\trunc_ln2_reg_293[7]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \trunc_ln2_reg_293[7]_i_2_n_0\
    );
\trunc_ln2_reg_293[7]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => acc_i_reg(15),
      O => \trunc_ln2_reg_293[7]_i_3_n_0\
    );
\trunc_ln2_reg_293[7]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(14),
      I1 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      O => \trunc_ln2_reg_293[7]_i_4_n_0\
    );
\trunc_ln2_reg_293[7]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(13),
      I1 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      O => \trunc_ln2_reg_293[7]_i_5_n_0\
    );
\trunc_ln2_reg_293[7]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_i_reg(12),
      I1 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      O => \trunc_ln2_reg_293[7]_i_6_n_0\
    );
\trunc_ln2_reg_293_reg[11]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln2_reg_293_reg[7]_i_1_n_0\,
      CO(3) => \trunc_ln2_reg_293_reg[11]_i_1_n_0\,
      CO(2) => \trunc_ln2_reg_293_reg[11]_i_1_n_1\,
      CO(1) => \trunc_ln2_reg_293_reg[11]_i_1_n_2\,
      CO(0) => \trunc_ln2_reg_293_reg[11]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 1) => acc_i_reg(18 downto 16),
      DI(0) => \trunc_ln2_reg_293[11]_i_2_n_0\,
      O(3 downto 0) => D(11 downto 8),
      S(3) => \trunc_ln2_reg_293[11]_i_3_n_0\,
      S(2) => \trunc_ln2_reg_293[11]_i_4_n_0\,
      S(1) => \trunc_ln2_reg_293[11]_i_5_n_0\,
      S(0) => \trunc_ln2_reg_293[11]_i_6_n_0\
    );
\trunc_ln2_reg_293_reg[15]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln2_reg_293_reg[11]_i_1_n_0\,
      CO(3) => \NLW_trunc_ln2_reg_293_reg[15]_i_2_CO_UNCONNECTED\(3),
      CO(2) => \trunc_ln2_reg_293_reg[15]_i_2_n_1\,
      CO(1) => \trunc_ln2_reg_293_reg[15]_i_2_n_2\,
      CO(0) => \trunc_ln2_reg_293_reg[15]_i_2_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2 downto 0) => acc_i_reg(21 downto 19),
      O(3 downto 0) => D(15 downto 12),
      S(3) => \trunc_ln2_reg_293[15]_i_4_n_0\,
      S(2) => \trunc_ln2_reg_293[15]_i_5_n_0\,
      S(1) => \trunc_ln2_reg_293[15]_i_6_n_0\,
      S(0) => \trunc_ln2_reg_293[15]_i_7_n_0\
    );
\trunc_ln2_reg_293_reg[3]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln2_reg_293_reg[3]_i_2_n_0\,
      CO(3) => \trunc_ln2_reg_293_reg[3]_i_1_n_0\,
      CO(2) => \trunc_ln2_reg_293_reg[3]_i_1_n_1\,
      CO(1) => \trunc_ln2_reg_293_reg[3]_i_1_n_2\,
      CO(0) => \trunc_ln2_reg_293_reg[3]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_i_reg(11 downto 8),
      O(3 downto 0) => D(3 downto 0),
      S(3) => \trunc_ln2_reg_293[3]_i_3_n_0\,
      S(2) => \trunc_ln2_reg_293[3]_i_4_n_0\,
      S(1) => \trunc_ln2_reg_293[3]_i_5_n_0\,
      S(0) => \trunc_ln2_reg_293[3]_i_6_n_0\
    );
\trunc_ln2_reg_293_reg[3]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln2_reg_293_reg[3]_i_7_n_0\,
      CO(3) => \trunc_ln2_reg_293_reg[3]_i_2_n_0\,
      CO(2) => \trunc_ln2_reg_293_reg[3]_i_2_n_1\,
      CO(1) => \trunc_ln2_reg_293_reg[3]_i_2_n_2\,
      CO(0) => \trunc_ln2_reg_293_reg[3]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_i_reg(7 downto 4),
      O(3 downto 0) => \NLW_trunc_ln2_reg_293_reg[3]_i_2_O_UNCONNECTED\(3 downto 0),
      S(3) => \trunc_ln2_reg_293[3]_i_8_n_0\,
      S(2) => \trunc_ln2_reg_293[3]_i_9_n_0\,
      S(1) => \trunc_ln2_reg_293[3]_i_10_n_0\,
      S(0) => \trunc_ln2_reg_293[3]_i_11_n_0\
    );
\trunc_ln2_reg_293_reg[3]_i_7\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \trunc_ln2_reg_293_reg[3]_i_7_n_0\,
      CO(2) => \trunc_ln2_reg_293_reg[3]_i_7_n_1\,
      CO(1) => \trunc_ln2_reg_293_reg[3]_i_7_n_2\,
      CO(0) => \trunc_ln2_reg_293_reg[3]_i_7_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_i_reg(3 downto 0),
      O(3 downto 0) => \NLW_trunc_ln2_reg_293_reg[3]_i_7_O_UNCONNECTED\(3 downto 0),
      S(3) => \trunc_ln2_reg_293[3]_i_12_n_0\,
      S(2) => \trunc_ln2_reg_293[3]_i_13_n_0\,
      S(1) => \trunc_ln2_reg_293[3]_i_14_n_0\,
      S(0) => \trunc_ln2_reg_293[3]_i_15_n_0\
    );
\trunc_ln2_reg_293_reg[7]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln2_reg_293_reg[3]_i_1_n_0\,
      CO(3) => \trunc_ln2_reg_293_reg[7]_i_1_n_0\,
      CO(2) => \trunc_ln2_reg_293_reg[7]_i_1_n_1\,
      CO(1) => \trunc_ln2_reg_293_reg[7]_i_1_n_2\,
      CO(0) => \trunc_ln2_reg_293_reg[7]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \trunc_ln2_reg_293[7]_i_2_n_0\,
      DI(2 downto 0) => acc_i_reg(14 downto 12),
      O(3 downto 0) => D(7 downto 4),
      S(3) => \trunc_ln2_reg_293[7]_i_3_n_0\,
      S(2) => \trunc_ln2_reg_293[7]_i_4_n_0\,
      S(1) => \trunc_ln2_reg_293[7]_i_5_n_0\,
      S(0) => \trunc_ln2_reg_293[7]_i_6_n_0\
    );
\trunc_ln3_reg_298[11]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"1D"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      O => \trunc_ln3_reg_298[11]_i_2_n_0\
    );
\trunc_ln3_reg_298[11]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_q_reg(18),
      I1 => acc_q_reg(19),
      O => \trunc_ln3_reg_298[11]_i_3_n_0\
    );
\trunc_ln3_reg_298[11]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_q_reg(17),
      I1 => acc_q_reg(18),
      O => \trunc_ln3_reg_298[11]_i_4_n_0\
    );
\trunc_ln3_reg_298[11]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_q_reg(16),
      I1 => acc_q_reg(17),
      O => \trunc_ln3_reg_298[11]_i_5_n_0\
    );
\trunc_ln3_reg_298[11]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(16),
      O => \trunc_ln3_reg_298[11]_i_6_n_0\
    );
\trunc_ln3_reg_298[15]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_q_reg(23),
      I1 => acc_q_reg(22),
      O => \trunc_ln3_reg_298[15]_i_2_n_0\
    );
\trunc_ln3_reg_298[15]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_q_reg(21),
      I1 => acc_q_reg(22),
      O => \trunc_ln3_reg_298[15]_i_3_n_0\
    );
\trunc_ln3_reg_298[15]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_q_reg(20),
      I1 => acc_q_reg(21),
      O => \trunc_ln3_reg_298[15]_i_4_n_0\
    );
\trunc_ln3_reg_298[15]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => acc_q_reg(19),
      I1 => acc_q_reg(20),
      O => \trunc_ln3_reg_298[15]_i_5_n_0\
    );
\trunc_ln3_reg_298[3]_i_10\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(5),
      I1 => \B_V_data_1_payload_A_reg_n_0_[21]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[21]\,
      O => \trunc_ln3_reg_298[3]_i_10_n_0\
    );
\trunc_ln3_reg_298[3]_i_11\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(4),
      I1 => \B_V_data_1_payload_A_reg_n_0_[20]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[20]\,
      O => \trunc_ln3_reg_298[3]_i_11_n_0\
    );
\trunc_ln3_reg_298[3]_i_12\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(3),
      I1 => \B_V_data_1_payload_A_reg_n_0_[19]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[19]\,
      O => \trunc_ln3_reg_298[3]_i_12_n_0\
    );
\trunc_ln3_reg_298[3]_i_13\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(2),
      I1 => \B_V_data_1_payload_A_reg_n_0_[18]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[18]\,
      O => \trunc_ln3_reg_298[3]_i_13_n_0\
    );
\trunc_ln3_reg_298[3]_i_14\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(1),
      I1 => \B_V_data_1_payload_A_reg_n_0_[17]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[17]\,
      O => \trunc_ln3_reg_298[3]_i_14_n_0\
    );
\trunc_ln3_reg_298[3]_i_15\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(0),
      I1 => \B_V_data_1_payload_A_reg_n_0_[16]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[16]\,
      O => \trunc_ln3_reg_298[3]_i_15_n_0\
    );
\trunc_ln3_reg_298[3]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(11),
      I1 => \B_V_data_1_payload_A_reg_n_0_[27]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[27]\,
      O => \trunc_ln3_reg_298[3]_i_3_n_0\
    );
\trunc_ln3_reg_298[3]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(10),
      I1 => \B_V_data_1_payload_A_reg_n_0_[26]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[26]\,
      O => \trunc_ln3_reg_298[3]_i_4_n_0\
    );
\trunc_ln3_reg_298[3]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(9),
      I1 => \B_V_data_1_payload_A_reg_n_0_[25]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[25]\,
      O => \trunc_ln3_reg_298[3]_i_5_n_0\
    );
\trunc_ln3_reg_298[3]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(8),
      I1 => \B_V_data_1_payload_A_reg_n_0_[24]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[24]\,
      O => \trunc_ln3_reg_298[3]_i_6_n_0\
    );
\trunc_ln3_reg_298[3]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(7),
      I1 => \B_V_data_1_payload_A_reg_n_0_[23]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[23]\,
      O => \trunc_ln3_reg_298[3]_i_8_n_0\
    );
\trunc_ln3_reg_298[3]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(6),
      I1 => \B_V_data_1_payload_A_reg_n_0_[22]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[22]\,
      O => \trunc_ln3_reg_298[3]_i_9_n_0\
    );
\trunc_ln3_reg_298[7]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      O => \trunc_ln3_reg_298[7]_i_2_n_0\
    );
\trunc_ln3_reg_298[7]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I1 => \^b_v_data_1_sel\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => acc_q_reg(15),
      O => \trunc_ln3_reg_298[7]_i_3_n_0\
    );
\trunc_ln3_reg_298[7]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(14),
      I1 => \B_V_data_1_payload_A_reg_n_0_[30]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[30]\,
      O => \trunc_ln3_reg_298[7]_i_4_n_0\
    );
\trunc_ln3_reg_298[7]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(13),
      I1 => \B_V_data_1_payload_A_reg_n_0_[29]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[29]\,
      O => \trunc_ln3_reg_298[7]_i_5_n_0\
    );
\trunc_ln3_reg_298[7]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => acc_q_reg(12),
      I1 => \B_V_data_1_payload_A_reg_n_0_[28]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[28]\,
      O => \trunc_ln3_reg_298[7]_i_6_n_0\
    );
\trunc_ln3_reg_298_reg[11]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln3_reg_298_reg[7]_i_1_n_0\,
      CO(3) => \trunc_ln3_reg_298_reg[11]_i_1_n_0\,
      CO(2) => \trunc_ln3_reg_298_reg[11]_i_1_n_1\,
      CO(1) => \trunc_ln3_reg_298_reg[11]_i_1_n_2\,
      CO(0) => \trunc_ln3_reg_298_reg[11]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 1) => acc_q_reg(18 downto 16),
      DI(0) => \trunc_ln3_reg_298[11]_i_2_n_0\,
      O(3 downto 0) => \acc_q_reg[21]\(11 downto 8),
      S(3) => \trunc_ln3_reg_298[11]_i_3_n_0\,
      S(2) => \trunc_ln3_reg_298[11]_i_4_n_0\,
      S(1) => \trunc_ln3_reg_298[11]_i_5_n_0\,
      S(0) => \trunc_ln3_reg_298[11]_i_6_n_0\
    );
\trunc_ln3_reg_298_reg[15]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln3_reg_298_reg[11]_i_1_n_0\,
      CO(3) => \NLW_trunc_ln3_reg_298_reg[15]_i_1_CO_UNCONNECTED\(3),
      CO(2) => \trunc_ln3_reg_298_reg[15]_i_1_n_1\,
      CO(1) => \trunc_ln3_reg_298_reg[15]_i_1_n_2\,
      CO(0) => \trunc_ln3_reg_298_reg[15]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2 downto 0) => acc_q_reg(21 downto 19),
      O(3 downto 0) => \acc_q_reg[21]\(15 downto 12),
      S(3) => \trunc_ln3_reg_298[15]_i_2_n_0\,
      S(2) => \trunc_ln3_reg_298[15]_i_3_n_0\,
      S(1) => \trunc_ln3_reg_298[15]_i_4_n_0\,
      S(0) => \trunc_ln3_reg_298[15]_i_5_n_0\
    );
\trunc_ln3_reg_298_reg[3]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln3_reg_298_reg[3]_i_2_n_0\,
      CO(3) => \trunc_ln3_reg_298_reg[3]_i_1_n_0\,
      CO(2) => \trunc_ln3_reg_298_reg[3]_i_1_n_1\,
      CO(1) => \trunc_ln3_reg_298_reg[3]_i_1_n_2\,
      CO(0) => \trunc_ln3_reg_298_reg[3]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_q_reg(11 downto 8),
      O(3 downto 0) => \acc_q_reg[21]\(3 downto 0),
      S(3) => \trunc_ln3_reg_298[3]_i_3_n_0\,
      S(2) => \trunc_ln3_reg_298[3]_i_4_n_0\,
      S(1) => \trunc_ln3_reg_298[3]_i_5_n_0\,
      S(0) => \trunc_ln3_reg_298[3]_i_6_n_0\
    );
\trunc_ln3_reg_298_reg[3]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln3_reg_298_reg[3]_i_7_n_0\,
      CO(3) => \trunc_ln3_reg_298_reg[3]_i_2_n_0\,
      CO(2) => \trunc_ln3_reg_298_reg[3]_i_2_n_1\,
      CO(1) => \trunc_ln3_reg_298_reg[3]_i_2_n_2\,
      CO(0) => \trunc_ln3_reg_298_reg[3]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_q_reg(7 downto 4),
      O(3 downto 0) => \NLW_trunc_ln3_reg_298_reg[3]_i_2_O_UNCONNECTED\(3 downto 0),
      S(3) => \trunc_ln3_reg_298[3]_i_8_n_0\,
      S(2) => \trunc_ln3_reg_298[3]_i_9_n_0\,
      S(1) => \trunc_ln3_reg_298[3]_i_10_n_0\,
      S(0) => \trunc_ln3_reg_298[3]_i_11_n_0\
    );
\trunc_ln3_reg_298_reg[3]_i_7\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \trunc_ln3_reg_298_reg[3]_i_7_n_0\,
      CO(2) => \trunc_ln3_reg_298_reg[3]_i_7_n_1\,
      CO(1) => \trunc_ln3_reg_298_reg[3]_i_7_n_2\,
      CO(0) => \trunc_ln3_reg_298_reg[3]_i_7_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => acc_q_reg(3 downto 0),
      O(3 downto 0) => \NLW_trunc_ln3_reg_298_reg[3]_i_7_O_UNCONNECTED\(3 downto 0),
      S(3) => \trunc_ln3_reg_298[3]_i_12_n_0\,
      S(2) => \trunc_ln3_reg_298[3]_i_13_n_0\,
      S(1) => \trunc_ln3_reg_298[3]_i_14_n_0\,
      S(0) => \trunc_ln3_reg_298[3]_i_15_n_0\
    );
\trunc_ln3_reg_298_reg[7]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \trunc_ln3_reg_298_reg[3]_i_1_n_0\,
      CO(3) => \trunc_ln3_reg_298_reg[7]_i_1_n_0\,
      CO(2) => \trunc_ln3_reg_298_reg[7]_i_1_n_1\,
      CO(1) => \trunc_ln3_reg_298_reg[7]_i_1_n_2\,
      CO(0) => \trunc_ln3_reg_298_reg[7]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \trunc_ln3_reg_298[7]_i_2_n_0\,
      DI(2 downto 0) => acc_q_reg(14 downto 12),
      O(3 downto 0) => \acc_q_reg[21]\(7 downto 4),
      S(3) => \trunc_ln3_reg_298[7]_i_3_n_0\,
      S(2) => \trunc_ln3_reg_298[7]_i_4_n_0\,
      S(1) => \trunc_ln3_reg_298[7]_i_5_n_0\,
      S(0) => \trunc_ln3_reg_298[7]_i_6_n_0\
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1\ is
  port (
    \B_V_data_1_state_reg[1]_0\ : out STD_LOGIC;
    ap_rst_n_inv : out STD_LOGIC;
    B_V_data_1_sel_wr : out STD_LOGIC;
    ap_enable_reg_pp0_iter1_reg : out STD_LOGIC;
    \B_V_data_1_state_reg[1]_1\ : out STD_LOGIC;
    dec_out_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    ap_clk : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_0\ : in STD_LOGIC;
    B_V_data_1_sel_wr_reg_0 : in STD_LOGIC;
    \B_V_data_1_state_reg[1]_2\ : in STD_LOGIC;
    rx_in_TVALID_int_regslice : in STD_LOGIC;
    \B_V_data_1_state_reg[1]_3\ : in STD_LOGIC;
    ap_enable_reg_pp0_iter1 : in STD_LOGIC;
    icmp_ln40_reg_289 : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    dec_out_TREADY : in STD_LOGIC;
    pkt_rx_last_V_reg_284 : in STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1\ : entity is "fsk_decimator_regslice_both";
end \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1\;

architecture STRUCTURE of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1\ is
  signal B_V_data_1_payload_A : STD_LOGIC;
  signal \B_V_data_1_payload_A[0]_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_payload_B : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__2_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_sel_wr\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[1]_i_2__0_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[1]_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \^ap_enable_reg_pp0_iter1_reg\ : STD_LOGIC;
  signal \^ap_rst_n_inv\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__2\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \B_V_data_1_state[0]_i_2\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_2__0\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \dec_out_TLAST[0]_INST_0\ : label is "soft_lutpair20";
begin
  B_V_data_1_sel_wr <= \^b_v_data_1_sel_wr\;
  \B_V_data_1_state_reg[1]_0\ <= \^b_v_data_1_state_reg[1]_0\;
  ap_enable_reg_pp0_iter1_reg <= \^ap_enable_reg_pp0_iter1_reg\;
  ap_rst_n_inv <= \^ap_rst_n_inv\;
\B_V_data_1_payload_A[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EFEE2022"
    )
        port map (
      I0 => pkt_rx_last_V_reg_284,
      I1 => \^b_v_data_1_sel_wr\,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => \B_V_data_1_state_reg_n_0_[0]\,
      I4 => B_V_data_1_payload_A,
      O => \B_V_data_1_payload_A[0]_i_1__0_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_A[0]_i_1__0_n_0\,
      Q => B_V_data_1_payload_A,
      R => '0'
    );
\B_V_data_1_payload_B[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BFBB8088"
    )
        port map (
      I0 => pkt_rx_last_V_reg_284,
      I1 => \^b_v_data_1_sel_wr\,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => \B_V_data_1_state_reg_n_0_[0]\,
      I4 => B_V_data_1_payload_B,
      O => \B_V_data_1_payload_B[0]_i_1__0_n_0\
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_B[0]_i_1__0_n_0\,
      Q => B_V_data_1_payload_B,
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => dec_out_TREADY,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__2_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__2_n_0\,
      Q => B_V_data_1_sel,
      R => \^ap_rst_n_inv\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_sel_wr_reg_0,
      Q => \^b_v_data_1_sel_wr\,
      R => \^ap_rst_n_inv\
    );
\B_V_data_1_state[0]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"8F"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[1]_0\,
      I1 => dec_out_TREADY,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      O => \B_V_data_1_state_reg[1]_1\
    );
\B_V_data_1_state[1]_i_1__0\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"DDDDDDDD5DDDDDDD"
    )
        port map (
      I0 => \B_V_data_1_state[1]_i_2__0_n_0\,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => \B_V_data_1_state_reg[1]_2\,
      I3 => rx_in_TVALID_int_regslice,
      I4 => \B_V_data_1_state_reg[1]_3\,
      I5 => \^ap_enable_reg_pp0_iter1_reg\,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state[1]_i_1__2\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => ap_rst_n,
      O => \^ap_rst_n_inv\
    );
\B_V_data_1_state[1]_i_2__0\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => dec_out_TREADY,
      O => \B_V_data_1_state[1]_i_2__0_n_0\
    );
\B_V_data_1_state[1]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter1,
      I1 => icmp_ln40_reg_289,
      O => \^ap_enable_reg_pp0_iter1_reg\
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state_reg[0]_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => \^ap_rst_n_inv\
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \^b_v_data_1_state_reg[1]_0\,
      R => \^ap_rst_n_inv\
    );
\dec_out_TLAST[0]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B,
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A,
      O => dec_out_TLAST(0)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1_1\ is
  port (
    rx_in_TLAST_int_regslice : out STD_LOGIC;
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    B_V_data_1_sel_rd_reg_0 : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_0\ : in STD_LOGIC;
    rx_in_TVALID : in STD_LOGIC;
    rx_in_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    rx_in_TVALID_int_regslice : in STD_LOGIC;
    B_V_data_1_sel_rd_reg_1 : in STD_LOGIC;
    B_V_data_1_sel_rd_reg_2 : in STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1_1\ : entity is "fsk_decimator_regslice_both";
end \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1_1\;

architecture STRUCTURE of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1_1\ is
  signal B_V_data_1_payload_A : STD_LOGIC;
  signal \B_V_data_1_payload_A[0]_i_1_n_0\ : STD_LOGIC;
  signal B_V_data_1_payload_B : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal B_V_data_1_sel_rd_i_1_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__2_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_state[0]_i_1__0\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__1\ : label is "soft_lutpair21";
begin
\B_V_data_1_payload_A[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EFEE2022"
    )
        port map (
      I0 => rx_in_TLAST(0),
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \B_V_data_1_state_reg_n_0_[0]\,
      I4 => B_V_data_1_payload_A,
      O => \B_V_data_1_payload_A[0]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_A[0]_i_1_n_0\,
      Q => B_V_data_1_payload_A,
      R => '0'
    );
\B_V_data_1_payload_B[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BFBB8088"
    )
        port map (
      I0 => rx_in_TLAST(0),
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \B_V_data_1_state_reg_n_0_[0]\,
      I4 => B_V_data_1_payload_B,
      O => \B_V_data_1_payload_B[0]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_B[0]_i_1_n_0\,
      Q => B_V_data_1_payload_B,
      R => '0'
    );
B_V_data_1_sel_rd_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"7F7F7FFF80808000"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => B_V_data_1_sel_rd_reg_0,
      I2 => rx_in_TVALID_int_regslice,
      I3 => B_V_data_1_sel_rd_reg_1,
      I4 => B_V_data_1_sel_rd_reg_2,
      I5 => B_V_data_1_sel,
      O => B_V_data_1_sel_rd_i_1_n_0
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_sel_rd_i_1_n_0,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[1]\,
      I1 => rx_in_TVALID,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__2_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__2_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"F8F8D8F8"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[1]\,
      I1 => rx_in_TVALID,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => B_V_data_1_sel_rd_reg_0,
      I4 => \B_V_data_1_state_reg[0]_0\,
      O => \B_V_data_1_state[0]_i_1__0_n_0\
    );
\B_V_data_1_state[1]_i_1__1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"5DFF5D5D"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => B_V_data_1_sel_rd_reg_0,
      I2 => \B_V_data_1_state_reg[0]_0\,
      I3 => rx_in_TVALID,
      I4 => \B_V_data_1_state_reg_n_0_[1]\,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__0_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => ap_rst_n_inv
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\pkt_rx_last_V_reg_284[0]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B,
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A,
      O => rx_in_TLAST_int_regslice
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator is
  port (
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    rx_in_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    rx_in_TVALID : in STD_LOGIC;
    rx_in_TREADY : out STD_LOGIC;
    rx_in_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    rx_in_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    rx_in_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    dec_out_TDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    dec_out_TVALID : out STD_LOGIC;
    dec_out_TREADY : in STD_LOGIC;
    dec_out_TKEEP : out STD_LOGIC_VECTOR ( 3 downto 0 );
    dec_out_TSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 );
    dec_out_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ap_ST_fsm_pp0_stage0 : string;
  attribute ap_ST_fsm_pp0_stage0 of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator : entity is "1'b1";
  attribute hls_module : string;
  attribute hls_module of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator : entity is "yes";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator is
  signal \<const0>\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal acc_i_reg : STD_LOGIC_VECTOR ( 23 downto 0 );
  signal acc_q_reg : STD_LOGIC_VECTOR ( 23 downto 0 );
  signal add_ln40_fu_216_p2 : STD_LOGIC_VECTOR ( 23 downto 8 );
  signal ap_block_pp0_stage0_11001 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter1 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter2 : STD_LOGIC;
  signal ap_rst_n_inv : STD_LOGIC;
  signal data_in : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal \dec_counter[1]_i_1_n_0\ : STD_LOGIC;
  signal dec_counter_reg : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal icmp_ln40_fu_226_p2 : STD_LOGIC;
  signal icmp_ln40_reg_289 : STD_LOGIC;
  signal icmp_ln40_reg_289_pp0_iter1_reg : STD_LOGIC;
  signal p_0_in : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal pkt_rx_last_V_reg_284 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_0 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_10 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_11 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_12 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_14 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_15 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_16 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_17 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_2 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_4 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_5 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_6 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_7 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_8 : STD_LOGIC;
  signal regslice_both_dec_out_V_data_V_U_n_9 : STD_LOGIC;
  signal regslice_both_dec_out_V_last_V_U_n_0 : STD_LOGIC;
  signal regslice_both_dec_out_V_last_V_U_n_3 : STD_LOGIC;
  signal regslice_both_dec_out_V_last_V_U_n_4 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_10 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_11 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_12 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_13 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_14 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_15 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_16 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_17 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_18 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_19 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_20 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_21 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_22 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_23 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_24 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_25 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_26 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_3 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_4 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_43 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_44 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_45 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_46 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_47 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_48 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_49 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_5 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_50 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_51 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_52 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_53 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_54 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_55 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_56 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_57 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_58 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_59 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_6 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_60 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_61 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_62 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_63 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_64 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_65 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_66 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_7 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_8 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_83 : STD_LOGIC;
  signal regslice_both_rx_in_V_data_V_U_n_9 : STD_LOGIC;
  signal rx_in_TLAST_int_regslice : STD_LOGIC;
  signal rx_in_TVALID_int_regslice : STD_LOGIC;
  signal trunc_ln2_reg_2930 : STD_LOGIC;
begin
  dec_out_TKEEP(3) <= \<const0>\;
  dec_out_TKEEP(2) <= \<const0>\;
  dec_out_TKEEP(1) <= \<const0>\;
  dec_out_TKEEP(0) <= \<const0>\;
  dec_out_TSTRB(3) <= \<const0>\;
  dec_out_TSTRB(2) <= \<const0>\;
  dec_out_TSTRB(1) <= \<const0>\;
  dec_out_TSTRB(0) <= \<const0>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
\acc_i_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_6,
      Q => acc_i_reg(0),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_12,
      Q => acc_i_reg(10),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_11,
      Q => acc_i_reg(11),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_18,
      Q => acc_i_reg(12),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_17,
      Q => acc_i_reg(13),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_16,
      Q => acc_i_reg(14),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_15,
      Q => acc_i_reg(15),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[16]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_22,
      Q => acc_i_reg(16),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[17]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_21,
      Q => acc_i_reg(17),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[18]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_20,
      Q => acc_i_reg(18),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[19]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_19,
      Q => acc_i_reg(19),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_5,
      Q => acc_i_reg(1),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[20]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_26,
      Q => acc_i_reg(20),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[21]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_25,
      Q => acc_i_reg(21),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[22]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_24,
      Q => acc_i_reg(22),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[23]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_23,
      Q => acc_i_reg(23),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_4,
      Q => acc_i_reg(2),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_3,
      Q => acc_i_reg(3),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_10,
      Q => acc_i_reg(4),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_9,
      Q => acc_i_reg(5),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_8,
      Q => acc_i_reg(6),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_7,
      Q => acc_i_reg(7),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_14,
      Q => acc_i_reg(8),
      R => trunc_ln2_reg_2930
    );
\acc_i_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_13,
      Q => acc_i_reg(9),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_46,
      Q => acc_q_reg(0),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_52,
      Q => acc_q_reg(10),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_51,
      Q => acc_q_reg(11),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_58,
      Q => acc_q_reg(12),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_57,
      Q => acc_q_reg(13),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_56,
      Q => acc_q_reg(14),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_55,
      Q => acc_q_reg(15),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[16]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_62,
      Q => acc_q_reg(16),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[17]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_61,
      Q => acc_q_reg(17),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[18]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_60,
      Q => acc_q_reg(18),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[19]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_59,
      Q => acc_q_reg(19),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_45,
      Q => acc_q_reg(1),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[20]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_66,
      Q => acc_q_reg(20),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[21]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_65,
      Q => acc_q_reg(21),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[22]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_64,
      Q => acc_q_reg(22),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[23]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_63,
      Q => acc_q_reg(23),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_44,
      Q => acc_q_reg(2),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_43,
      Q => acc_q_reg(3),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_50,
      Q => acc_q_reg(4),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_49,
      Q => acc_q_reg(5),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_48,
      Q => acc_q_reg(6),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_47,
      Q => acc_q_reg(7),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_54,
      Q => acc_q_reg(8),
      R => trunc_ln2_reg_2930
    );
\acc_q_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_rx_in_V_data_V_U_n_53,
      Q => acc_q_reg(9),
      R => trunc_ln2_reg_2930
    );
ap_enable_reg_pp0_iter1_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_rx_in_V_data_V_U_n_83,
      Q => ap_enable_reg_pp0_iter1,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter2_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_dec_out_V_data_V_U_n_16,
      Q => ap_enable_reg_pp0_iter2,
      R => ap_rst_n_inv
    );
\dec_counter[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => dec_counter_reg(1),
      I1 => dec_counter_reg(0),
      O => \dec_counter[1]_i_1_n_0\
    );
\dec_counter_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_dec_out_V_data_V_U_n_12,
      Q => dec_counter_reg(0),
      R => '0'
    );
\dec_counter_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \dec_counter[1]_i_1_n_0\,
      Q => dec_counter_reg(1),
      R => '0'
    );
\dec_counter_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_dec_out_V_data_V_U_n_10,
      Q => dec_counter_reg(2),
      R => '0'
    );
\dec_counter_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_dec_out_V_data_V_U_n_11,
      Q => dec_counter_reg(3),
      R => '0'
    );
\dec_counter_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_dec_out_V_data_V_U_n_9,
      Q => dec_counter_reg(4),
      R => '0'
    );
\dec_counter_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_dec_out_V_data_V_U_n_8,
      Q => dec_counter_reg(5),
      R => '0'
    );
\dec_counter_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_dec_out_V_data_V_U_n_7,
      Q => dec_counter_reg(6),
      R => '0'
    );
\dec_counter_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_dec_out_V_data_V_U_n_6,
      Q => dec_counter_reg(7),
      R => '0'
    );
\icmp_ln40_reg_289[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"80"
    )
        port map (
      I0 => dec_counter_reg(7),
      I1 => regslice_both_dec_out_V_data_V_U_n_5,
      I2 => dec_counter_reg(6),
      O => icmp_ln40_fu_226_p2
    );
\icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => icmp_ln40_reg_289,
      Q => icmp_ln40_reg_289_pp0_iter1_reg,
      R => '0'
    );
\icmp_ln40_reg_289_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => icmp_ln40_fu_226_p2,
      Q => icmp_ln40_reg_289,
      R => '0'
    );
\pkt_rx_last_V_reg_284_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => rx_in_TLAST_int_regslice,
      Q => pkt_rx_last_V_reg_284,
      R => '0'
    );
regslice_both_dec_out_V_data_V_U: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both
     port map (
      B_V_data_1_sel => B_V_data_1_sel,
      B_V_data_1_sel_wr => B_V_data_1_sel_wr,
      \B_V_data_1_state_reg[0]_0\ => dec_out_TVALID,
      \B_V_data_1_state_reg[0]_1\ => regslice_both_dec_out_V_data_V_U_n_16,
      \B_V_data_1_state_reg[0]_2\ => regslice_both_dec_out_V_last_V_U_n_3,
      \B_V_data_1_state_reg[0]_3\ => regslice_both_dec_out_V_last_V_U_n_0,
      \B_V_data_1_state_reg[0]_4\ => regslice_both_dec_out_V_last_V_U_n_4,
      \B_V_data_1_state_reg[1]_0\ => regslice_both_dec_out_V_data_V_U_n_0,
      \B_V_data_1_state_reg[1]_1\ => regslice_both_dec_out_V_data_V_U_n_14,
      \B_V_data_1_state_reg[1]_2\ => regslice_both_dec_out_V_data_V_U_n_15,
      D(31 downto 0) => data_in(31 downto 0),
      ap_block_pp0_stage0_11001 => ap_block_pp0_stage0_11001,
      ap_clk => ap_clk,
      ap_enable_reg_pp0_iter1 => ap_enable_reg_pp0_iter1,
      ap_enable_reg_pp0_iter1_reg => regslice_both_dec_out_V_data_V_U_n_4,
      ap_enable_reg_pp0_iter1_reg_0 => regslice_both_dec_out_V_data_V_U_n_17,
      ap_enable_reg_pp0_iter2 => ap_enable_reg_pp0_iter2,
      ap_rst_n_inv => ap_rst_n_inv,
      dec_counter_reg(7 downto 0) => dec_counter_reg(7 downto 0),
      \dec_counter_reg[5]_0\ => regslice_both_dec_out_V_data_V_U_n_8,
      dec_counter_reg_0_sp_1 => regslice_both_dec_out_V_data_V_U_n_12,
      dec_counter_reg_2_sp_1 => regslice_both_dec_out_V_data_V_U_n_10,
      dec_counter_reg_3_sp_1 => regslice_both_dec_out_V_data_V_U_n_11,
      dec_counter_reg_4_sp_1 => regslice_both_dec_out_V_data_V_U_n_9,
      dec_counter_reg_5_sp_1 => regslice_both_dec_out_V_data_V_U_n_5,
      dec_counter_reg_6_sp_1 => regslice_both_dec_out_V_data_V_U_n_7,
      dec_counter_reg_7_sp_1 => regslice_both_dec_out_V_data_V_U_n_6,
      dec_out_TDATA(31 downto 0) => dec_out_TDATA(31 downto 0),
      dec_out_TREADY => dec_out_TREADY,
      icmp_ln40_reg_289 => icmp_ln40_reg_289,
      icmp_ln40_reg_289_pp0_iter1_reg => icmp_ln40_reg_289_pp0_iter1_reg,
      \icmp_ln40_reg_289_pp0_iter1_reg_reg[0]\ => regslice_both_dec_out_V_data_V_U_n_2,
      rx_in_TVALID_int_regslice => rx_in_TVALID_int_regslice,
      trunc_ln2_reg_2930 => trunc_ln2_reg_2930
    );
regslice_both_dec_out_V_last_V_U: entity work.\decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1\
     port map (
      B_V_data_1_sel_wr => B_V_data_1_sel_wr,
      B_V_data_1_sel_wr_reg_0 => regslice_both_dec_out_V_data_V_U_n_15,
      \B_V_data_1_state_reg[0]_0\ => regslice_both_dec_out_V_data_V_U_n_14,
      \B_V_data_1_state_reg[1]_0\ => regslice_both_dec_out_V_last_V_U_n_0,
      \B_V_data_1_state_reg[1]_1\ => regslice_both_dec_out_V_last_V_U_n_4,
      \B_V_data_1_state_reg[1]_2\ => regslice_both_dec_out_V_data_V_U_n_0,
      \B_V_data_1_state_reg[1]_3\ => regslice_both_dec_out_V_data_V_U_n_2,
      ap_clk => ap_clk,
      ap_enable_reg_pp0_iter1 => ap_enable_reg_pp0_iter1,
      ap_enable_reg_pp0_iter1_reg => regslice_both_dec_out_V_last_V_U_n_3,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      dec_out_TLAST(0) => dec_out_TLAST(0),
      dec_out_TREADY => dec_out_TREADY,
      icmp_ln40_reg_289 => icmp_ln40_reg_289,
      pkt_rx_last_V_reg_284 => pkt_rx_last_V_reg_284,
      rx_in_TVALID_int_regslice => rx_in_TVALID_int_regslice
    );
regslice_both_rx_in_V_data_V_U: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both_0
     port map (
      \B_V_data_1_payload_B_reg[15]_0\(3) => regslice_both_rx_in_V_data_V_U_n_23,
      \B_V_data_1_payload_B_reg[15]_0\(2) => regslice_both_rx_in_V_data_V_U_n_24,
      \B_V_data_1_payload_B_reg[15]_0\(1) => regslice_both_rx_in_V_data_V_U_n_25,
      \B_V_data_1_payload_B_reg[15]_0\(0) => regslice_both_rx_in_V_data_V_U_n_26,
      \B_V_data_1_payload_B_reg[31]_0\(3) => regslice_both_rx_in_V_data_V_U_n_63,
      \B_V_data_1_payload_B_reg[31]_0\(2) => regslice_both_rx_in_V_data_V_U_n_64,
      \B_V_data_1_payload_B_reg[31]_0\(1) => regslice_both_rx_in_V_data_V_U_n_65,
      \B_V_data_1_payload_B_reg[31]_0\(0) => regslice_both_rx_in_V_data_V_U_n_66,
      B_V_data_1_sel => B_V_data_1_sel,
      B_V_data_1_sel_rd_reg_0 => regslice_both_dec_out_V_data_V_U_n_17,
      \B_V_data_1_state_reg[0]_0\ => regslice_both_rx_in_V_data_V_U_n_83,
      \B_V_data_1_state_reg[0]_1\ => regslice_both_dec_out_V_data_V_U_n_2,
      \B_V_data_1_state_reg[0]_2\ => regslice_both_dec_out_V_data_V_U_n_0,
      \B_V_data_1_state_reg[0]_3\ => regslice_both_dec_out_V_last_V_U_n_3,
      \B_V_data_1_state_reg[1]_0\ => rx_in_TREADY,
      D(15 downto 0) => p_0_in(15 downto 0),
      O(3) => regslice_both_rx_in_V_data_V_U_n_3,
      O(2) => regslice_both_rx_in_V_data_V_U_n_4,
      O(1) => regslice_both_rx_in_V_data_V_U_n_5,
      O(0) => regslice_both_rx_in_V_data_V_U_n_6,
      acc_i_reg(23 downto 0) => acc_i_reg(23 downto 0),
      \acc_i_reg[11]\(3) => regslice_both_rx_in_V_data_V_U_n_11,
      \acc_i_reg[11]\(2) => regslice_both_rx_in_V_data_V_U_n_12,
      \acc_i_reg[11]\(1) => regslice_both_rx_in_V_data_V_U_n_13,
      \acc_i_reg[11]\(0) => regslice_both_rx_in_V_data_V_U_n_14,
      \acc_i_reg[14]\(3) => regslice_both_rx_in_V_data_V_U_n_15,
      \acc_i_reg[14]\(2) => regslice_both_rx_in_V_data_V_U_n_16,
      \acc_i_reg[14]\(1) => regslice_both_rx_in_V_data_V_U_n_17,
      \acc_i_reg[14]\(0) => regslice_both_rx_in_V_data_V_U_n_18,
      \acc_i_reg[14]_0\(3) => regslice_both_rx_in_V_data_V_U_n_19,
      \acc_i_reg[14]_0\(2) => regslice_both_rx_in_V_data_V_U_n_20,
      \acc_i_reg[14]_0\(1) => regslice_both_rx_in_V_data_V_U_n_21,
      \acc_i_reg[14]_0\(0) => regslice_both_rx_in_V_data_V_U_n_22,
      \acc_i_reg[7]\(3) => regslice_both_rx_in_V_data_V_U_n_7,
      \acc_i_reg[7]\(2) => regslice_both_rx_in_V_data_V_U_n_8,
      \acc_i_reg[7]\(1) => regslice_both_rx_in_V_data_V_U_n_9,
      \acc_i_reg[7]\(0) => regslice_both_rx_in_V_data_V_U_n_10,
      acc_q_reg(23 downto 0) => acc_q_reg(23 downto 0),
      \acc_q_reg[11]\(3) => regslice_both_rx_in_V_data_V_U_n_51,
      \acc_q_reg[11]\(2) => regslice_both_rx_in_V_data_V_U_n_52,
      \acc_q_reg[11]\(1) => regslice_both_rx_in_V_data_V_U_n_53,
      \acc_q_reg[11]\(0) => regslice_both_rx_in_V_data_V_U_n_54,
      \acc_q_reg[14]\(3) => regslice_both_rx_in_V_data_V_U_n_55,
      \acc_q_reg[14]\(2) => regslice_both_rx_in_V_data_V_U_n_56,
      \acc_q_reg[14]\(1) => regslice_both_rx_in_V_data_V_U_n_57,
      \acc_q_reg[14]\(0) => regslice_both_rx_in_V_data_V_U_n_58,
      \acc_q_reg[14]_0\(3) => regslice_both_rx_in_V_data_V_U_n_59,
      \acc_q_reg[14]_0\(2) => regslice_both_rx_in_V_data_V_U_n_60,
      \acc_q_reg[14]_0\(1) => regslice_both_rx_in_V_data_V_U_n_61,
      \acc_q_reg[14]_0\(0) => regslice_both_rx_in_V_data_V_U_n_62,
      \acc_q_reg[21]\(15 downto 0) => add_ln40_fu_216_p2(23 downto 8),
      \acc_q_reg[3]\(3) => regslice_both_rx_in_V_data_V_U_n_43,
      \acc_q_reg[3]\(2) => regslice_both_rx_in_V_data_V_U_n_44,
      \acc_q_reg[3]\(1) => regslice_both_rx_in_V_data_V_U_n_45,
      \acc_q_reg[3]\(0) => regslice_both_rx_in_V_data_V_U_n_46,
      \acc_q_reg[7]\(3) => regslice_both_rx_in_V_data_V_U_n_47,
      \acc_q_reg[7]\(2) => regslice_both_rx_in_V_data_V_U_n_48,
      \acc_q_reg[7]\(1) => regslice_both_rx_in_V_data_V_U_n_49,
      \acc_q_reg[7]\(0) => regslice_both_rx_in_V_data_V_U_n_50,
      ap_clk => ap_clk,
      ap_enable_reg_pp0_iter1 => ap_enable_reg_pp0_iter1,
      ap_rst_n_inv => ap_rst_n_inv,
      rx_in_TDATA(31 downto 0) => rx_in_TDATA(31 downto 0),
      rx_in_TVALID => rx_in_TVALID,
      rx_in_TVALID_int_regslice => rx_in_TVALID_int_regslice
    );
regslice_both_rx_in_V_last_V_U: entity work.\decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator_regslice_both__parameterized1_1\
     port map (
      B_V_data_1_sel_rd_reg_0 => regslice_both_dec_out_V_data_V_U_n_2,
      B_V_data_1_sel_rd_reg_1 => regslice_both_dec_out_V_data_V_U_n_0,
      B_V_data_1_sel_rd_reg_2 => regslice_both_dec_out_V_last_V_U_n_3,
      \B_V_data_1_state_reg[0]_0\ => regslice_both_dec_out_V_data_V_U_n_4,
      ap_clk => ap_clk,
      ap_rst_n_inv => ap_rst_n_inv,
      rx_in_TLAST(0) => rx_in_TLAST(0),
      rx_in_TLAST_int_regslice => rx_in_TLAST_int_regslice,
      rx_in_TVALID => rx_in_TVALID,
      rx_in_TVALID_int_regslice => rx_in_TVALID_int_regslice
    );
\trunc_ln2_reg_293_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(0),
      Q => data_in(0),
      R => '0'
    );
\trunc_ln2_reg_293_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(10),
      Q => data_in(10),
      R => '0'
    );
\trunc_ln2_reg_293_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(11),
      Q => data_in(11),
      R => '0'
    );
\trunc_ln2_reg_293_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(12),
      Q => data_in(12),
      R => '0'
    );
\trunc_ln2_reg_293_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(13),
      Q => data_in(13),
      R => '0'
    );
\trunc_ln2_reg_293_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(14),
      Q => data_in(14),
      R => '0'
    );
\trunc_ln2_reg_293_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(15),
      Q => data_in(15),
      R => '0'
    );
\trunc_ln2_reg_293_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(1),
      Q => data_in(1),
      R => '0'
    );
\trunc_ln2_reg_293_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(2),
      Q => data_in(2),
      R => '0'
    );
\trunc_ln2_reg_293_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(3),
      Q => data_in(3),
      R => '0'
    );
\trunc_ln2_reg_293_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(4),
      Q => data_in(4),
      R => '0'
    );
\trunc_ln2_reg_293_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(5),
      Q => data_in(5),
      R => '0'
    );
\trunc_ln2_reg_293_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(6),
      Q => data_in(6),
      R => '0'
    );
\trunc_ln2_reg_293_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(7),
      Q => data_in(7),
      R => '0'
    );
\trunc_ln2_reg_293_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(8),
      Q => data_in(8),
      R => '0'
    );
\trunc_ln2_reg_293_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => p_0_in(9),
      Q => data_in(9),
      R => '0'
    );
\trunc_ln3_reg_298_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(8),
      Q => data_in(16),
      R => '0'
    );
\trunc_ln3_reg_298_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(18),
      Q => data_in(26),
      R => '0'
    );
\trunc_ln3_reg_298_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(19),
      Q => data_in(27),
      R => '0'
    );
\trunc_ln3_reg_298_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(20),
      Q => data_in(28),
      R => '0'
    );
\trunc_ln3_reg_298_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(21),
      Q => data_in(29),
      R => '0'
    );
\trunc_ln3_reg_298_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(22),
      Q => data_in(30),
      R => '0'
    );
\trunc_ln3_reg_298_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(23),
      Q => data_in(31),
      R => '0'
    );
\trunc_ln3_reg_298_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(9),
      Q => data_in(17),
      R => '0'
    );
\trunc_ln3_reg_298_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(10),
      Q => data_in(18),
      R => '0'
    );
\trunc_ln3_reg_298_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(11),
      Q => data_in(19),
      R => '0'
    );
\trunc_ln3_reg_298_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(12),
      Q => data_in(20),
      R => '0'
    );
\trunc_ln3_reg_298_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(13),
      Q => data_in(21),
      R => '0'
    );
\trunc_ln3_reg_298_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(14),
      Q => data_in(22),
      R => '0'
    );
\trunc_ln3_reg_298_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(15),
      Q => data_in(23),
      R => '0'
    );
\trunc_ln3_reg_298_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(16),
      Q => data_in(24),
      R => '0'
    );
\trunc_ln3_reg_298_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => trunc_ln2_reg_2930,
      D => add_ln40_fu_216_p2(17),
      Q => data_in(25),
      R => '0'
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  port (
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    rx_in_TVALID : in STD_LOGIC;
    rx_in_TREADY : out STD_LOGIC;
    rx_in_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    rx_in_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    rx_in_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    rx_in_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    dec_out_TVALID : out STD_LOGIC;
    dec_out_TREADY : in STD_LOGIC;
    dec_out_TDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    dec_out_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    dec_out_TKEEP : out STD_LOGIC_VECTOR ( 3 downto 0 );
    dec_out_TSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 )
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "system_fsk_decimator_0_0,fsk_decimator,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "HLS";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "fsk_decimator,Vivado 2023.1";
  attribute hls_module : string;
  attribute hls_module of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  signal \<const0>\ : STD_LOGIC;
  signal \<const1>\ : STD_LOGIC;
  signal NLW_inst_dec_out_TKEEP_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_inst_dec_out_TSTRB_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  attribute SDX_KERNEL : string;
  attribute SDX_KERNEL of inst : label is "true";
  attribute SDX_KERNEL_SYNTH_INST : string;
  attribute SDX_KERNEL_SYNTH_INST of inst : label is "inst";
  attribute SDX_KERNEL_TYPE : string;
  attribute SDX_KERNEL_TYPE of inst : label is "hls";
  attribute ap_ST_fsm_pp0_stage0 : string;
  attribute ap_ST_fsm_pp0_stage0 of inst : label is "1'b1";
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of ap_clk : signal is "xilinx.com:signal:clock:1.0 ap_clk CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of ap_clk : signal is "XIL_INTERFACENAME ap_clk, ASSOCIATED_BUSIF rx_in:dec_out, ASSOCIATED_RESET ap_rst_n, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of ap_rst_n : signal is "xilinx.com:signal:reset:1.0 ap_rst_n RST";
  attribute X_INTERFACE_PARAMETER of ap_rst_n : signal is "XIL_INTERFACENAME ap_rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of dec_out_TREADY : signal is "xilinx.com:interface:axis:1.0 dec_out TREADY";
  attribute X_INTERFACE_INFO of dec_out_TVALID : signal is "xilinx.com:interface:axis:1.0 dec_out TVALID";
  attribute X_INTERFACE_INFO of rx_in_TREADY : signal is "xilinx.com:interface:axis:1.0 rx_in TREADY";
  attribute X_INTERFACE_INFO of rx_in_TVALID : signal is "xilinx.com:interface:axis:1.0 rx_in TVALID";
  attribute X_INTERFACE_INFO of dec_out_TDATA : signal is "xilinx.com:interface:axis:1.0 dec_out TDATA";
  attribute X_INTERFACE_INFO of dec_out_TKEEP : signal is "xilinx.com:interface:axis:1.0 dec_out TKEEP";
  attribute X_INTERFACE_INFO of dec_out_TLAST : signal is "xilinx.com:interface:axis:1.0 dec_out TLAST";
  attribute X_INTERFACE_INFO of dec_out_TSTRB : signal is "xilinx.com:interface:axis:1.0 dec_out TSTRB";
  attribute X_INTERFACE_PARAMETER of dec_out_TSTRB : signal is "XIL_INTERFACENAME dec_out, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of rx_in_TDATA : signal is "xilinx.com:interface:axis:1.0 rx_in TDATA";
  attribute X_INTERFACE_INFO of rx_in_TKEEP : signal is "xilinx.com:interface:axis:1.0 rx_in TKEEP";
  attribute X_INTERFACE_INFO of rx_in_TLAST : signal is "xilinx.com:interface:axis:1.0 rx_in TLAST";
  attribute X_INTERFACE_INFO of rx_in_TSTRB : signal is "xilinx.com:interface:axis:1.0 rx_in TSTRB";
  attribute X_INTERFACE_PARAMETER of rx_in_TSTRB : signal is "XIL_INTERFACENAME rx_in, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
begin
  dec_out_TKEEP(3) <= \<const1>\;
  dec_out_TKEEP(2) <= \<const1>\;
  dec_out_TKEEP(1) <= \<const1>\;
  dec_out_TKEEP(0) <= \<const1>\;
  dec_out_TSTRB(3) <= \<const0>\;
  dec_out_TSTRB(2) <= \<const0>\;
  dec_out_TSTRB(1) <= \<const0>\;
  dec_out_TSTRB(0) <= \<const0>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
VCC: unisim.vcomponents.VCC
     port map (
      P => \<const1>\
    );
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_decimator
     port map (
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      dec_out_TDATA(31 downto 0) => dec_out_TDATA(31 downto 0),
      dec_out_TKEEP(3 downto 0) => NLW_inst_dec_out_TKEEP_UNCONNECTED(3 downto 0),
      dec_out_TLAST(0) => dec_out_TLAST(0),
      dec_out_TREADY => dec_out_TREADY,
      dec_out_TSTRB(3 downto 0) => NLW_inst_dec_out_TSTRB_UNCONNECTED(3 downto 0),
      dec_out_TVALID => dec_out_TVALID,
      rx_in_TDATA(31 downto 0) => rx_in_TDATA(31 downto 0),
      rx_in_TKEEP(3 downto 0) => B"0000",
      rx_in_TLAST(0) => rx_in_TLAST(0),
      rx_in_TREADY => rx_in_TREADY,
      rx_in_TSTRB(3 downto 0) => B"0000",
      rx_in_TVALID => rx_in_TVALID
    );
end STRUCTURE;
