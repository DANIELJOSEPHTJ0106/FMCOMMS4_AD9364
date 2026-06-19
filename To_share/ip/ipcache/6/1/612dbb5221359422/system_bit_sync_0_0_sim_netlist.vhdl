-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Sat Jan  3 12:24:47 2026
-- Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim
--               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_bit_sync_0_0/system_bit_sync_0_0_sim_netlist.vhdl
-- Design      : system_bit_sync_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_bit_sync_0_0_bit_sync_regslice_both is
  port (
    \B_V_data_1_state_reg[1]_0\ : out STD_LOGIC;
    in_stream_TVALID_int_regslice : out STD_LOGIC;
    B_V_data_1_sel : out STD_LOGIC;
    curr_sign_fu_104_p2 : out STD_LOGIC_VECTOR ( 0 to 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    B_V_data_1_sel_rd_reg_0 : in STD_LOGIC;
    B_V_data_1_sel0 : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 15 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of system_bit_sync_0_0_bit_sync_regslice_both : entity is "bit_sync_regslice_both";
end system_bit_sync_0_0_bit_sync_regslice_both;

architecture STRUCTURE of system_bit_sync_0_0_bit_sync_regslice_both is
  signal B_V_data_1_load_A : STD_LOGIC;
  signal B_V_data_1_load_B : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[2]\ : STD_LOGIC;
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
  signal \B_V_data_1_payload_B_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[2]\ : STD_LOGIC;
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
  signal \B_V_data_1_state[0]_i_1__2_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[1]_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_10_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_11_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_12_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_13_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_14_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_15_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_16_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_17_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_18_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_19_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_20_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_5_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_6_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_7_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_8_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177[0]_i_9_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177_reg[0]_i_2_n_1\ : STD_LOGIC;
  signal \curr_sign_reg_177_reg[0]_i_2_n_2\ : STD_LOGIC;
  signal \curr_sign_reg_177_reg[0]_i_2_n_3\ : STD_LOGIC;
  signal \curr_sign_reg_177_reg[0]_i_4_n_0\ : STD_LOGIC;
  signal \curr_sign_reg_177_reg[0]_i_4_n_1\ : STD_LOGIC;
  signal \curr_sign_reg_177_reg[0]_i_4_n_2\ : STD_LOGIC;
  signal \curr_sign_reg_177_reg[0]_i_4_n_3\ : STD_LOGIC;
  signal \^in_stream_tvalid_int_regslice\ : STD_LOGIC;
  signal \NLW_curr_sign_reg_177_reg[0]_i_2_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_curr_sign_reg_177_reg[0]_i_4_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_state[0]_i_1__2\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_2\ : label is "soft_lutpair0";
  attribute COMPARATOR_THRESHOLD : integer;
  attribute COMPARATOR_THRESHOLD of \curr_sign_reg_177_reg[0]_i_2\ : label is 11;
  attribute COMPARATOR_THRESHOLD of \curr_sign_reg_177_reg[0]_i_4\ : label is 11;
begin
  B_V_data_1_sel <= \^b_v_data_1_sel\;
  \B_V_data_1_state_reg[1]_0\ <= \^b_v_data_1_state_reg[1]_0\;
  in_stream_TVALID_int_regslice <= \^in_stream_tvalid_int_regslice\;
\B_V_data_1_payload_A[15]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"45"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => \^in_stream_tvalid_int_regslice\,
      O => B_V_data_1_load_A
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(0),
      Q => \B_V_data_1_payload_A_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(10),
      Q => \B_V_data_1_payload_A_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(11),
      Q => \B_V_data_1_payload_A_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(12),
      Q => \B_V_data_1_payload_A_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(13),
      Q => \B_V_data_1_payload_A_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(14),
      Q => \B_V_data_1_payload_A_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(15),
      Q => \B_V_data_1_payload_A_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(1),
      Q => \B_V_data_1_payload_A_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(2),
      Q => \B_V_data_1_payload_A_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(3),
      Q => \B_V_data_1_payload_A_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(4),
      Q => \B_V_data_1_payload_A_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(5),
      Q => \B_V_data_1_payload_A_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(6),
      Q => \B_V_data_1_payload_A_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(7),
      Q => \B_V_data_1_payload_A_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(8),
      Q => \B_V_data_1_payload_A_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(9),
      Q => \B_V_data_1_payload_A_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_payload_B[15]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"8A"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => \^in_stream_tvalid_int_regslice\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(0),
      Q => \B_V_data_1_payload_B_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(10),
      Q => \B_V_data_1_payload_B_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(11),
      Q => \B_V_data_1_payload_B_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(12),
      Q => \B_V_data_1_payload_B_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(13),
      Q => \B_V_data_1_payload_B_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(14),
      Q => \B_V_data_1_payload_B_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(15),
      Q => \B_V_data_1_payload_B_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(1),
      Q => \B_V_data_1_payload_B_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(2),
      Q => \B_V_data_1_payload_B_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(3),
      Q => \B_V_data_1_payload_B_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(4),
      Q => \B_V_data_1_payload_B_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(5),
      Q => \B_V_data_1_payload_B_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(6),
      Q => \B_V_data_1_payload_B_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(7),
      Q => \B_V_data_1_payload_B_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(8),
      Q => \B_V_data_1_payload_B_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(9),
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
      I0 => \^b_v_data_1_state_reg[1]_0\,
      I1 => in_stream_TVALID,
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
\B_V_data_1_state[0]_i_1__2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"D8F8"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[1]_0\,
      I1 => in_stream_TVALID,
      I2 => \^in_stream_tvalid_int_regslice\,
      I3 => B_V_data_1_sel0,
      O => \B_V_data_1_state[0]_i_1__2_n_0\
    );
\B_V_data_1_state[1]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"DFDD"
    )
        port map (
      I0 => \^in_stream_tvalid_int_regslice\,
      I1 => B_V_data_1_sel0,
      I2 => in_stream_TVALID,
      I3 => \^b_v_data_1_state_reg[1]_0\,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__2_n_0\,
      Q => \^in_stream_tvalid_int_regslice\,
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
\curr_sign_reg_177[0]_i_10\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00053035"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      O => \curr_sign_reg_177[0]_i_10_n_0\
    );
\curr_sign_reg_177[0]_i_11\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00053035"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      O => \curr_sign_reg_177[0]_i_11_n_0\
    );
\curr_sign_reg_177[0]_i_12\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00053035"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      O => \curr_sign_reg_177[0]_i_12_n_0\
    );
\curr_sign_reg_177[0]_i_13\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFCAFAC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      O => \curr_sign_reg_177[0]_i_13_n_0\
    );
\curr_sign_reg_177[0]_i_14\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFCAFAC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      O => \curr_sign_reg_177[0]_i_14_n_0\
    );
\curr_sign_reg_177[0]_i_15\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFCAFAC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      O => \curr_sign_reg_177[0]_i_15_n_0\
    );
\curr_sign_reg_177[0]_i_16\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFCAFAC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      O => \curr_sign_reg_177[0]_i_16_n_0\
    );
\curr_sign_reg_177[0]_i_17\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00053035"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      O => \curr_sign_reg_177[0]_i_17_n_0\
    );
\curr_sign_reg_177[0]_i_18\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00053035"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      O => \curr_sign_reg_177[0]_i_18_n_0\
    );
\curr_sign_reg_177[0]_i_19\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00053035"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      O => \curr_sign_reg_177[0]_i_19_n_0\
    );
\curr_sign_reg_177[0]_i_20\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00053035"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      O => \curr_sign_reg_177[0]_i_20_n_0\
    );
\curr_sign_reg_177[0]_i_5\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"000AC0CA"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      O => \curr_sign_reg_177[0]_i_5_n_0\
    );
\curr_sign_reg_177[0]_i_6\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFCAFAC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      O => \curr_sign_reg_177[0]_i_6_n_0\
    );
\curr_sign_reg_177[0]_i_7\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFCAFAC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      O => \curr_sign_reg_177[0]_i_7_n_0\
    );
\curr_sign_reg_177[0]_i_8\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFCAFAC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      O => \curr_sign_reg_177[0]_i_8_n_0\
    );
\curr_sign_reg_177[0]_i_9\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00053035"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I1 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I2 => \^b_v_data_1_sel\,
      I3 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      O => \curr_sign_reg_177[0]_i_9_n_0\
    );
\curr_sign_reg_177_reg[0]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \curr_sign_reg_177_reg[0]_i_4_n_0\,
      CO(3) => curr_sign_fu_104_p2(0),
      CO(2) => \curr_sign_reg_177_reg[0]_i_2_n_1\,
      CO(1) => \curr_sign_reg_177_reg[0]_i_2_n_2\,
      CO(0) => \curr_sign_reg_177_reg[0]_i_2_n_3\,
      CYINIT => '0',
      DI(3) => \curr_sign_reg_177[0]_i_5_n_0\,
      DI(2) => \curr_sign_reg_177[0]_i_6_n_0\,
      DI(1) => \curr_sign_reg_177[0]_i_7_n_0\,
      DI(0) => \curr_sign_reg_177[0]_i_8_n_0\,
      O(3 downto 0) => \NLW_curr_sign_reg_177_reg[0]_i_2_O_UNCONNECTED\(3 downto 0),
      S(3) => \curr_sign_reg_177[0]_i_9_n_0\,
      S(2) => \curr_sign_reg_177[0]_i_10_n_0\,
      S(1) => \curr_sign_reg_177[0]_i_11_n_0\,
      S(0) => \curr_sign_reg_177[0]_i_12_n_0\
    );
\curr_sign_reg_177_reg[0]_i_4\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \curr_sign_reg_177_reg[0]_i_4_n_0\,
      CO(2) => \curr_sign_reg_177_reg[0]_i_4_n_1\,
      CO(1) => \curr_sign_reg_177_reg[0]_i_4_n_2\,
      CO(0) => \curr_sign_reg_177_reg[0]_i_4_n_3\,
      CYINIT => '0',
      DI(3) => \curr_sign_reg_177[0]_i_13_n_0\,
      DI(2) => \curr_sign_reg_177[0]_i_14_n_0\,
      DI(1) => \curr_sign_reg_177[0]_i_15_n_0\,
      DI(0) => \curr_sign_reg_177[0]_i_16_n_0\,
      O(3 downto 0) => \NLW_curr_sign_reg_177_reg[0]_i_4_O_UNCONNECTED\(3 downto 0),
      S(3) => \curr_sign_reg_177[0]_i_17_n_0\,
      S(2) => \curr_sign_reg_177[0]_i_18_n_0\,
      S(1) => \curr_sign_reg_177[0]_i_19_n_0\,
      S(0) => \curr_sign_reg_177[0]_i_20_n_0\
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \system_bit_sync_0_0_bit_sync_regslice_both__parameterized1\ is
  port (
    in_stream_TLAST_int_regslice : out STD_LOGIC;
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    B_V_data_1_sel0 : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \system_bit_sync_0_0_bit_sync_regslice_both__parameterized1\ : entity is "bit_sync_regslice_both";
end \system_bit_sync_0_0_bit_sync_regslice_both__parameterized1\;

architecture STRUCTURE of \system_bit_sync_0_0_bit_sync_regslice_both__parameterized1\ is
  signal B_V_data_1_payload_A : STD_LOGIC;
  signal \B_V_data_1_payload_A[0]_i_1_n_0\ : STD_LOGIC;
  signal B_V_data_1_payload_B : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal B_V_data_1_sel_rd_i_1_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__2_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of B_V_data_1_sel_rd_i_1 : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \B_V_data_1_state[0]_i_1__1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \pkt_last_V_reg_172[0]_i_1\ : label is "soft_lutpair2";
begin
\B_V_data_1_payload_A[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EFEE2022"
    )
        port map (
      I0 => in_stream_TLAST(0),
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
      I0 => in_stream_TLAST(0),
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
B_V_data_1_sel_rd_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => B_V_data_1_sel0,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
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
      I1 => in_stream_TVALID,
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
\B_V_data_1_state[0]_i_1__1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"D8F8"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[1]\,
      I1 => in_stream_TVALID,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => B_V_data_1_sel0,
      O => \B_V_data_1_state[0]_i_1__1_n_0\
    );
\B_V_data_1_state[1]_i_1__1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"DFDD"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => B_V_data_1_sel0,
      I2 => in_stream_TVALID,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__1_n_0\,
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
\pkt_last_V_reg_172[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B,
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A,
      O => in_stream_TLAST_int_regslice
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \system_bit_sync_0_0_bit_sync_regslice_both__parameterized1_0\ is
  port (
    ap_rst_n_inv : out STD_LOGIC;
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    ap_clk : in STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    B_V_data_1_sel0 : in STD_LOGIC;
    ap_enable_reg_pp0_iter2 : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_0\ : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    pkt_last_V_reg_172_pp0_iter1_reg : in STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \system_bit_sync_0_0_bit_sync_regslice_both__parameterized1_0\ : entity is "bit_sync_regslice_both";
end \system_bit_sync_0_0_bit_sync_regslice_both__parameterized1_0\;

architecture STRUCTURE of \system_bit_sync_0_0_bit_sync_regslice_both__parameterized1_0\ is
  signal B_V_data_1_payload_A : STD_LOGIC;
  signal \B_V_data_1_payload_A[0]_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_payload_B : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__2_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal B_V_data_1_sel_wr_i_1_n_0 : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  signal \^ap_rst_n_inv\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__2\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \out_stream_TLAST[0]_INST_0\ : label is "soft_lutpair8";
begin
  ap_rst_n_inv <= \^ap_rst_n_inv\;
\B_V_data_1_payload_A[0]_i_1__1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EFEE2022"
    )
        port map (
      I0 => pkt_last_V_reg_172_pp0_iter1_reg,
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \B_V_data_1_state_reg_n_0_[0]\,
      I4 => B_V_data_1_payload_A,
      O => \B_V_data_1_payload_A[0]_i_1__1_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_A[0]_i_1__1_n_0\,
      Q => B_V_data_1_payload_A,
      R => '0'
    );
\B_V_data_1_payload_B[0]_i_1__1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BFBB8088"
    )
        port map (
      I0 => pkt_last_V_reg_172_pp0_iter1_reg,
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \B_V_data_1_state_reg_n_0_[0]\,
      I4 => B_V_data_1_payload_B,
      O => \B_V_data_1_payload_B[0]_i_1__1_n_0\
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_B[0]_i_1__1_n_0\,
      Q => B_V_data_1_payload_B,
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => out_stream_TREADY,
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
B_V_data_1_sel_wr_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFF8000"
    )
        port map (
      I0 => B_V_data_1_sel0,
      I1 => ap_enable_reg_pp0_iter2,
      I2 => \B_V_data_1_state_reg[0]_0\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => B_V_data_1_sel_wr,
      O => B_V_data_1_sel_wr_i_1_n_0
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_sel_wr_i_1_n_0,
      Q => B_V_data_1_sel_wr,
      R => \^ap_rst_n_inv\
    );
\B_V_data_1_state[0]_i_1__0\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8080FF80FF00FF00"
    )
        port map (
      I0 => B_V_data_1_sel0,
      I1 => ap_enable_reg_pp0_iter2,
      I2 => \B_V_data_1_state_reg[0]_0\,
      I3 => \B_V_data_1_state_reg_n_0_[0]\,
      I4 => out_stream_TREADY,
      I5 => \B_V_data_1_state_reg_n_0_[1]\,
      O => \B_V_data_1_state[0]_i_1__0_n_0\
    );
\B_V_data_1_state[1]_i_1__0\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"BBFBFBFBFBFBFBFB"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => B_V_data_1_sel0,
      I4 => ap_enable_reg_pp0_iter2,
      I5 => \B_V_data_1_state_reg[0]_0\,
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
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__0_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => \^ap_rst_n_inv\
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => \^ap_rst_n_inv\
    );
\out_stream_TLAST[0]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B,
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A,
      O => out_stream_TLAST(0)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \system_bit_sync_0_0_bit_sync_regslice_both__parameterized2\ is
  port (
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 0 to 0 );
    \B_V_data_1_state_reg[0]_0\ : out STD_LOGIC;
    D : out STD_LOGIC_VECTOR ( 30 downto 0 );
    CO : out STD_LOGIC_VECTOR ( 0 to 0 );
    B_V_data_1_sel0 : out STD_LOGIC;
    \icmp_ln45_reg_184_reg[0]\ : out STD_LOGIC;
    SR : out STD_LOGIC_VECTOR ( 0 to 0 );
    E : out STD_LOGIC_VECTOR ( 0 to 0 );
    ap_enable_reg_pp0_iter1_reg : out STD_LOGIC;
    ap_enable_reg_pp0_iter2_reg : out STD_LOGIC;
    ap_enable_reg_pp0_iter1_reg_0 : out STD_LOGIC;
    \curr_sign_reg_177_reg[0]\ : out STD_LOGIC;
    B_V_data_1_sel_rd_reg_0 : out STD_LOGIC;
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    ap_enable_reg_pp0_iter2 : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_1\ : in STD_LOGIC;
    in_stream_TVALID_int_regslice : in STD_LOGIC;
    icmp_ln45_reg_184_pp0_iter2_reg : in STD_LOGIC;
    ap_enable_reg_pp0_iter3 : in STD_LOGIC;
    Q : in STD_LOGIC_VECTOR ( 31 downto 0 );
    curr_sign_reg_177 : in STD_LOGIC;
    prev_sign : in STD_LOGIC;
    curr_sign_reg_177_pp0_iter1_reg : in STD_LOGIC;
    ap_enable_reg_pp0_iter1 : in STD_LOGIC;
    B_V_data_1_sel : in STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \system_bit_sync_0_0_bit_sync_regslice_both__parameterized2\ : entity is "bit_sync_regslice_both";
end \system_bit_sync_0_0_bit_sync_regslice_both__parameterized2\;

architecture STRUCTURE of \system_bit_sync_0_0_bit_sync_regslice_both__parameterized2\ is
  signal \B_V_data_1_payload_A[0]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[0]\ : STD_LOGIC;
  signal \^b_v_data_1_sel0\ : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_rd_reg_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[0]_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  signal \^co\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \^d\ : STD_LOGIC_VECTOR ( 30 downto 0 );
  signal \^e\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \curr_sign_reg_177[0]_i_3_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_10_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_2_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_3_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_4_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_5_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_6_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_7_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_8_n_0\ : STD_LOGIC;
  signal \icmp_ln45_reg_184[0]_i_9_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_10_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_11_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_12_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_13_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_14_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_16_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_17_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_18_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_19_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_20_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_21_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_22_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_23_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_25_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_26_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_27_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_28_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_29_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_30_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_31_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_32_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_33_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_34_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_35_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_36_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_37_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_38_n_0\ : STD_LOGIC;
  signal \phase_counter[31]_i_9_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[12]_i_1_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[12]_i_1_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[12]_i_1_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[12]_i_1_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[16]_i_1_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[16]_i_1_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[16]_i_1_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[20]_i_1_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[20]_i_1_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[20]_i_1_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[20]_i_1_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[24]_i_1_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[24]_i_1_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[24]_i_1_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[24]_i_1_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[28]_i_1_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[28]_i_1_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[28]_i_1_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[28]_i_1_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_15_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_15_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_15_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_15_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_24_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_24_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_24_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_24_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_3_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_3_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_4_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_4_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_8_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_8_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_8_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[31]_i_8_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[4]_i_1_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[4]_i_1_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[4]_i_1_n_3\ : STD_LOGIC;
  signal \phase_counter_reg[8]_i_1_n_0\ : STD_LOGIC;
  signal \phase_counter_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \phase_counter_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \phase_counter_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal select_ln40_fu_123_p3 : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal \NLW_phase_counter_reg[31]_i_15_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_phase_counter_reg[31]_i_24_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_phase_counter_reg[31]_i_3_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_phase_counter_reg[31]_i_3_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_phase_counter_reg[31]_i_4_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_phase_counter_reg[31]_i_4_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_phase_counter_reg[31]_i_8_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of B_V_data_1_data_out : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \B_V_data_1_sel_wr_i_1__0\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of ap_enable_reg_pp0_iter1_i_1 : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of ap_enable_reg_pp0_iter2_i_1 : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of ap_enable_reg_pp0_iter3_i_1 : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \icmp_ln45_reg_184[0]_i_5\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \icmp_ln45_reg_184[0]_i_7\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \phase_counter[31]_i_1\ : label is "soft_lutpair6";
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \phase_counter_reg[12]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \phase_counter_reg[16]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \phase_counter_reg[20]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \phase_counter_reg[24]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \phase_counter_reg[28]_i_1\ : label is 35;
  attribute COMPARATOR_THRESHOLD : integer;
  attribute COMPARATOR_THRESHOLD of \phase_counter_reg[31]_i_15\ : label is 11;
  attribute COMPARATOR_THRESHOLD of \phase_counter_reg[31]_i_24\ : label is 11;
  attribute ADDER_THRESHOLD of \phase_counter_reg[31]_i_3\ : label is 35;
  attribute COMPARATOR_THRESHOLD of \phase_counter_reg[31]_i_4\ : label is 11;
  attribute COMPARATOR_THRESHOLD of \phase_counter_reg[31]_i_8\ : label is 11;
  attribute ADDER_THRESHOLD of \phase_counter_reg[4]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \phase_counter_reg[8]_i_1\ : label is 35;
  attribute SOFT_HLUTNM of \prev_sign[0]_i_1\ : label is "soft_lutpair6";
begin
  B_V_data_1_sel0 <= \^b_v_data_1_sel0\;
  \B_V_data_1_state_reg[0]_0\ <= \^b_v_data_1_state_reg[0]_0\;
  CO(0) <= \^co\(0);
  D(30 downto 0) <= \^d\(30 downto 0);
  E(0) <= \^e\(0);
B_V_data_1_data_out: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(0)
    );
\B_V_data_1_payload_A[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EFEE2022"
    )
        port map (
      I0 => curr_sign_reg_177_pp0_iter1_reg,
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \^b_v_data_1_state_reg[0]_0\,
      I4 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      O => \B_V_data_1_payload_A[0]_i_1__0_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_A[0]_i_1__0_n_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_B[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BFBB8088"
    )
        port map (
      I0 => curr_sign_reg_177_pp0_iter1_reg,
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \^b_v_data_1_state_reg[0]_0\,
      I4 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      O => \B_V_data_1_payload_B[0]_i_1__0_n_0\
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_B[0]_i_1__0_n_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__0\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \^b_v_data_1_sel0\,
      I1 => B_V_data_1_sel,
      O => B_V_data_1_sel_rd_reg_0
    );
\B_V_data_1_sel_rd_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_0\,
      I1 => out_stream_TREADY,
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
\B_V_data_1_sel_wr_i_1__0\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7F80"
    )
        port map (
      I0 => \B_V_data_1_state_reg[0]_1\,
      I1 => ap_enable_reg_pp0_iter2,
      I2 => \^b_v_data_1_sel0\,
      I3 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__0_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__0_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FF2A2A2A2A2A2A2A"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_0\,
      I1 => out_stream_TREADY,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \B_V_data_1_state_reg[0]_1\,
      I4 => ap_enable_reg_pp0_iter2,
      I5 => \^b_v_data_1_sel0\,
      O => \B_V_data_1_state[0]_i_1_n_0\
    );
\B_V_data_1_state[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"BBFBFBFBFBFBFBFB"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \^b_v_data_1_state_reg[0]_0\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \^b_v_data_1_sel0\,
      I4 => ap_enable_reg_pp0_iter2,
      I5 => \B_V_data_1_state_reg[0]_1\,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1_n_0\,
      Q => \^b_v_data_1_state_reg[0]_0\,
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
ap_enable_reg_pp0_iter1_i_1: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^b_v_data_1_sel0\,
      I1 => ap_enable_reg_pp0_iter1,
      O => ap_enable_reg_pp0_iter1_reg
    );
ap_enable_reg_pp0_iter2_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter1,
      I1 => \^b_v_data_1_sel0\,
      I2 => ap_enable_reg_pp0_iter2,
      O => ap_enable_reg_pp0_iter1_reg_0
    );
ap_enable_reg_pp0_iter3_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter2,
      I1 => \^b_v_data_1_sel0\,
      I2 => ap_enable_reg_pp0_iter3,
      O => ap_enable_reg_pp0_iter2_reg
    );
\curr_sign_reg_177[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80888888"
    )
        port map (
      I0 => \curr_sign_reg_177[0]_i_3_n_0\,
      I1 => in_stream_TVALID_int_regslice,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => ap_enable_reg_pp0_iter2,
      I4 => \B_V_data_1_state_reg[0]_1\,
      O => \^b_v_data_1_sel0\
    );
\curr_sign_reg_177[0]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"8A8FFFFF"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[1]\,
      I1 => out_stream_TREADY,
      I2 => \^b_v_data_1_state_reg[0]_0\,
      I3 => icmp_ln45_reg_184_pp0_iter2_reg,
      I4 => ap_enable_reg_pp0_iter3,
      O => \curr_sign_reg_177[0]_i_3_n_0\
    );
\icmp_ln45_reg_184[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000AAAA0300AAAA"
    )
        port map (
      I0 => \B_V_data_1_state_reg[0]_1\,
      I1 => \icmp_ln45_reg_184[0]_i_2_n_0\,
      I2 => \icmp_ln45_reg_184[0]_i_3_n_0\,
      I3 => \icmp_ln45_reg_184[0]_i_4_n_0\,
      I4 => \^b_v_data_1_sel0\,
      I5 => \icmp_ln45_reg_184[0]_i_5_n_0\,
      O => \icmp_ln45_reg_184_reg[0]\
    );
\icmp_ln45_reg_184[0]_i_10\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFDFFFFFD"
    )
        port map (
      I0 => Q(2),
      I1 => Q(14),
      I2 => Q(12),
      I3 => curr_sign_reg_177,
      I4 => prev_sign,
      I5 => Q(24),
      O => \icmp_ln45_reg_184[0]_i_10_n_0\
    );
\icmp_ln45_reg_184[0]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFAFAFAFAE"
    )
        port map (
      I0 => \icmp_ln45_reg_184[0]_i_6_n_0\,
      I1 => Q(19),
      I2 => \icmp_ln45_reg_184[0]_i_7_n_0\,
      I3 => Q(8),
      I4 => Q(28),
      I5 => \icmp_ln45_reg_184[0]_i_8_n_0\,
      O => \icmp_ln45_reg_184[0]_i_2_n_0\
    );
\icmp_ln45_reg_184[0]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFF33333332"
    )
        port map (
      I0 => Q(25),
      I1 => \icmp_ln45_reg_184[0]_i_7_n_0\,
      I2 => Q(16),
      I3 => Q(21),
      I4 => Q(15),
      I5 => \icmp_ln45_reg_184[0]_i_9_n_0\,
      O => \icmp_ln45_reg_184[0]_i_3_n_0\
    );
\icmp_ln45_reg_184[0]_i_4\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000004"
    )
        port map (
      I0 => Q(31),
      I1 => Q(3),
      I2 => \icmp_ln45_reg_184[0]_i_7_n_0\,
      I3 => Q(17),
      I4 => Q(10),
      I5 => \icmp_ln45_reg_184[0]_i_10_n_0\,
      O => \icmp_ln45_reg_184[0]_i_4_n_0\
    );
\icmp_ln45_reg_184[0]_i_5\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"F00FE00E"
    )
        port map (
      I0 => Q(0),
      I1 => Q(6),
      I2 => curr_sign_reg_177,
      I3 => prev_sign,
      I4 => Q(23),
      O => \icmp_ln45_reg_184[0]_i_5_n_0\
    );
\icmp_ln45_reg_184[0]_i_6\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FF0000FFFE0000FE"
    )
        port map (
      I0 => Q(18),
      I1 => Q(30),
      I2 => Q(26),
      I3 => curr_sign_reg_177,
      I4 => prev_sign,
      I5 => Q(29),
      O => \icmp_ln45_reg_184[0]_i_6_n_0\
    );
\icmp_ln45_reg_184[0]_i_7\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => prev_sign,
      I1 => curr_sign_reg_177,
      O => \icmp_ln45_reg_184[0]_i_7_n_0\
    );
\icmp_ln45_reg_184[0]_i_8\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFFFFE"
    )
        port map (
      I0 => Q(1),
      I1 => Q(5),
      I2 => Q(22),
      I3 => Q(9),
      I4 => Q(11),
      I5 => Q(4),
      O => \icmp_ln45_reg_184[0]_i_8_n_0\
    );
\icmp_ln45_reg_184[0]_i_9\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FF0000FFFE0000FE"
    )
        port map (
      I0 => Q(7),
      I1 => Q(13),
      I2 => Q(20),
      I3 => curr_sign_reg_177,
      I4 => prev_sign,
      I5 => Q(27),
      O => \icmp_ln45_reg_184[0]_i_9_n_0\
    );
\phase_counter[12]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(12),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(12)
    );
\phase_counter[12]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(11),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(11)
    );
\phase_counter[12]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(10),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(10)
    );
\phase_counter[12]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(9),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(9)
    );
\phase_counter[16]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(16),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(16)
    );
\phase_counter[16]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(15),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(15)
    );
\phase_counter[16]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(14),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(14)
    );
\phase_counter[16]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(13),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(13)
    );
\phase_counter[20]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(20),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(20)
    );
\phase_counter[20]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(19),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(19)
    );
\phase_counter[20]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(18),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(18)
    );
\phase_counter[20]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(17),
      I1 => prev_sign,
      I2 => curr_sign_reg_177,
      O => select_ln40_fu_123_p3(17)
    );
\phase_counter[24]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(24),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(24)
    );
\phase_counter[24]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(23),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(23)
    );
\phase_counter[24]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(22),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(22)
    );
\phase_counter[24]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(21),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(21)
    );
\phase_counter[28]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(28),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(28)
    );
\phase_counter[28]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(27),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(27)
    );
\phase_counter[28]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(26),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(26)
    );
\phase_counter[28]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(25),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(25)
    );
\phase_counter[31]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \^co\(0),
      I1 => \^e\(0),
      O => SR(0)
    );
\phase_counter[31]_i_10\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(28),
      I1 => \^d\(27),
      O => \phase_counter[31]_i_10_n_0\
    );
\phase_counter[31]_i_11\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(26),
      I1 => \^d\(25),
      O => \phase_counter[31]_i_11_n_0\
    );
\phase_counter[31]_i_12\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(29),
      I1 => \^d\(30),
      O => \phase_counter[31]_i_12_n_0\
    );
\phase_counter[31]_i_13\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(27),
      I1 => \^d\(28),
      O => \phase_counter[31]_i_13_n_0\
    );
\phase_counter[31]_i_14\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(25),
      I1 => \^d\(26),
      O => \phase_counter[31]_i_14_n_0\
    );
\phase_counter[31]_i_16\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(24),
      I1 => \^d\(23),
      O => \phase_counter[31]_i_16_n_0\
    );
\phase_counter[31]_i_17\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(22),
      I1 => \^d\(21),
      O => \phase_counter[31]_i_17_n_0\
    );
\phase_counter[31]_i_18\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(20),
      I1 => \^d\(19),
      O => \phase_counter[31]_i_18_n_0\
    );
\phase_counter[31]_i_19\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(18),
      I1 => \^d\(17),
      O => \phase_counter[31]_i_19_n_0\
    );
\phase_counter[31]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AA2A000000000000"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter1,
      I1 => \B_V_data_1_state_reg[0]_1\,
      I2 => ap_enable_reg_pp0_iter2,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => in_stream_TVALID_int_regslice,
      I5 => \curr_sign_reg_177[0]_i_3_n_0\,
      O => \^e\(0)
    );
\phase_counter[31]_i_20\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(23),
      I1 => \^d\(24),
      O => \phase_counter[31]_i_20_n_0\
    );
\phase_counter[31]_i_21\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(21),
      I1 => \^d\(22),
      O => \phase_counter[31]_i_21_n_0\
    );
\phase_counter[31]_i_22\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(19),
      I1 => \^d\(20),
      O => \phase_counter[31]_i_22_n_0\
    );
\phase_counter[31]_i_23\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(17),
      I1 => \^d\(18),
      O => \phase_counter[31]_i_23_n_0\
    );
\phase_counter[31]_i_25\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(16),
      I1 => \^d\(15),
      O => \phase_counter[31]_i_25_n_0\
    );
\phase_counter[31]_i_26\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(14),
      I1 => \^d\(13),
      O => \phase_counter[31]_i_26_n_0\
    );
\phase_counter[31]_i_27\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(12),
      I1 => \^d\(11),
      O => \phase_counter[31]_i_27_n_0\
    );
\phase_counter[31]_i_28\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(10),
      I1 => \^d\(9),
      O => \phase_counter[31]_i_28_n_0\
    );
\phase_counter[31]_i_29\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(15),
      I1 => \^d\(16),
      O => \phase_counter[31]_i_29_n_0\
    );
\phase_counter[31]_i_30\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(13),
      I1 => \^d\(14),
      O => \phase_counter[31]_i_30_n_0\
    );
\phase_counter[31]_i_31\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(11),
      I1 => \^d\(12),
      O => \phase_counter[31]_i_31_n_0\
    );
\phase_counter[31]_i_32\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(9),
      I1 => \^d\(10),
      O => \phase_counter[31]_i_32_n_0\
    );
\phase_counter[31]_i_33\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(8),
      I1 => \^d\(7),
      O => \phase_counter[31]_i_33_n_0\
    );
\phase_counter[31]_i_34\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \^d\(6),
      I1 => \^d\(5),
      O => \phase_counter[31]_i_34_n_0\
    );
\phase_counter[31]_i_35\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(7),
      I1 => \^d\(8),
      O => \phase_counter[31]_i_35_n_0\
    );
\phase_counter[31]_i_36\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^d\(5),
      I1 => \^d\(6),
      O => \phase_counter[31]_i_36_n_0\
    );
\phase_counter[31]_i_37\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \^d\(3),
      I1 => \^d\(4),
      O => \phase_counter[31]_i_37_n_0\
    );
\phase_counter[31]_i_38\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \^d\(1),
      I1 => \^d\(2),
      O => \phase_counter[31]_i_38_n_0\
    );
\phase_counter[31]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(31),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(31)
    );
\phase_counter[31]_i_6\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(30),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(30)
    );
\phase_counter[31]_i_7\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(29),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(29)
    );
\phase_counter[31]_i_9\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \^d\(29),
      I1 => \^d\(30),
      O => \phase_counter[31]_i_9_n_0\
    );
\phase_counter[4]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(0),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(0)
    );
\phase_counter[4]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(4),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(4)
    );
\phase_counter[4]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(3),
      I1 => prev_sign,
      I2 => curr_sign_reg_177,
      O => select_ln40_fu_123_p3(3)
    );
\phase_counter[4]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(2),
      I1 => prev_sign,
      I2 => curr_sign_reg_177,
      O => select_ln40_fu_123_p3(2)
    );
\phase_counter[4]_i_6\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(1),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(1)
    );
\phase_counter[8]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(8),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(8)
    );
\phase_counter[8]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(7),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(7)
    );
\phase_counter[8]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(6),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(6)
    );
\phase_counter[8]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(5),
      I1 => curr_sign_reg_177,
      I2 => prev_sign,
      O => select_ln40_fu_123_p3(5)
    );
\phase_counter_reg[12]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[8]_i_1_n_0\,
      CO(3) => \phase_counter_reg[12]_i_1_n_0\,
      CO(2) => \phase_counter_reg[12]_i_1_n_1\,
      CO(1) => \phase_counter_reg[12]_i_1_n_2\,
      CO(0) => \phase_counter_reg[12]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \^d\(11 downto 8),
      S(3 downto 0) => select_ln40_fu_123_p3(12 downto 9)
    );
\phase_counter_reg[16]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[12]_i_1_n_0\,
      CO(3) => \phase_counter_reg[16]_i_1_n_0\,
      CO(2) => \phase_counter_reg[16]_i_1_n_1\,
      CO(1) => \phase_counter_reg[16]_i_1_n_2\,
      CO(0) => \phase_counter_reg[16]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \^d\(15 downto 12),
      S(3 downto 0) => select_ln40_fu_123_p3(16 downto 13)
    );
\phase_counter_reg[20]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[16]_i_1_n_0\,
      CO(3) => \phase_counter_reg[20]_i_1_n_0\,
      CO(2) => \phase_counter_reg[20]_i_1_n_1\,
      CO(1) => \phase_counter_reg[20]_i_1_n_2\,
      CO(0) => \phase_counter_reg[20]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \^d\(19 downto 16),
      S(3 downto 0) => select_ln40_fu_123_p3(20 downto 17)
    );
\phase_counter_reg[24]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[20]_i_1_n_0\,
      CO(3) => \phase_counter_reg[24]_i_1_n_0\,
      CO(2) => \phase_counter_reg[24]_i_1_n_1\,
      CO(1) => \phase_counter_reg[24]_i_1_n_2\,
      CO(0) => \phase_counter_reg[24]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \^d\(23 downto 20),
      S(3 downto 0) => select_ln40_fu_123_p3(24 downto 21)
    );
\phase_counter_reg[28]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[24]_i_1_n_0\,
      CO(3) => \phase_counter_reg[28]_i_1_n_0\,
      CO(2) => \phase_counter_reg[28]_i_1_n_1\,
      CO(1) => \phase_counter_reg[28]_i_1_n_2\,
      CO(0) => \phase_counter_reg[28]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \^d\(27 downto 24),
      S(3 downto 0) => select_ln40_fu_123_p3(28 downto 25)
    );
\phase_counter_reg[31]_i_15\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[31]_i_24_n_0\,
      CO(3) => \phase_counter_reg[31]_i_15_n_0\,
      CO(2) => \phase_counter_reg[31]_i_15_n_1\,
      CO(1) => \phase_counter_reg[31]_i_15_n_2\,
      CO(0) => \phase_counter_reg[31]_i_15_n_3\,
      CYINIT => '0',
      DI(3) => \phase_counter[31]_i_25_n_0\,
      DI(2) => \phase_counter[31]_i_26_n_0\,
      DI(1) => \phase_counter[31]_i_27_n_0\,
      DI(0) => \phase_counter[31]_i_28_n_0\,
      O(3 downto 0) => \NLW_phase_counter_reg[31]_i_15_O_UNCONNECTED\(3 downto 0),
      S(3) => \phase_counter[31]_i_29_n_0\,
      S(2) => \phase_counter[31]_i_30_n_0\,
      S(1) => \phase_counter[31]_i_31_n_0\,
      S(0) => \phase_counter[31]_i_32_n_0\
    );
\phase_counter_reg[31]_i_24\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \phase_counter_reg[31]_i_24_n_0\,
      CO(2) => \phase_counter_reg[31]_i_24_n_1\,
      CO(1) => \phase_counter_reg[31]_i_24_n_2\,
      CO(0) => \phase_counter_reg[31]_i_24_n_3\,
      CYINIT => '0',
      DI(3) => \phase_counter[31]_i_33_n_0\,
      DI(2) => \phase_counter[31]_i_34_n_0\,
      DI(1) => \^d\(4),
      DI(0) => \^d\(2),
      O(3 downto 0) => \NLW_phase_counter_reg[31]_i_24_O_UNCONNECTED\(3 downto 0),
      S(3) => \phase_counter[31]_i_35_n_0\,
      S(2) => \phase_counter[31]_i_36_n_0\,
      S(1) => \phase_counter[31]_i_37_n_0\,
      S(0) => \phase_counter[31]_i_38_n_0\
    );
\phase_counter_reg[31]_i_3\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[28]_i_1_n_0\,
      CO(3 downto 2) => \NLW_phase_counter_reg[31]_i_3_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \phase_counter_reg[31]_i_3_n_2\,
      CO(0) => \phase_counter_reg[31]_i_3_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_phase_counter_reg[31]_i_3_O_UNCONNECTED\(3),
      O(2 downto 0) => \^d\(30 downto 28),
      S(3) => '0',
      S(2 downto 0) => select_ln40_fu_123_p3(31 downto 29)
    );
\phase_counter_reg[31]_i_4\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[31]_i_8_n_0\,
      CO(3) => \NLW_phase_counter_reg[31]_i_4_CO_UNCONNECTED\(3),
      CO(2) => \^co\(0),
      CO(1) => \phase_counter_reg[31]_i_4_n_2\,
      CO(0) => \phase_counter_reg[31]_i_4_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2) => \phase_counter[31]_i_9_n_0\,
      DI(1) => \phase_counter[31]_i_10_n_0\,
      DI(0) => \phase_counter[31]_i_11_n_0\,
      O(3 downto 0) => \NLW_phase_counter_reg[31]_i_4_O_UNCONNECTED\(3 downto 0),
      S(3) => '0',
      S(2) => \phase_counter[31]_i_12_n_0\,
      S(1) => \phase_counter[31]_i_13_n_0\,
      S(0) => \phase_counter[31]_i_14_n_0\
    );
\phase_counter_reg[31]_i_8\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[31]_i_15_n_0\,
      CO(3) => \phase_counter_reg[31]_i_8_n_0\,
      CO(2) => \phase_counter_reg[31]_i_8_n_1\,
      CO(1) => \phase_counter_reg[31]_i_8_n_2\,
      CO(0) => \phase_counter_reg[31]_i_8_n_3\,
      CYINIT => '0',
      DI(3) => \phase_counter[31]_i_16_n_0\,
      DI(2) => \phase_counter[31]_i_17_n_0\,
      DI(1) => \phase_counter[31]_i_18_n_0\,
      DI(0) => \phase_counter[31]_i_19_n_0\,
      O(3 downto 0) => \NLW_phase_counter_reg[31]_i_8_O_UNCONNECTED\(3 downto 0),
      S(3) => \phase_counter[31]_i_20_n_0\,
      S(2) => \phase_counter[31]_i_21_n_0\,
      S(1) => \phase_counter[31]_i_22_n_0\,
      S(0) => \phase_counter[31]_i_23_n_0\
    );
\phase_counter_reg[4]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \phase_counter_reg[4]_i_1_n_0\,
      CO(2) => \phase_counter_reg[4]_i_1_n_1\,
      CO(1) => \phase_counter_reg[4]_i_1_n_2\,
      CO(0) => \phase_counter_reg[4]_i_1_n_3\,
      CYINIT => select_ln40_fu_123_p3(0),
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \^d\(3 downto 0),
      S(3 downto 0) => select_ln40_fu_123_p3(4 downto 1)
    );
\phase_counter_reg[8]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \phase_counter_reg[4]_i_1_n_0\,
      CO(3) => \phase_counter_reg[8]_i_1_n_0\,
      CO(2) => \phase_counter_reg[8]_i_1_n_1\,
      CO(1) => \phase_counter_reg[8]_i_1_n_2\,
      CO(0) => \phase_counter_reg[8]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \^d\(7 downto 4),
      S(3 downto 0) => select_ln40_fu_123_p3(8 downto 5)
    );
\prev_sign[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => curr_sign_reg_177,
      I1 => \^e\(0),
      I2 => prev_sign,
      O => \curr_sign_reg_177_reg[0]\
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_bit_sync_0_0_bit_sync is
  port (
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 15 downto 0 );
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TREADY : out STD_LOGIC;
    in_stream_TKEEP : in STD_LOGIC_VECTOR ( 1 downto 0 );
    in_stream_TSTRB : in STD_LOGIC_VECTOR ( 1 downto 0 );
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 7 downto 0 );
    out_stream_TVALID : out STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TKEEP : out STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TSTRB : out STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of system_bit_sync_0_0_bit_sync : entity is "bit_sync";
  attribute ap_ST_fsm_pp0_stage0 : string;
  attribute ap_ST_fsm_pp0_stage0 of system_bit_sync_0_0_bit_sync : entity is "1'b1";
  attribute hls_module : string;
  attribute hls_module of system_bit_sync_0_0_bit_sync : entity is "yes";
end system_bit_sync_0_0_bit_sync;

architecture STRUCTURE of system_bit_sync_0_0_bit_sync is
  signal \<const0>\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal B_V_data_1_sel0 : STD_LOGIC;
  signal add_ln60_fu_137_p2 : STD_LOGIC_VECTOR ( 31 downto 1 );
  signal ap_enable_reg_pp0_iter1 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter2 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter3 : STD_LOGIC;
  signal ap_rst_n_inv : STD_LOGIC;
  signal curr_sign_fu_104_p2 : STD_LOGIC;
  signal curr_sign_reg_177 : STD_LOGIC;
  signal curr_sign_reg_177_pp0_iter1_reg : STD_LOGIC;
  signal icmp_ln45_reg_184_pp0_iter2_reg : STD_LOGIC;
  signal \icmp_ln45_reg_184_reg_n_0_[0]\ : STD_LOGIC;
  signal in_stream_TLAST_int_regslice : STD_LOGIC;
  signal in_stream_TVALID_int_regslice : STD_LOGIC;
  signal \^out_stream_tdata\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal p_1_in : STD_LOGIC;
  signal phase_counter : STD_LOGIC;
  signal \phase_counter[0]_i_1_n_0\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[0]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[10]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[11]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[12]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[13]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[14]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[15]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[16]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[17]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[18]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[19]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[1]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[20]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[21]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[22]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[23]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[24]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[25]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[26]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[27]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[28]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[29]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[2]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[30]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[31]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[3]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[4]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[5]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[6]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[7]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[8]\ : STD_LOGIC;
  signal \phase_counter_reg_n_0_[9]\ : STD_LOGIC;
  signal pkt_last_V_reg_172 : STD_LOGIC;
  signal pkt_last_V_reg_172_pp0_iter1_reg : STD_LOGIC;
  signal prev_sign : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_35 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_37 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_38 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_39 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_40 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_41 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_42 : STD_LOGIC;
begin
  out_stream_TDATA(7) <= \<const0>\;
  out_stream_TDATA(6) <= \<const0>\;
  out_stream_TDATA(5) <= \<const0>\;
  out_stream_TDATA(4) <= \<const0>\;
  out_stream_TDATA(3) <= \<const0>\;
  out_stream_TDATA(2) <= \<const0>\;
  out_stream_TDATA(1) <= \<const0>\;
  out_stream_TDATA(0) <= \^out_stream_tdata\(0);
  out_stream_TKEEP(0) <= \<const0>\;
  out_stream_TSTRB(0) <= \<const0>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
ap_enable_reg_pp0_iter1_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_out_stream_V_data_V_U_n_38,
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
      D => regslice_both_out_stream_V_data_V_U_n_40,
      Q => ap_enable_reg_pp0_iter2,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter3_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_out_stream_V_data_V_U_n_39,
      Q => ap_enable_reg_pp0_iter3,
      R => ap_rst_n_inv
    );
\curr_sign_reg_177_pp0_iter1_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_sel0,
      D => curr_sign_reg_177,
      Q => curr_sign_reg_177_pp0_iter1_reg,
      R => '0'
    );
\curr_sign_reg_177_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_sel0,
      D => curr_sign_fu_104_p2,
      Q => curr_sign_reg_177,
      R => '0'
    );
\icmp_ln45_reg_184_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_sel0,
      D => \icmp_ln45_reg_184_reg_n_0_[0]\,
      Q => icmp_ln45_reg_184_pp0_iter2_reg,
      R => '0'
    );
\icmp_ln45_reg_184_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_out_stream_V_data_V_U_n_35,
      Q => \icmp_ln45_reg_184_reg_n_0_[0]\,
      R => '0'
    );
\phase_counter[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1455"
    )
        port map (
      I0 => p_1_in,
      I1 => prev_sign,
      I2 => curr_sign_reg_177,
      I3 => \phase_counter_reg_n_0_[0]\,
      O => \phase_counter[0]_i_1_n_0\
    );
\phase_counter_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => \phase_counter[0]_i_1_n_0\,
      Q => \phase_counter_reg_n_0_[0]\,
      R => '0'
    );
\phase_counter_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(10),
      Q => \phase_counter_reg_n_0_[10]\,
      R => phase_counter
    );
\phase_counter_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(11),
      Q => \phase_counter_reg_n_0_[11]\,
      R => phase_counter
    );
\phase_counter_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(12),
      Q => \phase_counter_reg_n_0_[12]\,
      R => phase_counter
    );
\phase_counter_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(13),
      Q => \phase_counter_reg_n_0_[13]\,
      R => phase_counter
    );
\phase_counter_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(14),
      Q => \phase_counter_reg_n_0_[14]\,
      R => phase_counter
    );
\phase_counter_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(15),
      Q => \phase_counter_reg_n_0_[15]\,
      R => phase_counter
    );
\phase_counter_reg[16]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(16),
      Q => \phase_counter_reg_n_0_[16]\,
      R => phase_counter
    );
\phase_counter_reg[17]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(17),
      Q => \phase_counter_reg_n_0_[17]\,
      R => phase_counter
    );
\phase_counter_reg[18]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(18),
      Q => \phase_counter_reg_n_0_[18]\,
      R => phase_counter
    );
\phase_counter_reg[19]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(19),
      Q => \phase_counter_reg_n_0_[19]\,
      R => phase_counter
    );
\phase_counter_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(1),
      Q => \phase_counter_reg_n_0_[1]\,
      R => phase_counter
    );
\phase_counter_reg[20]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(20),
      Q => \phase_counter_reg_n_0_[20]\,
      R => phase_counter
    );
\phase_counter_reg[21]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(21),
      Q => \phase_counter_reg_n_0_[21]\,
      R => phase_counter
    );
\phase_counter_reg[22]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(22),
      Q => \phase_counter_reg_n_0_[22]\,
      R => phase_counter
    );
\phase_counter_reg[23]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(23),
      Q => \phase_counter_reg_n_0_[23]\,
      R => phase_counter
    );
\phase_counter_reg[24]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(24),
      Q => \phase_counter_reg_n_0_[24]\,
      R => phase_counter
    );
\phase_counter_reg[25]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(25),
      Q => \phase_counter_reg_n_0_[25]\,
      R => phase_counter
    );
\phase_counter_reg[26]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(26),
      Q => \phase_counter_reg_n_0_[26]\,
      R => phase_counter
    );
\phase_counter_reg[27]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(27),
      Q => \phase_counter_reg_n_0_[27]\,
      R => phase_counter
    );
\phase_counter_reg[28]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(28),
      Q => \phase_counter_reg_n_0_[28]\,
      R => phase_counter
    );
\phase_counter_reg[29]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(29),
      Q => \phase_counter_reg_n_0_[29]\,
      R => phase_counter
    );
\phase_counter_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(2),
      Q => \phase_counter_reg_n_0_[2]\,
      R => phase_counter
    );
\phase_counter_reg[30]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(30),
      Q => \phase_counter_reg_n_0_[30]\,
      R => phase_counter
    );
\phase_counter_reg[31]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(31),
      Q => \phase_counter_reg_n_0_[31]\,
      R => phase_counter
    );
\phase_counter_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(3),
      Q => \phase_counter_reg_n_0_[3]\,
      R => phase_counter
    );
\phase_counter_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(4),
      Q => \phase_counter_reg_n_0_[4]\,
      R => phase_counter
    );
\phase_counter_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(5),
      Q => \phase_counter_reg_n_0_[5]\,
      R => phase_counter
    );
\phase_counter_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(6),
      Q => \phase_counter_reg_n_0_[6]\,
      R => phase_counter
    );
\phase_counter_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(7),
      Q => \phase_counter_reg_n_0_[7]\,
      R => phase_counter
    );
\phase_counter_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(8),
      Q => \phase_counter_reg_n_0_[8]\,
      R => phase_counter
    );
\phase_counter_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => regslice_both_out_stream_V_data_V_U_n_37,
      D => add_ln60_fu_137_p2(9),
      Q => \phase_counter_reg_n_0_[9]\,
      R => phase_counter
    );
\pkt_last_V_reg_172_pp0_iter1_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_sel0,
      D => pkt_last_V_reg_172,
      Q => pkt_last_V_reg_172_pp0_iter1_reg,
      R => '0'
    );
\pkt_last_V_reg_172_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_sel0,
      D => in_stream_TLAST_int_regslice,
      Q => pkt_last_V_reg_172,
      R => '0'
    );
\prev_sign_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_out_stream_V_data_V_U_n_41,
      Q => prev_sign,
      R => '0'
    );
regslice_both_in_stream_V_data_V_U: entity work.system_bit_sync_0_0_bit_sync_regslice_both
     port map (
      B_V_data_1_sel => B_V_data_1_sel,
      B_V_data_1_sel0 => B_V_data_1_sel0,
      B_V_data_1_sel_rd_reg_0 => regslice_both_out_stream_V_data_V_U_n_42,
      \B_V_data_1_state_reg[1]_0\ => in_stream_TREADY,
      ap_clk => ap_clk,
      ap_rst_n_inv => ap_rst_n_inv,
      curr_sign_fu_104_p2(0) => curr_sign_fu_104_p2,
      in_stream_TDATA(15 downto 0) => in_stream_TDATA(15 downto 0),
      in_stream_TVALID => in_stream_TVALID,
      in_stream_TVALID_int_regslice => in_stream_TVALID_int_regslice
    );
regslice_both_in_stream_V_last_V_U: entity work.\system_bit_sync_0_0_bit_sync_regslice_both__parameterized1\
     port map (
      B_V_data_1_sel0 => B_V_data_1_sel0,
      ap_clk => ap_clk,
      ap_rst_n_inv => ap_rst_n_inv,
      in_stream_TLAST(0) => in_stream_TLAST(0),
      in_stream_TLAST_int_regslice => in_stream_TLAST_int_regslice,
      in_stream_TVALID => in_stream_TVALID
    );
regslice_both_out_stream_V_data_V_U: entity work.\system_bit_sync_0_0_bit_sync_regslice_both__parameterized2\
     port map (
      B_V_data_1_sel => B_V_data_1_sel,
      B_V_data_1_sel0 => B_V_data_1_sel0,
      B_V_data_1_sel_rd_reg_0 => regslice_both_out_stream_V_data_V_U_n_42,
      \B_V_data_1_state_reg[0]_0\ => out_stream_TVALID,
      \B_V_data_1_state_reg[0]_1\ => \icmp_ln45_reg_184_reg_n_0_[0]\,
      CO(0) => p_1_in,
      D(30 downto 0) => add_ln60_fu_137_p2(31 downto 1),
      E(0) => regslice_both_out_stream_V_data_V_U_n_37,
      Q(31) => \phase_counter_reg_n_0_[31]\,
      Q(30) => \phase_counter_reg_n_0_[30]\,
      Q(29) => \phase_counter_reg_n_0_[29]\,
      Q(28) => \phase_counter_reg_n_0_[28]\,
      Q(27) => \phase_counter_reg_n_0_[27]\,
      Q(26) => \phase_counter_reg_n_0_[26]\,
      Q(25) => \phase_counter_reg_n_0_[25]\,
      Q(24) => \phase_counter_reg_n_0_[24]\,
      Q(23) => \phase_counter_reg_n_0_[23]\,
      Q(22) => \phase_counter_reg_n_0_[22]\,
      Q(21) => \phase_counter_reg_n_0_[21]\,
      Q(20) => \phase_counter_reg_n_0_[20]\,
      Q(19) => \phase_counter_reg_n_0_[19]\,
      Q(18) => \phase_counter_reg_n_0_[18]\,
      Q(17) => \phase_counter_reg_n_0_[17]\,
      Q(16) => \phase_counter_reg_n_0_[16]\,
      Q(15) => \phase_counter_reg_n_0_[15]\,
      Q(14) => \phase_counter_reg_n_0_[14]\,
      Q(13) => \phase_counter_reg_n_0_[13]\,
      Q(12) => \phase_counter_reg_n_0_[12]\,
      Q(11) => \phase_counter_reg_n_0_[11]\,
      Q(10) => \phase_counter_reg_n_0_[10]\,
      Q(9) => \phase_counter_reg_n_0_[9]\,
      Q(8) => \phase_counter_reg_n_0_[8]\,
      Q(7) => \phase_counter_reg_n_0_[7]\,
      Q(6) => \phase_counter_reg_n_0_[6]\,
      Q(5) => \phase_counter_reg_n_0_[5]\,
      Q(4) => \phase_counter_reg_n_0_[4]\,
      Q(3) => \phase_counter_reg_n_0_[3]\,
      Q(2) => \phase_counter_reg_n_0_[2]\,
      Q(1) => \phase_counter_reg_n_0_[1]\,
      Q(0) => \phase_counter_reg_n_0_[0]\,
      SR(0) => phase_counter,
      ap_clk => ap_clk,
      ap_enable_reg_pp0_iter1 => ap_enable_reg_pp0_iter1,
      ap_enable_reg_pp0_iter1_reg => regslice_both_out_stream_V_data_V_U_n_38,
      ap_enable_reg_pp0_iter1_reg_0 => regslice_both_out_stream_V_data_V_U_n_40,
      ap_enable_reg_pp0_iter2 => ap_enable_reg_pp0_iter2,
      ap_enable_reg_pp0_iter2_reg => regslice_both_out_stream_V_data_V_U_n_39,
      ap_enable_reg_pp0_iter3 => ap_enable_reg_pp0_iter3,
      ap_rst_n_inv => ap_rst_n_inv,
      curr_sign_reg_177 => curr_sign_reg_177,
      curr_sign_reg_177_pp0_iter1_reg => curr_sign_reg_177_pp0_iter1_reg,
      \curr_sign_reg_177_reg[0]\ => regslice_both_out_stream_V_data_V_U_n_41,
      icmp_ln45_reg_184_pp0_iter2_reg => icmp_ln45_reg_184_pp0_iter2_reg,
      \icmp_ln45_reg_184_reg[0]\ => regslice_both_out_stream_V_data_V_U_n_35,
      in_stream_TVALID_int_regslice => in_stream_TVALID_int_regslice,
      out_stream_TDATA(0) => \^out_stream_tdata\(0),
      out_stream_TREADY => out_stream_TREADY,
      prev_sign => prev_sign
    );
regslice_both_out_stream_V_last_V_U: entity work.\system_bit_sync_0_0_bit_sync_regslice_both__parameterized1_0\
     port map (
      B_V_data_1_sel0 => B_V_data_1_sel0,
      \B_V_data_1_state_reg[0]_0\ => \icmp_ln45_reg_184_reg_n_0_[0]\,
      ap_clk => ap_clk,
      ap_enable_reg_pp0_iter2 => ap_enable_reg_pp0_iter2,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      out_stream_TLAST(0) => out_stream_TLAST(0),
      out_stream_TREADY => out_stream_TREADY,
      pkt_last_V_reg_172_pp0_iter1_reg => pkt_last_V_reg_172_pp0_iter1_reg
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_bit_sync_0_0 is
  port (
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TREADY : out STD_LOGIC;
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 15 downto 0 );
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    in_stream_TKEEP : in STD_LOGIC_VECTOR ( 1 downto 0 );
    in_stream_TSTRB : in STD_LOGIC_VECTOR ( 1 downto 0 );
    out_stream_TVALID : out STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 7 downto 0 );
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TKEEP : out STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TSTRB : out STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of system_bit_sync_0_0 : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of system_bit_sync_0_0 : entity is "system_bit_sync_0_0,bit_sync,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of system_bit_sync_0_0 : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of system_bit_sync_0_0 : entity is "HLS";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of system_bit_sync_0_0 : entity is "bit_sync,Vivado 2023.1";
  attribute hls_module : string;
  attribute hls_module of system_bit_sync_0_0 : entity is "yes";
end system_bit_sync_0_0;

architecture STRUCTURE of system_bit_sync_0_0 is
  signal \<const0>\ : STD_LOGIC;
  signal \<const1>\ : STD_LOGIC;
  signal \^out_stream_tdata\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal NLW_inst_out_stream_TDATA_UNCONNECTED : STD_LOGIC_VECTOR ( 7 downto 1 );
  signal NLW_inst_out_stream_TKEEP_UNCONNECTED : STD_LOGIC_VECTOR ( 0 to 0 );
  signal NLW_inst_out_stream_TSTRB_UNCONNECTED : STD_LOGIC_VECTOR ( 0 to 0 );
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
  attribute X_INTERFACE_PARAMETER of ap_clk : signal is "XIL_INTERFACENAME ap_clk, ASSOCIATED_BUSIF in_stream:out_stream, ASSOCIATED_RESET ap_rst_n, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of ap_rst_n : signal is "xilinx.com:signal:reset:1.0 ap_rst_n RST";
  attribute X_INTERFACE_PARAMETER of ap_rst_n : signal is "XIL_INTERFACENAME ap_rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of in_stream_TREADY : signal is "xilinx.com:interface:axis:1.0 in_stream TREADY";
  attribute X_INTERFACE_INFO of in_stream_TVALID : signal is "xilinx.com:interface:axis:1.0 in_stream TVALID";
  attribute X_INTERFACE_INFO of out_stream_TREADY : signal is "xilinx.com:interface:axis:1.0 out_stream TREADY";
  attribute X_INTERFACE_INFO of out_stream_TVALID : signal is "xilinx.com:interface:axis:1.0 out_stream TVALID";
  attribute X_INTERFACE_INFO of in_stream_TDATA : signal is "xilinx.com:interface:axis:1.0 in_stream TDATA";
  attribute X_INTERFACE_INFO of in_stream_TKEEP : signal is "xilinx.com:interface:axis:1.0 in_stream TKEEP";
  attribute X_INTERFACE_INFO of in_stream_TLAST : signal is "xilinx.com:interface:axis:1.0 in_stream TLAST";
  attribute X_INTERFACE_INFO of in_stream_TSTRB : signal is "xilinx.com:interface:axis:1.0 in_stream TSTRB";
  attribute X_INTERFACE_PARAMETER of in_stream_TSTRB : signal is "XIL_INTERFACENAME in_stream, TDATA_NUM_BYTES 2, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of out_stream_TDATA : signal is "xilinx.com:interface:axis:1.0 out_stream TDATA";
  attribute X_INTERFACE_INFO of out_stream_TKEEP : signal is "xilinx.com:interface:axis:1.0 out_stream TKEEP";
  attribute X_INTERFACE_INFO of out_stream_TLAST : signal is "xilinx.com:interface:axis:1.0 out_stream TLAST";
  attribute X_INTERFACE_INFO of out_stream_TSTRB : signal is "xilinx.com:interface:axis:1.0 out_stream TSTRB";
  attribute X_INTERFACE_PARAMETER of out_stream_TSTRB : signal is "XIL_INTERFACENAME out_stream, TDATA_NUM_BYTES 1, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
begin
  out_stream_TDATA(7) <= \<const0>\;
  out_stream_TDATA(6) <= \<const0>\;
  out_stream_TDATA(5) <= \<const0>\;
  out_stream_TDATA(4) <= \<const0>\;
  out_stream_TDATA(3) <= \<const0>\;
  out_stream_TDATA(2) <= \<const0>\;
  out_stream_TDATA(1) <= \<const0>\;
  out_stream_TDATA(0) <= \^out_stream_tdata\(0);
  out_stream_TKEEP(0) <= \<const1>\;
  out_stream_TSTRB(0) <= \<const0>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
VCC: unisim.vcomponents.VCC
     port map (
      P => \<const1>\
    );
inst: entity work.system_bit_sync_0_0_bit_sync
     port map (
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      in_stream_TDATA(15 downto 0) => in_stream_TDATA(15 downto 0),
      in_stream_TKEEP(1 downto 0) => B"00",
      in_stream_TLAST(0) => in_stream_TLAST(0),
      in_stream_TREADY => in_stream_TREADY,
      in_stream_TSTRB(1 downto 0) => B"00",
      in_stream_TVALID => in_stream_TVALID,
      out_stream_TDATA(7 downto 1) => NLW_inst_out_stream_TDATA_UNCONNECTED(7 downto 1),
      out_stream_TDATA(0) => \^out_stream_tdata\(0),
      out_stream_TKEEP(0) => NLW_inst_out_stream_TKEEP_UNCONNECTED(0),
      out_stream_TLAST(0) => out_stream_TLAST(0),
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TSTRB(0) => NLW_inst_out_stream_TSTRB_UNCONNECTED(0),
      out_stream_TVALID => out_stream_TVALID
    );
end STRUCTURE;
