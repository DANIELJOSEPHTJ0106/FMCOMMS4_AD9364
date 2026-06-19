-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Sat Jan  3 12:24:46 2026
-- Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim
--               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_fsk_discriminator_0_0/system_fsk_discriminator_0_0_sim_netlist.vhdl
-- Design      : system_fsk_discriminator_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0 is
  port (
    P : out STD_LOGIC_VECTOR ( 1 downto 0 );
    \counter_reg[12]\ : out STD_LOGIC;
    \counter_reg[8]\ : out STD_LOGIC;
    \counter_reg[16]\ : out STD_LOGIC;
    \counter_reg[28]\ : out STD_LOGIC;
    S : out STD_LOGIC_VECTOR ( 2 downto 0 );
    \sum_i_reg[22]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[26]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[30]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[18]\ : out STD_LOGIC_VECTOR ( 2 downto 0 );
    \sum_q_reg[22]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[26]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[30]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    p_reg_reg_0 : out STD_LOGIC;
    p_reg_reg_1 : out STD_LOGIC;
    CO : out STD_LOGIC_VECTOR ( 0 to 0 );
    p_reg_reg_2 : out STD_LOGIC;
    p_reg_reg_3 : out STD_LOGIC;
    p_reg_reg_4 : out STD_LOGIC;
    p_reg_reg_5 : out STD_LOGIC;
    p_reg_reg_6 : out STD_LOGIC;
    p_reg_reg_7 : out STD_LOGIC;
    p_reg_reg_8 : out STD_LOGIC;
    p_reg_reg_9 : out STD_LOGIC;
    p_reg_reg_10 : out STD_LOGIC;
    p_reg_reg_11 : out STD_LOGIC;
    p_reg_reg_12 : out STD_LOGIC;
    p_reg_reg_13 : out STD_LOGIC;
    p_reg_reg_14 : out STD_LOGIC;
    p_reg_reg_15 : out STD_LOGIC;
    ap_block_pp0_stage0_11001 : in STD_LOGIC;
    p_reg_reg_16 : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    B : in STD_LOGIC_VECTOR ( 15 downto 0 );
    A : in STD_LOGIC_VECTOR ( 15 downto 0 );
    PCOUT : in STD_LOGIC_VECTOR ( 47 downto 0 );
    \out\ : in STD_LOGIC_VECTOR ( 31 downto 0 );
    sum_i_reg : in STD_LOGIC_VECTOR ( 15 downto 0 );
    sum_q_reg : in STD_LOGIC_VECTOR ( 15 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0 : entity is "fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0";
end system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0;

architecture STRUCTURE of system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0 is
  signal \B_V_data_1_payload_A[14]_i_10_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_11_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_12_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_13_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_14_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_15_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_16_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_17_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_18_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_19_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_6_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_8_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_9_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[14]_i_5_n_3\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[14]_i_7_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[14]_i_7_n_1\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[14]_i_7_n_2\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[14]_i_7_n_3\ : STD_LOGIC;
  signal \^co\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \^p\ : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal add_ln49_fu_218_p2 : STD_LOGIC_VECTOR ( 31 downto 1 );
  signal p_reg_reg_n_100 : STD_LOGIC;
  signal p_reg_reg_n_101 : STD_LOGIC;
  signal p_reg_reg_n_102 : STD_LOGIC;
  signal p_reg_reg_n_103 : STD_LOGIC;
  signal p_reg_reg_n_104 : STD_LOGIC;
  signal p_reg_reg_n_105 : STD_LOGIC;
  signal p_reg_reg_n_85 : STD_LOGIC;
  signal p_reg_reg_n_86 : STD_LOGIC;
  signal p_reg_reg_n_87 : STD_LOGIC;
  signal p_reg_reg_n_88 : STD_LOGIC;
  signal p_reg_reg_n_89 : STD_LOGIC;
  signal p_reg_reg_n_90 : STD_LOGIC;
  signal p_reg_reg_n_91 : STD_LOGIC;
  signal p_reg_reg_n_92 : STD_LOGIC;
  signal p_reg_reg_n_93 : STD_LOGIC;
  signal p_reg_reg_n_94 : STD_LOGIC;
  signal p_reg_reg_n_95 : STD_LOGIC;
  signal p_reg_reg_n_96 : STD_LOGIC;
  signal p_reg_reg_n_97 : STD_LOGIC;
  signal p_reg_reg_n_98 : STD_LOGIC;
  signal p_reg_reg_n_99 : STD_LOGIC;
  signal term1_reg_518_reg_i_123_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_123_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_123_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_123_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_54_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_54_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_54_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_54_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_55_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_55_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_55_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_55_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_56_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_57_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_57_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_57_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_57_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_58_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_58_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_58_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_58_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_59_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_60_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_60_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_60_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_60_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_61_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_61_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_62_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_63_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_63_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_63_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_63_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_64_n_0 : STD_LOGIC;
  signal tmp_2_fu_426_p4 : STD_LOGIC_VECTOR ( 9 downto 1 );
  signal \NLW_B_V_data_1_payload_A_reg[14]_i_5_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_B_V_data_1_payload_A_reg[14]_i_5_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_B_V_data_1_payload_A_reg[14]_i_7_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED : STD_LOGIC;
  signal NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED : STD_LOGIC;
  signal NLW_p_reg_reg_OVERFLOW_UNCONNECTED : STD_LOGIC;
  signal NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED : STD_LOGIC;
  signal NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED : STD_LOGIC;
  signal NLW_p_reg_reg_UNDERFLOW_UNCONNECTED : STD_LOGIC;
  signal NLW_p_reg_reg_ACOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 29 downto 0 );
  signal NLW_p_reg_reg_BCOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 17 downto 0 );
  signal NLW_p_reg_reg_CARRYOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_p_reg_reg_P_UNCONNECTED : STD_LOGIC_VECTOR ( 47 downto 32 );
  signal NLW_p_reg_reg_PCOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 47 downto 0 );
  signal NLW_term1_reg_518_reg_i_61_CO_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal NLW_term1_reg_518_reg_i_61_O_UNCONNECTED : STD_LOGIC_VECTOR ( 3 to 3 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[10]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[11]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[12]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[13]_i_1\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[14]_i_3\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[1]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[2]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[3]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[4]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[5]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[6]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[7]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[8]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \B_V_data_1_payload_A[9]_i_1\ : label is "soft_lutpair2";
  attribute COMPARATOR_THRESHOLD : integer;
  attribute COMPARATOR_THRESHOLD of \B_V_data_1_payload_A_reg[14]_i_5\ : label is 11;
  attribute COMPARATOR_THRESHOLD of \B_V_data_1_payload_A_reg[14]_i_7\ : label is 11;
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_123 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_54 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_55 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_57 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_58 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_60 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_61 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_63 : label is 35;
begin
  CO(0) <= \^co\(0);
  P(1 downto 0) <= \^p\(1 downto 0);
\B_V_data_1_payload_A[0]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_99,
      I1 => \^co\(0),
      O => p_reg_reg_15
    );
\B_V_data_1_payload_A[10]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_89,
      I1 => \^co\(0),
      O => p_reg_reg_5
    );
\B_V_data_1_payload_A[11]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_88,
      I1 => \^co\(0),
      O => p_reg_reg_4
    );
\B_V_data_1_payload_A[12]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_87,
      I1 => \^co\(0),
      O => p_reg_reg_3
    );
\B_V_data_1_payload_A[13]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_86,
      I1 => \^co\(0),
      O => p_reg_reg_2
    );
\B_V_data_1_payload_A[14]_i_10\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_2_fu_426_p4(9),
      I1 => \^p\(1),
      O => \B_V_data_1_payload_A[14]_i_10_n_0\
    );
\B_V_data_1_payload_A[14]_i_11\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_2_fu_426_p4(7),
      I1 => tmp_2_fu_426_p4(8),
      O => \B_V_data_1_payload_A[14]_i_11_n_0\
    );
\B_V_data_1_payload_A[14]_i_12\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => tmp_2_fu_426_p4(5),
      I1 => tmp_2_fu_426_p4(6),
      O => \B_V_data_1_payload_A[14]_i_12_n_0\
    );
\B_V_data_1_payload_A[14]_i_13\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => tmp_2_fu_426_p4(3),
      I1 => tmp_2_fu_426_p4(4),
      O => \B_V_data_1_payload_A[14]_i_13_n_0\
    );
\B_V_data_1_payload_A[14]_i_14\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => tmp_2_fu_426_p4(1),
      I1 => tmp_2_fu_426_p4(2),
      O => \B_V_data_1_payload_A[14]_i_14_n_0\
    );
\B_V_data_1_payload_A[14]_i_15\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^p\(0),
      O => \B_V_data_1_payload_A[14]_i_15_n_0\
    );
\B_V_data_1_payload_A[14]_i_16\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_2_fu_426_p4(5),
      I1 => tmp_2_fu_426_p4(6),
      O => \B_V_data_1_payload_A[14]_i_16_n_0\
    );
\B_V_data_1_payload_A[14]_i_17\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_2_fu_426_p4(3),
      I1 => tmp_2_fu_426_p4(4),
      O => \B_V_data_1_payload_A[14]_i_17_n_0\
    );
\B_V_data_1_payload_A[14]_i_18\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_2_fu_426_p4(1),
      I1 => tmp_2_fu_426_p4(2),
      O => \B_V_data_1_payload_A[14]_i_18_n_0\
    );
\B_V_data_1_payload_A[14]_i_19\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \^p\(0),
      I1 => p_reg_reg_n_85,
      O => \B_V_data_1_payload_A[14]_i_19_n_0\
    );
\B_V_data_1_payload_A[14]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_85,
      I1 => \^co\(0),
      O => p_reg_reg_1
    );
\B_V_data_1_payload_A[14]_i_4\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AAAAAAAAAAAAAAAB"
    )
        port map (
      I0 => \^p\(1),
      I1 => tmp_2_fu_426_p4(3),
      I2 => tmp_2_fu_426_p4(2),
      I3 => tmp_2_fu_426_p4(5),
      I4 => tmp_2_fu_426_p4(4),
      I5 => \B_V_data_1_payload_A[14]_i_6_n_0\,
      O => p_reg_reg_0
    );
\B_V_data_1_payload_A[14]_i_6\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFFFFE"
    )
        port map (
      I0 => tmp_2_fu_426_p4(8),
      I1 => tmp_2_fu_426_p4(9),
      I2 => tmp_2_fu_426_p4(6),
      I3 => tmp_2_fu_426_p4(7),
      I4 => tmp_2_fu_426_p4(1),
      I5 => \^p\(0),
      O => \B_V_data_1_payload_A[14]_i_6_n_0\
    );
\B_V_data_1_payload_A[14]_i_8\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \^p\(1),
      I1 => tmp_2_fu_426_p4(9),
      O => \B_V_data_1_payload_A[14]_i_8_n_0\
    );
\B_V_data_1_payload_A[14]_i_9\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => tmp_2_fu_426_p4(7),
      I1 => tmp_2_fu_426_p4(8),
      O => \B_V_data_1_payload_A[14]_i_9_n_0\
    );
\B_V_data_1_payload_A[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_98,
      I1 => \^co\(0),
      O => p_reg_reg_14
    );
\B_V_data_1_payload_A[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_97,
      I1 => \^co\(0),
      O => p_reg_reg_13
    );
\B_V_data_1_payload_A[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_96,
      I1 => \^co\(0),
      O => p_reg_reg_12
    );
\B_V_data_1_payload_A[4]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_95,
      I1 => \^co\(0),
      O => p_reg_reg_11
    );
\B_V_data_1_payload_A[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_94,
      I1 => \^co\(0),
      O => p_reg_reg_10
    );
\B_V_data_1_payload_A[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_93,
      I1 => \^co\(0),
      O => p_reg_reg_9
    );
\B_V_data_1_payload_A[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_92,
      I1 => \^co\(0),
      O => p_reg_reg_8
    );
\B_V_data_1_payload_A[8]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_91,
      I1 => \^co\(0),
      O => p_reg_reg_7
    );
\B_V_data_1_payload_A[9]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => p_reg_reg_n_90,
      I1 => \^co\(0),
      O => p_reg_reg_6
    );
\B_V_data_1_payload_A_reg[14]_i_5\: unisim.vcomponents.CARRY4
     port map (
      CI => \B_V_data_1_payload_A_reg[14]_i_7_n_0\,
      CO(3 downto 2) => \NLW_B_V_data_1_payload_A_reg[14]_i_5_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \^co\(0),
      CO(0) => \B_V_data_1_payload_A_reg[14]_i_5_n_3\,
      CYINIT => '0',
      DI(3 downto 2) => B"00",
      DI(1) => \B_V_data_1_payload_A[14]_i_8_n_0\,
      DI(0) => \B_V_data_1_payload_A[14]_i_9_n_0\,
      O(3 downto 0) => \NLW_B_V_data_1_payload_A_reg[14]_i_5_O_UNCONNECTED\(3 downto 0),
      S(3 downto 2) => B"00",
      S(1) => \B_V_data_1_payload_A[14]_i_10_n_0\,
      S(0) => \B_V_data_1_payload_A[14]_i_11_n_0\
    );
\B_V_data_1_payload_A_reg[14]_i_7\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \B_V_data_1_payload_A_reg[14]_i_7_n_0\,
      CO(2) => \B_V_data_1_payload_A_reg[14]_i_7_n_1\,
      CO(1) => \B_V_data_1_payload_A_reg[14]_i_7_n_2\,
      CO(0) => \B_V_data_1_payload_A_reg[14]_i_7_n_3\,
      CYINIT => '0',
      DI(3) => \B_V_data_1_payload_A[14]_i_12_n_0\,
      DI(2) => \B_V_data_1_payload_A[14]_i_13_n_0\,
      DI(1) => \B_V_data_1_payload_A[14]_i_14_n_0\,
      DI(0) => \B_V_data_1_payload_A[14]_i_15_n_0\,
      O(3 downto 0) => \NLW_B_V_data_1_payload_A_reg[14]_i_7_O_UNCONNECTED\(3 downto 0),
      S(3) => \B_V_data_1_payload_A[14]_i_16_n_0\,
      S(2) => \B_V_data_1_payload_A[14]_i_17_n_0\,
      S(1) => \B_V_data_1_payload_A[14]_i_18_n_0\,
      S(0) => \B_V_data_1_payload_A[14]_i_19_n_0\
    );
p_reg_reg: unisim.vcomponents.DSP48E1
    generic map(
      ACASCREG => 1,
      ADREG => 1,
      ALUMODEREG => 0,
      AREG => 1,
      AUTORESET_PATDET => "NO_RESET",
      A_INPUT => "DIRECT",
      BCASCREG => 2,
      BREG => 2,
      B_INPUT => "DIRECT",
      CARRYINREG => 0,
      CARRYINSELREG => 0,
      CREG => 1,
      DREG => 1,
      INMODEREG => 0,
      MASK => X"3FFFFFFFFFFF",
      MREG => 1,
      OPMODEREG => 0,
      PATTERN => X"000000000000",
      PREG => 1,
      SEL_MASK => "MASK",
      SEL_PATTERN => "PATTERN",
      USE_DPORT => false,
      USE_MULT => "MULTIPLY",
      USE_PATTERN_DETECT => "NO_PATDET",
      USE_SIMD => "ONE48"
    )
        port map (
      A(29) => A(15),
      A(28) => A(15),
      A(27) => A(15),
      A(26) => A(15),
      A(25) => A(15),
      A(24) => A(15),
      A(23) => A(15),
      A(22) => A(15),
      A(21) => A(15),
      A(20) => A(15),
      A(19) => A(15),
      A(18) => A(15),
      A(17) => A(15),
      A(16) => A(15),
      A(15 downto 0) => A(15 downto 0),
      ACIN(29 downto 0) => B"000000000000000000000000000000",
      ACOUT(29 downto 0) => NLW_p_reg_reg_ACOUT_UNCONNECTED(29 downto 0),
      ALUMODE(3 downto 0) => B"0011",
      B(17) => B(15),
      B(16) => B(15),
      B(15 downto 0) => B(15 downto 0),
      BCIN(17 downto 0) => B"000000000000000000",
      BCOUT(17 downto 0) => NLW_p_reg_reg_BCOUT_UNCONNECTED(17 downto 0),
      C(47 downto 0) => B"111111111111111111111111111111111111111111111111",
      CARRYCASCIN => '0',
      CARRYCASCOUT => NLW_p_reg_reg_CARRYCASCOUT_UNCONNECTED,
      CARRYIN => '0',
      CARRYINSEL(2 downto 0) => B"000",
      CARRYOUT(3 downto 0) => NLW_p_reg_reg_CARRYOUT_UNCONNECTED(3 downto 0),
      CEA1 => '0',
      CEA2 => ap_block_pp0_stage0_11001,
      CEAD => '0',
      CEALUMODE => '0',
      CEB1 => p_reg_reg_16,
      CEB2 => ap_block_pp0_stage0_11001,
      CEC => '0',
      CECARRYIN => '0',
      CECTRL => '0',
      CED => '0',
      CEINMODE => '0',
      CEM => ap_block_pp0_stage0_11001,
      CEP => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D(24 downto 0) => B"0000000000000000000000000",
      INMODE(4 downto 0) => B"00000",
      MULTSIGNIN => '0',
      MULTSIGNOUT => NLW_p_reg_reg_MULTSIGNOUT_UNCONNECTED,
      OPMODE(6 downto 0) => B"0010101",
      OVERFLOW => NLW_p_reg_reg_OVERFLOW_UNCONNECTED,
      P(47 downto 32) => NLW_p_reg_reg_P_UNCONNECTED(47 downto 32),
      P(31) => \^p\(1),
      P(30 downto 22) => tmp_2_fu_426_p4(9 downto 1),
      P(21) => \^p\(0),
      P(20) => p_reg_reg_n_85,
      P(19) => p_reg_reg_n_86,
      P(18) => p_reg_reg_n_87,
      P(17) => p_reg_reg_n_88,
      P(16) => p_reg_reg_n_89,
      P(15) => p_reg_reg_n_90,
      P(14) => p_reg_reg_n_91,
      P(13) => p_reg_reg_n_92,
      P(12) => p_reg_reg_n_93,
      P(11) => p_reg_reg_n_94,
      P(10) => p_reg_reg_n_95,
      P(9) => p_reg_reg_n_96,
      P(8) => p_reg_reg_n_97,
      P(7) => p_reg_reg_n_98,
      P(6) => p_reg_reg_n_99,
      P(5) => p_reg_reg_n_100,
      P(4) => p_reg_reg_n_101,
      P(3) => p_reg_reg_n_102,
      P(2) => p_reg_reg_n_103,
      P(1) => p_reg_reg_n_104,
      P(0) => p_reg_reg_n_105,
      PATTERNBDETECT => NLW_p_reg_reg_PATTERNBDETECT_UNCONNECTED,
      PATTERNDETECT => NLW_p_reg_reg_PATTERNDETECT_UNCONNECTED,
      PCIN(47 downto 0) => PCOUT(47 downto 0),
      PCOUT(47 downto 0) => NLW_p_reg_reg_PCOUT_UNCONNECTED(47 downto 0),
      RSTA => '0',
      RSTALLCARRYIN => '0',
      RSTALUMODE => '0',
      RSTB => '0',
      RSTC => '0',
      RSTCTRL => '0',
      RSTD => '0',
      RSTINMODE => '0',
      RSTM => '0',
      RSTP => '0',
      UNDERFLOW => NLW_p_reg_reg_UNDERFLOW_UNCONNECTED
    );
term1_reg_518_reg_i_101: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(10),
      I1 => sum_i_reg(11),
      O => \sum_i_reg[26]\(3)
    );
term1_reg_518_reg_i_102: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(9),
      I1 => sum_i_reg(10),
      O => \sum_i_reg[26]\(2)
    );
term1_reg_518_reg_i_103: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(8),
      I1 => sum_i_reg(9),
      O => \sum_i_reg[26]\(1)
    );
term1_reg_518_reg_i_104: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(7),
      I1 => sum_i_reg(8),
      O => \sum_i_reg[26]\(0)
    );
term1_reg_518_reg_i_105: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(6),
      I1 => sum_i_reg(7),
      O => \sum_i_reg[22]\(3)
    );
term1_reg_518_reg_i_106: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(5),
      I1 => sum_i_reg(6),
      O => \sum_i_reg[22]\(2)
    );
term1_reg_518_reg_i_107: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(4),
      I1 => sum_i_reg(5),
      O => \sum_i_reg[22]\(1)
    );
term1_reg_518_reg_i_108: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(3),
      I1 => sum_i_reg(4),
      O => \sum_i_reg[22]\(0)
    );
term1_reg_518_reg_i_110: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(2),
      I1 => sum_i_reg(3),
      O => S(2)
    );
term1_reg_518_reg_i_111: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(1),
      I1 => sum_i_reg(2),
      O => S(1)
    );
term1_reg_518_reg_i_112: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(0),
      I1 => sum_i_reg(1),
      O => S(0)
    );
term1_reg_518_reg_i_123: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_63_n_0,
      CO(3) => term1_reg_518_reg_i_123_n_0,
      CO(2) => term1_reg_518_reg_i_123_n_1,
      CO(1) => term1_reg_518_reg_i_123_n_2,
      CO(0) => term1_reg_518_reg_i_123_n_3,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln49_fu_218_p2(24 downto 21),
      S(3 downto 0) => \out\(24 downto 21)
    );
term1_reg_518_reg_i_124: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(14),
      I1 => sum_q_reg(15),
      O => \sum_q_reg[30]\(3)
    );
term1_reg_518_reg_i_125: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(13),
      I1 => sum_q_reg(14),
      O => \sum_q_reg[30]\(2)
    );
term1_reg_518_reg_i_126: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(12),
      I1 => sum_q_reg(13),
      O => \sum_q_reg[30]\(1)
    );
term1_reg_518_reg_i_127: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(11),
      I1 => sum_q_reg(12),
      O => \sum_q_reg[30]\(0)
    );
term1_reg_518_reg_i_133: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(14),
      I1 => sum_i_reg(15),
      O => \sum_i_reg[30]\(3)
    );
term1_reg_518_reg_i_134: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(13),
      I1 => sum_i_reg(14),
      O => \sum_i_reg[30]\(2)
    );
term1_reg_518_reg_i_135: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(12),
      I1 => sum_i_reg(13),
      O => \sum_i_reg[30]\(1)
    );
term1_reg_518_reg_i_136: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_i_reg(11),
      I1 => sum_i_reg(12),
      O => \sum_i_reg[30]\(0)
    );
term1_reg_518_reg_i_36: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00008000"
    )
        port map (
      I0 => add_ln49_fu_218_p2(6),
      I1 => add_ln49_fu_218_p2(7),
      I2 => add_ln49_fu_218_p2(4),
      I3 => add_ln49_fu_218_p2(5),
      I4 => term1_reg_518_reg_i_56_n_0,
      O => \counter_reg[8]\
    );
term1_reg_518_reg_i_37: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000001"
    )
        port map (
      I0 => add_ln49_fu_218_p2(12),
      I1 => add_ln49_fu_218_p2(13),
      I2 => add_ln49_fu_218_p2(14),
      I3 => add_ln49_fu_218_p2(15),
      I4 => term1_reg_518_reg_i_59_n_0,
      O => \counter_reg[12]\
    );
term1_reg_518_reg_i_38: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000001"
    )
        port map (
      I0 => add_ln49_fu_218_p2(28),
      I1 => add_ln49_fu_218_p2(29),
      I2 => add_ln49_fu_218_p2(31),
      I3 => add_ln49_fu_218_p2(30),
      I4 => term1_reg_518_reg_i_62_n_0,
      O => \counter_reg[28]\
    );
term1_reg_518_reg_i_39: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFEFFFF"
    )
        port map (
      I0 => add_ln49_fu_218_p2(16),
      I1 => add_ln49_fu_218_p2(17),
      I2 => add_ln49_fu_218_p2(18),
      I3 => add_ln49_fu_218_p2(19),
      I4 => term1_reg_518_reg_i_64_n_0,
      O => \counter_reg[16]\
    );
term1_reg_518_reg_i_54: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_55_n_0,
      CO(3) => term1_reg_518_reg_i_54_n_0,
      CO(2) => term1_reg_518_reg_i_54_n_1,
      CO(1) => term1_reg_518_reg_i_54_n_2,
      CO(0) => term1_reg_518_reg_i_54_n_3,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln49_fu_218_p2(8 downto 5),
      S(3 downto 0) => \out\(8 downto 5)
    );
term1_reg_518_reg_i_55: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => term1_reg_518_reg_i_55_n_0,
      CO(2) => term1_reg_518_reg_i_55_n_1,
      CO(1) => term1_reg_518_reg_i_55_n_2,
      CO(0) => term1_reg_518_reg_i_55_n_3,
      CYINIT => \out\(0),
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln49_fu_218_p2(4 downto 1),
      S(3 downto 0) => \out\(4 downto 1)
    );
term1_reg_518_reg_i_56: unisim.vcomponents.LUT4
    generic map(
      INIT => X"DFFF"
    )
        port map (
      I0 => add_ln49_fu_218_p2(1),
      I1 => \out\(0),
      I2 => add_ln49_fu_218_p2(3),
      I3 => add_ln49_fu_218_p2(2),
      O => term1_reg_518_reg_i_56_n_0
    );
term1_reg_518_reg_i_57: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_54_n_0,
      CO(3) => term1_reg_518_reg_i_57_n_0,
      CO(2) => term1_reg_518_reg_i_57_n_1,
      CO(1) => term1_reg_518_reg_i_57_n_2,
      CO(0) => term1_reg_518_reg_i_57_n_3,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln49_fu_218_p2(12 downto 9),
      S(3 downto 0) => \out\(12 downto 9)
    );
term1_reg_518_reg_i_58: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_57_n_0,
      CO(3) => term1_reg_518_reg_i_58_n_0,
      CO(2) => term1_reg_518_reg_i_58_n_1,
      CO(1) => term1_reg_518_reg_i_58_n_2,
      CO(0) => term1_reg_518_reg_i_58_n_3,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln49_fu_218_p2(16 downto 13),
      S(3 downto 0) => \out\(16 downto 13)
    );
term1_reg_518_reg_i_59: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => add_ln49_fu_218_p2(9),
      I1 => add_ln49_fu_218_p2(8),
      I2 => add_ln49_fu_218_p2(11),
      I3 => add_ln49_fu_218_p2(10),
      O => term1_reg_518_reg_i_59_n_0
    );
term1_reg_518_reg_i_60: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_123_n_0,
      CO(3) => term1_reg_518_reg_i_60_n_0,
      CO(2) => term1_reg_518_reg_i_60_n_1,
      CO(1) => term1_reg_518_reg_i_60_n_2,
      CO(0) => term1_reg_518_reg_i_60_n_3,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln49_fu_218_p2(28 downto 25),
      S(3 downto 0) => \out\(28 downto 25)
    );
term1_reg_518_reg_i_61: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_60_n_0,
      CO(3 downto 2) => NLW_term1_reg_518_reg_i_61_CO_UNCONNECTED(3 downto 2),
      CO(1) => term1_reg_518_reg_i_61_n_2,
      CO(0) => term1_reg_518_reg_i_61_n_3,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => NLW_term1_reg_518_reg_i_61_O_UNCONNECTED(3),
      O(2 downto 0) => add_ln49_fu_218_p2(31 downto 29),
      S(3) => '0',
      S(2 downto 0) => \out\(31 downto 29)
    );
term1_reg_518_reg_i_62: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => add_ln49_fu_218_p2(25),
      I1 => add_ln49_fu_218_p2(24),
      I2 => add_ln49_fu_218_p2(27),
      I3 => add_ln49_fu_218_p2(26),
      O => term1_reg_518_reg_i_62_n_0
    );
term1_reg_518_reg_i_63: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_58_n_0,
      CO(3) => term1_reg_518_reg_i_63_n_0,
      CO(2) => term1_reg_518_reg_i_63_n_1,
      CO(1) => term1_reg_518_reg_i_63_n_2,
      CO(0) => term1_reg_518_reg_i_63_n_3,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln49_fu_218_p2(20 downto 17),
      S(3 downto 0) => \out\(20 downto 17)
    );
term1_reg_518_reg_i_64: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0001"
    )
        port map (
      I0 => add_ln49_fu_218_p2(23),
      I1 => add_ln49_fu_218_p2(22),
      I2 => add_ln49_fu_218_p2(21),
      I3 => add_ln49_fu_218_p2(20),
      O => term1_reg_518_reg_i_64_n_0
    );
term1_reg_518_reg_i_72: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(10),
      I1 => sum_q_reg(11),
      O => \sum_q_reg[26]\(3)
    );
term1_reg_518_reg_i_73: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(9),
      I1 => sum_q_reg(10),
      O => \sum_q_reg[26]\(2)
    );
term1_reg_518_reg_i_74: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(8),
      I1 => sum_q_reg(9),
      O => \sum_q_reg[26]\(1)
    );
term1_reg_518_reg_i_75: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(7),
      I1 => sum_q_reg(8),
      O => \sum_q_reg[26]\(0)
    );
term1_reg_518_reg_i_76: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(6),
      I1 => sum_q_reg(7),
      O => \sum_q_reg[22]\(3)
    );
term1_reg_518_reg_i_77: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(5),
      I1 => sum_q_reg(6),
      O => \sum_q_reg[22]\(2)
    );
term1_reg_518_reg_i_78: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(4),
      I1 => sum_q_reg(5),
      O => \sum_q_reg[22]\(1)
    );
term1_reg_518_reg_i_79: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(3),
      I1 => sum_q_reg(4),
      O => \sum_q_reg[22]\(0)
    );
term1_reg_518_reg_i_81: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(2),
      I1 => sum_q_reg(3),
      O => \sum_q_reg[18]\(2)
    );
term1_reg_518_reg_i_82: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(1),
      I1 => sum_q_reg(2),
      O => \sum_q_reg[18]\(1)
    );
term1_reg_518_reg_i_83: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_q_reg(0),
      I1 => sum_q_reg(1),
      O => \sum_q_reg[18]\(0)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_fsk_discriminator_0_0_fsk_discriminator_regslice_both is
  port (
    \B_V_data_1_state_reg[1]_0\ : out STD_LOGIC;
    ap_rst_n_inv : out STD_LOGIC;
    in_stream_TVALID_int_regslice : out STD_LOGIC;
    B : out STD_LOGIC_VECTOR ( 15 downto 0 );
    A : out STD_LOGIC_VECTOR ( 15 downto 0 );
    O : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[7]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[11]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[15]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[19]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[23]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[27]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[30]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[3]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[7]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[11]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[15]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[19]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[23]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[27]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[30]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    \B_V_data_1_state_reg[1]_1\ : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    sum_i_reg : in STD_LOGIC_VECTOR ( 31 downto 0 );
    sum_q_reg : in STD_LOGIC_VECTOR ( 31 downto 0 );
    S : in STD_LOGIC_VECTOR ( 2 downto 0 );
    p_reg_reg : in STD_LOGIC_VECTOR ( 3 downto 0 );
    p_reg_reg_0 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    term1_reg_518_reg_i_99_0 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    p_reg_reg_1 : in STD_LOGIC_VECTOR ( 2 downto 0 );
    p_reg_reg_2 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    p_reg_reg_3 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    term1_reg_518_reg_i_70_0 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of system_fsk_discriminator_0_0_fsk_discriminator_regslice_both : entity is "fsk_discriminator_regslice_both";
end system_fsk_discriminator_0_0_fsk_discriminator_regslice_both;

architecture STRUCTURE of system_fsk_discriminator_0_0_fsk_discriminator_regslice_both is
  signal B_V_data_1_load_B : STD_LOGIC;
  signal \B_V_data_1_payload_A[31]_i_1_n_0\ : STD_LOGIC;
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
  signal \B_V_data_1_sel_rd_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_rd_reg_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal B_V_data_1_sel_wr_i_1_n_0 : STD_LOGIC;
  signal \B_V_data_1_state[0]_i_1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state[1]_i_2_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[1]_0\ : STD_LOGIC;
  signal \^ap_rst_n_inv\ : STD_LOGIC;
  signal icmp_ln23_1_fu_321_p2 : STD_LOGIC;
  signal icmp_ln23_fu_267_p2 : STD_LOGIC;
  signal \^in_stream_tvalid_int_regslice\ : STD_LOGIC;
  signal \sum_i[0]_i_2_n_0\ : STD_LOGIC;
  signal \sum_i[0]_i_3_n_0\ : STD_LOGIC;
  signal \sum_i[0]_i_4_n_0\ : STD_LOGIC;
  signal \sum_i[0]_i_5_n_0\ : STD_LOGIC;
  signal \sum_i[12]_i_2_n_0\ : STD_LOGIC;
  signal \sum_i[12]_i_3_n_0\ : STD_LOGIC;
  signal \sum_i[12]_i_4_n_0\ : STD_LOGIC;
  signal \sum_i[12]_i_5_n_0\ : STD_LOGIC;
  signal \sum_i[16]_i_2_n_0\ : STD_LOGIC;
  signal \sum_i[16]_i_3_n_0\ : STD_LOGIC;
  signal \sum_i[16]_i_4_n_0\ : STD_LOGIC;
  signal \sum_i[16]_i_5_n_0\ : STD_LOGIC;
  signal \sum_i[20]_i_2_n_0\ : STD_LOGIC;
  signal \sum_i[20]_i_3_n_0\ : STD_LOGIC;
  signal \sum_i[20]_i_4_n_0\ : STD_LOGIC;
  signal \sum_i[20]_i_5_n_0\ : STD_LOGIC;
  signal \sum_i[24]_i_2_n_0\ : STD_LOGIC;
  signal \sum_i[24]_i_3_n_0\ : STD_LOGIC;
  signal \sum_i[24]_i_4_n_0\ : STD_LOGIC;
  signal \sum_i[24]_i_5_n_0\ : STD_LOGIC;
  signal \sum_i[28]_i_2_n_0\ : STD_LOGIC;
  signal \sum_i[28]_i_3_n_0\ : STD_LOGIC;
  signal \sum_i[28]_i_4_n_0\ : STD_LOGIC;
  signal \sum_i[28]_i_5_n_0\ : STD_LOGIC;
  signal \sum_i[4]_i_2_n_0\ : STD_LOGIC;
  signal \sum_i[4]_i_3_n_0\ : STD_LOGIC;
  signal \sum_i[4]_i_4_n_0\ : STD_LOGIC;
  signal \sum_i[4]_i_5_n_0\ : STD_LOGIC;
  signal \sum_i[8]_i_2_n_0\ : STD_LOGIC;
  signal \sum_i[8]_i_3_n_0\ : STD_LOGIC;
  signal \sum_i[8]_i_4_n_0\ : STD_LOGIC;
  signal \sum_i[8]_i_5_n_0\ : STD_LOGIC;
  signal \sum_i_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \sum_i_reg[0]_i_1_n_1\ : STD_LOGIC;
  signal \sum_i_reg[0]_i_1_n_2\ : STD_LOGIC;
  signal \sum_i_reg[0]_i_1_n_3\ : STD_LOGIC;
  signal \sum_i_reg[12]_i_1_n_0\ : STD_LOGIC;
  signal \sum_i_reg[12]_i_1_n_1\ : STD_LOGIC;
  signal \sum_i_reg[12]_i_1_n_2\ : STD_LOGIC;
  signal \sum_i_reg[12]_i_1_n_3\ : STD_LOGIC;
  signal \sum_i_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \sum_i_reg[16]_i_1_n_1\ : STD_LOGIC;
  signal \sum_i_reg[16]_i_1_n_2\ : STD_LOGIC;
  signal \sum_i_reg[16]_i_1_n_3\ : STD_LOGIC;
  signal \sum_i_reg[20]_i_1_n_0\ : STD_LOGIC;
  signal \sum_i_reg[20]_i_1_n_1\ : STD_LOGIC;
  signal \sum_i_reg[20]_i_1_n_2\ : STD_LOGIC;
  signal \sum_i_reg[20]_i_1_n_3\ : STD_LOGIC;
  signal \sum_i_reg[24]_i_1_n_0\ : STD_LOGIC;
  signal \sum_i_reg[24]_i_1_n_1\ : STD_LOGIC;
  signal \sum_i_reg[24]_i_1_n_2\ : STD_LOGIC;
  signal \sum_i_reg[24]_i_1_n_3\ : STD_LOGIC;
  signal \sum_i_reg[28]_i_1_n_1\ : STD_LOGIC;
  signal \sum_i_reg[28]_i_1_n_2\ : STD_LOGIC;
  signal \sum_i_reg[28]_i_1_n_3\ : STD_LOGIC;
  signal \sum_i_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \sum_i_reg[4]_i_1_n_1\ : STD_LOGIC;
  signal \sum_i_reg[4]_i_1_n_2\ : STD_LOGIC;
  signal \sum_i_reg[4]_i_1_n_3\ : STD_LOGIC;
  signal \sum_i_reg[8]_i_1_n_0\ : STD_LOGIC;
  signal \sum_i_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \sum_i_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \sum_i_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal \sum_q[0]_i_2_n_0\ : STD_LOGIC;
  signal \sum_q[0]_i_3_n_0\ : STD_LOGIC;
  signal \sum_q[0]_i_4_n_0\ : STD_LOGIC;
  signal \sum_q[0]_i_5_n_0\ : STD_LOGIC;
  signal \sum_q[12]_i_2_n_0\ : STD_LOGIC;
  signal \sum_q[12]_i_3_n_0\ : STD_LOGIC;
  signal \sum_q[12]_i_4_n_0\ : STD_LOGIC;
  signal \sum_q[12]_i_5_n_0\ : STD_LOGIC;
  signal \sum_q[16]_i_2_n_0\ : STD_LOGIC;
  signal \sum_q[16]_i_3_n_0\ : STD_LOGIC;
  signal \sum_q[16]_i_4_n_0\ : STD_LOGIC;
  signal \sum_q[16]_i_5_n_0\ : STD_LOGIC;
  signal \sum_q[20]_i_2_n_0\ : STD_LOGIC;
  signal \sum_q[20]_i_3_n_0\ : STD_LOGIC;
  signal \sum_q[20]_i_4_n_0\ : STD_LOGIC;
  signal \sum_q[20]_i_5_n_0\ : STD_LOGIC;
  signal \sum_q[24]_i_2_n_0\ : STD_LOGIC;
  signal \sum_q[24]_i_3_n_0\ : STD_LOGIC;
  signal \sum_q[24]_i_4_n_0\ : STD_LOGIC;
  signal \sum_q[24]_i_5_n_0\ : STD_LOGIC;
  signal \sum_q[28]_i_2_n_0\ : STD_LOGIC;
  signal \sum_q[28]_i_3_n_0\ : STD_LOGIC;
  signal \sum_q[28]_i_4_n_0\ : STD_LOGIC;
  signal \sum_q[28]_i_5_n_0\ : STD_LOGIC;
  signal \sum_q[4]_i_2_n_0\ : STD_LOGIC;
  signal \sum_q[4]_i_3_n_0\ : STD_LOGIC;
  signal \sum_q[4]_i_4_n_0\ : STD_LOGIC;
  signal \sum_q[4]_i_5_n_0\ : STD_LOGIC;
  signal \sum_q[8]_i_2_n_0\ : STD_LOGIC;
  signal \sum_q[8]_i_3_n_0\ : STD_LOGIC;
  signal \sum_q[8]_i_4_n_0\ : STD_LOGIC;
  signal \sum_q[8]_i_5_n_0\ : STD_LOGIC;
  signal \sum_q_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \sum_q_reg[0]_i_1_n_1\ : STD_LOGIC;
  signal \sum_q_reg[0]_i_1_n_2\ : STD_LOGIC;
  signal \sum_q_reg[0]_i_1_n_3\ : STD_LOGIC;
  signal \sum_q_reg[12]_i_1_n_0\ : STD_LOGIC;
  signal \sum_q_reg[12]_i_1_n_1\ : STD_LOGIC;
  signal \sum_q_reg[12]_i_1_n_2\ : STD_LOGIC;
  signal \sum_q_reg[12]_i_1_n_3\ : STD_LOGIC;
  signal \sum_q_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \sum_q_reg[16]_i_1_n_1\ : STD_LOGIC;
  signal \sum_q_reg[16]_i_1_n_2\ : STD_LOGIC;
  signal \sum_q_reg[16]_i_1_n_3\ : STD_LOGIC;
  signal \sum_q_reg[20]_i_1_n_0\ : STD_LOGIC;
  signal \sum_q_reg[20]_i_1_n_1\ : STD_LOGIC;
  signal \sum_q_reg[20]_i_1_n_2\ : STD_LOGIC;
  signal \sum_q_reg[20]_i_1_n_3\ : STD_LOGIC;
  signal \sum_q_reg[24]_i_1_n_0\ : STD_LOGIC;
  signal \sum_q_reg[24]_i_1_n_1\ : STD_LOGIC;
  signal \sum_q_reg[24]_i_1_n_2\ : STD_LOGIC;
  signal \sum_q_reg[24]_i_1_n_3\ : STD_LOGIC;
  signal \sum_q_reg[28]_i_1_n_1\ : STD_LOGIC;
  signal \sum_q_reg[28]_i_1_n_2\ : STD_LOGIC;
  signal \sum_q_reg[28]_i_1_n_3\ : STD_LOGIC;
  signal \sum_q_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \sum_q_reg[4]_i_1_n_1\ : STD_LOGIC;
  signal \sum_q_reg[4]_i_1_n_2\ : STD_LOGIC;
  signal \sum_q_reg[4]_i_1_n_3\ : STD_LOGIC;
  signal \sum_q_reg[8]_i_1_n_0\ : STD_LOGIC;
  signal \sum_q_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \sum_q_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \sum_q_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal term1_reg_518_reg_i_100_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_109_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_113_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_114_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_115_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_116_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_117_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_118_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_118_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_118_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_118_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_119_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_120_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_121_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_122_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_128_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_128_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_128_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_128_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_129_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_130_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_131_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_132_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_137_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_137_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_137_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_137_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_138_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_139_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_140_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_141_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_142_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_143_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_144_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_145_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_146_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_147_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_148_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_149_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_40_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_40_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_40_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_41_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_42_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_42_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_42_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_42_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_42_n_6 : STD_LOGIC;
  signal term1_reg_518_reg_i_42_n_7 : STD_LOGIC;
  signal term1_reg_518_reg_i_43_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_43_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_43_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_43_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_43_n_4 : STD_LOGIC;
  signal term1_reg_518_reg_i_43_n_5 : STD_LOGIC;
  signal term1_reg_518_reg_i_43_n_6 : STD_LOGIC;
  signal term1_reg_518_reg_i_43_n_7 : STD_LOGIC;
  signal term1_reg_518_reg_i_44_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_44_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_44_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_44_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_44_n_4 : STD_LOGIC;
  signal term1_reg_518_reg_i_44_n_5 : STD_LOGIC;
  signal term1_reg_518_reg_i_44_n_6 : STD_LOGIC;
  signal term1_reg_518_reg_i_44_n_7 : STD_LOGIC;
  signal term1_reg_518_reg_i_45_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_45_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_45_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_45_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_45_n_4 : STD_LOGIC;
  signal term1_reg_518_reg_i_45_n_5 : STD_LOGIC;
  signal term1_reg_518_reg_i_45_n_6 : STD_LOGIC;
  signal term1_reg_518_reg_i_45_n_7 : STD_LOGIC;
  signal term1_reg_518_reg_i_46_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_46_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_46_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_46_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_46_n_4 : STD_LOGIC;
  signal term1_reg_518_reg_i_47_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_47_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_47_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_48_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_49_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_49_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_49_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_49_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_49_n_6 : STD_LOGIC;
  signal term1_reg_518_reg_i_49_n_7 : STD_LOGIC;
  signal term1_reg_518_reg_i_50_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_50_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_50_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_50_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_50_n_4 : STD_LOGIC;
  signal term1_reg_518_reg_i_50_n_5 : STD_LOGIC;
  signal term1_reg_518_reg_i_50_n_6 : STD_LOGIC;
  signal term1_reg_518_reg_i_50_n_7 : STD_LOGIC;
  signal term1_reg_518_reg_i_51_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_51_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_51_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_51_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_51_n_4 : STD_LOGIC;
  signal term1_reg_518_reg_i_51_n_5 : STD_LOGIC;
  signal term1_reg_518_reg_i_51_n_6 : STD_LOGIC;
  signal term1_reg_518_reg_i_51_n_7 : STD_LOGIC;
  signal term1_reg_518_reg_i_52_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_52_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_52_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_52_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_52_n_4 : STD_LOGIC;
  signal term1_reg_518_reg_i_52_n_5 : STD_LOGIC;
  signal term1_reg_518_reg_i_52_n_6 : STD_LOGIC;
  signal term1_reg_518_reg_i_52_n_7 : STD_LOGIC;
  signal term1_reg_518_reg_i_53_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_53_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_53_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_53_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_53_n_4 : STD_LOGIC;
  signal term1_reg_518_reg_i_65_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_66_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_67_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_68_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_68_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_68_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_69_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_70_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_71_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_80_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_84_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_85_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_86_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_87_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_88_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_89_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_89_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_89_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_89_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_90_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_91_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_92_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_93_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_94_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_95_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_96_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_97_n_1 : STD_LOGIC;
  signal term1_reg_518_reg_i_97_n_2 : STD_LOGIC;
  signal term1_reg_518_reg_i_97_n_3 : STD_LOGIC;
  signal term1_reg_518_reg_i_98_n_0 : STD_LOGIC;
  signal term1_reg_518_reg_i_99_n_0 : STD_LOGIC;
  signal tmp_1_fu_305_p4 : STD_LOGIC_VECTOR ( 5 downto 0 );
  signal tmp_fu_251_p4 : STD_LOGIC_VECTOR ( 5 downto 0 );
  signal \NLW_sum_i_reg[28]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_sum_q_reg[28]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal NLW_term1_reg_518_reg_i_118_O_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_term1_reg_518_reg_i_128_O_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_term1_reg_518_reg_i_137_O_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_term1_reg_518_reg_i_40_O_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_term1_reg_518_reg_i_46_O_UNCONNECTED : STD_LOGIC_VECTOR ( 2 downto 0 );
  signal NLW_term1_reg_518_reg_i_47_O_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_term1_reg_518_reg_i_53_O_UNCONNECTED : STD_LOGIC_VECTOR ( 2 downto 0 );
  signal NLW_term1_reg_518_reg_i_68_CO_UNCONNECTED : STD_LOGIC_VECTOR ( 3 to 3 );
  signal NLW_term1_reg_518_reg_i_89_O_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_term1_reg_518_reg_i_97_CO_UNCONNECTED : STD_LOGIC_VECTOR ( 3 to 3 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__0\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_2\ : label is "soft_lutpair7";
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \sum_i_reg[0]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_i_reg[12]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_i_reg[16]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_i_reg[20]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_i_reg[24]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_i_reg[28]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_i_reg[4]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_i_reg[8]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_q_reg[0]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_q_reg[12]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_q_reg[16]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_q_reg[20]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_q_reg[24]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_q_reg[28]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_q_reg[4]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_q_reg[8]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_118 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_128 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_137 : label is 35;
  attribute COMPARATOR_THRESHOLD : integer;
  attribute COMPARATOR_THRESHOLD of term1_reg_518_reg_i_40 : label is 11;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_42 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_43 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_44 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_45 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_46 : label is 35;
  attribute COMPARATOR_THRESHOLD of term1_reg_518_reg_i_47 : label is 11;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_49 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_50 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_51 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_52 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_53 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_68 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_89 : label is 35;
  attribute ADDER_THRESHOLD of term1_reg_518_reg_i_97 : label is 35;
begin
  \B_V_data_1_state_reg[1]_0\ <= \^b_v_data_1_state_reg[1]_0\;
  ap_rst_n_inv <= \^ap_rst_n_inv\;
  in_stream_TVALID_int_regslice <= \^in_stream_tvalid_int_regslice\;
\B_V_data_1_payload_A[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"0D"
    )
        port map (
      I0 => \^in_stream_tvalid_int_regslice\,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_payload_A[31]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(0),
      Q => \B_V_data_1_payload_A_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(10),
      Q => \B_V_data_1_payload_A_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(11),
      Q => \B_V_data_1_payload_A_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(12),
      Q => \B_V_data_1_payload_A_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(13),
      Q => \B_V_data_1_payload_A_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(14),
      Q => \B_V_data_1_payload_A_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(15),
      Q => \B_V_data_1_payload_A_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(16),
      Q => \B_V_data_1_payload_A_reg_n_0_[16]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(17),
      Q => \B_V_data_1_payload_A_reg_n_0_[17]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(18),
      Q => \B_V_data_1_payload_A_reg_n_0_[18]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(19),
      Q => \B_V_data_1_payload_A_reg_n_0_[19]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(1),
      Q => \B_V_data_1_payload_A_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(20),
      Q => \B_V_data_1_payload_A_reg_n_0_[20]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(21),
      Q => \B_V_data_1_payload_A_reg_n_0_[21]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(22),
      Q => \B_V_data_1_payload_A_reg_n_0_[22]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(23),
      Q => \B_V_data_1_payload_A_reg_n_0_[23]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(24),
      Q => \B_V_data_1_payload_A_reg_n_0_[24]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(25),
      Q => \B_V_data_1_payload_A_reg_n_0_[25]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(26),
      Q => \B_V_data_1_payload_A_reg_n_0_[26]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(27),
      Q => \B_V_data_1_payload_A_reg_n_0_[27]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(28),
      Q => \B_V_data_1_payload_A_reg_n_0_[28]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(29),
      Q => \B_V_data_1_payload_A_reg_n_0_[29]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(2),
      Q => \B_V_data_1_payload_A_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(30),
      Q => \B_V_data_1_payload_A_reg_n_0_[30]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(31),
      Q => \B_V_data_1_payload_A_reg_n_0_[31]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(3),
      Q => \B_V_data_1_payload_A_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(4),
      Q => \B_V_data_1_payload_A_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(5),
      Q => \B_V_data_1_payload_A_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(6),
      Q => \B_V_data_1_payload_A_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(7),
      Q => \B_V_data_1_payload_A_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(8),
      Q => \B_V_data_1_payload_A_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[31]_i_1_n_0\,
      D => in_stream_TDATA(9),
      Q => \B_V_data_1_payload_A_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_payload_B[31]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A2"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \^in_stream_tvalid_int_regslice\,
      I2 => \^b_v_data_1_state_reg[1]_0\,
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
\B_V_data_1_payload_B_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(16),
      Q => \B_V_data_1_payload_B_reg_n_0_[16]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(17),
      Q => \B_V_data_1_payload_B_reg_n_0_[17]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(18),
      Q => \B_V_data_1_payload_B_reg_n_0_[18]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(19),
      Q => \B_V_data_1_payload_B_reg_n_0_[19]\,
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
\B_V_data_1_payload_B_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(20),
      Q => \B_V_data_1_payload_B_reg_n_0_[20]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(21),
      Q => \B_V_data_1_payload_B_reg_n_0_[21]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(22),
      Q => \B_V_data_1_payload_B_reg_n_0_[22]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(23),
      Q => \B_V_data_1_payload_B_reg_n_0_[23]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(24),
      Q => \B_V_data_1_payload_B_reg_n_0_[24]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(25),
      Q => \B_V_data_1_payload_B_reg_n_0_[25]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(26),
      Q => \B_V_data_1_payload_B_reg_n_0_[26]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(27),
      Q => \B_V_data_1_payload_B_reg_n_0_[27]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(28),
      Q => \B_V_data_1_payload_B_reg_n_0_[28]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(29),
      Q => \B_V_data_1_payload_B_reg_n_0_[29]\,
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
\B_V_data_1_payload_B_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(30),
      Q => \B_V_data_1_payload_B_reg_n_0_[30]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(31),
      Q => \B_V_data_1_payload_B_reg_n_0_[31]\,
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
\B_V_data_1_sel_rd_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B4"
    )
        port map (
      I0 => \B_V_data_1_state_reg[1]_1\,
      I1 => \^in_stream_tvalid_int_regslice\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => \B_V_data_1_sel_rd_i_1__0_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__0_n_0\,
      Q => B_V_data_1_sel_rd_reg_n_0,
      R => \^ap_rst_n_inv\
    );
B_V_data_1_sel_wr_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => in_stream_TVALID,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => B_V_data_1_sel_wr,
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
\B_V_data_1_state[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8AAA000"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg[1]_1\,
      I2 => in_stream_TVALID,
      I3 => \^b_v_data_1_state_reg[1]_0\,
      I4 => \^in_stream_tvalid_int_regslice\,
      O => \B_V_data_1_state[0]_i_1_n_0\
    );
\B_V_data_1_state[1]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => ap_rst_n,
      O => \^ap_rst_n_inv\
    );
\B_V_data_1_state[1]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"77F7"
    )
        port map (
      I0 => \B_V_data_1_state_reg[1]_1\,
      I1 => \^in_stream_tvalid_int_regslice\,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => in_stream_TVALID,
      O => \B_V_data_1_state[1]_i_2_n_0\
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1_n_0\,
      Q => \^in_stream_tvalid_int_regslice\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[1]_i_2_n_0\,
      Q => \^b_v_data_1_state_reg[1]_0\,
      R => \^ap_rst_n_inv\
    );
\sum_i[0]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      I3 => sum_i_reg(3),
      O => \sum_i[0]_i_2_n_0\
    );
\sum_i[0]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I3 => sum_i_reg(2),
      O => \sum_i[0]_i_3_n_0\
    );
\sum_i[0]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      I3 => sum_i_reg(1),
      O => \sum_i[0]_i_4_n_0\
    );
\sum_i[0]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I3 => sum_i_reg(0),
      O => \sum_i[0]_i_5_n_0\
    );
\sum_i[12]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(15),
      O => \sum_i[12]_i_2_n_0\
    );
\sum_i[12]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I3 => sum_i_reg(14),
      O => \sum_i[12]_i_3_n_0\
    );
\sum_i[12]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      I3 => sum_i_reg(13),
      O => \sum_i[12]_i_4_n_0\
    );
\sum_i[12]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I3 => sum_i_reg(12),
      O => \sum_i[12]_i_5_n_0\
    );
\sum_i[16]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(19),
      O => \sum_i[16]_i_2_n_0\
    );
\sum_i[16]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(18),
      O => \sum_i[16]_i_3_n_0\
    );
\sum_i[16]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(17),
      O => \sum_i[16]_i_4_n_0\
    );
\sum_i[16]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(16),
      O => \sum_i[16]_i_5_n_0\
    );
\sum_i[20]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(23),
      O => \sum_i[20]_i_2_n_0\
    );
\sum_i[20]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(22),
      O => \sum_i[20]_i_3_n_0\
    );
\sum_i[20]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(21),
      O => \sum_i[20]_i_4_n_0\
    );
\sum_i[20]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(20),
      O => \sum_i[20]_i_5_n_0\
    );
\sum_i[24]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(27),
      O => \sum_i[24]_i_2_n_0\
    );
\sum_i[24]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(26),
      O => \sum_i[24]_i_3_n_0\
    );
\sum_i[24]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(25),
      O => \sum_i[24]_i_4_n_0\
    );
\sum_i[24]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(24),
      O => \sum_i[24]_i_5_n_0\
    );
\sum_i[28]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(31),
      O => \sum_i[28]_i_2_n_0\
    );
\sum_i[28]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(30),
      O => \sum_i[28]_i_3_n_0\
    );
\sum_i[28]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(29),
      O => \sum_i[28]_i_4_n_0\
    );
\sum_i[28]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(28),
      O => \sum_i[28]_i_5_n_0\
    );
\sum_i[4]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      I3 => sum_i_reg(7),
      O => \sum_i[4]_i_2_n_0\
    );
\sum_i[4]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I3 => sum_i_reg(6),
      O => \sum_i[4]_i_3_n_0\
    );
\sum_i[4]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      I3 => sum_i_reg(5),
      O => \sum_i[4]_i_4_n_0\
    );
\sum_i[4]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I3 => sum_i_reg(4),
      O => \sum_i[4]_i_5_n_0\
    );
\sum_i[8]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      I3 => sum_i_reg(11),
      O => \sum_i[8]_i_2_n_0\
    );
\sum_i[8]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I3 => sum_i_reg(10),
      O => \sum_i[8]_i_3_n_0\
    );
\sum_i[8]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      I3 => sum_i_reg(9),
      O => \sum_i[8]_i_4_n_0\
    );
\sum_i[8]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I3 => sum_i_reg(8),
      O => \sum_i[8]_i_5_n_0\
    );
\sum_i_reg[0]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \sum_i_reg[0]_i_1_n_0\,
      CO(2) => \sum_i_reg[0]_i_1_n_1\,
      CO(1) => \sum_i_reg[0]_i_1_n_2\,
      CO(0) => \sum_i_reg[0]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(3 downto 0),
      O(3 downto 0) => O(3 downto 0),
      S(3) => \sum_i[0]_i_2_n_0\,
      S(2) => \sum_i[0]_i_3_n_0\,
      S(1) => \sum_i[0]_i_4_n_0\,
      S(0) => \sum_i[0]_i_5_n_0\
    );
\sum_i_reg[12]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_i_reg[8]_i_1_n_0\,
      CO(3) => \sum_i_reg[12]_i_1_n_0\,
      CO(2) => \sum_i_reg[12]_i_1_n_1\,
      CO(1) => \sum_i_reg[12]_i_1_n_2\,
      CO(0) => \sum_i_reg[12]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(15 downto 12),
      O(3 downto 0) => \sum_i_reg[15]\(3 downto 0),
      S(3) => \sum_i[12]_i_2_n_0\,
      S(2) => \sum_i[12]_i_3_n_0\,
      S(1) => \sum_i[12]_i_4_n_0\,
      S(0) => \sum_i[12]_i_5_n_0\
    );
\sum_i_reg[16]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_i_reg[12]_i_1_n_0\,
      CO(3) => \sum_i_reg[16]_i_1_n_0\,
      CO(2) => \sum_i_reg[16]_i_1_n_1\,
      CO(1) => \sum_i_reg[16]_i_1_n_2\,
      CO(0) => \sum_i_reg[16]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(19 downto 16),
      O(3 downto 0) => \sum_i_reg[19]\(3 downto 0),
      S(3) => \sum_i[16]_i_2_n_0\,
      S(2) => \sum_i[16]_i_3_n_0\,
      S(1) => \sum_i[16]_i_4_n_0\,
      S(0) => \sum_i[16]_i_5_n_0\
    );
\sum_i_reg[20]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_i_reg[16]_i_1_n_0\,
      CO(3) => \sum_i_reg[20]_i_1_n_0\,
      CO(2) => \sum_i_reg[20]_i_1_n_1\,
      CO(1) => \sum_i_reg[20]_i_1_n_2\,
      CO(0) => \sum_i_reg[20]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(23 downto 20),
      O(3 downto 0) => \sum_i_reg[23]\(3 downto 0),
      S(3) => \sum_i[20]_i_2_n_0\,
      S(2) => \sum_i[20]_i_3_n_0\,
      S(1) => \sum_i[20]_i_4_n_0\,
      S(0) => \sum_i[20]_i_5_n_0\
    );
\sum_i_reg[24]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_i_reg[20]_i_1_n_0\,
      CO(3) => \sum_i_reg[24]_i_1_n_0\,
      CO(2) => \sum_i_reg[24]_i_1_n_1\,
      CO(1) => \sum_i_reg[24]_i_1_n_2\,
      CO(0) => \sum_i_reg[24]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(27 downto 24),
      O(3 downto 0) => \sum_i_reg[27]\(3 downto 0),
      S(3) => \sum_i[24]_i_2_n_0\,
      S(2) => \sum_i[24]_i_3_n_0\,
      S(1) => \sum_i[24]_i_4_n_0\,
      S(0) => \sum_i[24]_i_5_n_0\
    );
\sum_i_reg[28]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_i_reg[24]_i_1_n_0\,
      CO(3) => \NLW_sum_i_reg[28]_i_1_CO_UNCONNECTED\(3),
      CO(2) => \sum_i_reg[28]_i_1_n_1\,
      CO(1) => \sum_i_reg[28]_i_1_n_2\,
      CO(0) => \sum_i_reg[28]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2 downto 0) => sum_i_reg(30 downto 28),
      O(3 downto 0) => \sum_i_reg[30]\(3 downto 0),
      S(3) => \sum_i[28]_i_2_n_0\,
      S(2) => \sum_i[28]_i_3_n_0\,
      S(1) => \sum_i[28]_i_4_n_0\,
      S(0) => \sum_i[28]_i_5_n_0\
    );
\sum_i_reg[4]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_i_reg[0]_i_1_n_0\,
      CO(3) => \sum_i_reg[4]_i_1_n_0\,
      CO(2) => \sum_i_reg[4]_i_1_n_1\,
      CO(1) => \sum_i_reg[4]_i_1_n_2\,
      CO(0) => \sum_i_reg[4]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(7 downto 4),
      O(3 downto 0) => \sum_i_reg[7]\(3 downto 0),
      S(3) => \sum_i[4]_i_2_n_0\,
      S(2) => \sum_i[4]_i_3_n_0\,
      S(1) => \sum_i[4]_i_4_n_0\,
      S(0) => \sum_i[4]_i_5_n_0\
    );
\sum_i_reg[8]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_i_reg[4]_i_1_n_0\,
      CO(3) => \sum_i_reg[8]_i_1_n_0\,
      CO(2) => \sum_i_reg[8]_i_1_n_1\,
      CO(1) => \sum_i_reg[8]_i_1_n_2\,
      CO(0) => \sum_i_reg[8]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(11 downto 8),
      O(3 downto 0) => \sum_i_reg[11]\(3 downto 0),
      S(3) => \sum_i[8]_i_2_n_0\,
      S(2) => \sum_i[8]_i_3_n_0\,
      S(1) => \sum_i[8]_i_4_n_0\,
      S(0) => \sum_i[8]_i_5_n_0\
    );
\sum_q[0]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[19]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[19]\,
      I3 => sum_q_reg(3),
      O => \sum_q[0]_i_2_n_0\
    );
\sum_q[0]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[18]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[18]\,
      I3 => sum_q_reg(2),
      O => \sum_q[0]_i_3_n_0\
    );
\sum_q[0]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[17]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[17]\,
      I3 => sum_q_reg(1),
      O => \sum_q[0]_i_4_n_0\
    );
\sum_q[0]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[16]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[16]\,
      I3 => sum_q_reg(0),
      O => \sum_q[0]_i_5_n_0\
    );
\sum_q[12]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(15),
      O => \sum_q[12]_i_2_n_0\
    );
\sum_q[12]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[30]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[30]\,
      I3 => sum_q_reg(14),
      O => \sum_q[12]_i_3_n_0\
    );
\sum_q[12]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[29]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[29]\,
      I3 => sum_q_reg(13),
      O => \sum_q[12]_i_4_n_0\
    );
\sum_q[12]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[28]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[28]\,
      I3 => sum_q_reg(12),
      O => \sum_q[12]_i_5_n_0\
    );
\sum_q[16]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(19),
      O => \sum_q[16]_i_2_n_0\
    );
\sum_q[16]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(18),
      O => \sum_q[16]_i_3_n_0\
    );
\sum_q[16]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(17),
      O => \sum_q[16]_i_4_n_0\
    );
\sum_q[16]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(16),
      O => \sum_q[16]_i_5_n_0\
    );
\sum_q[20]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(23),
      O => \sum_q[20]_i_2_n_0\
    );
\sum_q[20]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(22),
      O => \sum_q[20]_i_3_n_0\
    );
\sum_q[20]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(21),
      O => \sum_q[20]_i_4_n_0\
    );
\sum_q[20]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(20),
      O => \sum_q[20]_i_5_n_0\
    );
\sum_q[24]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(27),
      O => \sum_q[24]_i_2_n_0\
    );
\sum_q[24]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(26),
      O => \sum_q[24]_i_3_n_0\
    );
\sum_q[24]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(25),
      O => \sum_q[24]_i_4_n_0\
    );
\sum_q[24]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(24),
      O => \sum_q[24]_i_5_n_0\
    );
\sum_q[28]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(31),
      O => \sum_q[28]_i_2_n_0\
    );
\sum_q[28]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(30),
      O => \sum_q[28]_i_3_n_0\
    );
\sum_q[28]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(29),
      O => \sum_q[28]_i_4_n_0\
    );
\sum_q[28]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(28),
      O => \sum_q[28]_i_5_n_0\
    );
\sum_q[4]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[23]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[23]\,
      I3 => sum_q_reg(7),
      O => \sum_q[4]_i_2_n_0\
    );
\sum_q[4]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[22]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[22]\,
      I3 => sum_q_reg(6),
      O => \sum_q[4]_i_3_n_0\
    );
\sum_q[4]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[21]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[21]\,
      I3 => sum_q_reg(5),
      O => \sum_q[4]_i_4_n_0\
    );
\sum_q[4]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[20]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[20]\,
      I3 => sum_q_reg(4),
      O => \sum_q[4]_i_5_n_0\
    );
\sum_q[8]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[27]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[27]\,
      I3 => sum_q_reg(11),
      O => \sum_q[8]_i_2_n_0\
    );
\sum_q[8]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[26]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[26]\,
      I3 => sum_q_reg(10),
      O => \sum_q[8]_i_3_n_0\
    );
\sum_q[8]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[25]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[25]\,
      I3 => sum_q_reg(9),
      O => \sum_q[8]_i_4_n_0\
    );
\sum_q[8]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[24]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[24]\,
      I3 => sum_q_reg(8),
      O => \sum_q[8]_i_5_n_0\
    );
\sum_q_reg[0]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \sum_q_reg[0]_i_1_n_0\,
      CO(2) => \sum_q_reg[0]_i_1_n_1\,
      CO(1) => \sum_q_reg[0]_i_1_n_2\,
      CO(0) => \sum_q_reg[0]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(3 downto 0),
      O(3 downto 0) => \sum_q_reg[3]\(3 downto 0),
      S(3) => \sum_q[0]_i_2_n_0\,
      S(2) => \sum_q[0]_i_3_n_0\,
      S(1) => \sum_q[0]_i_4_n_0\,
      S(0) => \sum_q[0]_i_5_n_0\
    );
\sum_q_reg[12]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_q_reg[8]_i_1_n_0\,
      CO(3) => \sum_q_reg[12]_i_1_n_0\,
      CO(2) => \sum_q_reg[12]_i_1_n_1\,
      CO(1) => \sum_q_reg[12]_i_1_n_2\,
      CO(0) => \sum_q_reg[12]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(15 downto 12),
      O(3 downto 0) => \sum_q_reg[15]\(3 downto 0),
      S(3) => \sum_q[12]_i_2_n_0\,
      S(2) => \sum_q[12]_i_3_n_0\,
      S(1) => \sum_q[12]_i_4_n_0\,
      S(0) => \sum_q[12]_i_5_n_0\
    );
\sum_q_reg[16]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_q_reg[12]_i_1_n_0\,
      CO(3) => \sum_q_reg[16]_i_1_n_0\,
      CO(2) => \sum_q_reg[16]_i_1_n_1\,
      CO(1) => \sum_q_reg[16]_i_1_n_2\,
      CO(0) => \sum_q_reg[16]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(19 downto 16),
      O(3 downto 0) => \sum_q_reg[19]\(3 downto 0),
      S(3) => \sum_q[16]_i_2_n_0\,
      S(2) => \sum_q[16]_i_3_n_0\,
      S(1) => \sum_q[16]_i_4_n_0\,
      S(0) => \sum_q[16]_i_5_n_0\
    );
\sum_q_reg[20]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_q_reg[16]_i_1_n_0\,
      CO(3) => \sum_q_reg[20]_i_1_n_0\,
      CO(2) => \sum_q_reg[20]_i_1_n_1\,
      CO(1) => \sum_q_reg[20]_i_1_n_2\,
      CO(0) => \sum_q_reg[20]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(23 downto 20),
      O(3 downto 0) => \sum_q_reg[23]\(3 downto 0),
      S(3) => \sum_q[20]_i_2_n_0\,
      S(2) => \sum_q[20]_i_3_n_0\,
      S(1) => \sum_q[20]_i_4_n_0\,
      S(0) => \sum_q[20]_i_5_n_0\
    );
\sum_q_reg[24]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_q_reg[20]_i_1_n_0\,
      CO(3) => \sum_q_reg[24]_i_1_n_0\,
      CO(2) => \sum_q_reg[24]_i_1_n_1\,
      CO(1) => \sum_q_reg[24]_i_1_n_2\,
      CO(0) => \sum_q_reg[24]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(27 downto 24),
      O(3 downto 0) => \sum_q_reg[27]\(3 downto 0),
      S(3) => \sum_q[24]_i_2_n_0\,
      S(2) => \sum_q[24]_i_3_n_0\,
      S(1) => \sum_q[24]_i_4_n_0\,
      S(0) => \sum_q[24]_i_5_n_0\
    );
\sum_q_reg[28]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_q_reg[24]_i_1_n_0\,
      CO(3) => \NLW_sum_q_reg[28]_i_1_CO_UNCONNECTED\(3),
      CO(2) => \sum_q_reg[28]_i_1_n_1\,
      CO(1) => \sum_q_reg[28]_i_1_n_2\,
      CO(0) => \sum_q_reg[28]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2 downto 0) => sum_q_reg(30 downto 28),
      O(3 downto 0) => \sum_q_reg[30]\(3 downto 0),
      S(3) => \sum_q[28]_i_2_n_0\,
      S(2) => \sum_q[28]_i_3_n_0\,
      S(1) => \sum_q[28]_i_4_n_0\,
      S(0) => \sum_q[28]_i_5_n_0\
    );
\sum_q_reg[4]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_q_reg[0]_i_1_n_0\,
      CO(3) => \sum_q_reg[4]_i_1_n_0\,
      CO(2) => \sum_q_reg[4]_i_1_n_1\,
      CO(1) => \sum_q_reg[4]_i_1_n_2\,
      CO(0) => \sum_q_reg[4]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(7 downto 4),
      O(3 downto 0) => \sum_q_reg[7]\(3 downto 0),
      S(3) => \sum_q[4]_i_2_n_0\,
      S(2) => \sum_q[4]_i_3_n_0\,
      S(1) => \sum_q[4]_i_4_n_0\,
      S(0) => \sum_q[4]_i_5_n_0\
    );
\sum_q_reg[8]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_q_reg[4]_i_1_n_0\,
      CO(3) => \sum_q_reg[8]_i_1_n_0\,
      CO(2) => \sum_q_reg[8]_i_1_n_1\,
      CO(1) => \sum_q_reg[8]_i_1_n_2\,
      CO(0) => \sum_q_reg[8]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(11 downto 8),
      O(3 downto 0) => \sum_q_reg[11]\(3 downto 0),
      S(3) => \sum_q[8]_i_2_n_0\,
      S(2) => \sum_q[8]_i_3_n_0\,
      S(1) => \sum_q[8]_i_4_n_0\,
      S(0) => \sum_q[8]_i_5_n_0\
    );
term1_reg_518_reg_i_10: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_43_n_7,
      O => B(9)
    );
term1_reg_518_reg_i_100: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => tmp_fu_251_p4(0),
      I1 => term1_reg_518_reg_i_49_n_6,
      O => term1_reg_518_reg_i_100_n_0
    );
term1_reg_518_reg_i_109: unisim.vcomponents.LUT3
    generic map(
      INIT => X"1B"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      O => term1_reg_518_reg_i_109_n_0
    );
term1_reg_518_reg_i_11: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_44_n_4,
      O => B(8)
    );
term1_reg_518_reg_i_113: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(16),
      O => term1_reg_518_reg_i_113_n_0
    );
term1_reg_518_reg_i_114: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_i_reg(15),
      O => term1_reg_518_reg_i_114_n_0
    );
term1_reg_518_reg_i_115: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(14),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      O => term1_reg_518_reg_i_115_n_0
    );
term1_reg_518_reg_i_116: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(13),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      O => term1_reg_518_reg_i_116_n_0
    );
term1_reg_518_reg_i_117: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(12),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      O => term1_reg_518_reg_i_117_n_0
    );
term1_reg_518_reg_i_118: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_137_n_0,
      CO(3) => term1_reg_518_reg_i_118_n_0,
      CO(2) => term1_reg_518_reg_i_118_n_1,
      CO(1) => term1_reg_518_reg_i_118_n_2,
      CO(0) => term1_reg_518_reg_i_118_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(7 downto 4),
      O(3 downto 0) => NLW_term1_reg_518_reg_i_118_O_UNCONNECTED(3 downto 0),
      S(3) => term1_reg_518_reg_i_138_n_0,
      S(2) => term1_reg_518_reg_i_139_n_0,
      S(1) => term1_reg_518_reg_i_140_n_0,
      S(0) => term1_reg_518_reg_i_141_n_0
    );
term1_reg_518_reg_i_119: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(11),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      O => term1_reg_518_reg_i_119_n_0
    );
term1_reg_518_reg_i_12: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_44_n_5,
      O => B(7)
    );
term1_reg_518_reg_i_120: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(10),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      O => term1_reg_518_reg_i_120_n_0
    );
term1_reg_518_reg_i_121: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(9),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      O => term1_reg_518_reg_i_121_n_0
    );
term1_reg_518_reg_i_122: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(8),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      O => term1_reg_518_reg_i_122_n_0
    );
term1_reg_518_reg_i_128: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => term1_reg_518_reg_i_128_n_0,
      CO(2) => term1_reg_518_reg_i_128_n_1,
      CO(1) => term1_reg_518_reg_i_128_n_2,
      CO(0) => term1_reg_518_reg_i_128_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(3 downto 0),
      O(3 downto 0) => NLW_term1_reg_518_reg_i_128_O_UNCONNECTED(3 downto 0),
      S(3) => term1_reg_518_reg_i_142_n_0,
      S(2) => term1_reg_518_reg_i_143_n_0,
      S(1) => term1_reg_518_reg_i_144_n_0,
      S(0) => term1_reg_518_reg_i_145_n_0
    );
term1_reg_518_reg_i_129: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(7),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[23]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[23]\,
      O => term1_reg_518_reg_i_129_n_0
    );
term1_reg_518_reg_i_13: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_44_n_6,
      O => B(6)
    );
term1_reg_518_reg_i_130: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(6),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[22]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[22]\,
      O => term1_reg_518_reg_i_130_n_0
    );
term1_reg_518_reg_i_131: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(5),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[21]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[21]\,
      O => term1_reg_518_reg_i_131_n_0
    );
term1_reg_518_reg_i_132: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(4),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[20]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[20]\,
      O => term1_reg_518_reg_i_132_n_0
    );
term1_reg_518_reg_i_137: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => term1_reg_518_reg_i_137_n_0,
      CO(2) => term1_reg_518_reg_i_137_n_1,
      CO(1) => term1_reg_518_reg_i_137_n_2,
      CO(0) => term1_reg_518_reg_i_137_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(3 downto 0),
      O(3 downto 0) => NLW_term1_reg_518_reg_i_137_O_UNCONNECTED(3 downto 0),
      S(3) => term1_reg_518_reg_i_146_n_0,
      S(2) => term1_reg_518_reg_i_147_n_0,
      S(1) => term1_reg_518_reg_i_148_n_0,
      S(0) => term1_reg_518_reg_i_149_n_0
    );
term1_reg_518_reg_i_138: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(7),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      O => term1_reg_518_reg_i_138_n_0
    );
term1_reg_518_reg_i_139: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(6),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      O => term1_reg_518_reg_i_139_n_0
    );
term1_reg_518_reg_i_14: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_44_n_7,
      O => B(5)
    );
term1_reg_518_reg_i_140: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(5),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      O => term1_reg_518_reg_i_140_n_0
    );
term1_reg_518_reg_i_141: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(4),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      O => term1_reg_518_reg_i_141_n_0
    );
term1_reg_518_reg_i_142: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(3),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[19]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[19]\,
      O => term1_reg_518_reg_i_142_n_0
    );
term1_reg_518_reg_i_143: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(2),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[18]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[18]\,
      O => term1_reg_518_reg_i_143_n_0
    );
term1_reg_518_reg_i_144: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(1),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[17]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[17]\,
      O => term1_reg_518_reg_i_144_n_0
    );
term1_reg_518_reg_i_145: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(0),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[16]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[16]\,
      O => term1_reg_518_reg_i_145_n_0
    );
term1_reg_518_reg_i_146: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(3),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      O => term1_reg_518_reg_i_146_n_0
    );
term1_reg_518_reg_i_147: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(2),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      O => term1_reg_518_reg_i_147_n_0
    );
term1_reg_518_reg_i_148: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(1),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      O => term1_reg_518_reg_i_148_n_0
    );
term1_reg_518_reg_i_149: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_i_reg(0),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      O => term1_reg_518_reg_i_149_n_0
    );
term1_reg_518_reg_i_15: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_45_n_4,
      O => B(4)
    );
term1_reg_518_reg_i_16: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_45_n_5,
      O => B(3)
    );
term1_reg_518_reg_i_17: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_45_n_6,
      O => B(2)
    );
term1_reg_518_reg_i_18: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_45_n_7,
      O => B(1)
    );
term1_reg_518_reg_i_19: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_46_n_4,
      O => B(0)
    );
term1_reg_518_reg_i_20: unisim.vcomponents.LUT3
    generic map(
      INIT => X"32"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => tmp_fu_251_p4(0),
      O => A(15)
    );
term1_reg_518_reg_i_21: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_49_n_6,
      O => A(14)
    );
term1_reg_518_reg_i_22: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_49_n_7,
      O => A(13)
    );
term1_reg_518_reg_i_23: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_50_n_4,
      O => A(12)
    );
term1_reg_518_reg_i_24: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_50_n_5,
      O => A(11)
    );
term1_reg_518_reg_i_25: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_50_n_6,
      O => A(10)
    );
term1_reg_518_reg_i_26: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_50_n_7,
      O => A(9)
    );
term1_reg_518_reg_i_27: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_51_n_4,
      O => A(8)
    );
term1_reg_518_reg_i_28: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_51_n_5,
      O => A(7)
    );
term1_reg_518_reg_i_29: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_51_n_6,
      O => A(6)
    );
term1_reg_518_reg_i_30: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_51_n_7,
      O => A(5)
    );
term1_reg_518_reg_i_31: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_52_n_4,
      O => A(4)
    );
term1_reg_518_reg_i_32: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_52_n_5,
      O => A(3)
    );
term1_reg_518_reg_i_33: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_52_n_6,
      O => A(2)
    );
term1_reg_518_reg_i_34: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_52_n_7,
      O => A(1)
    );
term1_reg_518_reg_i_35: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_fu_267_p2,
      I1 => term1_reg_518_reg_i_48_n_0,
      I2 => term1_reg_518_reg_i_53_n_4,
      O => A(0)
    );
term1_reg_518_reg_i_4: unisim.vcomponents.LUT3
    generic map(
      INIT => X"32"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => tmp_1_fu_305_p4(0),
      O => B(15)
    );
term1_reg_518_reg_i_40: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => icmp_ln23_1_fu_321_p2,
      CO(2) => term1_reg_518_reg_i_40_n_1,
      CO(1) => term1_reg_518_reg_i_40_n_2,
      CO(0) => term1_reg_518_reg_i_40_n_3,
      CYINIT => '0',
      DI(3) => '0',
      DI(2) => term1_reg_518_reg_i_65_n_0,
      DI(1) => term1_reg_518_reg_i_66_n_0,
      DI(0) => term1_reg_518_reg_i_67_n_0,
      O(3 downto 0) => NLW_term1_reg_518_reg_i_40_O_UNCONNECTED(3 downto 0),
      S(3) => tmp_1_fu_305_p4(5),
      S(2) => term1_reg_518_reg_i_69_n_0,
      S(1) => term1_reg_518_reg_i_70_n_0,
      S(0) => term1_reg_518_reg_i_71_n_0
    );
term1_reg_518_reg_i_41: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00000000FFFFFFFE"
    )
        port map (
      I0 => tmp_1_fu_305_p4(4),
      I1 => tmp_1_fu_305_p4(3),
      I2 => tmp_1_fu_305_p4(2),
      I3 => tmp_1_fu_305_p4(1),
      I4 => tmp_1_fu_305_p4(0),
      I5 => tmp_1_fu_305_p4(5),
      O => term1_reg_518_reg_i_41_n_0
    );
term1_reg_518_reg_i_42: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_43_n_0,
      CO(3) => term1_reg_518_reg_i_42_n_0,
      CO(2) => term1_reg_518_reg_i_42_n_1,
      CO(1) => term1_reg_518_reg_i_42_n_2,
      CO(0) => term1_reg_518_reg_i_42_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(26 downto 23),
      O(3 downto 2) => tmp_1_fu_305_p4(1 downto 0),
      O(1) => term1_reg_518_reg_i_42_n_6,
      O(0) => term1_reg_518_reg_i_42_n_7,
      S(3 downto 0) => p_reg_reg_3(3 downto 0)
    );
term1_reg_518_reg_i_43: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_44_n_0,
      CO(3) => term1_reg_518_reg_i_43_n_0,
      CO(2) => term1_reg_518_reg_i_43_n_1,
      CO(1) => term1_reg_518_reg_i_43_n_2,
      CO(0) => term1_reg_518_reg_i_43_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(22 downto 19),
      O(3) => term1_reg_518_reg_i_43_n_4,
      O(2) => term1_reg_518_reg_i_43_n_5,
      O(1) => term1_reg_518_reg_i_43_n_6,
      O(0) => term1_reg_518_reg_i_43_n_7,
      S(3 downto 0) => p_reg_reg_2(3 downto 0)
    );
term1_reg_518_reg_i_44: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_45_n_0,
      CO(3) => term1_reg_518_reg_i_44_n_0,
      CO(2) => term1_reg_518_reg_i_44_n_1,
      CO(1) => term1_reg_518_reg_i_44_n_2,
      CO(0) => term1_reg_518_reg_i_44_n_3,
      CYINIT => '0',
      DI(3 downto 1) => sum_q_reg(18 downto 16),
      DI(0) => term1_reg_518_reg_i_80_n_0,
      O(3) => term1_reg_518_reg_i_44_n_4,
      O(2) => term1_reg_518_reg_i_44_n_5,
      O(1) => term1_reg_518_reg_i_44_n_6,
      O(0) => term1_reg_518_reg_i_44_n_7,
      S(3 downto 1) => p_reg_reg_1(2 downto 0),
      S(0) => term1_reg_518_reg_i_84_n_0
    );
term1_reg_518_reg_i_45: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_46_n_0,
      CO(3) => term1_reg_518_reg_i_45_n_0,
      CO(2) => term1_reg_518_reg_i_45_n_1,
      CO(1) => term1_reg_518_reg_i_45_n_2,
      CO(0) => term1_reg_518_reg_i_45_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(15 downto 12),
      O(3) => term1_reg_518_reg_i_45_n_4,
      O(2) => term1_reg_518_reg_i_45_n_5,
      O(1) => term1_reg_518_reg_i_45_n_6,
      O(0) => term1_reg_518_reg_i_45_n_7,
      S(3) => term1_reg_518_reg_i_85_n_0,
      S(2) => term1_reg_518_reg_i_86_n_0,
      S(1) => term1_reg_518_reg_i_87_n_0,
      S(0) => term1_reg_518_reg_i_88_n_0
    );
term1_reg_518_reg_i_46: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_89_n_0,
      CO(3) => term1_reg_518_reg_i_46_n_0,
      CO(2) => term1_reg_518_reg_i_46_n_1,
      CO(1) => term1_reg_518_reg_i_46_n_2,
      CO(0) => term1_reg_518_reg_i_46_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(11 downto 8),
      O(3) => term1_reg_518_reg_i_46_n_4,
      O(2 downto 0) => NLW_term1_reg_518_reg_i_46_O_UNCONNECTED(2 downto 0),
      S(3) => term1_reg_518_reg_i_90_n_0,
      S(2) => term1_reg_518_reg_i_91_n_0,
      S(1) => term1_reg_518_reg_i_92_n_0,
      S(0) => term1_reg_518_reg_i_93_n_0
    );
term1_reg_518_reg_i_47: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => icmp_ln23_fu_267_p2,
      CO(2) => term1_reg_518_reg_i_47_n_1,
      CO(1) => term1_reg_518_reg_i_47_n_2,
      CO(0) => term1_reg_518_reg_i_47_n_3,
      CYINIT => '0',
      DI(3) => '0',
      DI(2) => term1_reg_518_reg_i_94_n_0,
      DI(1) => term1_reg_518_reg_i_95_n_0,
      DI(0) => term1_reg_518_reg_i_96_n_0,
      O(3 downto 0) => NLW_term1_reg_518_reg_i_47_O_UNCONNECTED(3 downto 0),
      S(3) => tmp_fu_251_p4(5),
      S(2) => term1_reg_518_reg_i_98_n_0,
      S(1) => term1_reg_518_reg_i_99_n_0,
      S(0) => term1_reg_518_reg_i_100_n_0
    );
term1_reg_518_reg_i_48: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00000000FFFFFFFE"
    )
        port map (
      I0 => tmp_fu_251_p4(4),
      I1 => tmp_fu_251_p4(3),
      I2 => tmp_fu_251_p4(2),
      I3 => tmp_fu_251_p4(1),
      I4 => tmp_fu_251_p4(0),
      I5 => tmp_fu_251_p4(5),
      O => term1_reg_518_reg_i_48_n_0
    );
term1_reg_518_reg_i_49: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_50_n_0,
      CO(3) => term1_reg_518_reg_i_49_n_0,
      CO(2) => term1_reg_518_reg_i_49_n_1,
      CO(1) => term1_reg_518_reg_i_49_n_2,
      CO(0) => term1_reg_518_reg_i_49_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(26 downto 23),
      O(3 downto 2) => tmp_fu_251_p4(1 downto 0),
      O(1) => term1_reg_518_reg_i_49_n_6,
      O(0) => term1_reg_518_reg_i_49_n_7,
      S(3 downto 0) => p_reg_reg_0(3 downto 0)
    );
term1_reg_518_reg_i_5: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_42_n_6,
      O => B(14)
    );
term1_reg_518_reg_i_50: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_51_n_0,
      CO(3) => term1_reg_518_reg_i_50_n_0,
      CO(2) => term1_reg_518_reg_i_50_n_1,
      CO(1) => term1_reg_518_reg_i_50_n_2,
      CO(0) => term1_reg_518_reg_i_50_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(22 downto 19),
      O(3) => term1_reg_518_reg_i_50_n_4,
      O(2) => term1_reg_518_reg_i_50_n_5,
      O(1) => term1_reg_518_reg_i_50_n_6,
      O(0) => term1_reg_518_reg_i_50_n_7,
      S(3 downto 0) => p_reg_reg(3 downto 0)
    );
term1_reg_518_reg_i_51: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_52_n_0,
      CO(3) => term1_reg_518_reg_i_51_n_0,
      CO(2) => term1_reg_518_reg_i_51_n_1,
      CO(1) => term1_reg_518_reg_i_51_n_2,
      CO(0) => term1_reg_518_reg_i_51_n_3,
      CYINIT => '0',
      DI(3 downto 1) => sum_i_reg(18 downto 16),
      DI(0) => term1_reg_518_reg_i_109_n_0,
      O(3) => term1_reg_518_reg_i_51_n_4,
      O(2) => term1_reg_518_reg_i_51_n_5,
      O(1) => term1_reg_518_reg_i_51_n_6,
      O(0) => term1_reg_518_reg_i_51_n_7,
      S(3 downto 1) => S(2 downto 0),
      S(0) => term1_reg_518_reg_i_113_n_0
    );
term1_reg_518_reg_i_52: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_53_n_0,
      CO(3) => term1_reg_518_reg_i_52_n_0,
      CO(2) => term1_reg_518_reg_i_52_n_1,
      CO(1) => term1_reg_518_reg_i_52_n_2,
      CO(0) => term1_reg_518_reg_i_52_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(15 downto 12),
      O(3) => term1_reg_518_reg_i_52_n_4,
      O(2) => term1_reg_518_reg_i_52_n_5,
      O(1) => term1_reg_518_reg_i_52_n_6,
      O(0) => term1_reg_518_reg_i_52_n_7,
      S(3) => term1_reg_518_reg_i_114_n_0,
      S(2) => term1_reg_518_reg_i_115_n_0,
      S(1) => term1_reg_518_reg_i_116_n_0,
      S(0) => term1_reg_518_reg_i_117_n_0
    );
term1_reg_518_reg_i_53: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_118_n_0,
      CO(3) => term1_reg_518_reg_i_53_n_0,
      CO(2) => term1_reg_518_reg_i_53_n_1,
      CO(1) => term1_reg_518_reg_i_53_n_2,
      CO(0) => term1_reg_518_reg_i_53_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_i_reg(11 downto 8),
      O(3) => term1_reg_518_reg_i_53_n_4,
      O(2 downto 0) => NLW_term1_reg_518_reg_i_53_O_UNCONNECTED(2 downto 0),
      S(3) => term1_reg_518_reg_i_119_n_0,
      S(2) => term1_reg_518_reg_i_120_n_0,
      S(1) => term1_reg_518_reg_i_121_n_0,
      S(0) => term1_reg_518_reg_i_122_n_0
    );
term1_reg_518_reg_i_6: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_42_n_7,
      O => B(13)
    );
term1_reg_518_reg_i_65: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => tmp_1_fu_305_p4(3),
      I1 => tmp_1_fu_305_p4(4),
      O => term1_reg_518_reg_i_65_n_0
    );
term1_reg_518_reg_i_66: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => tmp_1_fu_305_p4(1),
      I1 => tmp_1_fu_305_p4(2),
      O => term1_reg_518_reg_i_66_n_0
    );
term1_reg_518_reg_i_67: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => tmp_1_fu_305_p4(0),
      O => term1_reg_518_reg_i_67_n_0
    );
term1_reg_518_reg_i_68: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_42_n_0,
      CO(3) => NLW_term1_reg_518_reg_i_68_CO_UNCONNECTED(3),
      CO(2) => term1_reg_518_reg_i_68_n_1,
      CO(1) => term1_reg_518_reg_i_68_n_2,
      CO(0) => term1_reg_518_reg_i_68_n_3,
      CYINIT => '0',
      DI(3) => '0',
      DI(2 downto 0) => sum_q_reg(29 downto 27),
      O(3 downto 0) => tmp_1_fu_305_p4(5 downto 2),
      S(3 downto 0) => term1_reg_518_reg_i_70_0(3 downto 0)
    );
term1_reg_518_reg_i_69: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_1_fu_305_p4(3),
      I1 => tmp_1_fu_305_p4(4),
      O => term1_reg_518_reg_i_69_n_0
    );
term1_reg_518_reg_i_7: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_43_n_4,
      O => B(12)
    );
term1_reg_518_reg_i_70: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_1_fu_305_p4(1),
      I1 => tmp_1_fu_305_p4(2),
      O => term1_reg_518_reg_i_70_n_0
    );
term1_reg_518_reg_i_71: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => tmp_1_fu_305_p4(0),
      I1 => term1_reg_518_reg_i_42_n_6,
      O => term1_reg_518_reg_i_71_n_0
    );
term1_reg_518_reg_i_8: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_43_n_5,
      O => B(11)
    );
term1_reg_518_reg_i_80: unisim.vcomponents.LUT3
    generic map(
      INIT => X"1B"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      O => term1_reg_518_reg_i_80_n_0
    );
term1_reg_518_reg_i_84: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(16),
      O => term1_reg_518_reg_i_84_n_0
    );
term1_reg_518_reg_i_85: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1BE4"
    )
        port map (
      I0 => B_V_data_1_sel_rd_reg_n_0,
      I1 => \B_V_data_1_payload_A_reg_n_0_[31]\,
      I2 => \B_V_data_1_payload_B_reg_n_0_[31]\,
      I3 => sum_q_reg(15),
      O => term1_reg_518_reg_i_85_n_0
    );
term1_reg_518_reg_i_86: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(14),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[30]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[30]\,
      O => term1_reg_518_reg_i_86_n_0
    );
term1_reg_518_reg_i_87: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(13),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[29]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[29]\,
      O => term1_reg_518_reg_i_87_n_0
    );
term1_reg_518_reg_i_88: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(12),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[28]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[28]\,
      O => term1_reg_518_reg_i_88_n_0
    );
term1_reg_518_reg_i_89: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_128_n_0,
      CO(3) => term1_reg_518_reg_i_89_n_0,
      CO(2) => term1_reg_518_reg_i_89_n_1,
      CO(1) => term1_reg_518_reg_i_89_n_2,
      CO(0) => term1_reg_518_reg_i_89_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_q_reg(7 downto 4),
      O(3 downto 0) => NLW_term1_reg_518_reg_i_89_O_UNCONNECTED(3 downto 0),
      S(3) => term1_reg_518_reg_i_129_n_0,
      S(2) => term1_reg_518_reg_i_130_n_0,
      S(1) => term1_reg_518_reg_i_131_n_0,
      S(0) => term1_reg_518_reg_i_132_n_0
    );
term1_reg_518_reg_i_9: unisim.vcomponents.LUT3
    generic map(
      INIT => X"DC"
    )
        port map (
      I0 => icmp_ln23_1_fu_321_p2,
      I1 => term1_reg_518_reg_i_41_n_0,
      I2 => term1_reg_518_reg_i_43_n_6,
      O => B(10)
    );
term1_reg_518_reg_i_90: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(11),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[27]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[27]\,
      O => term1_reg_518_reg_i_90_n_0
    );
term1_reg_518_reg_i_91: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(10),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[26]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[26]\,
      O => term1_reg_518_reg_i_91_n_0
    );
term1_reg_518_reg_i_92: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(9),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[25]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[25]\,
      O => term1_reg_518_reg_i_92_n_0
    );
term1_reg_518_reg_i_93: unisim.vcomponents.LUT4
    generic map(
      INIT => X"569A"
    )
        port map (
      I0 => sum_q_reg(8),
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[24]\,
      I3 => \B_V_data_1_payload_B_reg_n_0_[24]\,
      O => term1_reg_518_reg_i_93_n_0
    );
term1_reg_518_reg_i_94: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => tmp_fu_251_p4(3),
      I1 => tmp_fu_251_p4(4),
      O => term1_reg_518_reg_i_94_n_0
    );
term1_reg_518_reg_i_95: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => tmp_fu_251_p4(1),
      I1 => tmp_fu_251_p4(2),
      O => term1_reg_518_reg_i_95_n_0
    );
term1_reg_518_reg_i_96: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => tmp_fu_251_p4(0),
      O => term1_reg_518_reg_i_96_n_0
    );
term1_reg_518_reg_i_97: unisim.vcomponents.CARRY4
     port map (
      CI => term1_reg_518_reg_i_49_n_0,
      CO(3) => NLW_term1_reg_518_reg_i_97_CO_UNCONNECTED(3),
      CO(2) => term1_reg_518_reg_i_97_n_1,
      CO(1) => term1_reg_518_reg_i_97_n_2,
      CO(0) => term1_reg_518_reg_i_97_n_3,
      CYINIT => '0',
      DI(3) => '0',
      DI(2 downto 0) => sum_i_reg(29 downto 27),
      O(3 downto 0) => tmp_fu_251_p4(5 downto 2),
      S(3 downto 0) => term1_reg_518_reg_i_99_0(3 downto 0)
    );
term1_reg_518_reg_i_98: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_fu_251_p4(3),
      I1 => tmp_fu_251_p4(4),
      O => term1_reg_518_reg_i_98_n_0
    );
term1_reg_518_reg_i_99: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => tmp_fu_251_p4(1),
      I1 => tmp_fu_251_p4(2),
      O => term1_reg_518_reg_i_99_n_0
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1\ is
  port (
    in_stream_TLAST_int_regslice : out STD_LOGIC;
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    \B_V_data_1_state_reg[1]_0\ : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1\ : entity is "fsk_discriminator_regslice_both";
end \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1\;

architecture STRUCTURE of \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1\ is
  signal B_V_data_1_payload_A : STD_LOGIC;
  signal \B_V_data_1_payload_A[0]_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_payload_B : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state[0]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state[1]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__0\ : label is "soft_lutpair8";
begin
\B_V_data_1_payload_A[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFAE00A2"
    )
        port map (
      I0 => in_stream_TLAST(0),
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => B_V_data_1_sel_wr,
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
\B_V_data_1_payload_B[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BBFB8808"
    )
        port map (
      I0 => in_stream_TLAST(0),
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
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
\B_V_data_1_sel_rd_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B4"
    )
        port map (
      I0 => \B_V_data_1_state_reg[1]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__1_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__1_n_0\,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => in_stream_TVALID,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
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
\B_V_data_1_state[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8AAA000"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg[1]_0\,
      I2 => in_stream_TVALID,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => \B_V_data_1_state_reg_n_0_[0]\,
      O => \B_V_data_1_state[0]_i_1__0_n_0\
    );
\B_V_data_1_state[1]_i_1__0\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"77F7"
    )
        port map (
      I0 => \B_V_data_1_state_reg[1]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => in_stream_TVALID,
      O => \B_V_data_1_state[1]_i_1__0_n_0\
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__0_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[1]_i_1__0_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2_i_1\: unisim.vcomponents.LUT3
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
entity \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1_0\ is
  port (
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TVALID_int_regslice : in STD_LOGIC;
    pkt_last_V_reg_489_pp0_iter2_reg : in STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1_0\ : entity is "fsk_discriminator_regslice_both";
end \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1_0\;

architecture STRUCTURE of \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1_0\ is
  signal B_V_data_1_payload_A : STD_LOGIC;
  signal \B_V_data_1_payload_A[0]_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_payload_B : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__2_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__2_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state[0]_i_1__2_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state[1]_i_1__2_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__2\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__2\ : label is "soft_lutpair18";
begin
\B_V_data_1_payload_A[0]_i_1__1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFAE00A2"
    )
        port map (
      I0 => pkt_last_V_reg_489_pp0_iter2_reg,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => B_V_data_1_sel_wr,
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
\B_V_data_1_payload_B[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BBFB8808"
    )
        port map (
      I0 => pkt_last_V_reg_489_pp0_iter2_reg,
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
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
      I0 => out_stream_TREADY,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__2_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__2_n_0\,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TVALID_int_regslice,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
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
\B_V_data_1_state[0]_i_1__2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A2AAA000"
    )
        port map (
      I0 => ap_rst_n,
      I1 => out_stream_TREADY,
      I2 => out_stream_TVALID_int_regslice,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => \B_V_data_1_state_reg_n_0_[0]\,
      O => \B_V_data_1_state[0]_i_1__2_n_0\
    );
\B_V_data_1_state[1]_i_1__2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"BBFB"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => out_stream_TVALID_int_regslice,
      O => \B_V_data_1_state[1]_i_1__2_n_0\
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__2_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[1]_i_1__2_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
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
entity \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized2\ is
  port (
    \B_V_data_1_state_reg[0]_0\ : out STD_LOGIC;
    i_old0 : out STD_LOGIC;
    \B_V_data_1_state_reg[0]_1\ : out STD_LOGIC;
    out_stream_TVALID_int_regslice : out STD_LOGIC;
    ap_block_pp0_stage0_11001 : out STD_LOGIC;
    \icmp_ln52_reg_494_reg[0]\ : out STD_LOGIC;
    \B_V_data_1_state_reg[0]_2\ : out STD_LOGIC;
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 15 downto 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    ap_enable_reg_pp0_iter1 : in STD_LOGIC;
    icmp_ln52_reg_494 : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    ap_enable_reg_pp0_iter3 : in STD_LOGIC;
    icmp_ln52_reg_494_pp0_iter2_reg : in STD_LOGIC;
    in_stream_TVALID_int_regslice : in STD_LOGIC;
    icmp_ln52_reg_494_pp0_iter3_reg : in STD_LOGIC;
    ap_enable_reg_pp0_iter4 : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[15]_0\ : in STD_LOGIC;
    p_reg_reg : in STD_LOGIC;
    p_reg_reg_0 : in STD_LOGIC;
    p_reg_reg_1 : in STD_LOGIC;
    p_reg_reg_2 : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[14]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[13]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[12]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[11]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[10]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[9]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[8]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[7]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[6]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[5]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[4]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[3]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[2]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[1]_0\ : in STD_LOGIC;
    \B_V_data_1_payload_A_reg[0]_0\ : in STD_LOGIC;
    P : in STD_LOGIC_VECTOR ( 1 downto 0 );
    CO : in STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized2\ : entity is "fsk_discriminator_regslice_both";
end \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized2\;

architecture STRUCTURE of \system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized2\ is
  signal B_V_data_1_load_B : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[14]_i_2_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[15]_i_1_n_0\ : STD_LOGIC;
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
  signal \B_V_data_1_payload_B[14]_i_1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_B[15]_i_1_n_0\ : STD_LOGIC;
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
  signal B_V_data_1_sel_rd_i_1_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_rd_reg_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state[0]_i_1__1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state[1]_i_1__1_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[0]_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[0]_1\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  signal ap_enable_reg_pp0_iter141_out : STD_LOGIC;
  signal \^out_stream_tvalid_int_regslice\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of B_V_data_1_sel_rd_i_1 : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \out_stream_TDATA[0]_INST_0\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \out_stream_TDATA[10]_INST_0\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \out_stream_TDATA[11]_INST_0\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \out_stream_TDATA[12]_INST_0\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \out_stream_TDATA[13]_INST_0\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \out_stream_TDATA[14]_INST_0\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \out_stream_TDATA[15]_INST_0\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \out_stream_TDATA[1]_INST_0\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \out_stream_TDATA[2]_INST_0\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \out_stream_TDATA[3]_INST_0\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \out_stream_TDATA[4]_INST_0\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \out_stream_TDATA[5]_INST_0\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \out_stream_TDATA[6]_INST_0\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \out_stream_TDATA[7]_INST_0\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \out_stream_TDATA[8]_INST_0\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \out_stream_TDATA[9]_INST_0\ : label is "soft_lutpair14";
begin
  \B_V_data_1_state_reg[0]_0\ <= \^b_v_data_1_state_reg[0]_0\;
  \B_V_data_1_state_reg[0]_1\ <= \^b_v_data_1_state_reg[0]_1\;
  out_stream_TVALID_int_regslice <= \^out_stream_tvalid_int_regslice\;
\B_V_data_1_payload_A[14]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0045"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => \^b_v_data_1_state_reg[0]_0\,
      I3 => \B_V_data_1_payload_A_reg[15]_0\,
      O => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A[14]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"0D"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_payload_A[14]_i_2_n_0\
    );
\B_V_data_1_payload_A[15]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"F888FFFFF8880000"
    )
        port map (
      I0 => P(1),
      I1 => P(0),
      I2 => CO(0),
      I3 => \B_V_data_1_payload_A_reg[15]_0\,
      I4 => \B_V_data_1_payload_A[14]_i_2_n_0\,
      I5 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \B_V_data_1_payload_A[15]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[0]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[0]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[10]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[10]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[10]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[11]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[11]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[11]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[12]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[12]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[12]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[13]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[13]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[13]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[14]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[14]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[14]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_A[15]_i_1_n_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[1]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[1]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[2]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[2]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[3]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[3]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[4]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[4]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[4]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[5]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[5]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[5]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[6]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[6]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[6]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[7]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[7]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[7]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[8]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[8]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[8]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[9]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[14]_i_2_n_0\,
      D => \B_V_data_1_payload_A_reg[9]_0\,
      Q => \B_V_data_1_payload_A_reg_n_0_[9]\,
      S => \B_V_data_1_payload_A[14]_i_1_n_0\
    );
\B_V_data_1_payload_B[14]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"00B0"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[1]\,
      I1 => \^b_v_data_1_state_reg[0]_0\,
      I2 => B_V_data_1_sel_wr,
      I3 => \B_V_data_1_payload_A_reg[15]_0\,
      O => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B[14]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A2"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \^b_v_data_1_state_reg[0]_0\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B[15]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"F888FFFFF8880000"
    )
        port map (
      I0 => P(1),
      I1 => P(0),
      I2 => CO(0),
      I3 => \B_V_data_1_payload_A_reg[15]_0\,
      I4 => B_V_data_1_load_B,
      I5 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      O => \B_V_data_1_payload_B[15]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[0]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[0]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[10]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[10]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[10]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[11]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[11]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[11]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[12]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[12]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[12]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[13]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[13]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[13]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[14]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[14]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[14]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_B[15]_i_1_n_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[1]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[1]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[2]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[2]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[3]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[3]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[4]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[4]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[4]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[5]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[5]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[5]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[6]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[6]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[6]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[7]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[7]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[7]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[8]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[8]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[8]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[9]\: unisim.vcomponents.FDSE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => \B_V_data_1_payload_A_reg[9]_0\,
      Q => \B_V_data_1_payload_B_reg_n_0_[9]\,
      S => \B_V_data_1_payload_B[14]_i_1_n_0\
    );
B_V_data_1_sel_rd_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \^b_v_data_1_state_reg[0]_0\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => B_V_data_1_sel_rd_i_1_n_0
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_sel_rd_i_1_n_0,
      Q => B_V_data_1_sel_rd_reg_n_0,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => \^out_stream_tvalid_int_regslice\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
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
\B_V_data_1_state[0]_i_1__1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A2AAA000"
    )
        port map (
      I0 => ap_rst_n,
      I1 => out_stream_TREADY,
      I2 => \^out_stream_tvalid_int_regslice\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => \^b_v_data_1_state_reg[0]_0\,
      O => \B_V_data_1_state[0]_i_1__1_n_0\
    );
\B_V_data_1_state[0]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"20"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter3,
      I1 => \^b_v_data_1_state_reg[0]_1\,
      I2 => icmp_ln52_reg_494_pp0_iter2_reg,
      O => \^out_stream_tvalid_int_regslice\
    );
\B_V_data_1_state[1]_i_1__1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"BBFB"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \^b_v_data_1_state_reg[0]_0\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => \^out_stream_tvalid_int_regslice\,
      O => \B_V_data_1_state[1]_i_1__1_n_0\
    );
\B_V_data_1_state[1]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BBFBBBBB"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter141_out,
      I1 => in_stream_TVALID_int_regslice,
      I2 => ap_enable_reg_pp0_iter3,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => icmp_ln52_reg_494_pp0_iter2_reg,
      O => \^b_v_data_1_state_reg[0]_1\
    );
\B_V_data_1_state[1]_i_4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"3F220000"
    )
        port map (
      I0 => icmp_ln52_reg_494_pp0_iter3_reg,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => out_stream_TREADY,
      I3 => \^b_v_data_1_state_reg[0]_0\,
      I4 => ap_enable_reg_pp0_iter4,
      O => ap_enable_reg_pp0_iter141_out
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__1_n_0\,
      Q => \^b_v_data_1_state_reg[0]_0\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[1]_i_1__1_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\out_stream_TDATA[0]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(0)
    );
\out_stream_TDATA[10]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(10)
    );
\out_stream_TDATA[11]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(11)
    );
\out_stream_TDATA[12]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(12)
    );
\out_stream_TDATA[13]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(13)
    );
\out_stream_TDATA[14]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(14)
    );
\out_stream_TDATA[15]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(15)
    );
\out_stream_TDATA[1]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(1)
    );
\out_stream_TDATA[2]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(2)
    );
\out_stream_TDATA[3]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(3)
    );
\out_stream_TDATA[4]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(4)
    );
\out_stream_TDATA[5]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(5)
    );
\out_stream_TDATA[6]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(6)
    );
\out_stream_TDATA[7]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(7)
    );
\out_stream_TDATA[8]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(8)
    );
\out_stream_TDATA[9]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      I1 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => out_stream_TDATA(9)
    );
p_reg_reg_i_1: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_1\,
      O => ap_block_pp0_stage0_11001
    );
term1_reg_518_reg_i_1: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000080"
    )
        port map (
      I0 => p_reg_reg,
      I1 => p_reg_reg_0,
      I2 => p_reg_reg_1,
      I3 => p_reg_reg_2,
      I4 => \^b_v_data_1_state_reg[0]_1\,
      O => \B_V_data_1_state_reg[0]_2\
    );
term1_reg_518_reg_i_2: unisim.vcomponents.LUT3
    generic map(
      INIT => X"20"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter1,
      I1 => \^b_v_data_1_state_reg[0]_1\,
      I2 => icmp_ln52_reg_494,
      O => i_old0
    );
term1_reg_518_reg_i_3: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => icmp_ln52_reg_494,
      I1 => \^b_v_data_1_state_reg[0]_1\,
      O => \icmp_ln52_reg_494_reg[0]\
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1 is
  port (
    P : out STD_LOGIC_VECTOR ( 1 downto 0 );
    \counter_reg[12]\ : out STD_LOGIC;
    \counter_reg[8]\ : out STD_LOGIC;
    \counter_reg[16]\ : out STD_LOGIC;
    \counter_reg[28]\ : out STD_LOGIC;
    S : out STD_LOGIC_VECTOR ( 2 downto 0 );
    \sum_i_reg[22]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[26]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_i_reg[30]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[18]\ : out STD_LOGIC_VECTOR ( 2 downto 0 );
    \sum_q_reg[22]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[26]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_q_reg[30]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    p_reg_reg : out STD_LOGIC;
    p_reg_reg_0 : out STD_LOGIC;
    CO : out STD_LOGIC_VECTOR ( 0 to 0 );
    p_reg_reg_1 : out STD_LOGIC;
    p_reg_reg_2 : out STD_LOGIC;
    p_reg_reg_3 : out STD_LOGIC;
    p_reg_reg_4 : out STD_LOGIC;
    p_reg_reg_5 : out STD_LOGIC;
    p_reg_reg_6 : out STD_LOGIC;
    p_reg_reg_7 : out STD_LOGIC;
    p_reg_reg_8 : out STD_LOGIC;
    p_reg_reg_9 : out STD_LOGIC;
    p_reg_reg_10 : out STD_LOGIC;
    p_reg_reg_11 : out STD_LOGIC;
    p_reg_reg_12 : out STD_LOGIC;
    p_reg_reg_13 : out STD_LOGIC;
    p_reg_reg_14 : out STD_LOGIC;
    ap_block_pp0_stage0_11001 : in STD_LOGIC;
    p_reg_reg_15 : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    B : in STD_LOGIC_VECTOR ( 15 downto 0 );
    A : in STD_LOGIC_VECTOR ( 15 downto 0 );
    PCOUT : in STD_LOGIC_VECTOR ( 47 downto 0 );
    \out\ : in STD_LOGIC_VECTOR ( 31 downto 0 );
    sum_i_reg : in STD_LOGIC_VECTOR ( 15 downto 0 );
    sum_q_reg : in STD_LOGIC_VECTOR ( 15 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1 : entity is "fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1";
end system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1;

architecture STRUCTURE of system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1 is
begin
fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0_U: entity work.system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1_DSP48_0
     port map (
      A(15 downto 0) => A(15 downto 0),
      B(15 downto 0) => B(15 downto 0),
      CO(0) => CO(0),
      P(1 downto 0) => P(1 downto 0),
      PCOUT(47 downto 0) => PCOUT(47 downto 0),
      S(2 downto 0) => S(2 downto 0),
      ap_block_pp0_stage0_11001 => ap_block_pp0_stage0_11001,
      ap_clk => ap_clk,
      \counter_reg[12]\ => \counter_reg[12]\,
      \counter_reg[16]\ => \counter_reg[16]\,
      \counter_reg[28]\ => \counter_reg[28]\,
      \counter_reg[8]\ => \counter_reg[8]\,
      \out\(31 downto 0) => \out\(31 downto 0),
      p_reg_reg_0 => p_reg_reg,
      p_reg_reg_1 => p_reg_reg_0,
      p_reg_reg_10 => p_reg_reg_9,
      p_reg_reg_11 => p_reg_reg_10,
      p_reg_reg_12 => p_reg_reg_11,
      p_reg_reg_13 => p_reg_reg_12,
      p_reg_reg_14 => p_reg_reg_13,
      p_reg_reg_15 => p_reg_reg_14,
      p_reg_reg_16 => p_reg_reg_15,
      p_reg_reg_2 => p_reg_reg_1,
      p_reg_reg_3 => p_reg_reg_2,
      p_reg_reg_4 => p_reg_reg_3,
      p_reg_reg_5 => p_reg_reg_4,
      p_reg_reg_6 => p_reg_reg_5,
      p_reg_reg_7 => p_reg_reg_6,
      p_reg_reg_8 => p_reg_reg_7,
      p_reg_reg_9 => p_reg_reg_8,
      sum_i_reg(15 downto 0) => sum_i_reg(15 downto 0),
      \sum_i_reg[22]\(3 downto 0) => \sum_i_reg[22]\(3 downto 0),
      \sum_i_reg[26]\(3 downto 0) => \sum_i_reg[26]\(3 downto 0),
      \sum_i_reg[30]\(3 downto 0) => \sum_i_reg[30]\(3 downto 0),
      sum_q_reg(15 downto 0) => sum_q_reg(15 downto 0),
      \sum_q_reg[18]\(2 downto 0) => \sum_q_reg[18]\(2 downto 0),
      \sum_q_reg[22]\(3 downto 0) => \sum_q_reg[22]\(3 downto 0),
      \sum_q_reg[26]\(3 downto 0) => \sum_q_reg[26]\(3 downto 0),
      \sum_q_reg[30]\(3 downto 0) => \sum_q_reg[30]\(3 downto 0)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_fsk_discriminator_0_0_fsk_discriminator is
  port (
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TREADY : out STD_LOGIC;
    in_stream_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    in_stream_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 15 downto 0 );
    out_stream_TVALID : out STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TKEEP : out STD_LOGIC_VECTOR ( 1 downto 0 );
    out_stream_TSTRB : out STD_LOGIC_VECTOR ( 1 downto 0 );
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of system_fsk_discriminator_0_0_fsk_discriminator : entity is "fsk_discriminator";
  attribute ap_ST_fsm_pp0_stage0 : string;
  attribute ap_ST_fsm_pp0_stage0 of system_fsk_discriminator_0_0_fsk_discriminator : entity is "1'b1";
  attribute hls_module : string;
  attribute hls_module of system_fsk_discriminator_0_0_fsk_discriminator : entity is "yes";
end system_fsk_discriminator_0_0_fsk_discriminator;

architecture STRUCTURE of system_fsk_discriminator_0_0_fsk_discriminator is
  signal \<const0>\ : STD_LOGIC;
  signal add_ln49_fu_218_p2 : STD_LOGIC_VECTOR ( 0 to 0 );
  signal ap_block_pp0_stage0_11001 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter1 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter2 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter3 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter4 : STD_LOGIC;
  signal ap_rst_n_inv : STD_LOGIC;
  signal counter_reg : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal \counter_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_7\ : STD_LOGIC;
  signal i_old0 : STD_LOGIC;
  signal icmp_ln23_2_fu_441_p2 : STD_LOGIC;
  signal icmp_ln52_fu_225_p2 : STD_LOGIC;
  signal icmp_ln52_reg_494 : STD_LOGIC;
  signal icmp_ln52_reg_494_pp0_iter1_reg : STD_LOGIC;
  signal icmp_ln52_reg_494_pp0_iter2_reg : STD_LOGIC;
  signal icmp_ln52_reg_494_pp0_iter3_reg : STD_LOGIC;
  signal in_stream_TLAST_int_regslice : STD_LOGIC;
  signal in_stream_TVALID_int_regslice : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_10 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_11 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_12 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_13 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_14 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_15 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_16 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_17 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_18 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_19 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_2 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_20 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_21 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_22 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_23 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_24 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_25 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_26 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_27 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_28 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_29 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_3 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_30 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_31 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_32 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_33 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_34 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_35 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_36 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_37 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_39 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_4 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_40 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_41 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_42 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_43 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_44 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_45 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_46 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_47 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_48 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_49 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_5 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_50 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_51 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_52 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_6 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_7 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_8 : STD_LOGIC;
  signal mac_mulsub_16s_16s_32s_32_4_1_U2_n_9 : STD_LOGIC;
  signal out_stream_TVALID_int_regslice : STD_LOGIC;
  signal \pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2_n_0\ : STD_LOGIC;
  signal pkt_last_V_reg_489_pp0_iter2_reg : STD_LOGIC;
  signal q_curr_fu_351_p3 : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal regslice_both_in_stream_V_data_V_U_n_19 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_20 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_21 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_22 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_23 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_24 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_25 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_26 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_27 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_28 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_29 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_30 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_31 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_32 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_33 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_34 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_35 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_36 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_37 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_38 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_39 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_40 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_41 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_42 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_43 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_44 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_45 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_46 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_47 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_48 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_49 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_50 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_51 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_52 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_53 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_54 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_55 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_56 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_57 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_58 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_59 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_60 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_61 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_62 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_63 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_64 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_65 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_66 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_67 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_68 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_69 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_70 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_71 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_72 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_73 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_74 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_75 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_76 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_77 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_78 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_79 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_80 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_81 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_82 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_83 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_84 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_85 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_86 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_87 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_88 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_89 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_90 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_91 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_92 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_93 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_94 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_95 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_96 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_97 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_98 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_2 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_5 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_6 : STD_LOGIC;
  signal sum_i_reg : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal sum_q_reg : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal term1_reg_518_reg_n_106 : STD_LOGIC;
  signal term1_reg_518_reg_n_107 : STD_LOGIC;
  signal term1_reg_518_reg_n_108 : STD_LOGIC;
  signal term1_reg_518_reg_n_109 : STD_LOGIC;
  signal term1_reg_518_reg_n_110 : STD_LOGIC;
  signal term1_reg_518_reg_n_111 : STD_LOGIC;
  signal term1_reg_518_reg_n_112 : STD_LOGIC;
  signal term1_reg_518_reg_n_113 : STD_LOGIC;
  signal term1_reg_518_reg_n_114 : STD_LOGIC;
  signal term1_reg_518_reg_n_115 : STD_LOGIC;
  signal term1_reg_518_reg_n_116 : STD_LOGIC;
  signal term1_reg_518_reg_n_117 : STD_LOGIC;
  signal term1_reg_518_reg_n_118 : STD_LOGIC;
  signal term1_reg_518_reg_n_119 : STD_LOGIC;
  signal term1_reg_518_reg_n_120 : STD_LOGIC;
  signal term1_reg_518_reg_n_121 : STD_LOGIC;
  signal term1_reg_518_reg_n_122 : STD_LOGIC;
  signal term1_reg_518_reg_n_123 : STD_LOGIC;
  signal term1_reg_518_reg_n_124 : STD_LOGIC;
  signal term1_reg_518_reg_n_125 : STD_LOGIC;
  signal term1_reg_518_reg_n_126 : STD_LOGIC;
  signal term1_reg_518_reg_n_127 : STD_LOGIC;
  signal term1_reg_518_reg_n_128 : STD_LOGIC;
  signal term1_reg_518_reg_n_129 : STD_LOGIC;
  signal term1_reg_518_reg_n_130 : STD_LOGIC;
  signal term1_reg_518_reg_n_131 : STD_LOGIC;
  signal term1_reg_518_reg_n_132 : STD_LOGIC;
  signal term1_reg_518_reg_n_133 : STD_LOGIC;
  signal term1_reg_518_reg_n_134 : STD_LOGIC;
  signal term1_reg_518_reg_n_135 : STD_LOGIC;
  signal term1_reg_518_reg_n_136 : STD_LOGIC;
  signal term1_reg_518_reg_n_137 : STD_LOGIC;
  signal term1_reg_518_reg_n_138 : STD_LOGIC;
  signal term1_reg_518_reg_n_139 : STD_LOGIC;
  signal term1_reg_518_reg_n_140 : STD_LOGIC;
  signal term1_reg_518_reg_n_141 : STD_LOGIC;
  signal term1_reg_518_reg_n_142 : STD_LOGIC;
  signal term1_reg_518_reg_n_143 : STD_LOGIC;
  signal term1_reg_518_reg_n_144 : STD_LOGIC;
  signal term1_reg_518_reg_n_145 : STD_LOGIC;
  signal term1_reg_518_reg_n_146 : STD_LOGIC;
  signal term1_reg_518_reg_n_147 : STD_LOGIC;
  signal term1_reg_518_reg_n_148 : STD_LOGIC;
  signal term1_reg_518_reg_n_149 : STD_LOGIC;
  signal term1_reg_518_reg_n_150 : STD_LOGIC;
  signal term1_reg_518_reg_n_151 : STD_LOGIC;
  signal term1_reg_518_reg_n_152 : STD_LOGIC;
  signal term1_reg_518_reg_n_153 : STD_LOGIC;
  signal tmp_2_fu_426_p4 : STD_LOGIC_VECTOR ( 10 downto 0 );
  signal \NLW_counter_reg[28]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal NLW_term1_reg_518_reg_CARRYCASCOUT_UNCONNECTED : STD_LOGIC;
  signal NLW_term1_reg_518_reg_MULTSIGNOUT_UNCONNECTED : STD_LOGIC;
  signal NLW_term1_reg_518_reg_OVERFLOW_UNCONNECTED : STD_LOGIC;
  signal NLW_term1_reg_518_reg_PATTERNBDETECT_UNCONNECTED : STD_LOGIC;
  signal NLW_term1_reg_518_reg_PATTERNDETECT_UNCONNECTED : STD_LOGIC;
  signal NLW_term1_reg_518_reg_UNDERFLOW_UNCONNECTED : STD_LOGIC;
  signal NLW_term1_reg_518_reg_ACOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 29 downto 0 );
  signal NLW_term1_reg_518_reg_BCOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 17 downto 0 );
  signal NLW_term1_reg_518_reg_CARRYOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_term1_reg_518_reg_P_UNCONNECTED : STD_LOGIC_VECTOR ( 47 downto 0 );
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \counter_reg[0]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[12]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[16]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[20]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[24]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[28]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[4]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[8]_i_1\ : label is 11;
  attribute srl_bus_name : string;
  attribute srl_bus_name of \pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\pkt_last_V_reg_489_pp0_iter1_reg_reg ";
  attribute srl_name : string;
  attribute srl_name of \pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2 ";
begin
  out_stream_TKEEP(1) <= \<const0>\;
  out_stream_TKEEP(0) <= \<const0>\;
  out_stream_TSTRB(1) <= \<const0>\;
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
      CE => ap_block_pp0_stage0_11001,
      D => '1',
      Q => ap_enable_reg_pp0_iter1,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter2_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => ap_enable_reg_pp0_iter1,
      Q => ap_enable_reg_pp0_iter2,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter3_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => ap_enable_reg_pp0_iter2,
      Q => ap_enable_reg_pp0_iter3,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter4_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => ap_enable_reg_pp0_iter3,
      Q => ap_enable_reg_pp0_iter4,
      R => ap_rst_n_inv
    );
\counter[0]_i_2\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => counter_reg(0),
      O => add_ln49_fu_218_p2(0)
    );
\counter_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[0]_i_1_n_7\,
      Q => counter_reg(0),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[0]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \counter_reg[0]_i_1_n_0\,
      CO(2) => \counter_reg[0]_i_1_n_1\,
      CO(1) => \counter_reg[0]_i_1_n_2\,
      CO(0) => \counter_reg[0]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0001",
      O(3) => \counter_reg[0]_i_1_n_4\,
      O(2) => \counter_reg[0]_i_1_n_5\,
      O(1) => \counter_reg[0]_i_1_n_6\,
      O(0) => \counter_reg[0]_i_1_n_7\,
      S(3 downto 1) => counter_reg(3 downto 1),
      S(0) => add_ln49_fu_218_p2(0)
    );
\counter_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[8]_i_1_n_5\,
      Q => counter_reg(10),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[8]_i_1_n_4\,
      Q => counter_reg(11),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[12]_i_1_n_7\,
      Q => counter_reg(12),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[12]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[8]_i_1_n_0\,
      CO(3) => \counter_reg[12]_i_1_n_0\,
      CO(2) => \counter_reg[12]_i_1_n_1\,
      CO(1) => \counter_reg[12]_i_1_n_2\,
      CO(0) => \counter_reg[12]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[12]_i_1_n_4\,
      O(2) => \counter_reg[12]_i_1_n_5\,
      O(1) => \counter_reg[12]_i_1_n_6\,
      O(0) => \counter_reg[12]_i_1_n_7\,
      S(3 downto 0) => counter_reg(15 downto 12)
    );
\counter_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[12]_i_1_n_6\,
      Q => counter_reg(13),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[12]_i_1_n_5\,
      Q => counter_reg(14),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[12]_i_1_n_4\,
      Q => counter_reg(15),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[16]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[16]_i_1_n_7\,
      Q => counter_reg(16),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[16]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[12]_i_1_n_0\,
      CO(3) => \counter_reg[16]_i_1_n_0\,
      CO(2) => \counter_reg[16]_i_1_n_1\,
      CO(1) => \counter_reg[16]_i_1_n_2\,
      CO(0) => \counter_reg[16]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[16]_i_1_n_4\,
      O(2) => \counter_reg[16]_i_1_n_5\,
      O(1) => \counter_reg[16]_i_1_n_6\,
      O(0) => \counter_reg[16]_i_1_n_7\,
      S(3 downto 0) => counter_reg(19 downto 16)
    );
\counter_reg[17]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[16]_i_1_n_6\,
      Q => counter_reg(17),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[18]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[16]_i_1_n_5\,
      Q => counter_reg(18),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[19]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[16]_i_1_n_4\,
      Q => counter_reg(19),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[0]_i_1_n_6\,
      Q => counter_reg(1),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[20]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[20]_i_1_n_7\,
      Q => counter_reg(20),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[20]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[16]_i_1_n_0\,
      CO(3) => \counter_reg[20]_i_1_n_0\,
      CO(2) => \counter_reg[20]_i_1_n_1\,
      CO(1) => \counter_reg[20]_i_1_n_2\,
      CO(0) => \counter_reg[20]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[20]_i_1_n_4\,
      O(2) => \counter_reg[20]_i_1_n_5\,
      O(1) => \counter_reg[20]_i_1_n_6\,
      O(0) => \counter_reg[20]_i_1_n_7\,
      S(3 downto 0) => counter_reg(23 downto 20)
    );
\counter_reg[21]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[20]_i_1_n_6\,
      Q => counter_reg(21),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[22]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[20]_i_1_n_5\,
      Q => counter_reg(22),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[23]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[20]_i_1_n_4\,
      Q => counter_reg(23),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[24]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[24]_i_1_n_7\,
      Q => counter_reg(24),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[24]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[20]_i_1_n_0\,
      CO(3) => \counter_reg[24]_i_1_n_0\,
      CO(2) => \counter_reg[24]_i_1_n_1\,
      CO(1) => \counter_reg[24]_i_1_n_2\,
      CO(0) => \counter_reg[24]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[24]_i_1_n_4\,
      O(2) => \counter_reg[24]_i_1_n_5\,
      O(1) => \counter_reg[24]_i_1_n_6\,
      O(0) => \counter_reg[24]_i_1_n_7\,
      S(3 downto 0) => counter_reg(27 downto 24)
    );
\counter_reg[25]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[24]_i_1_n_6\,
      Q => counter_reg(25),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[26]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[24]_i_1_n_5\,
      Q => counter_reg(26),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[27]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[24]_i_1_n_4\,
      Q => counter_reg(27),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[28]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[28]_i_1_n_7\,
      Q => counter_reg(28),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[28]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[24]_i_1_n_0\,
      CO(3) => \NLW_counter_reg[28]_i_1_CO_UNCONNECTED\(3),
      CO(2) => \counter_reg[28]_i_1_n_1\,
      CO(1) => \counter_reg[28]_i_1_n_2\,
      CO(0) => \counter_reg[28]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[28]_i_1_n_4\,
      O(2) => \counter_reg[28]_i_1_n_5\,
      O(1) => \counter_reg[28]_i_1_n_6\,
      O(0) => \counter_reg[28]_i_1_n_7\,
      S(3 downto 0) => counter_reg(31 downto 28)
    );
\counter_reg[29]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[28]_i_1_n_6\,
      Q => counter_reg(29),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[0]_i_1_n_5\,
      Q => counter_reg(2),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[30]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[28]_i_1_n_5\,
      Q => counter_reg(30),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[31]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[28]_i_1_n_4\,
      Q => counter_reg(31),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[0]_i_1_n_4\,
      Q => counter_reg(3),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[4]_i_1_n_7\,
      Q => counter_reg(4),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[4]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[0]_i_1_n_0\,
      CO(3) => \counter_reg[4]_i_1_n_0\,
      CO(2) => \counter_reg[4]_i_1_n_1\,
      CO(1) => \counter_reg[4]_i_1_n_2\,
      CO(0) => \counter_reg[4]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[4]_i_1_n_4\,
      O(2) => \counter_reg[4]_i_1_n_5\,
      O(1) => \counter_reg[4]_i_1_n_6\,
      O(0) => \counter_reg[4]_i_1_n_7\,
      S(3 downto 0) => counter_reg(7 downto 4)
    );
\counter_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[4]_i_1_n_6\,
      Q => counter_reg(5),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[4]_i_1_n_5\,
      Q => counter_reg(6),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[4]_i_1_n_4\,
      Q => counter_reg(7),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[8]_i_1_n_7\,
      Q => counter_reg(8),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\counter_reg[8]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[4]_i_1_n_0\,
      CO(3) => \counter_reg[8]_i_1_n_0\,
      CO(2) => \counter_reg[8]_i_1_n_1\,
      CO(1) => \counter_reg[8]_i_1_n_2\,
      CO(0) => \counter_reg[8]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[8]_i_1_n_4\,
      O(2) => \counter_reg[8]_i_1_n_5\,
      O(1) => \counter_reg[8]_i_1_n_6\,
      O(0) => \counter_reg[8]_i_1_n_7\,
      S(3 downto 0) => counter_reg(11 downto 8)
    );
\counter_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \counter_reg[8]_i_1_n_6\,
      Q => counter_reg(9),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\icmp_ln52_reg_494[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"4000"
    )
        port map (
      I0 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_4,
      I1 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_5,
      I2 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_2,
      I3 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_3,
      O => icmp_ln52_fu_225_p2
    );
\icmp_ln52_reg_494_pp0_iter1_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => icmp_ln52_reg_494,
      Q => icmp_ln52_reg_494_pp0_iter1_reg,
      R => '0'
    );
\icmp_ln52_reg_494_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => icmp_ln52_reg_494_pp0_iter1_reg,
      Q => icmp_ln52_reg_494_pp0_iter2_reg,
      R => '0'
    );
\icmp_ln52_reg_494_pp0_iter3_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => icmp_ln52_reg_494_pp0_iter2_reg,
      Q => icmp_ln52_reg_494_pp0_iter3_reg,
      R => '0'
    );
\icmp_ln52_reg_494_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => icmp_ln52_fu_225_p2,
      Q => icmp_ln52_reg_494,
      R => '0'
    );
mac_mulsub_16s_16s_32s_32_4_1_U2: entity work.system_fsk_discriminator_0_0_fsk_discriminator_mac_mulsub_16s_16s_32s_32_4_1
     port map (
      A(15) => regslice_both_in_stream_V_data_V_U_n_19,
      A(14) => regslice_both_in_stream_V_data_V_U_n_20,
      A(13) => regslice_both_in_stream_V_data_V_U_n_21,
      A(12) => regslice_both_in_stream_V_data_V_U_n_22,
      A(11) => regslice_both_in_stream_V_data_V_U_n_23,
      A(10) => regslice_both_in_stream_V_data_V_U_n_24,
      A(9) => regslice_both_in_stream_V_data_V_U_n_25,
      A(8) => regslice_both_in_stream_V_data_V_U_n_26,
      A(7) => regslice_both_in_stream_V_data_V_U_n_27,
      A(6) => regslice_both_in_stream_V_data_V_U_n_28,
      A(5) => regslice_both_in_stream_V_data_V_U_n_29,
      A(4) => regslice_both_in_stream_V_data_V_U_n_30,
      A(3) => regslice_both_in_stream_V_data_V_U_n_31,
      A(2) => regslice_both_in_stream_V_data_V_U_n_32,
      A(1) => regslice_both_in_stream_V_data_V_U_n_33,
      A(0) => regslice_both_in_stream_V_data_V_U_n_34,
      B(15 downto 0) => q_curr_fu_351_p3(15 downto 0),
      CO(0) => icmp_ln23_2_fu_441_p2,
      P(1) => tmp_2_fu_426_p4(10),
      P(0) => tmp_2_fu_426_p4(0),
      PCOUT(47) => term1_reg_518_reg_n_106,
      PCOUT(46) => term1_reg_518_reg_n_107,
      PCOUT(45) => term1_reg_518_reg_n_108,
      PCOUT(44) => term1_reg_518_reg_n_109,
      PCOUT(43) => term1_reg_518_reg_n_110,
      PCOUT(42) => term1_reg_518_reg_n_111,
      PCOUT(41) => term1_reg_518_reg_n_112,
      PCOUT(40) => term1_reg_518_reg_n_113,
      PCOUT(39) => term1_reg_518_reg_n_114,
      PCOUT(38) => term1_reg_518_reg_n_115,
      PCOUT(37) => term1_reg_518_reg_n_116,
      PCOUT(36) => term1_reg_518_reg_n_117,
      PCOUT(35) => term1_reg_518_reg_n_118,
      PCOUT(34) => term1_reg_518_reg_n_119,
      PCOUT(33) => term1_reg_518_reg_n_120,
      PCOUT(32) => term1_reg_518_reg_n_121,
      PCOUT(31) => term1_reg_518_reg_n_122,
      PCOUT(30) => term1_reg_518_reg_n_123,
      PCOUT(29) => term1_reg_518_reg_n_124,
      PCOUT(28) => term1_reg_518_reg_n_125,
      PCOUT(27) => term1_reg_518_reg_n_126,
      PCOUT(26) => term1_reg_518_reg_n_127,
      PCOUT(25) => term1_reg_518_reg_n_128,
      PCOUT(24) => term1_reg_518_reg_n_129,
      PCOUT(23) => term1_reg_518_reg_n_130,
      PCOUT(22) => term1_reg_518_reg_n_131,
      PCOUT(21) => term1_reg_518_reg_n_132,
      PCOUT(20) => term1_reg_518_reg_n_133,
      PCOUT(19) => term1_reg_518_reg_n_134,
      PCOUT(18) => term1_reg_518_reg_n_135,
      PCOUT(17) => term1_reg_518_reg_n_136,
      PCOUT(16) => term1_reg_518_reg_n_137,
      PCOUT(15) => term1_reg_518_reg_n_138,
      PCOUT(14) => term1_reg_518_reg_n_139,
      PCOUT(13) => term1_reg_518_reg_n_140,
      PCOUT(12) => term1_reg_518_reg_n_141,
      PCOUT(11) => term1_reg_518_reg_n_142,
      PCOUT(10) => term1_reg_518_reg_n_143,
      PCOUT(9) => term1_reg_518_reg_n_144,
      PCOUT(8) => term1_reg_518_reg_n_145,
      PCOUT(7) => term1_reg_518_reg_n_146,
      PCOUT(6) => term1_reg_518_reg_n_147,
      PCOUT(5) => term1_reg_518_reg_n_148,
      PCOUT(4) => term1_reg_518_reg_n_149,
      PCOUT(3) => term1_reg_518_reg_n_150,
      PCOUT(2) => term1_reg_518_reg_n_151,
      PCOUT(1) => term1_reg_518_reg_n_152,
      PCOUT(0) => term1_reg_518_reg_n_153,
      S(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_6,
      S(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_7,
      S(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_8,
      ap_block_pp0_stage0_11001 => ap_block_pp0_stage0_11001,
      ap_clk => ap_clk,
      \counter_reg[12]\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_2,
      \counter_reg[16]\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_4,
      \counter_reg[28]\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_5,
      \counter_reg[8]\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_3,
      \out\(31 downto 0) => counter_reg(31 downto 0),
      p_reg_reg => mac_mulsub_16s_16s_32s_32_4_1_U2_n_36,
      p_reg_reg_0 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_37,
      p_reg_reg_1 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_39,
      p_reg_reg_10 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_48,
      p_reg_reg_11 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_49,
      p_reg_reg_12 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_50,
      p_reg_reg_13 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_51,
      p_reg_reg_14 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_52,
      p_reg_reg_15 => regslice_both_out_stream_V_data_V_U_n_6,
      p_reg_reg_2 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_40,
      p_reg_reg_3 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_41,
      p_reg_reg_4 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_42,
      p_reg_reg_5 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_43,
      p_reg_reg_6 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_44,
      p_reg_reg_7 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_45,
      p_reg_reg_8 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_46,
      p_reg_reg_9 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_47,
      sum_i_reg(15 downto 0) => sum_i_reg(31 downto 16),
      \sum_i_reg[22]\(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_9,
      \sum_i_reg[22]\(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_10,
      \sum_i_reg[22]\(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_11,
      \sum_i_reg[22]\(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_12,
      \sum_i_reg[26]\(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_13,
      \sum_i_reg[26]\(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_14,
      \sum_i_reg[26]\(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_15,
      \sum_i_reg[26]\(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_16,
      \sum_i_reg[30]\(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_17,
      \sum_i_reg[30]\(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_18,
      \sum_i_reg[30]\(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_19,
      \sum_i_reg[30]\(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_20,
      sum_q_reg(15 downto 0) => sum_q_reg(31 downto 16),
      \sum_q_reg[18]\(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_21,
      \sum_q_reg[18]\(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_22,
      \sum_q_reg[18]\(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_23,
      \sum_q_reg[22]\(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_24,
      \sum_q_reg[22]\(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_25,
      \sum_q_reg[22]\(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_26,
      \sum_q_reg[22]\(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_27,
      \sum_q_reg[26]\(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_28,
      \sum_q_reg[26]\(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_29,
      \sum_q_reg[26]\(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_30,
      \sum_q_reg[26]\(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_31,
      \sum_q_reg[30]\(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_32,
      \sum_q_reg[30]\(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_33,
      \sum_q_reg[30]\(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_34,
      \sum_q_reg[30]\(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_35
    );
\pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TLAST_int_regslice,
      Q => \pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2_n_0\
    );
\pkt_last_V_reg_489_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_last_V_reg_489_pp0_iter1_reg_reg[0]_srl2_n_0\,
      Q => pkt_last_V_reg_489_pp0_iter2_reg,
      R => '0'
    );
regslice_both_in_stream_V_data_V_U: entity work.system_fsk_discriminator_0_0_fsk_discriminator_regslice_both
     port map (
      A(15) => regslice_both_in_stream_V_data_V_U_n_19,
      A(14) => regslice_both_in_stream_V_data_V_U_n_20,
      A(13) => regslice_both_in_stream_V_data_V_U_n_21,
      A(12) => regslice_both_in_stream_V_data_V_U_n_22,
      A(11) => regslice_both_in_stream_V_data_V_U_n_23,
      A(10) => regslice_both_in_stream_V_data_V_U_n_24,
      A(9) => regslice_both_in_stream_V_data_V_U_n_25,
      A(8) => regslice_both_in_stream_V_data_V_U_n_26,
      A(7) => regslice_both_in_stream_V_data_V_U_n_27,
      A(6) => regslice_both_in_stream_V_data_V_U_n_28,
      A(5) => regslice_both_in_stream_V_data_V_U_n_29,
      A(4) => regslice_both_in_stream_V_data_V_U_n_30,
      A(3) => regslice_both_in_stream_V_data_V_U_n_31,
      A(2) => regslice_both_in_stream_V_data_V_U_n_32,
      A(1) => regslice_both_in_stream_V_data_V_U_n_33,
      A(0) => regslice_both_in_stream_V_data_V_U_n_34,
      B(15 downto 0) => q_curr_fu_351_p3(15 downto 0),
      \B_V_data_1_state_reg[1]_0\ => in_stream_TREADY,
      \B_V_data_1_state_reg[1]_1\ => regslice_both_out_stream_V_data_V_U_n_2,
      O(3) => regslice_both_in_stream_V_data_V_U_n_35,
      O(2) => regslice_both_in_stream_V_data_V_U_n_36,
      O(1) => regslice_both_in_stream_V_data_V_U_n_37,
      O(0) => regslice_both_in_stream_V_data_V_U_n_38,
      S(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_6,
      S(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_7,
      S(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_8,
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      in_stream_TDATA(31 downto 0) => in_stream_TDATA(31 downto 0),
      in_stream_TVALID => in_stream_TVALID,
      in_stream_TVALID_int_regslice => in_stream_TVALID_int_regslice,
      p_reg_reg(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_9,
      p_reg_reg(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_10,
      p_reg_reg(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_11,
      p_reg_reg(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_12,
      p_reg_reg_0(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_13,
      p_reg_reg_0(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_14,
      p_reg_reg_0(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_15,
      p_reg_reg_0(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_16,
      p_reg_reg_1(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_21,
      p_reg_reg_1(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_22,
      p_reg_reg_1(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_23,
      p_reg_reg_2(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_24,
      p_reg_reg_2(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_25,
      p_reg_reg_2(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_26,
      p_reg_reg_2(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_27,
      p_reg_reg_3(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_28,
      p_reg_reg_3(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_29,
      p_reg_reg_3(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_30,
      p_reg_reg_3(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_31,
      sum_i_reg(31 downto 0) => sum_i_reg(31 downto 0),
      \sum_i_reg[11]\(3) => regslice_both_in_stream_V_data_V_U_n_43,
      \sum_i_reg[11]\(2) => regslice_both_in_stream_V_data_V_U_n_44,
      \sum_i_reg[11]\(1) => regslice_both_in_stream_V_data_V_U_n_45,
      \sum_i_reg[11]\(0) => regslice_both_in_stream_V_data_V_U_n_46,
      \sum_i_reg[15]\(3) => regslice_both_in_stream_V_data_V_U_n_47,
      \sum_i_reg[15]\(2) => regslice_both_in_stream_V_data_V_U_n_48,
      \sum_i_reg[15]\(1) => regslice_both_in_stream_V_data_V_U_n_49,
      \sum_i_reg[15]\(0) => regslice_both_in_stream_V_data_V_U_n_50,
      \sum_i_reg[19]\(3) => regslice_both_in_stream_V_data_V_U_n_51,
      \sum_i_reg[19]\(2) => regslice_both_in_stream_V_data_V_U_n_52,
      \sum_i_reg[19]\(1) => regslice_both_in_stream_V_data_V_U_n_53,
      \sum_i_reg[19]\(0) => regslice_both_in_stream_V_data_V_U_n_54,
      \sum_i_reg[23]\(3) => regslice_both_in_stream_V_data_V_U_n_55,
      \sum_i_reg[23]\(2) => regslice_both_in_stream_V_data_V_U_n_56,
      \sum_i_reg[23]\(1) => regslice_both_in_stream_V_data_V_U_n_57,
      \sum_i_reg[23]\(0) => regslice_both_in_stream_V_data_V_U_n_58,
      \sum_i_reg[27]\(3) => regslice_both_in_stream_V_data_V_U_n_59,
      \sum_i_reg[27]\(2) => regslice_both_in_stream_V_data_V_U_n_60,
      \sum_i_reg[27]\(1) => regslice_both_in_stream_V_data_V_U_n_61,
      \sum_i_reg[27]\(0) => regslice_both_in_stream_V_data_V_U_n_62,
      \sum_i_reg[30]\(3) => regslice_both_in_stream_V_data_V_U_n_63,
      \sum_i_reg[30]\(2) => regslice_both_in_stream_V_data_V_U_n_64,
      \sum_i_reg[30]\(1) => regslice_both_in_stream_V_data_V_U_n_65,
      \sum_i_reg[30]\(0) => regslice_both_in_stream_V_data_V_U_n_66,
      \sum_i_reg[7]\(3) => regslice_both_in_stream_V_data_V_U_n_39,
      \sum_i_reg[7]\(2) => regslice_both_in_stream_V_data_V_U_n_40,
      \sum_i_reg[7]\(1) => regslice_both_in_stream_V_data_V_U_n_41,
      \sum_i_reg[7]\(0) => regslice_both_in_stream_V_data_V_U_n_42,
      sum_q_reg(31 downto 0) => sum_q_reg(31 downto 0),
      \sum_q_reg[11]\(3) => regslice_both_in_stream_V_data_V_U_n_75,
      \sum_q_reg[11]\(2) => regslice_both_in_stream_V_data_V_U_n_76,
      \sum_q_reg[11]\(1) => regslice_both_in_stream_V_data_V_U_n_77,
      \sum_q_reg[11]\(0) => regslice_both_in_stream_V_data_V_U_n_78,
      \sum_q_reg[15]\(3) => regslice_both_in_stream_V_data_V_U_n_79,
      \sum_q_reg[15]\(2) => regslice_both_in_stream_V_data_V_U_n_80,
      \sum_q_reg[15]\(1) => regslice_both_in_stream_V_data_V_U_n_81,
      \sum_q_reg[15]\(0) => regslice_both_in_stream_V_data_V_U_n_82,
      \sum_q_reg[19]\(3) => regslice_both_in_stream_V_data_V_U_n_83,
      \sum_q_reg[19]\(2) => regslice_both_in_stream_V_data_V_U_n_84,
      \sum_q_reg[19]\(1) => regslice_both_in_stream_V_data_V_U_n_85,
      \sum_q_reg[19]\(0) => regslice_both_in_stream_V_data_V_U_n_86,
      \sum_q_reg[23]\(3) => regslice_both_in_stream_V_data_V_U_n_87,
      \sum_q_reg[23]\(2) => regslice_both_in_stream_V_data_V_U_n_88,
      \sum_q_reg[23]\(1) => regslice_both_in_stream_V_data_V_U_n_89,
      \sum_q_reg[23]\(0) => regslice_both_in_stream_V_data_V_U_n_90,
      \sum_q_reg[27]\(3) => regslice_both_in_stream_V_data_V_U_n_91,
      \sum_q_reg[27]\(2) => regslice_both_in_stream_V_data_V_U_n_92,
      \sum_q_reg[27]\(1) => regslice_both_in_stream_V_data_V_U_n_93,
      \sum_q_reg[27]\(0) => regslice_both_in_stream_V_data_V_U_n_94,
      \sum_q_reg[30]\(3) => regslice_both_in_stream_V_data_V_U_n_95,
      \sum_q_reg[30]\(2) => regslice_both_in_stream_V_data_V_U_n_96,
      \sum_q_reg[30]\(1) => regslice_both_in_stream_V_data_V_U_n_97,
      \sum_q_reg[30]\(0) => regslice_both_in_stream_V_data_V_U_n_98,
      \sum_q_reg[3]\(3) => regslice_both_in_stream_V_data_V_U_n_67,
      \sum_q_reg[3]\(2) => regslice_both_in_stream_V_data_V_U_n_68,
      \sum_q_reg[3]\(1) => regslice_both_in_stream_V_data_V_U_n_69,
      \sum_q_reg[3]\(0) => regslice_both_in_stream_V_data_V_U_n_70,
      \sum_q_reg[7]\(3) => regslice_both_in_stream_V_data_V_U_n_71,
      \sum_q_reg[7]\(2) => regslice_both_in_stream_V_data_V_U_n_72,
      \sum_q_reg[7]\(1) => regslice_both_in_stream_V_data_V_U_n_73,
      \sum_q_reg[7]\(0) => regslice_both_in_stream_V_data_V_U_n_74,
      term1_reg_518_reg_i_70_0(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_32,
      term1_reg_518_reg_i_70_0(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_33,
      term1_reg_518_reg_i_70_0(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_34,
      term1_reg_518_reg_i_70_0(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_35,
      term1_reg_518_reg_i_99_0(3) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_17,
      term1_reg_518_reg_i_99_0(2) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_18,
      term1_reg_518_reg_i_99_0(1) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_19,
      term1_reg_518_reg_i_99_0(0) => mac_mulsub_16s_16s_32s_32_4_1_U2_n_20
    );
regslice_both_in_stream_V_last_V_U: entity work.\system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1\
     port map (
      \B_V_data_1_state_reg[1]_0\ => regslice_both_out_stream_V_data_V_U_n_2,
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      in_stream_TLAST(0) => in_stream_TLAST(0),
      in_stream_TLAST_int_regslice => in_stream_TLAST_int_regslice,
      in_stream_TVALID => in_stream_TVALID
    );
regslice_both_out_stream_V_data_V_U: entity work.\system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized2\
     port map (
      \B_V_data_1_payload_A_reg[0]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_52,
      \B_V_data_1_payload_A_reg[10]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_42,
      \B_V_data_1_payload_A_reg[11]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_41,
      \B_V_data_1_payload_A_reg[12]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_40,
      \B_V_data_1_payload_A_reg[13]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_39,
      \B_V_data_1_payload_A_reg[14]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_37,
      \B_V_data_1_payload_A_reg[15]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_36,
      \B_V_data_1_payload_A_reg[1]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_51,
      \B_V_data_1_payload_A_reg[2]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_50,
      \B_V_data_1_payload_A_reg[3]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_49,
      \B_V_data_1_payload_A_reg[4]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_48,
      \B_V_data_1_payload_A_reg[5]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_47,
      \B_V_data_1_payload_A_reg[6]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_46,
      \B_V_data_1_payload_A_reg[7]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_45,
      \B_V_data_1_payload_A_reg[8]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_44,
      \B_V_data_1_payload_A_reg[9]_0\ => mac_mulsub_16s_16s_32s_32_4_1_U2_n_43,
      \B_V_data_1_state_reg[0]_0\ => out_stream_TVALID,
      \B_V_data_1_state_reg[0]_1\ => regslice_both_out_stream_V_data_V_U_n_2,
      \B_V_data_1_state_reg[0]_2\ => regslice_both_out_stream_V_data_V_U_n_6,
      CO(0) => icmp_ln23_2_fu_441_p2,
      P(1) => tmp_2_fu_426_p4(10),
      P(0) => tmp_2_fu_426_p4(0),
      ap_block_pp0_stage0_11001 => ap_block_pp0_stage0_11001,
      ap_clk => ap_clk,
      ap_enable_reg_pp0_iter1 => ap_enable_reg_pp0_iter1,
      ap_enable_reg_pp0_iter3 => ap_enable_reg_pp0_iter3,
      ap_enable_reg_pp0_iter4 => ap_enable_reg_pp0_iter4,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      i_old0 => i_old0,
      icmp_ln52_reg_494 => icmp_ln52_reg_494,
      icmp_ln52_reg_494_pp0_iter2_reg => icmp_ln52_reg_494_pp0_iter2_reg,
      icmp_ln52_reg_494_pp0_iter3_reg => icmp_ln52_reg_494_pp0_iter3_reg,
      \icmp_ln52_reg_494_reg[0]\ => regslice_both_out_stream_V_data_V_U_n_5,
      in_stream_TVALID_int_regslice => in_stream_TVALID_int_regslice,
      out_stream_TDATA(15 downto 0) => out_stream_TDATA(15 downto 0),
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TVALID_int_regslice => out_stream_TVALID_int_regslice,
      p_reg_reg => mac_mulsub_16s_16s_32s_32_4_1_U2_n_3,
      p_reg_reg_0 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_2,
      p_reg_reg_1 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_5,
      p_reg_reg_2 => mac_mulsub_16s_16s_32s_32_4_1_U2_n_4
    );
regslice_both_out_stream_V_last_V_U: entity work.\system_fsk_discriminator_0_0_fsk_discriminator_regslice_both__parameterized1_0\
     port map (
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      out_stream_TLAST(0) => out_stream_TLAST(0),
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TVALID_int_regslice => out_stream_TVALID_int_regslice,
      pkt_last_V_reg_489_pp0_iter2_reg => pkt_last_V_reg_489_pp0_iter2_reg
    );
\sum_i_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_38,
      Q => sum_i_reg(0),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_44,
      Q => sum_i_reg(10),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_43,
      Q => sum_i_reg(11),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_50,
      Q => sum_i_reg(12),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_49,
      Q => sum_i_reg(13),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_48,
      Q => sum_i_reg(14),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_47,
      Q => sum_i_reg(15),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[16]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_54,
      Q => sum_i_reg(16),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[17]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_53,
      Q => sum_i_reg(17),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[18]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_52,
      Q => sum_i_reg(18),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[19]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_51,
      Q => sum_i_reg(19),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_37,
      Q => sum_i_reg(1),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[20]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_58,
      Q => sum_i_reg(20),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[21]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_57,
      Q => sum_i_reg(21),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[22]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_56,
      Q => sum_i_reg(22),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[23]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_55,
      Q => sum_i_reg(23),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[24]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_62,
      Q => sum_i_reg(24),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[25]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_61,
      Q => sum_i_reg(25),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[26]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_60,
      Q => sum_i_reg(26),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[27]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_59,
      Q => sum_i_reg(27),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[28]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_66,
      Q => sum_i_reg(28),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[29]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_65,
      Q => sum_i_reg(29),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_36,
      Q => sum_i_reg(2),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[30]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_64,
      Q => sum_i_reg(30),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[31]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_63,
      Q => sum_i_reg(31),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_35,
      Q => sum_i_reg(3),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_42,
      Q => sum_i_reg(4),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_41,
      Q => sum_i_reg(5),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_40,
      Q => sum_i_reg(6),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_39,
      Q => sum_i_reg(7),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_46,
      Q => sum_i_reg(8),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_i_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_45,
      Q => sum_i_reg(9),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_70,
      Q => sum_q_reg(0),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_76,
      Q => sum_q_reg(10),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_75,
      Q => sum_q_reg(11),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_82,
      Q => sum_q_reg(12),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_81,
      Q => sum_q_reg(13),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_80,
      Q => sum_q_reg(14),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_79,
      Q => sum_q_reg(15),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[16]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_86,
      Q => sum_q_reg(16),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[17]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_85,
      Q => sum_q_reg(17),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[18]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_84,
      Q => sum_q_reg(18),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[19]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_83,
      Q => sum_q_reg(19),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_69,
      Q => sum_q_reg(1),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[20]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_90,
      Q => sum_q_reg(20),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[21]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_89,
      Q => sum_q_reg(21),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[22]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_88,
      Q => sum_q_reg(22),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[23]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_87,
      Q => sum_q_reg(23),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[24]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_94,
      Q => sum_q_reg(24),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[25]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_93,
      Q => sum_q_reg(25),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[26]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_92,
      Q => sum_q_reg(26),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[27]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_91,
      Q => sum_q_reg(27),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[28]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_98,
      Q => sum_q_reg(28),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[29]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_97,
      Q => sum_q_reg(29),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_68,
      Q => sum_q_reg(2),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[30]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_96,
      Q => sum_q_reg(30),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[31]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_95,
      Q => sum_q_reg(31),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_67,
      Q => sum_q_reg(3),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_74,
      Q => sum_q_reg(4),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_73,
      Q => sum_q_reg(5),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_72,
      Q => sum_q_reg(6),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_71,
      Q => sum_q_reg(7),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_78,
      Q => sum_q_reg(8),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
\sum_q_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => regslice_both_in_stream_V_data_V_U_n_77,
      Q => sum_q_reg(9),
      R => regslice_both_out_stream_V_data_V_U_n_6
    );
term1_reg_518_reg: unisim.vcomponents.DSP48E1
    generic map(
      ACASCREG => 2,
      ADREG => 1,
      ALUMODEREG => 0,
      AREG => 2,
      AUTORESET_PATDET => "NO_RESET",
      A_INPUT => "DIRECT",
      BCASCREG => 1,
      BREG => 1,
      B_INPUT => "DIRECT",
      CARRYINREG => 0,
      CARRYINSELREG => 0,
      CREG => 0,
      DREG => 1,
      INMODEREG => 0,
      MASK => X"3FFFFFFFFFFF",
      MREG => 0,
      OPMODEREG => 0,
      PATTERN => X"000000000000",
      PREG => 1,
      SEL_MASK => "MASK",
      SEL_PATTERN => "PATTERN",
      USE_DPORT => false,
      USE_MULT => "MULTIPLY",
      USE_PATTERN_DETECT => "NO_PATDET",
      USE_SIMD => "ONE48"
    )
        port map (
      A(29) => regslice_both_in_stream_V_data_V_U_n_19,
      A(28) => regslice_both_in_stream_V_data_V_U_n_19,
      A(27) => regslice_both_in_stream_V_data_V_U_n_19,
      A(26) => regslice_both_in_stream_V_data_V_U_n_19,
      A(25) => regslice_both_in_stream_V_data_V_U_n_19,
      A(24) => regslice_both_in_stream_V_data_V_U_n_19,
      A(23) => regslice_both_in_stream_V_data_V_U_n_19,
      A(22) => regslice_both_in_stream_V_data_V_U_n_19,
      A(21) => regslice_both_in_stream_V_data_V_U_n_19,
      A(20) => regslice_both_in_stream_V_data_V_U_n_19,
      A(19) => regslice_both_in_stream_V_data_V_U_n_19,
      A(18) => regslice_both_in_stream_V_data_V_U_n_19,
      A(17) => regslice_both_in_stream_V_data_V_U_n_19,
      A(16) => regslice_both_in_stream_V_data_V_U_n_19,
      A(15) => regslice_both_in_stream_V_data_V_U_n_19,
      A(14) => regslice_both_in_stream_V_data_V_U_n_20,
      A(13) => regslice_both_in_stream_V_data_V_U_n_21,
      A(12) => regslice_both_in_stream_V_data_V_U_n_22,
      A(11) => regslice_both_in_stream_V_data_V_U_n_23,
      A(10) => regslice_both_in_stream_V_data_V_U_n_24,
      A(9) => regslice_both_in_stream_V_data_V_U_n_25,
      A(8) => regslice_both_in_stream_V_data_V_U_n_26,
      A(7) => regslice_both_in_stream_V_data_V_U_n_27,
      A(6) => regslice_both_in_stream_V_data_V_U_n_28,
      A(5) => regslice_both_in_stream_V_data_V_U_n_29,
      A(4) => regslice_both_in_stream_V_data_V_U_n_30,
      A(3) => regslice_both_in_stream_V_data_V_U_n_31,
      A(2) => regslice_both_in_stream_V_data_V_U_n_32,
      A(1) => regslice_both_in_stream_V_data_V_U_n_33,
      A(0) => regslice_both_in_stream_V_data_V_U_n_34,
      ACIN(29 downto 0) => B"000000000000000000000000000000",
      ACOUT(29 downto 0) => NLW_term1_reg_518_reg_ACOUT_UNCONNECTED(29 downto 0),
      ALUMODE(3 downto 0) => B"0000",
      B(17) => q_curr_fu_351_p3(15),
      B(16) => q_curr_fu_351_p3(15),
      B(15 downto 0) => q_curr_fu_351_p3(15 downto 0),
      BCIN(17 downto 0) => B"000000000000000000",
      BCOUT(17 downto 0) => NLW_term1_reg_518_reg_BCOUT_UNCONNECTED(17 downto 0),
      C(47 downto 0) => B"111111111111111111111111111111111111111111111111",
      CARRYCASCIN => '0',
      CARRYCASCOUT => NLW_term1_reg_518_reg_CARRYCASCOUT_UNCONNECTED,
      CARRYIN => '0',
      CARRYINSEL(2 downto 0) => B"000",
      CARRYOUT(3 downto 0) => NLW_term1_reg_518_reg_CARRYOUT_UNCONNECTED(3 downto 0),
      CEA1 => regslice_both_out_stream_V_data_V_U_n_6,
      CEA2 => i_old0,
      CEAD => '0',
      CEALUMODE => '0',
      CEB1 => '0',
      CEB2 => regslice_both_out_stream_V_data_V_U_n_6,
      CEC => '0',
      CECARRYIN => '0',
      CECTRL => '0',
      CED => '0',
      CEINMODE => '0',
      CEM => '0',
      CEP => regslice_both_out_stream_V_data_V_U_n_5,
      CLK => ap_clk,
      D(24 downto 0) => B"0000000000000000000000000",
      INMODE(4 downto 0) => B"00000",
      MULTSIGNIN => '0',
      MULTSIGNOUT => NLW_term1_reg_518_reg_MULTSIGNOUT_UNCONNECTED,
      OPMODE(6 downto 0) => B"0000101",
      OVERFLOW => NLW_term1_reg_518_reg_OVERFLOW_UNCONNECTED,
      P(47 downto 0) => NLW_term1_reg_518_reg_P_UNCONNECTED(47 downto 0),
      PATTERNBDETECT => NLW_term1_reg_518_reg_PATTERNBDETECT_UNCONNECTED,
      PATTERNDETECT => NLW_term1_reg_518_reg_PATTERNDETECT_UNCONNECTED,
      PCIN(47 downto 0) => B"000000000000000000000000000000000000000000000000",
      PCOUT(47) => term1_reg_518_reg_n_106,
      PCOUT(46) => term1_reg_518_reg_n_107,
      PCOUT(45) => term1_reg_518_reg_n_108,
      PCOUT(44) => term1_reg_518_reg_n_109,
      PCOUT(43) => term1_reg_518_reg_n_110,
      PCOUT(42) => term1_reg_518_reg_n_111,
      PCOUT(41) => term1_reg_518_reg_n_112,
      PCOUT(40) => term1_reg_518_reg_n_113,
      PCOUT(39) => term1_reg_518_reg_n_114,
      PCOUT(38) => term1_reg_518_reg_n_115,
      PCOUT(37) => term1_reg_518_reg_n_116,
      PCOUT(36) => term1_reg_518_reg_n_117,
      PCOUT(35) => term1_reg_518_reg_n_118,
      PCOUT(34) => term1_reg_518_reg_n_119,
      PCOUT(33) => term1_reg_518_reg_n_120,
      PCOUT(32) => term1_reg_518_reg_n_121,
      PCOUT(31) => term1_reg_518_reg_n_122,
      PCOUT(30) => term1_reg_518_reg_n_123,
      PCOUT(29) => term1_reg_518_reg_n_124,
      PCOUT(28) => term1_reg_518_reg_n_125,
      PCOUT(27) => term1_reg_518_reg_n_126,
      PCOUT(26) => term1_reg_518_reg_n_127,
      PCOUT(25) => term1_reg_518_reg_n_128,
      PCOUT(24) => term1_reg_518_reg_n_129,
      PCOUT(23) => term1_reg_518_reg_n_130,
      PCOUT(22) => term1_reg_518_reg_n_131,
      PCOUT(21) => term1_reg_518_reg_n_132,
      PCOUT(20) => term1_reg_518_reg_n_133,
      PCOUT(19) => term1_reg_518_reg_n_134,
      PCOUT(18) => term1_reg_518_reg_n_135,
      PCOUT(17) => term1_reg_518_reg_n_136,
      PCOUT(16) => term1_reg_518_reg_n_137,
      PCOUT(15) => term1_reg_518_reg_n_138,
      PCOUT(14) => term1_reg_518_reg_n_139,
      PCOUT(13) => term1_reg_518_reg_n_140,
      PCOUT(12) => term1_reg_518_reg_n_141,
      PCOUT(11) => term1_reg_518_reg_n_142,
      PCOUT(10) => term1_reg_518_reg_n_143,
      PCOUT(9) => term1_reg_518_reg_n_144,
      PCOUT(8) => term1_reg_518_reg_n_145,
      PCOUT(7) => term1_reg_518_reg_n_146,
      PCOUT(6) => term1_reg_518_reg_n_147,
      PCOUT(5) => term1_reg_518_reg_n_148,
      PCOUT(4) => term1_reg_518_reg_n_149,
      PCOUT(3) => term1_reg_518_reg_n_150,
      PCOUT(2) => term1_reg_518_reg_n_151,
      PCOUT(1) => term1_reg_518_reg_n_152,
      PCOUT(0) => term1_reg_518_reg_n_153,
      RSTA => '0',
      RSTALLCARRYIN => '0',
      RSTALUMODE => '0',
      RSTB => '0',
      RSTC => '0',
      RSTCTRL => '0',
      RSTD => '0',
      RSTINMODE => '0',
      RSTM => '0',
      RSTP => '0',
      UNDERFLOW => NLW_term1_reg_518_reg_UNDERFLOW_UNCONNECTED
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity system_fsk_discriminator_0_0 is
  port (
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TREADY : out STD_LOGIC;
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    in_stream_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    in_stream_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    out_stream_TVALID : out STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 15 downto 0 );
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TKEEP : out STD_LOGIC_VECTOR ( 1 downto 0 );
    out_stream_TSTRB : out STD_LOGIC_VECTOR ( 1 downto 0 )
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of system_fsk_discriminator_0_0 : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of system_fsk_discriminator_0_0 : entity is "system_fsk_discriminator_0_0,fsk_discriminator,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of system_fsk_discriminator_0_0 : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of system_fsk_discriminator_0_0 : entity is "HLS";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of system_fsk_discriminator_0_0 : entity is "fsk_discriminator,Vivado 2023.1";
  attribute hls_module : string;
  attribute hls_module of system_fsk_discriminator_0_0 : entity is "yes";
end system_fsk_discriminator_0_0;

architecture STRUCTURE of system_fsk_discriminator_0_0 is
  signal \<const0>\ : STD_LOGIC;
  signal \<const1>\ : STD_LOGIC;
  signal NLW_inst_out_stream_TKEEP_UNCONNECTED : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal NLW_inst_out_stream_TSTRB_UNCONNECTED : STD_LOGIC_VECTOR ( 1 downto 0 );
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
  attribute X_INTERFACE_PARAMETER of in_stream_TSTRB : signal is "XIL_INTERFACENAME in_stream, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of out_stream_TDATA : signal is "xilinx.com:interface:axis:1.0 out_stream TDATA";
  attribute X_INTERFACE_INFO of out_stream_TKEEP : signal is "xilinx.com:interface:axis:1.0 out_stream TKEEP";
  attribute X_INTERFACE_INFO of out_stream_TLAST : signal is "xilinx.com:interface:axis:1.0 out_stream TLAST";
  attribute X_INTERFACE_INFO of out_stream_TSTRB : signal is "xilinx.com:interface:axis:1.0 out_stream TSTRB";
  attribute X_INTERFACE_PARAMETER of out_stream_TSTRB : signal is "XIL_INTERFACENAME out_stream, TDATA_NUM_BYTES 2, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
begin
  out_stream_TKEEP(1) <= \<const1>\;
  out_stream_TKEEP(0) <= \<const1>\;
  out_stream_TSTRB(1) <= \<const0>\;
  out_stream_TSTRB(0) <= \<const0>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
VCC: unisim.vcomponents.VCC
     port map (
      P => \<const1>\
    );
inst: entity work.system_fsk_discriminator_0_0_fsk_discriminator
     port map (
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      in_stream_TDATA(31 downto 0) => in_stream_TDATA(31 downto 0),
      in_stream_TKEEP(3 downto 0) => B"0000",
      in_stream_TLAST(0) => in_stream_TLAST(0),
      in_stream_TREADY => in_stream_TREADY,
      in_stream_TSTRB(3 downto 0) => B"0000",
      in_stream_TVALID => in_stream_TVALID,
      out_stream_TDATA(15 downto 0) => out_stream_TDATA(15 downto 0),
      out_stream_TKEEP(1 downto 0) => NLW_inst_out_stream_TKEEP_UNCONNECTED(1 downto 0),
      out_stream_TLAST(0) => out_stream_TLAST(0),
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TSTRB(1 downto 0) => NLW_inst_out_stream_TSTRB_UNCONNECTED(1 downto 0),
      out_stream_TVALID => out_stream_TVALID
    );
end STRUCTURE;
