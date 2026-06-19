-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Wed Jan  7 12:00:53 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_axi_pattern_gen_0_0_sim_netlist.vhdl
-- Design      : system_axi_pattern_gen_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_pattern_gen is
  port (
    Q : out STD_LOGIC_VECTOR ( 12 downto 0 );
    m_axis_tdata : out STD_LOGIC_VECTOR ( 0 to 0 );
    m_axis_tvalid : out STD_LOGIC;
    m_axis_tready : in STD_LOGIC;
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_pattern_gen;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_pattern_gen is
  signal \^q\ : STD_LOGIC_VECTOR ( 12 downto 0 );
  signal \bit_cntr[0]_i_1_n_0\ : STD_LOGIC;
  signal \bit_cntr[1]_i_1_n_0\ : STD_LOGIC;
  signal \bit_cntr[2]_i_1_n_0\ : STD_LOGIC;
  signal \bit_cntr[2]_i_2_n_0\ : STD_LOGIC;
  signal \bit_cntr[2]_i_3_n_0\ : STD_LOGIC;
  signal \bit_cntr_reg_n_0_[0]\ : STD_LOGIC;
  signal \bit_cntr_reg_n_0_[1]\ : STD_LOGIC;
  signal \bit_cntr_reg_n_0_[2]\ : STD_LOGIC;
  signal \^m_axis_tvalid\ : STD_LOGIC;
  signal m_axis_tvalid_i_1_n_0 : STD_LOGIC;
  signal sample_counter : STD_LOGIC_VECTOR ( 12 downto 0 );
  signal \sample_counter0_carry__0_n_0\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_1\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_2\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_3\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_4\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_5\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_6\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_7\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_1\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_2\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_3\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_4\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_5\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_6\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_7\ : STD_LOGIC;
  signal sample_counter0_carry_n_0 : STD_LOGIC;
  signal sample_counter0_carry_n_1 : STD_LOGIC;
  signal sample_counter0_carry_n_2 : STD_LOGIC;
  signal sample_counter0_carry_n_3 : STD_LOGIC;
  signal sample_counter0_carry_n_4 : STD_LOGIC;
  signal sample_counter0_carry_n_5 : STD_LOGIC;
  signal sample_counter0_carry_n_6 : STD_LOGIC;
  signal sample_counter0_carry_n_7 : STD_LOGIC;
  signal \sample_counter[12]_i_2_n_0\ : STD_LOGIC;
  signal \sample_counter[12]_i_3_n_0\ : STD_LOGIC;
  signal \sample_counter[12]_i_4_n_0\ : STD_LOGIC;
  signal \sample_counter[12]_i_5_n_0\ : STD_LOGIC;
  signal \temp_cntr[0]_i_1_n_0\ : STD_LOGIC;
  signal \temp_cntr[1]_i_1_n_0\ : STD_LOGIC;
  signal \temp_cntr[2]_i_1_n_0\ : STD_LOGIC;
  signal \temp_cntr[2]_i_2_n_0\ : STD_LOGIC;
  signal \temp_cntr[3]_i_1_n_0\ : STD_LOGIC;
  signal \temp_cntr[4]_i_1_n_0\ : STD_LOGIC;
  signal \temp_cntr[5]_i_1_n_0\ : STD_LOGIC;
  signal \temp_cntr[6]_i_1_n_0\ : STD_LOGIC;
  signal \temp_cntr[7]_i_1_n_0\ : STD_LOGIC;
  signal \temp_cntr[7]_i_2_n_0\ : STD_LOGIC;
  signal \temp_cntr[7]_i_3_n_0\ : STD_LOGIC;
  signal \temp_cntr_reg_n_0_[0]\ : STD_LOGIC;
  signal \temp_cntr_reg_n_0_[1]\ : STD_LOGIC;
  signal \temp_cntr_reg_n_0_[2]\ : STD_LOGIC;
  signal \temp_cntr_reg_n_0_[3]\ : STD_LOGIC;
  signal \temp_cntr_reg_n_0_[4]\ : STD_LOGIC;
  signal \temp_cntr_reg_n_0_[5]\ : STD_LOGIC;
  signal \temp_cntr_reg_n_0_[6]\ : STD_LOGIC;
  signal \temp_cntr_reg_n_0_[7]\ : STD_LOGIC;
  signal \NLW_sample_counter0_carry__1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \bit_cntr[1]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \bit_cntr[2]_i_2\ : label is "soft_lutpair3";
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of sample_counter0_carry : label is 35;
  attribute ADDER_THRESHOLD of \sample_counter0_carry__0\ : label is 35;
  attribute ADDER_THRESHOLD of \sample_counter0_carry__1\ : label is 35;
  attribute SOFT_HLUTNM of \sample_counter[0]_i_1\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \sample_counter[12]_i_1\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \temp_cntr[0]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \temp_cntr[2]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \temp_cntr[3]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \temp_cntr[4]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \temp_cntr[6]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \temp_cntr[7]_i_2\ : label is "soft_lutpair4";
begin
  Q(12 downto 0) <= \^q\(12 downto 0);
  m_axis_tvalid <= \^m_axis_tvalid\;
\bit_cntr[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \bit_cntr_reg_n_0_[0]\,
      O => \bit_cntr[0]_i_1_n_0\
    );
\bit_cntr[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => \bit_cntr_reg_n_0_[1]\,
      I1 => \bit_cntr_reg_n_0_[0]\,
      O => \bit_cntr[1]_i_1_n_0\
    );
\bit_cntr[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"04"
    )
        port map (
      I0 => \^q\(0),
      I1 => m_axis_tready,
      I2 => \bit_cntr[2]_i_3_n_0\,
      O => \bit_cntr[2]_i_1_n_0\
    );
\bit_cntr[2]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A9"
    )
        port map (
      I0 => \bit_cntr_reg_n_0_[2]\,
      I1 => \bit_cntr_reg_n_0_[0]\,
      I2 => \bit_cntr_reg_n_0_[1]\,
      O => \bit_cntr[2]_i_2_n_0\
    );
\bit_cntr[2]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFFBFFF"
    )
        port map (
      I0 => \sample_counter[12]_i_5_n_0\,
      I1 => \^q\(6),
      I2 => \^q\(5),
      I3 => \^q\(7),
      I4 => \^q\(8),
      I5 => \sample_counter[12]_i_3_n_0\,
      O => \bit_cntr[2]_i_3_n_0\
    );
\bit_cntr_reg[0]\: unisim.vcomponents.FDPE
     port map (
      C => aclk,
      CE => \bit_cntr[2]_i_1_n_0\,
      D => \bit_cntr[0]_i_1_n_0\,
      PRE => \sample_counter[12]_i_2_n_0\,
      Q => \bit_cntr_reg_n_0_[0]\
    );
\bit_cntr_reg[1]\: unisim.vcomponents.FDPE
     port map (
      C => aclk,
      CE => \bit_cntr[2]_i_1_n_0\,
      D => \bit_cntr[1]_i_1_n_0\,
      PRE => \sample_counter[12]_i_2_n_0\,
      Q => \bit_cntr_reg_n_0_[1]\
    );
\bit_cntr_reg[2]\: unisim.vcomponents.FDPE
     port map (
      C => aclk,
      CE => \bit_cntr[2]_i_1_n_0\,
      D => \bit_cntr[2]_i_2_n_0\,
      PRE => \sample_counter[12]_i_2_n_0\,
      Q => \bit_cntr_reg_n_0_[2]\
    );
\m_axis_tdata[0]_INST_0\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"080505050DA805AF"
    )
        port map (
      I0 => \bit_cntr_reg_n_0_[2]\,
      I1 => \bit_cntr_reg_n_0_[0]\,
      I2 => \temp_cntr_reg_n_0_[2]\,
      I3 => \temp_cntr_reg_n_0_[0]\,
      I4 => \bit_cntr_reg_n_0_[1]\,
      I5 => \temp_cntr_reg_n_0_[1]\,
      O => m_axis_tdata(0)
    );
m_axis_tvalid_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"F8"
    )
        port map (
      I0 => aresetn,
      I1 => m_axis_tready,
      I2 => \^m_axis_tvalid\,
      O => m_axis_tvalid_i_1_n_0
    );
m_axis_tvalid_reg: unisim.vcomponents.FDRE
     port map (
      C => aclk,
      CE => '1',
      D => m_axis_tvalid_i_1_n_0,
      Q => \^m_axis_tvalid\,
      R => '0'
    );
sample_counter0_carry: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => sample_counter0_carry_n_0,
      CO(2) => sample_counter0_carry_n_1,
      CO(1) => sample_counter0_carry_n_2,
      CO(0) => sample_counter0_carry_n_3,
      CYINIT => \^q\(0),
      DI(3 downto 0) => B"0000",
      O(3) => sample_counter0_carry_n_4,
      O(2) => sample_counter0_carry_n_5,
      O(1) => sample_counter0_carry_n_6,
      O(0) => sample_counter0_carry_n_7,
      S(3 downto 0) => \^q\(4 downto 1)
    );
\sample_counter0_carry__0\: unisim.vcomponents.CARRY4
     port map (
      CI => sample_counter0_carry_n_0,
      CO(3) => \sample_counter0_carry__0_n_0\,
      CO(2) => \sample_counter0_carry__0_n_1\,
      CO(1) => \sample_counter0_carry__0_n_2\,
      CO(0) => \sample_counter0_carry__0_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \sample_counter0_carry__0_n_4\,
      O(2) => \sample_counter0_carry__0_n_5\,
      O(1) => \sample_counter0_carry__0_n_6\,
      O(0) => \sample_counter0_carry__0_n_7\,
      S(3 downto 0) => \^q\(8 downto 5)
    );
\sample_counter0_carry__1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sample_counter0_carry__0_n_0\,
      CO(3) => \NLW_sample_counter0_carry__1_CO_UNCONNECTED\(3),
      CO(2) => \sample_counter0_carry__1_n_1\,
      CO(1) => \sample_counter0_carry__1_n_2\,
      CO(0) => \sample_counter0_carry__1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \sample_counter0_carry__1_n_4\,
      O(2) => \sample_counter0_carry__1_n_5\,
      O(1) => \sample_counter0_carry__1_n_6\,
      O(0) => \sample_counter0_carry__1_n_7\,
      S(3 downto 0) => \^q\(12 downto 9)
    );
\sample_counter[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^q\(0),
      O => sample_counter(0)
    );
\sample_counter[10]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => \sample_counter0_carry__1_n_6\,
      O => sample_counter(10)
    );
\sample_counter[11]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => \sample_counter0_carry__1_n_5\,
      O => sample_counter(11)
    );
\sample_counter[12]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => \sample_counter0_carry__1_n_4\,
      O => sample_counter(12)
    );
\sample_counter[12]_i_2\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => aresetn,
      O => \sample_counter[12]_i_2_n_0\
    );
\sample_counter[12]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => \^q\(2),
      I1 => \^q\(1),
      I2 => \^q\(4),
      I3 => \^q\(3),
      O => \sample_counter[12]_i_3_n_0\
    );
\sample_counter[12]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FF7F"
    )
        port map (
      I0 => \^q\(6),
      I1 => \^q\(5),
      I2 => \^q\(7),
      I3 => \^q\(8),
      O => \sample_counter[12]_i_4_n_0\
    );
\sample_counter[12]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"EFFF"
    )
        port map (
      I0 => \^q\(10),
      I1 => \^q\(9),
      I2 => \^q\(12),
      I3 => \^q\(11),
      O => \sample_counter[12]_i_5_n_0\
    );
\sample_counter[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => sample_counter0_carry_n_7,
      O => sample_counter(1)
    );
\sample_counter[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => sample_counter0_carry_n_6,
      O => sample_counter(2)
    );
\sample_counter[3]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => sample_counter0_carry_n_5,
      O => sample_counter(3)
    );
\sample_counter[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => sample_counter0_carry_n_4,
      O => sample_counter(4)
    );
\sample_counter[5]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => \sample_counter0_carry__0_n_7\,
      O => sample_counter(5)
    );
\sample_counter[6]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => \sample_counter0_carry__0_n_6\,
      O => sample_counter(6)
    );
\sample_counter[7]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => \sample_counter0_carry__0_n_5\,
      O => sample_counter(7)
    );
\sample_counter[8]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => \sample_counter0_carry__0_n_4\,
      O => sample_counter(8)
    );
\sample_counter[9]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFF0000"
    )
        port map (
      I0 => \sample_counter[12]_i_3_n_0\,
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => \sample_counter[12]_i_5_n_0\,
      I3 => \^q\(0),
      I4 => \sample_counter0_carry__1_n_7\,
      O => sample_counter(9)
    );
\sample_counter_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(0),
      Q => \^q\(0)
    );
\sample_counter_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(10),
      Q => \^q\(10)
    );
\sample_counter_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(11),
      Q => \^q\(11)
    );
\sample_counter_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(12),
      Q => \^q\(12)
    );
\sample_counter_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(1),
      Q => \^q\(1)
    );
\sample_counter_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(2),
      Q => \^q\(2)
    );
\sample_counter_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(3),
      Q => \^q\(3)
    );
\sample_counter_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(4),
      Q => \^q\(4)
    );
\sample_counter_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(5),
      Q => \^q\(5)
    );
\sample_counter_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(6),
      Q => \^q\(6)
    );
\sample_counter_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(7),
      Q => \^q\(7)
    );
\sample_counter_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(8),
      Q => \^q\(8)
    );
\sample_counter_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => m_axis_tready,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => sample_counter(9),
      Q => \^q\(9)
    );
\temp_cntr[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0F0B"
    )
        port map (
      I0 => \temp_cntr_reg_n_0_[1]\,
      I1 => \temp_cntr_reg_n_0_[2]\,
      I2 => \temp_cntr_reg_n_0_[0]\,
      I3 => \temp_cntr[2]_i_2_n_0\,
      O => \temp_cntr[0]_i_1_n_0\
    );
\temp_cntr[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \temp_cntr_reg_n_0_[0]\,
      I1 => \temp_cntr_reg_n_0_[1]\,
      O => \temp_cntr[1]_i_1_n_0\
    );
\temp_cntr[2]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"3CE0"
    )
        port map (
      I0 => \temp_cntr[2]_i_2_n_0\,
      I1 => \temp_cntr_reg_n_0_[0]\,
      I2 => \temp_cntr_reg_n_0_[2]\,
      I3 => \temp_cntr_reg_n_0_[1]\,
      O => \temp_cntr[2]_i_1_n_0\
    );
\temp_cntr[2]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFFFFFE"
    )
        port map (
      I0 => \temp_cntr_reg_n_0_[3]\,
      I1 => \temp_cntr_reg_n_0_[6]\,
      I2 => \temp_cntr_reg_n_0_[7]\,
      I3 => \temp_cntr_reg_n_0_[5]\,
      I4 => \temp_cntr_reg_n_0_[4]\,
      O => \temp_cntr[2]_i_2_n_0\
    );
\temp_cntr[3]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7F80"
    )
        port map (
      I0 => \temp_cntr_reg_n_0_[0]\,
      I1 => \temp_cntr_reg_n_0_[1]\,
      I2 => \temp_cntr_reg_n_0_[2]\,
      I3 => \temp_cntr_reg_n_0_[3]\,
      O => \temp_cntr[3]_i_1_n_0\
    );
\temp_cntr[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFF8000"
    )
        port map (
      I0 => \temp_cntr_reg_n_0_[3]\,
      I1 => \temp_cntr_reg_n_0_[2]\,
      I2 => \temp_cntr_reg_n_0_[1]\,
      I3 => \temp_cntr_reg_n_0_[0]\,
      I4 => \temp_cntr_reg_n_0_[4]\,
      O => \temp_cntr[4]_i_1_n_0\
    );
\temp_cntr[5]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"7FFFFFFF80000000"
    )
        port map (
      I0 => \temp_cntr_reg_n_0_[4]\,
      I1 => \temp_cntr_reg_n_0_[0]\,
      I2 => \temp_cntr_reg_n_0_[1]\,
      I3 => \temp_cntr_reg_n_0_[2]\,
      I4 => \temp_cntr_reg_n_0_[3]\,
      I5 => \temp_cntr_reg_n_0_[5]\,
      O => \temp_cntr[5]_i_1_n_0\
    );
\temp_cntr[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => \temp_cntr[7]_i_3_n_0\,
      I1 => \temp_cntr_reg_n_0_[6]\,
      O => \temp_cntr[6]_i_1_n_0\
    );
\temp_cntr[7]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000100"
    )
        port map (
      I0 => \bit_cntr_reg_n_0_[1]\,
      I1 => \bit_cntr_reg_n_0_[0]\,
      I2 => \bit_cntr_reg_n_0_[2]\,
      I3 => m_axis_tready,
      I4 => \^q\(0),
      I5 => \bit_cntr[2]_i_3_n_0\,
      O => \temp_cntr[7]_i_1_n_0\
    );
\temp_cntr[7]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"D2"
    )
        port map (
      I0 => \temp_cntr_reg_n_0_[6]\,
      I1 => \temp_cntr[7]_i_3_n_0\,
      I2 => \temp_cntr_reg_n_0_[7]\,
      O => \temp_cntr[7]_i_2_n_0\
    );
\temp_cntr[7]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"7FFFFFFFFFFFFFFF"
    )
        port map (
      I0 => \temp_cntr_reg_n_0_[4]\,
      I1 => \temp_cntr_reg_n_0_[0]\,
      I2 => \temp_cntr_reg_n_0_[1]\,
      I3 => \temp_cntr_reg_n_0_[2]\,
      I4 => \temp_cntr_reg_n_0_[3]\,
      I5 => \temp_cntr_reg_n_0_[5]\,
      O => \temp_cntr[7]_i_3_n_0\
    );
\temp_cntr_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \temp_cntr[7]_i_1_n_0\,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => \temp_cntr[0]_i_1_n_0\,
      Q => \temp_cntr_reg_n_0_[0]\
    );
\temp_cntr_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \temp_cntr[7]_i_1_n_0\,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => \temp_cntr[1]_i_1_n_0\,
      Q => \temp_cntr_reg_n_0_[1]\
    );
\temp_cntr_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \temp_cntr[7]_i_1_n_0\,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => \temp_cntr[2]_i_1_n_0\,
      Q => \temp_cntr_reg_n_0_[2]\
    );
\temp_cntr_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \temp_cntr[7]_i_1_n_0\,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => \temp_cntr[3]_i_1_n_0\,
      Q => \temp_cntr_reg_n_0_[3]\
    );
\temp_cntr_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \temp_cntr[7]_i_1_n_0\,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => \temp_cntr[4]_i_1_n_0\,
      Q => \temp_cntr_reg_n_0_[4]\
    );
\temp_cntr_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \temp_cntr[7]_i_1_n_0\,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => \temp_cntr[5]_i_1_n_0\,
      Q => \temp_cntr_reg_n_0_[5]\
    );
\temp_cntr_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \temp_cntr[7]_i_1_n_0\,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => \temp_cntr[6]_i_1_n_0\,
      Q => \temp_cntr_reg_n_0_[6]\
    );
\temp_cntr_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \temp_cntr[7]_i_1_n_0\,
      CLR => \sample_counter[12]_i_2_n_0\,
      D => \temp_cntr[7]_i_2_n_0\,
      Q => \temp_cntr_reg_n_0_[7]\
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  port (
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC;
    m_axis_tdata : out STD_LOGIC_VECTOR ( 7 downto 0 );
    m_axis_tuser : out STD_LOGIC_VECTOR ( 12 downto 0 );
    m_axis_tvalid : out STD_LOGIC;
    m_axis_tready : in STD_LOGIC;
    m_axis_tlast : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "system_axi_pattern_gen_0_0,axi_pattern_gen,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "module_ref";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "axi_pattern_gen,Vivado 2023.1";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  signal \<const0>\ : STD_LOGIC;
  signal \^m_axis_tdata\ : STD_LOGIC_VECTOR ( 0 to 0 );
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of aclk : signal is "xilinx.com:signal:clock:1.0 aclk CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of aclk : signal is "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF m_axis, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of aresetn : signal is "xilinx.com:signal:reset:1.0 aresetn RST";
  attribute X_INTERFACE_PARAMETER of aresetn : signal is "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of m_axis_tlast : signal is "xilinx.com:interface:axis:1.0 m_axis TLAST";
  attribute X_INTERFACE_PARAMETER of m_axis_tlast : signal is "XIL_INTERFACENAME m_axis, TDATA_NUM_BYTES 1, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 13, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 0, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, LAYERED_METADATA undef, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of m_axis_tready : signal is "xilinx.com:interface:axis:1.0 m_axis TREADY";
  attribute X_INTERFACE_INFO of m_axis_tvalid : signal is "xilinx.com:interface:axis:1.0 m_axis TVALID";
  attribute X_INTERFACE_INFO of m_axis_tdata : signal is "xilinx.com:interface:axis:1.0 m_axis TDATA";
  attribute X_INTERFACE_INFO of m_axis_tuser : signal is "xilinx.com:interface:axis:1.0 m_axis TUSER";
begin
  m_axis_tdata(7) <= \<const0>\;
  m_axis_tdata(6) <= \<const0>\;
  m_axis_tdata(5) <= \<const0>\;
  m_axis_tdata(4) <= \<const0>\;
  m_axis_tdata(3) <= \<const0>\;
  m_axis_tdata(2) <= \<const0>\;
  m_axis_tdata(1) <= \<const0>\;
  m_axis_tdata(0) <= \^m_axis_tdata\(0);
  m_axis_tlast <= \<const0>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_pattern_gen
     port map (
      Q(12 downto 0) => m_axis_tuser(12 downto 0),
      aclk => aclk,
      aresetn => aresetn,
      m_axis_tdata(0) => \^m_axis_tdata\(0),
      m_axis_tready => m_axis_tready,
      m_axis_tvalid => m_axis_tvalid
    );
end STRUCTURE;
