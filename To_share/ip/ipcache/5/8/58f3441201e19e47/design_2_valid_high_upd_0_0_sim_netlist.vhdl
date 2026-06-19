-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Wed Jan  7 13:01:28 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_2_valid_high_upd_0_0_sim_netlist.vhdl
-- Design      : design_2_valid_high_upd_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_valid_high_upd is
  port (
    m_axis_tuser : out STD_LOGIC_VECTOR ( 12 downto 0 );
    m_axis_tdata : out STD_LOGIC_VECTOR ( 7 downto 0 );
    m_axis_tvalid : out STD_LOGIC;
    s_axis_tvalid : in STD_LOGIC;
    s_axis_tdata : in STD_LOGIC_VECTOR ( 7 downto 0 );
    aclk : in STD_LOGIC;
    m_axis_tready : in STD_LOGIC;
    aresetn : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_valid_high_upd;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_valid_high_upd is
  signal \^m_axis_tuser\ : STD_LOGIC_VECTOR ( 12 downto 0 );
  signal sample_counter0 : STD_LOGIC_VECTOR ( 12 downto 1 );
  signal \sample_counter0_carry__0_n_0\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_1\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_2\ : STD_LOGIC;
  signal \sample_counter0_carry__0_n_3\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_1\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_2\ : STD_LOGIC;
  signal \sample_counter0_carry__1_n_3\ : STD_LOGIC;
  signal sample_counter0_carry_n_0 : STD_LOGIC;
  signal sample_counter0_carry_n_1 : STD_LOGIC;
  signal sample_counter0_carry_n_2 : STD_LOGIC;
  signal sample_counter0_carry_n_3 : STD_LOGIC;
  signal \sample_counter[11]_i_2_n_0\ : STD_LOGIC;
  signal \sample_counter[11]_i_3_n_0\ : STD_LOGIC;
  signal \sample_counter[11]_i_4_n_0\ : STD_LOGIC;
  signal \sample_counter[11]_i_5_n_0\ : STD_LOGIC;
  signal \sample_counter[12]_i_1_n_0\ : STD_LOGIC;
  signal \sample_counter[12]_i_3_n_0\ : STD_LOGIC;
  signal \sample_counter[12]_i_4_n_0\ : STD_LOGIC;
  signal \sample_counter[12]_i_5_n_0\ : STD_LOGIC;
  signal \sample_counter[12]_i_6_n_0\ : STD_LOGIC;
  signal \sample_counter[3]_i_2_n_0\ : STD_LOGIC;
  signal \sample_counter[3]_i_3_n_0\ : STD_LOGIC;
  signal \sample_counter[3]_i_4_n_0\ : STD_LOGIC;
  signal \sample_counter[3]_i_5_n_0\ : STD_LOGIC;
  signal \sample_counter[3]_i_6_n_0\ : STD_LOGIC;
  signal \sample_counter[7]_i_2_n_0\ : STD_LOGIC;
  signal \sample_counter[7]_i_3_n_0\ : STD_LOGIC;
  signal \sample_counter[7]_i_4_n_0\ : STD_LOGIC;
  signal \sample_counter[7]_i_5_n_0\ : STD_LOGIC;
  signal \sample_counter_reg[11]_i_1_n_0\ : STD_LOGIC;
  signal \sample_counter_reg[11]_i_1_n_1\ : STD_LOGIC;
  signal \sample_counter_reg[11]_i_1_n_2\ : STD_LOGIC;
  signal \sample_counter_reg[11]_i_1_n_3\ : STD_LOGIC;
  signal \sample_counter_reg[11]_i_1_n_4\ : STD_LOGIC;
  signal \sample_counter_reg[11]_i_1_n_5\ : STD_LOGIC;
  signal \sample_counter_reg[11]_i_1_n_6\ : STD_LOGIC;
  signal \sample_counter_reg[11]_i_1_n_7\ : STD_LOGIC;
  signal \sample_counter_reg[12]_i_2_n_7\ : STD_LOGIC;
  signal \sample_counter_reg[3]_i_1_n_0\ : STD_LOGIC;
  signal \sample_counter_reg[3]_i_1_n_1\ : STD_LOGIC;
  signal \sample_counter_reg[3]_i_1_n_2\ : STD_LOGIC;
  signal \sample_counter_reg[3]_i_1_n_3\ : STD_LOGIC;
  signal \sample_counter_reg[3]_i_1_n_4\ : STD_LOGIC;
  signal \sample_counter_reg[3]_i_1_n_5\ : STD_LOGIC;
  signal \sample_counter_reg[3]_i_1_n_6\ : STD_LOGIC;
  signal \sample_counter_reg[3]_i_1_n_7\ : STD_LOGIC;
  signal \sample_counter_reg[7]_i_1_n_0\ : STD_LOGIC;
  signal \sample_counter_reg[7]_i_1_n_1\ : STD_LOGIC;
  signal \sample_counter_reg[7]_i_1_n_2\ : STD_LOGIC;
  signal \sample_counter_reg[7]_i_1_n_3\ : STD_LOGIC;
  signal \sample_counter_reg[7]_i_1_n_4\ : STD_LOGIC;
  signal \sample_counter_reg[7]_i_1_n_5\ : STD_LOGIC;
  signal \sample_counter_reg[7]_i_1_n_6\ : STD_LOGIC;
  signal \sample_counter_reg[7]_i_1_n_7\ : STD_LOGIC;
  signal start_cntr_i_1_n_0 : STD_LOGIC;
  signal start_cntr_reg_n_0 : STD_LOGIC;
  signal \temp_data[7]_i_1_n_0\ : STD_LOGIC;
  signal \NLW_sample_counter0_carry__1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_sample_counter_reg[12]_i_2_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_sample_counter_reg[12]_i_2_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 1 );
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of sample_counter0_carry : label is 35;
  attribute ADDER_THRESHOLD of \sample_counter0_carry__0\ : label is 35;
  attribute ADDER_THRESHOLD of \sample_counter0_carry__1\ : label is 35;
  attribute ADDER_THRESHOLD of \sample_counter_reg[11]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sample_counter_reg[12]_i_2\ : label is 11;
  attribute ADDER_THRESHOLD of \sample_counter_reg[3]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sample_counter_reg[7]_i_1\ : label is 11;
begin
  m_axis_tuser(12 downto 0) <= \^m_axis_tuser\(12 downto 0);
sample_counter0_carry: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => sample_counter0_carry_n_0,
      CO(2) => sample_counter0_carry_n_1,
      CO(1) => sample_counter0_carry_n_2,
      CO(0) => sample_counter0_carry_n_3,
      CYINIT => \^m_axis_tuser\(0),
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => sample_counter0(4 downto 1),
      S(3 downto 0) => \^m_axis_tuser\(4 downto 1)
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
      O(3 downto 0) => sample_counter0(8 downto 5),
      S(3 downto 0) => \^m_axis_tuser\(8 downto 5)
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
      O(3 downto 0) => sample_counter0(12 downto 9),
      S(3 downto 0) => \^m_axis_tuser\(12 downto 9)
    );
\sample_counter[11]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(11),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(11),
      O => \sample_counter[11]_i_2_n_0\
    );
\sample_counter[11]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(10),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(10),
      O => \sample_counter[11]_i_3_n_0\
    );
\sample_counter[11]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(9),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(9),
      O => \sample_counter[11]_i_4_n_0\
    );
\sample_counter[11]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(8),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(8),
      O => \sample_counter[11]_i_5_n_0\
    );
\sample_counter[12]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"E0"
    )
        port map (
      I0 => start_cntr_reg_n_0,
      I1 => s_axis_tvalid,
      I2 => m_axis_tready,
      O => \sample_counter[12]_i_1_n_0\
    );
\sample_counter[12]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(12),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(12),
      O => \sample_counter[12]_i_3_n_0\
    );
\sample_counter[12]_i_4\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"EFFFFFFFFFFFFFFF"
    )
        port map (
      I0 => \sample_counter[12]_i_5_n_0\,
      I1 => \sample_counter[12]_i_6_n_0\,
      I2 => \^m_axis_tuser\(7),
      I3 => \^m_axis_tuser\(1),
      I4 => \^m_axis_tuser\(5),
      I5 => \^m_axis_tuser\(6),
      O => \sample_counter[12]_i_4_n_0\
    );
\sample_counter[12]_i_5\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BFFFFFFF"
    )
        port map (
      I0 => \^m_axis_tuser\(9),
      I1 => \^m_axis_tuser\(2),
      I2 => \^m_axis_tuser\(0),
      I3 => \^m_axis_tuser\(11),
      I4 => \^m_axis_tuser\(12),
      O => \sample_counter[12]_i_5_n_0\
    );
\sample_counter[12]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF7"
    )
        port map (
      I0 => \^m_axis_tuser\(3),
      I1 => \^m_axis_tuser\(4),
      I2 => \^m_axis_tuser\(8),
      I3 => \^m_axis_tuser\(10),
      O => \sample_counter[12]_i_6_n_0\
    );
\sample_counter[3]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"38"
    )
        port map (
      I0 => \sample_counter[12]_i_4_n_0\,
      I1 => start_cntr_reg_n_0,
      I2 => \^m_axis_tuser\(0),
      O => \sample_counter[3]_i_2_n_0\
    );
\sample_counter[3]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(3),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(3),
      O => \sample_counter[3]_i_3_n_0\
    );
\sample_counter[3]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(2),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(2),
      O => \sample_counter[3]_i_4_n_0\
    );
\sample_counter[3]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(1),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(1),
      O => \sample_counter[3]_i_5_n_0\
    );
\sample_counter[3]_i_6\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"45"
    )
        port map (
      I0 => \^m_axis_tuser\(0),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      O => \sample_counter[3]_i_6_n_0\
    );
\sample_counter[7]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(7),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(7),
      O => \sample_counter[7]_i_2_n_0\
    );
\sample_counter[7]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(6),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(6),
      O => \sample_counter[7]_i_3_n_0\
    );
\sample_counter[7]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(5),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(5),
      O => \sample_counter[7]_i_4_n_0\
    );
\sample_counter[7]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8F80"
    )
        port map (
      I0 => sample_counter0(4),
      I1 => \sample_counter[12]_i_4_n_0\,
      I2 => start_cntr_reg_n_0,
      I3 => \^m_axis_tuser\(4),
      O => \sample_counter[7]_i_5_n_0\
    );
\sample_counter_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[3]_i_1_n_7\,
      Q => \^m_axis_tuser\(0)
    );
\sample_counter_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[11]_i_1_n_5\,
      Q => \^m_axis_tuser\(10)
    );
\sample_counter_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[11]_i_1_n_4\,
      Q => \^m_axis_tuser\(11)
    );
\sample_counter_reg[11]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sample_counter_reg[7]_i_1_n_0\,
      CO(3) => \sample_counter_reg[11]_i_1_n_0\,
      CO(2) => \sample_counter_reg[11]_i_1_n_1\,
      CO(1) => \sample_counter_reg[11]_i_1_n_2\,
      CO(0) => \sample_counter_reg[11]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \sample_counter_reg[11]_i_1_n_4\,
      O(2) => \sample_counter_reg[11]_i_1_n_5\,
      O(1) => \sample_counter_reg[11]_i_1_n_6\,
      O(0) => \sample_counter_reg[11]_i_1_n_7\,
      S(3) => \sample_counter[11]_i_2_n_0\,
      S(2) => \sample_counter[11]_i_3_n_0\,
      S(1) => \sample_counter[11]_i_4_n_0\,
      S(0) => \sample_counter[11]_i_5_n_0\
    );
\sample_counter_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[12]_i_2_n_7\,
      Q => \^m_axis_tuser\(12)
    );
\sample_counter_reg[12]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \sample_counter_reg[11]_i_1_n_0\,
      CO(3 downto 0) => \NLW_sample_counter_reg[12]_i_2_CO_UNCONNECTED\(3 downto 0),
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 1) => \NLW_sample_counter_reg[12]_i_2_O_UNCONNECTED\(3 downto 1),
      O(0) => \sample_counter_reg[12]_i_2_n_7\,
      S(3 downto 1) => B"000",
      S(0) => \sample_counter[12]_i_3_n_0\
    );
\sample_counter_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[3]_i_1_n_6\,
      Q => \^m_axis_tuser\(1)
    );
\sample_counter_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[3]_i_1_n_5\,
      Q => \^m_axis_tuser\(2)
    );
\sample_counter_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[3]_i_1_n_4\,
      Q => \^m_axis_tuser\(3)
    );
\sample_counter_reg[3]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \sample_counter_reg[3]_i_1_n_0\,
      CO(2) => \sample_counter_reg[3]_i_1_n_1\,
      CO(1) => \sample_counter_reg[3]_i_1_n_2\,
      CO(0) => \sample_counter_reg[3]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 1) => B"000",
      DI(0) => \sample_counter[3]_i_2_n_0\,
      O(3) => \sample_counter_reg[3]_i_1_n_4\,
      O(2) => \sample_counter_reg[3]_i_1_n_5\,
      O(1) => \sample_counter_reg[3]_i_1_n_6\,
      O(0) => \sample_counter_reg[3]_i_1_n_7\,
      S(3) => \sample_counter[3]_i_3_n_0\,
      S(2) => \sample_counter[3]_i_4_n_0\,
      S(1) => \sample_counter[3]_i_5_n_0\,
      S(0) => \sample_counter[3]_i_6_n_0\
    );
\sample_counter_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[7]_i_1_n_7\,
      Q => \^m_axis_tuser\(4)
    );
\sample_counter_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[7]_i_1_n_6\,
      Q => \^m_axis_tuser\(5)
    );
\sample_counter_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[7]_i_1_n_5\,
      Q => \^m_axis_tuser\(6)
    );
\sample_counter_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[7]_i_1_n_4\,
      Q => \^m_axis_tuser\(7)
    );
\sample_counter_reg[7]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sample_counter_reg[3]_i_1_n_0\,
      CO(3) => \sample_counter_reg[7]_i_1_n_0\,
      CO(2) => \sample_counter_reg[7]_i_1_n_1\,
      CO(1) => \sample_counter_reg[7]_i_1_n_2\,
      CO(0) => \sample_counter_reg[7]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \sample_counter_reg[7]_i_1_n_4\,
      O(2) => \sample_counter_reg[7]_i_1_n_5\,
      O(1) => \sample_counter_reg[7]_i_1_n_6\,
      O(0) => \sample_counter_reg[7]_i_1_n_7\,
      S(3) => \sample_counter[7]_i_2_n_0\,
      S(2) => \sample_counter[7]_i_3_n_0\,
      S(1) => \sample_counter[7]_i_4_n_0\,
      S(0) => \sample_counter[7]_i_5_n_0\
    );
\sample_counter_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[11]_i_1_n_7\,
      Q => \^m_axis_tuser\(8)
    );
\sample_counter_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => \sample_counter[12]_i_1_n_0\,
      CLR => \temp_data[7]_i_1_n_0\,
      D => \sample_counter_reg[11]_i_1_n_6\,
      Q => \^m_axis_tuser\(9)
    );
start_cntr_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"F8"
    )
        port map (
      I0 => m_axis_tready,
      I1 => s_axis_tvalid,
      I2 => start_cntr_reg_n_0,
      O => start_cntr_i_1_n_0
    );
start_cntr_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => '1',
      CLR => \temp_data[7]_i_1_n_0\,
      D => start_cntr_i_1_n_0,
      Q => start_cntr_reg_n_0
    );
\temp_data[7]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => aresetn,
      O => \temp_data[7]_i_1_n_0\
    );
\temp_data_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => s_axis_tdata(0),
      Q => m_axis_tdata(0)
    );
\temp_data_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => s_axis_tdata(1),
      Q => m_axis_tdata(1)
    );
\temp_data_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => s_axis_tdata(2),
      Q => m_axis_tdata(2)
    );
\temp_data_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => s_axis_tdata(3),
      Q => m_axis_tdata(3)
    );
\temp_data_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => s_axis_tdata(4),
      Q => m_axis_tdata(4)
    );
\temp_data_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => s_axis_tdata(5),
      Q => m_axis_tdata(5)
    );
\temp_data_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => s_axis_tdata(6),
      Q => m_axis_tdata(6)
    );
\temp_data_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => s_axis_tdata(7),
      Q => m_axis_tdata(7)
    );
temp_valid_reg: unisim.vcomponents.FDCE
     port map (
      C => aclk,
      CE => s_axis_tvalid,
      CLR => \temp_data[7]_i_1_n_0\,
      D => '1',
      Q => m_axis_tvalid
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
    s_axis_tdata : in STD_LOGIC_VECTOR ( 7 downto 0 );
    s_axis_tvalid : in STD_LOGIC;
    m_axis_tdata : out STD_LOGIC_VECTOR ( 7 downto 0 );
    m_axis_tuser : out STD_LOGIC_VECTOR ( 12 downto 0 );
    m_axis_tvalid : out STD_LOGIC;
    m_axis_tready : in STD_LOGIC;
    m_axis_tlast : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "design_2_valid_high_upd_0_0,valid_high_upd,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "module_ref";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "valid_high_upd,Vivado 2023.1";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  signal \<const0>\ : STD_LOGIC;
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of aclk : signal is "xilinx.com:signal:clock:1.0 aclk CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of aclk : signal is "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF m_axis:s_axis, ASSOCIATED_RESET aresetn, FREQ_HZ 61440000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of aresetn : signal is "xilinx.com:signal:reset:1.0 aresetn RST";
  attribute X_INTERFACE_PARAMETER of aresetn : signal is "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of m_axis_tlast : signal is "xilinx.com:interface:axis:1.0 m_axis TLAST";
  attribute X_INTERFACE_PARAMETER of m_axis_tlast : signal is "XIL_INTERFACENAME m_axis, TDATA_NUM_BYTES 1, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 13, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 0, HAS_TLAST 1, FREQ_HZ 61440000, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, LAYERED_METADATA undef, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of m_axis_tready : signal is "xilinx.com:interface:axis:1.0 m_axis TREADY";
  attribute X_INTERFACE_INFO of m_axis_tvalid : signal is "xilinx.com:interface:axis:1.0 m_axis TVALID";
  attribute X_INTERFACE_INFO of s_axis_tvalid : signal is "xilinx.com:interface:axis:1.0 s_axis TVALID";
  attribute X_INTERFACE_PARAMETER of s_axis_tvalid : signal is "XIL_INTERFACENAME s_axis, TDATA_NUM_BYTES 1, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 0, HAS_TSTRB 0, HAS_TKEEP 0, HAS_TLAST 0, FREQ_HZ 61440000, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, LAYERED_METADATA undef, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of m_axis_tdata : signal is "xilinx.com:interface:axis:1.0 m_axis TDATA";
  attribute X_INTERFACE_INFO of m_axis_tuser : signal is "xilinx.com:interface:axis:1.0 m_axis TUSER";
  attribute X_INTERFACE_INFO of s_axis_tdata : signal is "xilinx.com:interface:axis:1.0 s_axis TDATA";
begin
  m_axis_tlast <= \<const0>\;
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_valid_high_upd
     port map (
      aclk => aclk,
      aresetn => aresetn,
      m_axis_tdata(7 downto 0) => m_axis_tdata(7 downto 0),
      m_axis_tready => m_axis_tready,
      m_axis_tuser(12 downto 0) => m_axis_tuser(12 downto 0),
      m_axis_tvalid => m_axis_tvalid,
      s_axis_tdata(7 downto 0) => s_axis_tdata(7 downto 0),
      s_axis_tvalid => s_axis_tvalid
    );
end STRUCTURE;
