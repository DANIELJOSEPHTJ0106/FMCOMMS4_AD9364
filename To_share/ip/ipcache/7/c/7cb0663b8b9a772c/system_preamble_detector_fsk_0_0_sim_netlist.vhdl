-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Wed Jan  7 18:04:05 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_preamble_detector_fsk_0_0_sim_netlist.vhdl
-- Design      : system_preamble_detector_fsk_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr is
  port (
    reset_n_0 : out STD_LOGIC;
    D : out STD_LOGIC_VECTOR ( 1 downto 0 );
    pn_bit_reg_0 : out STD_LOGIC;
    \preamble_det_bit_count_reg[7]\ : out STD_LOGIC_VECTOR ( 7 downto 0 );
    clock : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    reset_prng : in STD_LOGIC;
    Q : in STD_LOGIC_VECTOR ( 3 downto 0 );
    bsync_val_in : in STD_LOGIC;
    \FSM_onehot_preamble_Stm_reg[0]\ : in STD_LOGIC;
    delay_3_indata : in STD_LOGIC;
    \preamble_det_bit_count_reg[7]_0\ : in STD_LOGIC_VECTOR ( 7 downto 0 );
    \preamble_det_bit_count_reg[5]\ : in STD_LOGIC;
    \preamble_det_bit_count_reg[7]_1\ : in STD_LOGIC;
    preamble_en : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr is
  signal \FSM_onehot_preamble_Stm[3]_i_3_n_0\ : STD_LOGIC;
  signal delay_cntr : STD_LOGIC_VECTOR ( 5 downto 0 );
  signal \delay_cntr[3]_i_1_n_0\ : STD_LOGIC;
  signal \delay_cntr[4]_i_2_n_0\ : STD_LOGIC;
  signal \delay_cntr[5]_i_1_n_0\ : STD_LOGIC;
  signal \delay_cntr[5]_i_2_n_0\ : STD_LOGIC;
  signal \delay_cntr[5]_i_3_n_0\ : STD_LOGIC;
  signal p_0_in : STD_LOGIC_VECTOR ( 4 downto 0 );
  signal p_0_in_0 : STD_LOGIC;
  signal pn_bit : STD_LOGIC;
  signal pn_bit_i_1_n_0 : STD_LOGIC;
  signal \^pn_bit_reg_0\ : STD_LOGIC;
  signal \^reset_n_0\ : STD_LOGIC;
  signal \shift_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \shift_reg[1]_i_1_n_0\ : STD_LOGIC;
  signal \shift_reg[2]_i_1_n_0\ : STD_LOGIC;
  signal \shift_reg[3]_i_1_n_0\ : STD_LOGIC;
  signal \shift_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \shift_reg[5]_i_1_n_0\ : STD_LOGIC;
  signal \shift_reg[6]_i_1_n_0\ : STD_LOGIC;
  signal \shift_reg[7]_i_1_n_0\ : STD_LOGIC;
  signal \shift_reg[7]_i_2_n_0\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[0]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[1]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[2]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[3]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[4]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[5]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[6]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_onehot_preamble_Stm[3]_i_3\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \delay_cntr[1]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \delay_cntr[2]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \delay_cntr[4]_i_2\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \delay_cntr[5]_i_2\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \preamble_det_bit_count[0]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \preamble_det_bit_count[1]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \preamble_det_bit_count[5]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \preamble_det_bit_count[6]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of reset_prng_reg_i_1 : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \shift_reg[1]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \shift_reg[2]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \shift_reg[3]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \shift_reg[4]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \shift_reg[5]_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \shift_reg[6]_i_1\ : label is "soft_lutpair7";
begin
  pn_bit_reg_0 <= \^pn_bit_reg_0\;
  reset_n_0 <= \^reset_n_0\;
\FSM_onehot_preamble_Stm[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFEFEAEFEAEFEA"
    )
        port map (
      I0 => \^pn_bit_reg_0\,
      I1 => Q(1),
      I2 => bsync_val_in,
      I3 => Q(0),
      I4 => Q(3),
      I5 => \FSM_onehot_preamble_Stm_reg[0]\,
      O => D(0)
    );
\FSM_onehot_preamble_Stm[3]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0100FFFF01000100"
    )
        port map (
      I0 => \preamble_det_bit_count_reg[7]_0\(7),
      I1 => \preamble_det_bit_count_reg[7]_1\,
      I2 => \preamble_det_bit_count_reg[7]_0\(6),
      I3 => \FSM_onehot_preamble_Stm[3]_i_3_n_0\,
      I4 => bsync_val_in,
      I5 => Q(2),
      O => D(1)
    );
\FSM_onehot_preamble_Stm[3]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"82"
    )
        port map (
      I0 => Q(3),
      I1 => pn_bit,
      I2 => delay_3_indata,
      O => \FSM_onehot_preamble_Stm[3]_i_3_n_0\
    );
delay_1_indata_i_1: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => reset_n,
      O => \^reset_n_0\
    );
\delay_cntr[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"15"
    )
        port map (
      I0 => delay_cntr(0),
      I1 => reset_n,
      I2 => reset_prng,
      O => p_0_in(0)
    );
\delay_cntr[1]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0666"
    )
        port map (
      I0 => delay_cntr(1),
      I1 => delay_cntr(0),
      I2 => reset_n,
      I3 => reset_prng,
      O => p_0_in(1)
    );
\delay_cntr[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00787878"
    )
        port map (
      I0 => delay_cntr(0),
      I1 => delay_cntr(1),
      I2 => delay_cntr(2),
      I3 => reset_n,
      I4 => reset_prng,
      O => p_0_in(2)
    );
\delay_cntr[3]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0777777770000000"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => delay_cntr(2),
      I3 => delay_cntr(1),
      I4 => delay_cntr(0),
      I5 => delay_cntr(3),
      O => \delay_cntr[3]_i_1_n_0\
    );
\delay_cntr[4]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"000000007FFF8000"
    )
        port map (
      I0 => delay_cntr(1),
      I1 => delay_cntr(0),
      I2 => delay_cntr(3),
      I3 => delay_cntr(2),
      I4 => delay_cntr(4),
      I5 => \delay_cntr[4]_i_2_n_0\,
      O => p_0_in(4)
    );
\delay_cntr[4]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => reset_prng,
      I1 => reset_n,
      O => \delay_cntr[4]_i_2_n_0\
    );
\delay_cntr[5]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"EA"
    )
        port map (
      I0 => preamble_en,
      I1 => reset_n,
      I2 => reset_prng,
      O => \delay_cntr[5]_i_1_n_0\
    );
\delay_cntr[5]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"07777000"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => delay_cntr(4),
      I3 => \delay_cntr[5]_i_3_n_0\,
      I4 => delay_cntr(5),
      O => \delay_cntr[5]_i_2_n_0\
    );
\delay_cntr[5]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8000"
    )
        port map (
      I0 => delay_cntr(1),
      I1 => delay_cntr(0),
      I2 => delay_cntr(3),
      I3 => delay_cntr(2),
      O => \delay_cntr[5]_i_3_n_0\
    );
\delay_cntr_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      CLR => \^reset_n_0\,
      D => p_0_in(0),
      Q => delay_cntr(0)
    );
\delay_cntr_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      CLR => \^reset_n_0\,
      D => p_0_in(1),
      Q => delay_cntr(1)
    );
\delay_cntr_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      CLR => \^reset_n_0\,
      D => p_0_in(2),
      Q => delay_cntr(2)
    );
\delay_cntr_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      CLR => \^reset_n_0\,
      D => \delay_cntr[3]_i_1_n_0\,
      Q => delay_cntr(3)
    );
\delay_cntr_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      CLR => \^reset_n_0\,
      D => p_0_in(4),
      Q => delay_cntr(4)
    );
\delay_cntr_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      CLR => \^reset_n_0\,
      D => \delay_cntr[5]_i_2_n_0\,
      Q => delay_cntr(5)
    );
pn_bit_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => p_0_in_0,
      I1 => preamble_en,
      I2 => pn_bit,
      O => pn_bit_i_1_n_0
    );
pn_bit_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => \^reset_n_0\,
      D => pn_bit_i_1_n_0,
      Q => pn_bit
    );
\preamble_det_bit_count[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0090"
    )
        port map (
      I0 => delay_3_indata,
      I1 => pn_bit,
      I2 => Q(3),
      I3 => \preamble_det_bit_count_reg[7]_0\(0),
      O => \preamble_det_bit_count_reg[7]\(0)
    );
\preamble_det_bit_count[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00828200"
    )
        port map (
      I0 => Q(3),
      I1 => pn_bit,
      I2 => delay_3_indata,
      I3 => \preamble_det_bit_count_reg[7]_0\(0),
      I4 => \preamble_det_bit_count_reg[7]_0\(1),
      O => \preamble_det_bit_count_reg[7]\(1)
    );
\preamble_det_bit_count[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0082828282000000"
    )
        port map (
      I0 => Q(3),
      I1 => pn_bit,
      I2 => delay_3_indata,
      I3 => \preamble_det_bit_count_reg[7]_0\(1),
      I4 => \preamble_det_bit_count_reg[7]_0\(0),
      I5 => \preamble_det_bit_count_reg[7]_0\(2),
      O => \preamble_det_bit_count_reg[7]\(2)
    );
\preamble_det_bit_count[3]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7F008000"
    )
        port map (
      I0 => \preamble_det_bit_count_reg[7]_0\(1),
      I1 => \preamble_det_bit_count_reg[7]_0\(0),
      I2 => \preamble_det_bit_count_reg[7]_0\(2),
      I3 => \FSM_onehot_preamble_Stm[3]_i_3_n_0\,
      I4 => \preamble_det_bit_count_reg[7]_0\(3),
      O => \preamble_det_bit_count_reg[7]\(3)
    );
\preamble_det_bit_count[4]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"7FFF000080000000"
    )
        port map (
      I0 => \preamble_det_bit_count_reg[7]_0\(2),
      I1 => \preamble_det_bit_count_reg[7]_0\(0),
      I2 => \preamble_det_bit_count_reg[7]_0\(1),
      I3 => \preamble_det_bit_count_reg[7]_0\(3),
      I4 => \FSM_onehot_preamble_Stm[3]_i_3_n_0\,
      I5 => \preamble_det_bit_count_reg[7]_0\(4),
      O => \preamble_det_bit_count_reg[7]\(4)
    );
\preamble_det_bit_count[5]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"82004100"
    )
        port map (
      I0 => \preamble_det_bit_count_reg[5]\,
      I1 => delay_3_indata,
      I2 => pn_bit,
      I3 => Q(3),
      I4 => \preamble_det_bit_count_reg[7]_0\(5),
      O => \preamble_det_bit_count_reg[7]\(5)
    );
\preamble_det_bit_count[6]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"82004100"
    )
        port map (
      I0 => \preamble_det_bit_count_reg[7]_1\,
      I1 => delay_3_indata,
      I2 => pn_bit,
      I3 => Q(3),
      I4 => \preamble_det_bit_count_reg[7]_0\(6),
      O => \preamble_det_bit_count_reg[7]\(6)
    );
\preamble_det_bit_count[7]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"90090000C00C0000"
    )
        port map (
      I0 => \preamble_det_bit_count_reg[7]_1\,
      I1 => \preamble_det_bit_count_reg[7]_0\(7),
      I2 => delay_3_indata,
      I3 => pn_bit,
      I4 => Q(3),
      I5 => \preamble_det_bit_count_reg[7]_0\(6),
      O => \preamble_det_bit_count_reg[7]\(7)
    );
reset_prng_reg_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"60"
    )
        port map (
      I0 => pn_bit,
      I1 => delay_3_indata,
      I2 => Q(3),
      O => \^pn_bit_reg_0\
    );
\shift_reg[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFBEEBEBBE"
    )
        port map (
      I0 => \delay_cntr[4]_i_2_n_0\,
      I1 => \shift_reg_reg_n_0_[0]\,
      I2 => p_0_in_0,
      I3 => \shift_reg_reg_n_0_[6]\,
      I4 => \shift_reg_reg_n_0_[1]\,
      I5 => \shift_reg[7]_i_2_n_0\,
      O => \shift_reg[0]_i_1_n_0\
    );
\shift_reg[1]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF8"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => \shift_reg[7]_i_2_n_0\,
      I3 => \shift_reg_reg_n_0_[0]\,
      O => \shift_reg[1]_i_1_n_0\
    );
\shift_reg[2]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF8"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => \shift_reg[7]_i_2_n_0\,
      I3 => \shift_reg_reg_n_0_[1]\,
      O => \shift_reg[2]_i_1_n_0\
    );
\shift_reg[3]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF8"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => \shift_reg[7]_i_2_n_0\,
      I3 => \shift_reg_reg_n_0_[2]\,
      O => \shift_reg[3]_i_1_n_0\
    );
\shift_reg[4]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF8"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => \shift_reg[7]_i_2_n_0\,
      I3 => \shift_reg_reg_n_0_[3]\,
      O => \shift_reg[4]_i_1_n_0\
    );
\shift_reg[5]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF8"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => \shift_reg[7]_i_2_n_0\,
      I3 => \shift_reg_reg_n_0_[4]\,
      O => \shift_reg[5]_i_1_n_0\
    );
\shift_reg[6]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF8"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => \shift_reg[7]_i_2_n_0\,
      I3 => \shift_reg_reg_n_0_[5]\,
      O => \shift_reg[6]_i_1_n_0\
    );
\shift_reg[7]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF8"
    )
        port map (
      I0 => reset_n,
      I1 => reset_prng,
      I2 => \shift_reg[7]_i_2_n_0\,
      I3 => \shift_reg_reg_n_0_[6]\,
      O => \shift_reg[7]_i_1_n_0\
    );
\shift_reg[7]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8000000000000000"
    )
        port map (
      I0 => delay_cntr(2),
      I1 => delay_cntr(3),
      I2 => delay_cntr(0),
      I3 => delay_cntr(1),
      I4 => delay_cntr(5),
      I5 => delay_cntr(4),
      O => \shift_reg[7]_i_2_n_0\
    );
\shift_reg_reg[0]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      D => \shift_reg[0]_i_1_n_0\,
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[0]\
    );
\shift_reg_reg[1]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      D => \shift_reg[1]_i_1_n_0\,
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[1]\
    );
\shift_reg_reg[2]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      D => \shift_reg[2]_i_1_n_0\,
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[2]\
    );
\shift_reg_reg[3]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      D => \shift_reg[3]_i_1_n_0\,
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[3]\
    );
\shift_reg_reg[4]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      D => \shift_reg[4]_i_1_n_0\,
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[4]\
    );
\shift_reg_reg[5]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      D => \shift_reg[5]_i_1_n_0\,
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[5]\
    );
\shift_reg_reg[6]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      D => \shift_reg[6]_i_1_n_0\,
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[6]\
    );
\shift_reg_reg[7]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => \delay_cntr[5]_i_1_n_0\,
      D => \shift_reg[7]_i_1_n_0\,
      PRE => \^reset_n_0\,
      Q => p_0_in_0
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_preamble_detector_fsk is
  port (
    data_det_out : out STD_LOGIC;
    data_det_val_out : out STD_LOGIC;
    data_det_start : out STD_LOGIC;
    data_det_end : out STD_LOGIC;
    reset_n : in STD_LOGIC;
    clock : in STD_LOGIC;
    bsync_dat_in : in STD_LOGIC_VECTOR ( 0 to 0 );
    bsync_val_in : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_preamble_detector_fsk;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_preamble_detector_fsk is
  signal \FSM_onehot_preamble_Stm[0]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm[1]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm[2]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm[2]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm[2]_i_3_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm[2]_i_4_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm[2]_i_5_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm[3]_i_2_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm[6]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm_reg_n_0_[0]\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm_reg_n_0_[1]\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm_reg_n_0_[2]\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm_reg_n_0_[4]\ : STD_LOGIC;
  signal \FSM_onehot_preamble_Stm_reg_n_0_[5]\ : STD_LOGIC;
  signal data_bit_count : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal \data_bit_count0_carry__0_n_0\ : STD_LOGIC;
  signal \data_bit_count0_carry__0_n_1\ : STD_LOGIC;
  signal \data_bit_count0_carry__0_n_2\ : STD_LOGIC;
  signal \data_bit_count0_carry__0_n_3\ : STD_LOGIC;
  signal \data_bit_count0_carry__1_n_0\ : STD_LOGIC;
  signal \data_bit_count0_carry__1_n_1\ : STD_LOGIC;
  signal \data_bit_count0_carry__1_n_2\ : STD_LOGIC;
  signal \data_bit_count0_carry__1_n_3\ : STD_LOGIC;
  signal \data_bit_count0_carry__2_n_2\ : STD_LOGIC;
  signal \data_bit_count0_carry__2_n_3\ : STD_LOGIC;
  signal data_bit_count0_carry_n_0 : STD_LOGIC;
  signal data_bit_count0_carry_n_1 : STD_LOGIC;
  signal data_bit_count0_carry_n_2 : STD_LOGIC;
  signal data_bit_count0_carry_n_3 : STD_LOGIC;
  signal \data_bit_count[0]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[10]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[11]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[12]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[13]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[14]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[15]_i_2_n_0\ : STD_LOGIC;
  signal \data_bit_count[1]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[2]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[3]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[4]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[5]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[6]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[7]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[8]_i_1_n_0\ : STD_LOGIC;
  signal \data_bit_count[9]_i_1_n_0\ : STD_LOGIC;
  signal data_bit_count_1 : STD_LOGIC;
  signal \^data_det_out\ : STD_LOGIC;
  signal delay_2_indata : STD_LOGIC;
  signal delay_3_indata : STD_LOGIC;
  signal en_preamble : STD_LOGIC;
  signal end_flag : STD_LOGIC;
  signal in7 : STD_LOGIC_VECTOR ( 15 downto 1 );
  signal p_0_in0_in : STD_LOGIC;
  signal p_0_in_0 : STD_LOGIC;
  signal preamble_det_bit_count : STD_LOGIC;
  signal \preamble_det_bit_count[5]_i_2_n_0\ : STD_LOGIC;
  signal \preamble_det_bit_count_reg_n_0_[0]\ : STD_LOGIC;
  signal \preamble_det_bit_count_reg_n_0_[1]\ : STD_LOGIC;
  signal \preamble_det_bit_count_reg_n_0_[2]\ : STD_LOGIC;
  signal \preamble_det_bit_count_reg_n_0_[3]\ : STD_LOGIC;
  signal \preamble_det_bit_count_reg_n_0_[4]\ : STD_LOGIC;
  signal \preamble_det_bit_count_reg_n_0_[5]\ : STD_LOGIC;
  signal \preamble_det_bit_count_reg_n_0_[6]\ : STD_LOGIC;
  signal \preamble_det_bit_count_reg_n_0_[7]\ : STD_LOGIC;
  signal preamble_en : STD_LOGIC;
  signal preamble_en_i_1_n_0 : STD_LOGIC;
  signal preamble_mod_n_0 : STD_LOGIC;
  signal preamble_mod_n_1 : STD_LOGIC;
  signal preamble_mod_n_10 : STD_LOGIC;
  signal preamble_mod_n_11 : STD_LOGIC;
  signal preamble_mod_n_2 : STD_LOGIC;
  signal preamble_mod_n_3 : STD_LOGIC;
  signal preamble_mod_n_4 : STD_LOGIC;
  signal preamble_mod_n_5 : STD_LOGIC;
  signal preamble_mod_n_6 : STD_LOGIC;
  signal preamble_mod_n_7 : STD_LOGIC;
  signal preamble_mod_n_8 : STD_LOGIC;
  signal preamble_mod_n_9 : STD_LOGIC;
  signal reset_prng : STD_LOGIC;
  signal start_flag : STD_LOGIC;
  signal \NLW_data_bit_count0_carry__2_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_data_bit_count0_carry__2_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_onehot_preamble_Stm[1]_i_1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \FSM_onehot_preamble_Stm[2]_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \FSM_onehot_preamble_Stm[6]_i_1\ : label is "soft_lutpair10";
  attribute FSM_ENCODED_STATES : string;
  attribute FSM_ENCODED_STATES of \FSM_onehot_preamble_Stm_reg[0]\ : label is "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_preamble_Stm_reg[1]\ : label is "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_preamble_Stm_reg[2]\ : label is "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_preamble_Stm_reg[3]\ : label is "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_preamble_Stm_reg[4]\ : label is "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_preamble_Stm_reg[5]\ : label is "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_preamble_Stm_reg[6]\ : label is "QUEUE2:0100000,SAMPLE_COMPARE:0010000,QUEUE1:1000000,DATA_START_BYPASS:0001000,DATA_MID_BYPASS:0000100,DATA_END_BYPASS:0000010,VALID_DET:0000001";
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of data_bit_count0_carry : label is 35;
  attribute ADDER_THRESHOLD of \data_bit_count0_carry__0\ : label is 35;
  attribute ADDER_THRESHOLD of \data_bit_count0_carry__1\ : label is 35;
  attribute ADDER_THRESHOLD of \data_bit_count0_carry__2\ : label is 35;
  attribute SOFT_HLUTNM of \data_bit_count[0]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \data_bit_count[10]_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \data_bit_count[11]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \data_bit_count[12]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \data_bit_count[13]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \data_bit_count[14]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \data_bit_count[15]_i_2\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \data_bit_count[1]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \data_bit_count[2]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \data_bit_count[3]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \data_bit_count[4]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \data_bit_count[5]_i_1\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \data_bit_count[6]_i_1\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \data_bit_count[7]_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \data_bit_count[8]_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \data_bit_count[9]_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of data_det_end_i_1 : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of data_det_start_i_1 : label is "soft_lutpair8";
  attribute XILINX_LEGACY_PRIM : string;
  attribute XILINX_LEGACY_PRIM of reset_prng_reg : label is "LD";
  attribute XILINX_TRANSFORM_PINMAP : string;
  attribute XILINX_TRANSFORM_PINMAP of reset_prng_reg : label is "VCC:GE GND:CLR";
  attribute SOFT_HLUTNM of reset_prng_reg_i_2 : label is "soft_lutpair10";
begin
  data_det_out <= \^data_det_out\;
\FSM_onehot_preamble_Stm[0]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"FE"
    )
        port map (
      I0 => \preamble_det_bit_count_reg_n_0_[7]\,
      I1 => \FSM_onehot_preamble_Stm[3]_i_2_n_0\,
      I2 => \preamble_det_bit_count_reg_n_0_[6]\,
      O => \FSM_onehot_preamble_Stm[0]_i_2_n_0\
    );
\FSM_onehot_preamble_Stm[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"10FF1010"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm[2]_i_2_n_0\,
      I1 => \FSM_onehot_preamble_Stm[2]_i_3_n_0\,
      I2 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I3 => bsync_val_in,
      I4 => \FSM_onehot_preamble_Stm_reg_n_0_[1]\,
      O => \FSM_onehot_preamble_Stm[1]_i_1_n_0\
    );
\FSM_onehot_preamble_Stm[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFA8A8A8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => \FSM_onehot_preamble_Stm[2]_i_2_n_0\,
      I2 => \FSM_onehot_preamble_Stm[2]_i_3_n_0\,
      I3 => bsync_val_in,
      I4 => p_0_in0_in,
      O => \FSM_onehot_preamble_Stm[2]_i_1_n_0\
    );
\FSM_onehot_preamble_Stm[2]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EFFFFFFF"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm[2]_i_4_n_0\,
      I1 => data_bit_count(6),
      I2 => data_bit_count(5),
      I3 => data_bit_count(3),
      I4 => data_bit_count(4),
      O => \FSM_onehot_preamble_Stm[2]_i_2_n_0\
    );
\FSM_onehot_preamble_Stm[2]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFF7FFF"
    )
        port map (
      I0 => data_bit_count(9),
      I1 => data_bit_count(10),
      I2 => data_bit_count(7),
      I3 => data_bit_count(8),
      I4 => \FSM_onehot_preamble_Stm[2]_i_5_n_0\,
      O => \FSM_onehot_preamble_Stm[2]_i_3_n_0\
    );
\FSM_onehot_preamble_Stm[2]_i_4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"EFFFFFFF"
    )
        port map (
      I0 => data_bit_count(0),
      I1 => data_bit_count(15),
      I2 => bsync_val_in,
      I3 => data_bit_count(2),
      I4 => data_bit_count(1),
      O => \FSM_onehot_preamble_Stm[2]_i_4_n_0\
    );
\FSM_onehot_preamble_Stm[2]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => data_bit_count(12),
      I1 => data_bit_count(11),
      I2 => data_bit_count(14),
      I3 => data_bit_count(13),
      O => \FSM_onehot_preamble_Stm[2]_i_5_n_0\
    );
\FSM_onehot_preamble_Stm[3]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"7FFFFFFFFFFFFFFF"
    )
        port map (
      I0 => \preamble_det_bit_count_reg_n_0_[4]\,
      I1 => \preamble_det_bit_count_reg_n_0_[2]\,
      I2 => \preamble_det_bit_count_reg_n_0_[0]\,
      I3 => \preamble_det_bit_count_reg_n_0_[1]\,
      I4 => \preamble_det_bit_count_reg_n_0_[3]\,
      I5 => \preamble_det_bit_count_reg_n_0_[5]\,
      O => \FSM_onehot_preamble_Stm[3]_i_2_n_0\
    );
\FSM_onehot_preamble_Stm[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[0]\,
      I1 => bsync_val_in,
      O => \FSM_onehot_preamble_Stm[6]_i_1_n_0\
    );
\FSM_onehot_preamble_Stm_reg[0]\: unisim.vcomponents.FDPE
    generic map(
      INIT => '1'
    )
        port map (
      C => clock,
      CE => '1',
      D => preamble_mod_n_2,
      PRE => preamble_mod_n_0,
      Q => \FSM_onehot_preamble_Stm_reg_n_0_[0]\
    );
\FSM_onehot_preamble_Stm_reg[1]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \FSM_onehot_preamble_Stm[1]_i_1_n_0\,
      Q => \FSM_onehot_preamble_Stm_reg_n_0_[1]\
    );
\FSM_onehot_preamble_Stm_reg[2]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \FSM_onehot_preamble_Stm[2]_i_1_n_0\,
      Q => \FSM_onehot_preamble_Stm_reg_n_0_[2]\
    );
\FSM_onehot_preamble_Stm_reg[3]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_1,
      Q => p_0_in0_in
    );
\FSM_onehot_preamble_Stm_reg[4]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \FSM_onehot_preamble_Stm_reg_n_0_[5]\,
      Q => \FSM_onehot_preamble_Stm_reg_n_0_[4]\
    );
\FSM_onehot_preamble_Stm_reg[5]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => p_0_in_0,
      Q => \FSM_onehot_preamble_Stm_reg_n_0_[5]\
    );
\FSM_onehot_preamble_Stm_reg[6]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \FSM_onehot_preamble_Stm[6]_i_1_n_0\,
      Q => p_0_in_0
    );
data_bit_count0_carry: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => data_bit_count0_carry_n_0,
      CO(2) => data_bit_count0_carry_n_1,
      CO(1) => data_bit_count0_carry_n_2,
      CO(0) => data_bit_count0_carry_n_3,
      CYINIT => data_bit_count(0),
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => in7(4 downto 1),
      S(3 downto 0) => data_bit_count(4 downto 1)
    );
\data_bit_count0_carry__0\: unisim.vcomponents.CARRY4
     port map (
      CI => data_bit_count0_carry_n_0,
      CO(3) => \data_bit_count0_carry__0_n_0\,
      CO(2) => \data_bit_count0_carry__0_n_1\,
      CO(1) => \data_bit_count0_carry__0_n_2\,
      CO(0) => \data_bit_count0_carry__0_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => in7(8 downto 5),
      S(3 downto 0) => data_bit_count(8 downto 5)
    );
\data_bit_count0_carry__1\: unisim.vcomponents.CARRY4
     port map (
      CI => \data_bit_count0_carry__0_n_0\,
      CO(3) => \data_bit_count0_carry__1_n_0\,
      CO(2) => \data_bit_count0_carry__1_n_1\,
      CO(1) => \data_bit_count0_carry__1_n_2\,
      CO(0) => \data_bit_count0_carry__1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => in7(12 downto 9),
      S(3 downto 0) => data_bit_count(12 downto 9)
    );
\data_bit_count0_carry__2\: unisim.vcomponents.CARRY4
     port map (
      CI => \data_bit_count0_carry__1_n_0\,
      CO(3 downto 2) => \NLW_data_bit_count0_carry__2_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \data_bit_count0_carry__2_n_2\,
      CO(0) => \data_bit_count0_carry__2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_data_bit_count0_carry__2_O_UNCONNECTED\(3),
      O(2 downto 0) => in7(15 downto 13),
      S(3) => '0',
      S(2 downto 0) => data_bit_count(15 downto 13)
    );
\data_bit_count[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"F4"
    )
        port map (
      I0 => data_bit_count(0),
      I1 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I2 => p_0_in0_in,
      O => \data_bit_count[0]_i_1_n_0\
    );
\data_bit_count[10]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(10),
      O => \data_bit_count[10]_i_1_n_0\
    );
\data_bit_count[11]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(11),
      O => \data_bit_count[11]_i_1_n_0\
    );
\data_bit_count[12]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(12),
      O => \data_bit_count[12]_i_1_n_0\
    );
\data_bit_count[13]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(13),
      O => \data_bit_count[13]_i_1_n_0\
    );
\data_bit_count[14]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(14),
      O => \data_bit_count[14]_i_1_n_0\
    );
\data_bit_count[15]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FEEE"
    )
        port map (
      I0 => p_0_in0_in,
      I1 => \FSM_onehot_preamble_Stm_reg_n_0_[1]\,
      I2 => bsync_val_in,
      I3 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      O => data_bit_count_1
    );
\data_bit_count[15]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(15),
      O => \data_bit_count[15]_i_2_n_0\
    );
\data_bit_count[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(1),
      O => \data_bit_count[1]_i_1_n_0\
    );
\data_bit_count[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(2),
      O => \data_bit_count[2]_i_1_n_0\
    );
\data_bit_count[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(3),
      O => \data_bit_count[3]_i_1_n_0\
    );
\data_bit_count[4]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(4),
      O => \data_bit_count[4]_i_1_n_0\
    );
\data_bit_count[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(5),
      O => \data_bit_count[5]_i_1_n_0\
    );
\data_bit_count[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(6),
      O => \data_bit_count[6]_i_1_n_0\
    );
\data_bit_count[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(7),
      O => \data_bit_count[7]_i_1_n_0\
    );
\data_bit_count[8]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(8),
      O => \data_bit_count[8]_i_1_n_0\
    );
\data_bit_count[9]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[2]\,
      I1 => in7(9),
      O => \data_bit_count[9]_i_1_n_0\
    );
\data_bit_count_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[0]_i_1_n_0\,
      Q => data_bit_count(0)
    );
\data_bit_count_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[10]_i_1_n_0\,
      Q => data_bit_count(10)
    );
\data_bit_count_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[11]_i_1_n_0\,
      Q => data_bit_count(11)
    );
\data_bit_count_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[12]_i_1_n_0\,
      Q => data_bit_count(12)
    );
\data_bit_count_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[13]_i_1_n_0\,
      Q => data_bit_count(13)
    );
\data_bit_count_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[14]_i_1_n_0\,
      Q => data_bit_count(14)
    );
\data_bit_count_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[15]_i_2_n_0\,
      Q => data_bit_count(15)
    );
\data_bit_count_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[1]_i_1_n_0\,
      Q => data_bit_count(1)
    );
\data_bit_count_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[2]_i_1_n_0\,
      Q => data_bit_count(2)
    );
\data_bit_count_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[3]_i_1_n_0\,
      Q => data_bit_count(3)
    );
\data_bit_count_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[4]_i_1_n_0\,
      Q => data_bit_count(4)
    );
\data_bit_count_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[5]_i_1_n_0\,
      Q => data_bit_count(5)
    );
\data_bit_count_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[6]_i_1_n_0\,
      Q => data_bit_count(6)
    );
\data_bit_count_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[7]_i_1_n_0\,
      Q => data_bit_count(7)
    );
\data_bit_count_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[8]_i_1_n_0\,
      Q => data_bit_count(8)
    );
\data_bit_count_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => data_bit_count_1,
      CLR => preamble_mod_n_0,
      D => \data_bit_count[9]_i_1_n_0\,
      Q => data_bit_count(9)
    );
data_det_end_i_1: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => bsync_val_in,
      I1 => \FSM_onehot_preamble_Stm_reg_n_0_[1]\,
      O => end_flag
    );
data_det_end_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => end_flag,
      Q => data_det_end
    );
data_det_start_i_1: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => bsync_val_in,
      I1 => p_0_in0_in,
      O => start_flag
    );
data_det_start_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => start_flag,
      Q => data_det_start
    );
data_det_val_out_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => bsync_val_in,
      Q => data_det_val_out
    );
delay_1_indata_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => bsync_dat_in(0),
      Q => \^data_det_out\
    );
delay_2_indata_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \^data_det_out\,
      Q => delay_2_indata
    );
delay_3_indata_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => delay_2_indata,
      Q => delay_3_indata
    );
\preamble_det_bit_count[5]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFFFFFF"
    )
        port map (
      I0 => \preamble_det_bit_count_reg_n_0_[3]\,
      I1 => \preamble_det_bit_count_reg_n_0_[1]\,
      I2 => \preamble_det_bit_count_reg_n_0_[0]\,
      I3 => \preamble_det_bit_count_reg_n_0_[2]\,
      I4 => \preamble_det_bit_count_reg_n_0_[4]\,
      O => \preamble_det_bit_count[5]_i_2_n_0\
    );
\preamble_det_bit_count[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => p_0_in0_in,
      I1 => \FSM_onehot_preamble_Stm_reg_n_0_[4]\,
      O => preamble_det_bit_count
    );
\preamble_det_bit_count_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => preamble_det_bit_count,
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_11,
      Q => \preamble_det_bit_count_reg_n_0_[0]\
    );
\preamble_det_bit_count_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => preamble_det_bit_count,
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_10,
      Q => \preamble_det_bit_count_reg_n_0_[1]\
    );
\preamble_det_bit_count_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => preamble_det_bit_count,
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_9,
      Q => \preamble_det_bit_count_reg_n_0_[2]\
    );
\preamble_det_bit_count_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => preamble_det_bit_count,
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_8,
      Q => \preamble_det_bit_count_reg_n_0_[3]\
    );
\preamble_det_bit_count_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => preamble_det_bit_count,
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_7,
      Q => \preamble_det_bit_count_reg_n_0_[4]\
    );
\preamble_det_bit_count_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => preamble_det_bit_count,
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_6,
      Q => \preamble_det_bit_count_reg_n_0_[5]\
    );
\preamble_det_bit_count_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => preamble_det_bit_count,
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_5,
      Q => \preamble_det_bit_count_reg_n_0_[6]\
    );
\preamble_det_bit_count_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => preamble_det_bit_count,
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_4,
      Q => \preamble_det_bit_count_reg_n_0_[7]\
    );
preamble_en_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AAAAAAABAAAAAAA8"
    )
        port map (
      I0 => bsync_val_in,
      I1 => \FSM_onehot_preamble_Stm_reg_n_0_[4]\,
      I2 => \FSM_onehot_preamble_Stm_reg_n_0_[0]\,
      I3 => p_0_in_0,
      I4 => \FSM_onehot_preamble_Stm_reg_n_0_[5]\,
      I5 => preamble_en,
      O => preamble_en_i_1_n_0
    );
preamble_en_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => preamble_en_i_1_n_0,
      Q => preamble_en
    );
preamble_mod: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr
     port map (
      D(1) => preamble_mod_n_1,
      D(0) => preamble_mod_n_2,
      \FSM_onehot_preamble_Stm_reg[0]\ => \FSM_onehot_preamble_Stm[0]_i_2_n_0\,
      Q(3) => \FSM_onehot_preamble_Stm_reg_n_0_[4]\,
      Q(2) => p_0_in0_in,
      Q(1) => \FSM_onehot_preamble_Stm_reg_n_0_[1]\,
      Q(0) => \FSM_onehot_preamble_Stm_reg_n_0_[0]\,
      bsync_val_in => bsync_val_in,
      clock => clock,
      delay_3_indata => delay_3_indata,
      pn_bit_reg_0 => preamble_mod_n_3,
      \preamble_det_bit_count_reg[5]\ => \preamble_det_bit_count[5]_i_2_n_0\,
      \preamble_det_bit_count_reg[7]\(7) => preamble_mod_n_4,
      \preamble_det_bit_count_reg[7]\(6) => preamble_mod_n_5,
      \preamble_det_bit_count_reg[7]\(5) => preamble_mod_n_6,
      \preamble_det_bit_count_reg[7]\(4) => preamble_mod_n_7,
      \preamble_det_bit_count_reg[7]\(3) => preamble_mod_n_8,
      \preamble_det_bit_count_reg[7]\(2) => preamble_mod_n_9,
      \preamble_det_bit_count_reg[7]\(1) => preamble_mod_n_10,
      \preamble_det_bit_count_reg[7]\(0) => preamble_mod_n_11,
      \preamble_det_bit_count_reg[7]_0\(7) => \preamble_det_bit_count_reg_n_0_[7]\,
      \preamble_det_bit_count_reg[7]_0\(6) => \preamble_det_bit_count_reg_n_0_[6]\,
      \preamble_det_bit_count_reg[7]_0\(5) => \preamble_det_bit_count_reg_n_0_[5]\,
      \preamble_det_bit_count_reg[7]_0\(4) => \preamble_det_bit_count_reg_n_0_[4]\,
      \preamble_det_bit_count_reg[7]_0\(3) => \preamble_det_bit_count_reg_n_0_[3]\,
      \preamble_det_bit_count_reg[7]_0\(2) => \preamble_det_bit_count_reg_n_0_[2]\,
      \preamble_det_bit_count_reg[7]_0\(1) => \preamble_det_bit_count_reg_n_0_[1]\,
      \preamble_det_bit_count_reg[7]_0\(0) => \preamble_det_bit_count_reg_n_0_[0]\,
      \preamble_det_bit_count_reg[7]_1\ => \FSM_onehot_preamble_Stm[3]_i_2_n_0\,
      preamble_en => preamble_en,
      reset_n => reset_n,
      reset_n_0 => preamble_mod_n_0,
      reset_prng => reset_prng
    );
reset_prng_reg: unisim.vcomponents.LDCE
    generic map(
      INIT => '0'
    )
        port map (
      CLR => '0',
      D => preamble_mod_n_3,
      G => en_preamble,
      GE => '1',
      Q => reset_prng
    );
reset_prng_reg_i_2: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \FSM_onehot_preamble_Stm_reg_n_0_[4]\,
      I1 => \FSM_onehot_preamble_Stm_reg_n_0_[0]\,
      I2 => p_0_in_0,
      I3 => \FSM_onehot_preamble_Stm_reg_n_0_[5]\,
      O => en_preamble
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  port (
    clock : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    bsync_dat_in : in STD_LOGIC_VECTOR ( 7 downto 0 );
    bsync_val_in : in STD_LOGIC;
    data_det_out : out STD_LOGIC;
    data_det_val_out : out STD_LOGIC;
    data_det_start : out STD_LOGIC;
    data_det_end : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "system_preamble_detector_fsk_0_0,preamble_detector_fsk,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "module_ref";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "preamble_detector_fsk,Vivado 2023.1";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of clock : signal is "xilinx.com:signal:clock:1.0 clock CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of clock : signal is "XIL_INTERFACENAME clock, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of reset_n : signal is "xilinx.com:signal:reset:1.0 reset_n RST";
  attribute X_INTERFACE_PARAMETER of reset_n : signal is "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
begin
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_preamble_detector_fsk
     port map (
      bsync_dat_in(0) => bsync_dat_in(0),
      bsync_val_in => bsync_val_in,
      clock => clock,
      data_det_end => data_det_end,
      data_det_out => data_det_out,
      data_det_start => data_det_start,
      data_det_val_out => data_det_val_out,
      reset_n => reset_n
    );
end STRUCTURE;
