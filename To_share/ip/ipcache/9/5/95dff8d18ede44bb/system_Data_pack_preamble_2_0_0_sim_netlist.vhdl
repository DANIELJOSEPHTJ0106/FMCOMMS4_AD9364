-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Wed Jan  7 18:03:55 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_Data_pack_preamble_2_0_0_sim_netlist.vhdl
-- Design      : system_Data_pack_preamble_2_0_0
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
    pn_bit_reg_0 : out STD_LOGIC;
    E : in STD_LOGIC_VECTOR ( 0 to 0 );
    clock : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    out_bit_to_fsk_reg : in STD_LOGIC;
    out_bit_to_fsk_reg_0 : in STD_LOGIC;
    out_bit_to_fsk_reg_1 : in STD_LOGIC;
    out_bit_to_fsk_reg_2 : in STD_LOGIC;
    out_bit_to_fsk : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_PN_seq_64_8_lfsr is
  signal delay_cntr : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal \delay_cntr[7]_i_2_n_0\ : STD_LOGIC;
  signal delay_cntr_0 : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal p_0_in : STD_LOGIC;
  signal preamble_bit : STD_LOGIC;
  signal \^reset_n_0\ : STD_LOGIC;
  signal shift_reg : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal \shift_reg[7]_i_2_n_0\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[0]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[1]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[2]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[3]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[4]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[5]\ : STD_LOGIC;
  signal \shift_reg_reg_n_0_[6]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \delay_cntr[0]_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \delay_cntr[1]_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \delay_cntr[2]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \delay_cntr[3]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \delay_cntr[4]_i_1\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \delay_cntr[6]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \delay_cntr[7]_i_2\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \shift_reg[0]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \shift_reg[1]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \shift_reg[2]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \shift_reg[3]_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \shift_reg[4]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \shift_reg[5]_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \shift_reg[6]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \shift_reg[7]_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \shift_reg[7]_i_2\ : label is "soft_lutpair2";
begin
  reset_n_0 <= \^reset_n_0\;
\delay_cntr[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => delay_cntr(0),
      O => delay_cntr_0(0)
    );
\delay_cntr[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => delay_cntr(0),
      I1 => delay_cntr(1),
      O => delay_cntr_0(1)
    );
\delay_cntr[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6A"
    )
        port map (
      I0 => delay_cntr(2),
      I1 => delay_cntr(1),
      I2 => delay_cntr(0),
      O => delay_cntr_0(2)
    );
\delay_cntr[3]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"6AAA"
    )
        port map (
      I0 => delay_cntr(3),
      I1 => delay_cntr(0),
      I2 => delay_cntr(1),
      I3 => delay_cntr(2),
      O => delay_cntr_0(3)
    );
\delay_cntr[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"6AAAAAAA"
    )
        port map (
      I0 => delay_cntr(4),
      I1 => delay_cntr(2),
      I2 => delay_cntr(3),
      I3 => delay_cntr(0),
      I4 => delay_cntr(1),
      O => delay_cntr_0(4)
    );
\delay_cntr[5]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"6AAAAAAAAAAAAAAA"
    )
        port map (
      I0 => delay_cntr(5),
      I1 => delay_cntr(1),
      I2 => delay_cntr(0),
      I3 => delay_cntr(3),
      I4 => delay_cntr(2),
      I5 => delay_cntr(4),
      O => delay_cntr_0(5)
    );
\delay_cntr[6]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"3FFF8000"
    )
        port map (
      I0 => delay_cntr(7),
      I1 => delay_cntr(5),
      I2 => \delay_cntr[7]_i_2_n_0\,
      I3 => delay_cntr(4),
      I4 => delay_cntr(6),
      O => delay_cntr_0(6)
    );
\delay_cntr[7]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"6AAAAAAA"
    )
        port map (
      I0 => delay_cntr(7),
      I1 => delay_cntr(6),
      I2 => delay_cntr(5),
      I3 => \delay_cntr[7]_i_2_n_0\,
      I4 => delay_cntr(4),
      O => delay_cntr_0(7)
    );
\delay_cntr[7]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8000"
    )
        port map (
      I0 => delay_cntr(1),
      I1 => delay_cntr(0),
      I2 => delay_cntr(3),
      I3 => delay_cntr(2),
      O => \delay_cntr[7]_i_2_n_0\
    );
\delay_cntr_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => delay_cntr_0(0),
      Q => delay_cntr(0)
    );
\delay_cntr_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => delay_cntr_0(1),
      Q => delay_cntr(1)
    );
\delay_cntr_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => delay_cntr_0(2),
      Q => delay_cntr(2)
    );
\delay_cntr_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => delay_cntr_0(3),
      Q => delay_cntr(3)
    );
\delay_cntr_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => delay_cntr_0(4),
      Q => delay_cntr(4)
    );
\delay_cntr_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => delay_cntr_0(5),
      Q => delay_cntr(5)
    );
\delay_cntr_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => delay_cntr_0(6),
      Q => delay_cntr(6)
    );
\delay_cntr_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => delay_cntr_0(7),
      Q => delay_cntr(7)
    );
out_bit_to_fsk_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0FFCFACF000C0AC0"
    )
        port map (
      I0 => out_bit_to_fsk_reg,
      I1 => preamble_bit,
      I2 => out_bit_to_fsk_reg_0,
      I3 => out_bit_to_fsk_reg_1,
      I4 => out_bit_to_fsk_reg_2,
      I5 => out_bit_to_fsk,
      O => pn_bit_reg_0
    );
out_bit_to_fsk_i_2: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => reset_n,
      O => \^reset_n_0\
    );
pn_bit_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => E(0),
      CLR => \^reset_n_0\,
      D => p_0_in,
      Q => preamble_bit
    );
\shift_reg[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BEEBEBBE"
    )
        port map (
      I0 => \shift_reg[7]_i_2_n_0\,
      I1 => p_0_in,
      I2 => \shift_reg_reg_n_0_[0]\,
      I3 => \shift_reg_reg_n_0_[1]\,
      I4 => \shift_reg_reg_n_0_[6]\,
      O => shift_reg(0)
    );
\shift_reg[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \shift_reg_reg_n_0_[0]\,
      I1 => \shift_reg[7]_i_2_n_0\,
      O => shift_reg(1)
    );
\shift_reg[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \shift_reg_reg_n_0_[1]\,
      I1 => \shift_reg[7]_i_2_n_0\,
      O => shift_reg(2)
    );
\shift_reg[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \shift_reg_reg_n_0_[2]\,
      I1 => \shift_reg[7]_i_2_n_0\,
      O => shift_reg(3)
    );
\shift_reg[4]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \shift_reg_reg_n_0_[3]\,
      I1 => \shift_reg[7]_i_2_n_0\,
      O => shift_reg(4)
    );
\shift_reg[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \shift_reg_reg_n_0_[4]\,
      I1 => \shift_reg[7]_i_2_n_0\,
      O => shift_reg(5)
    );
\shift_reg[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \shift_reg_reg_n_0_[5]\,
      I1 => \shift_reg[7]_i_2_n_0\,
      O => shift_reg(6)
    );
\shift_reg[7]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \shift_reg_reg_n_0_[6]\,
      I1 => \shift_reg[7]_i_2_n_0\,
      O => shift_reg(7)
    );
\shift_reg[7]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000080"
    )
        port map (
      I0 => \delay_cntr[7]_i_2_n_0\,
      I1 => delay_cntr(4),
      I2 => delay_cntr(5),
      I3 => delay_cntr(7),
      I4 => delay_cntr(6),
      O => \shift_reg[7]_i_2_n_0\
    );
\shift_reg_reg[0]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => E(0),
      D => shift_reg(0),
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[0]\
    );
\shift_reg_reg[1]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => E(0),
      D => shift_reg(1),
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[1]\
    );
\shift_reg_reg[2]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => E(0),
      D => shift_reg(2),
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[2]\
    );
\shift_reg_reg[3]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => E(0),
      D => shift_reg(3),
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[3]\
    );
\shift_reg_reg[4]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => E(0),
      D => shift_reg(4),
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[4]\
    );
\shift_reg_reg[5]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => E(0),
      D => shift_reg(5),
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[5]\
    );
\shift_reg_reg[6]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => E(0),
      D => shift_reg(6),
      PRE => \^reset_n_0\,
      Q => \shift_reg_reg_n_0_[6]\
    );
\shift_reg_reg[7]\: unisim.vcomponents.FDPE
     port map (
      C => clock,
      CE => E(0),
      D => shift_reg(7),
      PRE => \^reset_n_0\,
      Q => p_0_in
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_Data_pack_preamble_2FSK_upd is
  port (
    out_bit_valid : out STD_LOGIC;
    out_bit_prmbl_strt : out STD_LOGIC;
    out_bit_prmbl_end : out STD_LOGIC;
    out_bit_to_fsk : out STD_LOGIC;
    clock : in STD_LOGIC;
    reset_n : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_Data_pack_preamble_2FSK_upd;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_Data_pack_preamble_2FSK_upd is
  signal \bit_cnt[5]_i_2_n_0\ : STD_LOGIC;
  signal \bit_cnt[5]_i_3_n_0\ : STD_LOGIC;
  signal \bit_cnt[5]_i_4_n_0\ : STD_LOGIC;
  signal \bit_cnt[6]_i_2_n_0\ : STD_LOGIC;
  signal \bit_cnt[6]_i_3_n_0\ : STD_LOGIC;
  signal \bit_cnt[7]_i_1_n_0\ : STD_LOGIC;
  signal \bit_cnt[7]_i_3_n_0\ : STD_LOGIC;
  signal \bit_cnt[7]_i_4_n_0\ : STD_LOGIC;
  signal \bit_cnt[7]_i_5_n_0\ : STD_LOGIC;
  signal \bit_cnt_reg_n_0_[0]\ : STD_LOGIC;
  signal \bit_cnt_reg_n_0_[1]\ : STD_LOGIC;
  signal \bit_cnt_reg_n_0_[2]\ : STD_LOGIC;
  signal \bit_cnt_reg_n_0_[3]\ : STD_LOGIC;
  signal \bit_cnt_reg_n_0_[4]\ : STD_LOGIC;
  signal \bit_cnt_reg_n_0_[5]\ : STD_LOGIC;
  signal \bit_cnt_reg_n_0_[6]\ : STD_LOGIC;
  signal \bit_cnt_reg_n_0_[7]\ : STD_LOGIC;
  signal bit_dur_cnt : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal bit_dur_cnt0 : STD_LOGIC_VECTOR ( 15 downto 1 );
  signal \bit_dur_cnt0_carry__0_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__0_n_1\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__0_n_2\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__0_n_3\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__1_n_1\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__1_n_2\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__1_n_3\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__2_n_2\ : STD_LOGIC;
  signal \bit_dur_cnt0_carry__2_n_3\ : STD_LOGIC;
  signal bit_dur_cnt0_carry_n_0 : STD_LOGIC;
  signal bit_dur_cnt0_carry_n_1 : STD_LOGIC;
  signal bit_dur_cnt0_carry_n_2 : STD_LOGIC;
  signal bit_dur_cnt0_carry_n_3 : STD_LOGIC;
  signal \bit_dur_cnt[0]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[10]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[11]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[12]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[13]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[14]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[15]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[15]_i_2_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[15]_i_3_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[1]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[2]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[3]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[4]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[5]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[6]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[7]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[8]_i_1_n_0\ : STD_LOGIC;
  signal \bit_dur_cnt[9]_i_1_n_0\ : STD_LOGIC;
  signal out_bit_prmbl_end_i_1_n_0 : STD_LOGIC;
  signal out_bit_prmbl_strt_i_1_n_0 : STD_LOGIC;
  signal out_bit_prmbl_strt_i_2_n_0 : STD_LOGIC;
  signal \^out_bit_to_fsk\ : STD_LOGIC;
  signal out_bit_to_fsk_i_4_n_0 : STD_LOGIC;
  signal out_bit_to_fsk_i_5_n_0 : STD_LOGIC;
  signal out_bit_to_fsk_reg_i_3_n_0 : STD_LOGIC;
  signal out_bit_valid_i_1_n_0 : STD_LOGIC;
  signal p_1_in : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal preamble_en : STD_LOGIC;
  signal preamble_en_i_1_n_0 : STD_LOGIC;
  signal preamble_en_i_2_n_0 : STD_LOGIC;
  signal preamble_en_i_3_n_0 : STD_LOGIC;
  signal preamble_mod_n_0 : STD_LOGIC;
  signal preamble_mod_n_1 : STD_LOGIC;
  signal \state_udp[0]_i_1_n_0\ : STD_LOGIC;
  signal \state_udp[0]_i_2_n_0\ : STD_LOGIC;
  signal \state_udp[0]_i_3_n_0\ : STD_LOGIC;
  signal \state_udp[0]_i_4_n_0\ : STD_LOGIC;
  signal \state_udp[0]_i_5_n_0\ : STD_LOGIC;
  signal \state_udp[0]_i_6_n_0\ : STD_LOGIC;
  signal \state_udp[1]_i_1_n_0\ : STD_LOGIC;
  signal \state_udp[1]_i_2_n_0\ : STD_LOGIC;
  signal \state_udp[1]_i_3_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_1_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_2_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_3_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_4_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_5_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_6_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_7_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_8_n_0\ : STD_LOGIC;
  signal \state_udp[2]_i_9_n_0\ : STD_LOGIC;
  signal \state_udp_nxt[0]_i_1_n_0\ : STD_LOGIC;
  signal \state_udp_nxt[0]_i_2_n_0\ : STD_LOGIC;
  signal \state_udp_nxt[1]_i_1_n_0\ : STD_LOGIC;
  signal \state_udp_nxt[1]_i_2_n_0\ : STD_LOGIC;
  signal \state_udp_nxt[1]_i_3_n_0\ : STD_LOGIC;
  signal \state_udp_nxt[1]_i_4_n_0\ : STD_LOGIC;
  signal \state_udp_nxt[2]_i_1_n_0\ : STD_LOGIC;
  signal \state_udp_nxt_reg_n_0_[0]\ : STD_LOGIC;
  signal \state_udp_nxt_reg_n_0_[1]\ : STD_LOGIC;
  signal \state_udp_nxt_reg_n_0_[2]\ : STD_LOGIC;
  signal \state_udp_reg_n_0_[0]\ : STD_LOGIC;
  signal \state_udp_reg_n_0_[1]\ : STD_LOGIC;
  signal \state_udp_reg_n_0_[2]\ : STD_LOGIC;
  signal temp_Data : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal \temp_Data[0]_i_1_n_0\ : STD_LOGIC;
  signal \temp_Data[1]_i_1_n_0\ : STD_LOGIC;
  signal \temp_Data[2]_i_1_n_0\ : STD_LOGIC;
  signal \temp_Data[3]_i_1_n_0\ : STD_LOGIC;
  signal \temp_Data[4]_i_1_n_0\ : STD_LOGIC;
  signal \temp_Data[4]_i_2_n_0\ : STD_LOGIC;
  signal \temp_Data[5]_i_1_n_0\ : STD_LOGIC;
  signal \temp_Data[5]_i_2_n_0\ : STD_LOGIC;
  signal \temp_Data[6]_i_1_n_0\ : STD_LOGIC;
  signal \temp_Data[6]_i_2_n_0\ : STD_LOGIC;
  signal \temp_Data[7]_i_1_n_0\ : STD_LOGIC;
  signal \temp_Data[7]_i_2_n_0\ : STD_LOGIC;
  signal \temp_Data[7]_i_3_n_0\ : STD_LOGIC;
  signal \temp_Data[7]_i_4_n_0\ : STD_LOGIC;
  signal word_count : STD_LOGIC_VECTOR ( 7 downto 0 );
  signal \word_count[0]_i_1_n_0\ : STD_LOGIC;
  signal \word_count[1]_i_1_n_0\ : STD_LOGIC;
  signal \word_count[2]_i_1_n_0\ : STD_LOGIC;
  signal \word_count[3]_i_1_n_0\ : STD_LOGIC;
  signal \word_count[4]_i_1_n_0\ : STD_LOGIC;
  signal \word_count[5]_i_1_n_0\ : STD_LOGIC;
  signal \word_count[5]_i_2_n_0\ : STD_LOGIC;
  signal \word_count[6]_i_1_n_0\ : STD_LOGIC;
  signal \word_count[6]_i_2_n_0\ : STD_LOGIC;
  signal \word_count[6]_i_3_n_0\ : STD_LOGIC;
  signal \word_count[7]_i_1_n_0\ : STD_LOGIC;
  signal \word_count[7]_i_2_n_0\ : STD_LOGIC;
  signal \word_count[7]_i_3_n_0\ : STD_LOGIC;
  signal \word_count[7]_i_4_n_0\ : STD_LOGIC;
  signal \word_count_reg_rep_n_0_[0]\ : STD_LOGIC;
  signal \word_count_reg_rep_n_0_[1]\ : STD_LOGIC;
  signal \word_count_reg_rep_n_0_[2]\ : STD_LOGIC;
  signal \word_count_reg_rep_n_0_[3]\ : STD_LOGIC;
  signal \word_count_reg_rep_n_0_[4]\ : STD_LOGIC;
  signal \word_count_reg_rep_n_0_[5]\ : STD_LOGIC;
  signal \word_count_reg_rep_n_0_[6]\ : STD_LOGIC;
  signal \word_count_reg_rep_n_0_[7]\ : STD_LOGIC;
  signal \NLW_bit_dur_cnt0_carry__2_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_bit_dur_cnt0_carry__2_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \bit_cnt[0]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \bit_cnt[1]_i_1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \bit_cnt[5]_i_2\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \bit_cnt[5]_i_3\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \bit_cnt[5]_i_4\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \bit_cnt[7]_i_5\ : label is "soft_lutpair21";
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of bit_dur_cnt0_carry : label is 35;
  attribute ADDER_THRESHOLD of \bit_dur_cnt0_carry__0\ : label is 35;
  attribute ADDER_THRESHOLD of \bit_dur_cnt0_carry__1\ : label is 35;
  attribute ADDER_THRESHOLD of \bit_dur_cnt0_carry__2\ : label is 35;
  attribute SOFT_HLUTNM of \bit_dur_cnt[0]_i_1\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \bit_dur_cnt[15]_i_3\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \bit_dur_cnt[1]_i_1\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of out_bit_prmbl_end_i_1 : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of out_bit_prmbl_strt_i_1 : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of out_bit_prmbl_strt_i_2 : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of out_bit_valid_i_1 : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \state_udp[2]_i_2\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \state_udp[2]_i_3\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \state_udp_nxt[0]_i_2\ : label is "soft_lutpair22";
  attribute SOFT_HLUTNM of \state_udp_nxt[1]_i_2\ : label is "soft_lutpair22";
  attribute SOFT_HLUTNM of \state_udp_nxt[1]_i_3\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \state_udp_nxt[1]_i_4\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \state_udp_nxt[2]_i_1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \temp_Data[0]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \temp_Data[1]_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \temp_Data[2]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \temp_Data[4]_i_1\ : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \temp_Data[4]_i_2\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \temp_Data[5]_i_1\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \temp_Data[5]_i_2\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \temp_Data[6]_i_1\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \temp_Data[6]_i_2\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \temp_Data[7]_i_4\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \word_count[0]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \word_count[1]_i_1\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \word_count[6]_i_2\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \word_count[6]_i_3\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \word_count[7]_i_2\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \word_count[7]_i_3\ : label is "soft_lutpair16";
  attribute equivalent_register_removal : string;
  attribute equivalent_register_removal of \word_count_reg_rep[0]\ : label is "no";
  attribute equivalent_register_removal of \word_count_reg_rep[1]\ : label is "no";
  attribute equivalent_register_removal of \word_count_reg_rep[2]\ : label is "no";
  attribute equivalent_register_removal of \word_count_reg_rep[3]\ : label is "no";
  attribute equivalent_register_removal of \word_count_reg_rep[4]\ : label is "no";
  attribute equivalent_register_removal of \word_count_reg_rep[5]\ : label is "no";
  attribute equivalent_register_removal of \word_count_reg_rep[6]\ : label is "no";
  attribute equivalent_register_removal of \word_count_reg_rep[7]\ : label is "no";
begin
  out_bit_to_fsk <= \^out_bit_to_fsk\;
\bit_cnt[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"3354"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[0]\,
      I1 => \state_udp_reg_n_0_[2]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \state_udp_reg_n_0_[1]\,
      O => p_1_in(0)
    );
\bit_cnt[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00F9FF60"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[1]\,
      I1 => \bit_cnt_reg_n_0_[0]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \state_udp_reg_n_0_[1]\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => p_1_in(1)
    );
\bit_cnt[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"3C3E3E2E2E0C0C3C"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[2]\,
      I3 => \bit_cnt_reg_n_0_[0]\,
      I4 => \bit_cnt_reg_n_0_[1]\,
      I5 => \bit_cnt_reg_n_0_[2]\,
      O => p_1_in(2)
    );
\bit_cnt[3]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"BCCCCCCE88888882"
    )
        port map (
      I0 => \bit_cnt[6]_i_3_n_0\,
      I1 => \bit_cnt_reg_n_0_[3]\,
      I2 => \bit_cnt_reg_n_0_[2]\,
      I3 => \bit_cnt_reg_n_0_[1]\,
      I4 => \bit_cnt_reg_n_0_[0]\,
      I5 => \bit_cnt[5]_i_4_n_0\,
      O => p_1_in(3)
    );
\bit_cnt[4]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"884488448FF48844"
    )
        port map (
      I0 => \bit_cnt[5]_i_2_n_0\,
      I1 => \bit_cnt[6]_i_3_n_0\,
      I2 => \bit_cnt[5]_i_3_n_0\,
      I3 => \bit_cnt_reg_n_0_[4]\,
      I4 => \state_udp_reg_n_0_[0]\,
      I5 => \state_udp_reg_n_0_[2]\,
      O => p_1_in(4)
    );
\bit_cnt[5]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"CFF4FF04C804C804"
    )
        port map (
      I0 => \bit_cnt[5]_i_2_n_0\,
      I1 => \bit_cnt[6]_i_3_n_0\,
      I2 => \bit_cnt_reg_n_0_[4]\,
      I3 => \bit_cnt_reg_n_0_[5]\,
      I4 => \bit_cnt[5]_i_3_n_0\,
      I5 => \bit_cnt[5]_i_4_n_0\,
      O => p_1_in(5)
    );
\bit_cnt[5]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[3]\,
      I1 => \bit_cnt_reg_n_0_[0]\,
      I2 => \bit_cnt_reg_n_0_[1]\,
      I3 => \bit_cnt_reg_n_0_[2]\,
      O => \bit_cnt[5]_i_2_n_0\
    );
\bit_cnt[5]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8000"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[0]\,
      I1 => \bit_cnt_reg_n_0_[1]\,
      I2 => \bit_cnt_reg_n_0_[2]\,
      I3 => \bit_cnt_reg_n_0_[3]\,
      O => \bit_cnt[5]_i_3_n_0\
    );
\bit_cnt[5]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp_reg_n_0_[2]\,
      O => \bit_cnt[5]_i_4_n_0\
    );
\bit_cnt[6]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"448844884FF84488"
    )
        port map (
      I0 => \bit_cnt[6]_i_2_n_0\,
      I1 => \bit_cnt[6]_i_3_n_0\,
      I2 => \bit_cnt[7]_i_3_n_0\,
      I3 => \bit_cnt_reg_n_0_[6]\,
      I4 => \state_udp_reg_n_0_[0]\,
      I5 => \state_udp_reg_n_0_[2]\,
      O => p_1_in(6)
    );
\bit_cnt[6]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000001"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[5]\,
      I1 => \bit_cnt_reg_n_0_[3]\,
      I2 => \bit_cnt_reg_n_0_[0]\,
      I3 => \bit_cnt_reg_n_0_[1]\,
      I4 => \bit_cnt_reg_n_0_[2]\,
      I5 => \bit_cnt_reg_n_0_[4]\,
      O => \bit_cnt[6]_i_2_n_0\
    );
\bit_cnt[6]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AAAAAAAAAAAAAAA8"
    )
        port map (
      I0 => \bit_cnt[7]_i_5_n_0\,
      I1 => \bit_cnt_reg_n_0_[6]\,
      I2 => \bit_cnt_reg_n_0_[7]\,
      I3 => \bit_cnt_reg_n_0_[5]\,
      I4 => \bit_cnt_reg_n_0_[4]\,
      I5 => \bit_cnt[5]_i_2_n_0\,
      O => \bit_cnt[6]_i_3_n_0\
    );
\bit_cnt[7]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FF0000EF00FFFFFF"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[7]\,
      I1 => \bit_cnt_reg_n_0_[6]\,
      I2 => \bit_cnt[7]_i_3_n_0\,
      I3 => \state_udp_reg_n_0_[1]\,
      I4 => \state_udp_reg_n_0_[2]\,
      I5 => \state_udp_reg_n_0_[0]\,
      O => \bit_cnt[7]_i_1_n_0\
    );
\bit_cnt[7]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AAAAAAAABFEAAAAA"
    )
        port map (
      I0 => \bit_cnt[7]_i_4_n_0\,
      I1 => \bit_cnt_reg_n_0_[6]\,
      I2 => \bit_cnt[7]_i_3_n_0\,
      I3 => \bit_cnt_reg_n_0_[7]\,
      I4 => \state_udp_reg_n_0_[0]\,
      I5 => \state_udp_reg_n_0_[2]\,
      O => p_1_in(7)
    );
\bit_cnt[7]_i_3\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8000000000000000"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[3]\,
      I1 => \bit_cnt_reg_n_0_[2]\,
      I2 => \bit_cnt_reg_n_0_[1]\,
      I3 => \bit_cnt_reg_n_0_[0]\,
      I4 => \bit_cnt_reg_n_0_[5]\,
      I5 => \bit_cnt_reg_n_0_[4]\,
      O => \bit_cnt[7]_i_3_n_0\
    );
\bit_cnt[7]_i_4\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFE000000000000"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[5]\,
      I1 => \bit_cnt[5]_i_2_n_0\,
      I2 => \bit_cnt_reg_n_0_[4]\,
      I3 => \bit_cnt_reg_n_0_[6]\,
      I4 => \bit_cnt[7]_i_5_n_0\,
      I5 => \bit_cnt_reg_n_0_[7]\,
      O => \bit_cnt[7]_i_4_n_0\
    );
\bit_cnt[7]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"2"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \state_udp_reg_n_0_[1]\,
      O => \bit_cnt[7]_i_5_n_0\
    );
\bit_cnt_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_cnt[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => p_1_in(0),
      Q => \bit_cnt_reg_n_0_[0]\
    );
\bit_cnt_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_cnt[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => p_1_in(1),
      Q => \bit_cnt_reg_n_0_[1]\
    );
\bit_cnt_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_cnt[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => p_1_in(2),
      Q => \bit_cnt_reg_n_0_[2]\
    );
\bit_cnt_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_cnt[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => p_1_in(3),
      Q => \bit_cnt_reg_n_0_[3]\
    );
\bit_cnt_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_cnt[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => p_1_in(4),
      Q => \bit_cnt_reg_n_0_[4]\
    );
\bit_cnt_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_cnt[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => p_1_in(5),
      Q => \bit_cnt_reg_n_0_[5]\
    );
\bit_cnt_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_cnt[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => p_1_in(6),
      Q => \bit_cnt_reg_n_0_[6]\
    );
\bit_cnt_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_cnt[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => p_1_in(7),
      Q => \bit_cnt_reg_n_0_[7]\
    );
bit_dur_cnt0_carry: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => bit_dur_cnt0_carry_n_0,
      CO(2) => bit_dur_cnt0_carry_n_1,
      CO(1) => bit_dur_cnt0_carry_n_2,
      CO(0) => bit_dur_cnt0_carry_n_3,
      CYINIT => bit_dur_cnt(0),
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => bit_dur_cnt0(4 downto 1),
      S(3 downto 0) => bit_dur_cnt(4 downto 1)
    );
\bit_dur_cnt0_carry__0\: unisim.vcomponents.CARRY4
     port map (
      CI => bit_dur_cnt0_carry_n_0,
      CO(3) => \bit_dur_cnt0_carry__0_n_0\,
      CO(2) => \bit_dur_cnt0_carry__0_n_1\,
      CO(1) => \bit_dur_cnt0_carry__0_n_2\,
      CO(0) => \bit_dur_cnt0_carry__0_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => bit_dur_cnt0(8 downto 5),
      S(3 downto 0) => bit_dur_cnt(8 downto 5)
    );
\bit_dur_cnt0_carry__1\: unisim.vcomponents.CARRY4
     port map (
      CI => \bit_dur_cnt0_carry__0_n_0\,
      CO(3) => \bit_dur_cnt0_carry__1_n_0\,
      CO(2) => \bit_dur_cnt0_carry__1_n_1\,
      CO(1) => \bit_dur_cnt0_carry__1_n_2\,
      CO(0) => \bit_dur_cnt0_carry__1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => bit_dur_cnt0(12 downto 9),
      S(3 downto 0) => bit_dur_cnt(12 downto 9)
    );
\bit_dur_cnt0_carry__2\: unisim.vcomponents.CARRY4
     port map (
      CI => \bit_dur_cnt0_carry__1_n_0\,
      CO(3 downto 2) => \NLW_bit_dur_cnt0_carry__2_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \bit_dur_cnt0_carry__2_n_2\,
      CO(0) => \bit_dur_cnt0_carry__2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_bit_dur_cnt0_carry__2_O_UNCONNECTED\(3),
      O(2 downto 0) => bit_dur_cnt0(15 downto 13),
      S(3) => '0',
      S(2 downto 0) => bit_dur_cnt(15 downto 13)
    );
\bit_dur_cnt[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"15111010"
    )
        port map (
      I0 => bit_dur_cnt(0),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[0]_i_1_n_0\
    );
\bit_dur_cnt[10]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(10),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[10]_i_1_n_0\
    );
\bit_dur_cnt[11]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(11),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[11]_i_1_n_0\
    );
\bit_dur_cnt[12]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(12),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[12]_i_1_n_0\
    );
\bit_dur_cnt[13]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(13),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[13]_i_1_n_0\
    );
\bit_dur_cnt[14]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(14),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[14]_i_1_n_0\
    );
\bit_dur_cnt[15]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \state_udp_reg_n_0_[1]\,
      O => \bit_dur_cnt[15]_i_1_n_0\
    );
\bit_dur_cnt[15]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(15),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[15]_i_2_n_0\
    );
\bit_dur_cnt[15]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \state_udp[2]_i_7_n_0\,
      I1 => \state_udp[2]_i_6_n_0\,
      I2 => \state_udp[2]_i_5_n_0\,
      I3 => \state_udp[2]_i_4_n_0\,
      O => \bit_dur_cnt[15]_i_3_n_0\
    );
\bit_dur_cnt[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(1),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[1]_i_1_n_0\
    );
\bit_dur_cnt[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(2),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[2]_i_1_n_0\
    );
\bit_dur_cnt[3]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(3),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[3]_i_1_n_0\
    );
\bit_dur_cnt[4]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(4),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[4]_i_1_n_0\
    );
\bit_dur_cnt[5]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(5),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[5]_i_1_n_0\
    );
\bit_dur_cnt[6]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(6),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[6]_i_1_n_0\
    );
\bit_dur_cnt[7]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(7),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[7]_i_1_n_0\
    );
\bit_dur_cnt[8]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(8),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[8]_i_1_n_0\
    );
\bit_dur_cnt[9]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"2A222020"
    )
        port map (
      I0 => bit_dur_cnt0(9),
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \bit_dur_cnt[15]_i_3_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \bit_dur_cnt[9]_i_1_n_0\
    );
\bit_dur_cnt_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[0]_i_1_n_0\,
      Q => bit_dur_cnt(0)
    );
\bit_dur_cnt_reg[10]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[10]_i_1_n_0\,
      Q => bit_dur_cnt(10)
    );
\bit_dur_cnt_reg[11]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[11]_i_1_n_0\,
      Q => bit_dur_cnt(11)
    );
\bit_dur_cnt_reg[12]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[12]_i_1_n_0\,
      Q => bit_dur_cnt(12)
    );
\bit_dur_cnt_reg[13]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[13]_i_1_n_0\,
      Q => bit_dur_cnt(13)
    );
\bit_dur_cnt_reg[14]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[14]_i_1_n_0\,
      Q => bit_dur_cnt(14)
    );
\bit_dur_cnt_reg[15]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[15]_i_2_n_0\,
      Q => bit_dur_cnt(15)
    );
\bit_dur_cnt_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[1]_i_1_n_0\,
      Q => bit_dur_cnt(1)
    );
\bit_dur_cnt_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[2]_i_1_n_0\,
      Q => bit_dur_cnt(2)
    );
\bit_dur_cnt_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[3]_i_1_n_0\,
      Q => bit_dur_cnt(3)
    );
\bit_dur_cnt_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[4]_i_1_n_0\,
      Q => bit_dur_cnt(4)
    );
\bit_dur_cnt_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[5]_i_1_n_0\,
      Q => bit_dur_cnt(5)
    );
\bit_dur_cnt_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[6]_i_1_n_0\,
      Q => bit_dur_cnt(6)
    );
\bit_dur_cnt_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[7]_i_1_n_0\,
      Q => bit_dur_cnt(7)
    );
\bit_dur_cnt_reg[8]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[8]_i_1_n_0\,
      Q => bit_dur_cnt(8)
    );
\bit_dur_cnt_reg[9]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \bit_dur_cnt[15]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \bit_dur_cnt[9]_i_1_n_0\,
      Q => bit_dur_cnt(9)
    );
out_bit_prmbl_end_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"04"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[2]\,
      O => out_bit_prmbl_end_i_1_n_0
    );
out_bit_prmbl_end_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => out_bit_prmbl_end_i_1_n_0,
      Q => out_bit_prmbl_end
    );
out_bit_prmbl_strt_i_1: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0020"
    )
        port map (
      I0 => out_bit_prmbl_strt_i_2_n_0,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \state_udp_reg_n_0_[2]\,
      O => out_bit_prmbl_strt_i_1_n_0
    );
out_bit_prmbl_strt_i_2: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00000001"
    )
        port map (
      I0 => \bit_cnt[5]_i_2_n_0\,
      I1 => \bit_cnt_reg_n_0_[4]\,
      I2 => \bit_cnt_reg_n_0_[5]\,
      I3 => \bit_cnt_reg_n_0_[7]\,
      I4 => \bit_cnt_reg_n_0_[6]\,
      O => out_bit_prmbl_strt_i_2_n_0
    );
out_bit_prmbl_strt_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => out_bit_prmbl_strt_i_1_n_0,
      Q => out_bit_prmbl_strt
    );
out_bit_to_fsk_i_4: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AFA0CFCFAFA0C0C0"
    )
        port map (
      I0 => temp_Data(3),
      I1 => temp_Data(2),
      I2 => \bit_cnt_reg_n_0_[1]\,
      I3 => temp_Data(1),
      I4 => \bit_cnt_reg_n_0_[0]\,
      I5 => temp_Data(0),
      O => out_bit_to_fsk_i_4_n_0
    );
out_bit_to_fsk_i_5: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AFA0CFCFAFA0C0C0"
    )
        port map (
      I0 => temp_Data(7),
      I1 => temp_Data(6),
      I2 => \bit_cnt_reg_n_0_[1]\,
      I3 => temp_Data(5),
      I4 => \bit_cnt_reg_n_0_[0]\,
      I5 => temp_Data(4),
      O => out_bit_to_fsk_i_5_n_0
    );
out_bit_to_fsk_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => preamble_mod_n_1,
      Q => \^out_bit_to_fsk\
    );
out_bit_to_fsk_reg_i_3: unisim.vcomponents.MUXF7
     port map (
      I0 => out_bit_to_fsk_i_4_n_0,
      I1 => out_bit_to_fsk_i_5_n_0,
      O => out_bit_to_fsk_reg_i_3_n_0,
      S => \bit_cnt_reg_n_0_[2]\
    );
out_bit_valid_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"16"
    )
        port map (
      I0 => \state_udp_reg_n_0_[1]\,
      I1 => \state_udp_reg_n_0_[0]\,
      I2 => \state_udp_reg_n_0_[2]\,
      O => out_bit_valid_i_1_n_0
    );
out_bit_valid_reg: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => out_bit_valid_i_1_n_0,
      Q => out_bit_valid
    );
preamble_en_i_1: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FF44FF33EF000303"
    )
        port map (
      I0 => preamble_en_i_2_n_0,
      I1 => \state_udp_reg_n_0_[0]\,
      I2 => \word_count[5]_i_2_n_0\,
      I3 => \state_udp_reg_n_0_[2]\,
      I4 => \state_udp_reg_n_0_[1]\,
      I5 => preamble_en,
      O => preamble_en_i_1_n_0
    );
preamble_en_i_2: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000000000002"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => preamble_en_i_3_n_0,
      I2 => \state_udp[0]_i_3_n_0\,
      I3 => \state_udp[0]_i_4_n_0\,
      I4 => \state_udp[0]_i_5_n_0\,
      I5 => \state_udp[0]_i_6_n_0\,
      O => preamble_en_i_2_n_0
    );
preamble_en_i_3: unisim.vcomponents.LUT3
    generic map(
      INIT => X"FB"
    )
        port map (
      I0 => \state_udp_nxt_reg_n_0_[2]\,
      I1 => \state_udp_nxt_reg_n_0_[0]\,
      I2 => \state_udp_nxt_reg_n_0_[1]\,
      O => preamble_en_i_3_n_0
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
      E(0) => preamble_en,
      clock => clock,
      out_bit_to_fsk => \^out_bit_to_fsk\,
      out_bit_to_fsk_reg => out_bit_to_fsk_reg_i_3_n_0,
      out_bit_to_fsk_reg_0 => \state_udp_reg_n_0_[0]\,
      out_bit_to_fsk_reg_1 => \state_udp_reg_n_0_[2]\,
      out_bit_to_fsk_reg_2 => \state_udp_reg_n_0_[1]\,
      pn_bit_reg_0 => preamble_mod_n_1,
      reset_n => reset_n,
      reset_n_0 => preamble_mod_n_0
    );
\state_udp[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"4400FFFF74330000"
    )
        port map (
      I0 => \state_udp[0]_i_2_n_0\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => out_bit_prmbl_strt_i_2_n_0,
      I3 => \state_udp_reg_n_0_[2]\,
      I4 => \state_udp[2]_i_3_n_0\,
      I5 => \state_udp_reg_n_0_[0]\,
      O => \state_udp[0]_i_1_n_0\
    );
\state_udp[0]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"5555555455555555"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp[0]_i_3_n_0\,
      I2 => \state_udp[0]_i_4_n_0\,
      I3 => \state_udp[0]_i_5_n_0\,
      I4 => \state_udp[0]_i_6_n_0\,
      I5 => \state_udp_nxt_reg_n_0_[0]\,
      O => \state_udp[0]_i_2_n_0\
    );
\state_udp[0]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => bit_dur_cnt(2),
      I1 => bit_dur_cnt(3),
      I2 => bit_dur_cnt(1),
      I3 => bit_dur_cnt(0),
      O => \state_udp[0]_i_3_n_0\
    );
\state_udp[0]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => bit_dur_cnt(6),
      I1 => bit_dur_cnt(7),
      I2 => bit_dur_cnt(4),
      I3 => bit_dur_cnt(5),
      O => \state_udp[0]_i_4_n_0\
    );
\state_udp[0]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => bit_dur_cnt(10),
      I1 => bit_dur_cnt(13),
      I2 => bit_dur_cnt(15),
      I3 => bit_dur_cnt(14),
      O => \state_udp[0]_i_5_n_0\
    );
\state_udp[0]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"EFFF"
    )
        port map (
      I0 => bit_dur_cnt(8),
      I1 => bit_dur_cnt(9),
      I2 => bit_dur_cnt(11),
      I3 => bit_dur_cnt(12),
      O => \state_udp[0]_i_6_n_0\
    );
\state_udp[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"00FCFFFF55550000"
    )
        port map (
      I0 => \state_udp[1]_i_2_n_0\,
      I1 => \state_udp_nxt_reg_n_0_[1]\,
      I2 => \state_udp[2]_i_2_n_0\,
      I3 => \state_udp_reg_n_0_[0]\,
      I4 => \state_udp[2]_i_3_n_0\,
      I5 => \state_udp_reg_n_0_[1]\,
      O => \state_udp[1]_i_1_n_0\
    );
\state_udp[1]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0000000155555555"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \bit_cnt[5]_i_2_n_0\,
      I2 => \bit_cnt_reg_n_0_[4]\,
      I3 => \bit_cnt_reg_n_0_[5]\,
      I4 => \state_udp[1]_i_3_n_0\,
      I5 => \state_udp_reg_n_0_[2]\,
      O => \state_udp[1]_i_2_n_0\
    );
\state_udp[1]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[7]\,
      I1 => \bit_cnt_reg_n_0_[6]\,
      O => \state_udp[1]_i_3_n_0\
    );
\state_udp[2]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0EFFFFFF0EE00000"
    )
        port map (
      I0 => \state_udp_nxt_reg_n_0_[2]\,
      I1 => \state_udp[2]_i_2_n_0\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \state_udp_reg_n_0_[1]\,
      I4 => \state_udp[2]_i_3_n_0\,
      I5 => \state_udp_reg_n_0_[2]\,
      O => \state_udp[2]_i_1_n_0\
    );
\state_udp[2]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFEFFFF"
    )
        port map (
      I0 => \state_udp[2]_i_4_n_0\,
      I1 => \state_udp[2]_i_5_n_0\,
      I2 => \state_udp[2]_i_6_n_0\,
      I3 => \state_udp[2]_i_7_n_0\,
      I4 => \state_udp_reg_n_0_[2]\,
      O => \state_udp[2]_i_2_n_0\
    );
\state_udp[2]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FEFEFEFF"
    )
        port map (
      I0 => \state_udp_reg_n_0_[1]\,
      I1 => \state_udp_reg_n_0_[0]\,
      I2 => \state_udp_reg_n_0_[2]\,
      I3 => \state_udp[2]_i_8_n_0\,
      I4 => \state_udp[2]_i_9_n_0\,
      O => \state_udp[2]_i_3_n_0\
    );
\state_udp[2]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"DFFF"
    )
        port map (
      I0 => bit_dur_cnt(7),
      I1 => bit_dur_cnt(13),
      I2 => bit_dur_cnt(12),
      I3 => bit_dur_cnt(3),
      O => \state_udp[2]_i_4_n_0\
    );
\state_udp[2]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FF7F"
    )
        port map (
      I0 => bit_dur_cnt(11),
      I1 => bit_dur_cnt(2),
      I2 => bit_dur_cnt(0),
      I3 => bit_dur_cnt(10),
      O => \state_udp[2]_i_5_n_0\
    );
\state_udp[2]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFDF"
    )
        port map (
      I0 => bit_dur_cnt(1),
      I1 => bit_dur_cnt(9),
      I2 => bit_dur_cnt(5),
      I3 => bit_dur_cnt(14),
      O => \state_udp[2]_i_6_n_0\
    );
\state_udp[2]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFDF"
    )
        port map (
      I0 => bit_dur_cnt(4),
      I1 => bit_dur_cnt(15),
      I2 => bit_dur_cnt(6),
      I3 => bit_dur_cnt(8),
      O => \state_udp[2]_i_7_n_0\
    );
\state_udp[2]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFF7"
    )
        port map (
      I0 => word_count(1),
      I1 => word_count(4),
      I2 => word_count(3),
      I3 => word_count(7),
      O => \state_udp[2]_i_8_n_0\
    );
\state_udp[2]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFEF"
    )
        port map (
      I0 => word_count(2),
      I1 => word_count(6),
      I2 => word_count(5),
      I3 => word_count(0),
      O => \state_udp[2]_i_9_n_0\
    );
\state_udp_nxt[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"8AFF8A00"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp_nxt[0]_i_2_n_0\,
      I2 => \word_count[7]_i_3_n_0\,
      I3 => \state_udp_nxt[1]_i_3_n_0\,
      I4 => \state_udp_nxt_reg_n_0_[0]\,
      O => \state_udp_nxt[0]_i_1_n_0\
    );
\state_udp_nxt[0]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"5455"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \bit_cnt_reg_n_0_[7]\,
      I2 => \bit_cnt_reg_n_0_[6]\,
      I3 => \bit_cnt[7]_i_3_n_0\,
      O => \state_udp_nxt[0]_i_2_n_0\
    );
\state_udp_nxt[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"4C44FFFF4C440000"
    )
        port map (
      I0 => \word_count[7]_i_3_n_0\,
      I1 => \state_udp_reg_n_0_[0]\,
      I2 => \state_udp_reg_n_0_[2]\,
      I3 => \state_udp_nxt[1]_i_2_n_0\,
      I4 => \state_udp_nxt[1]_i_3_n_0\,
      I5 => \state_udp_nxt_reg_n_0_[1]\,
      O => \state_udp_nxt[1]_i_1_n_0\
    );
\state_udp_nxt[1]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"02"
    )
        port map (
      I0 => \bit_cnt[7]_i_3_n_0\,
      I1 => \bit_cnt_reg_n_0_[6]\,
      I2 => \bit_cnt_reg_n_0_[7]\,
      O => \state_udp_nxt[1]_i_2_n_0\
    );
\state_udp_nxt[1]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"0055FEAA"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \bit_cnt[5]_i_2_n_0\,
      I2 => \state_udp_nxt[1]_i_4_n_0\,
      I3 => \state_udp_reg_n_0_[2]\,
      I4 => \state_udp_reg_n_0_[1]\,
      O => \state_udp_nxt[1]_i_3_n_0\
    );
\state_udp_nxt[1]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFFE"
    )
        port map (
      I0 => \bit_cnt_reg_n_0_[6]\,
      I1 => \bit_cnt_reg_n_0_[7]\,
      I2 => \bit_cnt_reg_n_0_[5]\,
      I3 => \bit_cnt_reg_n_0_[4]\,
      O => \state_udp_nxt[1]_i_4_n_0\
    );
\state_udp_nxt[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFF505B0"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => out_bit_prmbl_strt_i_2_n_0,
      I2 => \state_udp_reg_n_0_[2]\,
      I3 => \state_udp_reg_n_0_[1]\,
      I4 => \state_udp_nxt_reg_n_0_[2]\,
      O => \state_udp_nxt[2]_i_1_n_0\
    );
\state_udp_nxt_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \state_udp_nxt[0]_i_1_n_0\,
      Q => \state_udp_nxt_reg_n_0_[0]\
    );
\state_udp_nxt_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \state_udp_nxt[1]_i_1_n_0\,
      Q => \state_udp_nxt_reg_n_0_[1]\
    );
\state_udp_nxt_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \state_udp_nxt[2]_i_1_n_0\,
      Q => \state_udp_nxt_reg_n_0_[2]\
    );
\state_udp_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \state_udp[0]_i_1_n_0\,
      Q => \state_udp_reg_n_0_[0]\
    );
\state_udp_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \state_udp[1]_i_1_n_0\,
      Q => \state_udp_reg_n_0_[1]\
    );
\state_udp_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => '1',
      CLR => preamble_mod_n_0,
      D => \state_udp[2]_i_1_n_0\,
      Q => \state_udp_reg_n_0_[2]\
    );
\temp_Data[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"FFDF"
    )
        port map (
      I0 => \word_count_reg_rep_n_0_[3]\,
      I1 => \temp_Data[4]_i_2_n_0\,
      I2 => \state_udp_reg_n_0_[2]\,
      I3 => \temp_Data[7]_i_3_n_0\,
      O => \temp_Data[0]_i_1_n_0\
    );
\temp_Data[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8AA8888"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \temp_Data[7]_i_3_n_0\,
      I2 => \temp_Data[4]_i_2_n_0\,
      I3 => \word_count_reg_rep_n_0_[3]\,
      I4 => \word_count_reg_rep_n_0_[0]\,
      O => \temp_Data[1]_i_1_n_0\
    );
\temp_Data[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8AA8888"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \temp_Data[7]_i_3_n_0\,
      I2 => \temp_Data[4]_i_2_n_0\,
      I3 => \word_count_reg_rep_n_0_[3]\,
      I4 => \word_count_reg_rep_n_0_[1]\,
      O => \temp_Data[2]_i_1_n_0\
    );
\temp_Data[3]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8AA8888"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \temp_Data[7]_i_3_n_0\,
      I2 => \temp_Data[4]_i_2_n_0\,
      I3 => \word_count_reg_rep_n_0_[3]\,
      I4 => \word_count_reg_rep_n_0_[2]\,
      O => \temp_Data[3]_i_1_n_0\
    );
\temp_Data[4]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0080"
    )
        port map (
      I0 => \word_count_reg_rep_n_0_[3]\,
      I1 => \temp_Data[4]_i_2_n_0\,
      I2 => \state_udp_reg_n_0_[2]\,
      I3 => \temp_Data[7]_i_3_n_0\,
      O => \temp_Data[4]_i_1_n_0\
    );
\temp_Data[4]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"7FFF"
    )
        port map (
      I0 => \word_count_reg_rep_n_0_[6]\,
      I1 => \word_count_reg_rep_n_0_[5]\,
      I2 => \word_count_reg_rep_n_0_[4]\,
      I3 => \word_count_reg_rep_n_0_[7]\,
      O => \temp_Data[4]_i_2_n_0\
    );
\temp_Data[5]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A8"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \temp_Data[7]_i_3_n_0\,
      I2 => \temp_Data[5]_i_2_n_0\,
      O => \temp_Data[5]_i_1_n_0\
    );
\temp_Data[5]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFF0000"
    )
        port map (
      I0 => \word_count_reg_rep_n_0_[5]\,
      I1 => \word_count_reg_rep_n_0_[6]\,
      I2 => \word_count_reg_rep_n_0_[7]\,
      I3 => \word_count_reg_rep_n_0_[3]\,
      I4 => \word_count_reg_rep_n_0_[4]\,
      O => \temp_Data[5]_i_2_n_0\
    );
\temp_Data[6]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A8"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \temp_Data[7]_i_3_n_0\,
      I2 => \temp_Data[6]_i_2_n_0\,
      O => \temp_Data[6]_i_1_n_0\
    );
\temp_Data[6]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"7FFF0000"
    )
        port map (
      I0 => \word_count_reg_rep_n_0_[4]\,
      I1 => \word_count_reg_rep_n_0_[6]\,
      I2 => \word_count_reg_rep_n_0_[7]\,
      I3 => \word_count_reg_rep_n_0_[3]\,
      I4 => \word_count_reg_rep_n_0_[5]\,
      O => \temp_Data[6]_i_2_n_0\
    );
\temp_Data[7]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1044"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => out_bit_prmbl_strt_i_2_n_0,
      I3 => \state_udp_reg_n_0_[2]\,
      O => \temp_Data[7]_i_1_n_0\
    );
\temp_Data[7]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"A8AAAAAA88888888"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \temp_Data[7]_i_3_n_0\,
      I2 => \temp_Data[7]_i_4_n_0\,
      I3 => \word_count_reg_rep_n_0_[7]\,
      I4 => \word_count_reg_rep_n_0_[3]\,
      I5 => \word_count_reg_rep_n_0_[6]\,
      O => \temp_Data[7]_i_2_n_0\
    );
\temp_Data[7]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"80000000"
    )
        port map (
      I0 => word_count(3),
      I1 => word_count(4),
      I2 => word_count(6),
      I3 => word_count(5),
      I4 => word_count(7),
      O => \temp_Data[7]_i_3_n_0\
    );
\temp_Data[7]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"7"
    )
        port map (
      I0 => \word_count_reg_rep_n_0_[5]\,
      I1 => \word_count_reg_rep_n_0_[4]\,
      O => \temp_Data[7]_i_4_n_0\
    );
\temp_Data_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \temp_Data[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \temp_Data[0]_i_1_n_0\,
      Q => temp_Data(0)
    );
\temp_Data_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \temp_Data[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \temp_Data[1]_i_1_n_0\,
      Q => temp_Data(1)
    );
\temp_Data_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \temp_Data[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \temp_Data[2]_i_1_n_0\,
      Q => temp_Data(2)
    );
\temp_Data_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \temp_Data[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \temp_Data[3]_i_1_n_0\,
      Q => temp_Data(3)
    );
\temp_Data_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \temp_Data[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \temp_Data[4]_i_1_n_0\,
      Q => temp_Data(4)
    );
\temp_Data_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \temp_Data[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \temp_Data[5]_i_1_n_0\,
      Q => temp_Data(5)
    );
\temp_Data_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \temp_Data[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \temp_Data[6]_i_1_n_0\,
      Q => temp_Data(6)
    );
\temp_Data_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \temp_Data[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \temp_Data[7]_i_2_n_0\,
      Q => temp_Data(7)
    );
\word_count[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"2322"
    )
        port map (
      I0 => \state_udp_reg_n_0_[1]\,
      I1 => \state_udp_reg_n_0_[0]\,
      I2 => word_count(0),
      I3 => \word_count[5]_i_2_n_0\,
      O => \word_count[0]_i_1_n_0\
    );
\word_count[1]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00020200"
    )
        port map (
      I0 => \word_count[5]_i_2_n_0\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => word_count(0),
      I4 => word_count(1),
      O => \word_count[1]_i_1_n_0\
    );
\word_count[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"01111000"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => word_count(1),
      I3 => word_count(0),
      I4 => word_count(2),
      O => \word_count[2]_i_1_n_0\
    );
\word_count[3]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0111111110000000"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => word_count(0),
      I3 => word_count(1),
      I4 => word_count(2),
      I5 => word_count(3),
      O => \word_count[3]_i_1_n_0\
    );
\word_count[4]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"0200020200020000"
    )
        port map (
      I0 => \word_count[5]_i_2_n_0\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \state_udp_reg_n_0_[0]\,
      I3 => \word_count[6]_i_3_n_0\,
      I4 => word_count(3),
      I5 => word_count(4),
      O => \word_count[4]_i_1_n_0\
    );
\word_count[5]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8088888808000000"
    )
        port map (
      I0 => \word_count[5]_i_2_n_0\,
      I1 => \word_count[6]_i_2_n_0\,
      I2 => \word_count[6]_i_3_n_0\,
      I3 => word_count(3),
      I4 => word_count(4),
      I5 => word_count(5),
      O => \word_count[5]_i_1_n_0\
    );
\word_count[5]_i_2\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFFFFFEFFFF"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => \state_udp[2]_i_8_n_0\,
      I2 => word_count(2),
      I3 => word_count(6),
      I4 => word_count(5),
      I5 => word_count(0),
      O => \word_count[5]_i_2_n_0\
    );
\word_count[6]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AAAA2AAA00008000"
    )
        port map (
      I0 => \word_count[6]_i_2_n_0\,
      I1 => word_count(5),
      I2 => word_count(4),
      I3 => word_count(3),
      I4 => \word_count[6]_i_3_n_0\,
      I5 => word_count(6),
      O => \word_count[6]_i_1_n_0\
    );
\word_count[6]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \state_udp_reg_n_0_[1]\,
      I1 => \state_udp_reg_n_0_[0]\,
      O => \word_count[6]_i_2_n_0\
    );
\word_count[6]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"7F"
    )
        port map (
      I0 => word_count(0),
      I1 => word_count(1),
      I2 => word_count(2),
      O => \word_count[6]_i_3_n_0\
    );
\word_count[7]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"005D0F5D"
    )
        port map (
      I0 => \state_udp_reg_n_0_[2]\,
      I1 => out_bit_prmbl_strt_i_2_n_0,
      I2 => \state_udp_reg_n_0_[1]\,
      I3 => \state_udp_reg_n_0_[0]\,
      I4 => \word_count[7]_i_3_n_0\,
      O => \word_count[7]_i_1_n_0\
    );
\word_count[7]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"01111000"
    )
        port map (
      I0 => \state_udp_reg_n_0_[0]\,
      I1 => \state_udp_reg_n_0_[1]\,
      I2 => \word_count[7]_i_4_n_0\,
      I3 => word_count(6),
      I4 => word_count(7),
      O => \word_count[7]_i_2_n_0\
    );
\word_count[7]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FBFFFFFF"
    )
        port map (
      I0 => word_count(2),
      I1 => \state_udp_reg_n_0_[2]\,
      I2 => word_count(1),
      I3 => word_count(0),
      I4 => \temp_Data[7]_i_3_n_0\,
      O => \word_count[7]_i_3_n_0\
    );
\word_count[7]_i_4\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"8000000000000000"
    )
        port map (
      I0 => word_count(5),
      I1 => word_count(4),
      I2 => word_count(3),
      I3 => word_count(0),
      I4 => word_count(1),
      I5 => word_count(2),
      O => \word_count[7]_i_4_n_0\
    );
\word_count_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[0]_i_1_n_0\,
      Q => word_count(0)
    );
\word_count_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[1]_i_1_n_0\,
      Q => word_count(1)
    );
\word_count_reg[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[2]_i_1_n_0\,
      Q => word_count(2)
    );
\word_count_reg[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[3]_i_1_n_0\,
      Q => word_count(3)
    );
\word_count_reg[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[4]_i_1_n_0\,
      Q => word_count(4)
    );
\word_count_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[5]_i_1_n_0\,
      Q => word_count(5)
    );
\word_count_reg[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[6]_i_1_n_0\,
      Q => word_count(6)
    );
\word_count_reg[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[7]_i_2_n_0\,
      Q => word_count(7)
    );
\word_count_reg_rep[0]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[0]_i_1_n_0\,
      Q => \word_count_reg_rep_n_0_[0]\
    );
\word_count_reg_rep[1]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[1]_i_1_n_0\,
      Q => \word_count_reg_rep_n_0_[1]\
    );
\word_count_reg_rep[2]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[2]_i_1_n_0\,
      Q => \word_count_reg_rep_n_0_[2]\
    );
\word_count_reg_rep[3]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[3]_i_1_n_0\,
      Q => \word_count_reg_rep_n_0_[3]\
    );
\word_count_reg_rep[4]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[4]_i_1_n_0\,
      Q => \word_count_reg_rep_n_0_[4]\
    );
\word_count_reg_rep[5]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[5]_i_1_n_0\,
      Q => \word_count_reg_rep_n_0_[5]\
    );
\word_count_reg_rep[6]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[6]_i_1_n_0\,
      Q => \word_count_reg_rep_n_0_[6]\
    );
\word_count_reg_rep[7]\: unisim.vcomponents.FDCE
     port map (
      C => clock,
      CE => \word_count[7]_i_1_n_0\,
      CLR => preamble_mod_n_0,
      D => \word_count[7]_i_2_n_0\,
      Q => \word_count_reg_rep_n_0_[7]\
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
    out_bit_to_fsk : out STD_LOGIC;
    out_bit_valid : out STD_LOGIC;
    out_bit_prmbl_strt : out STD_LOGIC;
    out_bit_prmbl_end : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "system_Data_pack_preamble_2_0_0,Data_pack_preamble_2FSK_upd,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "module_ref";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "Data_pack_preamble_2FSK_upd,Vivado 2023.1";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of clock : signal is "xilinx.com:signal:clock:1.0 clock CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of clock : signal is "XIL_INTERFACENAME clock, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of out_bit_valid : signal is "analog.com:interface:fifo_rd:1.0 out_bit VALID";
  attribute X_INTERFACE_INFO of reset_n : signal is "xilinx.com:signal:reset:1.0 reset_n RST";
  attribute X_INTERFACE_PARAMETER of reset_n : signal is "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
begin
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_Data_pack_preamble_2FSK_upd
     port map (
      clock => clock,
      out_bit_prmbl_end => out_bit_prmbl_end,
      out_bit_prmbl_strt => out_bit_prmbl_strt,
      out_bit_to_fsk => out_bit_to_fsk,
      out_bit_valid => out_bit_valid,
      reset_n => reset_n
    );
end STRUCTURE;
