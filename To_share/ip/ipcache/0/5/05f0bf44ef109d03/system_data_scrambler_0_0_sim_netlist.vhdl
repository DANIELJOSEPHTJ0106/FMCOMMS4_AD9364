-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Wed Jan  7 18:03:57 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_data_scrambler_0_0_sim_netlist.vhdl
-- Design      : system_data_scrambler_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_scrambler is
  port (
    validout : out STD_LOGIC;
    dataout : out STD_LOGIC_VECTOR ( 0 to 0 );
    clk : in STD_LOGIC;
    validin : in STD_LOGIC;
    datain : in STD_LOGIC;
    preamble_start : in STD_LOGIC;
    preamble_end : in STD_LOGIC;
    rst_n : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_scrambler;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_scrambler is
  signal \FSM_onehot_cntr_stm[0]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm[1]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm[2]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_reg_n_0_[0]\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_reg_n_0_[1]\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_reg_n_0_[2]\ : STD_LOGIC;
  signal \^dataout\ : STD_LOGIC_VECTOR ( 0 to 0 );
  signal \dataout[0]_i_1_n_0\ : STD_LOGIC;
  signal \dataout[0]_i_2_n_0\ : STD_LOGIC;
  signal \dataout[0]_i_3_n_0\ : STD_LOGIC;
  signal prev_data_in : STD_LOGIC;
  signal prev_preamble_end : STD_LOGIC;
  signal prev_preamble_start : STD_LOGIC;
  signal prev_prev_data_in : STD_LOGIC;
  signal prev_prev_valid_in : STD_LOGIC;
  signal prev_valid_in : STD_LOGIC;
  signal state : STD_LOGIC_VECTOR ( 5 downto 1 );
  signal \state[0]_i_1_n_0\ : STD_LOGIC;
  signal \state[2]_i_1_n_0\ : STD_LOGIC;
  signal \state[3]_i_1_n_0\ : STD_LOGIC;
  signal \state[4]_i_1_n_0\ : STD_LOGIC;
  signal \state[6]_i_1_n_0\ : STD_LOGIC;
  signal \state_reg_n_0_[0]\ : STD_LOGIC;
  signal \state_reg_n_0_[1]\ : STD_LOGIC;
  signal \state_reg_n_0_[2]\ : STD_LOGIC;
  signal \state_reg_n_0_[3]\ : STD_LOGIC;
  signal \state_reg_n_0_[4]\ : STD_LOGIC;
  signal \state_reg_n_0_[5]\ : STD_LOGIC;
  signal \state_reg_n_0_[6]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \FSM_onehot_cntr_stm[2]_i_1\ : label is "soft_lutpair0";
  attribute FSM_ENCODED_STATES : string;
  attribute FSM_ENCODED_STATES of \FSM_onehot_cntr_stm_reg[0]\ : label is "iSTATE:010,iSTATE0:100,iSTATE1:001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_cntr_stm_reg[1]\ : label is "iSTATE:010,iSTATE0:100,iSTATE1:001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_cntr_stm_reg[2]\ : label is "iSTATE:010,iSTATE0:100,iSTATE1:001";
  attribute SOFT_HLUTNM of \dataout[0]_i_3\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \state[0]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \state[1]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \state[3]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \state[4]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \state[5]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \state[6]_i_1\ : label is "soft_lutpair1";
begin
  dataout(0) <= \^dataout\(0);
\FSM_onehot_cntr_stm[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFD0D0D0"
    )
        port map (
      I0 => prev_preamble_start,
      I1 => preamble_start,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      I3 => validin,
      I4 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      O => \FSM_onehot_cntr_stm[0]_i_1_n_0\
    );
\FSM_onehot_cntr_stm[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFF20FF20202020"
    )
        port map (
      I0 => prev_preamble_start,
      I1 => preamble_start,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      I3 => prev_preamble_end,
      I4 => preamble_end,
      I5 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      O => \FSM_onehot_cntr_stm[1]_i_1_n_0\
    );
\FSM_onehot_cntr_stm[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"20FF2020"
    )
        port map (
      I0 => prev_preamble_end,
      I1 => preamble_end,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      I3 => validin,
      I4 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      O => \FSM_onehot_cntr_stm[2]_i_1_n_0\
    );
\FSM_onehot_cntr_stm_reg[0]\: unisim.vcomponents.FDPE
    generic map(
      INIT => '1'
    )
        port map (
      C => clk,
      CE => '1',
      D => \FSM_onehot_cntr_stm[0]_i_1_n_0\,
      PRE => \dataout[0]_i_2_n_0\,
      Q => \FSM_onehot_cntr_stm_reg_n_0_[0]\
    );
\FSM_onehot_cntr_stm_reg[1]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clk,
      CE => '1',
      CLR => \dataout[0]_i_2_n_0\,
      D => \FSM_onehot_cntr_stm[1]_i_1_n_0\,
      Q => \FSM_onehot_cntr_stm_reg_n_0_[1]\
    );
\FSM_onehot_cntr_stm_reg[2]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clk,
      CE => '1',
      CLR => \dataout[0]_i_2_n_0\,
      D => \FSM_onehot_cntr_stm[2]_i_1_n_0\,
      Q => \FSM_onehot_cntr_stm_reg_n_0_[2]\
    );
\dataout[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"AA3CAAFFAA3CAA00"
    )
        port map (
      I0 => prev_prev_data_in,
      I1 => \state_reg_n_0_[6]\,
      I2 => prev_data_in,
      I3 => \dataout[0]_i_3_n_0\,
      I4 => prev_prev_valid_in,
      I5 => \^dataout\(0),
      O => \dataout[0]_i_1_n_0\
    );
\dataout[0]_i_2\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => rst_n,
      O => \dataout[0]_i_2_n_0\
    );
\dataout[0]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"E"
    )
        port map (
      I0 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      O => \dataout[0]_i_3_n_0\
    );
\dataout_reg[0]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => \dataout[0]_i_2_n_0\,
      D => \dataout[0]_i_1_n_0\,
      Q => \^dataout\(0)
    );
prev_data_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => datain,
      Q => prev_data_in,
      R => \dataout[0]_i_2_n_0\
    );
prev_preamble_end_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => preamble_end,
      Q => prev_preamble_end,
      R => \dataout[0]_i_2_n_0\
    );
prev_preamble_start_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => preamble_start,
      Q => prev_preamble_start,
      R => \dataout[0]_i_2_n_0\
    );
prev_prev_data_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => prev_data_in,
      Q => prev_prev_data_in,
      R => \dataout[0]_i_2_n_0\
    );
prev_prev_valid_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => prev_valid_in,
      Q => prev_prev_valid_in,
      R => \dataout[0]_i_2_n_0\
    );
prev_valid_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => validin,
      Q => prev_valid_in,
      R => \dataout[0]_i_2_n_0\
    );
\state[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"6F"
    )
        port map (
      I0 => prev_data_in,
      I1 => \state_reg_n_0_[6]\,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      O => \state[0]_i_1_n_0\
    );
\state[1]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      I1 => \state_reg_n_0_[0]\,
      O => state(1)
    );
\state[2]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => \state_reg_n_0_[1]\,
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      O => \state[2]_i_1_n_0\
    );
\state[3]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => \state_reg_n_0_[2]\,
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      O => \state[3]_i_1_n_0\
    );
\state[4]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => \state_reg_n_0_[3]\,
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      O => \state[4]_i_1_n_0\
    );
\state[5]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"8"
    )
        port map (
      I0 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      I1 => \state_reg_n_0_[4]\,
      O => state(5)
    );
\state[6]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"B"
    )
        port map (
      I0 => \state_reg_n_0_[5]\,
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      O => \state[6]_i_1_n_0\
    );
\state_reg[0]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => prev_prev_valid_in,
      D => \state[0]_i_1_n_0\,
      PRE => \dataout[0]_i_2_n_0\,
      Q => \state_reg_n_0_[0]\
    );
\state_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => prev_prev_valid_in,
      CLR => \dataout[0]_i_2_n_0\,
      D => state(1),
      Q => \state_reg_n_0_[1]\
    );
\state_reg[2]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => prev_prev_valid_in,
      D => \state[2]_i_1_n_0\,
      PRE => \dataout[0]_i_2_n_0\,
      Q => \state_reg_n_0_[2]\
    );
\state_reg[3]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => prev_prev_valid_in,
      D => \state[3]_i_1_n_0\,
      PRE => \dataout[0]_i_2_n_0\,
      Q => \state_reg_n_0_[3]\
    );
\state_reg[4]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => prev_prev_valid_in,
      D => \state[4]_i_1_n_0\,
      PRE => \dataout[0]_i_2_n_0\,
      Q => \state_reg_n_0_[4]\
    );
\state_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => prev_prev_valid_in,
      CLR => \dataout[0]_i_2_n_0\,
      D => state(5),
      Q => \state_reg_n_0_[5]\
    );
\state_reg[6]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => prev_prev_valid_in,
      D => \state[6]_i_1_n_0\,
      PRE => \dataout[0]_i_2_n_0\,
      Q => \state_reg_n_0_[6]\
    );
validout_reg: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => \dataout[0]_i_2_n_0\,
      D => prev_prev_valid_in,
      Q => validout
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  port (
    clk : in STD_LOGIC;
    rst_n : in STD_LOGIC;
    datain : in STD_LOGIC;
    validin : in STD_LOGIC;
    preamble_start : in STD_LOGIC;
    preamble_end : in STD_LOGIC;
    dataout : out STD_LOGIC_VECTOR ( 7 downto 0 );
    validout : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "system_data_scrambler_0_0,data_scrambler,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "module_ref";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "data_scrambler,Vivado 2023.1";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  signal \<const0>\ : STD_LOGIC;
  signal \^dataout\ : STD_LOGIC_VECTOR ( 0 to 0 );
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of clk : signal is "xilinx.com:signal:clock:1.0 clk CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of clk : signal is "XIL_INTERFACENAME clk, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of rst_n : signal is "xilinx.com:signal:reset:1.0 rst_n RST";
  attribute X_INTERFACE_PARAMETER of rst_n : signal is "XIL_INTERFACENAME rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
begin
  dataout(7) <= \<const0>\;
  dataout(6) <= \<const0>\;
  dataout(5) <= \<const0>\;
  dataout(4) <= \<const0>\;
  dataout(3) <= \<const0>\;
  dataout(2) <= \<const0>\;
  dataout(1) <= \<const0>\;
  dataout(0) <= \^dataout\(0);
GND: unisim.vcomponents.GND
     port map (
      G => \<const0>\
    );
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_scrambler
     port map (
      clk => clk,
      datain => datain,
      dataout(0) => \^dataout\(0),
      preamble_end => preamble_end,
      preamble_start => preamble_start,
      rst_n => rst_n,
      validin => validin,
      validout => validout
    );
end STRUCTURE;
