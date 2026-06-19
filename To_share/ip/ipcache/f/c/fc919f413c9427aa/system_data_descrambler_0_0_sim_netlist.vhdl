-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Wed Jan  7 18:04:02 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_data_descrambler_0_0_sim_netlist.vhdl
-- Design      : system_data_descrambler_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_descrambler is
  port (
    dataout1 : out STD_LOGIC;
    validout : out STD_LOGIC;
    clk : in STD_LOGIC;
    rst_n : in STD_LOGIC;
    validin : in STD_LOGIC;
    data_start : in STD_LOGIC;
    data_end : in STD_LOGIC;
    datain : in STD_LOGIC
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_descrambler;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_descrambler is
  signal \FSM_onehot_cntr_stm_nxt_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_nxt_reg[1]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_nxt_reg[2]_i_1_n_0\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_nxt_reg_n_0_[0]\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_nxt_reg_n_0_[1]\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_nxt_reg_n_0_[2]\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_reg_n_0_[0]\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_reg_n_0_[1]\ : STD_LOGIC;
  signal \FSM_onehot_cntr_stm_reg_n_0_[2]\ : STD_LOGIC;
  signal \__1/i__n_0\ : STD_LOGIC;
  signal dataout1_i_1_n_0 : STD_LOGIC;
  signal p_0_in : STD_LOGIC_VECTOR ( 6 downto 0 );
  signal p_prev_prev_data_in : STD_LOGIC;
  signal p_prev_prev_valid_in : STD_LOGIC;
  signal prev_data_end : STD_LOGIC;
  signal prev_data_in : STD_LOGIC;
  signal prev_data_start : STD_LOGIC;
  signal prev_prev_valid_in : STD_LOGIC;
  signal prev_valid_in : STD_LOGIC;
  signal \state[0]_i_1_n_0\ : STD_LOGIC;
  signal \state[1]_i_1_n_0\ : STD_LOGIC;
  signal \state[2]_i_1_n_0\ : STD_LOGIC;
  signal \state[3]_i_1_n_0\ : STD_LOGIC;
  signal \state[4]_i_1_n_0\ : STD_LOGIC;
  signal \state[5]_i_1_n_0\ : STD_LOGIC;
  signal \state[6]_i_1_n_0\ : STD_LOGIC;
  signal \state_reg_n_0_[6]\ : STD_LOGIC;
  signal validout_i_1_n_0 : STD_LOGIC;
  attribute XILINX_LEGACY_PRIM : string;
  attribute XILINX_LEGACY_PRIM of \FSM_onehot_cntr_stm_nxt_reg[0]\ : label is "LDP";
  attribute XILINX_TRANSFORM_PINMAP : string;
  attribute XILINX_TRANSFORM_PINMAP of \FSM_onehot_cntr_stm_nxt_reg[0]\ : label is "VCC:GE";
  attribute XILINX_LEGACY_PRIM of \FSM_onehot_cntr_stm_nxt_reg[1]\ : label is "LDC";
  attribute XILINX_TRANSFORM_PINMAP of \FSM_onehot_cntr_stm_nxt_reg[1]\ : label is "VCC:GE";
  attribute XILINX_LEGACY_PRIM of \FSM_onehot_cntr_stm_nxt_reg[2]\ : label is "LDC";
  attribute XILINX_TRANSFORM_PINMAP of \FSM_onehot_cntr_stm_nxt_reg[2]\ : label is "VCC:GE";
  attribute FSM_ENCODED_STATES : string;
  attribute FSM_ENCODED_STATES of \FSM_onehot_cntr_stm_reg[0]\ : label is "iSTATE:010,iSTATE0:100,iSTATE1:001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_cntr_stm_reg[1]\ : label is "iSTATE:010,iSTATE0:100,iSTATE1:001";
  attribute FSM_ENCODED_STATES of \FSM_onehot_cntr_stm_reg[2]\ : label is "iSTATE:010,iSTATE0:100,iSTATE1:001";
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \__1/i_\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \state[0]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \state[1]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \state[2]_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \state[3]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \state[4]_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \state[5]_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of validout_i_1 : label is "soft_lutpair0";
begin
\FSM_onehot_cntr_stm_nxt_reg[0]\: unisim.vcomponents.LDPE
    generic map(
      INIT => '1'
    )
        port map (
      D => \FSM_onehot_cntr_stm_nxt_reg[0]_i_1_n_0\,
      G => rst_n,
      GE => '1',
      PRE => dataout1_i_1_n_0,
      Q => \FSM_onehot_cntr_stm_nxt_reg_n_0_[0]\
    );
\FSM_onehot_cntr_stm_nxt_reg[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFD0D0D0"
    )
        port map (
      I0 => data_start,
      I1 => prev_data_start,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      I3 => validin,
      I4 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      O => \FSM_onehot_cntr_stm_nxt_reg[0]_i_1_n_0\
    );
\FSM_onehot_cntr_stm_nxt_reg[1]\: unisim.vcomponents.LDCE
    generic map(
      INIT => '0'
    )
        port map (
      CLR => dataout1_i_1_n_0,
      D => \FSM_onehot_cntr_stm_nxt_reg[1]_i_1_n_0\,
      G => rst_n,
      GE => '1',
      Q => \FSM_onehot_cntr_stm_nxt_reg_n_0_[1]\
    );
\FSM_onehot_cntr_stm_nxt_reg[1]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFF20FF20202020"
    )
        port map (
      I0 => data_start,
      I1 => prev_data_start,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[0]\,
      I3 => prev_data_end,
      I4 => data_end,
      I5 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      O => \FSM_onehot_cntr_stm_nxt_reg[1]_i_1_n_0\
    );
\FSM_onehot_cntr_stm_nxt_reg[2]\: unisim.vcomponents.LDCE
    generic map(
      INIT => '0'
    )
        port map (
      CLR => dataout1_i_1_n_0,
      D => \FSM_onehot_cntr_stm_nxt_reg[2]_i_1_n_0\,
      G => rst_n,
      GE => '1',
      Q => \FSM_onehot_cntr_stm_nxt_reg_n_0_[2]\
    );
\FSM_onehot_cntr_stm_nxt_reg[2]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"20FF2020"
    )
        port map (
      I0 => prev_data_end,
      I1 => data_end,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      I3 => validin,
      I4 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      O => \FSM_onehot_cntr_stm_nxt_reg[2]_i_1_n_0\
    );
\FSM_onehot_cntr_stm_reg[0]\: unisim.vcomponents.FDPE
    generic map(
      INIT => '1'
    )
        port map (
      C => clk,
      CE => '1',
      D => \FSM_onehot_cntr_stm_nxt_reg_n_0_[0]\,
      PRE => dataout1_i_1_n_0,
      Q => \FSM_onehot_cntr_stm_reg_n_0_[0]\
    );
\FSM_onehot_cntr_stm_reg[1]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clk,
      CE => '1',
      CLR => dataout1_i_1_n_0,
      D => \FSM_onehot_cntr_stm_nxt_reg_n_0_[1]\,
      Q => \FSM_onehot_cntr_stm_reg_n_0_[1]\
    );
\FSM_onehot_cntr_stm_reg[2]\: unisim.vcomponents.FDCE
    generic map(
      INIT => '0'
    )
        port map (
      C => clk,
      CE => '1',
      CLR => dataout1_i_1_n_0,
      D => \FSM_onehot_cntr_stm_nxt_reg_n_0_[2]\,
      Q => \FSM_onehot_cntr_stm_reg_n_0_[2]\
    );
\__1/i_\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"28282800"
    )
        port map (
      I0 => p_prev_prev_valid_in,
      I1 => \state_reg_n_0_[6]\,
      I2 => p_prev_prev_data_in,
      I3 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      I4 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      O => \__1/i__n_0\
    );
dataout1_i_1: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => rst_n,
      O => dataout1_i_1_n_0
    );
dataout1_reg: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => dataout1_i_1_n_0,
      D => \__1/i__n_0\,
      Q => dataout1
    );
p_prev_prev_data_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => p_0_in(0),
      Q => p_prev_prev_data_in,
      R => dataout1_i_1_n_0
    );
p_prev_prev_valid_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => prev_prev_valid_in,
      Q => p_prev_prev_valid_in,
      R => dataout1_i_1_n_0
    );
prev_data_end_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => data_end,
      Q => prev_data_end,
      R => dataout1_i_1_n_0
    );
prev_data_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => datain,
      Q => prev_data_in,
      R => dataout1_i_1_n_0
    );
prev_data_start_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => data_start,
      Q => prev_data_start,
      R => dataout1_i_1_n_0
    );
prev_prev_data_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => prev_data_in,
      Q => p_0_in(0),
      R => dataout1_i_1_n_0
    );
prev_prev_valid_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => prev_valid_in,
      Q => prev_prev_valid_in,
      R => dataout1_i_1_n_0
    );
prev_valid_in_reg: unisim.vcomponents.FDRE
     port map (
      C => clk,
      CE => '1',
      D => validin,
      Q => prev_valid_in,
      R => dataout1_i_1_n_0
    );
\state[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AB"
    )
        port map (
      I0 => p_0_in(0),
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      O => \state[0]_i_1_n_0\
    );
\state[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"E0"
    )
        port map (
      I0 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      I2 => p_0_in(1),
      O => \state[1]_i_1_n_0\
    );
\state[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AB"
    )
        port map (
      I0 => p_0_in(2),
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      O => \state[2]_i_1_n_0\
    );
\state[3]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AB"
    )
        port map (
      I0 => p_0_in(3),
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      O => \state[3]_i_1_n_0\
    );
\state[4]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AB"
    )
        port map (
      I0 => p_0_in(4),
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      O => \state[4]_i_1_n_0\
    );
\state[5]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"E0"
    )
        port map (
      I0 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      I2 => p_0_in(5),
      O => \state[5]_i_1_n_0\
    );
\state[6]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AB"
    )
        port map (
      I0 => p_0_in(6),
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      O => \state[6]_i_1_n_0\
    );
\state_reg[0]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => p_prev_prev_valid_in,
      D => \state[0]_i_1_n_0\,
      PRE => dataout1_i_1_n_0,
      Q => p_0_in(1)
    );
\state_reg[1]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => p_prev_prev_valid_in,
      CLR => dataout1_i_1_n_0,
      D => \state[1]_i_1_n_0\,
      Q => p_0_in(2)
    );
\state_reg[2]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => p_prev_prev_valid_in,
      D => \state[2]_i_1_n_0\,
      PRE => dataout1_i_1_n_0,
      Q => p_0_in(3)
    );
\state_reg[3]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => p_prev_prev_valid_in,
      D => \state[3]_i_1_n_0\,
      PRE => dataout1_i_1_n_0,
      Q => p_0_in(4)
    );
\state_reg[4]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => p_prev_prev_valid_in,
      D => \state[4]_i_1_n_0\,
      PRE => dataout1_i_1_n_0,
      Q => p_0_in(5)
    );
\state_reg[5]\: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => p_prev_prev_valid_in,
      CLR => dataout1_i_1_n_0,
      D => \state[5]_i_1_n_0\,
      Q => p_0_in(6)
    );
\state_reg[6]\: unisim.vcomponents.FDPE
     port map (
      C => clk,
      CE => p_prev_prev_valid_in,
      D => \state[6]_i_1_n_0\,
      PRE => dataout1_i_1_n_0,
      Q => \state_reg_n_0_[6]\
    );
validout_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A8"
    )
        port map (
      I0 => p_prev_prev_valid_in,
      I1 => \FSM_onehot_cntr_stm_reg_n_0_[2]\,
      I2 => \FSM_onehot_cntr_stm_reg_n_0_[1]\,
      O => validout_i_1_n_0
    );
validout_reg: unisim.vcomponents.FDCE
     port map (
      C => clk,
      CE => '1',
      CLR => dataout1_i_1_n_0,
      D => validout_i_1_n_0,
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
    data_start : in STD_LOGIC;
    data_end : in STD_LOGIC;
    dataout1 : out STD_LOGIC;
    validout : out STD_LOGIC
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "system_data_descrambler_0_0,data_descrambler,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "module_ref";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "data_descrambler,Vivado 2023.1";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of clk : signal is "xilinx.com:signal:clock:1.0 clk CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of clk : signal is "XIL_INTERFACENAME clk, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of rst_n : signal is "xilinx.com:signal:reset:1.0 rst_n RST";
  attribute X_INTERFACE_PARAMETER of rst_n : signal is "XIL_INTERFACENAME rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
begin
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_data_descrambler
     port map (
      clk => clk,
      data_end => data_end,
      data_start => data_start,
      datain => datain,
      dataout1 => dataout1,
      rst_n => rst_n,
      validin => validin,
      validout => validout
    );
end STRUCTURE;
