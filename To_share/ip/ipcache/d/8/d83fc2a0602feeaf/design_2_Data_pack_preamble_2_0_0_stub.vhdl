-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Wed Jan  7 13:01:31 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ design_2_Data_pack_preamble_2_0_0_stub.vhdl
-- Design      : design_2_Data_pack_preamble_2_0_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  Port ( 
    clock : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    out_bit_to_fsk : out STD_LOGIC;
    out_bit_valid : out STD_LOGIC;
    out_bit_prmbl_strt : out STD_LOGIC;
    out_bit_prmbl_end : out STD_LOGIC
  );

end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture stub of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clock,reset_n,out_bit_to_fsk,out_bit_valid,out_bit_prmbl_strt,out_bit_prmbl_end";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "Data_pack_preamble_2FSK_upd,Vivado 2023.1";
begin
end;
