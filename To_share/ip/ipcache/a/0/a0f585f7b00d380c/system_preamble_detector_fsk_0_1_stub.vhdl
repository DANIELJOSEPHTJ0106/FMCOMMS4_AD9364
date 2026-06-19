-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Mon Jan 12 13:34:42 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_preamble_detector_fsk_0_1_stub.vhdl
-- Design      : system_preamble_detector_fsk_0_1
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  Port ( 
    clock : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    bsync_dat_in : in STD_LOGIC_VECTOR ( 7 downto 0 );
    bsync_val_in : in STD_LOGIC;
    data_det_out : out STD_LOGIC;
    data_det_val_out : out STD_LOGIC;
    data_det_start : out STD_LOGIC;
    data_det_end : out STD_LOGIC
  );

end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture stub of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clock,reset_n,bsync_dat_in[7:0],bsync_val_in,data_det_out,data_det_val_out,data_det_start,data_det_end";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "preamble_detector_fsk,Vivado 2023.1";
begin
end;
