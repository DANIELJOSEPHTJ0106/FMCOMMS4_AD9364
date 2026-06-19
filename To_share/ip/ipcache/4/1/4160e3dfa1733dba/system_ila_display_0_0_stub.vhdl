-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Sat Jan  3 12:25:20 2026
-- Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode synth_stub
--               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_ila_display_0_0/system_ila_display_0_0_stub.vhdl
-- Design      : system_ila_display_0_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity system_ila_display_0_0 is
  Port ( 
    clock_61 : in STD_LOGIC;
    clock_100 : in STD_LOGIC;
    reset_n : in STD_LOGIC;
    pattern_gen_dat_out : in STD_LOGIC_VECTOR ( 7 downto 0 );
    pattern_gen_val_out : in STD_LOGIC;
    bsync_dat : in STD_LOGIC_VECTOR ( 7 downto 0 );
    bsync_val : in STD_LOGIC;
    ila_pattern_gen_dat : out STD_LOGIC_VECTOR ( 7 downto 0 );
    ila_pattern_gen_val : out STD_LOGIC;
    ila_bsync_dat : out STD_LOGIC_VECTOR ( 7 downto 0 );
    ila_bsync_val : out STD_LOGIC
  );

end system_ila_display_0_0;

architecture stub of system_ila_display_0_0 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clock_61,clock_100,reset_n,pattern_gen_dat_out[7:0],pattern_gen_val_out,bsync_dat[7:0],bsync_val,ila_pattern_gen_dat[7:0],ila_pattern_gen_val,ila_bsync_dat[7:0],ila_bsync_val";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "ila_display,Vivado 2023.1";
begin
end;
