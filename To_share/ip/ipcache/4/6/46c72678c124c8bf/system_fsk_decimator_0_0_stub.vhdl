-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Tue Feb  3 16:25:28 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode synth_stub -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_fsk_decimator_0_0_stub.vhdl
-- Design      : system_fsk_decimator_0_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  Port ( 
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    rx_in_TVALID : in STD_LOGIC;
    rx_in_TREADY : out STD_LOGIC;
    rx_in_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    rx_in_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    rx_in_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    rx_in_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    dec_out_TVALID : out STD_LOGIC;
    dec_out_TREADY : in STD_LOGIC;
    dec_out_TDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    dec_out_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    dec_out_TKEEP : out STD_LOGIC_VECTOR ( 3 downto 0 );
    dec_out_TSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 )
  );

end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture stub of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "ap_clk,ap_rst_n,rx_in_TVALID,rx_in_TREADY,rx_in_TDATA[31:0],rx_in_TLAST[0:0],rx_in_TKEEP[3:0],rx_in_TSTRB[3:0],dec_out_TVALID,dec_out_TREADY,dec_out_TDATA[31:0],dec_out_TLAST[0:0],dec_out_TKEEP[3:0],dec_out_TSTRB[3:0]";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "fsk_decimator,Vivado 2023.1";
begin
end;
