-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Sat Jan  3 12:24:46 2026
-- Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode synth_stub
--               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_fsk_ddc_0_0/system_fsk_ddc_0_0_stub.vhdl
-- Design      : system_fsk_ddc_0_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity system_fsk_ddc_0_0 is
  Port ( 
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    rx_in_TVALID : in STD_LOGIC;
    rx_in_TREADY : out STD_LOGIC;
    rx_in_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    rx_in_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    rx_in_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    rx_in_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    dds_lo_TVALID : in STD_LOGIC;
    dds_lo_TREADY : out STD_LOGIC;
    dds_lo_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    dds_lo_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    dds_lo_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    dds_lo_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    baseband_out_TVALID : out STD_LOGIC;
    baseband_out_TREADY : in STD_LOGIC;
    baseband_out_TDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    baseband_out_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    baseband_out_TKEEP : out STD_LOGIC_VECTOR ( 3 downto 0 );
    baseband_out_TSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 )
  );

end system_fsk_ddc_0_0;

architecture stub of system_fsk_ddc_0_0 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "ap_clk,ap_rst_n,rx_in_TVALID,rx_in_TREADY,rx_in_TDATA[31:0],rx_in_TLAST[0:0],rx_in_TKEEP[3:0],rx_in_TSTRB[3:0],dds_lo_TVALID,dds_lo_TREADY,dds_lo_TDATA[31:0],dds_lo_TLAST[0:0],dds_lo_TKEEP[3:0],dds_lo_TSTRB[3:0],baseband_out_TVALID,baseband_out_TREADY,baseband_out_TDATA[31:0],baseband_out_TLAST[0:0],baseband_out_TKEEP[3:0],baseband_out_TSTRB[3:0]";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "fsk_ddc,Vivado 2023.1";
begin
end;
