-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Sat Jan  3 12:24:47 2026
-- Host        : iRFMW running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode synth_stub
--               /home/mrg99/Desktop/Github_repo/hdl/projects/fmcomms2/Zed_fsk_hls_IP/fmcomms2_zed.gen/sources_1/bd/system/ip/system_gaussian_shaping_v2_0_0/system_gaussian_shaping_v2_0_0_stub.vhdl
-- Design      : system_gaussian_shaping_v2_0_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity system_gaussian_shaping_v2_0_0 is
  Port ( 
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TREADY : out STD_LOGIC;
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 7 downto 0 );
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    in_stream_TKEEP : in STD_LOGIC_VECTOR ( 0 to 0 );
    in_stream_TSTRB : in STD_LOGIC_VECTOR ( 0 to 0 );
    in_stream_TUSER : in STD_LOGIC_VECTOR ( 12 downto 0 );
    fcw_out_TVALID : out STD_LOGIC;
    fcw_out_TREADY : in STD_LOGIC;
    fcw_out_TDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    fcw_out_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    fcw_out_TKEEP : out STD_LOGIC_VECTOR ( 3 downto 0 );
    fcw_out_TSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 )
  );

end system_gaussian_shaping_v2_0_0;

architecture stub of system_gaussian_shaping_v2_0_0 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "ap_clk,ap_rst_n,in_stream_TVALID,in_stream_TREADY,in_stream_TDATA[7:0],in_stream_TLAST[0:0],in_stream_TKEEP[0:0],in_stream_TSTRB[0:0],in_stream_TUSER[12:0],fcw_out_TVALID,fcw_out_TREADY,fcw_out_TDATA[31:0],fcw_out_TLAST[0:0],fcw_out_TKEEP[3:0],fcw_out_TSTRB[3:0]";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "gaussian_shaping_v2,Vivado 2023.1";
begin
end;
