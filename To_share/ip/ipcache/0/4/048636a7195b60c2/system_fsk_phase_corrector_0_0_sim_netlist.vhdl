-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2023.1 (lin64) Build 3865809 Sun May  7 15:04:56 MDT 2023
-- Date        : Tue Feb  3 16:25:29 2026
-- Host        : rfmw running 64-bit Ubuntu 22.04.5 LTS
-- Command     : write_vhdl -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
--               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ system_fsk_phase_corrector_0_0_sim_netlist.vhdl
-- Design      : system_fsk_phase_corrector_0_0
-- Purpose     : This VHDL netlist is a functional simulation representation of the design and should not be modified or
--               synthesized. This netlist cannot be used for SDF annotated simulation.
-- Device      : xc7z020clg484-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_mul_32s_34ns_65_2_1 is
  port (
    S : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_reg[18]\ : out STD_LOGIC_VECTOR ( 2 downto 0 );
    \sum_reg[22]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_reg[26]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    D : out STD_LOGIC_VECTOR ( 41 downto 0 );
    \buff0_reg__0_0\ : out STD_LOGIC_VECTOR ( 15 downto 0 );
    add_ln32_reg_2830 : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    add_ln32_fu_145_p2 : in STD_LOGIC_VECTOR ( 31 downto 0 );
    ap_block_pp0_stage0_11001 : in STD_LOGIC;
    sum_reg : in STD_LOGIC_VECTOR ( 15 downto 0 )
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_mul_32s_34ns_65_2_1;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_mul_32s_34ns_65_2_1 is
  signal \buff0_reg[16]__0_n_0\ : STD_LOGIC;
  signal \buff0_reg__0_n_100\ : STD_LOGIC;
  signal \buff0_reg__0_n_101\ : STD_LOGIC;
  signal \buff0_reg__0_n_102\ : STD_LOGIC;
  signal \buff0_reg__0_n_103\ : STD_LOGIC;
  signal \buff0_reg__0_n_104\ : STD_LOGIC;
  signal \buff0_reg__0_n_105\ : STD_LOGIC;
  signal \buff0_reg__0_n_58\ : STD_LOGIC;
  signal \buff0_reg__0_n_59\ : STD_LOGIC;
  signal \buff0_reg__0_n_60\ : STD_LOGIC;
  signal \buff0_reg__0_n_61\ : STD_LOGIC;
  signal \buff0_reg__0_n_62\ : STD_LOGIC;
  signal \buff0_reg__0_n_63\ : STD_LOGIC;
  signal \buff0_reg__0_n_64\ : STD_LOGIC;
  signal \buff0_reg__0_n_65\ : STD_LOGIC;
  signal \buff0_reg__0_n_66\ : STD_LOGIC;
  signal \buff0_reg__0_n_67\ : STD_LOGIC;
  signal \buff0_reg__0_n_68\ : STD_LOGIC;
  signal \buff0_reg__0_n_69\ : STD_LOGIC;
  signal \buff0_reg__0_n_70\ : STD_LOGIC;
  signal \buff0_reg__0_n_71\ : STD_LOGIC;
  signal \buff0_reg__0_n_72\ : STD_LOGIC;
  signal \buff0_reg__0_n_73\ : STD_LOGIC;
  signal \buff0_reg__0_n_74\ : STD_LOGIC;
  signal \buff0_reg__0_n_75\ : STD_LOGIC;
  signal \buff0_reg__0_n_76\ : STD_LOGIC;
  signal \buff0_reg__0_n_77\ : STD_LOGIC;
  signal \buff0_reg__0_n_78\ : STD_LOGIC;
  signal \buff0_reg__0_n_79\ : STD_LOGIC;
  signal \buff0_reg__0_n_80\ : STD_LOGIC;
  signal \buff0_reg__0_n_81\ : STD_LOGIC;
  signal \buff0_reg__0_n_82\ : STD_LOGIC;
  signal \buff0_reg__0_n_83\ : STD_LOGIC;
  signal \buff0_reg__0_n_84\ : STD_LOGIC;
  signal \buff0_reg__0_n_85\ : STD_LOGIC;
  signal \buff0_reg__0_n_86\ : STD_LOGIC;
  signal \buff0_reg__0_n_87\ : STD_LOGIC;
  signal \buff0_reg__0_n_88\ : STD_LOGIC;
  signal \buff0_reg__0_n_89\ : STD_LOGIC;
  signal \buff0_reg__0_n_90\ : STD_LOGIC;
  signal \buff0_reg__0_n_91\ : STD_LOGIC;
  signal \buff0_reg__0_n_92\ : STD_LOGIC;
  signal \buff0_reg__0_n_93\ : STD_LOGIC;
  signal \buff0_reg__0_n_94\ : STD_LOGIC;
  signal \buff0_reg__0_n_95\ : STD_LOGIC;
  signal \buff0_reg__0_n_96\ : STD_LOGIC;
  signal \buff0_reg__0_n_97\ : STD_LOGIC;
  signal \buff0_reg__0_n_98\ : STD_LOGIC;
  signal \buff0_reg__0_n_99\ : STD_LOGIC;
  signal \buff0_reg_n_0_[0]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[10]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[11]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[12]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[13]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[14]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[15]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[16]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[1]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[2]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[3]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[4]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[5]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[6]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[7]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[8]\ : STD_LOGIC;
  signal \buff0_reg_n_0_[9]\ : STD_LOGIC;
  signal buff0_reg_n_100 : STD_LOGIC;
  signal buff0_reg_n_101 : STD_LOGIC;
  signal buff0_reg_n_102 : STD_LOGIC;
  signal buff0_reg_n_103 : STD_LOGIC;
  signal buff0_reg_n_104 : STD_LOGIC;
  signal buff0_reg_n_105 : STD_LOGIC;
  signal buff0_reg_n_58 : STD_LOGIC;
  signal buff0_reg_n_59 : STD_LOGIC;
  signal buff0_reg_n_60 : STD_LOGIC;
  signal buff0_reg_n_61 : STD_LOGIC;
  signal buff0_reg_n_62 : STD_LOGIC;
  signal buff0_reg_n_63 : STD_LOGIC;
  signal buff0_reg_n_64 : STD_LOGIC;
  signal buff0_reg_n_65 : STD_LOGIC;
  signal buff0_reg_n_66 : STD_LOGIC;
  signal buff0_reg_n_67 : STD_LOGIC;
  signal buff0_reg_n_68 : STD_LOGIC;
  signal buff0_reg_n_69 : STD_LOGIC;
  signal buff0_reg_n_70 : STD_LOGIC;
  signal buff0_reg_n_71 : STD_LOGIC;
  signal buff0_reg_n_72 : STD_LOGIC;
  signal buff0_reg_n_73 : STD_LOGIC;
  signal buff0_reg_n_74 : STD_LOGIC;
  signal buff0_reg_n_75 : STD_LOGIC;
  signal buff0_reg_n_76 : STD_LOGIC;
  signal buff0_reg_n_77 : STD_LOGIC;
  signal buff0_reg_n_78 : STD_LOGIC;
  signal buff0_reg_n_79 : STD_LOGIC;
  signal buff0_reg_n_80 : STD_LOGIC;
  signal buff0_reg_n_81 : STD_LOGIC;
  signal buff0_reg_n_82 : STD_LOGIC;
  signal buff0_reg_n_83 : STD_LOGIC;
  signal buff0_reg_n_84 : STD_LOGIC;
  signal buff0_reg_n_85 : STD_LOGIC;
  signal buff0_reg_n_86 : STD_LOGIC;
  signal buff0_reg_n_87 : STD_LOGIC;
  signal buff0_reg_n_88 : STD_LOGIC;
  signal buff0_reg_n_89 : STD_LOGIC;
  signal buff0_reg_n_90 : STD_LOGIC;
  signal buff0_reg_n_91 : STD_LOGIC;
  signal buff0_reg_n_92 : STD_LOGIC;
  signal buff0_reg_n_93 : STD_LOGIC;
  signal buff0_reg_n_94 : STD_LOGIC;
  signal buff0_reg_n_95 : STD_LOGIC;
  signal buff0_reg_n_96 : STD_LOGIC;
  signal buff0_reg_n_97 : STD_LOGIC;
  signal buff0_reg_n_98 : STD_LOGIC;
  signal buff0_reg_n_99 : STD_LOGIC;
  signal \mul_ln39_reg_304[19]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[19]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[19]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[23]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[23]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[23]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[23]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[27]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[27]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[27]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[27]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[31]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[31]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[31]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[31]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[35]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[35]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[35]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[35]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[39]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[39]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[39]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[39]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[41]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[41]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[41]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[41]_i_6_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[44]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[44]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[44]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[44]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[48]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[48]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[48]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[48]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[52]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[52]_i_3_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[52]_i_4_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304[52]_i_5_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[19]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[19]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[19]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[19]_i_1_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[23]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[23]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[23]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[23]_i_1_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[27]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[27]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[27]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[27]_i_1_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[31]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[31]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[31]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[31]_i_1_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[35]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[35]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[35]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[35]_i_1_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[39]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[39]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[39]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[39]_i_1_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[41]_i_2_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[41]_i_2_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[41]_i_2_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[41]_i_2_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[44]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[44]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[44]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[44]_i_1_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[48]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[48]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[48]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[48]_i_1_n_3\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[52]_i_1_n_0\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[52]_i_1_n_1\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[52]_i_1_n_2\ : STD_LOGIC;
  signal \mul_ln39_reg_304_reg[52]_i_1_n_3\ : STD_LOGIC;
  signal \tmp_2_reg_309[15]_i_2_n_0\ : STD_LOGIC;
  signal \tmp_2_reg_309[15]_i_3_n_0\ : STD_LOGIC;
  signal \tmp_2_reg_309_reg[15]_i_1_n_3\ : STD_LOGIC;
  signal \tmp_product__0_n_100\ : STD_LOGIC;
  signal \tmp_product__0_n_101\ : STD_LOGIC;
  signal \tmp_product__0_n_102\ : STD_LOGIC;
  signal \tmp_product__0_n_103\ : STD_LOGIC;
  signal \tmp_product__0_n_104\ : STD_LOGIC;
  signal \tmp_product__0_n_105\ : STD_LOGIC;
  signal \tmp_product__0_n_106\ : STD_LOGIC;
  signal \tmp_product__0_n_107\ : STD_LOGIC;
  signal \tmp_product__0_n_108\ : STD_LOGIC;
  signal \tmp_product__0_n_109\ : STD_LOGIC;
  signal \tmp_product__0_n_110\ : STD_LOGIC;
  signal \tmp_product__0_n_111\ : STD_LOGIC;
  signal \tmp_product__0_n_112\ : STD_LOGIC;
  signal \tmp_product__0_n_113\ : STD_LOGIC;
  signal \tmp_product__0_n_114\ : STD_LOGIC;
  signal \tmp_product__0_n_115\ : STD_LOGIC;
  signal \tmp_product__0_n_116\ : STD_LOGIC;
  signal \tmp_product__0_n_117\ : STD_LOGIC;
  signal \tmp_product__0_n_118\ : STD_LOGIC;
  signal \tmp_product__0_n_119\ : STD_LOGIC;
  signal \tmp_product__0_n_120\ : STD_LOGIC;
  signal \tmp_product__0_n_121\ : STD_LOGIC;
  signal \tmp_product__0_n_122\ : STD_LOGIC;
  signal \tmp_product__0_n_123\ : STD_LOGIC;
  signal \tmp_product__0_n_124\ : STD_LOGIC;
  signal \tmp_product__0_n_125\ : STD_LOGIC;
  signal \tmp_product__0_n_126\ : STD_LOGIC;
  signal \tmp_product__0_n_127\ : STD_LOGIC;
  signal \tmp_product__0_n_128\ : STD_LOGIC;
  signal \tmp_product__0_n_129\ : STD_LOGIC;
  signal \tmp_product__0_n_130\ : STD_LOGIC;
  signal \tmp_product__0_n_131\ : STD_LOGIC;
  signal \tmp_product__0_n_132\ : STD_LOGIC;
  signal \tmp_product__0_n_133\ : STD_LOGIC;
  signal \tmp_product__0_n_134\ : STD_LOGIC;
  signal \tmp_product__0_n_135\ : STD_LOGIC;
  signal \tmp_product__0_n_136\ : STD_LOGIC;
  signal \tmp_product__0_n_137\ : STD_LOGIC;
  signal \tmp_product__0_n_138\ : STD_LOGIC;
  signal \tmp_product__0_n_139\ : STD_LOGIC;
  signal \tmp_product__0_n_140\ : STD_LOGIC;
  signal \tmp_product__0_n_141\ : STD_LOGIC;
  signal \tmp_product__0_n_142\ : STD_LOGIC;
  signal \tmp_product__0_n_143\ : STD_LOGIC;
  signal \tmp_product__0_n_144\ : STD_LOGIC;
  signal \tmp_product__0_n_145\ : STD_LOGIC;
  signal \tmp_product__0_n_146\ : STD_LOGIC;
  signal \tmp_product__0_n_147\ : STD_LOGIC;
  signal \tmp_product__0_n_148\ : STD_LOGIC;
  signal \tmp_product__0_n_149\ : STD_LOGIC;
  signal \tmp_product__0_n_150\ : STD_LOGIC;
  signal \tmp_product__0_n_151\ : STD_LOGIC;
  signal \tmp_product__0_n_152\ : STD_LOGIC;
  signal \tmp_product__0_n_153\ : STD_LOGIC;
  signal \tmp_product__0_n_24\ : STD_LOGIC;
  signal \tmp_product__0_n_25\ : STD_LOGIC;
  signal \tmp_product__0_n_26\ : STD_LOGIC;
  signal \tmp_product__0_n_27\ : STD_LOGIC;
  signal \tmp_product__0_n_28\ : STD_LOGIC;
  signal \tmp_product__0_n_29\ : STD_LOGIC;
  signal \tmp_product__0_n_30\ : STD_LOGIC;
  signal \tmp_product__0_n_31\ : STD_LOGIC;
  signal \tmp_product__0_n_32\ : STD_LOGIC;
  signal \tmp_product__0_n_33\ : STD_LOGIC;
  signal \tmp_product__0_n_34\ : STD_LOGIC;
  signal \tmp_product__0_n_35\ : STD_LOGIC;
  signal \tmp_product__0_n_36\ : STD_LOGIC;
  signal \tmp_product__0_n_37\ : STD_LOGIC;
  signal \tmp_product__0_n_38\ : STD_LOGIC;
  signal \tmp_product__0_n_39\ : STD_LOGIC;
  signal \tmp_product__0_n_40\ : STD_LOGIC;
  signal \tmp_product__0_n_41\ : STD_LOGIC;
  signal \tmp_product__0_n_42\ : STD_LOGIC;
  signal \tmp_product__0_n_43\ : STD_LOGIC;
  signal \tmp_product__0_n_44\ : STD_LOGIC;
  signal \tmp_product__0_n_45\ : STD_LOGIC;
  signal \tmp_product__0_n_46\ : STD_LOGIC;
  signal \tmp_product__0_n_47\ : STD_LOGIC;
  signal \tmp_product__0_n_48\ : STD_LOGIC;
  signal \tmp_product__0_n_49\ : STD_LOGIC;
  signal \tmp_product__0_n_50\ : STD_LOGIC;
  signal \tmp_product__0_n_51\ : STD_LOGIC;
  signal \tmp_product__0_n_52\ : STD_LOGIC;
  signal \tmp_product__0_n_53\ : STD_LOGIC;
  signal \tmp_product__0_n_58\ : STD_LOGIC;
  signal \tmp_product__0_n_59\ : STD_LOGIC;
  signal \tmp_product__0_n_60\ : STD_LOGIC;
  signal \tmp_product__0_n_61\ : STD_LOGIC;
  signal \tmp_product__0_n_62\ : STD_LOGIC;
  signal \tmp_product__0_n_63\ : STD_LOGIC;
  signal \tmp_product__0_n_64\ : STD_LOGIC;
  signal \tmp_product__0_n_65\ : STD_LOGIC;
  signal \tmp_product__0_n_66\ : STD_LOGIC;
  signal \tmp_product__0_n_67\ : STD_LOGIC;
  signal \tmp_product__0_n_68\ : STD_LOGIC;
  signal \tmp_product__0_n_69\ : STD_LOGIC;
  signal \tmp_product__0_n_70\ : STD_LOGIC;
  signal \tmp_product__0_n_71\ : STD_LOGIC;
  signal \tmp_product__0_n_72\ : STD_LOGIC;
  signal \tmp_product__0_n_73\ : STD_LOGIC;
  signal \tmp_product__0_n_74\ : STD_LOGIC;
  signal \tmp_product__0_n_75\ : STD_LOGIC;
  signal \tmp_product__0_n_76\ : STD_LOGIC;
  signal \tmp_product__0_n_77\ : STD_LOGIC;
  signal \tmp_product__0_n_78\ : STD_LOGIC;
  signal \tmp_product__0_n_79\ : STD_LOGIC;
  signal \tmp_product__0_n_80\ : STD_LOGIC;
  signal \tmp_product__0_n_81\ : STD_LOGIC;
  signal \tmp_product__0_n_82\ : STD_LOGIC;
  signal \tmp_product__0_n_83\ : STD_LOGIC;
  signal \tmp_product__0_n_84\ : STD_LOGIC;
  signal \tmp_product__0_n_85\ : STD_LOGIC;
  signal \tmp_product__0_n_86\ : STD_LOGIC;
  signal \tmp_product__0_n_87\ : STD_LOGIC;
  signal \tmp_product__0_n_88\ : STD_LOGIC;
  signal \tmp_product__0_n_89\ : STD_LOGIC;
  signal \tmp_product__0_n_90\ : STD_LOGIC;
  signal \tmp_product__0_n_91\ : STD_LOGIC;
  signal \tmp_product__0_n_92\ : STD_LOGIC;
  signal \tmp_product__0_n_93\ : STD_LOGIC;
  signal \tmp_product__0_n_94\ : STD_LOGIC;
  signal \tmp_product__0_n_95\ : STD_LOGIC;
  signal \tmp_product__0_n_96\ : STD_LOGIC;
  signal \tmp_product__0_n_97\ : STD_LOGIC;
  signal \tmp_product__0_n_98\ : STD_LOGIC;
  signal \tmp_product__0_n_99\ : STD_LOGIC;
  signal tmp_product_n_100 : STD_LOGIC;
  signal tmp_product_n_101 : STD_LOGIC;
  signal tmp_product_n_102 : STD_LOGIC;
  signal tmp_product_n_103 : STD_LOGIC;
  signal tmp_product_n_104 : STD_LOGIC;
  signal tmp_product_n_105 : STD_LOGIC;
  signal tmp_product_n_106 : STD_LOGIC;
  signal tmp_product_n_107 : STD_LOGIC;
  signal tmp_product_n_108 : STD_LOGIC;
  signal tmp_product_n_109 : STD_LOGIC;
  signal tmp_product_n_110 : STD_LOGIC;
  signal tmp_product_n_111 : STD_LOGIC;
  signal tmp_product_n_112 : STD_LOGIC;
  signal tmp_product_n_113 : STD_LOGIC;
  signal tmp_product_n_114 : STD_LOGIC;
  signal tmp_product_n_115 : STD_LOGIC;
  signal tmp_product_n_116 : STD_LOGIC;
  signal tmp_product_n_117 : STD_LOGIC;
  signal tmp_product_n_118 : STD_LOGIC;
  signal tmp_product_n_119 : STD_LOGIC;
  signal tmp_product_n_120 : STD_LOGIC;
  signal tmp_product_n_121 : STD_LOGIC;
  signal tmp_product_n_122 : STD_LOGIC;
  signal tmp_product_n_123 : STD_LOGIC;
  signal tmp_product_n_124 : STD_LOGIC;
  signal tmp_product_n_125 : STD_LOGIC;
  signal tmp_product_n_126 : STD_LOGIC;
  signal tmp_product_n_127 : STD_LOGIC;
  signal tmp_product_n_128 : STD_LOGIC;
  signal tmp_product_n_129 : STD_LOGIC;
  signal tmp_product_n_130 : STD_LOGIC;
  signal tmp_product_n_131 : STD_LOGIC;
  signal tmp_product_n_132 : STD_LOGIC;
  signal tmp_product_n_133 : STD_LOGIC;
  signal tmp_product_n_134 : STD_LOGIC;
  signal tmp_product_n_135 : STD_LOGIC;
  signal tmp_product_n_136 : STD_LOGIC;
  signal tmp_product_n_137 : STD_LOGIC;
  signal tmp_product_n_138 : STD_LOGIC;
  signal tmp_product_n_139 : STD_LOGIC;
  signal tmp_product_n_140 : STD_LOGIC;
  signal tmp_product_n_141 : STD_LOGIC;
  signal tmp_product_n_142 : STD_LOGIC;
  signal tmp_product_n_143 : STD_LOGIC;
  signal tmp_product_n_144 : STD_LOGIC;
  signal tmp_product_n_145 : STD_LOGIC;
  signal tmp_product_n_146 : STD_LOGIC;
  signal tmp_product_n_147 : STD_LOGIC;
  signal tmp_product_n_148 : STD_LOGIC;
  signal tmp_product_n_149 : STD_LOGIC;
  signal tmp_product_n_150 : STD_LOGIC;
  signal tmp_product_n_151 : STD_LOGIC;
  signal tmp_product_n_152 : STD_LOGIC;
  signal tmp_product_n_153 : STD_LOGIC;
  signal tmp_product_n_58 : STD_LOGIC;
  signal tmp_product_n_59 : STD_LOGIC;
  signal tmp_product_n_60 : STD_LOGIC;
  signal tmp_product_n_61 : STD_LOGIC;
  signal tmp_product_n_62 : STD_LOGIC;
  signal tmp_product_n_63 : STD_LOGIC;
  signal tmp_product_n_64 : STD_LOGIC;
  signal tmp_product_n_65 : STD_LOGIC;
  signal tmp_product_n_66 : STD_LOGIC;
  signal tmp_product_n_67 : STD_LOGIC;
  signal tmp_product_n_68 : STD_LOGIC;
  signal tmp_product_n_69 : STD_LOGIC;
  signal tmp_product_n_70 : STD_LOGIC;
  signal tmp_product_n_71 : STD_LOGIC;
  signal tmp_product_n_72 : STD_LOGIC;
  signal tmp_product_n_73 : STD_LOGIC;
  signal tmp_product_n_74 : STD_LOGIC;
  signal tmp_product_n_75 : STD_LOGIC;
  signal tmp_product_n_76 : STD_LOGIC;
  signal tmp_product_n_77 : STD_LOGIC;
  signal tmp_product_n_78 : STD_LOGIC;
  signal tmp_product_n_79 : STD_LOGIC;
  signal tmp_product_n_80 : STD_LOGIC;
  signal tmp_product_n_81 : STD_LOGIC;
  signal tmp_product_n_82 : STD_LOGIC;
  signal tmp_product_n_83 : STD_LOGIC;
  signal tmp_product_n_84 : STD_LOGIC;
  signal tmp_product_n_85 : STD_LOGIC;
  signal tmp_product_n_86 : STD_LOGIC;
  signal tmp_product_n_87 : STD_LOGIC;
  signal tmp_product_n_88 : STD_LOGIC;
  signal tmp_product_n_89 : STD_LOGIC;
  signal tmp_product_n_90 : STD_LOGIC;
  signal tmp_product_n_91 : STD_LOGIC;
  signal tmp_product_n_92 : STD_LOGIC;
  signal tmp_product_n_93 : STD_LOGIC;
  signal tmp_product_n_94 : STD_LOGIC;
  signal tmp_product_n_95 : STD_LOGIC;
  signal tmp_product_n_96 : STD_LOGIC;
  signal tmp_product_n_97 : STD_LOGIC;
  signal tmp_product_n_98 : STD_LOGIC;
  signal tmp_product_n_99 : STD_LOGIC;
  signal NLW_buff0_reg_CARRYCASCOUT_UNCONNECTED : STD_LOGIC;
  signal NLW_buff0_reg_MULTSIGNOUT_UNCONNECTED : STD_LOGIC;
  signal NLW_buff0_reg_OVERFLOW_UNCONNECTED : STD_LOGIC;
  signal NLW_buff0_reg_PATTERNBDETECT_UNCONNECTED : STD_LOGIC;
  signal NLW_buff0_reg_PATTERNDETECT_UNCONNECTED : STD_LOGIC;
  signal NLW_buff0_reg_UNDERFLOW_UNCONNECTED : STD_LOGIC;
  signal NLW_buff0_reg_ACOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 29 downto 0 );
  signal NLW_buff0_reg_BCOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 17 downto 0 );
  signal NLW_buff0_reg_CARRYOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal NLW_buff0_reg_PCOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 47 downto 0 );
  signal \NLW_buff0_reg__0_CARRYCASCOUT_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_buff0_reg__0_MULTSIGNOUT_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_buff0_reg__0_OVERFLOW_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_buff0_reg__0_PATTERNBDETECT_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_buff0_reg__0_PATTERNDETECT_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_buff0_reg__0_UNDERFLOW_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_buff0_reg__0_ACOUT_UNCONNECTED\ : STD_LOGIC_VECTOR ( 29 downto 0 );
  signal \NLW_buff0_reg__0_BCOUT_UNCONNECTED\ : STD_LOGIC_VECTOR ( 17 downto 0 );
  signal \NLW_buff0_reg__0_CARRYOUT_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_buff0_reg__0_PCOUT_UNCONNECTED\ : STD_LOGIC_VECTOR ( 47 downto 0 );
  signal \NLW_tmp_2_reg_309_reg[15]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 1 );
  signal \NLW_tmp_2_reg_309_reg[15]_i_1_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal NLW_tmp_product_CARRYCASCOUT_UNCONNECTED : STD_LOGIC;
  signal NLW_tmp_product_MULTSIGNOUT_UNCONNECTED : STD_LOGIC;
  signal NLW_tmp_product_OVERFLOW_UNCONNECTED : STD_LOGIC;
  signal NLW_tmp_product_PATTERNBDETECT_UNCONNECTED : STD_LOGIC;
  signal NLW_tmp_product_PATTERNDETECT_UNCONNECTED : STD_LOGIC;
  signal NLW_tmp_product_UNDERFLOW_UNCONNECTED : STD_LOGIC;
  signal NLW_tmp_product_ACOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 29 downto 0 );
  signal NLW_tmp_product_BCOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 17 downto 0 );
  signal NLW_tmp_product_CARRYOUT_UNCONNECTED : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_tmp_product__0_CARRYCASCOUT_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_tmp_product__0_MULTSIGNOUT_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_tmp_product__0_OVERFLOW_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_tmp_product__0_PATTERNBDETECT_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_tmp_product__0_PATTERNDETECT_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_tmp_product__0_UNDERFLOW_UNCONNECTED\ : STD_LOGIC;
  signal \NLW_tmp_product__0_BCOUT_UNCONNECTED\ : STD_LOGIC_VECTOR ( 17 downto 0 );
  signal \NLW_tmp_product__0_CARRYOUT_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  attribute METHODOLOGY_DRC_VIOS : string;
  attribute METHODOLOGY_DRC_VIOS of buff0_reg : label is "{SYNTH-10 {cell *THIS*} {string 15x17 4}}";
  attribute METHODOLOGY_DRC_VIOS of \buff0_reg__0\ : label is "{SYNTH-10 {cell *THIS*} {string 18x17 4}}";
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[19]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[23]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[27]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[31]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[35]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[39]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[41]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[44]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[48]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \mul_ln39_reg_304_reg[52]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \tmp_2_reg_309_reg[15]_i_1\ : label is 35;
  attribute METHODOLOGY_DRC_VIOS of tmp_product : label is "{SYNTH-10 {cell *THIS*} {string 15x18 4}}";
  attribute METHODOLOGY_DRC_VIOS of \tmp_product__0\ : label is "{SYNTH-10 {cell *THIS*} {string 18x18 4}}";
begin
\add_ln32_reg_283[31]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(14),
      I1 => sum_reg(15),
      O => S(3)
    );
\add_ln32_reg_283[31]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(13),
      I1 => sum_reg(14),
      O => S(2)
    );
\add_ln32_reg_283[31]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(12),
      I1 => sum_reg(13),
      O => S(1)
    );
\add_ln32_reg_283[31]_i_6\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(11),
      I1 => sum_reg(12),
      O => S(0)
    );
buff0_reg: unisim.vcomponents.DSP48E1
    generic map(
      ACASCREG => 0,
      ADREG => 1,
      ALUMODEREG => 0,
      AREG => 0,
      AUTORESET_PATDET => "NO_RESET",
      A_INPUT => "DIRECT",
      BCASCREG => 1,
      BREG => 1,
      B_INPUT => "DIRECT",
      CARRYINREG => 0,
      CARRYINSELREG => 0,
      CREG => 1,
      DREG => 1,
      INMODEREG => 0,
      MASK => X"3FFFFFFFFFFF",
      MREG => 0,
      OPMODEREG => 0,
      PATTERN => X"000000000000",
      PREG => 1,
      SEL_MASK => "MASK",
      SEL_PATTERN => "PATTERN",
      USE_DPORT => false,
      USE_MULT => "MULTIPLY",
      USE_PATTERN_DETECT => "NO_PATDET",
      USE_SIMD => "ONE48"
    )
        port map (
      A(29 downto 0) => B"000000000000001010001111010111",
      ACIN(29 downto 0) => B"000000000000000000000000000000",
      ACOUT(29 downto 0) => NLW_buff0_reg_ACOUT_UNCONNECTED(29 downto 0),
      ALUMODE(3 downto 0) => B"0000",
      B(17) => add_ln32_fu_145_p2(31),
      B(16) => add_ln32_fu_145_p2(31),
      B(15) => add_ln32_fu_145_p2(31),
      B(14 downto 0) => add_ln32_fu_145_p2(31 downto 17),
      BCIN(17 downto 0) => B"000000000000000000",
      BCOUT(17 downto 0) => NLW_buff0_reg_BCOUT_UNCONNECTED(17 downto 0),
      C(47 downto 0) => B"111111111111111111111111111111111111111111111111",
      CARRYCASCIN => '0',
      CARRYCASCOUT => NLW_buff0_reg_CARRYCASCOUT_UNCONNECTED,
      CARRYIN => '0',
      CARRYINSEL(2 downto 0) => B"000",
      CARRYOUT(3 downto 0) => NLW_buff0_reg_CARRYOUT_UNCONNECTED(3 downto 0),
      CEA1 => '0',
      CEA2 => '0',
      CEAD => '0',
      CEALUMODE => '0',
      CEB1 => '0',
      CEB2 => add_ln32_reg_2830,
      CEC => '0',
      CECARRYIN => '0',
      CECTRL => '0',
      CED => '0',
      CEINMODE => '0',
      CEM => '0',
      CEP => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D(24 downto 0) => B"0000000000000000000000000",
      INMODE(4 downto 0) => B"00000",
      MULTSIGNIN => '0',
      MULTSIGNOUT => NLW_buff0_reg_MULTSIGNOUT_UNCONNECTED,
      OPMODE(6 downto 0) => B"1010101",
      OVERFLOW => NLW_buff0_reg_OVERFLOW_UNCONNECTED,
      P(47) => buff0_reg_n_58,
      P(46) => buff0_reg_n_59,
      P(45) => buff0_reg_n_60,
      P(44) => buff0_reg_n_61,
      P(43) => buff0_reg_n_62,
      P(42) => buff0_reg_n_63,
      P(41) => buff0_reg_n_64,
      P(40) => buff0_reg_n_65,
      P(39) => buff0_reg_n_66,
      P(38) => buff0_reg_n_67,
      P(37) => buff0_reg_n_68,
      P(36) => buff0_reg_n_69,
      P(35) => buff0_reg_n_70,
      P(34) => buff0_reg_n_71,
      P(33) => buff0_reg_n_72,
      P(32) => buff0_reg_n_73,
      P(31) => buff0_reg_n_74,
      P(30) => buff0_reg_n_75,
      P(29) => buff0_reg_n_76,
      P(28) => buff0_reg_n_77,
      P(27) => buff0_reg_n_78,
      P(26) => buff0_reg_n_79,
      P(25) => buff0_reg_n_80,
      P(24) => buff0_reg_n_81,
      P(23) => buff0_reg_n_82,
      P(22) => buff0_reg_n_83,
      P(21) => buff0_reg_n_84,
      P(20) => buff0_reg_n_85,
      P(19) => buff0_reg_n_86,
      P(18) => buff0_reg_n_87,
      P(17) => buff0_reg_n_88,
      P(16) => buff0_reg_n_89,
      P(15) => buff0_reg_n_90,
      P(14) => buff0_reg_n_91,
      P(13) => buff0_reg_n_92,
      P(12) => buff0_reg_n_93,
      P(11) => buff0_reg_n_94,
      P(10) => buff0_reg_n_95,
      P(9) => buff0_reg_n_96,
      P(8) => buff0_reg_n_97,
      P(7) => buff0_reg_n_98,
      P(6) => buff0_reg_n_99,
      P(5) => buff0_reg_n_100,
      P(4) => buff0_reg_n_101,
      P(3) => buff0_reg_n_102,
      P(2) => buff0_reg_n_103,
      P(1) => buff0_reg_n_104,
      P(0) => buff0_reg_n_105,
      PATTERNBDETECT => NLW_buff0_reg_PATTERNBDETECT_UNCONNECTED,
      PATTERNDETECT => NLW_buff0_reg_PATTERNDETECT_UNCONNECTED,
      PCIN(47) => tmp_product_n_106,
      PCIN(46) => tmp_product_n_107,
      PCIN(45) => tmp_product_n_108,
      PCIN(44) => tmp_product_n_109,
      PCIN(43) => tmp_product_n_110,
      PCIN(42) => tmp_product_n_111,
      PCIN(41) => tmp_product_n_112,
      PCIN(40) => tmp_product_n_113,
      PCIN(39) => tmp_product_n_114,
      PCIN(38) => tmp_product_n_115,
      PCIN(37) => tmp_product_n_116,
      PCIN(36) => tmp_product_n_117,
      PCIN(35) => tmp_product_n_118,
      PCIN(34) => tmp_product_n_119,
      PCIN(33) => tmp_product_n_120,
      PCIN(32) => tmp_product_n_121,
      PCIN(31) => tmp_product_n_122,
      PCIN(30) => tmp_product_n_123,
      PCIN(29) => tmp_product_n_124,
      PCIN(28) => tmp_product_n_125,
      PCIN(27) => tmp_product_n_126,
      PCIN(26) => tmp_product_n_127,
      PCIN(25) => tmp_product_n_128,
      PCIN(24) => tmp_product_n_129,
      PCIN(23) => tmp_product_n_130,
      PCIN(22) => tmp_product_n_131,
      PCIN(21) => tmp_product_n_132,
      PCIN(20) => tmp_product_n_133,
      PCIN(19) => tmp_product_n_134,
      PCIN(18) => tmp_product_n_135,
      PCIN(17) => tmp_product_n_136,
      PCIN(16) => tmp_product_n_137,
      PCIN(15) => tmp_product_n_138,
      PCIN(14) => tmp_product_n_139,
      PCIN(13) => tmp_product_n_140,
      PCIN(12) => tmp_product_n_141,
      PCIN(11) => tmp_product_n_142,
      PCIN(10) => tmp_product_n_143,
      PCIN(9) => tmp_product_n_144,
      PCIN(8) => tmp_product_n_145,
      PCIN(7) => tmp_product_n_146,
      PCIN(6) => tmp_product_n_147,
      PCIN(5) => tmp_product_n_148,
      PCIN(4) => tmp_product_n_149,
      PCIN(3) => tmp_product_n_150,
      PCIN(2) => tmp_product_n_151,
      PCIN(1) => tmp_product_n_152,
      PCIN(0) => tmp_product_n_153,
      PCOUT(47 downto 0) => NLW_buff0_reg_PCOUT_UNCONNECTED(47 downto 0),
      RSTA => '0',
      RSTALLCARRYIN => '0',
      RSTALUMODE => '0',
      RSTB => '0',
      RSTC => '0',
      RSTCTRL => '0',
      RSTD => '0',
      RSTINMODE => '0',
      RSTM => '0',
      RSTP => '0',
      UNDERFLOW => NLW_buff0_reg_UNDERFLOW_UNCONNECTED
    );
\buff0_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_105,
      Q => \buff0_reg_n_0_[0]\,
      R => '0'
    );
\buff0_reg[0]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_105\,
      Q => D(0),
      R => '0'
    );
\buff0_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_95,
      Q => \buff0_reg_n_0_[10]\,
      R => '0'
    );
\buff0_reg[10]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_95\,
      Q => D(10),
      R => '0'
    );
\buff0_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_94,
      Q => \buff0_reg_n_0_[11]\,
      R => '0'
    );
\buff0_reg[11]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_94\,
      Q => D(11),
      R => '0'
    );
\buff0_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_93,
      Q => \buff0_reg_n_0_[12]\,
      R => '0'
    );
\buff0_reg[12]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_93\,
      Q => D(12),
      R => '0'
    );
\buff0_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_92,
      Q => \buff0_reg_n_0_[13]\,
      R => '0'
    );
\buff0_reg[13]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_92\,
      Q => D(13),
      R => '0'
    );
\buff0_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_91,
      Q => \buff0_reg_n_0_[14]\,
      R => '0'
    );
\buff0_reg[14]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_91\,
      Q => D(14),
      R => '0'
    );
\buff0_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_90,
      Q => \buff0_reg_n_0_[15]\,
      R => '0'
    );
\buff0_reg[15]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_90\,
      Q => D(15),
      R => '0'
    );
\buff0_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_89,
      Q => \buff0_reg_n_0_[16]\,
      R => '0'
    );
\buff0_reg[16]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_89\,
      Q => \buff0_reg[16]__0_n_0\,
      R => '0'
    );
\buff0_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_104,
      Q => \buff0_reg_n_0_[1]\,
      R => '0'
    );
\buff0_reg[1]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_104\,
      Q => D(1),
      R => '0'
    );
\buff0_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_103,
      Q => \buff0_reg_n_0_[2]\,
      R => '0'
    );
\buff0_reg[2]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_103\,
      Q => D(2),
      R => '0'
    );
\buff0_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_102,
      Q => \buff0_reg_n_0_[3]\,
      R => '0'
    );
\buff0_reg[3]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_102\,
      Q => D(3),
      R => '0'
    );
\buff0_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_101,
      Q => \buff0_reg_n_0_[4]\,
      R => '0'
    );
\buff0_reg[4]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_101\,
      Q => D(4),
      R => '0'
    );
\buff0_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_100,
      Q => \buff0_reg_n_0_[5]\,
      R => '0'
    );
\buff0_reg[5]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_100\,
      Q => D(5),
      R => '0'
    );
\buff0_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_99,
      Q => \buff0_reg_n_0_[6]\,
      R => '0'
    );
\buff0_reg[6]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_99\,
      Q => D(6),
      R => '0'
    );
\buff0_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_98,
      Q => \buff0_reg_n_0_[7]\,
      R => '0'
    );
\buff0_reg[7]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_98\,
      Q => D(7),
      R => '0'
    );
\buff0_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_97,
      Q => \buff0_reg_n_0_[8]\,
      R => '0'
    );
\buff0_reg[8]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_97\,
      Q => D(8),
      R => '0'
    );
\buff0_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_product_n_96,
      Q => \buff0_reg_n_0_[9]\,
      R => '0'
    );
\buff0_reg[9]__0\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \tmp_product__0_n_96\,
      Q => D(9),
      R => '0'
    );
\buff0_reg__0\: unisim.vcomponents.DSP48E1
    generic map(
      ACASCREG => 0,
      ADREG => 1,
      ALUMODEREG => 0,
      AREG => 0,
      AUTORESET_PATDET => "NO_RESET",
      A_INPUT => "CASCADE",
      BCASCREG => 0,
      BREG => 0,
      B_INPUT => "DIRECT",
      CARRYINREG => 0,
      CARRYINSELREG => 0,
      CREG => 1,
      DREG => 1,
      INMODEREG => 0,
      MASK => X"3FFFFFFFFFFF",
      MREG => 0,
      OPMODEREG => 0,
      PATTERN => X"000000000000",
      PREG => 1,
      SEL_MASK => "MASK",
      SEL_PATTERN => "PATTERN",
      USE_DPORT => false,
      USE_MULT => "MULTIPLY",
      USE_PATTERN_DETECT => "NO_PATDET",
      USE_SIMD => "ONE48"
    )
        port map (
      A(29 downto 0) => B"000000000000000000000000000000",
      ACIN(29) => \tmp_product__0_n_24\,
      ACIN(28) => \tmp_product__0_n_25\,
      ACIN(27) => \tmp_product__0_n_26\,
      ACIN(26) => \tmp_product__0_n_27\,
      ACIN(25) => \tmp_product__0_n_28\,
      ACIN(24) => \tmp_product__0_n_29\,
      ACIN(23) => \tmp_product__0_n_30\,
      ACIN(22) => \tmp_product__0_n_31\,
      ACIN(21) => \tmp_product__0_n_32\,
      ACIN(20) => \tmp_product__0_n_33\,
      ACIN(19) => \tmp_product__0_n_34\,
      ACIN(18) => \tmp_product__0_n_35\,
      ACIN(17) => \tmp_product__0_n_36\,
      ACIN(16) => \tmp_product__0_n_37\,
      ACIN(15) => \tmp_product__0_n_38\,
      ACIN(14) => \tmp_product__0_n_39\,
      ACIN(13) => \tmp_product__0_n_40\,
      ACIN(12) => \tmp_product__0_n_41\,
      ACIN(11) => \tmp_product__0_n_42\,
      ACIN(10) => \tmp_product__0_n_43\,
      ACIN(9) => \tmp_product__0_n_44\,
      ACIN(8) => \tmp_product__0_n_45\,
      ACIN(7) => \tmp_product__0_n_46\,
      ACIN(6) => \tmp_product__0_n_47\,
      ACIN(5) => \tmp_product__0_n_48\,
      ACIN(4) => \tmp_product__0_n_49\,
      ACIN(3) => \tmp_product__0_n_50\,
      ACIN(2) => \tmp_product__0_n_51\,
      ACIN(1) => \tmp_product__0_n_52\,
      ACIN(0) => \tmp_product__0_n_53\,
      ACOUT(29 downto 0) => \NLW_buff0_reg__0_ACOUT_UNCONNECTED\(29 downto 0),
      ALUMODE(3 downto 0) => B"0000",
      B(17 downto 0) => B"001010001111010111",
      BCIN(17 downto 0) => B"000000000000000000",
      BCOUT(17 downto 0) => \NLW_buff0_reg__0_BCOUT_UNCONNECTED\(17 downto 0),
      C(47 downto 0) => B"111111111111111111111111111111111111111111111111",
      CARRYCASCIN => '0',
      CARRYCASCOUT => \NLW_buff0_reg__0_CARRYCASCOUT_UNCONNECTED\,
      CARRYIN => '0',
      CARRYINSEL(2 downto 0) => B"000",
      CARRYOUT(3 downto 0) => \NLW_buff0_reg__0_CARRYOUT_UNCONNECTED\(3 downto 0),
      CEA1 => '0',
      CEA2 => '0',
      CEAD => '0',
      CEALUMODE => '0',
      CEB1 => '0',
      CEB2 => '0',
      CEC => '0',
      CECARRYIN => '0',
      CECTRL => '0',
      CED => '0',
      CEINMODE => '0',
      CEM => '0',
      CEP => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D(24 downto 0) => B"0000000000000000000000000",
      INMODE(4 downto 0) => B"00000",
      MULTSIGNIN => '0',
      MULTSIGNOUT => \NLW_buff0_reg__0_MULTSIGNOUT_UNCONNECTED\,
      OPMODE(6 downto 0) => B"1010101",
      OVERFLOW => \NLW_buff0_reg__0_OVERFLOW_UNCONNECTED\,
      P(47) => \buff0_reg__0_n_58\,
      P(46) => \buff0_reg__0_n_59\,
      P(45) => \buff0_reg__0_n_60\,
      P(44) => \buff0_reg__0_n_61\,
      P(43) => \buff0_reg__0_n_62\,
      P(42) => \buff0_reg__0_n_63\,
      P(41) => \buff0_reg__0_n_64\,
      P(40) => \buff0_reg__0_n_65\,
      P(39) => \buff0_reg__0_n_66\,
      P(38) => \buff0_reg__0_n_67\,
      P(37) => \buff0_reg__0_n_68\,
      P(36) => \buff0_reg__0_n_69\,
      P(35) => \buff0_reg__0_n_70\,
      P(34) => \buff0_reg__0_n_71\,
      P(33) => \buff0_reg__0_n_72\,
      P(32) => \buff0_reg__0_n_73\,
      P(31) => \buff0_reg__0_n_74\,
      P(30) => \buff0_reg__0_n_75\,
      P(29) => \buff0_reg__0_n_76\,
      P(28) => \buff0_reg__0_n_77\,
      P(27) => \buff0_reg__0_n_78\,
      P(26) => \buff0_reg__0_n_79\,
      P(25) => \buff0_reg__0_n_80\,
      P(24) => \buff0_reg__0_n_81\,
      P(23) => \buff0_reg__0_n_82\,
      P(22) => \buff0_reg__0_n_83\,
      P(21) => \buff0_reg__0_n_84\,
      P(20) => \buff0_reg__0_n_85\,
      P(19) => \buff0_reg__0_n_86\,
      P(18) => \buff0_reg__0_n_87\,
      P(17) => \buff0_reg__0_n_88\,
      P(16) => \buff0_reg__0_n_89\,
      P(15) => \buff0_reg__0_n_90\,
      P(14) => \buff0_reg__0_n_91\,
      P(13) => \buff0_reg__0_n_92\,
      P(12) => \buff0_reg__0_n_93\,
      P(11) => \buff0_reg__0_n_94\,
      P(10) => \buff0_reg__0_n_95\,
      P(9) => \buff0_reg__0_n_96\,
      P(8) => \buff0_reg__0_n_97\,
      P(7) => \buff0_reg__0_n_98\,
      P(6) => \buff0_reg__0_n_99\,
      P(5) => \buff0_reg__0_n_100\,
      P(4) => \buff0_reg__0_n_101\,
      P(3) => \buff0_reg__0_n_102\,
      P(2) => \buff0_reg__0_n_103\,
      P(1) => \buff0_reg__0_n_104\,
      P(0) => \buff0_reg__0_n_105\,
      PATTERNBDETECT => \NLW_buff0_reg__0_PATTERNBDETECT_UNCONNECTED\,
      PATTERNDETECT => \NLW_buff0_reg__0_PATTERNDETECT_UNCONNECTED\,
      PCIN(47) => \tmp_product__0_n_106\,
      PCIN(46) => \tmp_product__0_n_107\,
      PCIN(45) => \tmp_product__0_n_108\,
      PCIN(44) => \tmp_product__0_n_109\,
      PCIN(43) => \tmp_product__0_n_110\,
      PCIN(42) => \tmp_product__0_n_111\,
      PCIN(41) => \tmp_product__0_n_112\,
      PCIN(40) => \tmp_product__0_n_113\,
      PCIN(39) => \tmp_product__0_n_114\,
      PCIN(38) => \tmp_product__0_n_115\,
      PCIN(37) => \tmp_product__0_n_116\,
      PCIN(36) => \tmp_product__0_n_117\,
      PCIN(35) => \tmp_product__0_n_118\,
      PCIN(34) => \tmp_product__0_n_119\,
      PCIN(33) => \tmp_product__0_n_120\,
      PCIN(32) => \tmp_product__0_n_121\,
      PCIN(31) => \tmp_product__0_n_122\,
      PCIN(30) => \tmp_product__0_n_123\,
      PCIN(29) => \tmp_product__0_n_124\,
      PCIN(28) => \tmp_product__0_n_125\,
      PCIN(27) => \tmp_product__0_n_126\,
      PCIN(26) => \tmp_product__0_n_127\,
      PCIN(25) => \tmp_product__0_n_128\,
      PCIN(24) => \tmp_product__0_n_129\,
      PCIN(23) => \tmp_product__0_n_130\,
      PCIN(22) => \tmp_product__0_n_131\,
      PCIN(21) => \tmp_product__0_n_132\,
      PCIN(20) => \tmp_product__0_n_133\,
      PCIN(19) => \tmp_product__0_n_134\,
      PCIN(18) => \tmp_product__0_n_135\,
      PCIN(17) => \tmp_product__0_n_136\,
      PCIN(16) => \tmp_product__0_n_137\,
      PCIN(15) => \tmp_product__0_n_138\,
      PCIN(14) => \tmp_product__0_n_139\,
      PCIN(13) => \tmp_product__0_n_140\,
      PCIN(12) => \tmp_product__0_n_141\,
      PCIN(11) => \tmp_product__0_n_142\,
      PCIN(10) => \tmp_product__0_n_143\,
      PCIN(9) => \tmp_product__0_n_144\,
      PCIN(8) => \tmp_product__0_n_145\,
      PCIN(7) => \tmp_product__0_n_146\,
      PCIN(6) => \tmp_product__0_n_147\,
      PCIN(5) => \tmp_product__0_n_148\,
      PCIN(4) => \tmp_product__0_n_149\,
      PCIN(3) => \tmp_product__0_n_150\,
      PCIN(2) => \tmp_product__0_n_151\,
      PCIN(1) => \tmp_product__0_n_152\,
      PCIN(0) => \tmp_product__0_n_153\,
      PCOUT(47 downto 0) => \NLW_buff0_reg__0_PCOUT_UNCONNECTED\(47 downto 0),
      RSTA => '0',
      RSTALLCARRYIN => '0',
      RSTALUMODE => '0',
      RSTB => '0',
      RSTC => '0',
      RSTCTRL => '0',
      RSTD => '0',
      RSTINMODE => '0',
      RSTM => '0',
      RSTP => '0',
      UNDERFLOW => \NLW_buff0_reg__0_UNDERFLOW_UNCONNECTED\
    );
\mul_ln39_reg_304[19]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_103\,
      I1 => \buff0_reg_n_0_[2]\,
      O => \mul_ln39_reg_304[19]_i_2_n_0\
    );
\mul_ln39_reg_304[19]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_104\,
      I1 => \buff0_reg_n_0_[1]\,
      O => \mul_ln39_reg_304[19]_i_3_n_0\
    );
\mul_ln39_reg_304[19]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_105\,
      I1 => \buff0_reg_n_0_[0]\,
      O => \mul_ln39_reg_304[19]_i_4_n_0\
    );
\mul_ln39_reg_304[23]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_99\,
      I1 => \buff0_reg_n_0_[6]\,
      O => \mul_ln39_reg_304[23]_i_2_n_0\
    );
\mul_ln39_reg_304[23]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_100\,
      I1 => \buff0_reg_n_0_[5]\,
      O => \mul_ln39_reg_304[23]_i_3_n_0\
    );
\mul_ln39_reg_304[23]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_101\,
      I1 => \buff0_reg_n_0_[4]\,
      O => \mul_ln39_reg_304[23]_i_4_n_0\
    );
\mul_ln39_reg_304[23]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_102\,
      I1 => \buff0_reg_n_0_[3]\,
      O => \mul_ln39_reg_304[23]_i_5_n_0\
    );
\mul_ln39_reg_304[27]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_95\,
      I1 => \buff0_reg_n_0_[10]\,
      O => \mul_ln39_reg_304[27]_i_2_n_0\
    );
\mul_ln39_reg_304[27]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_96\,
      I1 => \buff0_reg_n_0_[9]\,
      O => \mul_ln39_reg_304[27]_i_3_n_0\
    );
\mul_ln39_reg_304[27]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_97\,
      I1 => \buff0_reg_n_0_[8]\,
      O => \mul_ln39_reg_304[27]_i_4_n_0\
    );
\mul_ln39_reg_304[27]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_98\,
      I1 => \buff0_reg_n_0_[7]\,
      O => \mul_ln39_reg_304[27]_i_5_n_0\
    );
\mul_ln39_reg_304[31]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_91\,
      I1 => \buff0_reg_n_0_[14]\,
      O => \mul_ln39_reg_304[31]_i_2_n_0\
    );
\mul_ln39_reg_304[31]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_92\,
      I1 => \buff0_reg_n_0_[13]\,
      O => \mul_ln39_reg_304[31]_i_3_n_0\
    );
\mul_ln39_reg_304[31]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_93\,
      I1 => \buff0_reg_n_0_[12]\,
      O => \mul_ln39_reg_304[31]_i_4_n_0\
    );
\mul_ln39_reg_304[31]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_94\,
      I1 => \buff0_reg_n_0_[11]\,
      O => \mul_ln39_reg_304[31]_i_5_n_0\
    );
\mul_ln39_reg_304[35]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_87\,
      I1 => buff0_reg_n_104,
      O => \mul_ln39_reg_304[35]_i_2_n_0\
    );
\mul_ln39_reg_304[35]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_88\,
      I1 => buff0_reg_n_105,
      O => \mul_ln39_reg_304[35]_i_3_n_0\
    );
\mul_ln39_reg_304[35]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_89\,
      I1 => \buff0_reg_n_0_[16]\,
      O => \mul_ln39_reg_304[35]_i_4_n_0\
    );
\mul_ln39_reg_304[35]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_90\,
      I1 => \buff0_reg_n_0_[15]\,
      O => \mul_ln39_reg_304[35]_i_5_n_0\
    );
\mul_ln39_reg_304[39]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_83\,
      I1 => buff0_reg_n_100,
      O => \mul_ln39_reg_304[39]_i_2_n_0\
    );
\mul_ln39_reg_304[39]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_84\,
      I1 => buff0_reg_n_101,
      O => \mul_ln39_reg_304[39]_i_3_n_0\
    );
\mul_ln39_reg_304[39]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_85\,
      I1 => buff0_reg_n_102,
      O => \mul_ln39_reg_304[39]_i_4_n_0\
    );
\mul_ln39_reg_304[39]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_86\,
      I1 => buff0_reg_n_103,
      O => \mul_ln39_reg_304[39]_i_5_n_0\
    );
\mul_ln39_reg_304[41]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_79\,
      I1 => buff0_reg_n_96,
      O => \mul_ln39_reg_304[41]_i_3_n_0\
    );
\mul_ln39_reg_304[41]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_80\,
      I1 => buff0_reg_n_97,
      O => \mul_ln39_reg_304[41]_i_4_n_0\
    );
\mul_ln39_reg_304[41]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_81\,
      I1 => buff0_reg_n_98,
      O => \mul_ln39_reg_304[41]_i_5_n_0\
    );
\mul_ln39_reg_304[41]_i_6\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_82\,
      I1 => buff0_reg_n_99,
      O => \mul_ln39_reg_304[41]_i_6_n_0\
    );
\mul_ln39_reg_304[44]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_75\,
      I1 => buff0_reg_n_92,
      O => \mul_ln39_reg_304[44]_i_2_n_0\
    );
\mul_ln39_reg_304[44]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_76\,
      I1 => buff0_reg_n_93,
      O => \mul_ln39_reg_304[44]_i_3_n_0\
    );
\mul_ln39_reg_304[44]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_77\,
      I1 => buff0_reg_n_94,
      O => \mul_ln39_reg_304[44]_i_4_n_0\
    );
\mul_ln39_reg_304[44]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_78\,
      I1 => buff0_reg_n_95,
      O => \mul_ln39_reg_304[44]_i_5_n_0\
    );
\mul_ln39_reg_304[48]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_71\,
      I1 => buff0_reg_n_88,
      O => \mul_ln39_reg_304[48]_i_2_n_0\
    );
\mul_ln39_reg_304[48]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_72\,
      I1 => buff0_reg_n_89,
      O => \mul_ln39_reg_304[48]_i_3_n_0\
    );
\mul_ln39_reg_304[48]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_73\,
      I1 => buff0_reg_n_90,
      O => \mul_ln39_reg_304[48]_i_4_n_0\
    );
\mul_ln39_reg_304[48]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_74\,
      I1 => buff0_reg_n_91,
      O => \mul_ln39_reg_304[48]_i_5_n_0\
    );
\mul_ln39_reg_304[52]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_67\,
      I1 => buff0_reg_n_84,
      O => \mul_ln39_reg_304[52]_i_2_n_0\
    );
\mul_ln39_reg_304[52]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_68\,
      I1 => buff0_reg_n_85,
      O => \mul_ln39_reg_304[52]_i_3_n_0\
    );
\mul_ln39_reg_304[52]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_69\,
      I1 => buff0_reg_n_86,
      O => \mul_ln39_reg_304[52]_i_4_n_0\
    );
\mul_ln39_reg_304[52]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_70\,
      I1 => buff0_reg_n_87,
      O => \mul_ln39_reg_304[52]_i_5_n_0\
    );
\mul_ln39_reg_304_reg[19]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \mul_ln39_reg_304_reg[19]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[19]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[19]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[19]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_103\,
      DI(2) => \buff0_reg__0_n_104\,
      DI(1) => \buff0_reg__0_n_105\,
      DI(0) => '0',
      O(3 downto 0) => D(19 downto 16),
      S(3) => \mul_ln39_reg_304[19]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[19]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[19]_i_4_n_0\,
      S(0) => \buff0_reg[16]__0_n_0\
    );
\mul_ln39_reg_304_reg[23]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[19]_i_1_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[23]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[23]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[23]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[23]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_99\,
      DI(2) => \buff0_reg__0_n_100\,
      DI(1) => \buff0_reg__0_n_101\,
      DI(0) => \buff0_reg__0_n_102\,
      O(3 downto 0) => D(23 downto 20),
      S(3) => \mul_ln39_reg_304[23]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[23]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[23]_i_4_n_0\,
      S(0) => \mul_ln39_reg_304[23]_i_5_n_0\
    );
\mul_ln39_reg_304_reg[27]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[23]_i_1_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[27]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[27]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[27]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[27]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_95\,
      DI(2) => \buff0_reg__0_n_96\,
      DI(1) => \buff0_reg__0_n_97\,
      DI(0) => \buff0_reg__0_n_98\,
      O(3 downto 0) => D(27 downto 24),
      S(3) => \mul_ln39_reg_304[27]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[27]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[27]_i_4_n_0\,
      S(0) => \mul_ln39_reg_304[27]_i_5_n_0\
    );
\mul_ln39_reg_304_reg[31]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[27]_i_1_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[31]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[31]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[31]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[31]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_91\,
      DI(2) => \buff0_reg__0_n_92\,
      DI(1) => \buff0_reg__0_n_93\,
      DI(0) => \buff0_reg__0_n_94\,
      O(3 downto 0) => D(31 downto 28),
      S(3) => \mul_ln39_reg_304[31]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[31]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[31]_i_4_n_0\,
      S(0) => \mul_ln39_reg_304[31]_i_5_n_0\
    );
\mul_ln39_reg_304_reg[35]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[31]_i_1_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[35]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[35]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[35]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[35]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_87\,
      DI(2) => \buff0_reg__0_n_88\,
      DI(1) => \buff0_reg__0_n_89\,
      DI(0) => \buff0_reg__0_n_90\,
      O(3 downto 0) => D(35 downto 32),
      S(3) => \mul_ln39_reg_304[35]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[35]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[35]_i_4_n_0\,
      S(0) => \mul_ln39_reg_304[35]_i_5_n_0\
    );
\mul_ln39_reg_304_reg[39]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[35]_i_1_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[39]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[39]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[39]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[39]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_83\,
      DI(2) => \buff0_reg__0_n_84\,
      DI(1) => \buff0_reg__0_n_85\,
      DI(0) => \buff0_reg__0_n_86\,
      O(3 downto 0) => D(39 downto 36),
      S(3) => \mul_ln39_reg_304[39]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[39]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[39]_i_4_n_0\,
      S(0) => \mul_ln39_reg_304[39]_i_5_n_0\
    );
\mul_ln39_reg_304_reg[41]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[39]_i_1_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[41]_i_2_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[41]_i_2_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[41]_i_2_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[41]_i_2_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_79\,
      DI(2) => \buff0_reg__0_n_80\,
      DI(1) => \buff0_reg__0_n_81\,
      DI(0) => \buff0_reg__0_n_82\,
      O(3 downto 2) => \buff0_reg__0_0\(1 downto 0),
      O(1 downto 0) => D(41 downto 40),
      S(3) => \mul_ln39_reg_304[41]_i_3_n_0\,
      S(2) => \mul_ln39_reg_304[41]_i_4_n_0\,
      S(1) => \mul_ln39_reg_304[41]_i_5_n_0\,
      S(0) => \mul_ln39_reg_304[41]_i_6_n_0\
    );
\mul_ln39_reg_304_reg[44]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[41]_i_2_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[44]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[44]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[44]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[44]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_75\,
      DI(2) => \buff0_reg__0_n_76\,
      DI(1) => \buff0_reg__0_n_77\,
      DI(0) => \buff0_reg__0_n_78\,
      O(3 downto 0) => \buff0_reg__0_0\(5 downto 2),
      S(3) => \mul_ln39_reg_304[44]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[44]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[44]_i_4_n_0\,
      S(0) => \mul_ln39_reg_304[44]_i_5_n_0\
    );
\mul_ln39_reg_304_reg[48]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[44]_i_1_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[48]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[48]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[48]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[48]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_71\,
      DI(2) => \buff0_reg__0_n_72\,
      DI(1) => \buff0_reg__0_n_73\,
      DI(0) => \buff0_reg__0_n_74\,
      O(3 downto 0) => \buff0_reg__0_0\(9 downto 6),
      S(3) => \mul_ln39_reg_304[48]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[48]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[48]_i_4_n_0\,
      S(0) => \mul_ln39_reg_304[48]_i_5_n_0\
    );
\mul_ln39_reg_304_reg[52]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[48]_i_1_n_0\,
      CO(3) => \mul_ln39_reg_304_reg[52]_i_1_n_0\,
      CO(2) => \mul_ln39_reg_304_reg[52]_i_1_n_1\,
      CO(1) => \mul_ln39_reg_304_reg[52]_i_1_n_2\,
      CO(0) => \mul_ln39_reg_304_reg[52]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \buff0_reg__0_n_67\,
      DI(2) => \buff0_reg__0_n_68\,
      DI(1) => \buff0_reg__0_n_69\,
      DI(0) => \buff0_reg__0_n_70\,
      O(3 downto 0) => \buff0_reg__0_0\(13 downto 10),
      S(3) => \mul_ln39_reg_304[52]_i_2_n_0\,
      S(2) => \mul_ln39_reg_304[52]_i_3_n_0\,
      S(1) => \mul_ln39_reg_304[52]_i_4_n_0\,
      S(0) => \mul_ln39_reg_304[52]_i_5_n_0\
    );
\tmp_2_reg_309[15]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_65\,
      I1 => buff0_reg_n_82,
      O => \tmp_2_reg_309[15]_i_2_n_0\
    );
\tmp_2_reg_309[15]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"6"
    )
        port map (
      I0 => \buff0_reg__0_n_66\,
      I1 => buff0_reg_n_83,
      O => \tmp_2_reg_309[15]_i_3_n_0\
    );
\tmp_2_reg_309_reg[15]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \mul_ln39_reg_304_reg[52]_i_1_n_0\,
      CO(3 downto 1) => \NLW_tmp_2_reg_309_reg[15]_i_1_CO_UNCONNECTED\(3 downto 1),
      CO(0) => \tmp_2_reg_309_reg[15]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 1) => B"000",
      DI(0) => \buff0_reg__0_n_66\,
      O(3 downto 2) => \NLW_tmp_2_reg_309_reg[15]_i_1_O_UNCONNECTED\(3 downto 2),
      O(1 downto 0) => \buff0_reg__0_0\(15 downto 14),
      S(3 downto 2) => B"00",
      S(1) => \tmp_2_reg_309[15]_i_2_n_0\,
      S(0) => \tmp_2_reg_309[15]_i_3_n_0\
    );
tmp_product: unisim.vcomponents.DSP48E1
    generic map(
      ACASCREG => 1,
      ADREG => 1,
      ALUMODEREG => 0,
      AREG => 1,
      AUTORESET_PATDET => "NO_RESET",
      A_INPUT => "DIRECT",
      BCASCREG => 0,
      BREG => 0,
      B_INPUT => "DIRECT",
      CARRYINREG => 0,
      CARRYINSELREG => 0,
      CREG => 1,
      DREG => 1,
      INMODEREG => 0,
      MASK => X"3FFFFFFFFFFF",
      MREG => 0,
      OPMODEREG => 0,
      PATTERN => X"000000000000",
      PREG => 0,
      SEL_MASK => "MASK",
      SEL_PATTERN => "PATTERN",
      USE_DPORT => false,
      USE_MULT => "MULTIPLY",
      USE_PATTERN_DETECT => "NO_PATDET",
      USE_SIMD => "ONE48"
    )
        port map (
      A(29) => add_ln32_fu_145_p2(31),
      A(28) => add_ln32_fu_145_p2(31),
      A(27) => add_ln32_fu_145_p2(31),
      A(26) => add_ln32_fu_145_p2(31),
      A(25) => add_ln32_fu_145_p2(31),
      A(24) => add_ln32_fu_145_p2(31),
      A(23) => add_ln32_fu_145_p2(31),
      A(22) => add_ln32_fu_145_p2(31),
      A(21) => add_ln32_fu_145_p2(31),
      A(20) => add_ln32_fu_145_p2(31),
      A(19) => add_ln32_fu_145_p2(31),
      A(18) => add_ln32_fu_145_p2(31),
      A(17) => add_ln32_fu_145_p2(31),
      A(16) => add_ln32_fu_145_p2(31),
      A(15) => add_ln32_fu_145_p2(31),
      A(14 downto 0) => add_ln32_fu_145_p2(31 downto 17),
      ACIN(29 downto 0) => B"000000000000000000000000000000",
      ACOUT(29 downto 0) => NLW_tmp_product_ACOUT_UNCONNECTED(29 downto 0),
      ALUMODE(3 downto 0) => B"0000",
      B(17 downto 0) => B"000001010001111011",
      BCIN(17 downto 0) => B"000000000000000000",
      BCOUT(17 downto 0) => NLW_tmp_product_BCOUT_UNCONNECTED(17 downto 0),
      C(47 downto 0) => B"111111111111111111111111111111111111111111111111",
      CARRYCASCIN => '0',
      CARRYCASCOUT => NLW_tmp_product_CARRYCASCOUT_UNCONNECTED,
      CARRYIN => '0',
      CARRYINSEL(2 downto 0) => B"000",
      CARRYOUT(3 downto 0) => NLW_tmp_product_CARRYOUT_UNCONNECTED(3 downto 0),
      CEA1 => '0',
      CEA2 => add_ln32_reg_2830,
      CEAD => '0',
      CEALUMODE => '0',
      CEB1 => '0',
      CEB2 => '0',
      CEC => '0',
      CECARRYIN => '0',
      CECTRL => '0',
      CED => '0',
      CEINMODE => '0',
      CEM => '0',
      CEP => '0',
      CLK => ap_clk,
      D(24 downto 0) => B"0000000000000000000000000",
      INMODE(4 downto 0) => B"00000",
      MULTSIGNIN => '0',
      MULTSIGNOUT => NLW_tmp_product_MULTSIGNOUT_UNCONNECTED,
      OPMODE(6 downto 0) => B"0000101",
      OVERFLOW => NLW_tmp_product_OVERFLOW_UNCONNECTED,
      P(47) => tmp_product_n_58,
      P(46) => tmp_product_n_59,
      P(45) => tmp_product_n_60,
      P(44) => tmp_product_n_61,
      P(43) => tmp_product_n_62,
      P(42) => tmp_product_n_63,
      P(41) => tmp_product_n_64,
      P(40) => tmp_product_n_65,
      P(39) => tmp_product_n_66,
      P(38) => tmp_product_n_67,
      P(37) => tmp_product_n_68,
      P(36) => tmp_product_n_69,
      P(35) => tmp_product_n_70,
      P(34) => tmp_product_n_71,
      P(33) => tmp_product_n_72,
      P(32) => tmp_product_n_73,
      P(31) => tmp_product_n_74,
      P(30) => tmp_product_n_75,
      P(29) => tmp_product_n_76,
      P(28) => tmp_product_n_77,
      P(27) => tmp_product_n_78,
      P(26) => tmp_product_n_79,
      P(25) => tmp_product_n_80,
      P(24) => tmp_product_n_81,
      P(23) => tmp_product_n_82,
      P(22) => tmp_product_n_83,
      P(21) => tmp_product_n_84,
      P(20) => tmp_product_n_85,
      P(19) => tmp_product_n_86,
      P(18) => tmp_product_n_87,
      P(17) => tmp_product_n_88,
      P(16) => tmp_product_n_89,
      P(15) => tmp_product_n_90,
      P(14) => tmp_product_n_91,
      P(13) => tmp_product_n_92,
      P(12) => tmp_product_n_93,
      P(11) => tmp_product_n_94,
      P(10) => tmp_product_n_95,
      P(9) => tmp_product_n_96,
      P(8) => tmp_product_n_97,
      P(7) => tmp_product_n_98,
      P(6) => tmp_product_n_99,
      P(5) => tmp_product_n_100,
      P(4) => tmp_product_n_101,
      P(3) => tmp_product_n_102,
      P(2) => tmp_product_n_103,
      P(1) => tmp_product_n_104,
      P(0) => tmp_product_n_105,
      PATTERNBDETECT => NLW_tmp_product_PATTERNBDETECT_UNCONNECTED,
      PATTERNDETECT => NLW_tmp_product_PATTERNDETECT_UNCONNECTED,
      PCIN(47 downto 0) => B"000000000000000000000000000000000000000000000000",
      PCOUT(47) => tmp_product_n_106,
      PCOUT(46) => tmp_product_n_107,
      PCOUT(45) => tmp_product_n_108,
      PCOUT(44) => tmp_product_n_109,
      PCOUT(43) => tmp_product_n_110,
      PCOUT(42) => tmp_product_n_111,
      PCOUT(41) => tmp_product_n_112,
      PCOUT(40) => tmp_product_n_113,
      PCOUT(39) => tmp_product_n_114,
      PCOUT(38) => tmp_product_n_115,
      PCOUT(37) => tmp_product_n_116,
      PCOUT(36) => tmp_product_n_117,
      PCOUT(35) => tmp_product_n_118,
      PCOUT(34) => tmp_product_n_119,
      PCOUT(33) => tmp_product_n_120,
      PCOUT(32) => tmp_product_n_121,
      PCOUT(31) => tmp_product_n_122,
      PCOUT(30) => tmp_product_n_123,
      PCOUT(29) => tmp_product_n_124,
      PCOUT(28) => tmp_product_n_125,
      PCOUT(27) => tmp_product_n_126,
      PCOUT(26) => tmp_product_n_127,
      PCOUT(25) => tmp_product_n_128,
      PCOUT(24) => tmp_product_n_129,
      PCOUT(23) => tmp_product_n_130,
      PCOUT(22) => tmp_product_n_131,
      PCOUT(21) => tmp_product_n_132,
      PCOUT(20) => tmp_product_n_133,
      PCOUT(19) => tmp_product_n_134,
      PCOUT(18) => tmp_product_n_135,
      PCOUT(17) => tmp_product_n_136,
      PCOUT(16) => tmp_product_n_137,
      PCOUT(15) => tmp_product_n_138,
      PCOUT(14) => tmp_product_n_139,
      PCOUT(13) => tmp_product_n_140,
      PCOUT(12) => tmp_product_n_141,
      PCOUT(11) => tmp_product_n_142,
      PCOUT(10) => tmp_product_n_143,
      PCOUT(9) => tmp_product_n_144,
      PCOUT(8) => tmp_product_n_145,
      PCOUT(7) => tmp_product_n_146,
      PCOUT(6) => tmp_product_n_147,
      PCOUT(5) => tmp_product_n_148,
      PCOUT(4) => tmp_product_n_149,
      PCOUT(3) => tmp_product_n_150,
      PCOUT(2) => tmp_product_n_151,
      PCOUT(1) => tmp_product_n_152,
      PCOUT(0) => tmp_product_n_153,
      RSTA => '0',
      RSTALLCARRYIN => '0',
      RSTALUMODE => '0',
      RSTB => '0',
      RSTC => '0',
      RSTCTRL => '0',
      RSTD => '0',
      RSTINMODE => '0',
      RSTM => '0',
      RSTP => '0',
      UNDERFLOW => NLW_tmp_product_UNDERFLOW_UNCONNECTED
    );
\tmp_product__0\: unisim.vcomponents.DSP48E1
    generic map(
      ACASCREG => 1,
      ADREG => 1,
      ALUMODEREG => 0,
      AREG => 1,
      AUTORESET_PATDET => "NO_RESET",
      A_INPUT => "DIRECT",
      BCASCREG => 0,
      BREG => 0,
      B_INPUT => "DIRECT",
      CARRYINREG => 0,
      CARRYINSELREG => 0,
      CREG => 1,
      DREG => 1,
      INMODEREG => 0,
      MASK => X"3FFFFFFFFFFF",
      MREG => 0,
      OPMODEREG => 0,
      PATTERN => X"000000000000",
      PREG => 0,
      SEL_MASK => "MASK",
      SEL_PATTERN => "PATTERN",
      USE_DPORT => false,
      USE_MULT => "MULTIPLY",
      USE_PATTERN_DETECT => "NO_PATDET",
      USE_SIMD => "ONE48"
    )
        port map (
      A(29 downto 17) => B"0000000000000",
      A(16 downto 0) => add_ln32_fu_145_p2(16 downto 0),
      ACIN(29 downto 0) => B"000000000000000000000000000000",
      ACOUT(29) => \tmp_product__0_n_24\,
      ACOUT(28) => \tmp_product__0_n_25\,
      ACOUT(27) => \tmp_product__0_n_26\,
      ACOUT(26) => \tmp_product__0_n_27\,
      ACOUT(25) => \tmp_product__0_n_28\,
      ACOUT(24) => \tmp_product__0_n_29\,
      ACOUT(23) => \tmp_product__0_n_30\,
      ACOUT(22) => \tmp_product__0_n_31\,
      ACOUT(21) => \tmp_product__0_n_32\,
      ACOUT(20) => \tmp_product__0_n_33\,
      ACOUT(19) => \tmp_product__0_n_34\,
      ACOUT(18) => \tmp_product__0_n_35\,
      ACOUT(17) => \tmp_product__0_n_36\,
      ACOUT(16) => \tmp_product__0_n_37\,
      ACOUT(15) => \tmp_product__0_n_38\,
      ACOUT(14) => \tmp_product__0_n_39\,
      ACOUT(13) => \tmp_product__0_n_40\,
      ACOUT(12) => \tmp_product__0_n_41\,
      ACOUT(11) => \tmp_product__0_n_42\,
      ACOUT(10) => \tmp_product__0_n_43\,
      ACOUT(9) => \tmp_product__0_n_44\,
      ACOUT(8) => \tmp_product__0_n_45\,
      ACOUT(7) => \tmp_product__0_n_46\,
      ACOUT(6) => \tmp_product__0_n_47\,
      ACOUT(5) => \tmp_product__0_n_48\,
      ACOUT(4) => \tmp_product__0_n_49\,
      ACOUT(3) => \tmp_product__0_n_50\,
      ACOUT(2) => \tmp_product__0_n_51\,
      ACOUT(1) => \tmp_product__0_n_52\,
      ACOUT(0) => \tmp_product__0_n_53\,
      ALUMODE(3 downto 0) => B"0000",
      B(17 downto 0) => B"000001010001111011",
      BCIN(17 downto 0) => B"000000000000000000",
      BCOUT(17 downto 0) => \NLW_tmp_product__0_BCOUT_UNCONNECTED\(17 downto 0),
      C(47 downto 0) => B"111111111111111111111111111111111111111111111111",
      CARRYCASCIN => '0',
      CARRYCASCOUT => \NLW_tmp_product__0_CARRYCASCOUT_UNCONNECTED\,
      CARRYIN => '0',
      CARRYINSEL(2 downto 0) => B"000",
      CARRYOUT(3 downto 0) => \NLW_tmp_product__0_CARRYOUT_UNCONNECTED\(3 downto 0),
      CEA1 => '0',
      CEA2 => add_ln32_reg_2830,
      CEAD => '0',
      CEALUMODE => '0',
      CEB1 => '0',
      CEB2 => '0',
      CEC => '0',
      CECARRYIN => '0',
      CECTRL => '0',
      CED => '0',
      CEINMODE => '0',
      CEM => '0',
      CEP => '0',
      CLK => ap_clk,
      D(24 downto 0) => B"0000000000000000000000000",
      INMODE(4 downto 0) => B"00000",
      MULTSIGNIN => '0',
      MULTSIGNOUT => \NLW_tmp_product__0_MULTSIGNOUT_UNCONNECTED\,
      OPMODE(6 downto 0) => B"0000101",
      OVERFLOW => \NLW_tmp_product__0_OVERFLOW_UNCONNECTED\,
      P(47) => \tmp_product__0_n_58\,
      P(46) => \tmp_product__0_n_59\,
      P(45) => \tmp_product__0_n_60\,
      P(44) => \tmp_product__0_n_61\,
      P(43) => \tmp_product__0_n_62\,
      P(42) => \tmp_product__0_n_63\,
      P(41) => \tmp_product__0_n_64\,
      P(40) => \tmp_product__0_n_65\,
      P(39) => \tmp_product__0_n_66\,
      P(38) => \tmp_product__0_n_67\,
      P(37) => \tmp_product__0_n_68\,
      P(36) => \tmp_product__0_n_69\,
      P(35) => \tmp_product__0_n_70\,
      P(34) => \tmp_product__0_n_71\,
      P(33) => \tmp_product__0_n_72\,
      P(32) => \tmp_product__0_n_73\,
      P(31) => \tmp_product__0_n_74\,
      P(30) => \tmp_product__0_n_75\,
      P(29) => \tmp_product__0_n_76\,
      P(28) => \tmp_product__0_n_77\,
      P(27) => \tmp_product__0_n_78\,
      P(26) => \tmp_product__0_n_79\,
      P(25) => \tmp_product__0_n_80\,
      P(24) => \tmp_product__0_n_81\,
      P(23) => \tmp_product__0_n_82\,
      P(22) => \tmp_product__0_n_83\,
      P(21) => \tmp_product__0_n_84\,
      P(20) => \tmp_product__0_n_85\,
      P(19) => \tmp_product__0_n_86\,
      P(18) => \tmp_product__0_n_87\,
      P(17) => \tmp_product__0_n_88\,
      P(16) => \tmp_product__0_n_89\,
      P(15) => \tmp_product__0_n_90\,
      P(14) => \tmp_product__0_n_91\,
      P(13) => \tmp_product__0_n_92\,
      P(12) => \tmp_product__0_n_93\,
      P(11) => \tmp_product__0_n_94\,
      P(10) => \tmp_product__0_n_95\,
      P(9) => \tmp_product__0_n_96\,
      P(8) => \tmp_product__0_n_97\,
      P(7) => \tmp_product__0_n_98\,
      P(6) => \tmp_product__0_n_99\,
      P(5) => \tmp_product__0_n_100\,
      P(4) => \tmp_product__0_n_101\,
      P(3) => \tmp_product__0_n_102\,
      P(2) => \tmp_product__0_n_103\,
      P(1) => \tmp_product__0_n_104\,
      P(0) => \tmp_product__0_n_105\,
      PATTERNBDETECT => \NLW_tmp_product__0_PATTERNBDETECT_UNCONNECTED\,
      PATTERNDETECT => \NLW_tmp_product__0_PATTERNDETECT_UNCONNECTED\,
      PCIN(47 downto 0) => B"000000000000000000000000000000000000000000000000",
      PCOUT(47) => \tmp_product__0_n_106\,
      PCOUT(46) => \tmp_product__0_n_107\,
      PCOUT(45) => \tmp_product__0_n_108\,
      PCOUT(44) => \tmp_product__0_n_109\,
      PCOUT(43) => \tmp_product__0_n_110\,
      PCOUT(42) => \tmp_product__0_n_111\,
      PCOUT(41) => \tmp_product__0_n_112\,
      PCOUT(40) => \tmp_product__0_n_113\,
      PCOUT(39) => \tmp_product__0_n_114\,
      PCOUT(38) => \tmp_product__0_n_115\,
      PCOUT(37) => \tmp_product__0_n_116\,
      PCOUT(36) => \tmp_product__0_n_117\,
      PCOUT(35) => \tmp_product__0_n_118\,
      PCOUT(34) => \tmp_product__0_n_119\,
      PCOUT(33) => \tmp_product__0_n_120\,
      PCOUT(32) => \tmp_product__0_n_121\,
      PCOUT(31) => \tmp_product__0_n_122\,
      PCOUT(30) => \tmp_product__0_n_123\,
      PCOUT(29) => \tmp_product__0_n_124\,
      PCOUT(28) => \tmp_product__0_n_125\,
      PCOUT(27) => \tmp_product__0_n_126\,
      PCOUT(26) => \tmp_product__0_n_127\,
      PCOUT(25) => \tmp_product__0_n_128\,
      PCOUT(24) => \tmp_product__0_n_129\,
      PCOUT(23) => \tmp_product__0_n_130\,
      PCOUT(22) => \tmp_product__0_n_131\,
      PCOUT(21) => \tmp_product__0_n_132\,
      PCOUT(20) => \tmp_product__0_n_133\,
      PCOUT(19) => \tmp_product__0_n_134\,
      PCOUT(18) => \tmp_product__0_n_135\,
      PCOUT(17) => \tmp_product__0_n_136\,
      PCOUT(16) => \tmp_product__0_n_137\,
      PCOUT(15) => \tmp_product__0_n_138\,
      PCOUT(14) => \tmp_product__0_n_139\,
      PCOUT(13) => \tmp_product__0_n_140\,
      PCOUT(12) => \tmp_product__0_n_141\,
      PCOUT(11) => \tmp_product__0_n_142\,
      PCOUT(10) => \tmp_product__0_n_143\,
      PCOUT(9) => \tmp_product__0_n_144\,
      PCOUT(8) => \tmp_product__0_n_145\,
      PCOUT(7) => \tmp_product__0_n_146\,
      PCOUT(6) => \tmp_product__0_n_147\,
      PCOUT(5) => \tmp_product__0_n_148\,
      PCOUT(4) => \tmp_product__0_n_149\,
      PCOUT(3) => \tmp_product__0_n_150\,
      PCOUT(2) => \tmp_product__0_n_151\,
      PCOUT(1) => \tmp_product__0_n_152\,
      PCOUT(0) => \tmp_product__0_n_153\,
      RSTA => '0',
      RSTALLCARRYIN => '0',
      RSTALUMODE => '0',
      RSTB => '0',
      RSTC => '0',
      RSTCTRL => '0',
      RSTD => '0',
      RSTINMODE => '0',
      RSTM => '0',
      RSTP => '0',
      UNDERFLOW => \NLW_tmp_product__0_UNDERFLOW_UNCONNECTED\
    );
tmp_product_i_10: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(4),
      I1 => sum_reg(5),
      O => \sum_reg[22]\(1)
    );
tmp_product_i_11: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(3),
      I1 => sum_reg(4),
      O => \sum_reg[22]\(0)
    );
tmp_product_i_13: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(2),
      I1 => sum_reg(3),
      O => \sum_reg[18]\(2)
    );
tmp_product_i_14: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(1),
      I1 => sum_reg(2),
      O => \sum_reg[18]\(1)
    );
tmp_product_i_15: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(0),
      I1 => sum_reg(1),
      O => \sum_reg[18]\(0)
    );
tmp_product_i_4: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(10),
      I1 => sum_reg(11),
      O => \sum_reg[26]\(3)
    );
tmp_product_i_5: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(9),
      I1 => sum_reg(10),
      O => \sum_reg[26]\(2)
    );
tmp_product_i_6: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(8),
      I1 => sum_reg(9),
      O => \sum_reg[26]\(1)
    );
tmp_product_i_7: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(7),
      I1 => sum_reg(8),
      O => \sum_reg[26]\(0)
    );
tmp_product_i_8: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(6),
      I1 => sum_reg(7),
      O => \sum_reg[22]\(3)
    );
tmp_product_i_9: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => sum_reg(5),
      I1 => sum_reg(6),
      O => \sum_reg[22]\(2)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both is
  port (
    \B_V_data_1_state_reg[1]_0\ : out STD_LOGIC;
    ap_rst_n_inv : out STD_LOGIC;
    in_stream_TVALID_int_regslice : out STD_LOGIC;
    \in\ : out STD_LOGIC_VECTOR ( 15 downto 0 );
    O : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_reg[7]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_reg[11]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_reg[14]\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \sum_reg[14]_0\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \B_V_data_1_payload_B_reg[15]_0\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \B_V_data_1_payload_B_reg[15]_1\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    \B_V_data_1_payload_B_reg[15]_2\ : out STD_LOGIC_VECTOR ( 3 downto 0 );
    add_ln32_fu_145_p2 : out STD_LOGIC_VECTOR ( 31 downto 0 );
    ap_clk : in STD_LOGIC;
    \B_V_data_1_state_reg[1]_1\ : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    sum_reg : in STD_LOGIC_VECTOR ( 31 downto 0 );
    tmp_product : in STD_LOGIC_VECTOR ( 2 downto 0 );
    tmp_product_0 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    tmp_product_1 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    S : in STD_LOGIC_VECTOR ( 3 downto 0 );
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 15 downto 0 )
  );
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both is
  signal B_V_data_1_load_A : STD_LOGIC;
  signal B_V_data_1_load_B : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[2]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[3]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[4]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[5]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[6]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[7]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[8]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[9]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[2]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[3]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[4]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[5]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[6]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[7]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[8]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[9]\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__2_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__3_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__3_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[1]_0\ : STD_LOGIC;
  signal \add_ln32_reg_283_reg[31]_i_2_n_1\ : STD_LOGIC;
  signal \add_ln32_reg_283_reg[31]_i_2_n_2\ : STD_LOGIC;
  signal \add_ln32_reg_283_reg[31]_i_2_n_3\ : STD_LOGIC;
  signal \^ap_rst_n_inv\ : STD_LOGIC;
  signal \^in_stream_tvalid_int_regslice\ : STD_LOGIC;
  signal \sum[0]_i_2_n_0\ : STD_LOGIC;
  signal \sum[0]_i_3_n_0\ : STD_LOGIC;
  signal \sum[0]_i_4_n_0\ : STD_LOGIC;
  signal \sum[0]_i_5_n_0\ : STD_LOGIC;
  signal \sum[12]_i_2_n_0\ : STD_LOGIC;
  signal \sum[12]_i_3_n_0\ : STD_LOGIC;
  signal \sum[12]_i_4_n_0\ : STD_LOGIC;
  signal \sum[12]_i_5_n_0\ : STD_LOGIC;
  signal \sum[12]_i_6_n_0\ : STD_LOGIC;
  signal \sum[16]_i_2_n_0\ : STD_LOGIC;
  signal \sum[16]_i_3_n_0\ : STD_LOGIC;
  signal \sum[16]_i_4_n_0\ : STD_LOGIC;
  signal \sum[16]_i_5_n_0\ : STD_LOGIC;
  signal \sum[16]_i_6_n_0\ : STD_LOGIC;
  signal \sum[16]_i_7_n_0\ : STD_LOGIC;
  signal \sum[16]_i_8_n_0\ : STD_LOGIC;
  signal \sum[16]_i_9_n_0\ : STD_LOGIC;
  signal \sum[20]_i_2_n_0\ : STD_LOGIC;
  signal \sum[20]_i_3_n_0\ : STD_LOGIC;
  signal \sum[20]_i_4_n_0\ : STD_LOGIC;
  signal \sum[20]_i_5_n_0\ : STD_LOGIC;
  signal \sum[20]_i_6_n_0\ : STD_LOGIC;
  signal \sum[20]_i_7_n_0\ : STD_LOGIC;
  signal \sum[20]_i_8_n_0\ : STD_LOGIC;
  signal \sum[20]_i_9_n_0\ : STD_LOGIC;
  signal \sum[24]_i_2_n_0\ : STD_LOGIC;
  signal \sum[24]_i_3_n_0\ : STD_LOGIC;
  signal \sum[24]_i_4_n_0\ : STD_LOGIC;
  signal \sum[24]_i_5_n_0\ : STD_LOGIC;
  signal \sum[24]_i_6_n_0\ : STD_LOGIC;
  signal \sum[24]_i_7_n_0\ : STD_LOGIC;
  signal \sum[24]_i_8_n_0\ : STD_LOGIC;
  signal \sum[24]_i_9_n_0\ : STD_LOGIC;
  signal \sum[28]_i_2_n_0\ : STD_LOGIC;
  signal \sum[28]_i_3_n_0\ : STD_LOGIC;
  signal \sum[28]_i_4_n_0\ : STD_LOGIC;
  signal \sum[28]_i_5_n_0\ : STD_LOGIC;
  signal \sum[28]_i_6_n_0\ : STD_LOGIC;
  signal \sum[28]_i_7_n_0\ : STD_LOGIC;
  signal \sum[28]_i_8_n_0\ : STD_LOGIC;
  signal \sum[4]_i_2_n_0\ : STD_LOGIC;
  signal \sum[4]_i_3_n_0\ : STD_LOGIC;
  signal \sum[4]_i_4_n_0\ : STD_LOGIC;
  signal \sum[4]_i_5_n_0\ : STD_LOGIC;
  signal \sum[8]_i_2_n_0\ : STD_LOGIC;
  signal \sum[8]_i_3_n_0\ : STD_LOGIC;
  signal \sum[8]_i_4_n_0\ : STD_LOGIC;
  signal \sum[8]_i_5_n_0\ : STD_LOGIC;
  signal \sum_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \sum_reg[0]_i_1_n_1\ : STD_LOGIC;
  signal \sum_reg[0]_i_1_n_2\ : STD_LOGIC;
  signal \sum_reg[0]_i_1_n_3\ : STD_LOGIC;
  signal \sum_reg[12]_i_1_n_0\ : STD_LOGIC;
  signal \sum_reg[12]_i_1_n_1\ : STD_LOGIC;
  signal \sum_reg[12]_i_1_n_2\ : STD_LOGIC;
  signal \sum_reg[12]_i_1_n_3\ : STD_LOGIC;
  signal \sum_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \sum_reg[16]_i_1_n_1\ : STD_LOGIC;
  signal \sum_reg[16]_i_1_n_2\ : STD_LOGIC;
  signal \sum_reg[16]_i_1_n_3\ : STD_LOGIC;
  signal \sum_reg[20]_i_1_n_0\ : STD_LOGIC;
  signal \sum_reg[20]_i_1_n_1\ : STD_LOGIC;
  signal \sum_reg[20]_i_1_n_2\ : STD_LOGIC;
  signal \sum_reg[20]_i_1_n_3\ : STD_LOGIC;
  signal \sum_reg[24]_i_1_n_0\ : STD_LOGIC;
  signal \sum_reg[24]_i_1_n_1\ : STD_LOGIC;
  signal \sum_reg[24]_i_1_n_2\ : STD_LOGIC;
  signal \sum_reg[24]_i_1_n_3\ : STD_LOGIC;
  signal \sum_reg[28]_i_1_n_1\ : STD_LOGIC;
  signal \sum_reg[28]_i_1_n_2\ : STD_LOGIC;
  signal \sum_reg[28]_i_1_n_3\ : STD_LOGIC;
  signal \sum_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \sum_reg[4]_i_1_n_1\ : STD_LOGIC;
  signal \sum_reg[4]_i_1_n_2\ : STD_LOGIC;
  signal \sum_reg[4]_i_1_n_3\ : STD_LOGIC;
  signal \sum_reg[8]_i_1_n_0\ : STD_LOGIC;
  signal \sum_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \sum_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \sum_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal \tmp_product__0_i_10_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_11_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_12_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_13_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_14_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_15_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_16_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_17_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_18_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_19_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_1_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_1_n_1\ : STD_LOGIC;
  signal \tmp_product__0_i_1_n_2\ : STD_LOGIC;
  signal \tmp_product__0_i_1_n_3\ : STD_LOGIC;
  signal \tmp_product__0_i_20_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_21_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_2_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_2_n_1\ : STD_LOGIC;
  signal \tmp_product__0_i_2_n_2\ : STD_LOGIC;
  signal \tmp_product__0_i_2_n_3\ : STD_LOGIC;
  signal \tmp_product__0_i_3_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_3_n_1\ : STD_LOGIC;
  signal \tmp_product__0_i_3_n_2\ : STD_LOGIC;
  signal \tmp_product__0_i_3_n_3\ : STD_LOGIC;
  signal \tmp_product__0_i_4_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_4_n_1\ : STD_LOGIC;
  signal \tmp_product__0_i_4_n_2\ : STD_LOGIC;
  signal \tmp_product__0_i_4_n_3\ : STD_LOGIC;
  signal \tmp_product__0_i_5_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_6_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_7_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_8_n_0\ : STD_LOGIC;
  signal \tmp_product__0_i_9_n_0\ : STD_LOGIC;
  signal tmp_product_i_12_n_0 : STD_LOGIC;
  signal tmp_product_i_16_n_0 : STD_LOGIC;
  signal tmp_product_i_1_n_0 : STD_LOGIC;
  signal tmp_product_i_1_n_1 : STD_LOGIC;
  signal tmp_product_i_1_n_2 : STD_LOGIC;
  signal tmp_product_i_1_n_3 : STD_LOGIC;
  signal tmp_product_i_2_n_0 : STD_LOGIC;
  signal tmp_product_i_2_n_1 : STD_LOGIC;
  signal tmp_product_i_2_n_2 : STD_LOGIC;
  signal tmp_product_i_2_n_3 : STD_LOGIC;
  signal tmp_product_i_3_n_0 : STD_LOGIC;
  signal tmp_product_i_3_n_1 : STD_LOGIC;
  signal tmp_product_i_3_n_2 : STD_LOGIC;
  signal tmp_product_i_3_n_3 : STD_LOGIC;
  signal \NLW_add_ln32_reg_283_reg[31]_i_2_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_sum_reg[28]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__2\ : label is "soft_lutpair0";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_2\ : label is "soft_lutpair0";
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \add_ln32_reg_283_reg[31]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \sum_reg[0]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_reg[12]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_reg[16]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_reg[20]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_reg[24]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_reg[28]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_reg[4]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \sum_reg[8]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \tmp_product__0_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \tmp_product__0_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \tmp_product__0_i_3\ : label is 35;
  attribute ADDER_THRESHOLD of \tmp_product__0_i_4\ : label is 35;
  attribute ADDER_THRESHOLD of tmp_product_i_1 : label is 35;
  attribute ADDER_THRESHOLD of tmp_product_i_2 : label is 35;
  attribute ADDER_THRESHOLD of tmp_product_i_3 : label is 35;
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_i_1\ : label is "soft_lutpair3";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_i_1\ : label is "soft_lutpair2";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_i_1\ : label is "soft_lutpair1";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_i_1\ : label is "soft_lutpair8";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_i_1\ : label is "soft_lutpair7";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_i_1\ : label is "soft_lutpair6";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_i_1\ : label is "soft_lutpair5";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_i_1\ : label is "soft_lutpair4";
  attribute SOFT_HLUTNM of \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_i_1\ : label is "soft_lutpair4";
begin
  \B_V_data_1_state_reg[1]_0\ <= \^b_v_data_1_state_reg[1]_0\;
  ap_rst_n_inv <= \^ap_rst_n_inv\;
  in_stream_TVALID_int_regslice <= \^in_stream_tvalid_int_regslice\;
\B_V_data_1_payload_A[15]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"0D"
    )
        port map (
      I0 => \^in_stream_tvalid_int_regslice\,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => B_V_data_1_sel_wr,
      O => B_V_data_1_load_A
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(0),
      Q => \B_V_data_1_payload_A_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(10),
      Q => \B_V_data_1_payload_A_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(11),
      Q => \B_V_data_1_payload_A_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(12),
      Q => \B_V_data_1_payload_A_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(13),
      Q => \B_V_data_1_payload_A_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(14),
      Q => \B_V_data_1_payload_A_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(15),
      Q => \B_V_data_1_payload_A_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(1),
      Q => \B_V_data_1_payload_A_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(2),
      Q => \B_V_data_1_payload_A_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(3),
      Q => \B_V_data_1_payload_A_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(4),
      Q => \B_V_data_1_payload_A_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(5),
      Q => \B_V_data_1_payload_A_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(6),
      Q => \B_V_data_1_payload_A_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(7),
      Q => \B_V_data_1_payload_A_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(8),
      Q => \B_V_data_1_payload_A_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TDATA(9),
      Q => \B_V_data_1_payload_A_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_payload_B[15]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A2"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \^in_stream_tvalid_int_regslice\,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(0),
      Q => \B_V_data_1_payload_B_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(10),
      Q => \B_V_data_1_payload_B_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(11),
      Q => \B_V_data_1_payload_B_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(12),
      Q => \B_V_data_1_payload_B_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(13),
      Q => \B_V_data_1_payload_B_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(14),
      Q => \B_V_data_1_payload_B_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(15),
      Q => \B_V_data_1_payload_B_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(1),
      Q => \B_V_data_1_payload_B_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(2),
      Q => \B_V_data_1_payload_B_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(3),
      Q => \B_V_data_1_payload_B_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(4),
      Q => \B_V_data_1_payload_B_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(5),
      Q => \B_V_data_1_payload_B_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(6),
      Q => \B_V_data_1_payload_B_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(7),
      Q => \B_V_data_1_payload_B_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(8),
      Q => \B_V_data_1_payload_B_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TDATA(9),
      Q => \B_V_data_1_payload_B_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B4"
    )
        port map (
      I0 => \B_V_data_1_state_reg[1]_1\,
      I1 => \^in_stream_tvalid_int_regslice\,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__2_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__2_n_0\,
      Q => B_V_data_1_sel,
      R => \^ap_rst_n_inv\
    );
\B_V_data_1_sel_wr_i_1__3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => in_stream_TVALID,
      I1 => \^b_v_data_1_state_reg[1]_0\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__3_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__3_n_0\,
      Q => B_V_data_1_sel_wr,
      R => \^ap_rst_n_inv\
    );
\B_V_data_1_state[0]_i_1__3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"AAA080A0"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg[1]_1\,
      I2 => \^in_stream_tvalid_int_regslice\,
      I3 => \^b_v_data_1_state_reg[1]_0\,
      I4 => in_stream_TVALID,
      O => \B_V_data_1_state[0]_i_1__3_n_0\
    );
\B_V_data_1_state[1]_i_1__6\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => ap_rst_n,
      O => \^ap_rst_n_inv\
    );
\B_V_data_1_state[1]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"77F7"
    )
        port map (
      I0 => \B_V_data_1_state_reg[1]_1\,
      I1 => \^in_stream_tvalid_int_regslice\,
      I2 => \^b_v_data_1_state_reg[1]_0\,
      I3 => in_stream_TVALID,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__3_n_0\,
      Q => \^in_stream_tvalid_int_regslice\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \^b_v_data_1_state_reg[1]_0\,
      R => \^ap_rst_n_inv\
    );
\add_ln32_reg_283_reg[31]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => tmp_product_i_1_n_0,
      CO(3) => \NLW_add_ln32_reg_283_reg[31]_i_2_CO_UNCONNECTED\(3),
      CO(2) => \add_ln32_reg_283_reg[31]_i_2_n_1\,
      CO(1) => \add_ln32_reg_283_reg[31]_i_2_n_2\,
      CO(0) => \add_ln32_reg_283_reg[31]_i_2_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2 downto 0) => sum_reg(29 downto 27),
      O(3 downto 0) => add_ln32_fu_145_p2(31 downto 28),
      S(3 downto 0) => S(3 downto 0)
    );
\sum[0]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      I3 => sum_reg(3),
      O => \sum[0]_i_2_n_0\
    );
\sum[0]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I3 => sum_reg(2),
      O => \sum[0]_i_3_n_0\
    );
\sum[0]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      I3 => sum_reg(1),
      O => \sum[0]_i_4_n_0\
    );
\sum[0]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I3 => sum_reg(0),
      O => \sum[0]_i_5_n_0\
    );
\sum[12]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[12]_i_2_n_0\
    );
\sum[12]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(15),
      O => \sum[12]_i_3_n_0\
    );
\sum[12]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I3 => sum_reg(14),
      O => \sum[12]_i_4_n_0\
    );
\sum[12]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      I3 => sum_reg(13),
      O => \sum[12]_i_5_n_0\
    );
\sum[12]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I3 => sum_reg(12),
      O => \sum[12]_i_6_n_0\
    );
\sum[16]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[16]_i_2_n_0\
    );
\sum[16]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[16]_i_3_n_0\
    );
\sum[16]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[16]_i_4_n_0\
    );
\sum[16]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[16]_i_5_n_0\
    );
\sum[16]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(19),
      O => \sum[16]_i_6_n_0\
    );
\sum[16]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(18),
      O => \sum[16]_i_7_n_0\
    );
\sum[16]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(17),
      O => \sum[16]_i_8_n_0\
    );
\sum[16]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(16),
      O => \sum[16]_i_9_n_0\
    );
\sum[20]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[20]_i_2_n_0\
    );
\sum[20]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[20]_i_3_n_0\
    );
\sum[20]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[20]_i_4_n_0\
    );
\sum[20]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[20]_i_5_n_0\
    );
\sum[20]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(23),
      O => \sum[20]_i_6_n_0\
    );
\sum[20]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(22),
      O => \sum[20]_i_7_n_0\
    );
\sum[20]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(21),
      O => \sum[20]_i_8_n_0\
    );
\sum[20]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(20),
      O => \sum[20]_i_9_n_0\
    );
\sum[24]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[24]_i_2_n_0\
    );
\sum[24]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[24]_i_3_n_0\
    );
\sum[24]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[24]_i_4_n_0\
    );
\sum[24]_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[24]_i_5_n_0\
    );
\sum[24]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(27),
      O => \sum[24]_i_6_n_0\
    );
\sum[24]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(26),
      O => \sum[24]_i_7_n_0\
    );
\sum[24]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(25),
      O => \sum[24]_i_8_n_0\
    );
\sum[24]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(24),
      O => \sum[24]_i_9_n_0\
    );
\sum[28]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[28]_i_2_n_0\
    );
\sum[28]_i_3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[28]_i_3_n_0\
    );
\sum[28]_i_4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \sum[28]_i_4_n_0\
    );
\sum[28]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(31),
      O => \sum[28]_i_5_n_0\
    );
\sum[28]_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(30),
      O => \sum[28]_i_6_n_0\
    );
\sum[28]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(29),
      O => \sum[28]_i_7_n_0\
    );
\sum[28]_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(28),
      O => \sum[28]_i_8_n_0\
    );
\sum[4]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      I3 => sum_reg(7),
      O => \sum[4]_i_2_n_0\
    );
\sum[4]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I3 => sum_reg(6),
      O => \sum[4]_i_3_n_0\
    );
\sum[4]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      I3 => sum_reg(5),
      O => \sum[4]_i_4_n_0\
    );
\sum[4]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I3 => sum_reg(4),
      O => \sum[4]_i_5_n_0\
    );
\sum[8]_i_2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      I3 => sum_reg(11),
      O => \sum[8]_i_2_n_0\
    );
\sum[8]_i_3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I3 => sum_reg(10),
      O => \sum[8]_i_3_n_0\
    );
\sum[8]_i_4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      I3 => sum_reg(9),
      O => \sum[8]_i_4_n_0\
    );
\sum[8]_i_5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I3 => sum_reg(8),
      O => \sum[8]_i_5_n_0\
    );
\sum_reg[0]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \sum_reg[0]_i_1_n_0\,
      CO(2) => \sum_reg[0]_i_1_n_1\,
      CO(1) => \sum_reg[0]_i_1_n_2\,
      CO(0) => \sum_reg[0]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_reg(3 downto 0),
      O(3 downto 0) => O(3 downto 0),
      S(3) => \sum[0]_i_2_n_0\,
      S(2) => \sum[0]_i_3_n_0\,
      S(1) => \sum[0]_i_4_n_0\,
      S(0) => \sum[0]_i_5_n_0\
    );
\sum_reg[12]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_reg[8]_i_1_n_0\,
      CO(3) => \sum_reg[12]_i_1_n_0\,
      CO(2) => \sum_reg[12]_i_1_n_1\,
      CO(1) => \sum_reg[12]_i_1_n_2\,
      CO(0) => \sum_reg[12]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \sum[12]_i_2_n_0\,
      DI(2 downto 0) => sum_reg(14 downto 12),
      O(3 downto 0) => \sum_reg[14]\(3 downto 0),
      S(3) => \sum[12]_i_3_n_0\,
      S(2) => \sum[12]_i_4_n_0\,
      S(1) => \sum[12]_i_5_n_0\,
      S(0) => \sum[12]_i_6_n_0\
    );
\sum_reg[16]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_reg[12]_i_1_n_0\,
      CO(3) => \sum_reg[16]_i_1_n_0\,
      CO(2) => \sum_reg[16]_i_1_n_1\,
      CO(1) => \sum_reg[16]_i_1_n_2\,
      CO(0) => \sum_reg[16]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \sum[16]_i_2_n_0\,
      DI(2) => \sum[16]_i_3_n_0\,
      DI(1) => \sum[16]_i_4_n_0\,
      DI(0) => \sum[16]_i_5_n_0\,
      O(3 downto 0) => \sum_reg[14]_0\(3 downto 0),
      S(3) => \sum[16]_i_6_n_0\,
      S(2) => \sum[16]_i_7_n_0\,
      S(1) => \sum[16]_i_8_n_0\,
      S(0) => \sum[16]_i_9_n_0\
    );
\sum_reg[20]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_reg[16]_i_1_n_0\,
      CO(3) => \sum_reg[20]_i_1_n_0\,
      CO(2) => \sum_reg[20]_i_1_n_1\,
      CO(1) => \sum_reg[20]_i_1_n_2\,
      CO(0) => \sum_reg[20]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \sum[20]_i_2_n_0\,
      DI(2) => \sum[20]_i_3_n_0\,
      DI(1) => \sum[20]_i_4_n_0\,
      DI(0) => \sum[20]_i_5_n_0\,
      O(3 downto 0) => \B_V_data_1_payload_B_reg[15]_0\(3 downto 0),
      S(3) => \sum[20]_i_6_n_0\,
      S(2) => \sum[20]_i_7_n_0\,
      S(1) => \sum[20]_i_8_n_0\,
      S(0) => \sum[20]_i_9_n_0\
    );
\sum_reg[24]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_reg[20]_i_1_n_0\,
      CO(3) => \sum_reg[24]_i_1_n_0\,
      CO(2) => \sum_reg[24]_i_1_n_1\,
      CO(1) => \sum_reg[24]_i_1_n_2\,
      CO(0) => \sum_reg[24]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \sum[24]_i_2_n_0\,
      DI(2) => \sum[24]_i_3_n_0\,
      DI(1) => \sum[24]_i_4_n_0\,
      DI(0) => \sum[24]_i_5_n_0\,
      O(3 downto 0) => \B_V_data_1_payload_B_reg[15]_1\(3 downto 0),
      S(3) => \sum[24]_i_6_n_0\,
      S(2) => \sum[24]_i_7_n_0\,
      S(1) => \sum[24]_i_8_n_0\,
      S(0) => \sum[24]_i_9_n_0\
    );
\sum_reg[28]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_reg[24]_i_1_n_0\,
      CO(3) => \NLW_sum_reg[28]_i_1_CO_UNCONNECTED\(3),
      CO(2) => \sum_reg[28]_i_1_n_1\,
      CO(1) => \sum_reg[28]_i_1_n_2\,
      CO(0) => \sum_reg[28]_i_1_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2) => \sum[28]_i_2_n_0\,
      DI(1) => \sum[28]_i_3_n_0\,
      DI(0) => \sum[28]_i_4_n_0\,
      O(3 downto 0) => \B_V_data_1_payload_B_reg[15]_2\(3 downto 0),
      S(3) => \sum[28]_i_5_n_0\,
      S(2) => \sum[28]_i_6_n_0\,
      S(1) => \sum[28]_i_7_n_0\,
      S(0) => \sum[28]_i_8_n_0\
    );
\sum_reg[4]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_reg[0]_i_1_n_0\,
      CO(3) => \sum_reg[4]_i_1_n_0\,
      CO(2) => \sum_reg[4]_i_1_n_1\,
      CO(1) => \sum_reg[4]_i_1_n_2\,
      CO(0) => \sum_reg[4]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_reg(7 downto 4),
      O(3 downto 0) => \sum_reg[7]\(3 downto 0),
      S(3) => \sum[4]_i_2_n_0\,
      S(2) => \sum[4]_i_3_n_0\,
      S(1) => \sum[4]_i_4_n_0\,
      S(0) => \sum[4]_i_5_n_0\
    );
\sum_reg[8]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \sum_reg[4]_i_1_n_0\,
      CO(3) => \sum_reg[8]_i_1_n_0\,
      CO(2) => \sum_reg[8]_i_1_n_1\,
      CO(1) => \sum_reg[8]_i_1_n_2\,
      CO(0) => \sum_reg[8]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_reg(11 downto 8),
      O(3 downto 0) => \sum_reg[11]\(3 downto 0),
      S(3) => \sum[8]_i_2_n_0\,
      S(2) => \sum[8]_i_3_n_0\,
      S(1) => \sum[8]_i_4_n_0\,
      S(0) => \sum[8]_i_5_n_0\
    );
\tmp_product__0_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \tmp_product__0_i_2_n_0\,
      CO(3) => \tmp_product__0_i_1_n_0\,
      CO(2) => \tmp_product__0_i_1_n_1\,
      CO(1) => \tmp_product__0_i_1_n_2\,
      CO(0) => \tmp_product__0_i_1_n_3\,
      CYINIT => '0',
      DI(3) => \tmp_product__0_i_5_n_0\,
      DI(2 downto 0) => sum_reg(14 downto 12),
      O(3 downto 0) => add_ln32_fu_145_p2(15 downto 12),
      S(3) => \tmp_product__0_i_6_n_0\,
      S(2) => \tmp_product__0_i_7_n_0\,
      S(1) => \tmp_product__0_i_8_n_0\,
      S(0) => \tmp_product__0_i_9_n_0\
    );
\tmp_product__0_i_10\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(11),
      I1 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      O => \tmp_product__0_i_10_n_0\
    );
\tmp_product__0_i_11\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(10),
      I1 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      O => \tmp_product__0_i_11_n_0\
    );
\tmp_product__0_i_12\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(9),
      I1 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      O => \tmp_product__0_i_12_n_0\
    );
\tmp_product__0_i_13\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(8),
      I1 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      O => \tmp_product__0_i_13_n_0\
    );
\tmp_product__0_i_14\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(7),
      I1 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      O => \tmp_product__0_i_14_n_0\
    );
\tmp_product__0_i_15\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(6),
      I1 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      O => \tmp_product__0_i_15_n_0\
    );
\tmp_product__0_i_16\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(5),
      I1 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      O => \tmp_product__0_i_16_n_0\
    );
\tmp_product__0_i_17\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(4),
      I1 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      O => \tmp_product__0_i_17_n_0\
    );
\tmp_product__0_i_18\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(3),
      I1 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      O => \tmp_product__0_i_18_n_0\
    );
\tmp_product__0_i_19\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(2),
      I1 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      O => \tmp_product__0_i_19_n_0\
    );
\tmp_product__0_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \tmp_product__0_i_3_n_0\,
      CO(3) => \tmp_product__0_i_2_n_0\,
      CO(2) => \tmp_product__0_i_2_n_1\,
      CO(1) => \tmp_product__0_i_2_n_2\,
      CO(0) => \tmp_product__0_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_reg(11 downto 8),
      O(3 downto 0) => add_ln32_fu_145_p2(11 downto 8),
      S(3) => \tmp_product__0_i_10_n_0\,
      S(2) => \tmp_product__0_i_11_n_0\,
      S(1) => \tmp_product__0_i_12_n_0\,
      S(0) => \tmp_product__0_i_13_n_0\
    );
\tmp_product__0_i_20\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(1),
      I1 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      O => \tmp_product__0_i_20_n_0\
    );
\tmp_product__0_i_21\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(0),
      I1 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      O => \tmp_product__0_i_21_n_0\
    );
\tmp_product__0_i_3\: unisim.vcomponents.CARRY4
     port map (
      CI => \tmp_product__0_i_4_n_0\,
      CO(3) => \tmp_product__0_i_3_n_0\,
      CO(2) => \tmp_product__0_i_3_n_1\,
      CO(1) => \tmp_product__0_i_3_n_2\,
      CO(0) => \tmp_product__0_i_3_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_reg(7 downto 4),
      O(3 downto 0) => add_ln32_fu_145_p2(7 downto 4),
      S(3) => \tmp_product__0_i_14_n_0\,
      S(2) => \tmp_product__0_i_15_n_0\,
      S(1) => \tmp_product__0_i_16_n_0\,
      S(0) => \tmp_product__0_i_17_n_0\
    );
\tmp_product__0_i_4\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \tmp_product__0_i_4_n_0\,
      CO(2) => \tmp_product__0_i_4_n_1\,
      CO(1) => \tmp_product__0_i_4_n_2\,
      CO(0) => \tmp_product__0_i_4_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => sum_reg(3 downto 0),
      O(3 downto 0) => add_ln32_fu_145_p2(3 downto 0),
      S(3) => \tmp_product__0_i_18_n_0\,
      S(2) => \tmp_product__0_i_19_n_0\,
      S(1) => \tmp_product__0_i_20_n_0\,
      S(0) => \tmp_product__0_i_21_n_0\
    );
\tmp_product__0_i_5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \tmp_product__0_i_5_n_0\
    );
\tmp_product__0_i_6\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(15),
      O => \tmp_product__0_i_6_n_0\
    );
\tmp_product__0_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(14),
      I1 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      O => \tmp_product__0_i_7_n_0\
    );
\tmp_product__0_i_8\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(13),
      I1 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      O => \tmp_product__0_i_8_n_0\
    );
\tmp_product__0_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"56A6"
    )
        port map (
      I0 => sum_reg(12),
      I1 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      I2 => B_V_data_1_sel,
      I3 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      O => \tmp_product__0_i_9_n_0\
    );
tmp_product_i_1: unisim.vcomponents.CARRY4
     port map (
      CI => tmp_product_i_2_n_0,
      CO(3) => tmp_product_i_1_n_0,
      CO(2) => tmp_product_i_1_n_1,
      CO(1) => tmp_product_i_1_n_2,
      CO(0) => tmp_product_i_1_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_reg(26 downto 23),
      O(3 downto 0) => add_ln32_fu_145_p2(27 downto 24),
      S(3 downto 0) => tmp_product_1(3 downto 0)
    );
tmp_product_i_12: unisim.vcomponents.LUT3
    generic map(
      INIT => X"1D"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      O => tmp_product_i_12_n_0
    );
tmp_product_i_16: unisim.vcomponents.LUT4
    generic map(
      INIT => X"1DE2"
    )
        port map (
      I0 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I3 => sum_reg(16),
      O => tmp_product_i_16_n_0
    );
tmp_product_i_2: unisim.vcomponents.CARRY4
     port map (
      CI => tmp_product_i_3_n_0,
      CO(3) => tmp_product_i_2_n_0,
      CO(2) => tmp_product_i_2_n_1,
      CO(1) => tmp_product_i_2_n_2,
      CO(0) => tmp_product_i_2_n_3,
      CYINIT => '0',
      DI(3 downto 0) => sum_reg(22 downto 19),
      O(3 downto 0) => add_ln32_fu_145_p2(23 downto 20),
      S(3 downto 0) => tmp_product_0(3 downto 0)
    );
tmp_product_i_3: unisim.vcomponents.CARRY4
     port map (
      CI => \tmp_product__0_i_1_n_0\,
      CO(3) => tmp_product_i_3_n_0,
      CO(2) => tmp_product_i_3_n_1,
      CO(1) => tmp_product_i_3_n_2,
      CO(0) => tmp_product_i_3_n_3,
      CYINIT => '0',
      DI(3 downto 1) => sum_reg(18 downto 16),
      DI(0) => tmp_product_i_12_n_0,
      O(3 downto 0) => add_ln32_fu_145_p2(19 downto 16),
      S(3 downto 1) => tmp_product(2 downto 0),
      S(0) => tmp_product_i_16_n_0
    );
\val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      O => \in\(0)
    );
\val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      O => \in\(10)
    );
\val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      O => \in\(11)
    );
\val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      O => \in\(12)
    );
\val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      O => \in\(13)
    );
\val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      O => \in\(14)
    );
\val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => \in\(15)
    );
\val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      O => \in\(1)
    );
\val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      O => \in\(2)
    );
\val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      O => \in\(3)
    );
\val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      O => \in\(4)
    );
\val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      O => \in\(5)
    );
\val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      O => \in\(6)
    );
\val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      O => \in\(7)
    );
\val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      O => \in\(8)
    );
\val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      I1 => B_V_data_1_sel,
      I2 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      O => \in\(9)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both_1 is
  port (
    \B_V_data_1_state_reg[0]_0\ : out STD_LOGIC;
    out_stream_TVALID_int_regslice : out STD_LOGIC;
    E : out STD_LOGIC_VECTOR ( 0 to 0 );
    \B_V_data_1_state_reg[0]_1\ : out STD_LOGIC;
    \icmp_ln35_reg_289_pp0_iter2_reg_reg[0]\ : out STD_LOGIC_VECTOR ( 0 to 0 );
    \calibrated_reg[0]\ : out STD_LOGIC;
    \counter_reg[0]\ : out STD_LOGIC;
    \counter_reg[0]_0\ : out STD_LOGIC;
    \counter_reg[28]\ : out STD_LOGIC;
    \counter_reg[20]\ : out STD_LOGIC;
    add_ln32_reg_2830 : out STD_LOGIC;
    ap_block_pp0_stage0_11001 : out STD_LOGIC;
    \add_ln32_reg_283_reg[31]\ : out STD_LOGIC;
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 15 downto 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    calibrated_load_reg_279_pp0_iter1_reg : in STD_LOGIC;
    icmp_ln35_reg_289_pp0_iter1_reg : in STD_LOGIC;
    icmp_ln35_reg_289_pp0_iter2_reg : in STD_LOGIC;
    ap_enable_reg_pp0_iter3 : in STD_LOGIC;
    calibrated_load_reg_279_pp0_iter2_reg : in STD_LOGIC;
    calibrated : in STD_LOGIC;
    in_stream_TVALID_int_regslice : in STD_LOGIC;
    ap_enable_reg_pp0_iter4 : in STD_LOGIC;
    calibrated_load_reg_279_pp0_iter3_reg : in STD_LOGIC;
    \out\ : in STD_LOGIC_VECTOR ( 31 downto 0 );
    p_0_in : in STD_LOGIC;
    calibrated_load_reg_279 : in STD_LOGIC;
    icmp_ln35_reg_289 : in STD_LOGIC;
    tmp_reg_298 : in STD_LOGIC;
    val_in_reg_274_pp0_iter2_reg : in STD_LOGIC_VECTOR ( 15 downto 0 );
    Q : in STD_LOGIC_VECTOR ( 15 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both_1 : entity is "fsk_phase_corrector_regslice_both";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both_1;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both_1 is
  signal \B_V_data_1_payload_A[11]_i_2_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[11]_i_3_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[11]_i_4_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[11]_i_5_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[15]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[15]_i_3_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[15]_i_4_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[15]_i_5_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[15]_i_6_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[3]_i_2_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[3]_i_3_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[3]_i_4_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[3]_i_5_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[7]_i_2_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[7]_i_3_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[7]_i_4_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A[7]_i_5_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[11]_i_1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[11]_i_1_n_1\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[11]_i_1_n_2\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[11]_i_1_n_3\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[15]_i_2_n_1\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[15]_i_2_n_2\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[15]_i_2_n_3\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[3]_i_1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[3]_i_1_n_1\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[3]_i_1_n_2\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[3]_i_1_n_3\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[7]_i_1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[7]_i_1_n_1\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[7]_i_1_n_2\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg[7]_i_1_n_3\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[2]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[3]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[4]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[5]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[6]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[7]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[8]\ : STD_LOGIC;
  signal \B_V_data_1_payload_A_reg_n_0_[9]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B[15]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[10]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[11]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[12]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[13]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[14]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[15]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[1]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[2]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[3]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[4]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[5]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[6]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[7]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[8]\ : STD_LOGIC;
  signal \B_V_data_1_payload_B_reg_n_0_[9]\ : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__3_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_rd_reg_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__2_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state[1]_i_4_n_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[0]_0\ : STD_LOGIC;
  signal \^b_v_data_1_state_reg[0]_1\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  signal add_ln33_fu_161_p2 : STD_LOGIC_VECTOR ( 31 downto 1 );
  signal \^counter_reg[0]\ : STD_LOGIC;
  signal \^counter_reg[0]_0\ : STD_LOGIC;
  signal \^counter_reg[20]\ : STD_LOGIC;
  signal \^counter_reg[28]\ : STD_LOGIC;
  signal \icmp_ln35_reg_289[0]_i_12_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289[0]_i_14_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289[0]_i_7_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289[0]_i_9_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_10_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_10_n_1\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_10_n_2\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_10_n_3\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_11_n_2\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_11_n_3\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_13_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_13_n_1\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_13_n_2\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_13_n_3\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_15_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_15_n_1\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_15_n_2\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_15_n_3\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_16_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_16_n_1\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_16_n_2\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_16_n_3\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_17_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_17_n_1\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_17_n_2\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_17_n_3\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_6_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_6_n_1\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_6_n_2\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_6_n_3\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_8_n_0\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_8_n_1\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_8_n_2\ : STD_LOGIC;
  signal \icmp_ln35_reg_289_reg[0]_i_8_n_3\ : STD_LOGIC;
  signal \^out_stream_tvalid_int_regslice\ : STD_LOGIC;
  signal val_corrected_fu_249_p2 : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal \NLW_B_V_data_1_payload_A_reg[15]_i_2_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_icmp_ln35_reg_289_reg[0]_i_11_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_icmp_ln35_reg_289_reg[0]_i_11_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \B_V_data_1_payload_A_reg[11]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \B_V_data_1_payload_A_reg[15]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \B_V_data_1_payload_A_reg[3]_i_1\ : label is 35;
  attribute ADDER_THRESHOLD of \B_V_data_1_payload_A_reg[7]_i_1\ : label is 35;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__3\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \B_V_data_1_state[0]_i_2\ : label is "soft_lutpair16";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1\ : label is "soft_lutpair17";
  attribute SOFT_HLUTNM of \dc_offset[15]_i_1\ : label is "soft_lutpair16";
  attribute ADDER_THRESHOLD of \icmp_ln35_reg_289_reg[0]_i_10\ : label is 35;
  attribute ADDER_THRESHOLD of \icmp_ln35_reg_289_reg[0]_i_11\ : label is 35;
  attribute ADDER_THRESHOLD of \icmp_ln35_reg_289_reg[0]_i_13\ : label is 35;
  attribute ADDER_THRESHOLD of \icmp_ln35_reg_289_reg[0]_i_15\ : label is 35;
  attribute ADDER_THRESHOLD of \icmp_ln35_reg_289_reg[0]_i_16\ : label is 35;
  attribute ADDER_THRESHOLD of \icmp_ln35_reg_289_reg[0]_i_17\ : label is 35;
  attribute ADDER_THRESHOLD of \icmp_ln35_reg_289_reg[0]_i_6\ : label is 35;
  attribute ADDER_THRESHOLD of \icmp_ln35_reg_289_reg[0]_i_8\ : label is 35;
  attribute SOFT_HLUTNM of \out_stream_TDATA[0]_INST_0\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \out_stream_TDATA[10]_INST_0\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \out_stream_TDATA[11]_INST_0\ : label is "soft_lutpair23";
  attribute SOFT_HLUTNM of \out_stream_TDATA[12]_INST_0\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \out_stream_TDATA[13]_INST_0\ : label is "soft_lutpair24";
  attribute SOFT_HLUTNM of \out_stream_TDATA[14]_INST_0\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \out_stream_TDATA[15]_INST_0\ : label is "soft_lutpair25";
  attribute SOFT_HLUTNM of \out_stream_TDATA[1]_INST_0\ : label is "soft_lutpair18";
  attribute SOFT_HLUTNM of \out_stream_TDATA[2]_INST_0\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \out_stream_TDATA[3]_INST_0\ : label is "soft_lutpair19";
  attribute SOFT_HLUTNM of \out_stream_TDATA[4]_INST_0\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \out_stream_TDATA[5]_INST_0\ : label is "soft_lutpair20";
  attribute SOFT_HLUTNM of \out_stream_TDATA[6]_INST_0\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \out_stream_TDATA[7]_INST_0\ : label is "soft_lutpair21";
  attribute SOFT_HLUTNM of \out_stream_TDATA[8]_INST_0\ : label is "soft_lutpair22";
  attribute SOFT_HLUTNM of \out_stream_TDATA[9]_INST_0\ : label is "soft_lutpair22";
begin
  \B_V_data_1_state_reg[0]_0\ <= \^b_v_data_1_state_reg[0]_0\;
  \B_V_data_1_state_reg[0]_1\ <= \^b_v_data_1_state_reg[0]_1\;
  \counter_reg[0]\ <= \^counter_reg[0]\;
  \counter_reg[0]_0\ <= \^counter_reg[0]_0\;
  \counter_reg[20]\ <= \^counter_reg[20]\;
  \counter_reg[28]\ <= \^counter_reg[28]\;
  out_stream_TVALID_int_regslice <= \^out_stream_tvalid_int_regslice\;
\B_V_data_1_payload_A[11]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(11),
      I1 => Q(11),
      O => \B_V_data_1_payload_A[11]_i_2_n_0\
    );
\B_V_data_1_payload_A[11]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(10),
      I1 => Q(10),
      O => \B_V_data_1_payload_A[11]_i_3_n_0\
    );
\B_V_data_1_payload_A[11]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(9),
      I1 => Q(9),
      O => \B_V_data_1_payload_A[11]_i_4_n_0\
    );
\B_V_data_1_payload_A[11]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(8),
      I1 => Q(8),
      O => \B_V_data_1_payload_A[11]_i_5_n_0\
    );
\B_V_data_1_payload_A[15]_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"0B"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[1]\,
      I1 => \^b_v_data_1_state_reg[0]_0\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_payload_A[15]_i_1__0_n_0\
    );
\B_V_data_1_payload_A[15]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(15),
      I1 => Q(15),
      O => \B_V_data_1_payload_A[15]_i_3_n_0\
    );
\B_V_data_1_payload_A[15]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(14),
      I1 => Q(14),
      O => \B_V_data_1_payload_A[15]_i_4_n_0\
    );
\B_V_data_1_payload_A[15]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(13),
      I1 => Q(13),
      O => \B_V_data_1_payload_A[15]_i_5_n_0\
    );
\B_V_data_1_payload_A[15]_i_6\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(12),
      I1 => Q(12),
      O => \B_V_data_1_payload_A[15]_i_6_n_0\
    );
\B_V_data_1_payload_A[3]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(3),
      I1 => Q(3),
      O => \B_V_data_1_payload_A[3]_i_2_n_0\
    );
\B_V_data_1_payload_A[3]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(2),
      I1 => Q(2),
      O => \B_V_data_1_payload_A[3]_i_3_n_0\
    );
\B_V_data_1_payload_A[3]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(1),
      I1 => Q(1),
      O => \B_V_data_1_payload_A[3]_i_4_n_0\
    );
\B_V_data_1_payload_A[3]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(0),
      I1 => Q(0),
      O => \B_V_data_1_payload_A[3]_i_5_n_0\
    );
\B_V_data_1_payload_A[7]_i_2\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(7),
      I1 => Q(7),
      O => \B_V_data_1_payload_A[7]_i_2_n_0\
    );
\B_V_data_1_payload_A[7]_i_3\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(6),
      I1 => Q(6),
      O => \B_V_data_1_payload_A[7]_i_3_n_0\
    );
\B_V_data_1_payload_A[7]_i_4\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(5),
      I1 => Q(5),
      O => \B_V_data_1_payload_A[7]_i_4_n_0\
    );
\B_V_data_1_payload_A[7]_i_5\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"9"
    )
        port map (
      I0 => val_in_reg_274_pp0_iter2_reg(4),
      I1 => Q(4),
      O => \B_V_data_1_payload_A[7]_i_5_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(0),
      Q => \B_V_data_1_payload_A_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(10),
      Q => \B_V_data_1_payload_A_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(11),
      Q => \B_V_data_1_payload_A_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[11]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \B_V_data_1_payload_A_reg[7]_i_1_n_0\,
      CO(3) => \B_V_data_1_payload_A_reg[11]_i_1_n_0\,
      CO(2) => \B_V_data_1_payload_A_reg[11]_i_1_n_1\,
      CO(1) => \B_V_data_1_payload_A_reg[11]_i_1_n_2\,
      CO(0) => \B_V_data_1_payload_A_reg[11]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => val_in_reg_274_pp0_iter2_reg(11 downto 8),
      O(3 downto 0) => val_corrected_fu_249_p2(11 downto 8),
      S(3) => \B_V_data_1_payload_A[11]_i_2_n_0\,
      S(2) => \B_V_data_1_payload_A[11]_i_3_n_0\,
      S(1) => \B_V_data_1_payload_A[11]_i_4_n_0\,
      S(0) => \B_V_data_1_payload_A[11]_i_5_n_0\
    );
\B_V_data_1_payload_A_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(12),
      Q => \B_V_data_1_payload_A_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(13),
      Q => \B_V_data_1_payload_A_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(14),
      Q => \B_V_data_1_payload_A_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(15),
      Q => \B_V_data_1_payload_A_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[15]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \B_V_data_1_payload_A_reg[11]_i_1_n_0\,
      CO(3) => \NLW_B_V_data_1_payload_A_reg[15]_i_2_CO_UNCONNECTED\(3),
      CO(2) => \B_V_data_1_payload_A_reg[15]_i_2_n_1\,
      CO(1) => \B_V_data_1_payload_A_reg[15]_i_2_n_2\,
      CO(0) => \B_V_data_1_payload_A_reg[15]_i_2_n_3\,
      CYINIT => '0',
      DI(3) => '0',
      DI(2 downto 0) => val_in_reg_274_pp0_iter2_reg(14 downto 12),
      O(3 downto 0) => val_corrected_fu_249_p2(15 downto 12),
      S(3) => \B_V_data_1_payload_A[15]_i_3_n_0\,
      S(2) => \B_V_data_1_payload_A[15]_i_4_n_0\,
      S(1) => \B_V_data_1_payload_A[15]_i_5_n_0\,
      S(0) => \B_V_data_1_payload_A[15]_i_6_n_0\
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(1),
      Q => \B_V_data_1_payload_A_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(2),
      Q => \B_V_data_1_payload_A_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(3),
      Q => \B_V_data_1_payload_A_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \B_V_data_1_payload_A_reg[3]_i_1_n_0\,
      CO(2) => \B_V_data_1_payload_A_reg[3]_i_1_n_1\,
      CO(1) => \B_V_data_1_payload_A_reg[3]_i_1_n_2\,
      CO(0) => \B_V_data_1_payload_A_reg[3]_i_1_n_3\,
      CYINIT => '1',
      DI(3 downto 0) => val_in_reg_274_pp0_iter2_reg(3 downto 0),
      O(3 downto 0) => val_corrected_fu_249_p2(3 downto 0),
      S(3) => \B_V_data_1_payload_A[3]_i_2_n_0\,
      S(2) => \B_V_data_1_payload_A[3]_i_3_n_0\,
      S(1) => \B_V_data_1_payload_A[3]_i_4_n_0\,
      S(0) => \B_V_data_1_payload_A[3]_i_5_n_0\
    );
\B_V_data_1_payload_A_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(4),
      Q => \B_V_data_1_payload_A_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(5),
      Q => \B_V_data_1_payload_A_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(6),
      Q => \B_V_data_1_payload_A_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(7),
      Q => \B_V_data_1_payload_A_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[7]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \B_V_data_1_payload_A_reg[3]_i_1_n_0\,
      CO(3) => \B_V_data_1_payload_A_reg[7]_i_1_n_0\,
      CO(2) => \B_V_data_1_payload_A_reg[7]_i_1_n_1\,
      CO(1) => \B_V_data_1_payload_A_reg[7]_i_1_n_2\,
      CO(0) => \B_V_data_1_payload_A_reg[7]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => val_in_reg_274_pp0_iter2_reg(7 downto 4),
      O(3 downto 0) => val_corrected_fu_249_p2(7 downto 4),
      S(3) => \B_V_data_1_payload_A[7]_i_2_n_0\,
      S(2) => \B_V_data_1_payload_A[7]_i_3_n_0\,
      S(1) => \B_V_data_1_payload_A[7]_i_4_n_0\,
      S(0) => \B_V_data_1_payload_A[7]_i_5_n_0\
    );
\B_V_data_1_payload_A_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(8),
      Q => \B_V_data_1_payload_A_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_A_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_A[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(9),
      Q => \B_V_data_1_payload_A_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_payload_B[15]_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"8A"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => \^b_v_data_1_state_reg[0]_0\,
      O => \B_V_data_1_payload_B[15]_i_1__0_n_0\
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(0),
      Q => \B_V_data_1_payload_B_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(10),
      Q => \B_V_data_1_payload_B_reg_n_0_[10]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(11),
      Q => \B_V_data_1_payload_B_reg_n_0_[11]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(12),
      Q => \B_V_data_1_payload_B_reg_n_0_[12]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(13),
      Q => \B_V_data_1_payload_B_reg_n_0_[13]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(14),
      Q => \B_V_data_1_payload_B_reg_n_0_[14]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(15),
      Q => \B_V_data_1_payload_B_reg_n_0_[15]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(1),
      Q => \B_V_data_1_payload_B_reg_n_0_[1]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(2),
      Q => \B_V_data_1_payload_B_reg_n_0_[2]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(3),
      Q => \B_V_data_1_payload_B_reg_n_0_[3]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(4),
      Q => \B_V_data_1_payload_B_reg_n_0_[4]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(5),
      Q => \B_V_data_1_payload_B_reg_n_0_[5]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(6),
      Q => \B_V_data_1_payload_B_reg_n_0_[6]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(7),
      Q => \B_V_data_1_payload_B_reg_n_0_[7]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(8),
      Q => \B_V_data_1_payload_B_reg_n_0_[8]\,
      R => '0'
    );
\B_V_data_1_payload_B_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => \B_V_data_1_payload_B[15]_i_1__0_n_0\,
      D => val_corrected_fu_249_p2(9),
      Q => \B_V_data_1_payload_B_reg_n_0_[9]\,
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__3\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \^b_v_data_1_state_reg[0]_0\,
      I2 => B_V_data_1_sel_rd_reg_n_0,
      O => \B_V_data_1_sel_rd_i_1__3_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__3_n_0\,
      Q => B_V_data_1_sel_rd_reg_n_0,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => \^out_stream_tvalid_int_regslice\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__2_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__2_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8A820A0"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => \^b_v_data_1_state_reg[0]_0\,
      I3 => out_stream_TREADY,
      I4 => \^out_stream_tvalid_int_regslice\,
      O => \B_V_data_1_state[0]_i_1_n_0\
    );
\B_V_data_1_state[0]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"08"
    )
        port map (
      I0 => calibrated_load_reg_279_pp0_iter2_reg,
      I1 => ap_enable_reg_pp0_iter3,
      I2 => \^b_v_data_1_state_reg[0]_1\,
      O => \^out_stream_tvalid_int_regslice\
    );
\B_V_data_1_state[1]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F3FB"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[1]\,
      I1 => \^b_v_data_1_state_reg[0]_0\,
      I2 => out_stream_TREADY,
      I3 => \^out_stream_tvalid_int_regslice\,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state[1]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFFF7555"
    )
        port map (
      I0 => in_stream_TVALID_int_regslice,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => calibrated_load_reg_279_pp0_iter2_reg,
      I3 => ap_enable_reg_pp0_iter3,
      I4 => \B_V_data_1_state[1]_i_4_n_0\,
      O => \^b_v_data_1_state_reg[0]_1\
    );
\B_V_data_1_state[1]_i_4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"22A220A0"
    )
        port map (
      I0 => ap_enable_reg_pp0_iter4,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => \^b_v_data_1_state_reg[0]_0\,
      I3 => out_stream_TREADY,
      I4 => calibrated_load_reg_279_pp0_iter3_reg,
      O => \B_V_data_1_state[1]_i_4_n_0\
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1_n_0\,
      Q => \^b_v_data_1_state_reg[0]_0\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\add_ln32_reg_283[31]_i_1\: unisim.vcomponents.LUT2
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_1\,
      I1 => calibrated,
      O => add_ln32_reg_2830
    );
\calibrated[0]_i_1\: unisim.vcomponents.LUT6
    generic map(
      INIT => X"FFFFFFFF40000000"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_1\,
      I1 => \^counter_reg[0]\,
      I2 => \^counter_reg[0]_0\,
      I3 => \^counter_reg[28]\,
      I4 => \^counter_reg[20]\,
      I5 => calibrated,
      O => \calibrated_reg[0]\
    );
\dc_offset[15]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0008"
    )
        port map (
      I0 => icmp_ln35_reg_289_pp0_iter2_reg,
      I1 => ap_enable_reg_pp0_iter3,
      I2 => calibrated_load_reg_279_pp0_iter2_reg,
      I3 => \^b_v_data_1_state_reg[0]_1\,
      O => \icmp_ln35_reg_289_pp0_iter2_reg_reg[0]\(0)
    );
\icmp_ln35_reg_289[0]_i_12\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0001"
    )
        port map (
      I0 => add_ln33_fu_161_p2(27),
      I1 => add_ln33_fu_161_p2(26),
      I2 => add_ln33_fu_161_p2(25),
      I3 => add_ln33_fu_161_p2(24),
      O => \icmp_ln35_reg_289[0]_i_12_n_0\
    );
\icmp_ln35_reg_289[0]_i_14\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0001"
    )
        port map (
      I0 => add_ln33_fu_161_p2(23),
      I1 => add_ln33_fu_161_p2(22),
      I2 => add_ln33_fu_161_p2(21),
      I3 => add_ln33_fu_161_p2(20),
      O => \icmp_ln35_reg_289[0]_i_14_n_0\
    );
\icmp_ln35_reg_289[0]_i_2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00100000"
    )
        port map (
      I0 => add_ln33_fu_161_p2(13),
      I1 => add_ln33_fu_161_p2(14),
      I2 => \out\(0),
      I3 => add_ln33_fu_161_p2(15),
      I4 => \icmp_ln35_reg_289[0]_i_7_n_0\,
      O => \^counter_reg[0]\
    );
\icmp_ln35_reg_289[0]_i_3\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00010000"
    )
        port map (
      I0 => add_ln33_fu_161_p2(3),
      I1 => add_ln33_fu_161_p2(4),
      I2 => add_ln33_fu_161_p2(1),
      I3 => add_ln33_fu_161_p2(2),
      I4 => \icmp_ln35_reg_289[0]_i_9_n_0\,
      O => \^counter_reg[0]_0\
    );
\icmp_ln35_reg_289[0]_i_4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00010000"
    )
        port map (
      I0 => add_ln33_fu_161_p2(28),
      I1 => add_ln33_fu_161_p2(29),
      I2 => add_ln33_fu_161_p2(30),
      I3 => add_ln33_fu_161_p2(31),
      I4 => \icmp_ln35_reg_289[0]_i_12_n_0\,
      O => \^counter_reg[28]\
    );
\icmp_ln35_reg_289[0]_i_5\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"00010000"
    )
        port map (
      I0 => add_ln33_fu_161_p2(18),
      I1 => add_ln33_fu_161_p2(19),
      I2 => add_ln33_fu_161_p2(16),
      I3 => add_ln33_fu_161_p2(17),
      I4 => \icmp_ln35_reg_289[0]_i_14_n_0\,
      O => \^counter_reg[20]\
    );
\icmp_ln35_reg_289[0]_i_7\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0100"
    )
        port map (
      I0 => add_ln33_fu_161_p2(12),
      I1 => add_ln33_fu_161_p2(11),
      I2 => add_ln33_fu_161_p2(10),
      I3 => add_ln33_fu_161_p2(9),
      O => \icmp_ln35_reg_289[0]_i_7_n_0\
    );
\icmp_ln35_reg_289[0]_i_9\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"0400"
    )
        port map (
      I0 => add_ln33_fu_161_p2(7),
      I1 => add_ln33_fu_161_p2(8),
      I2 => add_ln33_fu_161_p2(6),
      I3 => add_ln33_fu_161_p2(5),
      O => \icmp_ln35_reg_289[0]_i_9_n_0\
    );
\icmp_ln35_reg_289_reg[0]_i_10\: unisim.vcomponents.CARRY4
     port map (
      CI => \icmp_ln35_reg_289_reg[0]_i_17_n_0\,
      CO(3) => \icmp_ln35_reg_289_reg[0]_i_10_n_0\,
      CO(2) => \icmp_ln35_reg_289_reg[0]_i_10_n_1\,
      CO(1) => \icmp_ln35_reg_289_reg[0]_i_10_n_2\,
      CO(0) => \icmp_ln35_reg_289_reg[0]_i_10_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln33_fu_161_p2(28 downto 25),
      S(3 downto 0) => \out\(28 downto 25)
    );
\icmp_ln35_reg_289_reg[0]_i_11\: unisim.vcomponents.CARRY4
     port map (
      CI => \icmp_ln35_reg_289_reg[0]_i_10_n_0\,
      CO(3 downto 2) => \NLW_icmp_ln35_reg_289_reg[0]_i_11_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \icmp_ln35_reg_289_reg[0]_i_11_n_2\,
      CO(0) => \icmp_ln35_reg_289_reg[0]_i_11_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_icmp_ln35_reg_289_reg[0]_i_11_O_UNCONNECTED\(3),
      O(2 downto 0) => add_ln33_fu_161_p2(31 downto 29),
      S(3) => '0',
      S(2 downto 0) => \out\(31 downto 29)
    );
\icmp_ln35_reg_289_reg[0]_i_13\: unisim.vcomponents.CARRY4
     port map (
      CI => \icmp_ln35_reg_289_reg[0]_i_6_n_0\,
      CO(3) => \icmp_ln35_reg_289_reg[0]_i_13_n_0\,
      CO(2) => \icmp_ln35_reg_289_reg[0]_i_13_n_1\,
      CO(1) => \icmp_ln35_reg_289_reg[0]_i_13_n_2\,
      CO(0) => \icmp_ln35_reg_289_reg[0]_i_13_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln33_fu_161_p2(20 downto 17),
      S(3 downto 0) => \out\(20 downto 17)
    );
\icmp_ln35_reg_289_reg[0]_i_15\: unisim.vcomponents.CARRY4
     port map (
      CI => \icmp_ln35_reg_289_reg[0]_i_16_n_0\,
      CO(3) => \icmp_ln35_reg_289_reg[0]_i_15_n_0\,
      CO(2) => \icmp_ln35_reg_289_reg[0]_i_15_n_1\,
      CO(1) => \icmp_ln35_reg_289_reg[0]_i_15_n_2\,
      CO(0) => \icmp_ln35_reg_289_reg[0]_i_15_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln33_fu_161_p2(12 downto 9),
      S(3 downto 0) => \out\(12 downto 9)
    );
\icmp_ln35_reg_289_reg[0]_i_16\: unisim.vcomponents.CARRY4
     port map (
      CI => \icmp_ln35_reg_289_reg[0]_i_8_n_0\,
      CO(3) => \icmp_ln35_reg_289_reg[0]_i_16_n_0\,
      CO(2) => \icmp_ln35_reg_289_reg[0]_i_16_n_1\,
      CO(1) => \icmp_ln35_reg_289_reg[0]_i_16_n_2\,
      CO(0) => \icmp_ln35_reg_289_reg[0]_i_16_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln33_fu_161_p2(8 downto 5),
      S(3 downto 0) => \out\(8 downto 5)
    );
\icmp_ln35_reg_289_reg[0]_i_17\: unisim.vcomponents.CARRY4
     port map (
      CI => \icmp_ln35_reg_289_reg[0]_i_13_n_0\,
      CO(3) => \icmp_ln35_reg_289_reg[0]_i_17_n_0\,
      CO(2) => \icmp_ln35_reg_289_reg[0]_i_17_n_1\,
      CO(1) => \icmp_ln35_reg_289_reg[0]_i_17_n_2\,
      CO(0) => \icmp_ln35_reg_289_reg[0]_i_17_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln33_fu_161_p2(24 downto 21),
      S(3 downto 0) => \out\(24 downto 21)
    );
\icmp_ln35_reg_289_reg[0]_i_6\: unisim.vcomponents.CARRY4
     port map (
      CI => \icmp_ln35_reg_289_reg[0]_i_15_n_0\,
      CO(3) => \icmp_ln35_reg_289_reg[0]_i_6_n_0\,
      CO(2) => \icmp_ln35_reg_289_reg[0]_i_6_n_1\,
      CO(1) => \icmp_ln35_reg_289_reg[0]_i_6_n_2\,
      CO(0) => \icmp_ln35_reg_289_reg[0]_i_6_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln33_fu_161_p2(16 downto 13),
      S(3 downto 0) => \out\(16 downto 13)
    );
\icmp_ln35_reg_289_reg[0]_i_8\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \icmp_ln35_reg_289_reg[0]_i_8_n_0\,
      CO(2) => \icmp_ln35_reg_289_reg[0]_i_8_n_1\,
      CO(1) => \icmp_ln35_reg_289_reg[0]_i_8_n_2\,
      CO(0) => \icmp_ln35_reg_289_reg[0]_i_8_n_3\,
      CYINIT => \out\(0),
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => add_ln33_fu_161_p2(4 downto 1),
      S(3 downto 0) => \out\(4 downto 1)
    );
\mul_ln39_reg_304[41]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"04"
    )
        port map (
      I0 => calibrated_load_reg_279_pp0_iter1_reg,
      I1 => icmp_ln35_reg_289_pp0_iter1_reg,
      I2 => \^b_v_data_1_state_reg[0]_1\,
      O => E(0)
    );
\out_stream_TDATA[0]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[0]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[0]\,
      O => out_stream_TDATA(0)
    );
\out_stream_TDATA[10]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[10]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[10]\,
      O => out_stream_TDATA(10)
    );
\out_stream_TDATA[11]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[11]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[11]\,
      O => out_stream_TDATA(11)
    );
\out_stream_TDATA[12]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[12]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[12]\,
      O => out_stream_TDATA(12)
    );
\out_stream_TDATA[13]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[13]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[13]\,
      O => out_stream_TDATA(13)
    );
\out_stream_TDATA[14]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[14]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[14]\,
      O => out_stream_TDATA(14)
    );
\out_stream_TDATA[15]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[15]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[15]\,
      O => out_stream_TDATA(15)
    );
\out_stream_TDATA[1]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[1]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[1]\,
      O => out_stream_TDATA(1)
    );
\out_stream_TDATA[2]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[2]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[2]\,
      O => out_stream_TDATA(2)
    );
\out_stream_TDATA[3]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[3]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[3]\,
      O => out_stream_TDATA(3)
    );
\out_stream_TDATA[4]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[4]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[4]\,
      O => out_stream_TDATA(4)
    );
\out_stream_TDATA[5]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[5]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[5]\,
      O => out_stream_TDATA(5)
    );
\out_stream_TDATA[6]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[6]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[6]\,
      O => out_stream_TDATA(6)
    );
\out_stream_TDATA[7]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[7]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[7]\,
      O => out_stream_TDATA(7)
    );
\out_stream_TDATA[8]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[8]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[8]\,
      O => out_stream_TDATA(8)
    );
\out_stream_TDATA[9]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => \B_V_data_1_payload_B_reg_n_0_[9]\,
      I1 => B_V_data_1_sel_rd_reg_n_0,
      I2 => \B_V_data_1_payload_A_reg_n_0_[9]\,
      O => out_stream_TDATA(9)
    );
\tmp_reg_298[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFEF0020"
    )
        port map (
      I0 => p_0_in,
      I1 => calibrated_load_reg_279,
      I2 => icmp_ln35_reg_289,
      I3 => \^b_v_data_1_state_reg[0]_1\,
      I4 => tmp_reg_298,
      O => \add_ln32_reg_283_reg[31]\
    );
\tmp_reg_298_pp0_iter2_reg[0]_i_1\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => \^b_v_data_1_state_reg[0]_1\,
      O => ap_block_pp0_stage0_11001
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0\ is
  port (
    in_stream_TKEEP_int_regslice : out STD_LOGIC_VECTOR ( 3 downto 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_0\ : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0\ : entity is "fsk_phase_corrector_regslice_both";
end \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0\;

architecture STRUCTURE of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0\ is
  signal B_V_data_1_load_A : STD_LOGIC;
  signal B_V_data_1_load_B : STD_LOGIC;
  signal B_V_data_1_payload_A : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal B_V_data_1_payload_B : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__4_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__2_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__1\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__2\ : label is "soft_lutpair9";
  attribute SOFT_HLUTNM of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_i_1\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_i_1\ : label is "soft_lutpair10";
  attribute SOFT_HLUTNM of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_i_1\ : label is "soft_lutpair11";
  attribute SOFT_HLUTNM of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_i_1\ : label is "soft_lutpair11";
begin
\B_V_data_1_payload_A[3]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"0D"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => B_V_data_1_load_A
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TKEEP(0),
      Q => B_V_data_1_payload_A(0),
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TKEEP(1),
      Q => B_V_data_1_payload_A(1),
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TKEEP(2),
      Q => B_V_data_1_payload_A(2),
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TKEEP(3),
      Q => B_V_data_1_payload_A(3),
      R => '0'
    );
\B_V_data_1_payload_B[3]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A2"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TKEEP(0),
      Q => B_V_data_1_payload_B(0),
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TKEEP(1),
      Q => B_V_data_1_payload_B(1),
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TKEEP(2),
      Q => B_V_data_1_payload_B(2),
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TKEEP(3),
      Q => B_V_data_1_payload_B(3),
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B4"
    )
        port map (
      I0 => \B_V_data_1_state_reg[0]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__1_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__1_n_0\,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => in_stream_TVALID,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__4_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__4_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1__2\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"AAA080A0"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg[0]_0\,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => in_stream_TVALID,
      O => \B_V_data_1_state[0]_i_1__2_n_0\
    );
\B_V_data_1_state[1]_i_1__2\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"77F7"
    )
        port map (
      I0 => \B_V_data_1_state_reg[0]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => in_stream_TVALID,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__2_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(0),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(0),
      O => in_stream_TKEEP_int_regslice(0)
    );
\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(1),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(1),
      O => in_stream_TKEEP_int_regslice(1)
    );
\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(2),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(2),
      O => in_stream_TKEEP_int_regslice(2)
    );
\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(3),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(3),
      O => in_stream_TKEEP_int_regslice(3)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_0\ is
  port (
    in_stream_TSTRB_int_regslice : out STD_LOGIC_VECTOR ( 3 downto 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_0\ : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_0\ : entity is "fsk_phase_corrector_regslice_both";
end \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_0\;

architecture STRUCTURE of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_0\ is
  signal B_V_data_1_load_A : STD_LOGIC;
  signal B_V_data_1_load_B : STD_LOGIC;
  signal B_V_data_1_payload_A : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal B_V_data_1_payload_B : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__5_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__1_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__0\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__1\ : label is "soft_lutpair13";
  attribute SOFT_HLUTNM of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_i_1\ : label is "soft_lutpair14";
  attribute SOFT_HLUTNM of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_i_1\ : label is "soft_lutpair15";
  attribute SOFT_HLUTNM of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_i_1\ : label is "soft_lutpair15";
begin
\B_V_data_1_payload_A[3]_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"0D"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => B_V_data_1_load_A
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TSTRB(0),
      Q => B_V_data_1_payload_A(0),
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TSTRB(1),
      Q => B_V_data_1_payload_A(1),
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TSTRB(2),
      Q => B_V_data_1_payload_A(2),
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => in_stream_TSTRB(3),
      Q => B_V_data_1_payload_A(3),
      R => '0'
    );
\B_V_data_1_payload_B[3]_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A2"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TSTRB(0),
      Q => B_V_data_1_payload_B(0),
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TSTRB(1),
      Q => B_V_data_1_payload_B(1),
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TSTRB(2),
      Q => B_V_data_1_payload_B(2),
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => in_stream_TSTRB(3),
      Q => B_V_data_1_payload_B(3),
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B4"
    )
        port map (
      I0 => \B_V_data_1_state_reg[0]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__0_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__0_n_0\,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => in_stream_TVALID,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__5_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__5_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1__1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"AAA080A0"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg[0]_0\,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => in_stream_TVALID,
      O => \B_V_data_1_state[0]_i_1__1_n_0\
    );
\B_V_data_1_state[1]_i_1__1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"77F7"
    )
        port map (
      I0 => \B_V_data_1_state_reg[0]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => in_stream_TVALID,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__1_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(0),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(0),
      O => in_stream_TSTRB_int_regslice(0)
    );
\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(1),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(1),
      O => in_stream_TSTRB_int_regslice(1)
    );
\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(2),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(2),
      O => in_stream_TSTRB_int_regslice(2)
    );
\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(3),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(3),
      O => in_stream_TSTRB_int_regslice(3)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_2\ is
  port (
    out_stream_TKEEP : out STD_LOGIC_VECTOR ( 3 downto 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TVALID_int_regslice : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    D : in STD_LOGIC_VECTOR ( 3 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_2\ : entity is "fsk_phase_corrector_regslice_both";
end \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_2\;

architecture STRUCTURE of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_2\ is
  signal B_V_data_1_load_A : STD_LOGIC;
  signal B_V_data_1_load_B : STD_LOGIC;
  signal B_V_data_1_payload_A : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal B_V_data_1_payload_B : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__4_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__1_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__6_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__4\ : label is "soft_lutpair26";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__5\ : label is "soft_lutpair26";
  attribute SOFT_HLUTNM of \out_stream_TKEEP[0]_INST_0\ : label is "soft_lutpair27";
  attribute SOFT_HLUTNM of \out_stream_TKEEP[1]_INST_0\ : label is "soft_lutpair27";
  attribute SOFT_HLUTNM of \out_stream_TKEEP[2]_INST_0\ : label is "soft_lutpair28";
  attribute SOFT_HLUTNM of \out_stream_TKEEP[3]_INST_0\ : label is "soft_lutpair28";
begin
\B_V_data_1_payload_A[3]_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"0D"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => B_V_data_1_load_A
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(0),
      Q => B_V_data_1_payload_A(0),
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(1),
      Q => B_V_data_1_payload_A(1),
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(2),
      Q => B_V_data_1_payload_A(2),
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(3),
      Q => B_V_data_1_payload_A(3),
      R => '0'
    );
\B_V_data_1_payload_B[3]_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A2"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(0),
      Q => B_V_data_1_payload_B(0),
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(1),
      Q => B_V_data_1_payload_B(1),
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(2),
      Q => B_V_data_1_payload_B(2),
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(3),
      Q => B_V_data_1_payload_B(3),
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__4\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__4_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__4_n_0\,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TVALID_int_regslice,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__1_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__1_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1__6\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8A80888"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => out_stream_TREADY,
      I4 => out_stream_TVALID_int_regslice,
      O => \B_V_data_1_state[0]_i_1__6_n_0\
    );
\B_V_data_1_state[1]_i_1__5\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F5FD"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => out_stream_TREADY,
      I3 => out_stream_TVALID_int_regslice,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__6_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\out_stream_TKEEP[0]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(0),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(0),
      O => out_stream_TKEEP(0)
    );
\out_stream_TKEEP[1]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(1),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(1),
      O => out_stream_TKEEP(1)
    );
\out_stream_TKEEP[2]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(2),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(2),
      O => out_stream_TKEEP(2)
    );
\out_stream_TKEEP[3]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(3),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(3),
      O => out_stream_TKEEP(3)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_4\ is
  port (
    out_stream_TSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TVALID_int_regslice : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    D : in STD_LOGIC_VECTOR ( 3 downto 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_4\ : entity is "fsk_phase_corrector_regslice_both";
end \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_4\;

architecture STRUCTURE of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_4\ is
  signal B_V_data_1_load_A : STD_LOGIC;
  signal B_V_data_1_load_B : STD_LOGIC;
  signal B_V_data_1_payload_A : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal B_V_data_1_payload_B : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__5_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__5_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__5\ : label is "soft_lutpair30";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__4\ : label is "soft_lutpair30";
  attribute SOFT_HLUTNM of \out_stream_TSTRB[0]_INST_0\ : label is "soft_lutpair31";
  attribute SOFT_HLUTNM of \out_stream_TSTRB[1]_INST_0\ : label is "soft_lutpair31";
  attribute SOFT_HLUTNM of \out_stream_TSTRB[2]_INST_0\ : label is "soft_lutpair32";
  attribute SOFT_HLUTNM of \out_stream_TSTRB[3]_INST_0\ : label is "soft_lutpair32";
begin
\B_V_data_1_payload_A[3]_i_1__2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"0D"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => B_V_data_1_load_A
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(0),
      Q => B_V_data_1_payload_A(0),
      R => '0'
    );
\B_V_data_1_payload_A_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(1),
      Q => B_V_data_1_payload_A(1),
      R => '0'
    );
\B_V_data_1_payload_A_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(2),
      Q => B_V_data_1_payload_A(2),
      R => '0'
    );
\B_V_data_1_payload_A_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_A,
      D => D(3),
      Q => B_V_data_1_payload_A(3),
      R => '0'
    );
\B_V_data_1_payload_B[3]_i_1__2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"A2"
    )
        port map (
      I0 => B_V_data_1_sel_wr,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      O => B_V_data_1_load_B
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(0),
      Q => B_V_data_1_payload_B(0),
      R => '0'
    );
\B_V_data_1_payload_B_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(1),
      Q => B_V_data_1_payload_B(1),
      R => '0'
    );
\B_V_data_1_payload_B_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(2),
      Q => B_V_data_1_payload_B(2),
      R => '0'
    );
\B_V_data_1_payload_B_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => B_V_data_1_load_B,
      D => D(3),
      Q => B_V_data_1_payload_B(3),
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__5\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__5_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__5_n_0\,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TVALID_int_regslice,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__0_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__0_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1__5\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8A80888"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => out_stream_TREADY,
      I4 => out_stream_TVALID_int_regslice,
      O => \B_V_data_1_state[0]_i_1__5_n_0\
    );
\B_V_data_1_state[1]_i_1__4\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F5FD"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => out_stream_TREADY,
      I3 => out_stream_TVALID_int_regslice,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__5_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\out_stream_TSTRB[0]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(0),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(0),
      O => out_stream_TSTRB(0)
    );
\out_stream_TSTRB[1]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(1),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(1),
      O => out_stream_TSTRB(1)
    );
\out_stream_TSTRB[2]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(2),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(2),
      O => out_stream_TSTRB(2)
    );
\out_stream_TSTRB[3]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B(3),
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A(3),
      O => out_stream_TSTRB(3)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1\ is
  port (
    in_stream_TLAST_int_regslice : out STD_LOGIC;
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    \B_V_data_1_state_reg[0]_0\ : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1\ : entity is "fsk_phase_corrector_regslice_both";
end \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1\;

architecture STRUCTURE of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1\ is
  signal B_V_data_1_payload_A : STD_LOGIC;
  signal \B_V_data_1_payload_A[0]_i_1_n_0\ : STD_LOGIC;
  signal B_V_data_1_payload_B : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal B_V_data_1_sel_rd_i_1_n_0 : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal \B_V_data_1_sel_wr_i_1__6_n_0\ : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__0_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of B_V_data_1_sel_rd_i_1 : label is "soft_lutpair12";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__0\ : label is "soft_lutpair12";
begin
\B_V_data_1_payload_A[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFAE00A2"
    )
        port map (
      I0 => in_stream_TLAST(0),
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => B_V_data_1_sel_wr,
      I4 => B_V_data_1_payload_A,
      O => \B_V_data_1_payload_A[0]_i_1_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_A[0]_i_1_n_0\,
      Q => B_V_data_1_payload_A,
      R => '0'
    );
\B_V_data_1_payload_B[0]_i_1\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BBFB8808"
    )
        port map (
      I0 => in_stream_TLAST(0),
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => B_V_data_1_payload_B,
      O => \B_V_data_1_payload_B[0]_i_1_n_0\
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_B[0]_i_1_n_0\,
      Q => B_V_data_1_payload_B,
      R => '0'
    );
B_V_data_1_sel_rd_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B4"
    )
        port map (
      I0 => \B_V_data_1_state_reg[0]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      O => B_V_data_1_sel_rd_i_1_n_0
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_sel_rd_i_1_n_0,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
\B_V_data_1_sel_wr_i_1__6\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => in_stream_TVALID,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => \B_V_data_1_sel_wr_i_1__6_n_0\
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_wr_i_1__6_n_0\,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"AAA080A0"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg[0]_0\,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => in_stream_TVALID,
      O => \B_V_data_1_state[0]_i_1__0_n_0\
    );
\B_V_data_1_state[1]_i_1__0\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"77F7"
    )
        port map (
      I0 => \B_V_data_1_state_reg[0]_0\,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => in_stream_TVALID,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__0_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B,
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A,
      O => in_stream_TLAST_int_regslice
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1_3\ is
  port (
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    ap_rst_n_inv : in STD_LOGIC;
    ap_clk : in STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TVALID_int_regslice : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    pkt_in_last_V_reg_269_pp0_iter2_reg : in STD_LOGIC
  );
  attribute ORIG_REF_NAME : string;
  attribute ORIG_REF_NAME of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1_3\ : entity is "fsk_phase_corrector_regslice_both";
end \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1_3\;

architecture STRUCTURE of \decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1_3\ is
  signal B_V_data_1_payload_A : STD_LOGIC;
  signal \B_V_data_1_payload_A[0]_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_payload_B : STD_LOGIC;
  signal \B_V_data_1_payload_B[0]_i_1__0_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel : STD_LOGIC;
  signal \B_V_data_1_sel_rd_i_1__6_n_0\ : STD_LOGIC;
  signal B_V_data_1_sel_wr : STD_LOGIC;
  signal B_V_data_1_sel_wr_i_1_n_0 : STD_LOGIC;
  signal B_V_data_1_state : STD_LOGIC_VECTOR ( 1 to 1 );
  signal \B_V_data_1_state[0]_i_1__4_n_0\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[0]\ : STD_LOGIC;
  signal \B_V_data_1_state_reg_n_0_[1]\ : STD_LOGIC;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \B_V_data_1_sel_rd_i_1__6\ : label is "soft_lutpair29";
  attribute SOFT_HLUTNM of \B_V_data_1_state[1]_i_1__3\ : label is "soft_lutpair29";
begin
\B_V_data_1_payload_A[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"FFAE00A2"
    )
        port map (
      I0 => pkt_in_last_V_reg_269_pp0_iter2_reg,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => B_V_data_1_sel_wr,
      I4 => B_V_data_1_payload_A,
      O => \B_V_data_1_payload_A[0]_i_1__0_n_0\
    );
\B_V_data_1_payload_A_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_A[0]_i_1__0_n_0\,
      Q => B_V_data_1_payload_A,
      R => '0'
    );
\B_V_data_1_payload_B[0]_i_1__0\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"BBFB8808"
    )
        port map (
      I0 => pkt_in_last_V_reg_269_pp0_iter2_reg,
      I1 => B_V_data_1_sel_wr,
      I2 => \B_V_data_1_state_reg_n_0_[0]\,
      I3 => \B_V_data_1_state_reg_n_0_[1]\,
      I4 => B_V_data_1_payload_B,
      O => \B_V_data_1_payload_B[0]_i_1__0_n_0\
    );
\B_V_data_1_payload_B_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_payload_B[0]_i_1__0_n_0\,
      Q => B_V_data_1_payload_B,
      R => '0'
    );
\B_V_data_1_sel_rd_i_1__6\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TREADY,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => B_V_data_1_sel,
      O => \B_V_data_1_sel_rd_i_1__6_n_0\
    );
B_V_data_1_sel_rd_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_sel_rd_i_1__6_n_0\,
      Q => B_V_data_1_sel,
      R => ap_rst_n_inv
    );
B_V_data_1_sel_wr_i_1: unisim.vcomponents.LUT3
    generic map(
      INIT => X"78"
    )
        port map (
      I0 => out_stream_TVALID_int_regslice,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => B_V_data_1_sel_wr,
      O => B_V_data_1_sel_wr_i_1_n_0
    );
B_V_data_1_sel_wr_reg: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_sel_wr_i_1_n_0,
      Q => B_V_data_1_sel_wr,
      R => ap_rst_n_inv
    );
\B_V_data_1_state[0]_i_1__4\: unisim.vcomponents.LUT5
    generic map(
      INIT => X"A8A80888"
    )
        port map (
      I0 => ap_rst_n,
      I1 => \B_V_data_1_state_reg_n_0_[0]\,
      I2 => \B_V_data_1_state_reg_n_0_[1]\,
      I3 => out_stream_TREADY,
      I4 => out_stream_TVALID_int_regslice,
      O => \B_V_data_1_state[0]_i_1__4_n_0\
    );
\B_V_data_1_state[1]_i_1__3\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"F5FD"
    )
        port map (
      I0 => \B_V_data_1_state_reg_n_0_[0]\,
      I1 => \B_V_data_1_state_reg_n_0_[1]\,
      I2 => out_stream_TREADY,
      I3 => out_stream_TVALID_int_regslice,
      O => B_V_data_1_state(1)
    );
\B_V_data_1_state_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => \B_V_data_1_state[0]_i_1__4_n_0\,
      Q => \B_V_data_1_state_reg_n_0_[0]\,
      R => '0'
    );
\B_V_data_1_state_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => B_V_data_1_state(1),
      Q => \B_V_data_1_state_reg_n_0_[1]\,
      R => ap_rst_n_inv
    );
\out_stream_TLAST[0]_INST_0\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"B8"
    )
        port map (
      I0 => B_V_data_1_payload_B,
      I1 => B_V_data_1_sel,
      I2 => B_V_data_1_payload_A,
      O => out_stream_TLAST(0)
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector is
  port (
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TREADY : out STD_LOGIC;
    in_stream_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    in_stream_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    out_stream_TVALID : out STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TKEEP : out STD_LOGIC_VECTOR ( 3 downto 0 );
    out_stream_TSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 );
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 )
  );
  attribute ap_ST_fsm_pp0_stage0 : string;
  attribute ap_ST_fsm_pp0_stage0 of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector : entity is "1'b1";
  attribute hls_module : string;
  attribute hls_module of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector : entity is "yes";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector is
  signal add_ln32_fu_145_p2 : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal add_ln32_reg_2830 : STD_LOGIC;
  signal add_ln33_fu_161_p2 : STD_LOGIC_VECTOR ( 0 to 0 );
  signal ap_block_pp0_stage0_11001 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter1 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter2 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter3 : STD_LOGIC;
  signal ap_enable_reg_pp0_iter4 : STD_LOGIC;
  signal ap_rst_n_inv : STD_LOGIC;
  signal \buff0_reg__1\ : STD_LOGIC_VECTOR ( 57 downto 16 );
  signal calibrated : STD_LOGIC;
  signal calibrated_load_reg_279 : STD_LOGIC;
  signal calibrated_load_reg_279_pp0_iter1_reg : STD_LOGIC;
  signal calibrated_load_reg_279_pp0_iter2_reg : STD_LOGIC;
  signal calibrated_load_reg_279_pp0_iter3_reg : STD_LOGIC;
  signal counter_reg : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal \counter_reg[0]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[0]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[12]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[16]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[20]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[24]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[28]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[4]_i_1_n_7\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_0\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_1\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_2\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_3\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_4\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_5\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_6\ : STD_LOGIC;
  signal \counter_reg[8]_i_1_n_7\ : STD_LOGIC;
  signal dc_offset : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal dc_offset0 : STD_LOGIC;
  signal \dc_offset[0]_i_10_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_11_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_12_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_14_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_15_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_16_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_17_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_19_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_20_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_21_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_22_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_24_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_25_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_26_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_27_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_29_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_30_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_31_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_32_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_34_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_35_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_36_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_37_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_39_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_40_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_41_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_42_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_44_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_45_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_46_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_47_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_49_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_4_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_50_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_51_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_52_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_53_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_54_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_55_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_5_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_6_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_7_n_0\ : STD_LOGIC;
  signal \dc_offset[0]_i_9_n_0\ : STD_LOGIC;
  signal \dc_offset[12]_i_10_n_0\ : STD_LOGIC;
  signal \dc_offset[12]_i_11_n_0\ : STD_LOGIC;
  signal \dc_offset[12]_i_3_n_0\ : STD_LOGIC;
  signal \dc_offset[12]_i_4_n_0\ : STD_LOGIC;
  signal \dc_offset[12]_i_5_n_0\ : STD_LOGIC;
  signal \dc_offset[12]_i_6_n_0\ : STD_LOGIC;
  signal \dc_offset[12]_i_8_n_0\ : STD_LOGIC;
  signal \dc_offset[12]_i_9_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_10_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_11_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_12_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_13_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_14_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_4_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_5_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_6_n_0\ : STD_LOGIC;
  signal \dc_offset[15]_i_9_n_0\ : STD_LOGIC;
  signal \dc_offset[4]_i_3_n_0\ : STD_LOGIC;
  signal \dc_offset[4]_i_4_n_0\ : STD_LOGIC;
  signal \dc_offset[4]_i_5_n_0\ : STD_LOGIC;
  signal \dc_offset[4]_i_6_n_0\ : STD_LOGIC;
  signal \dc_offset[4]_i_7_n_0\ : STD_LOGIC;
  signal \dc_offset[8]_i_10_n_0\ : STD_LOGIC;
  signal \dc_offset[8]_i_11_n_0\ : STD_LOGIC;
  signal \dc_offset[8]_i_3_n_0\ : STD_LOGIC;
  signal \dc_offset[8]_i_4_n_0\ : STD_LOGIC;
  signal \dc_offset[8]_i_5_n_0\ : STD_LOGIC;
  signal \dc_offset[8]_i_6_n_0\ : STD_LOGIC;
  signal \dc_offset[8]_i_8_n_0\ : STD_LOGIC;
  signal \dc_offset[8]_i_9_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_13_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_13_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_13_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_13_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_18_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_18_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_18_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_18_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_23_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_23_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_23_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_23_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_28_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_28_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_28_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_28_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_2_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_2_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_2_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_2_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_33_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_33_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_33_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_33_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_38_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_38_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_38_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_38_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_3_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_3_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_3_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_3_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_43_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_43_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_43_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_43_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_48_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_48_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_48_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_48_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_8_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_8_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_8_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[0]_i_8_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[12]_i_2_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[12]_i_2_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[12]_i_2_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[12]_i_2_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[12]_i_7_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[12]_i_7_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[12]_i_7_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[12]_i_7_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[15]_i_3_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[15]_i_3_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[15]_i_7_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[15]_i_8_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[15]_i_8_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[15]_i_8_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[15]_i_8_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[4]_i_2_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[4]_i_2_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[4]_i_2_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[4]_i_2_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[8]_i_2_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[8]_i_2_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[8]_i_2_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[8]_i_2_n_3\ : STD_LOGIC;
  signal \dc_offset_reg[8]_i_7_n_0\ : STD_LOGIC;
  signal \dc_offset_reg[8]_i_7_n_1\ : STD_LOGIC;
  signal \dc_offset_reg[8]_i_7_n_2\ : STD_LOGIC;
  signal \dc_offset_reg[8]_i_7_n_3\ : STD_LOGIC;
  signal icmp_ln35_fu_173_p2 : STD_LOGIC;
  signal icmp_ln35_reg_289 : STD_LOGIC;
  signal icmp_ln35_reg_289_pp0_iter1_reg : STD_LOGIC;
  signal icmp_ln35_reg_289_pp0_iter2_reg : STD_LOGIC;
  signal \in\ : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal in_stream_TKEEP_int_regslice : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal in_stream_TLAST_int_regslice : STD_LOGIC;
  signal in_stream_TSTRB_int_regslice : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal in_stream_TVALID_int_regslice : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_0 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_1 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_10 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_11 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_12 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_13 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_14 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_2 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_3 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_4 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_41 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_42 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_43 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_44 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_45 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_46 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_47 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_48 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_49 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_5 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_50 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_51 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_52 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_53 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_54 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_55 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_56 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_6 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_7 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_8 : STD_LOGIC;
  signal mul_32s_34ns_65_2_1_U1_n_9 : STD_LOGIC;
  signal mul_ln39_reg_304 : STD_LOGIC_VECTOR ( 55 downto 0 );
  signal mul_ln39_reg_3040 : STD_LOGIC;
  signal \^out_stream_tdata\ : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal out_stream_TVALID_int_regslice : STD_LOGIC;
  signal p_0_in : STD_LOGIC;
  signal \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_n_0\ : STD_LOGIC;
  signal \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_n_0\ : STD_LOGIC;
  signal \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_n_0\ : STD_LOGIC;
  signal \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_n_0\ : STD_LOGIC;
  signal pkt_in_keep_V_reg_259_pp0_iter2_reg : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2_n_0\ : STD_LOGIC;
  signal pkt_in_last_V_reg_269_pp0_iter2_reg : STD_LOGIC;
  signal \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_n_0\ : STD_LOGIC;
  signal \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_n_0\ : STD_LOGIC;
  signal \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_n_0\ : STD_LOGIC;
  signal \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_n_0\ : STD_LOGIC;
  signal pkt_in_strb_V_reg_264_pp0_iter2_reg : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal regslice_both_in_stream_V_data_V_U_n_19 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_20 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_21 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_22 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_23 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_24 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_25 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_26 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_27 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_28 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_29 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_30 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_31 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_32 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_33 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_34 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_35 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_36 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_37 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_38 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_39 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_40 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_41 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_42 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_43 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_44 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_45 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_46 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_47 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_48 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_49 : STD_LOGIC;
  signal regslice_both_in_stream_V_data_V_U_n_50 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_12 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_3 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_5 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_6 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_7 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_8 : STD_LOGIC;
  signal regslice_both_out_stream_V_data_V_U_n_9 : STD_LOGIC;
  signal select_ln39_1_fu_233_p3 : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal sub_ln39_1_fu_227_p2 : STD_LOGIC_VECTOR ( 15 downto 1 );
  signal sub_ln39_fu_206_p2 : STD_LOGIC_VECTOR ( 57 downto 42 );
  signal sum_reg : STD_LOGIC_VECTOR ( 31 downto 0 );
  signal tmp_2_reg_309 : STD_LOGIC_VECTOR ( 15 downto 14 );
  signal tmp_reg_298 : STD_LOGIC;
  signal tmp_reg_298_pp0_iter2_reg : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_n_0\ : STD_LOGIC;
  signal \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_n_0\ : STD_LOGIC;
  signal val_in_reg_274_pp0_iter2_reg : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal \NLW_counter_reg[28]_i_1_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_dc_offset_reg[0]_i_13_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_18_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_2_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 1 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_23_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_28_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_3_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_33_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_38_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_43_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_48_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[0]_i_8_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 0 );
  signal \NLW_dc_offset_reg[15]_i_3_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  signal \NLW_dc_offset_reg[15]_i_3_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 to 3 );
  signal \NLW_dc_offset_reg[15]_i_7_CO_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 1 );
  signal \NLW_dc_offset_reg[15]_i_7_O_UNCONNECTED\ : STD_LOGIC_VECTOR ( 3 downto 2 );
  attribute ADDER_THRESHOLD : integer;
  attribute ADDER_THRESHOLD of \counter_reg[0]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[12]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[16]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[20]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[24]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[28]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[4]_i_1\ : label is 11;
  attribute ADDER_THRESHOLD of \counter_reg[8]_i_1\ : label is 11;
  attribute SOFT_HLUTNM : string;
  attribute SOFT_HLUTNM of \dc_offset[0]_i_1\ : label is "soft_lutpair40";
  attribute SOFT_HLUTNM of \dc_offset[10]_i_1\ : label is "soft_lutpair34";
  attribute SOFT_HLUTNM of \dc_offset[11]_i_1\ : label is "soft_lutpair35";
  attribute SOFT_HLUTNM of \dc_offset[12]_i_1\ : label is "soft_lutpair35";
  attribute SOFT_HLUTNM of \dc_offset[13]_i_1\ : label is "soft_lutpair34";
  attribute SOFT_HLUTNM of \dc_offset[14]_i_1\ : label is "soft_lutpair33";
  attribute SOFT_HLUTNM of \dc_offset[15]_i_2\ : label is "soft_lutpair33";
  attribute SOFT_HLUTNM of \dc_offset[1]_i_1\ : label is "soft_lutpair40";
  attribute SOFT_HLUTNM of \dc_offset[2]_i_1\ : label is "soft_lutpair39";
  attribute SOFT_HLUTNM of \dc_offset[3]_i_1\ : label is "soft_lutpair39";
  attribute SOFT_HLUTNM of \dc_offset[4]_i_1\ : label is "soft_lutpair38";
  attribute SOFT_HLUTNM of \dc_offset[5]_i_1\ : label is "soft_lutpair38";
  attribute SOFT_HLUTNM of \dc_offset[6]_i_1\ : label is "soft_lutpair36";
  attribute SOFT_HLUTNM of \dc_offset[7]_i_1\ : label is "soft_lutpair37";
  attribute SOFT_HLUTNM of \dc_offset[8]_i_1\ : label is "soft_lutpair37";
  attribute SOFT_HLUTNM of \dc_offset[9]_i_1\ : label is "soft_lutpair36";
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_13\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_18\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_23\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_28\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_3\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_33\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_38\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_43\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_48\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[0]_i_8\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[12]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[12]_i_7\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[15]_i_3\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[15]_i_7\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[15]_i_8\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[4]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[8]_i_2\ : label is 35;
  attribute ADDER_THRESHOLD of \dc_offset_reg[8]_i_7\ : label is 35;
  attribute srl_bus_name : string;
  attribute srl_bus_name of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg ";
  attribute srl_name : string;
  attribute srl_name of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2 ";
  attribute srl_bus_name of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2\ : label is "inst/\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg ";
  attribute srl_name of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2\ : label is "inst/\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2 ";
  attribute srl_bus_name of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2\ : label is "inst/\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg ";
  attribute srl_name of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2\ : label is "inst/\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2 ";
  attribute srl_bus_name of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2\ : label is "inst/\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg ";
  attribute srl_name of \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2\ : label is "inst/\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2 ";
  attribute srl_bus_name of \pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\pkt_in_last_V_reg_269_pp0_iter1_reg_reg ";
  attribute srl_name of \pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2 ";
  attribute srl_bus_name of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg ";
  attribute srl_name of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2 ";
  attribute srl_bus_name of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2\ : label is "inst/\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg ";
  attribute srl_name of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2\ : label is "inst/\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2 ";
  attribute srl_bus_name of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2\ : label is "inst/\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg ";
  attribute srl_name of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2\ : label is "inst/\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2 ";
  attribute srl_bus_name of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2\ : label is "inst/\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg ";
  attribute srl_name of \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2\ : label is "inst/\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[0]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[10]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[11]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[12]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[13]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[14]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[15]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[1]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[2]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[3]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[4]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[5]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[6]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[7]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[8]_srl2 ";
  attribute srl_bus_name of \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg ";
  attribute srl_name of \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2\ : label is "inst/\val_in_reg_274_pp0_iter1_reg_reg[9]_srl2 ";
begin
  out_stream_TDATA(31) <= \^out_stream_tdata\(31);
  out_stream_TDATA(30) <= \^out_stream_tdata\(31);
  out_stream_TDATA(29) <= \^out_stream_tdata\(31);
  out_stream_TDATA(28) <= \^out_stream_tdata\(31);
  out_stream_TDATA(27) <= \^out_stream_tdata\(31);
  out_stream_TDATA(26) <= \^out_stream_tdata\(31);
  out_stream_TDATA(25) <= \^out_stream_tdata\(31);
  out_stream_TDATA(24) <= \^out_stream_tdata\(31);
  out_stream_TDATA(23) <= \^out_stream_tdata\(31);
  out_stream_TDATA(22) <= \^out_stream_tdata\(31);
  out_stream_TDATA(21) <= \^out_stream_tdata\(31);
  out_stream_TDATA(20) <= \^out_stream_tdata\(31);
  out_stream_TDATA(19) <= \^out_stream_tdata\(31);
  out_stream_TDATA(18) <= \^out_stream_tdata\(31);
  out_stream_TDATA(17) <= \^out_stream_tdata\(31);
  out_stream_TDATA(16) <= \^out_stream_tdata\(31);
  out_stream_TDATA(15) <= \^out_stream_tdata\(31);
  out_stream_TDATA(14 downto 0) <= \^out_stream_tdata\(14 downto 0);
\add_ln32_reg_283_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => add_ln32_fu_145_p2(31),
      Q => p_0_in,
      R => '0'
    );
ap_enable_reg_pp0_iter1_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => '1',
      Q => ap_enable_reg_pp0_iter1,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter2_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => ap_enable_reg_pp0_iter1,
      Q => ap_enable_reg_pp0_iter2,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter3_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => ap_enable_reg_pp0_iter2,
      Q => ap_enable_reg_pp0_iter3,
      R => ap_rst_n_inv
    );
ap_enable_reg_pp0_iter4_reg: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => ap_enable_reg_pp0_iter3,
      Q => ap_enable_reg_pp0_iter4,
      R => ap_rst_n_inv
    );
\calibrated_load_reg_279_pp0_iter1_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => calibrated_load_reg_279,
      Q => calibrated_load_reg_279_pp0_iter1_reg,
      R => '0'
    );
\calibrated_load_reg_279_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => calibrated_load_reg_279_pp0_iter1_reg,
      Q => calibrated_load_reg_279_pp0_iter2_reg,
      R => '0'
    );
\calibrated_load_reg_279_pp0_iter3_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => calibrated_load_reg_279_pp0_iter2_reg,
      Q => calibrated_load_reg_279_pp0_iter3_reg,
      R => '0'
    );
\calibrated_load_reg_279_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => calibrated,
      Q => calibrated_load_reg_279,
      R => '0'
    );
\calibrated_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_out_stream_V_data_V_U_n_5,
      Q => calibrated,
      R => '0'
    );
\counter[0]_i_2\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => counter_reg(0),
      O => add_ln33_fu_161_p2(0)
    );
\counter_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[0]_i_1_n_7\,
      Q => counter_reg(0),
      R => '0'
    );
\counter_reg[0]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \counter_reg[0]_i_1_n_0\,
      CO(2) => \counter_reg[0]_i_1_n_1\,
      CO(1) => \counter_reg[0]_i_1_n_2\,
      CO(0) => \counter_reg[0]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0001",
      O(3) => \counter_reg[0]_i_1_n_4\,
      O(2) => \counter_reg[0]_i_1_n_5\,
      O(1) => \counter_reg[0]_i_1_n_6\,
      O(0) => \counter_reg[0]_i_1_n_7\,
      S(3 downto 1) => counter_reg(3 downto 1),
      S(0) => add_ln33_fu_161_p2(0)
    );
\counter_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[8]_i_1_n_5\,
      Q => counter_reg(10),
      R => '0'
    );
\counter_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[8]_i_1_n_4\,
      Q => counter_reg(11),
      R => '0'
    );
\counter_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[12]_i_1_n_7\,
      Q => counter_reg(12),
      R => '0'
    );
\counter_reg[12]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[8]_i_1_n_0\,
      CO(3) => \counter_reg[12]_i_1_n_0\,
      CO(2) => \counter_reg[12]_i_1_n_1\,
      CO(1) => \counter_reg[12]_i_1_n_2\,
      CO(0) => \counter_reg[12]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[12]_i_1_n_4\,
      O(2) => \counter_reg[12]_i_1_n_5\,
      O(1) => \counter_reg[12]_i_1_n_6\,
      O(0) => \counter_reg[12]_i_1_n_7\,
      S(3 downto 0) => counter_reg(15 downto 12)
    );
\counter_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[12]_i_1_n_6\,
      Q => counter_reg(13),
      R => '0'
    );
\counter_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[12]_i_1_n_5\,
      Q => counter_reg(14),
      R => '0'
    );
\counter_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[12]_i_1_n_4\,
      Q => counter_reg(15),
      R => '0'
    );
\counter_reg[16]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[16]_i_1_n_7\,
      Q => counter_reg(16),
      R => '0'
    );
\counter_reg[16]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[12]_i_1_n_0\,
      CO(3) => \counter_reg[16]_i_1_n_0\,
      CO(2) => \counter_reg[16]_i_1_n_1\,
      CO(1) => \counter_reg[16]_i_1_n_2\,
      CO(0) => \counter_reg[16]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[16]_i_1_n_4\,
      O(2) => \counter_reg[16]_i_1_n_5\,
      O(1) => \counter_reg[16]_i_1_n_6\,
      O(0) => \counter_reg[16]_i_1_n_7\,
      S(3 downto 0) => counter_reg(19 downto 16)
    );
\counter_reg[17]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[16]_i_1_n_6\,
      Q => counter_reg(17),
      R => '0'
    );
\counter_reg[18]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[16]_i_1_n_5\,
      Q => counter_reg(18),
      R => '0'
    );
\counter_reg[19]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[16]_i_1_n_4\,
      Q => counter_reg(19),
      R => '0'
    );
\counter_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[0]_i_1_n_6\,
      Q => counter_reg(1),
      R => '0'
    );
\counter_reg[20]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[20]_i_1_n_7\,
      Q => counter_reg(20),
      R => '0'
    );
\counter_reg[20]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[16]_i_1_n_0\,
      CO(3) => \counter_reg[20]_i_1_n_0\,
      CO(2) => \counter_reg[20]_i_1_n_1\,
      CO(1) => \counter_reg[20]_i_1_n_2\,
      CO(0) => \counter_reg[20]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[20]_i_1_n_4\,
      O(2) => \counter_reg[20]_i_1_n_5\,
      O(1) => \counter_reg[20]_i_1_n_6\,
      O(0) => \counter_reg[20]_i_1_n_7\,
      S(3 downto 0) => counter_reg(23 downto 20)
    );
\counter_reg[21]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[20]_i_1_n_6\,
      Q => counter_reg(21),
      R => '0'
    );
\counter_reg[22]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[20]_i_1_n_5\,
      Q => counter_reg(22),
      R => '0'
    );
\counter_reg[23]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[20]_i_1_n_4\,
      Q => counter_reg(23),
      R => '0'
    );
\counter_reg[24]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[24]_i_1_n_7\,
      Q => counter_reg(24),
      R => '0'
    );
\counter_reg[24]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[20]_i_1_n_0\,
      CO(3) => \counter_reg[24]_i_1_n_0\,
      CO(2) => \counter_reg[24]_i_1_n_1\,
      CO(1) => \counter_reg[24]_i_1_n_2\,
      CO(0) => \counter_reg[24]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[24]_i_1_n_4\,
      O(2) => \counter_reg[24]_i_1_n_5\,
      O(1) => \counter_reg[24]_i_1_n_6\,
      O(0) => \counter_reg[24]_i_1_n_7\,
      S(3 downto 0) => counter_reg(27 downto 24)
    );
\counter_reg[25]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[24]_i_1_n_6\,
      Q => counter_reg(25),
      R => '0'
    );
\counter_reg[26]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[24]_i_1_n_5\,
      Q => counter_reg(26),
      R => '0'
    );
\counter_reg[27]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[24]_i_1_n_4\,
      Q => counter_reg(27),
      R => '0'
    );
\counter_reg[28]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[28]_i_1_n_7\,
      Q => counter_reg(28),
      R => '0'
    );
\counter_reg[28]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[24]_i_1_n_0\,
      CO(3) => \NLW_counter_reg[28]_i_1_CO_UNCONNECTED\(3),
      CO(2) => \counter_reg[28]_i_1_n_1\,
      CO(1) => \counter_reg[28]_i_1_n_2\,
      CO(0) => \counter_reg[28]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[28]_i_1_n_4\,
      O(2) => \counter_reg[28]_i_1_n_5\,
      O(1) => \counter_reg[28]_i_1_n_6\,
      O(0) => \counter_reg[28]_i_1_n_7\,
      S(3 downto 0) => counter_reg(31 downto 28)
    );
\counter_reg[29]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[28]_i_1_n_6\,
      Q => counter_reg(29),
      R => '0'
    );
\counter_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[0]_i_1_n_5\,
      Q => counter_reg(2),
      R => '0'
    );
\counter_reg[30]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[28]_i_1_n_5\,
      Q => counter_reg(30),
      R => '0'
    );
\counter_reg[31]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[28]_i_1_n_4\,
      Q => counter_reg(31),
      R => '0'
    );
\counter_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[0]_i_1_n_4\,
      Q => counter_reg(3),
      R => '0'
    );
\counter_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[4]_i_1_n_7\,
      Q => counter_reg(4),
      R => '0'
    );
\counter_reg[4]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[0]_i_1_n_0\,
      CO(3) => \counter_reg[4]_i_1_n_0\,
      CO(2) => \counter_reg[4]_i_1_n_1\,
      CO(1) => \counter_reg[4]_i_1_n_2\,
      CO(0) => \counter_reg[4]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[4]_i_1_n_4\,
      O(2) => \counter_reg[4]_i_1_n_5\,
      O(1) => \counter_reg[4]_i_1_n_6\,
      O(0) => \counter_reg[4]_i_1_n_7\,
      S(3 downto 0) => counter_reg(7 downto 4)
    );
\counter_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[4]_i_1_n_6\,
      Q => counter_reg(5),
      R => '0'
    );
\counter_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[4]_i_1_n_5\,
      Q => counter_reg(6),
      R => '0'
    );
\counter_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[4]_i_1_n_4\,
      Q => counter_reg(7),
      R => '0'
    );
\counter_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[8]_i_1_n_7\,
      Q => counter_reg(8),
      R => '0'
    );
\counter_reg[8]_i_1\: unisim.vcomponents.CARRY4
     port map (
      CI => \counter_reg[4]_i_1_n_0\,
      CO(3) => \counter_reg[8]_i_1_n_0\,
      CO(2) => \counter_reg[8]_i_1_n_1\,
      CO(1) => \counter_reg[8]_i_1_n_2\,
      CO(0) => \counter_reg[8]_i_1_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \counter_reg[8]_i_1_n_4\,
      O(2) => \counter_reg[8]_i_1_n_5\,
      O(1) => \counter_reg[8]_i_1_n_6\,
      O(0) => \counter_reg[8]_i_1_n_7\,
      S(3 downto 0) => counter_reg(11 downto 8)
    );
\counter_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => \counter_reg[8]_i_1_n_6\,
      Q => counter_reg(9),
      R => '0'
    );
\dc_offset[0]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(42),
      I1 => mul_ln39_reg_304(42),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(0)
    );
\dc_offset[0]_i_10\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(38),
      O => \dc_offset[0]_i_10_n_0\
    );
\dc_offset[0]_i_11\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(37),
      O => \dc_offset[0]_i_11_n_0\
    );
\dc_offset[0]_i_12\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(36),
      O => \dc_offset[0]_i_12_n_0\
    );
\dc_offset[0]_i_14\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(35),
      O => \dc_offset[0]_i_14_n_0\
    );
\dc_offset[0]_i_15\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(34),
      O => \dc_offset[0]_i_15_n_0\
    );
\dc_offset[0]_i_16\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(33),
      O => \dc_offset[0]_i_16_n_0\
    );
\dc_offset[0]_i_17\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(32),
      O => \dc_offset[0]_i_17_n_0\
    );
\dc_offset[0]_i_19\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(31),
      O => \dc_offset[0]_i_19_n_0\
    );
\dc_offset[0]_i_20\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(30),
      O => \dc_offset[0]_i_20_n_0\
    );
\dc_offset[0]_i_21\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(29),
      O => \dc_offset[0]_i_21_n_0\
    );
\dc_offset[0]_i_22\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(28),
      O => \dc_offset[0]_i_22_n_0\
    );
\dc_offset[0]_i_24\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(27),
      O => \dc_offset[0]_i_24_n_0\
    );
\dc_offset[0]_i_25\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(26),
      O => \dc_offset[0]_i_25_n_0\
    );
\dc_offset[0]_i_26\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(25),
      O => \dc_offset[0]_i_26_n_0\
    );
\dc_offset[0]_i_27\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(24),
      O => \dc_offset[0]_i_27_n_0\
    );
\dc_offset[0]_i_29\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(23),
      O => \dc_offset[0]_i_29_n_0\
    );
\dc_offset[0]_i_30\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(22),
      O => \dc_offset[0]_i_30_n_0\
    );
\dc_offset[0]_i_31\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(21),
      O => \dc_offset[0]_i_31_n_0\
    );
\dc_offset[0]_i_32\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(20),
      O => \dc_offset[0]_i_32_n_0\
    );
\dc_offset[0]_i_34\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(19),
      O => \dc_offset[0]_i_34_n_0\
    );
\dc_offset[0]_i_35\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(18),
      O => \dc_offset[0]_i_35_n_0\
    );
\dc_offset[0]_i_36\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(17),
      O => \dc_offset[0]_i_36_n_0\
    );
\dc_offset[0]_i_37\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(16),
      O => \dc_offset[0]_i_37_n_0\
    );
\dc_offset[0]_i_39\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(15),
      O => \dc_offset[0]_i_39_n_0\
    );
\dc_offset[0]_i_4\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(43),
      O => \dc_offset[0]_i_4_n_0\
    );
\dc_offset[0]_i_40\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(14),
      O => \dc_offset[0]_i_40_n_0\
    );
\dc_offset[0]_i_41\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(13),
      O => \dc_offset[0]_i_41_n_0\
    );
\dc_offset[0]_i_42\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(12),
      O => \dc_offset[0]_i_42_n_0\
    );
\dc_offset[0]_i_44\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(11),
      O => \dc_offset[0]_i_44_n_0\
    );
\dc_offset[0]_i_45\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(10),
      O => \dc_offset[0]_i_45_n_0\
    );
\dc_offset[0]_i_46\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(9),
      O => \dc_offset[0]_i_46_n_0\
    );
\dc_offset[0]_i_47\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(8),
      O => \dc_offset[0]_i_47_n_0\
    );
\dc_offset[0]_i_49\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(7),
      O => \dc_offset[0]_i_49_n_0\
    );
\dc_offset[0]_i_5\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(42),
      O => \dc_offset[0]_i_5_n_0\
    );
\dc_offset[0]_i_50\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(6),
      O => \dc_offset[0]_i_50_n_0\
    );
\dc_offset[0]_i_51\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(5),
      O => \dc_offset[0]_i_51_n_0\
    );
\dc_offset[0]_i_52\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(4),
      O => \dc_offset[0]_i_52_n_0\
    );
\dc_offset[0]_i_53\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(3),
      O => \dc_offset[0]_i_53_n_0\
    );
\dc_offset[0]_i_54\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(2),
      O => \dc_offset[0]_i_54_n_0\
    );
\dc_offset[0]_i_55\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(1),
      O => \dc_offset[0]_i_55_n_0\
    );
\dc_offset[0]_i_6\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(41),
      O => \dc_offset[0]_i_6_n_0\
    );
\dc_offset[0]_i_7\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(40),
      O => \dc_offset[0]_i_7_n_0\
    );
\dc_offset[0]_i_9\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(39),
      O => \dc_offset[0]_i_9_n_0\
    );
\dc_offset[10]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(10),
      I1 => mul_ln39_reg_304(52),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(10)
    );
\dc_offset[11]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(11),
      I1 => mul_ln39_reg_304(53),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(11)
    );
\dc_offset[12]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(12),
      I1 => mul_ln39_reg_304(54),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(12)
    );
\dc_offset[12]_i_10\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(49),
      O => \dc_offset[12]_i_10_n_0\
    );
\dc_offset[12]_i_11\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(48),
      O => \dc_offset[12]_i_11_n_0\
    );
\dc_offset[12]_i_3\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(54),
      O => \dc_offset[12]_i_3_n_0\
    );
\dc_offset[12]_i_4\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(53),
      O => \dc_offset[12]_i_4_n_0\
    );
\dc_offset[12]_i_5\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(52),
      O => \dc_offset[12]_i_5_n_0\
    );
\dc_offset[12]_i_6\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(51),
      O => \dc_offset[12]_i_6_n_0\
    );
\dc_offset[12]_i_8\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(51),
      O => \dc_offset[12]_i_8_n_0\
    );
\dc_offset[12]_i_9\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(50),
      O => \dc_offset[12]_i_9_n_0\
    );
\dc_offset[13]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(13),
      I1 => mul_ln39_reg_304(55),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(13)
    );
\dc_offset[14]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(14),
      I1 => tmp_2_reg_309(14),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(14)
    );
\dc_offset[15]_i_10\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => tmp_2_reg_309(14),
      O => \dc_offset[15]_i_10_n_0\
    );
\dc_offset[15]_i_11\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(55),
      O => \dc_offset[15]_i_11_n_0\
    );
\dc_offset[15]_i_12\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(54),
      O => \dc_offset[15]_i_12_n_0\
    );
\dc_offset[15]_i_13\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(53),
      O => \dc_offset[15]_i_13_n_0\
    );
\dc_offset[15]_i_14\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(52),
      O => \dc_offset[15]_i_14_n_0\
    );
\dc_offset[15]_i_2\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(15),
      I1 => tmp_2_reg_309(15),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(15)
    );
\dc_offset[15]_i_4\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(57),
      O => \dc_offset[15]_i_4_n_0\
    );
\dc_offset[15]_i_5\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(56),
      O => \dc_offset[15]_i_5_n_0\
    );
\dc_offset[15]_i_6\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(55),
      O => \dc_offset[15]_i_6_n_0\
    );
\dc_offset[15]_i_9\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => tmp_2_reg_309(15),
      O => \dc_offset[15]_i_9_n_0\
    );
\dc_offset[1]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(1),
      I1 => mul_ln39_reg_304(43),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(1)
    );
\dc_offset[2]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(2),
      I1 => mul_ln39_reg_304(44),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(2)
    );
\dc_offset[3]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(3),
      I1 => mul_ln39_reg_304(45),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(3)
    );
\dc_offset[4]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(4),
      I1 => mul_ln39_reg_304(46),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(4)
    );
\dc_offset[4]_i_3\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(42),
      O => \dc_offset[4]_i_3_n_0\
    );
\dc_offset[4]_i_4\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(46),
      O => \dc_offset[4]_i_4_n_0\
    );
\dc_offset[4]_i_5\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(45),
      O => \dc_offset[4]_i_5_n_0\
    );
\dc_offset[4]_i_6\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(44),
      O => \dc_offset[4]_i_6_n_0\
    );
\dc_offset[4]_i_7\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(43),
      O => \dc_offset[4]_i_7_n_0\
    );
\dc_offset[5]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(5),
      I1 => mul_ln39_reg_304(47),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(5)
    );
\dc_offset[6]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(6),
      I1 => mul_ln39_reg_304(48),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(6)
    );
\dc_offset[7]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(7),
      I1 => mul_ln39_reg_304(49),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(7)
    );
\dc_offset[8]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(8),
      I1 => mul_ln39_reg_304(50),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(8)
    );
\dc_offset[8]_i_10\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(45),
      O => \dc_offset[8]_i_10_n_0\
    );
\dc_offset[8]_i_11\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(44),
      O => \dc_offset[8]_i_11_n_0\
    );
\dc_offset[8]_i_3\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(50),
      O => \dc_offset[8]_i_3_n_0\
    );
\dc_offset[8]_i_4\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(49),
      O => \dc_offset[8]_i_4_n_0\
    );
\dc_offset[8]_i_5\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(48),
      O => \dc_offset[8]_i_5_n_0\
    );
\dc_offset[8]_i_6\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => sub_ln39_fu_206_p2(47),
      O => \dc_offset[8]_i_6_n_0\
    );
\dc_offset[8]_i_8\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(47),
      O => \dc_offset[8]_i_8_n_0\
    );
\dc_offset[8]_i_9\: unisim.vcomponents.LUT1
    generic map(
      INIT => X"1"
    )
        port map (
      I0 => mul_ln39_reg_304(46),
      O => \dc_offset[8]_i_9_n_0\
    );
\dc_offset[9]_i_1\: unisim.vcomponents.LUT3
    generic map(
      INIT => X"AC"
    )
        port map (
      I0 => sub_ln39_1_fu_227_p2(9),
      I1 => mul_ln39_reg_304(51),
      I2 => tmp_reg_298_pp0_iter2_reg,
      O => select_ln39_1_fu_233_p3(9)
    );
\dc_offset_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(0),
      Q => dc_offset(0),
      R => '0'
    );
\dc_offset_reg[0]_i_13\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_18_n_0\,
      CO(3) => \dc_offset_reg[0]_i_13_n_0\,
      CO(2) => \dc_offset_reg[0]_i_13_n_1\,
      CO(1) => \dc_offset_reg[0]_i_13_n_2\,
      CO(0) => \dc_offset_reg[0]_i_13_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_13_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_19_n_0\,
      S(2) => \dc_offset[0]_i_20_n_0\,
      S(1) => \dc_offset[0]_i_21_n_0\,
      S(0) => \dc_offset[0]_i_22_n_0\
    );
\dc_offset_reg[0]_i_18\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_23_n_0\,
      CO(3) => \dc_offset_reg[0]_i_18_n_0\,
      CO(2) => \dc_offset_reg[0]_i_18_n_1\,
      CO(1) => \dc_offset_reg[0]_i_18_n_2\,
      CO(0) => \dc_offset_reg[0]_i_18_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_18_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_24_n_0\,
      S(2) => \dc_offset[0]_i_25_n_0\,
      S(1) => \dc_offset[0]_i_26_n_0\,
      S(0) => \dc_offset[0]_i_27_n_0\
    );
\dc_offset_reg[0]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_3_n_0\,
      CO(3) => \dc_offset_reg[0]_i_2_n_0\,
      CO(2) => \dc_offset_reg[0]_i_2_n_1\,
      CO(1) => \dc_offset_reg[0]_i_2_n_2\,
      CO(0) => \dc_offset_reg[0]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 2) => sub_ln39_fu_206_p2(43 downto 42),
      O(1 downto 0) => \NLW_dc_offset_reg[0]_i_2_O_UNCONNECTED\(1 downto 0),
      S(3) => \dc_offset[0]_i_4_n_0\,
      S(2) => \dc_offset[0]_i_5_n_0\,
      S(1) => \dc_offset[0]_i_6_n_0\,
      S(0) => \dc_offset[0]_i_7_n_0\
    );
\dc_offset_reg[0]_i_23\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_28_n_0\,
      CO(3) => \dc_offset_reg[0]_i_23_n_0\,
      CO(2) => \dc_offset_reg[0]_i_23_n_1\,
      CO(1) => \dc_offset_reg[0]_i_23_n_2\,
      CO(0) => \dc_offset_reg[0]_i_23_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_23_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_29_n_0\,
      S(2) => \dc_offset[0]_i_30_n_0\,
      S(1) => \dc_offset[0]_i_31_n_0\,
      S(0) => \dc_offset[0]_i_32_n_0\
    );
\dc_offset_reg[0]_i_28\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_33_n_0\,
      CO(3) => \dc_offset_reg[0]_i_28_n_0\,
      CO(2) => \dc_offset_reg[0]_i_28_n_1\,
      CO(1) => \dc_offset_reg[0]_i_28_n_2\,
      CO(0) => \dc_offset_reg[0]_i_28_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_28_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_34_n_0\,
      S(2) => \dc_offset[0]_i_35_n_0\,
      S(1) => \dc_offset[0]_i_36_n_0\,
      S(0) => \dc_offset[0]_i_37_n_0\
    );
\dc_offset_reg[0]_i_3\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_8_n_0\,
      CO(3) => \dc_offset_reg[0]_i_3_n_0\,
      CO(2) => \dc_offset_reg[0]_i_3_n_1\,
      CO(1) => \dc_offset_reg[0]_i_3_n_2\,
      CO(0) => \dc_offset_reg[0]_i_3_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_3_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_9_n_0\,
      S(2) => \dc_offset[0]_i_10_n_0\,
      S(1) => \dc_offset[0]_i_11_n_0\,
      S(0) => \dc_offset[0]_i_12_n_0\
    );
\dc_offset_reg[0]_i_33\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_38_n_0\,
      CO(3) => \dc_offset_reg[0]_i_33_n_0\,
      CO(2) => \dc_offset_reg[0]_i_33_n_1\,
      CO(1) => \dc_offset_reg[0]_i_33_n_2\,
      CO(0) => \dc_offset_reg[0]_i_33_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_33_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_39_n_0\,
      S(2) => \dc_offset[0]_i_40_n_0\,
      S(1) => \dc_offset[0]_i_41_n_0\,
      S(0) => \dc_offset[0]_i_42_n_0\
    );
\dc_offset_reg[0]_i_38\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_43_n_0\,
      CO(3) => \dc_offset_reg[0]_i_38_n_0\,
      CO(2) => \dc_offset_reg[0]_i_38_n_1\,
      CO(1) => \dc_offset_reg[0]_i_38_n_2\,
      CO(0) => \dc_offset_reg[0]_i_38_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_38_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_44_n_0\,
      S(2) => \dc_offset[0]_i_45_n_0\,
      S(1) => \dc_offset[0]_i_46_n_0\,
      S(0) => \dc_offset[0]_i_47_n_0\
    );
\dc_offset_reg[0]_i_43\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_48_n_0\,
      CO(3) => \dc_offset_reg[0]_i_43_n_0\,
      CO(2) => \dc_offset_reg[0]_i_43_n_1\,
      CO(1) => \dc_offset_reg[0]_i_43_n_2\,
      CO(0) => \dc_offset_reg[0]_i_43_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_43_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_49_n_0\,
      S(2) => \dc_offset[0]_i_50_n_0\,
      S(1) => \dc_offset[0]_i_51_n_0\,
      S(0) => \dc_offset[0]_i_52_n_0\
    );
\dc_offset_reg[0]_i_48\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \dc_offset_reg[0]_i_48_n_0\,
      CO(2) => \dc_offset_reg[0]_i_48_n_1\,
      CO(1) => \dc_offset_reg[0]_i_48_n_2\,
      CO(0) => \dc_offset_reg[0]_i_48_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0001",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_48_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_53_n_0\,
      S(2) => \dc_offset[0]_i_54_n_0\,
      S(1) => \dc_offset[0]_i_55_n_0\,
      S(0) => mul_ln39_reg_304(0)
    );
\dc_offset_reg[0]_i_8\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_13_n_0\,
      CO(3) => \dc_offset_reg[0]_i_8_n_0\,
      CO(2) => \dc_offset_reg[0]_i_8_n_1\,
      CO(1) => \dc_offset_reg[0]_i_8_n_2\,
      CO(0) => \dc_offset_reg[0]_i_8_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => \NLW_dc_offset_reg[0]_i_8_O_UNCONNECTED\(3 downto 0),
      S(3) => \dc_offset[0]_i_14_n_0\,
      S(2) => \dc_offset[0]_i_15_n_0\,
      S(1) => \dc_offset[0]_i_16_n_0\,
      S(0) => \dc_offset[0]_i_17_n_0\
    );
\dc_offset_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(10),
      Q => dc_offset(10),
      R => '0'
    );
\dc_offset_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(11),
      Q => dc_offset(11),
      R => '0'
    );
\dc_offset_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(12),
      Q => dc_offset(12),
      R => '0'
    );
\dc_offset_reg[12]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[8]_i_2_n_0\,
      CO(3) => \dc_offset_reg[12]_i_2_n_0\,
      CO(2) => \dc_offset_reg[12]_i_2_n_1\,
      CO(1) => \dc_offset_reg[12]_i_2_n_2\,
      CO(0) => \dc_offset_reg[12]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => sub_ln39_1_fu_227_p2(12 downto 9),
      S(3) => \dc_offset[12]_i_3_n_0\,
      S(2) => \dc_offset[12]_i_4_n_0\,
      S(1) => \dc_offset[12]_i_5_n_0\,
      S(0) => \dc_offset[12]_i_6_n_0\
    );
\dc_offset_reg[12]_i_7\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[8]_i_7_n_0\,
      CO(3) => \dc_offset_reg[12]_i_7_n_0\,
      CO(2) => \dc_offset_reg[12]_i_7_n_1\,
      CO(1) => \dc_offset_reg[12]_i_7_n_2\,
      CO(0) => \dc_offset_reg[12]_i_7_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => sub_ln39_fu_206_p2(51 downto 48),
      S(3) => \dc_offset[12]_i_8_n_0\,
      S(2) => \dc_offset[12]_i_9_n_0\,
      S(1) => \dc_offset[12]_i_10_n_0\,
      S(0) => \dc_offset[12]_i_11_n_0\
    );
\dc_offset_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(13),
      Q => dc_offset(13),
      R => '0'
    );
\dc_offset_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(14),
      Q => dc_offset(14),
      R => '0'
    );
\dc_offset_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(15),
      Q => dc_offset(15),
      R => '0'
    );
\dc_offset_reg[15]_i_3\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[12]_i_2_n_0\,
      CO(3 downto 2) => \NLW_dc_offset_reg[15]_i_3_CO_UNCONNECTED\(3 downto 2),
      CO(1) => \dc_offset_reg[15]_i_3_n_2\,
      CO(0) => \dc_offset_reg[15]_i_3_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3) => \NLW_dc_offset_reg[15]_i_3_O_UNCONNECTED\(3),
      O(2 downto 0) => sub_ln39_1_fu_227_p2(15 downto 13),
      S(3) => '0',
      S(2) => \dc_offset[15]_i_4_n_0\,
      S(1) => \dc_offset[15]_i_5_n_0\,
      S(0) => \dc_offset[15]_i_6_n_0\
    );
\dc_offset_reg[15]_i_7\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[15]_i_8_n_0\,
      CO(3 downto 1) => \NLW_dc_offset_reg[15]_i_7_CO_UNCONNECTED\(3 downto 1),
      CO(0) => \dc_offset_reg[15]_i_7_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 2) => \NLW_dc_offset_reg[15]_i_7_O_UNCONNECTED\(3 downto 2),
      O(1 downto 0) => sub_ln39_fu_206_p2(57 downto 56),
      S(3 downto 2) => B"00",
      S(1) => \dc_offset[15]_i_9_n_0\,
      S(0) => \dc_offset[15]_i_10_n_0\
    );
\dc_offset_reg[15]_i_8\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[12]_i_7_n_0\,
      CO(3) => \dc_offset_reg[15]_i_8_n_0\,
      CO(2) => \dc_offset_reg[15]_i_8_n_1\,
      CO(1) => \dc_offset_reg[15]_i_8_n_2\,
      CO(0) => \dc_offset_reg[15]_i_8_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => sub_ln39_fu_206_p2(55 downto 52),
      S(3) => \dc_offset[15]_i_11_n_0\,
      S(2) => \dc_offset[15]_i_12_n_0\,
      S(1) => \dc_offset[15]_i_13_n_0\,
      S(0) => \dc_offset[15]_i_14_n_0\
    );
\dc_offset_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(1),
      Q => dc_offset(1),
      R => '0'
    );
\dc_offset_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(2),
      Q => dc_offset(2),
      R => '0'
    );
\dc_offset_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(3),
      Q => dc_offset(3),
      R => '0'
    );
\dc_offset_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(4),
      Q => dc_offset(4),
      R => '0'
    );
\dc_offset_reg[4]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => '0',
      CO(3) => \dc_offset_reg[4]_i_2_n_0\,
      CO(2) => \dc_offset_reg[4]_i_2_n_1\,
      CO(1) => \dc_offset_reg[4]_i_2_n_2\,
      CO(0) => \dc_offset_reg[4]_i_2_n_3\,
      CYINIT => \dc_offset[4]_i_3_n_0\,
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => sub_ln39_1_fu_227_p2(4 downto 1),
      S(3) => \dc_offset[4]_i_4_n_0\,
      S(2) => \dc_offset[4]_i_5_n_0\,
      S(1) => \dc_offset[4]_i_6_n_0\,
      S(0) => \dc_offset[4]_i_7_n_0\
    );
\dc_offset_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(5),
      Q => dc_offset(5),
      R => '0'
    );
\dc_offset_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(6),
      Q => dc_offset(6),
      R => '0'
    );
\dc_offset_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(7),
      Q => dc_offset(7),
      R => '0'
    );
\dc_offset_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(8),
      Q => dc_offset(8),
      R => '0'
    );
\dc_offset_reg[8]_i_2\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[4]_i_2_n_0\,
      CO(3) => \dc_offset_reg[8]_i_2_n_0\,
      CO(2) => \dc_offset_reg[8]_i_2_n_1\,
      CO(1) => \dc_offset_reg[8]_i_2_n_2\,
      CO(0) => \dc_offset_reg[8]_i_2_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => sub_ln39_1_fu_227_p2(8 downto 5),
      S(3) => \dc_offset[8]_i_3_n_0\,
      S(2) => \dc_offset[8]_i_4_n_0\,
      S(1) => \dc_offset[8]_i_5_n_0\,
      S(0) => \dc_offset[8]_i_6_n_0\
    );
\dc_offset_reg[8]_i_7\: unisim.vcomponents.CARRY4
     port map (
      CI => \dc_offset_reg[0]_i_2_n_0\,
      CO(3) => \dc_offset_reg[8]_i_7_n_0\,
      CO(2) => \dc_offset_reg[8]_i_7_n_1\,
      CO(1) => \dc_offset_reg[8]_i_7_n_2\,
      CO(0) => \dc_offset_reg[8]_i_7_n_3\,
      CYINIT => '0',
      DI(3 downto 0) => B"0000",
      O(3 downto 0) => sub_ln39_fu_206_p2(47 downto 44),
      S(3) => \dc_offset[8]_i_8_n_0\,
      S(2) => \dc_offset[8]_i_9_n_0\,
      S(1) => \dc_offset[8]_i_10_n_0\,
      S(0) => \dc_offset[8]_i_11_n_0\
    );
\dc_offset_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => dc_offset0,
      D => select_ln39_1_fu_233_p3(9),
      Q => dc_offset(9),
      R => '0'
    );
\icmp_ln35_reg_289[0]_i_1\: unisim.vcomponents.LUT4
    generic map(
      INIT => X"8000"
    )
        port map (
      I0 => regslice_both_out_stream_V_data_V_U_n_6,
      I1 => regslice_both_out_stream_V_data_V_U_n_7,
      I2 => regslice_both_out_stream_V_data_V_U_n_8,
      I3 => regslice_both_out_stream_V_data_V_U_n_9,
      O => icmp_ln35_fu_173_p2
    );
\icmp_ln35_reg_289_pp0_iter1_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => icmp_ln35_reg_289,
      Q => icmp_ln35_reg_289_pp0_iter1_reg,
      R => '0'
    );
\icmp_ln35_reg_289_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => icmp_ln35_reg_289_pp0_iter1_reg,
      Q => icmp_ln35_reg_289_pp0_iter2_reg,
      R => '0'
    );
\icmp_ln35_reg_289_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => icmp_ln35_fu_173_p2,
      Q => icmp_ln35_reg_289,
      R => '0'
    );
mul_32s_34ns_65_2_1_U1: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_mul_32s_34ns_65_2_1
     port map (
      D(41 downto 16) => \buff0_reg__1\(41 downto 16),
      D(15) => mul_32s_34ns_65_2_1_U1_n_41,
      D(14) => mul_32s_34ns_65_2_1_U1_n_42,
      D(13) => mul_32s_34ns_65_2_1_U1_n_43,
      D(12) => mul_32s_34ns_65_2_1_U1_n_44,
      D(11) => mul_32s_34ns_65_2_1_U1_n_45,
      D(10) => mul_32s_34ns_65_2_1_U1_n_46,
      D(9) => mul_32s_34ns_65_2_1_U1_n_47,
      D(8) => mul_32s_34ns_65_2_1_U1_n_48,
      D(7) => mul_32s_34ns_65_2_1_U1_n_49,
      D(6) => mul_32s_34ns_65_2_1_U1_n_50,
      D(5) => mul_32s_34ns_65_2_1_U1_n_51,
      D(4) => mul_32s_34ns_65_2_1_U1_n_52,
      D(3) => mul_32s_34ns_65_2_1_U1_n_53,
      D(2) => mul_32s_34ns_65_2_1_U1_n_54,
      D(1) => mul_32s_34ns_65_2_1_U1_n_55,
      D(0) => mul_32s_34ns_65_2_1_U1_n_56,
      S(3) => mul_32s_34ns_65_2_1_U1_n_0,
      S(2) => mul_32s_34ns_65_2_1_U1_n_1,
      S(1) => mul_32s_34ns_65_2_1_U1_n_2,
      S(0) => mul_32s_34ns_65_2_1_U1_n_3,
      add_ln32_fu_145_p2(31 downto 0) => add_ln32_fu_145_p2(31 downto 0),
      add_ln32_reg_2830 => add_ln32_reg_2830,
      ap_block_pp0_stage0_11001 => ap_block_pp0_stage0_11001,
      ap_clk => ap_clk,
      \buff0_reg__0_0\(15 downto 0) => \buff0_reg__1\(57 downto 42),
      sum_reg(15 downto 0) => sum_reg(31 downto 16),
      \sum_reg[18]\(2) => mul_32s_34ns_65_2_1_U1_n_4,
      \sum_reg[18]\(1) => mul_32s_34ns_65_2_1_U1_n_5,
      \sum_reg[18]\(0) => mul_32s_34ns_65_2_1_U1_n_6,
      \sum_reg[22]\(3) => mul_32s_34ns_65_2_1_U1_n_7,
      \sum_reg[22]\(2) => mul_32s_34ns_65_2_1_U1_n_8,
      \sum_reg[22]\(1) => mul_32s_34ns_65_2_1_U1_n_9,
      \sum_reg[22]\(0) => mul_32s_34ns_65_2_1_U1_n_10,
      \sum_reg[26]\(3) => mul_32s_34ns_65_2_1_U1_n_11,
      \sum_reg[26]\(2) => mul_32s_34ns_65_2_1_U1_n_12,
      \sum_reg[26]\(1) => mul_32s_34ns_65_2_1_U1_n_13,
      \sum_reg[26]\(0) => mul_32s_34ns_65_2_1_U1_n_14
    );
\mul_ln39_reg_304_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_56,
      Q => mul_ln39_reg_304(0),
      R => '0'
    );
\mul_ln39_reg_304_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_46,
      Q => mul_ln39_reg_304(10),
      R => '0'
    );
\mul_ln39_reg_304_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_45,
      Q => mul_ln39_reg_304(11),
      R => '0'
    );
\mul_ln39_reg_304_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_44,
      Q => mul_ln39_reg_304(12),
      R => '0'
    );
\mul_ln39_reg_304_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_43,
      Q => mul_ln39_reg_304(13),
      R => '0'
    );
\mul_ln39_reg_304_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_42,
      Q => mul_ln39_reg_304(14),
      R => '0'
    );
\mul_ln39_reg_304_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_41,
      Q => mul_ln39_reg_304(15),
      R => '0'
    );
\mul_ln39_reg_304_reg[16]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(16),
      Q => mul_ln39_reg_304(16),
      R => '0'
    );
\mul_ln39_reg_304_reg[17]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(17),
      Q => mul_ln39_reg_304(17),
      R => '0'
    );
\mul_ln39_reg_304_reg[18]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(18),
      Q => mul_ln39_reg_304(18),
      R => '0'
    );
\mul_ln39_reg_304_reg[19]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(19),
      Q => mul_ln39_reg_304(19),
      R => '0'
    );
\mul_ln39_reg_304_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_55,
      Q => mul_ln39_reg_304(1),
      R => '0'
    );
\mul_ln39_reg_304_reg[20]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(20),
      Q => mul_ln39_reg_304(20),
      R => '0'
    );
\mul_ln39_reg_304_reg[21]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(21),
      Q => mul_ln39_reg_304(21),
      R => '0'
    );
\mul_ln39_reg_304_reg[22]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(22),
      Q => mul_ln39_reg_304(22),
      R => '0'
    );
\mul_ln39_reg_304_reg[23]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(23),
      Q => mul_ln39_reg_304(23),
      R => '0'
    );
\mul_ln39_reg_304_reg[24]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(24),
      Q => mul_ln39_reg_304(24),
      R => '0'
    );
\mul_ln39_reg_304_reg[25]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(25),
      Q => mul_ln39_reg_304(25),
      R => '0'
    );
\mul_ln39_reg_304_reg[26]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(26),
      Q => mul_ln39_reg_304(26),
      R => '0'
    );
\mul_ln39_reg_304_reg[27]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(27),
      Q => mul_ln39_reg_304(27),
      R => '0'
    );
\mul_ln39_reg_304_reg[28]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(28),
      Q => mul_ln39_reg_304(28),
      R => '0'
    );
\mul_ln39_reg_304_reg[29]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(29),
      Q => mul_ln39_reg_304(29),
      R => '0'
    );
\mul_ln39_reg_304_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_54,
      Q => mul_ln39_reg_304(2),
      R => '0'
    );
\mul_ln39_reg_304_reg[30]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(30),
      Q => mul_ln39_reg_304(30),
      R => '0'
    );
\mul_ln39_reg_304_reg[31]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(31),
      Q => mul_ln39_reg_304(31),
      R => '0'
    );
\mul_ln39_reg_304_reg[32]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(32),
      Q => mul_ln39_reg_304(32),
      R => '0'
    );
\mul_ln39_reg_304_reg[33]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(33),
      Q => mul_ln39_reg_304(33),
      R => '0'
    );
\mul_ln39_reg_304_reg[34]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(34),
      Q => mul_ln39_reg_304(34),
      R => '0'
    );
\mul_ln39_reg_304_reg[35]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(35),
      Q => mul_ln39_reg_304(35),
      R => '0'
    );
\mul_ln39_reg_304_reg[36]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(36),
      Q => mul_ln39_reg_304(36),
      R => '0'
    );
\mul_ln39_reg_304_reg[37]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(37),
      Q => mul_ln39_reg_304(37),
      R => '0'
    );
\mul_ln39_reg_304_reg[38]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(38),
      Q => mul_ln39_reg_304(38),
      R => '0'
    );
\mul_ln39_reg_304_reg[39]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(39),
      Q => mul_ln39_reg_304(39),
      R => '0'
    );
\mul_ln39_reg_304_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_53,
      Q => mul_ln39_reg_304(3),
      R => '0'
    );
\mul_ln39_reg_304_reg[40]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(40),
      Q => mul_ln39_reg_304(40),
      R => '0'
    );
\mul_ln39_reg_304_reg[41]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(41),
      Q => mul_ln39_reg_304(41),
      R => '0'
    );
\mul_ln39_reg_304_reg[42]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(42),
      Q => mul_ln39_reg_304(42),
      R => '0'
    );
\mul_ln39_reg_304_reg[43]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(43),
      Q => mul_ln39_reg_304(43),
      R => '0'
    );
\mul_ln39_reg_304_reg[44]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(44),
      Q => mul_ln39_reg_304(44),
      R => '0'
    );
\mul_ln39_reg_304_reg[45]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(45),
      Q => mul_ln39_reg_304(45),
      R => '0'
    );
\mul_ln39_reg_304_reg[46]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(46),
      Q => mul_ln39_reg_304(46),
      R => '0'
    );
\mul_ln39_reg_304_reg[47]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(47),
      Q => mul_ln39_reg_304(47),
      R => '0'
    );
\mul_ln39_reg_304_reg[48]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(48),
      Q => mul_ln39_reg_304(48),
      R => '0'
    );
\mul_ln39_reg_304_reg[49]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(49),
      Q => mul_ln39_reg_304(49),
      R => '0'
    );
\mul_ln39_reg_304_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_52,
      Q => mul_ln39_reg_304(4),
      R => '0'
    );
\mul_ln39_reg_304_reg[50]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(50),
      Q => mul_ln39_reg_304(50),
      R => '0'
    );
\mul_ln39_reg_304_reg[51]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(51),
      Q => mul_ln39_reg_304(51),
      R => '0'
    );
\mul_ln39_reg_304_reg[52]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(52),
      Q => mul_ln39_reg_304(52),
      R => '0'
    );
\mul_ln39_reg_304_reg[53]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(53),
      Q => mul_ln39_reg_304(53),
      R => '0'
    );
\mul_ln39_reg_304_reg[54]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(54),
      Q => mul_ln39_reg_304(54),
      R => '0'
    );
\mul_ln39_reg_304_reg[55]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(55),
      Q => mul_ln39_reg_304(55),
      R => '0'
    );
\mul_ln39_reg_304_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_51,
      Q => mul_ln39_reg_304(5),
      R => '0'
    );
\mul_ln39_reg_304_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_50,
      Q => mul_ln39_reg_304(6),
      R => '0'
    );
\mul_ln39_reg_304_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_49,
      Q => mul_ln39_reg_304(7),
      R => '0'
    );
\mul_ln39_reg_304_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_48,
      Q => mul_ln39_reg_304(8),
      R => '0'
    );
\mul_ln39_reg_304_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => mul_32s_34ns_65_2_1_U1_n_47,
      Q => mul_ln39_reg_304(9),
      R => '0'
    );
\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TKEEP_int_regslice(0),
      Q => \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_n_0\
    );
\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TKEEP_int_regslice(1),
      Q => \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_n_0\
    );
\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TKEEP_int_regslice(2),
      Q => \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_n_0\
    );
\pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TKEEP_int_regslice(3),
      Q => \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_n_0\
    );
\pkt_in_keep_V_reg_259_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[0]_srl2_n_0\,
      Q => pkt_in_keep_V_reg_259_pp0_iter2_reg(0),
      R => '0'
    );
\pkt_in_keep_V_reg_259_pp0_iter2_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[1]_srl2_n_0\,
      Q => pkt_in_keep_V_reg_259_pp0_iter2_reg(1),
      R => '0'
    );
\pkt_in_keep_V_reg_259_pp0_iter2_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[2]_srl2_n_0\,
      Q => pkt_in_keep_V_reg_259_pp0_iter2_reg(2),
      R => '0'
    );
\pkt_in_keep_V_reg_259_pp0_iter2_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_keep_V_reg_259_pp0_iter1_reg_reg[3]_srl2_n_0\,
      Q => pkt_in_keep_V_reg_259_pp0_iter2_reg(3),
      R => '0'
    );
\pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TLAST_int_regslice,
      Q => \pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2_n_0\
    );
\pkt_in_last_V_reg_269_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_last_V_reg_269_pp0_iter1_reg_reg[0]_srl2_n_0\,
      Q => pkt_in_last_V_reg_269_pp0_iter2_reg,
      R => '0'
    );
\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TSTRB_int_regslice(0),
      Q => \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_n_0\
    );
\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TSTRB_int_regslice(1),
      Q => \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_n_0\
    );
\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TSTRB_int_regslice(2),
      Q => \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_n_0\
    );
\pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => in_stream_TSTRB_int_regslice(3),
      Q => \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_n_0\
    );
\pkt_in_strb_V_reg_264_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[0]_srl2_n_0\,
      Q => pkt_in_strb_V_reg_264_pp0_iter2_reg(0),
      R => '0'
    );
\pkt_in_strb_V_reg_264_pp0_iter2_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[1]_srl2_n_0\,
      Q => pkt_in_strb_V_reg_264_pp0_iter2_reg(1),
      R => '0'
    );
\pkt_in_strb_V_reg_264_pp0_iter2_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[2]_srl2_n_0\,
      Q => pkt_in_strb_V_reg_264_pp0_iter2_reg(2),
      R => '0'
    );
\pkt_in_strb_V_reg_264_pp0_iter2_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \pkt_in_strb_V_reg_264_pp0_iter1_reg_reg[3]_srl2_n_0\,
      Q => pkt_in_strb_V_reg_264_pp0_iter2_reg(3),
      R => '0'
    );
regslice_both_in_stream_V_data_V_U: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both
     port map (
      \B_V_data_1_payload_B_reg[15]_0\(3) => regslice_both_in_stream_V_data_V_U_n_39,
      \B_V_data_1_payload_B_reg[15]_0\(2) => regslice_both_in_stream_V_data_V_U_n_40,
      \B_V_data_1_payload_B_reg[15]_0\(1) => regslice_both_in_stream_V_data_V_U_n_41,
      \B_V_data_1_payload_B_reg[15]_0\(0) => regslice_both_in_stream_V_data_V_U_n_42,
      \B_V_data_1_payload_B_reg[15]_1\(3) => regslice_both_in_stream_V_data_V_U_n_43,
      \B_V_data_1_payload_B_reg[15]_1\(2) => regslice_both_in_stream_V_data_V_U_n_44,
      \B_V_data_1_payload_B_reg[15]_1\(1) => regslice_both_in_stream_V_data_V_U_n_45,
      \B_V_data_1_payload_B_reg[15]_1\(0) => regslice_both_in_stream_V_data_V_U_n_46,
      \B_V_data_1_payload_B_reg[15]_2\(3) => regslice_both_in_stream_V_data_V_U_n_47,
      \B_V_data_1_payload_B_reg[15]_2\(2) => regslice_both_in_stream_V_data_V_U_n_48,
      \B_V_data_1_payload_B_reg[15]_2\(1) => regslice_both_in_stream_V_data_V_U_n_49,
      \B_V_data_1_payload_B_reg[15]_2\(0) => regslice_both_in_stream_V_data_V_U_n_50,
      \B_V_data_1_state_reg[1]_0\ => in_stream_TREADY,
      \B_V_data_1_state_reg[1]_1\ => regslice_both_out_stream_V_data_V_U_n_3,
      O(3) => regslice_both_in_stream_V_data_V_U_n_19,
      O(2) => regslice_both_in_stream_V_data_V_U_n_20,
      O(1) => regslice_both_in_stream_V_data_V_U_n_21,
      O(0) => regslice_both_in_stream_V_data_V_U_n_22,
      S(3) => mul_32s_34ns_65_2_1_U1_n_0,
      S(2) => mul_32s_34ns_65_2_1_U1_n_1,
      S(1) => mul_32s_34ns_65_2_1_U1_n_2,
      S(0) => mul_32s_34ns_65_2_1_U1_n_3,
      add_ln32_fu_145_p2(31 downto 0) => add_ln32_fu_145_p2(31 downto 0),
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      \in\(15 downto 0) => \in\(15 downto 0),
      in_stream_TDATA(15 downto 0) => in_stream_TDATA(15 downto 0),
      in_stream_TVALID => in_stream_TVALID,
      in_stream_TVALID_int_regslice => in_stream_TVALID_int_regslice,
      sum_reg(31 downto 0) => sum_reg(31 downto 0),
      \sum_reg[11]\(3) => regslice_both_in_stream_V_data_V_U_n_27,
      \sum_reg[11]\(2) => regslice_both_in_stream_V_data_V_U_n_28,
      \sum_reg[11]\(1) => regslice_both_in_stream_V_data_V_U_n_29,
      \sum_reg[11]\(0) => regslice_both_in_stream_V_data_V_U_n_30,
      \sum_reg[14]\(3) => regslice_both_in_stream_V_data_V_U_n_31,
      \sum_reg[14]\(2) => regslice_both_in_stream_V_data_V_U_n_32,
      \sum_reg[14]\(1) => regslice_both_in_stream_V_data_V_U_n_33,
      \sum_reg[14]\(0) => regslice_both_in_stream_V_data_V_U_n_34,
      \sum_reg[14]_0\(3) => regslice_both_in_stream_V_data_V_U_n_35,
      \sum_reg[14]_0\(2) => regslice_both_in_stream_V_data_V_U_n_36,
      \sum_reg[14]_0\(1) => regslice_both_in_stream_V_data_V_U_n_37,
      \sum_reg[14]_0\(0) => regslice_both_in_stream_V_data_V_U_n_38,
      \sum_reg[7]\(3) => regslice_both_in_stream_V_data_V_U_n_23,
      \sum_reg[7]\(2) => regslice_both_in_stream_V_data_V_U_n_24,
      \sum_reg[7]\(1) => regslice_both_in_stream_V_data_V_U_n_25,
      \sum_reg[7]\(0) => regslice_both_in_stream_V_data_V_U_n_26,
      tmp_product(2) => mul_32s_34ns_65_2_1_U1_n_4,
      tmp_product(1) => mul_32s_34ns_65_2_1_U1_n_5,
      tmp_product(0) => mul_32s_34ns_65_2_1_U1_n_6,
      tmp_product_0(3) => mul_32s_34ns_65_2_1_U1_n_7,
      tmp_product_0(2) => mul_32s_34ns_65_2_1_U1_n_8,
      tmp_product_0(1) => mul_32s_34ns_65_2_1_U1_n_9,
      tmp_product_0(0) => mul_32s_34ns_65_2_1_U1_n_10,
      tmp_product_1(3) => mul_32s_34ns_65_2_1_U1_n_11,
      tmp_product_1(2) => mul_32s_34ns_65_2_1_U1_n_12,
      tmp_product_1(1) => mul_32s_34ns_65_2_1_U1_n_13,
      tmp_product_1(0) => mul_32s_34ns_65_2_1_U1_n_14
    );
regslice_both_in_stream_V_keep_V_U: entity work.\decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0\
     port map (
      \B_V_data_1_state_reg[0]_0\ => regslice_both_out_stream_V_data_V_U_n_3,
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      in_stream_TKEEP(3 downto 0) => in_stream_TKEEP(3 downto 0),
      in_stream_TKEEP_int_regslice(3 downto 0) => in_stream_TKEEP_int_regslice(3 downto 0),
      in_stream_TVALID => in_stream_TVALID
    );
regslice_both_in_stream_V_last_V_U: entity work.\decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1\
     port map (
      \B_V_data_1_state_reg[0]_0\ => regslice_both_out_stream_V_data_V_U_n_3,
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      in_stream_TLAST(0) => in_stream_TLAST(0),
      in_stream_TLAST_int_regslice => in_stream_TLAST_int_regslice,
      in_stream_TVALID => in_stream_TVALID
    );
regslice_both_in_stream_V_strb_V_U: entity work.\decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_0\
     port map (
      \B_V_data_1_state_reg[0]_0\ => regslice_both_out_stream_V_data_V_U_n_3,
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      in_stream_TSTRB(3 downto 0) => in_stream_TSTRB(3 downto 0),
      in_stream_TSTRB_int_regslice(3 downto 0) => in_stream_TSTRB_int_regslice(3 downto 0),
      in_stream_TVALID => in_stream_TVALID
    );
regslice_both_out_stream_V_data_V_U: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both_1
     port map (
      \B_V_data_1_state_reg[0]_0\ => out_stream_TVALID,
      \B_V_data_1_state_reg[0]_1\ => regslice_both_out_stream_V_data_V_U_n_3,
      E(0) => mul_ln39_reg_3040,
      Q(15 downto 0) => dc_offset(15 downto 0),
      add_ln32_reg_2830 => add_ln32_reg_2830,
      \add_ln32_reg_283_reg[31]\ => regslice_both_out_stream_V_data_V_U_n_12,
      ap_block_pp0_stage0_11001 => ap_block_pp0_stage0_11001,
      ap_clk => ap_clk,
      ap_enable_reg_pp0_iter3 => ap_enable_reg_pp0_iter3,
      ap_enable_reg_pp0_iter4 => ap_enable_reg_pp0_iter4,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      calibrated => calibrated,
      calibrated_load_reg_279 => calibrated_load_reg_279,
      calibrated_load_reg_279_pp0_iter1_reg => calibrated_load_reg_279_pp0_iter1_reg,
      calibrated_load_reg_279_pp0_iter2_reg => calibrated_load_reg_279_pp0_iter2_reg,
      calibrated_load_reg_279_pp0_iter3_reg => calibrated_load_reg_279_pp0_iter3_reg,
      \calibrated_reg[0]\ => regslice_both_out_stream_V_data_V_U_n_5,
      \counter_reg[0]\ => regslice_both_out_stream_V_data_V_U_n_6,
      \counter_reg[0]_0\ => regslice_both_out_stream_V_data_V_U_n_7,
      \counter_reg[20]\ => regslice_both_out_stream_V_data_V_U_n_9,
      \counter_reg[28]\ => regslice_both_out_stream_V_data_V_U_n_8,
      icmp_ln35_reg_289 => icmp_ln35_reg_289,
      icmp_ln35_reg_289_pp0_iter1_reg => icmp_ln35_reg_289_pp0_iter1_reg,
      icmp_ln35_reg_289_pp0_iter2_reg => icmp_ln35_reg_289_pp0_iter2_reg,
      \icmp_ln35_reg_289_pp0_iter2_reg_reg[0]\(0) => dc_offset0,
      in_stream_TVALID_int_regslice => in_stream_TVALID_int_regslice,
      \out\(31 downto 0) => counter_reg(31 downto 0),
      out_stream_TDATA(15) => \^out_stream_tdata\(31),
      out_stream_TDATA(14 downto 0) => \^out_stream_tdata\(14 downto 0),
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TVALID_int_regslice => out_stream_TVALID_int_regslice,
      p_0_in => p_0_in,
      tmp_reg_298 => tmp_reg_298,
      val_in_reg_274_pp0_iter2_reg(15 downto 0) => val_in_reg_274_pp0_iter2_reg(15 downto 0)
    );
regslice_both_out_stream_V_keep_V_U: entity work.\decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_2\
     port map (
      D(3 downto 0) => pkt_in_keep_V_reg_259_pp0_iter2_reg(3 downto 0),
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      out_stream_TKEEP(3 downto 0) => out_stream_TKEEP(3 downto 0),
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TVALID_int_regslice => out_stream_TVALID_int_regslice
    );
regslice_both_out_stream_V_last_V_U: entity work.\decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized1_3\
     port map (
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      out_stream_TLAST(0) => out_stream_TLAST(0),
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TVALID_int_regslice => out_stream_TVALID_int_regslice,
      pkt_in_last_V_reg_269_pp0_iter2_reg => pkt_in_last_V_reg_269_pp0_iter2_reg
    );
regslice_both_out_stream_V_strb_V_U: entity work.\decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector_regslice_both__parameterized0_4\
     port map (
      D(3 downto 0) => pkt_in_strb_V_reg_264_pp0_iter2_reg(3 downto 0),
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      ap_rst_n_inv => ap_rst_n_inv,
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TSTRB(3 downto 0) => out_stream_TSTRB(3 downto 0),
      out_stream_TVALID_int_regslice => out_stream_TVALID_int_regslice
    );
\sum_reg[0]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_22,
      Q => sum_reg(0),
      R => '0'
    );
\sum_reg[10]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_28,
      Q => sum_reg(10),
      R => '0'
    );
\sum_reg[11]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_27,
      Q => sum_reg(11),
      R => '0'
    );
\sum_reg[12]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_34,
      Q => sum_reg(12),
      R => '0'
    );
\sum_reg[13]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_33,
      Q => sum_reg(13),
      R => '0'
    );
\sum_reg[14]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_32,
      Q => sum_reg(14),
      R => '0'
    );
\sum_reg[15]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_31,
      Q => sum_reg(15),
      R => '0'
    );
\sum_reg[16]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_38,
      Q => sum_reg(16),
      R => '0'
    );
\sum_reg[17]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_37,
      Q => sum_reg(17),
      R => '0'
    );
\sum_reg[18]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_36,
      Q => sum_reg(18),
      R => '0'
    );
\sum_reg[19]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_35,
      Q => sum_reg(19),
      R => '0'
    );
\sum_reg[1]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_21,
      Q => sum_reg(1),
      R => '0'
    );
\sum_reg[20]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_42,
      Q => sum_reg(20),
      R => '0'
    );
\sum_reg[21]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_41,
      Q => sum_reg(21),
      R => '0'
    );
\sum_reg[22]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_40,
      Q => sum_reg(22),
      R => '0'
    );
\sum_reg[23]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_39,
      Q => sum_reg(23),
      R => '0'
    );
\sum_reg[24]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_46,
      Q => sum_reg(24),
      R => '0'
    );
\sum_reg[25]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_45,
      Q => sum_reg(25),
      R => '0'
    );
\sum_reg[26]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_44,
      Q => sum_reg(26),
      R => '0'
    );
\sum_reg[27]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_43,
      Q => sum_reg(27),
      R => '0'
    );
\sum_reg[28]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_50,
      Q => sum_reg(28),
      R => '0'
    );
\sum_reg[29]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_49,
      Q => sum_reg(29),
      R => '0'
    );
\sum_reg[2]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_20,
      Q => sum_reg(2),
      R => '0'
    );
\sum_reg[30]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_48,
      Q => sum_reg(30),
      R => '0'
    );
\sum_reg[31]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_47,
      Q => sum_reg(31),
      R => '0'
    );
\sum_reg[3]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_19,
      Q => sum_reg(3),
      R => '0'
    );
\sum_reg[4]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_26,
      Q => sum_reg(4),
      R => '0'
    );
\sum_reg[5]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_25,
      Q => sum_reg(5),
      R => '0'
    );
\sum_reg[6]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_24,
      Q => sum_reg(6),
      R => '0'
    );
\sum_reg[7]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_23,
      Q => sum_reg(7),
      R => '0'
    );
\sum_reg[8]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_30,
      Q => sum_reg(8),
      R => '0'
    );
\sum_reg[9]\: unisim.vcomponents.FDRE
    generic map(
      INIT => '0'
    )
        port map (
      C => ap_clk,
      CE => add_ln32_reg_2830,
      D => regslice_both_in_stream_V_data_V_U_n_29,
      Q => sum_reg(9),
      R => '0'
    );
\tmp_2_reg_309_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(56),
      Q => tmp_2_reg_309(14),
      R => '0'
    );
\tmp_2_reg_309_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => mul_ln39_reg_3040,
      D => \buff0_reg__1\(57),
      Q => tmp_2_reg_309(15),
      R => '0'
    );
\tmp_reg_298_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => tmp_reg_298,
      Q => tmp_reg_298_pp0_iter2_reg,
      R => '0'
    );
\tmp_reg_298_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => '1',
      D => regslice_both_out_stream_V_data_V_U_n_12,
      Q => tmp_reg_298,
      R => '0'
    );
\val_in_reg_274_pp0_iter1_reg_reg[0]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(0),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[10]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(10),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[11]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(11),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[12]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(12),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[13]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(13),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[14]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(14),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[15]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(15),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[1]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(1),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[2]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(2),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[3]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(3),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[4]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(4),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[5]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(5),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[6]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(6),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[7]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(7),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[8]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(8),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter1_reg_reg[9]_srl2\: unisim.vcomponents.SRL16E
     port map (
      A0 => '1',
      A1 => '0',
      A2 => '0',
      A3 => '0',
      CE => ap_block_pp0_stage0_11001,
      CLK => ap_clk,
      D => \in\(9),
      Q => \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_n_0\
    );
\val_in_reg_274_pp0_iter2_reg_reg[0]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[0]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(0),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[10]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[10]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(10),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[11]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[11]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(11),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[12]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[12]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(12),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[13]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[13]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(13),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[14]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[14]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(14),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[15]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[15]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(15),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[1]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[1]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(1),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[2]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[2]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(2),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[3]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[3]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(3),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[4]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[4]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(4),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[5]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[5]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(5),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[6]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[6]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(6),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[7]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[7]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(7),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[8]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[8]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(8),
      R => '0'
    );
\val_in_reg_274_pp0_iter2_reg_reg[9]\: unisim.vcomponents.FDRE
     port map (
      C => ap_clk,
      CE => ap_block_pp0_stage0_11001,
      D => \val_in_reg_274_pp0_iter1_reg_reg[9]_srl2_n_0\,
      Q => val_in_reg_274_pp0_iter2_reg(9),
      R => '0'
    );
end STRUCTURE;
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  port (
    ap_clk : in STD_LOGIC;
    ap_rst_n : in STD_LOGIC;
    in_stream_TVALID : in STD_LOGIC;
    in_stream_TREADY : out STD_LOGIC;
    in_stream_TDATA : in STD_LOGIC_VECTOR ( 31 downto 0 );
    in_stream_TLAST : in STD_LOGIC_VECTOR ( 0 to 0 );
    in_stream_TKEEP : in STD_LOGIC_VECTOR ( 3 downto 0 );
    in_stream_TSTRB : in STD_LOGIC_VECTOR ( 3 downto 0 );
    out_stream_TVALID : out STD_LOGIC;
    out_stream_TREADY : in STD_LOGIC;
    out_stream_TDATA : out STD_LOGIC_VECTOR ( 31 downto 0 );
    out_stream_TLAST : out STD_LOGIC_VECTOR ( 0 to 0 );
    out_stream_TKEEP : out STD_LOGIC_VECTOR ( 3 downto 0 );
    out_stream_TSTRB : out STD_LOGIC_VECTOR ( 3 downto 0 )
  );
  attribute NotValidForBitStream : boolean;
  attribute NotValidForBitStream of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is true;
  attribute CHECK_LICENSE_TYPE : string;
  attribute CHECK_LICENSE_TYPE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "system_fsk_phase_corrector_0_0,fsk_phase_corrector,{}";
  attribute DowngradeIPIdentifiedWarnings : string;
  attribute DowngradeIPIdentifiedWarnings of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
  attribute IP_DEFINITION_SOURCE : string;
  attribute IP_DEFINITION_SOURCE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "HLS";
  attribute X_CORE_INFO : string;
  attribute X_CORE_INFO of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "fsk_phase_corrector,Vivado 2023.1";
  attribute hls_module : string;
  attribute hls_module of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix : entity is "yes";
end decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix;

architecture STRUCTURE of decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix is
  attribute SDX_KERNEL : string;
  attribute SDX_KERNEL of inst : label is "true";
  attribute SDX_KERNEL_SYNTH_INST : string;
  attribute SDX_KERNEL_SYNTH_INST of inst : label is "inst";
  attribute SDX_KERNEL_TYPE : string;
  attribute SDX_KERNEL_TYPE of inst : label is "hls";
  attribute ap_ST_fsm_pp0_stage0 : string;
  attribute ap_ST_fsm_pp0_stage0 of inst : label is "1'b1";
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of ap_clk : signal is "xilinx.com:signal:clock:1.0 ap_clk CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of ap_clk : signal is "XIL_INTERFACENAME ap_clk, ASSOCIATED_BUSIF in_stream:out_stream, ASSOCIATED_RESET ap_rst_n, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of ap_rst_n : signal is "xilinx.com:signal:reset:1.0 ap_rst_n RST";
  attribute X_INTERFACE_PARAMETER of ap_rst_n : signal is "XIL_INTERFACENAME ap_rst_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of in_stream_TREADY : signal is "xilinx.com:interface:axis:1.0 in_stream TREADY";
  attribute X_INTERFACE_INFO of in_stream_TVALID : signal is "xilinx.com:interface:axis:1.0 in_stream TVALID";
  attribute X_INTERFACE_INFO of out_stream_TREADY : signal is "xilinx.com:interface:axis:1.0 out_stream TREADY";
  attribute X_INTERFACE_INFO of out_stream_TVALID : signal is "xilinx.com:interface:axis:1.0 out_stream TVALID";
  attribute X_INTERFACE_INFO of in_stream_TDATA : signal is "xilinx.com:interface:axis:1.0 in_stream TDATA";
  attribute X_INTERFACE_INFO of in_stream_TKEEP : signal is "xilinx.com:interface:axis:1.0 in_stream TKEEP";
  attribute X_INTERFACE_INFO of in_stream_TLAST : signal is "xilinx.com:interface:axis:1.0 in_stream TLAST";
  attribute X_INTERFACE_INFO of in_stream_TSTRB : signal is "xilinx.com:interface:axis:1.0 in_stream TSTRB";
  attribute X_INTERFACE_PARAMETER of in_stream_TSTRB : signal is "XIL_INTERFACENAME in_stream, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
  attribute X_INTERFACE_INFO of out_stream_TDATA : signal is "xilinx.com:interface:axis:1.0 out_stream TDATA";
  attribute X_INTERFACE_INFO of out_stream_TKEEP : signal is "xilinx.com:interface:axis:1.0 out_stream TKEEP";
  attribute X_INTERFACE_INFO of out_stream_TLAST : signal is "xilinx.com:interface:axis:1.0 out_stream TLAST";
  attribute X_INTERFACE_INFO of out_stream_TSTRB : signal is "xilinx.com:interface:axis:1.0 out_stream TSTRB";
  attribute X_INTERFACE_PARAMETER of out_stream_TSTRB : signal is "XIL_INTERFACENAME out_stream, TDATA_NUM_BYTES 4, TUSER_WIDTH 0, TDEST_WIDTH 0, TID_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 1, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN system_axi_ad9361_0_l_clk, INSERT_VIP 0";
begin
inst: entity work.decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fsk_phase_corrector
     port map (
      ap_clk => ap_clk,
      ap_rst_n => ap_rst_n,
      in_stream_TDATA(31 downto 16) => B"0000000000000000",
      in_stream_TDATA(15 downto 0) => in_stream_TDATA(15 downto 0),
      in_stream_TKEEP(3 downto 0) => in_stream_TKEEP(3 downto 0),
      in_stream_TLAST(0) => in_stream_TLAST(0),
      in_stream_TREADY => in_stream_TREADY,
      in_stream_TSTRB(3 downto 0) => in_stream_TSTRB(3 downto 0),
      in_stream_TVALID => in_stream_TVALID,
      out_stream_TDATA(31 downto 0) => out_stream_TDATA(31 downto 0),
      out_stream_TKEEP(3 downto 0) => out_stream_TKEEP(3 downto 0),
      out_stream_TLAST(0) => out_stream_TLAST(0),
      out_stream_TREADY => out_stream_TREADY,
      out_stream_TSTRB(3 downto 0) => out_stream_TSTRB(3 downto 0),
      out_stream_TVALID => out_stream_TVALID
    );
end STRUCTURE;
